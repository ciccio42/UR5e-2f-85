import importlib
import os
import pickle
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
from moveit_controller_srvs.srv import GoHome, GoToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class RolloutUnpickler(pickle.Unpickler):
    """Resolves the 'Trajectory' class saved by dataset_collector_pkg's savers.py."""

    def find_class(self, module, name):
        if module.startswith('multi_task_il') and name == 'Trajectory':
            return importlib.import_module('savers').Trajectory
        return super().find_class(module, name)


def _add_dataset_collector_scripts_to_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        scripts_dir = Path(get_package_share_directory('dataset_collector_pkg')) / 'scripts'
    except Exception:
        return

    if scripts_dir.is_dir() and str(scripts_dir) not in sys.path:
        sys.path.append(str(scripts_dir))


class ReplicateRollout(Node):
    """Loads a rollout .pkl saved by ai_controller_node.py's save_rollout() and, per step,
    checks that it can be parsed into a robot command. In dry_run mode (default) it only
    logs the pose/gripper command it WOULD send; with dry_run:=false it actually sends
    them via the same MoveIt/gripper interfaces as ai_controller_node.py.

    Unlike dataset_collector_pkg's replicate_trajectory.py (which replays recorded
    demonstrations keyed by eef_pos/eef_quat/gripper_qpos), this reads the rollout's
    top-level `action` field directly: an 8-vector [x, y, z, qx, qy, qz, qw, gripper_pos]
    as produced by CODController._post_process_action / used in ai_controller_node.py's
    control_loop (gripper_pos is already a raw 0/255 GripperCommand position, not the
    0.0-1.0 style used by the recorded-demonstration pipeline).
    """

    REQUIRED_OBS_FIELDS = ('eef_pos', 'eef_quat', 'joint_pos', 'joint_vel', 'gripper_qpos', 'gripper_qvel')

    def __init__(self):
        super().__init__('replicate_rollout')

        self.declare_parameter('rollout_path', '')
        self.declare_parameter('dry_run', True)
        self.declare_parameter('set_home_service', 'set_robot_to_home')
        self.declare_parameter('set_pose_service', 'set_robot_to_pose')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('move_home_before_start', False)
        self.declare_parameter('step_delay_sec', 0.0)
        self.declare_parameter('service_timeout_sec', 10.0)
        self.declare_parameter('gripper_action_topic', '/robotiq_gripper_controller/gripper_cmd')
        self.declare_parameter('gripper_max_effort', 50.0)
        self.declare_parameter('gripper_result_timeout_sec', 10.0)
        self.declare_parameter('show_images', True)
        self.declare_parameter('preview_wait_ms', 200)
        self.declare_parameter('context_path', '')
        self.declare_parameter('save_video', False)
        self.declare_parameter('video_output_dir', '')
        self.declare_parameter('video_fps', 5.0)

        self.rollout_path = self.get_parameter('rollout_path').value
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.set_home_service_name = self.get_parameter('set_home_service').value
        self.set_pose_service_name = self.get_parameter('set_pose_service').value
        self.frame_id = self.get_parameter('frame_id').value
        self.move_home_before_start = bool(self.get_parameter('move_home_before_start').value)
        self.step_delay_sec = float(self.get_parameter('step_delay_sec').value)
        self.service_timeout_sec = float(self.get_parameter('service_timeout_sec').value)
        self.gripper_action_topic = self.get_parameter('gripper_action_topic').value
        self.gripper_max_effort = float(self.get_parameter('gripper_max_effort').value)
        self.gripper_result_timeout_sec = float(self.get_parameter('gripper_result_timeout_sec').value)
        self.show_images = bool(self.get_parameter('show_images').value)
        self.preview_wait_ms = int(self.get_parameter('preview_wait_ms').value)

        self.preview_window_name = 'Rollout Check: original vs cropped+predicted_bb'
        if self.show_images:
            try:
                cv2.namedWindow(self.preview_window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.preview_window_name, 1000, 400)
            except cv2.error as exc:
                self.get_logger().warning(f'Could not create preview window: {exc}')
                self.show_images = False

        self.get_logger().info(
            f"{'[DRY-RUN] ' if self.dry_run else ''}Loading rollout from: {self.rollout_path}"
        )

        self.go_home_client = self.create_client(GoHome, self.set_home_service_name)
        self.go_to_pose_client = self.create_client(GoToPose, self.set_pose_service_name)
        self.gripper_action_client = ActionClient(self, GripperCommand, self.gripper_action_topic)

        self.steps = self._load_rollout(self.rollout_path)

        self.context_path = self._resolve_context_path(
            self.rollout_path, self.get_parameter('context_path').value)
        self.context_frames = self._load_context_frames(self.context_path)

        self.get_logger().info(
            f"{'[DRY-RUN] ' if self.dry_run else ''}"
            f'Loaded {len(self.steps)} steps from {self.rollout_path!r}. '
            f'Loaded {len(self.context_frames)} context frames from {self.context_path!r}.'
        )

        self.save_video = bool(self.get_parameter('save_video').value)
        self.video_fps = float(self.get_parameter('video_fps').value)
        video_output_dir_param = self.get_parameter('video_output_dir').value
        self.video_output_dir = video_output_dir_param or str(Path(self.rollout_path).expanduser().parent)
        self._video_basename = Path(self.rollout_path).stem
        self.original_video_writer = None
        self.annotated_video_writer = None
        if self.save_video:
            os.makedirs(self.video_output_dir, exist_ok=True)
            self.get_logger().info(f'Saving comparison videos to {self.video_output_dir}')

    def _load_rollout(self, rollout_path):
        if not rollout_path:
            raise ValueError('Parameter rollout_path must point to a rollout .pkl file saved by ai_controller_node.')

        path = Path(rollout_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f'Rollout file does not exist: {path}')

        _add_dataset_collector_scripts_to_path()

        with path.open('rb') as f:
            data = RolloutUnpickler(f).load()

        trajectory = data['traj']
        return [trajectory[t] for t in range(len(trajectory))]

    @staticmethod
    def _resolve_context_path(rollout_path, context_path_param):
        """CODController._save_command_trajectory() saves 'context_{traj_cnt:03d}.pkl'
        next to save_rollout()'s 'traj_{traj_cnt:03d}.pkl', in the same task directory,
        using the same counter. If context_path isn't given explicitly, try to derive it
        from rollout_path by that naming convention."""
        if context_path_param:
            return context_path_param

        rollout_file = Path(rollout_path)
        if rollout_file.name.startswith('traj_'):
            candidate = rollout_file.with_name(rollout_file.name.replace('traj_', 'context_', 1))
            if candidate.is_file():
                return str(candidate)

        return ''

    def _load_context_frames(self, context_path):
        """Load a context_*.pkl (saved by CODController._save_command_trajectory) so the
        command frames that conditioned this rollout can be shown alongside it."""
        if not context_path:
            return []

        path = Path(context_path).expanduser()
        if not path.is_file():
            self.get_logger().warning(f'Context trajectory file does not exist: {path}; skipping context preview.')
            return []

        _add_dataset_collector_scripts_to_path()

        with path.open('rb') as f:
            data = RolloutUnpickler(f).load()

        trajectory = data['traj']
        frames = []
        for t in range(len(trajectory)):
            obs = trajectory[t].get('obs', {})
            frame = obs.get('camera_front_image') if isinstance(obs, dict) else None
            if frame is not None:
                frames.append(frame[:,:,::-1])  # convert RGB->BGR for OpenCV display

        return frames

    def wait_for_moveit_services(self):
        self.get_logger().info('Waiting for MoveIt controller services...')
        home_ready = self.go_home_client.wait_for_service(timeout_sec=self.service_timeout_sec)
        pose_ready = self.go_to_pose_client.wait_for_service(timeout_sec=self.service_timeout_sec)
        gripper_ready = self.gripper_action_client.wait_for_server(timeout_sec=self.service_timeout_sec)

        if not home_ready:
            self.get_logger().error(f'Service not available: {self.set_home_service_name}')
        if not pose_ready:
            self.get_logger().error(f'Service not available: {self.set_pose_service_name}')
        if not gripper_ready:
            self.get_logger().error(f'Action server not available: {self.gripper_action_topic}')

        return home_ready and pose_ready and gripper_ready

    @staticmethod
    def _draw_predicted_bb(image_bgr, predicted_bb):
        """Draw predicted_bb boxes on a copy of image_bgr, mirroring cod_controller.py's
        own visualization (predicted_bb[0][0] -> Nx4 [x1, y1, x2, y2] boxes in the
        cropped image's pixel space)."""
        if predicted_bb is None:
            return image_bgr

        boxes = predicted_bb
        if hasattr(boxes, 'ndim'):
            if boxes.ndim == 4:
                boxes = boxes[0][0]
            elif boxes.ndim == 3:
                boxes = boxes[0]

        image_bgr = np.ascontiguousarray(image_bgr.copy())
        for box in boxes:
            try:
                x1, y1, x2, y2 = box
                cv2.rectangle(image_bgr, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
            except (TypeError, ValueError):
                continue

        return image_bgr

    def _build_context_panel(self, target_height):
        """Tile the (up to 4) context frames into a single 2x2 grid image, scaled to
        target_height so it hstacks cleanly with the other preview panels."""
        if not self.context_frames:
            return None

        n_cols, n_rows = 2, 2
        n_tiles = n_cols * n_rows
        frames = list(self.context_frames[:n_tiles])

        tile_height = max(1, target_height // n_rows)
        ref_frame = frames[0]
        tile_width = max(1, int(tile_height * ref_frame.shape[1] / ref_frame.shape[0]))
        blank_tile = np.zeros((tile_height, tile_width, 3), dtype=np.uint8)

        tiles = []
        for i in range(n_tiles):
            frame = frames[i] if i < len(frames) else None
            tile = cv2.resize(frame, (tile_width, tile_height)) if frame is not None else blank_tile.copy()
            # cv2.putText(tile, f'ctx {i}', (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            tiles.append(tile)

        rows = [np.hstack(tiles[r * n_cols:(r + 1) * n_cols]) for r in range(n_rows)]
        panel = np.vstack(rows)

        if panel.shape[0] != target_height:
            panel = cv2.resize(panel, (panel.shape[1], target_height))

        # cv2.putText(panel, 'Context frames (2x2)', (10, panel.shape[0] - 10),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        return panel

    @staticmethod
    def _add_label_bar(image_bgr, text, bar_height=32):
        """Stack a captioned black bar on top of image_bgr (used for video panels)."""
        bar = np.zeros((bar_height, image_bgr.shape[1], 3), dtype=np.uint8)
        cv2.putText(bar, text, (10, int(bar_height * 0.72)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return np.vstack([bar, image_bgr])

    def _compose_execution_vs_demo(self, execution_bgr, target_height):
        """Build one video frame: Robot-Execution (left) next to Human-Demonstration
        (right, the context frames), each captioned with a label bar."""
        execution_resized = cv2.resize(
            execution_bgr,
            (int(execution_bgr.shape[1] * target_height / execution_bgr.shape[0]), target_height))

        context_panel = self._build_context_panel(target_height)
        if context_panel is None:
            context_panel = np.zeros((target_height, execution_resized.shape[1], 3), dtype=np.uint8)

        execution_labeled = self._add_label_bar(execution_resized, 'Robot-Execution')
        context_labeled = self._add_label_bar(context_panel, 'Human-Demonstration')

        return np.hstack([execution_labeled, context_labeled])

    def _ensure_video_writer(self, attr_name, filename, frame):
        writer = getattr(self, attr_name)
        if writer is None:
            height, width = frame.shape[:2]
            path = os.path.join(self.video_output_dir, filename)
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            writer = cv2.VideoWriter(path, fourcc, self.video_fps, (width, height))
            if not writer.isOpened():
                self.get_logger().error(f'Could not open video writer for {path}')
                writer = False
            else:
                self.get_logger().info(f'Recording video to {path}')
            setattr(self, attr_name, writer)
        return writer

    def _record_video_frames(self, obs):
        """Append one frame to each comparison video: 'original' from the raw
        camera_front_image, 'annotated' from cropped_image + predicted_bb — both
        composed as Robot-Execution (left) vs Human-Demonstration (right)."""
        if not self.save_video:
            return

        front_image = obs.get('camera_front_image')
        if front_image is not None:
            original_frame = self._compose_execution_vs_demo(front_image, front_image.shape[0])
            writer = self._ensure_video_writer(
                'original_video_writer', f'{self._video_basename}_original.mp4', original_frame)
            if writer:
                writer.write(original_frame)

        cropped_image = obs.get('cropped_image')
        if cropped_image is not None:
            cropped_bgr = cv2.cvtColor(cropped_image, cv2.COLOR_RGB2BGR)
            annotated_execution = self._draw_predicted_bb(cropped_bgr, obs.get('predicted_bb'))
            annotated_frame = self._compose_execution_vs_demo(annotated_execution, annotated_execution.shape[0])
            writer = self._ensure_video_writer(
                'annotated_video_writer', f'{self._video_basename}_annotated.mp4', annotated_frame)
            if writer:
                writer.write(annotated_frame)

    def _close_video_writers(self):
        for attr in ('original_video_writer', 'annotated_video_writer'):
            writer = getattr(self, attr, None)
            if writer:
                writer.release()
                setattr(self, attr, None)

    def _update_preview(self, index, obs):
        """Show the original camera_front_image next to the cropped model-input image
        with predicted_bb boxes overlaid, so a rollout can be visually inspected."""
        if not self.show_images:
            return

        front_image = obs.get('camera_front_image')
        cropped_image = obs.get('cropped_image')
        if front_image is None and cropped_image is None:
            return

        if cropped_image is not None:
            cropped_bgr = cv2.cvtColor(cropped_image, cv2.COLOR_RGB2BGR)
            cropped_bgr = self._draw_predicted_bb(cropped_bgr, obs.get('predicted_bb'))
        else:
            cropped_bgr = np.zeros_like(front_image)

        if front_image is None:
            front_image = np.zeros_like(cropped_bgr)

        target_height = max(front_image.shape[0], cropped_bgr.shape[0])
        front_resized = cv2.resize(
            front_image, (int(front_image.shape[1] * target_height / front_image.shape[0]), target_height))
        cropped_resized = cv2.resize(
            cropped_bgr, (int(cropped_bgr.shape[1] * target_height / cropped_bgr.shape[0]), target_height))

        cv2.putText(front_resized, f'Step {index}: camera_front_image', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(cropped_resized, 'cropped_image + predicted_bb', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        panels = [front_resized, cropped_resized]
        context_panel = self._build_context_panel(target_height)
        if context_panel is not None:
            panels.append(context_panel)

        combined = np.hstack(panels)

        try:
            cv2.imshow(self.preview_window_name, combined)
            cv2.waitKey(self.preview_wait_ms)
        except cv2.error as exc:
            self.get_logger().warning(f'Preview display failed: {exc}')
            self.show_images = False

    def _check_step(self, step):
        """Validate a single rollout step and build its PoseStamped + gripper position.

        Returns (ok, pose_stamped_or_None, gripper_position_or_None, issue_messages).
        """
        issues = []

        obs = step.get('obs') if isinstance(step, dict) else None
        if not isinstance(obs, dict):
            issues.append("step has no 'obs' dict")
            obs = {}

        for field in self.REQUIRED_OBS_FIELDS:
            if obs.get(field) is None:
                issues.append(f"missing robot-state field obs[{field!r}]")

        if obs.get('camera_front_image') is None:
            issues.append("missing obs['camera_front_image']")
        if obs.get('cropped_image') is None:
            issues.append("missing obs['cropped_image']")
        if obs.get('predicted_bb') is None:
            issues.append("missing obs['predicted_bb'] (expected for cod_controller, not openvla_controller)")

        action = step.get('action') if isinstance(step, dict) else None
        if action is None or len(action) < 8:
            issues.append(
                f"'action' missing or too short (expected 8 values, got "
                f"{0 if action is None else len(action)})"
            )
            return False, None, None, issues

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(action[0])
        pose.pose.position.y = float(action[1])
        pose.pose.position.z = float(action[2])
        pose.pose.orientation.x = float(action[3])
        pose.pose.orientation.y = float(action[4])
        pose.pose.orientation.z = float(action[5])
        pose.pose.orientation.w = float(action[6])
        gripper_position = float(action[7])

        return True, pose, gripper_position, issues

    def replicate_rollout(self):
        try:
            return self._replicate_rollout()
        finally:
            self._close_video_writers()

    def _replicate_rollout(self):
        if not self.wait_for_moveit_services():
            return False

        if self.move_home_before_start:
            if self.dry_run:
                self.get_logger().info(f'[DRY-RUN] Would call {self.set_home_service_name} before starting.')
            elif not self.call_go_home():
                return False

        all_valid = True
        for index, step in enumerate(self.steps):
            ok, pose, gripper_position, issues = self._check_step(step)
            obs = step.get('obs', {}) if isinstance(step, dict) else {}
            self._update_preview(index, obs)
            self._record_video_frames(obs)
            for issue in issues:
                self.get_logger().warning(f'Step {index}: {issue}')
            if not ok:
                all_valid = False
                continue

            self.get_logger().info(
                f'Step {index + 1}/{len(self.steps)} pose: '
                f'{pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}, '
                f'{pose.pose.orientation.x:.3f}, {pose.pose.orientation.y:.3f}, '
                f'{pose.pose.orientation.z:.3f}, {pose.pose.orientation.w:.3f}, gripper={gripper_position}'
            )

            if self.dry_run:
                self.get_logger().info(f'[DRY-RUN] Step {index}: would send the pose + gripper command above.')
            else:
                # wait the user to press enter before sending the command, so they can inspect the preview window
                input(f'Press Enter to send the pose + gripper command for step {index} (or Ctrl+C to abort)...')
                if not self.call_go_to_pose(pose):
                    self.get_logger().error(f'Failed while executing rollout step {index}.')
                    return False
                if not self.call_gripper(gripper_position):
                    self.get_logger().error(f'Failed while commanding gripper at rollout step {index}.')
                    return False

            if self.step_delay_sec > 0.0:
                time.sleep(self.step_delay_sec)

        self.get_logger().info(
            f"{'[DRY-RUN] ' if self.dry_run else ''}"
            f'Rollout check complete: {len(self.steps)} steps, '
            f"{'ALL VALID' if all_valid else 'SOME STEPS HAD ISSUES (see warnings above)'}."
        )
        return all_valid

    def call_go_home(self):
        request = GoHome.Request()
        future = self.go_home_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is None:
            self.get_logger().error('set_robot_to_home service call failed.')
            return False

        if response.success:
            self.get_logger().info(response.message)
            return True

        self.get_logger().error(response.message)
        return False

    def call_go_to_pose(self, pose):
        request = GoToPose.Request()
        request.pose = pose
        future = self.go_to_pose_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is None:
            self.get_logger().error('set_robot_to_pose service call failed.')
            return False

        if response.success:
            self.get_logger().info(response.message)
            return True

        self.get_logger().error(response.message)
        return False

    def call_gripper(self, position):
        self.get_logger().info(
            f'Sending gripper command: position={position}, max_effort={self.gripper_max_effort}'
        )

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = self.gripper_max_effort

        future = self.gripper_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.gripper_result_timeout_sec)
        if not future.done():
            self.get_logger().error('Timed out while sending gripper command.')
            return False

        goal_handle = future.result()
        if goal_handle is None:
            self.get_logger().error('Gripper command failed: no goal handle returned.')
            return False
        if not goal_handle.accepted:
            self.get_logger().error('Gripper command rejected.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.gripper_result_timeout_sec)
        if not result_future.done():
            self.get_logger().error('Timed out while waiting for gripper action completion.')
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            return False

        result_response = result_future.result()
        if result_response is None:
            self.get_logger().error('Gripper command failed: no result returned.')
            return False
        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'Gripper command finished with status: {result_response.status}')
            return False

        return True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ReplicateRollout()
        success = node.replicate_rollout()
        node.get_logger().info('Rollout check PASSED.' if success else 'Rollout check FAILED — see warnings above.')
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
