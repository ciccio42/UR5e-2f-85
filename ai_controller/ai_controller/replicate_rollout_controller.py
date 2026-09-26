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
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

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


class ReplicateRolloutController(Node):
    """Loads a rollout .pkl saved by script_controller_node.py and, per step,
    checks that it can be parsed into a robot command. In dry_run mode (default)
    it only logs the pose/gripper command it WOULD send; with dry_run:=false it
    actually sends them via the same MoveIt/gripper interfaces as
    replicate_rollout.py.

    Unlike replicate_rollout.py (which validates AI/COD rollout fields such as
    camera_front_image, cropped_image and predicted_bb), this reads the
    script_controller_node.py schema directly:
    front/left/right/gripper RGB+depth images, robot state fields and
    the top-level `action` field. The action is an 8-vector
    [x, y, z, qx, qy, qz, qw, gripper_normalized], where gripper_normalized is
    mapped back to gripper_open_position / gripper_closed_position before
    replay.
    """

    REQUIRED_OBS_FIELDS = ('eef_pos', 'eef_quat', 'joint_pos', 'joint_vel', 'gripper_qpos', 'gripper_qvel')
    CAMERA_FIELDS = (
        ('front', ('front_camera_image', 'camera_front_image'),
         ('front_camera_depth', 'camera_front_depth')),
        ('left', ('left_camera_image',), ('left_camera_depth',)),
        ('right', ('right_camera_image',), ('right_camera_depth',)),
        ('wrist', ('gripper_camera_image',), ('gripper_camera_depth',)),
    )

    def __init__(self):
        super().__init__('replicate_rollout_controller')

        self.declare_parameter('rollout_path', '')
        self.declare_parameter('dry_run', True)
        self.declare_parameter('set_home_service', 'set_robot_to_home')
        self.declare_parameter('set_pose_service', 'set_robot_to_pose')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('move_home_before_start', False)
        self.declare_parameter('step_delay_sec', 0.0)
        self.declare_parameter('service_timeout_sec', 10.0)
        self.declare_parameter('gripper_action_topic', '/robotiq_gripper_controller/gripper_cmd')
        self.declare_parameter('gripper_open_position', 0.1)
        self.declare_parameter('gripper_closed_position', 0.8)
        self.declare_parameter('gripper_closed_threshold', 0.5)
        self.declare_parameter('gripper_max_effort', 50.0)
        self.declare_parameter('gripper_result_timeout_sec', 10.0)
        self.declare_parameter('show_images', True)
        self.declare_parameter('preview_wait_ms', 200)
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
        self.gripper_open_position = float(self.get_parameter('gripper_open_position').value)
        self.gripper_closed_position = float(self.get_parameter('gripper_closed_position').value)
        self.gripper_closed_threshold = float(self.get_parameter('gripper_closed_threshold').value)
        self.gripper_max_effort = float(self.get_parameter('gripper_max_effort').value)
        self.gripper_result_timeout_sec = float(self.get_parameter('gripper_result_timeout_sec').value)
        self.show_images = bool(self.get_parameter('show_images').value)
        self.preview_wait_ms = int(self.get_parameter('preview_wait_ms').value)

        self.preview_window_name = 'Controller Rollout Check: cameras + robot state'
        if self.show_images:
            try:
                cv2.namedWindow(self.preview_window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.preview_window_name, 1400, 900)
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

        self.get_logger().info(
            f"{'[DRY-RUN] ' if self.dry_run else ''}"
            f'Loaded {len(self.steps)} steps from {self.rollout_path!r}.'
        )

        self.save_video = bool(self.get_parameter('save_video').value)
        self.video_fps = float(self.get_parameter('video_fps').value)
        video_output_dir_param = self.get_parameter('video_output_dir').value
        self.video_output_dir = video_output_dir_param or str(Path(self.rollout_path).expanduser().parent)
        self._video_basename = Path(self.rollout_path).stem
        self.preview_video_writer = None
        if self.save_video:
            os.makedirs(self.video_output_dir, exist_ok=True)
            self.get_logger().info(f'Saving full preview video to {self.video_output_dir}')

        # create tf-listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


    def _load_rollout(self, rollout_path):
        if not rollout_path:
            raise ValueError('Parameter rollout_path must point to a rollout .pkl file saved by script_controller_node.')

        path = Path(rollout_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f'Rollout file does not exist: {path}')

        _add_dataset_collector_scripts_to_path()

        with path.open('rb') as f:
            data = RolloutUnpickler(f).load()

        trajectory = data['traj']
        return [trajectory[t] for t in range(len(trajectory))]

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
    def _ensure_bgr_image(image):
        if image is None:
            return None

        if isinstance(image, np.ndarray):
            if image.ndim == 2:
                return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            if image.ndim == 3 and image.shape[2] == 3:
                return image
            if image.ndim == 3 and image.shape[2] == 4:
                return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
            if image.ndim == 1:
                decoded = cv2.imdecode(image.astype(np.uint8, copy=False), cv2.IMREAD_COLOR)
                return decoded

        if isinstance(image, (bytes, bytearray, memoryview)):
            buffer = np.frombuffer(image, dtype=np.uint8)
            return cv2.imdecode(buffer, cv2.IMREAD_COLOR)

        return None

    @staticmethod
    def _get_obs_value(obs, keys):
        for key in keys:
            value = obs.get(key)
            if value is not None:
                return value
        return None

    @staticmethod
    def _pad_to_cell(image_bgr, cell_width, cell_height):
        if image_bgr is None:
            return np.zeros((cell_height, cell_width, 3), dtype=np.uint8)

        height, width = image_bgr.shape[:2]
        canvas = np.zeros((cell_height, cell_width, 3), dtype=np.uint8)
        canvas[:height, :width] = image_bgr
        return canvas

    @staticmethod
    def _put_label(image_bgr, text, color=(0, 255, 0)):
        labeled = image_bgr.copy()
        cv2.putText(labeled, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, color, 2, cv2.LINE_AA)
        return labeled

    @staticmethod
    def _format_array(values, precision=3, max_values=None):
        if values is None:
            return 'None'

        array = np.asarray(values).flatten()
        if max_values is not None:
            array = array[:max_values]

        return np.array2string(array, precision=precision, suppress_small=True, separator=', ')

    @staticmethod
    def _ensure_gray_depth_bgr(depth_image):
        if depth_image is None:
            return None

        if isinstance(depth_image, np.ndarray) and depth_image.ndim == 1:
            decoded = cv2.imdecode(depth_image.astype(np.uint8, copy=False), cv2.IMREAD_UNCHANGED)
            if decoded is None:
                return None
            depth_image = decoded
        elif isinstance(depth_image, (bytes, bytearray, memoryview)):
            buffer = np.frombuffer(depth_image, dtype=np.uint8)
            decoded = cv2.imdecode(buffer, cv2.IMREAD_UNCHANGED)
            if decoded is None:
                return None
            depth_image = decoded

        depth = np.asarray(depth_image)
        if depth.ndim == 3 and depth.shape[2] == 3:
            gray = cv2.cvtColor(depth, cv2.COLOR_BGR2GRAY)
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if depth.ndim == 3 and depth.shape[2] == 4:
            gray = cv2.cvtColor(depth, cv2.COLOR_BGRA2GRAY)
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if depth.ndim != 2:
            return None

        if depth.dtype == np.uint8:
            normalized = depth
        else:
            depth_float = depth.astype(np.float32)
            valid = np.isfinite(depth_float)
            positive_valid = valid & (depth_float > 0.0)
            if np.any(positive_valid):
                valid_values = depth_float[positive_valid]
            elif np.any(valid):
                valid_values = depth_float[valid]
            else:
                normalized = np.zeros(depth_float.shape, dtype=np.uint8)
                return cv2.cvtColor(normalized, cv2.COLOR_GRAY2BGR)

            vmin = float(np.percentile(valid_values, 5))
            vmax = float(np.percentile(valid_values, 95))
            if vmax <= vmin:
                vmax = vmin + 1e-6

            clipped = np.clip(depth_float, vmin, vmax)
            clipped[~valid] = vmin
            normalized = ((clipped - vmin) / (vmax - vmin) * 255.0).astype(np.uint8)

        return cv2.cvtColor(normalized, cv2.COLOR_GRAY2BGR)

    def _build_camera_grid(self, obs):
        rows = []
        rgb_width = 0
        depth_width = 0
        row_height = 0

        for label, image_keys, depth_keys in self.CAMERA_FIELDS:
            rgb = self._ensure_bgr_image(self._get_obs_value(obs, image_keys))
            depth = self._ensure_gray_depth_bgr(self._get_obs_value(obs, depth_keys))
            if rgb is not None:
                rgb_width = max(rgb_width, rgb.shape[1])
                row_height = max(row_height, rgb.shape[0])
            if depth is not None:
                depth_width = max(depth_width, depth.shape[1])
                row_height = max(row_height, depth.shape[0])
            rows.append((label, rgb, depth))

        if rgb_width == 0:
            rgb_width = 420
        if depth_width == 0:
            depth_width = 420
        if row_height == 0:
            row_height = 240

        rendered_rows = []
        for label, rgb, depth in rows:
            rgb_tile = self._put_label(
                self._pad_to_cell(rgb, rgb_width, row_height),
                f'{label} rgb')
            depth_tile = self._put_label(
                self._pad_to_cell(depth, depth_width, row_height),
                f'{label} depth',
                color=(255, 255, 255))
            rendered_rows.append(np.hstack([rgb_tile, depth_tile]))

        return np.vstack(rendered_rows)

    def _build_text_panel(self, index, step, obs, gripper_position, issues, height, width=620):
        panel = np.zeros((height, width, 3), dtype=np.uint8)
        action = step.get('action') if isinstance(step, dict) else None
        raw_gripper = None
        if action is not None and len(action) >= 8:
            raw_gripper = float(action[7])

        lines = [
            f'Step {index + 1}/{len(self.steps)}',
            '',
            'EEF obs:',
            f"pos  {self._format_array(obs.get('eef_pos'))}",
            f"quat {self._format_array(obs.get('eef_quat'))}",
            '',
            'Action pose:',
            f"pos  {self._format_array(action[:3] if action is not None and len(action) >= 3 else None)}",
            f"quat {self._format_array(action[3:7] if action is not None and len(action) >= 7 else None)}",
            '',
            'Joint state:',
            f"pos {self._format_array(obs.get('joint_pos'))}",
            f"vel {self._format_array(obs.get('joint_vel'))}",
            '',
            'Gripper:',
            f"action normalized: {raw_gripper}",
            f'command position: {gripper_position}',
            f"qpos {self._format_array(obs.get('gripper_qpos'))}",
            f"qvel {self._format_array(obs.get('gripper_qvel'))}",
            f"done={step.get('done') if isinstance(step, dict) else None}, "
            f"reward={step.get('reward') if isinstance(step, dict) else None}",
        ]

        if issues:
            lines += ['', 'Issues:']
            lines += [issue[:70] for issue in issues[:6]]

        y = 28
        for line in lines:
            if y > height - 12:
                break
            color = (0, 255, 255) if line.startswith('Issues') or line.startswith('missing') else (230, 230, 230)
            cv2.putText(panel, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.48, color, 1, cv2.LINE_AA)
            y += 24 if line else 12

        return panel

    def _compose_preview(self, index, step, obs, gripper_position, issues):
        camera_grid = self._build_camera_grid(obs)
        text_panel = self._build_text_panel(
            index, step, obs, gripper_position, issues, camera_grid.shape[0])
        return np.hstack([camera_grid, text_panel])

    def _update_preview(self, preview):
        """Show the 4 controller RGB+depth camera pairs and the robot state fields
        saved in this rollout step."""
        if not self.show_images:
            return

        try:
            cv2.imshow(self.preview_window_name, preview)
            cv2.waitKey(self.preview_wait_ms)
        except cv2.error as exc:
            self.get_logger().warning(f'Preview display failed: {exc}')
            self.show_images = False

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

    def _record_video_frame(self, preview):
        """Append one full preview frame to the controller rollout video."""
        if not self.save_video:
            return

        if preview is None:
            return

        writer = self._ensure_video_writer(
            'preview_video_writer', f'{self._video_basename}_preview.mp4', preview)
        if writer:
            writer.write(np.ascontiguousarray(preview))

    def _close_video_writers(self):
        writer = getattr(self, 'preview_video_writer', None)
        if writer:
            writer.release()
            self.preview_video_writer = None

    def _controller_gripper_to_command(self, value):
        if value >= self.gripper_closed_threshold:
            return self.gripper_closed_position
        return self.gripper_open_position

    def _check_step(self, step):
        """Validate a single controller rollout step and build its PoseStamped +
        gripper command position.

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

        for label, image_keys, depth_keys in self.CAMERA_FIELDS:
            if self._get_obs_value(obs, image_keys) is None:
                issues.append(f"missing {label} RGB obs field: one of {image_keys}")
            if self._get_obs_value(obs, depth_keys) is None:
                issues.append(f"missing {label} depth obs field: one of {depth_keys}")

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
        gripper_position = self._controller_gripper_to_command(float(action[7]))

        return True, pose, gripper_position, issues

    def replicate_rollout_controller(self):
        try:
            return self._replicate_rollout_controller()
        finally:
            self._close_video_writers()

    def _replicate_rollout_controller(self):
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
            preview = self._compose_preview(index, step, obs, gripper_position, issues)
            self._update_preview(preview)
            self._record_video_frame(preview)
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
                self.compute_pose_error(pose)
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

    def compute_pose_error(self, target_pose):
        # This function computes the error between the target pose and the current pose of the robot.
        # Take the current robot pose
        cnt = 100
        while cnt > 0:
            try:
                # do a node spin_once to process any pending callbacks and update the tf buffer
                rclpy.spin_once(self, timeout_sec=0.1)
                transform = self.tf_buffer.lookup_transform(
                    target_pose.header.frame_id,
                    'tcp_link',  # Assuming 'ee_link' is the end-effector link
                    rclpy.time.Time())
                break
            except Exception as e:
                self.get_logger().warning(f'Failed to get transform: {e}')
                cnt -= 1
                time.sleep(0.1)

        # get position from target pose
        target_position = np.array([
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            target_pose.pose.position.z
        ])

        # get position from current transform
        current_position = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])

        error = np.linalg.norm(current_position - target_position)
        delta = current_position - target_position
        self.get_logger().info(
            f'Pose error: target={target_position}, current={current_position}, '
            f'delta={delta}, norm={error:.6f}'
        )

        


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
        node = ReplicateRolloutController()
        success = node.replicate_rollout_controller()
        node.get_logger().info('Rollout check PASSED.' if success else 'Rollout check FAILED - see warnings above.')
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
