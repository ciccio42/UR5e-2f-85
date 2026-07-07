import importlib
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

        self.get_logger().info(
            f"{'[DRY-RUN] ' if self.dry_run else ''}"
            f'Loaded {len(self.steps)} steps from {self.rollout_path!r}.'
        )

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

        combined = np.hstack([front_resized, cropped_resized])

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
            self._update_preview(index, step.get('obs', {}) if isinstance(step, dict) else {})
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
