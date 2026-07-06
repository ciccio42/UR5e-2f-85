import importlib
import pickle
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
from moveit_controller_srvs.srv import GoHome, GoToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np



class TrajectoryUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if module.startswith('multi_task_il') and name == 'Trajectory':
            return importlib.import_module('savers').Trajectory
        return super().find_class(module, name)


def add_scripts_dirs_to_path():
    candidate_dirs = [Path(__file__).resolve().parents[1] / 'scripts']

    try:
        from ament_index_python.packages import get_package_share_directory
        candidate_dirs.append(Path(get_package_share_directory('dataset_collector_pkg')) / 'scripts')
    except Exception:
        pass

    for scripts_dir in candidate_dirs:
        if scripts_dir.is_dir() and str(scripts_dir) not in sys.path:
            sys.path.append(str(scripts_dir))


class ReplicateTrajectory(Node):
    def __init__(self):
        super().__init__('replicate_trajectory')

        self.declare_parameter('trajectory_path', '')
        self.declare_parameter('set_home_service', 'set_robot_to_home')
        self.declare_parameter('set_pose_service', 'set_robot_to_pose')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('execute_on_start', False)
        self.declare_parameter('move_home_before_start', False)
        self.declare_parameter('step_delay_sec', 0.0)
        self.declare_parameter('service_timeout_sec', 10.0)
        self.declare_parameter('replay_gripper', True)
        self.declare_parameter('gripper_action_topic', '/robotiq_gripper_controller/gripper_cmd')
        self.declare_parameter('gripper_open_position', 0.1)
        self.declare_parameter('gripper_closed_position', 0.8)
        self.declare_parameter('gripper_closed_threshold', 0.5)
        self.declare_parameter('gripper_raw_open_value', 0.0)
        self.declare_parameter('gripper_raw_closed_value', 255.0)
        self.declare_parameter('gripper_max_effort', 50.0)
        self.declare_parameter('gripper_result_timeout_sec', 10.0)

        self.trajectory_path = self.get_parameter('trajectory_path').value
        self.get_logger().info(f'Loading trajectory from: {self.trajectory_path}')
        self.set_home_service_name = self.get_parameter('set_home_service').value
        self.set_pose_service_name = self.get_parameter('set_pose_service').value
        self.frame_id = self.get_parameter('frame_id').value
        self.execute_on_start = self.get_parameter('execute_on_start').value
        self.move_home_before_start = self.get_parameter('move_home_before_start').value
        self.step_delay_sec = float(self.get_parameter('step_delay_sec').value)
        self.service_timeout_sec = float(self.get_parameter('service_timeout_sec').value)
        self.replay_gripper = bool(self.get_parameter('replay_gripper').value)
        self.gripper_action_topic = self.get_parameter('gripper_action_topic').value
        self.gripper_open_position = float(self.get_parameter('gripper_open_position').value)
        self.gripper_closed_position = float(self.get_parameter('gripper_closed_position').value)
        self.gripper_closed_threshold = float(self.get_parameter('gripper_closed_threshold').value)
        self.gripper_raw_open_value = float(self.get_parameter('gripper_raw_open_value').value)
        self.gripper_raw_closed_value = float(self.get_parameter('gripper_raw_closed_value').value)
        self.gripper_max_effort = float(self.get_parameter('gripper_max_effort').value)
        self.gripper_result_timeout_sec = float(self.get_parameter('gripper_result_timeout_sec').value)
        self.last_gripper_closed = None
        self.front_camera_topic = '/zed_front/zed_node/left/color/rect/image/compressed'
        self.front_camera_window_name = 'Front Camera Preview'
        self.latest_front_camera_frame = None

        self.front_camera_subscription = self.create_subscription(
            CompressedImage,
            self.front_camera_topic,
            self._front_camera_callback,
            10,
        )

        try:
            cv2.namedWindow(self.front_camera_window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.front_camera_window_name, 1280, 360)
            cv2.moveWindow(self.front_camera_window_name, 0, 0)
        except cv2.error as exc:
            self.get_logger().warning(f'Could not create front camera preview window: {exc}')
            self.front_camera_window_name = None

        self.get_logger().info(f'Parameters:\n'
                               f'  trajectory_path: {self.trajectory_path}\n'
                               f'  set_home_service: {self.set_home_service_name}\n'
                               f'  set_pose_service: {self.set_pose_service_name}\n'
                               f'  frame_id: {self.frame_id}\n'
                               f'  execute_on_start: {self.execute_on_start}\n'
                               f'  move_home_before_start: {self.move_home_before_start}\n'
                               f'  step_delay_sec: {self.step_delay_sec}\n'
                               f'  service_timeout_sec: {self.service_timeout_sec}\n'
                               f'  replay_gripper: {self.replay_gripper}\n'
                               f'  gripper_action_topic: {self.gripper_action_topic}\n'
                               f'  gripper_open_position: {self.gripper_open_position}\n'
                               f'  gripper_closed_position: {self.gripper_closed_position}\n'
                               f'  gripper_closed_threshold: {self.gripper_closed_threshold}\n'
                               f'  gripper_raw_open_value: {self.gripper_raw_open_value}\n'
                               f'  gripper_raw_closed_value: {self.gripper_raw_closed_value}\n'
                               f'  gripper_max_effort: {self.gripper_max_effort}\n'
                               f'  gripper_result_timeout_sec: {self.gripper_result_timeout_sec}')

        self.go_home_client = self.create_client(GoHome, self.set_home_service_name)
        self.go_to_pose_client = self.create_client(GoToPose, self.set_pose_service_name)
        self.gripper_action_client = ActionClient(
            self,
            GripperCommand,
            self.gripper_action_topic,
        )

        self.trajectory = self._load_trajectory(self.trajectory_path)['traj']
        self.steps = self._trajectory_to_steps(self.trajectory)

        self.get_logger().info(
            'ReplicateTrajectory initialized. '
            f'Loaded {len(self.steps)} steps from {self.trajectory_path!r}.'
        )

    def wait_for_moveit_services(self):
        self.get_logger().info('Waiting for MoveIt controller services...')
        home_ready = self.go_home_client.wait_for_service(timeout_sec=self.service_timeout_sec)
        pose_ready = self.go_to_pose_client.wait_for_service(timeout_sec=self.service_timeout_sec)
        gripper_ready = True
        if self.replay_gripper:
            gripper_ready = self.gripper_action_client.wait_for_server(timeout_sec=self.service_timeout_sec)

        if not home_ready:
            self.get_logger().error(f'Service not available: {self.set_home_service_name}')
        if not pose_ready:
            self.get_logger().error(f'Service not available: {self.set_pose_service_name}')
        if not gripper_ready:
            self.get_logger().error(f'Action server not available: {self.gripper_action_topic}')

        return home_ready and pose_ready and gripper_ready

    def replicate_trajectory(self):
        if not self.wait_for_moveit_services():
            return False

        if self.move_home_before_start and not self.call_go_home():
            return False

        self._update_front_camera_preview()

        for index, step in enumerate(self.steps):
            self._update_front_camera_preview(step)
            pose = self._step_to_pose_stamped(step)
            if index == 0:
                self.get_logger().info(f'Initial step pose: {pose}')
                raise RuntimeError('Initial step pose is not None; ensure the robot is in the correct starting position before executing the trajectory.')
            self.get_logger().info(f'Step action: {step["gripper_qpos"]}')
            if pose is None:
                self.get_logger().warn(f'Step {index} has no supported pose/action data, skipping.')
                continue

            self.get_logger().info(f'Sending trajectory step {index + 1}/{len(self.steps)}')
            if not self.call_go_to_pose(pose):
                self.get_logger().error(f'Failed while executing trajectory step {index}.')
                return False

            if self.replay_gripper and not self.call_gripper_for_step(step):
                self.get_logger().error(f'Failed while commanding gripper at trajectory step {index}.')
                return False

            if self.step_delay_sec > 0.0:
                time.sleep(self.step_delay_sec)

            self._update_front_camera_preview(step)

        self.get_logger().info('Trajectory replication completed.')
        return True

    def _front_camera_callback(self, msg):
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8)
            decoded = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            if decoded is not None:
                self.latest_front_camera_frame = decoded
        except Exception as exc:
            self.get_logger().warning(f'Failed to decode front camera preview frame: {exc}')

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

        if isinstance(image, (bytes, bytearray, memoryview, np.ndarray)):
            buffer = np.frombuffer(image, dtype=np.uint8) if not isinstance(image, np.ndarray) else image.astype(np.uint8, copy=False)
            decoded = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
            return decoded

        return None

    def _extract_step_front_image(self, step):
        if not isinstance(step, dict):
            return None

        front_image = step.get('camera_front_image')
        if front_image is None:
            obs = step.get('obs', step)
            if isinstance(obs, dict):
                front_image = obs.get('camera_front_image')

        return self._ensure_bgr_image(front_image)

    def _update_front_camera_preview(self, step=None):
        try:
            rclpy.spin_once(self, timeout_sec=0.0)
        except Exception:
            pass

        if self.front_camera_window_name is None or self.latest_front_camera_frame is None:
            return

        trajectory_frame = self._extract_step_front_image(step)
        live_frame = self.latest_front_camera_frame

        if trajectory_frame is None:
            trajectory_frame = np.zeros_like(live_frame)

        target_height = min(live_frame.shape[0], trajectory_frame.shape[0])
        live_frame_resized = cv2.resize(live_frame, (int(live_frame.shape[1] * target_height / live_frame.shape[0]), target_height))
        trajectory_frame_resized = cv2.resize(
            trajectory_frame,
            (int(trajectory_frame.shape[1] * target_height / trajectory_frame.shape[0]), target_height),
        )

        cv2.putText(
            live_frame_resized,
            'Live /zed_front/zed_node/left/color/rect/image/compressed',
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            trajectory_frame_resized,
            "Trajectory step['camera_front_image']",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
        )

        combined = np.hstack([live_frame_resized, trajectory_frame_resized])

        try:
            cv2.imshow(self.front_camera_window_name, combined)
            cv2.waitKey(1)
        except cv2.error as exc:
            self.get_logger().warning(f'Front camera preview display failed: {exc}')
            self.front_camera_window_name = None

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

    def call_gripper_for_step(self, step):
        gripper_closed = self._extract_gripper_closed(step)
        if gripper_closed is None:
            return True

        if self.last_gripper_closed is not None and gripper_closed == self.last_gripper_closed:
            return True

        position = self.gripper_closed_position if gripper_closed else self.gripper_open_position
        state_name = 'closed' if gripper_closed else 'open'
        self.get_logger().info(
            f'Sending gripper command: {state_name} position={position}, max_effort={self.gripper_max_effort}'
        )

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = self.gripper_max_effort

        future = self.gripper_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=self.gripper_result_timeout_sec,
        )
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

        self.get_logger().info('Gripper command accepted; waiting for action completion...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=self.gripper_result_timeout_sec,
        )
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

        result = result_response.result
        self.get_logger().info(
            'Gripper action completed: '
            f'position={result.position}, effort={result.effort}, '
            f'stalled={result.stalled}, reached_goal={result.reached_goal}'
        )

        self.last_gripper_closed = gripper_closed
        return True

    def _load_trajectory(self, trajectory_path):
        if not trajectory_path:
            raise ValueError('Parameter trajectory_path must point to a .pkl trajectory file.')

        path = Path(trajectory_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f'Trajectory file does not exist: {path}')

        add_scripts_dirs_to_path()

        with path.open('rb') as f:
            return TrajectoryUnpickler(f).load()

    def _trajectory_to_steps(self, trajectory):
        self.get_logger().info('Extracting steps from trajectory...\nKeys in each step: ' + ', '.join(trajectory[0]['obs'].keys() if len(trajectory) > 0 else []))
        steps = []
        for t in range(len(trajectory)):
            step_data = trajectory[t]
            steps.append(step_data['obs'])
        return steps

    def _raw_gripper_to_action_position(self, raw_value):
        raw_span = self.gripper_raw_closed_value - self.gripper_raw_open_value
        if raw_span == 0.0:
            return self.gripper_open_position

        ratio = (raw_value - self.gripper_raw_open_value) / raw_span
        ratio = min(max(ratio, 0.0), 1.0)
        return self.gripper_open_position + ratio * (
            self.gripper_closed_position - self.gripper_open_position
        )

    def _extract_gripper_closed(self, step):
        if not isinstance(step, dict):
            return None

        action = step.get('action')
        if action is not None and len(action) >= 8:
            return float(action[7]) >= self.gripper_closed_threshold

        obs = step.get('obs', step)
        if not isinstance(obs, dict):
            return None

        gripper_qpos = obs.get('gripper_qpos')
        if gripper_qpos is None:
            return None

        try:
            gripper_value = float(gripper_qpos[0])
        except (TypeError, IndexError):
            gripper_value = float(gripper_qpos)

        converted_position = self._raw_gripper_to_action_position(gripper_value)
        self.get_logger().info(
            f'Raw trajectory gripper_qpos={gripper_value} converted_position={converted_position} '
            f'(closed threshold={self.gripper_closed_threshold})'
        )
        return converted_position >= self.gripper_closed_threshold

    def _step_to_pose_stamped(self, step):
        pose_values = self._extract_pose_values(step)
        if pose_values is None:
            return None

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(pose_values[0])
        pose.pose.position.y = float(pose_values[1])
        pose.pose.position.z = float(pose_values[2])
        pose.pose.orientation.x = float(pose_values[3])
        pose.pose.orientation.y = float(pose_values[4])
        pose.pose.orientation.z = float(pose_values[5])
        pose.pose.orientation.w = float(pose_values[6])
        return pose

    def _extract_pose_values(self, step):
        if isinstance(step, dict):
            action = step.get('action')
            if action is not None and len(action) >= 7:
                return action[:7]

            obs = step.get('obs', step)
            if isinstance(obs, dict):
                eef_pos = obs.get('eef_pos')
                eef_quat = obs.get('eef_quat')
                if eef_pos is not None and eef_quat is not None:
                    return list(eef_pos[:3]) + list(eef_quat[:4])

                pose = obs.get('pose')
                if pose is not None and len(pose) >= 7:
                    return pose[:7]

        if isinstance(step, (list, tuple)) and len(step) >= 7:
            return step[:7]

        return None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ReplicateTrajectory()
        node.replicate_trajectory()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
