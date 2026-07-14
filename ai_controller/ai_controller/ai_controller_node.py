import importlib
import json
import math
import pickle
import sys
import threading
from pathlib import Path

import message_filters
import numpy as np
import rclpy
import cv2
import tf2_ros
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as RosImage, JointState
from PIL import Image
import os
from moveit_controller_srvs.srv import GoHome, GoToPose
from control_msgs.action import GripperCommand
from ai_controller.utils.utils import _euler2quat, _quat2mat, _mat2euler_sxyz, _normalize_angle, EEF_POS_NAME, EEF_QUAT_NAME, JOINT_POS_NAME, JOINT_VEL_NAME, GRIPPER_QPOS_NAME, GRIPPER_QVEL_NAME

_trajectory_cls = None

def _get_trajectory_cls(node):
    """Lazily resolve dataset_collector_pkg's savers.Trajectory class."""
    global _trajectory_cls
    if _trajectory_cls is None:
        node._add_dataset_collector_scripts_to_path()
        from savers import Trajectory as TrajectoryClass
        _trajectory_cls = TrajectoryClass
    return _trajectory_cls


class DebugTrajectoryUnpickler(pickle.Unpickler):
    """Resolves the 'Trajectory' class saved by dataset_collector_pkg's savers.py."""

    def find_class(self, module, name):
        if module.startswith('multi_task_il') and name == 'Trajectory':
            return importlib.import_module('savers').Trajectory
        return super().find_class(module, name)

class AIControllerNode(Node):
    
    def __init__(self):
        super().__init__('ai_controller_node')
        self.get_logger().info('AI Controller Node has been started.')
        
        # define parameters
        self.declare_parameter('ai_controller_target', 
                                                    'cod_controller')
        self.declare_parameter('model_config_path', 
                                                '/home/ros2_ws/src/ai_controller/checkpoint_folder/Real-1Task-pick_place-Simulated-Agent-Human-Demonstration-UR5e-Agent-MOSAIC-COD-SKIP-0-5-10-15-Batch24/config.yaml')
        self.declare_parameter('frame_id', 
                                        'base_link')
        self.declare_parameter('set_home_service', 
                                                'set_robot_to_home')
        self.declare_parameter('set_pose_service', 
                                                'set_robot_to_pose')
        self.declare_parameter('gripper_action_topic', 
                                                    '/robotiq_gripper_controller/gripper_cmd')
        self.declare_parameter('camera_topic', 
                                            ['/zed_front/zed_node/rgb/color/rect/image', 
                                            '/zed_left/zed_node/rgb/color/rect/image',
                                            '/zed_right/zed_node/rgb/color/rect/image',
                                            '/zed_gripper/zed_node/rgb/color/rect/image'])
        self.declare_parameter('task_name', 
                                            "pick_place")
        self.declare_parameter('demo_path', 
                                            "/dataset/pick_place/human_rgb_pick_place")
        self.declare_parameter('pose_before_first_inference', [-0.15552094619366708,
                                                               0.34869994018501943,
                                                               0.1532803451753288,
                                                               0.9994452044624775,
                                                               0.03161651380119412,
                                                               0.0021438049655468088,
                                                               0.010251021036213035])
        # DEBUG TEST ONLY: when enabled, camera_front_image frames are read from
        # a saved trajectory .pkl file instead of the live camera topics.
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('debug_trajectory_path', '')
        self.declare_parameter('save_rollout_path', '/home/ros2_ws/src/ai_controller/saved_rollouts')
        # Robot-state capture (for recording the executed rollout as a Trajectory)
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('joint_robot_names', ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'])
        self.declare_parameter('gripper_robot_names', ['robotiq_85_left_knuckle_joint'])
        self.declare_parameter('eef_frame_name', 'tcp_link')
        self.declare_parameter('move_robot', False)



        # get parameters
        self.ai_controller_target = self.get_parameter('ai_controller_target').get_parameter_value().string_value
        self.model_config_path = self.get_parameter('model_config_path').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.set_home_service = self.get_parameter('set_home_service').get_parameter_value().string_value
        self.set_pose_service = self.get_parameter('set_pose_service').get_parameter_value().string_value
        self.gripper_action_topic = self.get_parameter('gripper_action_topic').get_parameter_value().string_value
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_array_value
        self.task_name = self.get_parameter('task_name').get_parameter_value().string_value
        self.demo_path = self.get_parameter('demo_path').get_parameter_value().string_value
        self.pose_before_first_inference = self.get_parameter('pose_before_first_inference').get_parameter_value().double_array_value
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        self.debug_trajectory_path = self.get_parameter('debug_trajectory_path').get_parameter_value().string_value
        self.save_rollout_path = self.get_parameter('save_rollout_path').get_parameter_value().string_value
        self.joint_states_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value
        self.joint_robot_names = self.get_parameter('joint_robot_names').get_parameter_value().string_array_value
        self.gripper_robot_names = self.get_parameter('gripper_robot_names').get_parameter_value().string_array_value
        self.eef_frame_name = self.get_parameter('eef_frame_name').get_parameter_value().string_value
        self.move_robot = self.get_parameter('move_robot').get_parameter_value().bool_value
        self.debug_steps = None
        self.debug_step_index = 0
        self.latest_joint_state = None

        # 1. Initialize the AI controller
        self.get_logger().info(f'Initializing AI Controller: {self.ai_controller_target}')
        if self.ai_controller_target == 'cod_controller':
            from ai_controller.models.cod_controller.cod_controller import CODController
            self.controller = CODController(self.model_config_path, self.task_name)
            
        elif self.ai_controller_target == 'openvla_controller':
            from ai_controller.models.openvla_controller.openvla_controller import OpenVLAController
            self.controller = OpenVLAController(self.model_config_path, self.task_name)
        elif self.ai_controller_target == 'tinyvla_controller':
            from ai_controller.models.tinyvla_controller.tinyvla_controller import TinyVLAController
            self.controller = TinyVLAController(self.model_config_path, self.task_name)
        else:
            self.get_logger().error(f'Unknown AI Controller target: {self.ai_controller_target}')
            raise ValueError(f'Unknown AI Controller target: {self.ai_controller_target}')
        
        # add controller  name to save_rollout_path
        self.save_rollout_path = os.path.join(self.save_rollout_path, self.ai_controller_target, self.task_name)
        os.makedirs(self.save_rollout_path, exist_ok=True)
        
        # 2. Set up ROS2 interfaces (publishers, subscribers, services)
        self.get_logger().info(f'Waiting for service {self.set_home_service}...')
        self.set_home_client = self.create_client(GoHome, self.set_home_service)
        self.set_home_client.wait_for_service()
        self.get_logger().info(f'Service {self.set_home_service} is available.')
        
        self.get_logger().info(f'Waiting for service {self.set_pose_service}...')
        self.set_pose_client = self.create_client(GoToPose, self.set_pose_service)
        self.set_pose_client.wait_for_service()
        self.get_logger().info(f'Service {self.set_pose_service} is available.')
        
        self.get_logger().info(f"Creating publisher for gripper action on topic {self.gripper_action_topic}...")
        self.gripper_action_client = ActionClient(
            self,
            GripperCommand,
            self.gripper_action_topic,
        )
        self.get_logger().info(f"Publisher for gripper action created on topic {self.gripper_action_topic}.")

        # Robot-state capture: joint states + TF (base_link -> eef_frame_name), used to
        # record the actually-executed rollout as a Trajectory (see save_rollout()).
        self.create_subscription(JointState, self.joint_states_topic, self._joint_state_callback, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        if self.debug_mode:
            self.get_logger().warning(
                'DEBUG MODE ENABLED: camera topics will NOT be used. '
                f'Frames will be read from debug_trajectory_path={self.debug_trajectory_path!r} instead.'
            )
            self._load_debug_trajectory(self.debug_trajectory_path)
        else:
            # 3. Wait for camera topics to be available
            self.get_logger().info(f'Waiting for camera topics: {self.camera_topic}')
            for camera_topic in self.camera_topic:
                self.get_logger().info(f'Waiting for camera topic: {camera_topic}')
                # wait for the topic to be available
                find = False
                while not find:
                    topics_info = self.get_topic_names_and_types()
                    for topic_info in topics_info:
                        topic_name = topic_info[0]
                        if topic_name == camera_topic:
                            self.get_logger().info(f'Camera topic {camera_topic} is available.')
                            find = True
                            break

                    if not find:
                        self.get_logger().info(f'Camera topic {camera_topic} not available yet. Waiting...')
                        rclpy.spin_once(self, timeout_sec=1.0)

                self.get_logger().info(f'Camera topic {camera_topic} is available.')

            # 4. Set up synchronized camera subscribers
            self.bridge = CvBridge()
            self.latest_synced_images = None
            self.synced_images_event = threading.Event()

            self.camera_subs = [
                message_filters.Subscriber(self, RosImage, topic, qos_profile=qos_profile_sensor_data)
                for topic in self.camera_topic
            ]
            self.camera_sync = message_filters.ApproximateTimeSynchronizer(
                self.camera_subs, queue_size=10, slop=10
            )
            self.camera_sync.registerCallback(self.synced_images_callback)

        self.traj_cnt = 0
        self.max_step = 90
        self.gripper_closed = False
        self.previous_gripper_position = 0.0    

        self.get_logger().info('AI Controller Node initialization complete. Ready to start control loop.')
        self.control_loop()

    def synced_images_callback(self, *image_msgs):
        """Called once per cycle when all camera topics have a message within the sync window."""
        self.latest_synced_images = [
            self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8') for msg in image_msgs
        ]
        self.synced_images_event.set()

    def get_synced_images(self, timeout_sec=5.0):
        """Block (while spinning callbacks) until a fresh synchronized set of camera images arrives."""
        if self.debug_mode:
            return self._get_debug_images()

        self.synced_images_event.clear()
        start = self.get_clock().now()
        while not self.synced_images_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().error('Timed out waiting for synchronized camera images.')
                return None
        return self.latest_synced_images

    def _add_dataset_collector_scripts_to_path(self):
        """Best-effort: make dataset_collector_pkg's savers.Trajectory importable."""
        try:
            from ament_index_python.packages import get_package_share_directory
            scripts_dir = Path(get_package_share_directory('dataset_collector_pkg')) / 'scripts'
        except Exception as exc:
            self.get_logger().error(f'Could not locate dataset_collector_pkg scripts directory: {exc}')
            return

        if scripts_dir.is_dir() and str(scripts_dir) not in sys.path:
            sys.path.append(str(scripts_dir))

    def _load_debug_trajectory(self, trajectory_path):
        """DEBUG TEST ONLY: load a saved trajectory .pkl to replay its camera_front_image frames."""
        if not trajectory_path:
            raise ValueError('Parameter debug_trajectory_path must be set when debug_mode is enabled.')

        path = Path(trajectory_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f'Debug trajectory file does not exist: {path}')

        self._add_dataset_collector_scripts_to_path()

        with path.open('rb') as f:
            data = DebugTrajectoryUnpickler(f).load()

        trajectory = data['traj']
        self.debug_steps = [trajectory[t]['obs'] for t in range(len(trajectory))]
        self.debug_step_index = 0
        self.get_logger().warning(
            f'DEBUG MODE: loaded {len(self.debug_steps)} frames from {path}.'
        )

    def _get_debug_images(self):
        """DEBUG TEST ONLY: return the next camera_front_image from the loaded trajectory."""
        if not self.debug_steps:
            self.get_logger().error('Debug trajectory has no steps to replay.')
            return None

        index = self.debug_step_index % len(self.debug_steps)
        front_image = self.debug_steps[index].get('camera_front_image')
        if front_image is None:
            self.get_logger().error(f'Debug step {index} has no camera_front_image.')
            return None

        if hasattr(front_image, 'ndim') and front_image.ndim == 1:
            front_image = cv2.imdecode(front_image, cv2.IMREAD_COLOR)
            front_image = cv2.cvtColor(front_image, cv2.COLOR_BGR2RGB)

        self.get_logger().info(
            f'[DEBUG] Using camera_front_image from trajectory step {index + 1}/{len(self.debug_steps)}.'
        )
        self.debug_step_index += 1
        return [front_image]

    def _joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def _capture_robot_state(self, timeout_sec=1.0):
        """Spin briefly to receive a fresh /joint_states + TF, then build a robot-state obs dict.

        Field names mirror dataset_collector_pkg/utils.py so rollouts saved here are
        readable by the same tooling (replicate_trajectory.py, cod_controller.py) as
        recorded demonstrations.
        """
        start = self.get_clock().now()
        while self.latest_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - start).nanoseconds / 1e9 > timeout_sec:
                self.get_logger().warning('Timed out waiting for /joint_states message.')
                break

        state = {}
        joint_state = self.latest_joint_state
        if joint_state is not None:
            joint_name_to_index = {name: idx for idx, name in enumerate(joint_state.name)}
            try:
                state[JOINT_POS_NAME] = np.array(
                    [joint_state.position[joint_name_to_index[j]] for j in self.joint_robot_names])
                state[JOINT_VEL_NAME] = np.array(
                    [joint_state.velocity[joint_name_to_index[j]] for j in self.joint_robot_names])
                state[GRIPPER_QPOS_NAME] = np.array(
                    [joint_state.position[joint_name_to_index[j]] for j in self.gripper_robot_names])
                state[GRIPPER_QVEL_NAME] = np.array(
                    [joint_state.velocity[joint_name_to_index[j]] for j in self.gripper_robot_names])
            except KeyError as exc:
                self.get_logger().warning(f'Joint name missing from /joint_states: {exc}')
        else:
            self.get_logger().warning('No /joint_states message received; skipping joint/gripper state fields.')

        try:
            trans = self.tf_buffer.lookup_transform(self.frame_id, self.eef_frame_name, rclpy.time.Time())
            state[EEF_POS_NAME] = np.array([trans.transform.translation.x,
                                            trans.transform.translation.y,
                                            trans.transform.translation.z])
            state[EEF_QUAT_NAME] = np.array([trans.transform.rotation.x,
                                             trans.transform.rotation.y,
                                             trans.transform.rotation.z,
                                             trans.transform.rotation.w])
        except Exception as exc:
            self.get_logger().warning(
                f'Could not look up EEF pose via TF ({self.frame_id} -> {self.eef_frame_name}): {exc}')

        return state

    def _build_openvla_state(self, robot_state):
        """Build the 8-dim proprio vector [eef_x, eef_y, eef_z, roll, pitch, yaw,
        gripper_open, gripper_closed] expected by OpenVLAController (proprio_dim: 8
        in openvla_config.yaml), matching the EEF_state (6) + gripper_state (2)
        fields recorded by real_ur5e_pick_place_delta_removed_0_5_10_15/ur5e_pick_place.py
        (PickPlaceEnv._create_step): eef_pose = [eef_pos(3), eef_euler_rpy(3)] and
        gripper_state = [0.0, last commanded gripper action].
        """
        eef_pos = robot_state.get(EEF_POS_NAME)
        eef_quat = robot_state.get(EEF_QUAT_NAME)
        if eef_pos is None or eef_quat is None:
            self.get_logger().warning(
                'Missing eef_pos/eef_quat from robot state; cannot build OpenVLA proprio state.')
            return None

        euler = _mat2euler_sxyz(_quat2mat(eef_quat))
        euler = np.array([_normalize_angle(angle) for angle in euler])

        gripper_open = 0.0
        gripper_closed = 1.0 if self.gripper_closed else 0.0

        return np.concatenate([
            np.asarray(eef_pos, dtype=np.float64),
            euler,
            [gripper_open, gripper_closed],
        ])

    def _build_tinyvla_state(self, robot_state):
        """Build the 7-dim [eef_x, eef_y, eef_z, qx, qy, qz, qw] vector expected by
        TinyVLAController. Unlike _build_openvla_state, orientation is passed as a
        raw quaternion rather than pre-converted Euler angles: TinyVLAPolicy derives
        its own gripper-frame Euler angles via R_EE_TO_GRIPPER (see tinyvla.py's
        prepare_observation), which uses a different EEF->gripper frame convention
        than OpenVLA's.
        """
        eef_pos = robot_state.get(EEF_POS_NAME)
        eef_quat = robot_state.get(EEF_QUAT_NAME)
        if eef_pos is None or eef_quat is None:
            self.get_logger().warning(
                'Missing eef_pos/eef_quat from robot state; cannot build TinyVLA proprio state.')
            return None

        return np.concatenate([
            np.asarray(eef_pos, dtype=np.float64),
            np.asarray(eef_quat, dtype=np.float64),
        ])

    def move_to_initial_pose(self):
        """Move the robot to the initial pose before starting the control loop."""
        self.get_logger().info('Moving robot to initial pose before first inference...')
        input("Press Enter to move the robot to the initial pose. Make sure the robot is in a safe position.")
        pose_request = GoToPose.Request()
        pose_request.pose.header.stamp = self.get_clock().now().to_msg()
        pose_request.pose.header.frame_id = self.frame_id
        pose_request.pose.pose.position.x = self.pose_before_first_inference[0]
        pose_request.pose.pose.position.y = self.pose_before_first_inference[1]
        pose_request.pose.pose.position.z = self.pose_before_first_inference[2]
        pose_request.pose.pose.orientation.x = self.pose_before_first_inference[3]
        pose_request.pose.pose.orientation.y = self.pose_before_first_inference[4]
        pose_request.pose.pose.orientation.z = self.pose_before_first_inference[5]
        pose_request.pose.pose.orientation.w = self.pose_before_first_inference[6]
        
        future = self.set_pose_client.call_async(pose_request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().error('set_pose_client service call failed.')
            raise RuntimeError('set_pose_client service call failed.')

        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().error(response.message)
            raise RuntimeError(f'Failed to move robot to initial pose: {response.message}')
        
        # Open the gripper to a known position (e.g., fully open) before starting
        self.get_logger().info('Opening gripper to a known position before first inference...')
        gripper_goal = GripperCommand.Goal()
        gripper_goal.command.position = 0.1  # Fully open position (adjust as needed)
        gripper_goal.command.max_effort = 50.0
        future = self.gripper_action_client.send_goal_async(gripper_goal)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Gripper opened successfully before first inference.')
        else:
            self.get_logger().error(f'Failed to open gripper before first inference: {future.exception()}')
            raise RuntimeError(f'Failed to open gripper before first inference: {future.exception()}')
    
    def save_rollout(self, traj=None, save_path=None, task_id=None, traj_number=None):
        """Save the current rollout Trajectory to a .pkl file, plus outcome metadata to a .json file."""
        complete_save_path = os.path.join(save_path, f'task_{task_id}')
        os.makedirs(complete_save_path,
                    exist_ok=True)

        traj_name = 'traj_{:03d}'.format(traj_number)

        if traj is not None:
            trajectory_path = os.path.join(complete_save_path, traj_name + '.pkl')
            traj.save(
                trajectory_path,
                task_id=task_id,
                traj_number=traj_number,
                ai_controller_target=self.ai_controller_target,
                task_name=self.task_name,
            )
            self.get_logger().info(f'Saved rollout trajectory to {trajectory_path}')

        res_dict = dict()
        # 1. Ask for object reached
        object_reached = input("Did the robot successfully reach the target object? [1,0]: ")
        res_dict['object_reached'] = int(object_reached)
        
        # 2. Ask for object picked
        object_picked = input("Did the robot successfully pick the target object? [1,0]: ")
        res_dict['object_picked'] = int(object_picked)
        
        # 3. Ask for object placed
        object_placed = input("Did the robot successfully place the target object? [1,0]: ")
        res_dict['object_placed'] = int(object_placed)
        
        res_dict['reached_wrong'] = 0
        res_dict['picked_wrong'] = 0
        res_dict['place_wrong_correct_bin'] = 0
        res_dict['place_wrong_wrong_bin'] = 0
        if res_dict['object_reached'] != 1:
            reached_wrong = input("Did the robot reach the wrong object? [1,0]: ")
            res_dict['reached_wrong'] = int(reached_wrong)
            
            picked_wrong = input("Did the robot pick the wrong object? [1,0]: ")
            res_dict['picked_wrong'] = int(picked_wrong)
            
            place_wrong_correct_bin = input("Did the robot place the wrong object in correct bin? [1,0]: ")
            res_dict['place_wrong_correct_bin'] = int(place_wrong_correct_bin)
            
            place_wrong_wrong_bin = input("Did the robot place the wrong object in wrong bin? [1,0]: ")
            res_dict['place_wrong_wrong_bin'] = int(place_wrong_wrong_bin)
            
        
        with open(os.path.join(complete_save_path, traj_name + '.json'), 'w') as f:
            json.dump(res_dict, f)

    def control_loop(self):
        """Main control loop for the AI controller."""
        
        self.get_logger().info('Starting control loop...')
        # create a directory to save the images
        save_path = f'/home/ros2_ws/src/ai_controller/saved_images/task_{self.task_name}'
        os.makedirs(save_path, exist_ok=True)

        Trajectory = _get_trajectory_cls(self)
        
        traj_cnt = int(input("Write the current trajectory count to the console: "))
        self.traj_cnt = traj_cnt
        
        while rclpy.ok():
        
            input("Press Enter to start the control loop. Make sure the robot is in a safe position.")
            
            enter_task_id = input("Enter task ID (e.g., 1, 2, 3): ")
            self.get_logger().info(f'Starting control loop for task ID: {enter_task_id}')
            # make task_id like XX
            enter_task_id = enter_task_id.zfill(2)
            
            # create a new trajectory
            traj = Trajectory()
            
            for step in range(self.max_step):
                
                if step == 0:
                    # resetting controller state for the new task
                    self.controller.reset()
                    self.get_logger().info(f'Setting robot to home position for task ID: {enter_task_id}')
                    # call service to set robot to home position
                    # wait for the service to complete
                    future = self.set_home_client.call_async(GoHome.Request())
                    rclpy.spin_until_future_complete(self, future)
                    response = future.result()
                    if response is None:
                        self.get_logger().error('set_home_client service call failed.')
                        raise RuntimeError('set_home_client service call failed.')

                    if response.success:
                        self.get_logger().info(response.message)
                        
                    else:
                        self.get_logger().error(response.message)
                        raise RuntimeError(f'Failed to set robot to home position: {response.message}')      
                    
                    self.move_to_initial_pose()
                    
                    # load the demo data for the given task_id
                    self.get_logger().info(f'Loading demo data for task ID: {enter_task_id}')
                    self.controller.load_command(self.demo_path, 
                                                 enter_task_id,
                                                 save_demo_frames=True,
                                                 traj_cnt=self.traj_cnt,
                                                 save_path=self.save_rollout_path)
                
            
                # 1. Get sensor data (e.g., camera images)
                images = self.get_synced_images()
                if images is None:
                    self.get_logger().error('Skipping step: failed to get synchronized camera images.')
                    continue
                # images is a list of cv2/numpy arrays in the same order as self.camera_topic
                # save the images with PIL format for debugging
                for i, image in enumerate(images):
                    img = Image.fromarray(image)
                    img.save(f'{save_path}/camera_image_{i}.png')

                # capture the robot state (eef pose, joint pos/vel, gripper qpos/qvel) paired
                # with the observation image used for this step's inference
                robot_state = self._capture_robot_state()

                # 2. Get joint-states or other relevant robot states (if needed for inference).
                # Each controller expects a different state format (or none at all), so branch
                # on the loaded model: CODController.pre_process() raises NotImplementedError
                # if states is not None, while OpenVLAController needs the 8-dim proprio vector.
                if self.ai_controller_target == 'openvla_controller':
                    states = self._build_openvla_state(robot_state)
                elif self.ai_controller_target == 'tinyvla_controller':
                    states = self._build_tinyvla_state(robot_state)
                else:
                    states = None

                # 3. Perform inference using the AI controller
                step_save_path = f'{save_path}/step_{step}'
                out = self.controller.inference(
                                                input_data=[images, states],
                                                t=step,
                                                save_path=step_save_path)
                
                predicted_bb = None
                target_obj_prediction = None
                if self.ai_controller_target == 'cod_controller':
                    pred_action, predicted_bb, target_obj_prediction = out
                    actions = [pred_action]
                elif self.ai_controller_target in ('openvla_controller', 'tinyvla_controller'):
                    actions = out
                    # OpenVLAController/TinyVLAController both return a list of
                    # actions wrt base_link frame [x, y, z, roll, pitch, yaw, gripper_position]
                    # convert orientation from roll/pitch/yaw to quaternion
                    for i in range(len(actions)):
                        new_action = np.zeros(8)
                        new_action[:3] = actions[i][:3] # position remains the same
                        roll, pitch, yaw = actions[i][3:6]
                        quat = _euler2quat(roll, pitch, yaw)
                        new_action[3:7] = quat
                        new_action[7] = actions[i][6] # gripper position remains the same
                        actions[i] = new_action

                for indx, action in enumerate(actions):
                    self.get_logger().info(f'Computed Action at step {step} - Indx {indx}: {action}')
                    if self.move_robot:
                        # 5. Send commands to the robot (e.g., set pose, control gripper)
                        # call service to set robot to the desired pose
                        self.get_logger().info(f'\tSetting robot to desired pose at step {step}')
                        # input("Press Enter to set the robot to the desired pose. Make sure the robot is in a safe position.")
                        pose_request = GoToPose.Request()
                        pose_request.pose.header.stamp = self.get_clock().now().to_msg()
                        pose_request.pose.header.frame_id = self.frame_id
                        pose_request.pose.pose.position.x = action[0]
                        pose_request.pose.pose.position.y = action[1]
                        pose_request.pose.pose.position.z = action[2]
                        pose_request.pose.pose.orientation.x = action[3]
                        pose_request.pose.pose.orientation.y = action[4]
                        pose_request.pose.pose.orientation.z = action[5]
                        pose_request.pose.pose.orientation.w = action[6]
                        future = self.set_pose_client.call_async(pose_request)
                        rclpy.spin_until_future_complete(self, future)
                        if future.result() is not None:
                            self.get_logger().info(f'Robot set to desired pose at step {step}')
                        else:
                            self.get_logger().error(f'Service call failed for setting robot to desired pose: {future.exception()}')
                            raise RuntimeError(f'Service call failed for setting robot to desired pose: {future.exception()}')
                        
                        # 6. Control the gripper based on the predicted action
                        self.get_logger().info(f'Controlling gripper at step {step}')   
                        gripper_goal = GripperCommand.Goal()
                        gripper_goal.command.position = action[-1]  # Assuming the last element of action is the gripper position
                        # if self.gripper_closed:
                        #     self.get_logger().info(f'Keeping gripper closed at step {step}')
                        #     gripper_goal.command.position = 255.0  # Keep the gripper closed
                        gripper_goal.command.max_effort = 50.0
                        future = self.gripper_action_client.send_goal_async(gripper_goal)
                        rclpy.spin_until_future_complete(self, future)
                        if future.result() is not None:
                            self.get_logger().info(f'Gripper command sent at step {step}')
                        else:
                            self.get_logger().error(f'Failed to send gripper command: {future.exception()}')
                            raise RuntimeError(f'Failed to send gripper command: {future.exception()}')
                        
                        self.get_logger().info(f'Gripper command position: {gripper_goal.command.position}')
                        if not self.gripper_closed and gripper_goal.command.position == 255.0:
                            self.get_logger().info(f'Gripper is closing at step {step}')
                            self.gripper_closed = True

                # check if a transiction close->open has been made
                episode_done = False
                if self.gripper_closed and gripper_goal.command.position == 0.0:
                    self.get_logger().info(f'Gripper is opening at step {step}')
                    self.gripper_closed = False
                    episode_done = True

                # 7. Record this step (observation image, cropped model input, predicted
                # bounding boxes, computed action and robot state) into the rollout Trajectory
                step_obs = dict(robot_state)
                step_obs['camera_front_image'] = cv2.cvtColor(images[0], cv2.COLOR_RGB2BGR)

                cropped_image_path = os.path.join(step_save_path, 'pre_processed_img_0.png')
                if os.path.isfile(cropped_image_path):
                    step_obs['cropped_image'] = np.array(Image.open(cropped_image_path))
                else:
                    self.get_logger().warning(
                        f'No cropped model-input image found at {cropped_image_path}; skipping cropped_image field.')

                if predicted_bb is not None:
                    step_obs['predicted_bb'] = predicted_bb.detach().cpu().numpy() if hasattr(predicted_bb, 'detach') else predicted_bb

                traj.append(
                    obs=step_obs,
                    action=action,
                    done=episode_done,
                    reward=1 if episode_done else 0,
                )

                if episode_done:
                    break  # exit the loop if the gripper has opened after being closed

            self.save_rollout(
                              traj=traj,
                              save_path=self.save_rollout_path,
                              task_id=enter_task_id,
                              traj_number=self.traj_cnt
                              )
            self.traj_cnt += 1
                    
        

def main(args=None):
    rclpy.init()
    node = AIControllerNode()
    executor = MultiThreadedExecutor(num_threads=1)
    executor.add_node(node)
    
    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        node.control_loop()
        executor.spin()
        node.get_logger().info('Shutting down AIControllerNode\n')
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    node.destroy_node()
    rclpy.shutdown()