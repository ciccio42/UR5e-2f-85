import importlib
import json
import math
import pickle
import sys
import threading
import time
import traceback
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
from sensor_msgs.msg import Image as RosImage, JointState, CameraInfo
from PIL import Image
import os
from moveit_controller_srvs.srv import GoHome, GoToPose
from control_msgs.action import GripperCommand
from ai_controller.utils.utils import _euler2quat, _quat2mat, _mat2euler_sxyz, _normalize_angle, EEF_POS_NAME, EEF_QUAT_NAME, JOINT_POS_NAME, JOINT_VEL_NAME, GRIPPER_QPOS_NAME, GRIPPER_QVEL_NAME
from ai_controller.models.seedo_controller.timing_utils import TIMING
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
        self.declare_parameter(
            'seedo_execute_gripper',
            False,
        )

        # SeeDo-specific parameters
        self.declare_parameter(
            'seedo_depth_topic',
            '/zed_front/zed_node/depth/depth_registered',
        )
        self.declare_parameter(
            'seedo_record_depth_topics',
            [
                '/zed_front/zed_node/depth/depth_registered',
                '/zed_left/zed_node/depth/depth_registered',
                '/zed_right/zed_node/depth/depth_registered',
                '/zed_gripper/zed_node/depth/depth_registered',
            ],
        )
        self.declare_parameter(
            'seedo_camera_info_topic',
            '/zed_front/zed_node/rgb/color/rect/camera_info',
        )
        self.declare_parameter(
            'seedo_rgb_topic',
            '/zed_front/zed_node/rgb/color/rect/image',
        )
        self.declare_parameter(
            'seedo_table_frame',
            'table_0',
        )
        self.declare_parameter(
            'seedo_artifacts_dir',
            '',
        )
        self.declare_parameter(
            'seedo_precomputed_action_plan_path',
            '',
        )

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
        self.seedo_execute_gripper = self.get_parameter(
            'seedo_execute_gripper'
        ).get_parameter_value().bool_value

        self.seedo_depth_topic = self.get_parameter(
            'seedo_depth_topic'
        ).get_parameter_value().string_value

        self.seedo_record_depth_topics = (
            self.get_parameter(
                'seedo_record_depth_topics'
            )
            .get_parameter_value()
            .string_array_value
        )

        self.seedo_camera_info_topic = self.get_parameter(
            'seedo_camera_info_topic'
        ).get_parameter_value().string_value

        self.seedo_rgb_topic = self.get_parameter(
            'seedo_rgb_topic'
        ).get_parameter_value().string_value

        self.seedo_table_frame = self.get_parameter(
            'seedo_table_frame'
        ).get_parameter_value().string_value

        self.seedo_artifacts_dir = self.get_parameter(
            'seedo_artifacts_dir'
        ).get_parameter_value().string_value

        self.seedo_precomputed_action_plan_path = self.get_parameter(
            'seedo_precomputed_action_plan_path'
        ).get_parameter_value().string_value

        self.seedo_record_camera_names = (
            'camera_front',
            'camera_lateral_left',
            'camera_lateral_right',
            'eye_in_hand',
        )

        self.debug_steps = None
        self.debug_step_index = 0
        self.latest_joint_state = None
        self.joint_state_event = threading.Event()

        self.pause_executor = threading.Event()

        # Runtime context used to persist failure information
        # even when the rollout terminates with an exception.
        self.current_task_id = None
        self.current_failure_stage = None
        self.current_failure_step = None
        self.current_failure_action_index = None

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

        elif self.ai_controller_target == 'seedo_controller':
            from ai_controller.models.seedo_controller.seedo_controller import (
                SeeDoController,
            )

            self.controller = SeeDoController(
                model_config=self.model_config_path,
            )

        else:
            self.get_logger().error(f'Unknown AI Controller target: {self.ai_controller_target}')
            raise ValueError(f'Unknown AI Controller target: {self.ai_controller_target}')
        
        # add controller  name to save_rollout_path
        self.save_rollout_path = os.path.join(self.save_rollout_path, self.ai_controller_target, self.task_name)
        if self.ai_controller_target == 'cod_controller':
            # further split rollouts by checkpoint epoch/step and whether the
            # wrist/eye-in-hand image was used, so runs from different
            # checkpoints don't get mixed together on disk.
            epoch = getattr(self.controller, 'epoch', 'unknown')
            wrist_dir = 'wrist' if getattr(self.controller, 'use_wrist_img', False) else 'no_wrist'
            self.save_rollout_path = os.path.join(self.save_rollout_path, f'epoch_{epoch}', wrist_dir)
        os.makedirs(self.save_rollout_path, exist_ok=True)
        
        # 2. Set up ROS2 interfaces (publishers, subscribers, services)
        if self.ai_controller_target != 'seedo_controller':
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

        elif self.move_robot:

            self.get_logger().info(
                f'Waiting for service {self.set_home_service}...'
            )

            self.set_home_client = self.create_client(
                GoHome,
                self.set_home_service,
            )

            self.set_home_client.wait_for_service()

            self.get_logger().info(
                f'Service {self.set_home_service} is available.'
            )

            # GoToPose is required for both plan-only and real execution.
            self.get_logger().info(
                f'Waiting for service {self.set_pose_service}...'
            )

            self.set_pose_client = self.create_client(
                GoToPose,
                self.set_pose_service,
            )

            self.set_pose_client.wait_for_service()

            self.get_logger().info(
                f'Service {self.set_pose_service} is available.'
            )

            if self.seedo_execute_gripper:
                self.get_logger().info(
                    f'Creating gripper action client on '
                    f'{self.gripper_action_topic}...'
                )

                self.gripper_action_client = ActionClient(
                    self,
                    GripperCommand,
                    self.gripper_action_topic,
                )

                self.get_logger().info(
                    f'Waiting for gripper action server '
                    f'{self.gripper_action_topic}...'
                )

                if not self.gripper_action_client.wait_for_server(
                    timeout_sec=10.0
                ):
                    raise RuntimeError(
                        f'Gripper action server '
                        f'{self.gripper_action_topic} '
                        f'is not available.'
                    )

                self.get_logger().info(
                    f'Gripper action server '
                    f'{self.gripper_action_topic} is available.'
                )

            else:
                self.get_logger().warning(
                    'SeeDo gripper execution is DISABLED.'
                )

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
            if self.ai_controller_target != 'seedo_controller':
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

            # SeeDo runtime perception state
            self.seedo_rgb_msg = None
            self.seedo_depth_msg = None
            self.seedo_camera_info_msg = None

            self.seedo_rgbd_event = threading.Event()

            # Latest RGB/depth messages used only for rollout recording.
            # The callbacks store ROS messages without converting them to numpy,
            # keeping the runtime overhead minimal.
            self.seedo_record_rgb_msgs = {}
            self.seedo_record_depth_msgs = {}

            self.seedo_record_lock = threading.Lock()

            if self.ai_controller_target != 'seedo_controller':
                self.camera_subs = [
                    message_filters.Subscriber(
                        self,
                        RosImage,
                        topic,
                        qos_profile=qos_profile_sensor_data,
                    )
                    for topic in self.camera_topic
                ]

                self.camera_sync = (
                    message_filters.ApproximateTimeSynchronizer(
                        self.camera_subs,
                        queue_size=10,
                        slop=100,
                    )
                )

                self.camera_sync.registerCallback(
                    self.synced_images_callback
                )

            # SeeDo RGB-D subscribers
            if self.ai_controller_target == 'seedo_controller':
                self.seedo_rgb_sub = message_filters.Subscriber(
                    self,
                    RosImage,
                    self.seedo_rgb_topic,
                    qos_profile=qos_profile_sensor_data,
                )

                self.seedo_depth_sub = message_filters.Subscriber(
                    self,
                    RosImage,
                    self.seedo_depth_topic,
                    qos_profile=qos_profile_sensor_data,
                )

                self.seedo_rgbd_sync = (
                    message_filters.ApproximateTimeSynchronizer(
                        [
                            self.seedo_rgb_sub,
                            self.seedo_depth_sub,
                        ],
                        queue_size=10,
                        slop=0.1,
                    )
                )

                self.seedo_rgbd_sync.registerCallback(
                    self._seedo_rgbd_callback
                )

                self.seedo_camera_info_sub = self.create_subscription(
                    CameraInfo,
                    self.seedo_camera_info_topic,
                    self._seedo_camera_info_callback,
                    qos_profile_sensor_data,
                )

                # Additional cameras used only for rollout recording.
                # Front RGB-D is already handled by _seedo_rgbd_callback,
                # therefore only cameras 1..3 are subscribed here.

                self.seedo_record_rgb_subs = []
                self.seedo_record_depth_subs = []

                for index in range(1, len(self.seedo_record_camera_names)):

                    camera_name = self.seedo_record_camera_names[index]

                    rgb_topic = self.camera_topic[index]
                    depth_topic = self.seedo_record_depth_topics[index]

                    rgb_sub = self.create_subscription(
                        RosImage,
                        rgb_topic,
                        lambda msg, name=camera_name:
                            self._seedo_record_rgb_callback(
                                msg,
                                name,
                            ),
                        qos_profile_sensor_data,
                    )

                    depth_sub = self.create_subscription(
                        RosImage,
                        depth_topic,
                        lambda msg, name=camera_name:
                            self._seedo_record_depth_callback(
                                msg,
                                name,
                            ),
                        qos_profile_sensor_data,
                    )

                    self.seedo_record_rgb_subs.append(
                        rgb_sub
                    )

                    self.seedo_record_depth_subs.append(
                        depth_sub
                    )

        self.traj_cnt = 0
        self.max_step = 90
        self.gripper_closed = False
        self.previous_gripper_position = 0.0    

        self.get_logger().info('AI Controller Node initialization complete. Ready to start control loop.')
        # self.control_loop()

    def _wait_for_future(self, future, timeout_sec=None):
        done_event = threading.Event()

        future.add_done_callback(
            lambda _: done_event.set()
        )

        if not done_event.wait(timeout=timeout_sec):
            return None

        return future.result()

    def _lookup_transform(
        self,
        target_frame,
        source_frame,
        max_attempts=100,
    ):
        last_exc = None

        for attempt in range(1, max_attempts + 1):
            try:
                return self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),
                )

            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as exc:
                last_exc = exc

                self.get_logger().warning(
                    f'lookup_transform('
                    f'{target_frame} -> {source_frame}) failed '
                    f'(attempt {attempt}/{max_attempts}): {exc}'
                )

                if self.ai_controller_target == 'seedo_controller':
                    time.sleep(0.1)
                else:
                    rclpy.spin_once(
                        self,
                        timeout_sec=0.1,
                    )

        raise RuntimeError(
            f'Failed to look up transform '
            f'{target_frame} -> {source_frame} after '
            f'{max_attempts} attempts: {last_exc}'
        )

    def _get_seedo_base_to_table_transform(self):
        """Return the current table_0 -> base_link transform in SeeDo format.

        The returned transform maps points expressed in table_0 coordinates
        into base_link coordinates:

            p_base = R_base_table @ p_table + t_base_table

        Returns
        -------
        dict
            {
                "rotation": np.ndarray shape (3, 3),
                "translation": np.ndarray shape (3,)
            }
        """
        transform = self._lookup_transform(
            self.frame_id,          # base_link
            self.seedo_table_frame  # table_0
        )

        t = transform.transform.translation
        q = transform.transform.rotation

        quat = np.array(
            [q.x, q.y, q.z, q.w],
            dtype=np.float64,
        )

        return {
            "rotation": _quat2mat(quat),
            "translation": np.array(
                [t.x, t.y, t.z],
                dtype=np.float64,
            ),
        }

    def _seedo_rgbd_callback(
        self,
        rgb_msg: RosImage,
        depth_msg: RosImage,
    ):
        self.seedo_rgb_msg = rgb_msg
        self.seedo_depth_msg = depth_msg

        with self.seedo_record_lock:
            self.seedo_record_rgb_msgs[
                'camera_front'
            ] = rgb_msg

            self.seedo_record_depth_msgs[
                'camera_front'
            ] = depth_msg

        self.seedo_rgbd_event.set()

    def _seedo_record_rgb_callback(
        self,
        msg: RosImage,
        camera_name: str,
    ):
        with self.seedo_record_lock:
            self.seedo_record_rgb_msgs[
                camera_name
            ] = msg


    def _seedo_record_depth_callback(
        self,
        msg: RosImage,
        camera_name: str,
    ):
        with self.seedo_record_lock:
            self.seedo_record_depth_msgs[
                camera_name
            ] = msg

    def _get_seedo_record_camera_data(
        self,
        timeout_sec=5.0,
    ):
        """
        Return the latest RGB + depth frame from all four cameras
        using the same observation keys as the reference dataset.
        """

        deadline = time.monotonic() + timeout_sec

        expected_names = self.seedo_record_camera_names

        # Wait until at least one RGB and depth frame has been received
        # from every camera.
        while time.monotonic() < deadline:

            with self.seedo_record_lock:
                rgb_ready = all(
                    name in self.seedo_record_rgb_msgs
                    for name in expected_names
                )

                depth_ready = all(
                    name in self.seedo_record_depth_msgs
                    for name in expected_names
                )

            if rgb_ready and depth_ready:
                break

            time.sleep(0.01)

        with self.seedo_record_lock:
            missing_rgb = [
                name
                for name in expected_names
                if name not in self.seedo_record_rgb_msgs
            ]

            missing_depth = [
                name
                for name in expected_names
                if name not in self.seedo_record_depth_msgs
            ]

            if missing_rgb or missing_depth:
                raise RuntimeError(
                    "Missing rollout camera data. "
                    f"RGB missing: {missing_rgb}; "
                    f"Depth missing: {missing_depth}"
                )

            # Keep local references so the callbacks can continue updating
            # the dictionaries while conversion is performed.
            rgb_msgs = {
                name: self.seedo_record_rgb_msgs[name]
                for name in expected_names
            }

            depth_msgs = {
                name: self.seedo_record_depth_msgs[name]
                for name in expected_names
            }

        camera_data = {}

        dataset_keys = {
            "camera_front": (
                "camera_front_image",
                "camera_front_depth",
            ),
            "camera_lateral_left": (
                "camera_lateral_left_image",
                "camera_lateral_left_depth",
            ),
            "camera_lateral_right": (
                "camera_lateral_right_image",
                "camera_lateral_right_depth",
            ),
            "eye_in_hand": (
                "eye_in_hand_image",
                "eye_in_hand_depth",
            ),
        }

        for camera_name in expected_names:

            rgb_key, depth_key = dataset_keys[
                camera_name
            ]

            # Dataset RGB images are stored in BGR/OpenCV format.
            rgb_image = self.bridge.imgmsg_to_cv2(
                rgb_msgs[camera_name],
                desired_encoding="bgr8",
            )

            # Keep native depth representation (normally float32).
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msgs[camera_name],
                desired_encoding="passthrough",
            )

            camera_data[rgb_key] = np.asarray(
                rgb_image
            )

            camera_data[depth_key] = np.asarray(
                depth_image,
                dtype=np.float32,
            )

        return camera_data


    def _compress_seedo_dataset_rgb(
        self,
        camera_data,
    ):
        """
        JPEG-compress only the RGB images stored in the SeeDo
        dataset rollout. This does not affect legacy controllers.
        """

        rgb_keys = (
            "camera_front_image",
            "camera_lateral_left_image",
            "camera_lateral_right_image",
            "eye_in_hand_image",
        )

        compressed = dict(
            camera_data
        )

        for key in rgb_keys:
            if key not in compressed:
                raise RuntimeError(
                    f"Missing SeeDo rollout RGB image: {key}"
                )

            image = np.asarray(
                compressed[key]
            )

            if image.ndim != 3 or image.shape[2] != 3:
                raise RuntimeError(
                    f"Invalid image shape for {key}: "
                    f"{image.shape}"
                )

            okay, encoded = cv2.imencode(
                ".jpg",
                image,
            )

            if not okay:
                raise RuntimeError(
                    f"JPEG encoding failed for {key}"
                )

            compressed[key] = encoded

        return compressed

    def _build_seedo_front_obj_bb(self):
        """
        Build the reference-dataset obj_bb structure from the
        runtime ScenePerceiver detections.

        Bounding boxes are derived from the SAM masks produced
        on the live front-camera scene.
        """

        perception_result = (
            self.controller.perception_result
        )

        scene_state = (
            self.controller.scene_state
        )

        if perception_result is None:
            raise RuntimeError(
                "SeeDo perception_result is not available."
            )

        if scene_state is None:
            raise RuntimeError(
                "SeeDo scene_state is not available."
            )

        raw_objects = (
            perception_result
            .raw_scene
            .objects
        )

        semantic_objects = (
            scene_state.objects
        )

        if len(raw_objects) != len(semantic_objects):
            raise RuntimeError(
                "Raw and semantic scene object counts differ: "
                f"{len(raw_objects)} != "
                f"{len(semantic_objects)}"
            )

        semantic_to_dataset_name = {
            "red cube": "redbox",
            "green cube": "greenbox",
            "blue cube": "bluebox",
            "yellow cube": "yellowbox",

            "first bin from the left": "bin_0",
            "second bin from the left": "bin_1",
            "third bin from the left": "bin_2",
            "fourth bin from the left": "bin_3",
        }

        camera_front_bb = {}

        for raw_object, semantic_object in zip(
            raw_objects,
            semantic_objects,
        ):
            semantic_name = (
                semantic_object.object_id
                .strip()
                .lower()
            )

            if semantic_name in semantic_to_dataset_name:
                dataset_name = semantic_to_dataset_name[
                    semantic_name
                ]
            else:
                # Generic semantic objects introduced by the new
                # ScenePerceiver, e.g.:
                #
                #   "blue ring"      -> "blue_ring"
                #   "red cylinder"   -> "red_cylinder"
                #   "green star"     -> "green_star"
                #
                # The four legacy cubes and storage bins keep their
                # original dataset-compatible names through the map above.
                dataset_name = (
                    semantic_name
                    .replace(" ", "_")
                )

            mask = raw_object.mask

            if mask is None:
                raise RuntimeError(
                    f"Object {semantic_name!r} has no SAM mask."
                )

            mask = np.asarray(
                mask,
                dtype=bool,
            )

            ys, xs = np.where(mask)

            if xs.size == 0 or ys.size == 0:
                raise RuntimeError(
                    f"Object {semantic_name!r} has an empty SAM mask."
                )

            x_min = int(xs.min())
            x_max = int(xs.max())

            y_min = int(ys.min())
            y_max = int(ys.max())

            center_x = int(
                np.rint(
                    (x_min + x_max) / 2.0
                )
            )

            center_y = int(
                np.rint(
                    (y_min + y_max) / 2.0
                )
            )

            camera_front_bb[
                dataset_name
            ] = {
                "upper_left_corner": [
                    x_max,
                    y_max,
                ],
                "bottom_right_corner": [
                    x_min,
                    y_min,
                ],
                "center": [
                    center_x,
                    center_y,
                ],
            }

        return {
            "camera_front": camera_front_bb
        }

    def _get_seedo_dataset_status(
        self,
        primitive_name,
        is_final_action=False,
    ):
        """
        Map a SeeDo primitive to the trajectory status used
        by the reference robot dataset.
        """

        if is_final_action:
            return "end"

        status_map = {
            "reach": "start",
            "approaching": "approaching",
            "pick": "picking",
            "lift_up": "picking",
            "moving": "moving",
            "placing": "placing",
        }

        primitive_name = str(
            primitive_name
        ).strip().lower()

        if primitive_name not in status_map:
            raise RuntimeError(
                "Unsupported SeeDo primitive for dataset status: "
                f"{primitive_name!r}"
            )

        return status_map[
            primitive_name
        ]

    def _seedo_camera_info_callback(
        self,
        msg: CameraInfo,
    ):
        self.seedo_camera_info_msg = msg

    def _get_seedo_artifacts_dir(self):
        if not self.seedo_artifacts_dir.strip():
            return None

        return self.seedo_artifacts_dir

    def _wait_for_seedo_runtime_data(
        self,
        timeout: float = 10.0,
    ) -> None:
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            rgbd_ready = self.seedo_rgbd_event.is_set()
            camera_info_ready = self.seedo_camera_info_msg is not None

            if rgbd_ready and camera_info_ready:
                return

            time.sleep(0.01)

        if not self.seedo_rgbd_event.is_set():
            raise TimeoutError(
                "Timed out waiting for synchronized SeeDo RGB-D data."
            )

        raise TimeoutError(
            "Timed out waiting for SeeDo CameraInfo data."
        )

    def _get_seedo_runtime_input(
        self,
        base_to_table_transform,
    ):
        if self.seedo_rgb_msg is None:
            raise RuntimeError(
                "No synchronized SeeDo RGB frame is available."
            )

        if self.seedo_depth_msg is None:
            raise RuntimeError(
                "No synchronized SeeDo depth frame is available."
            )

        if self.seedo_camera_info_msg is None:
            raise RuntimeError(
                "No SeeDo CameraInfo message is available."
            )

        rgb_image = self.bridge.imgmsg_to_cv2(
            self.seedo_rgb_msg,
            desired_encoding="rgb8",
        )

        depth_image = self.bridge.imgmsg_to_cv2(
            self.seedo_depth_msg,
            desired_encoding="passthrough",
        )

        camera_info_msg = self.seedo_camera_info_msg

        camera_info = {
            "height": camera_info_msg.height,
            "width": camera_info_msg.width,
            "distortion_model": camera_info_msg.distortion_model,
            "d": list(camera_info_msg.d),
            "k": list(camera_info_msg.k),
            "r": list(camera_info_msg.r),
            "p": list(camera_info_msg.p),
        }

        return self._build_seedo_runtime_input(
            rgb_image=rgb_image,
            depth_image=depth_image,
            camera_info=camera_info,
            base_to_table_transform=base_to_table_transform,
        )

    def _build_seedo_runtime_input(
        self,
        rgb_image,
        depth_image,
        camera_info,
        base_to_table_transform,
    ):
        if rgb_image is None:
            raise ValueError(
                "SeeDo runtime input is missing the RGB image."
            )

        if depth_image is None:
            raise ValueError(
                "SeeDo runtime input is missing the depth image."
            )

        if camera_info is None:
            raise ValueError(
                "SeeDo runtime input is missing camera_info."
            )

        if base_to_table_transform is None:
            raise ValueError(
                "SeeDo runtime input is missing "
                "base_to_table_transform."
            )

        return {
            "rgb": rgb_image,
            "depth": depth_image,
            "camera_info": camera_info,
            "base_to_table_transform": base_to_table_transform,
        }

    def synced_images_callback(self, *image_msgs):
        """Called once per cycle when all camera topics have a message within the sync window."""
        self.latest_synced_images = [
            self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8') for msg in image_msgs
        ]
        self.synced_images_event.set()

    def get_synced_images(self, timeout_sec=5.0):
        """Wait for a fresh synchronized set of camera images."""
        if self.debug_mode:
            return self._get_debug_images()

        self.synced_images_event.clear()

        if self.ai_controller_target == 'seedo_controller':
            if self.seedo_rgb_msg is None:
                if not self.seedo_rgbd_event.wait(timeout=timeout_sec):
                    self.get_logger().error(
                        'Timed out waiting for SeeDo RGB image.'
                    )
                    return None

            front_image = self.bridge.imgmsg_to_cv2(
                self.seedo_rgb_msg,
                desired_encoding='rgb8',
            )

            return [front_image]

        start = self.get_clock().now()

        while not self.synced_images_event.is_set():
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

            elapsed = (
                self.get_clock().now() - start
            ).nanoseconds / 1e9

            if elapsed > timeout_sec:
                self.get_logger().error(
                    'Timed out waiting for synchronized camera images.'
                )
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
        self.joint_state_event.set()

    def _quat2axisangle(self, quat):
        """
        Convert quaternion [x, y, z, w] to axis-angle representation.
        """

        quat = np.asarray(
            quat,
            dtype=np.float64,
        ).copy()

        quat[3] = np.clip(
            quat[3],
            -1.0,
            1.0,
        )

        den = np.sqrt(
            1.0 - quat[3] * quat[3]
        )

        if math.isclose(
            den,
            0.0,
            abs_tol=1e-8,
        ):
            return np.zeros(
                3,
                dtype=np.float64,
            )

        return (
            quat[:3]
            * 2.0
            * math.acos(quat[3])
            / den
        )

    def _gripper_joint_position_to_raw(
        self,
        joint_position,
    ):
        """
        Convert the Robotiq joint position in radians to the
        raw representation used by the reference dataset.
        """

        joint_position = float(
            np.asarray(
                joint_position
            ).reshape(-1)[0]
        )

        raw_open = 3
        raw_closed = 230

        joint_open = 0.0
        joint_closed = 0.8

        ratio = (
            (joint_position - joint_open)
            / (joint_closed - joint_open)
        )

        ratio = np.clip(
            ratio,
            0.0,
            1.0,
        )

        raw_position = (
            raw_open
            + ratio * (
                raw_closed - raw_open
            )
        )

        return int(
            np.rint(
                raw_position
            )
        )

    def _capture_robot_state(self, timeout_sec=1.0):
        """Spin briefly to receive a fresh /joint_states + TF, then build a robot-state obs dict.

        Field names mirror dataset_collector_pkg/utils.py so rollouts saved here are
        readable by the same tooling (replicate_trajectory.py, cod_controller.py) as
        recorded demonstrations.
        """
        if self.ai_controller_target == 'seedo_controller':
            self.joint_state_event.clear()

            if not self.joint_state_event.wait(
                timeout=timeout_sec
            ):
                self.get_logger().warning(
                    'Timed out waiting for a fresh /joint_states message.'
                )

        else:
            start = self.get_clock().now()

            while self.latest_joint_state is None:
                rclpy.spin_once(
                    self,
                    timeout_sec=0.1,
                )

                if (
                    self.get_clock().now() - start
                ).nanoseconds / 1e9 > timeout_sec:
                    self.get_logger().warning(
                        'Timed out waiting for /joint_states message.'
                    )
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
            if self.ai_controller_target == 'seedo_controller':
                trans = self._lookup_transform(
                    self.frame_id,
                    self.eef_frame_name,
                )
            else:
                trans = self.tf_buffer.lookup_transform(
                    self.frame_id,
                    self.eef_frame_name,
                    rclpy.time.Time(),
                )
            state[EEF_POS_NAME] = np.array([trans.transform.translation.x,
                                            trans.transform.translation.y,
                                            trans.transform.translation.z])
            state[EEF_QUAT_NAME] = np.array([trans.transform.rotation.x,
                                             trans.transform.rotation.y,
                                             trans.transform.rotation.z,
                                             trans.transform.rotation.w])
            state["ee_aa"] = self._quat2axisangle(
                state[EEF_QUAT_NAME]
            )
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

        if self.ai_controller_target == 'seedo_controller':
            response = self._wait_for_future(
                future,
                timeout_sec=30.0,
            )
        else:
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
        if (
            self.ai_controller_target != 'seedo_controller'
            or self.seedo_execute_gripper
        ):
            # Open the gripper to a known position before starting
            self.get_logger().info(
                'Opening gripper to a known position '
                'before first inference...'
            )

            gripper_goal = GripperCommand.Goal()
            gripper_goal.command.position = 0.1
            gripper_goal.command.max_effort = 50.0

            future = self.gripper_action_client.send_goal_async(
                gripper_goal
            )

            if self.ai_controller_target == 'seedo_controller':
                goal_handle = self._wait_for_future(
                    future,
                    timeout_sec=10.0,
                )

                if goal_handle is None:
                    raise RuntimeError(
                        'Timed out sending initial gripper command.'
                    )

                if not goal_handle.accepted:
                    raise RuntimeError(
                        'Initial gripper goal was rejected.'
                    )

                result_future = goal_handle.get_result_async()

                gripper_result = self._wait_for_future(
                    result_future,
                    timeout_sec=10.0,
                )

                if gripper_result is None:
                    raise RuntimeError(
                        'Timed out waiting for initial gripper result.'
                    )

                self.get_logger().info(
                    'Gripper opened successfully before first inference.'
                )

                self.previous_gripper_position = (
                    gripper_goal.command.position
                )

            else:
                rclpy.spin_until_future_complete(
                    self,
                    future,
                )

                if future.result() is not None:
                    self.get_logger().info(
                        'Gripper opened successfully before first inference.'
                    )
                else:
                    self.get_logger().error(
                        f'Failed to open gripper before first inference: '
                        f'{future.exception()}'
                    )

                    raise RuntimeError(
                        f'Failed to open gripper before first inference: '
                        f'{future.exception()}'
                    )

        else:
            self.get_logger().info(
                'Skipping initial gripper command: '
                'SeeDo gripper execution is disabled.'
            )
    
    def save_rollout_failure(
        self,
        exception,
    ):
        """
        Persist information about a failed rollout even when the normal
        save_rollout() path is never reached.
        """

        if self.current_task_id is None:
            self.get_logger().error(
                "Cannot save rollout failure: current task ID is unknown."
            )
            return

        complete_save_path = os.path.join(
            self.save_rollout_path,
            f"task_{self.current_task_id}",
        )

        os.makedirs(
            complete_save_path,
            exist_ok=True,
        )

        traj_name = "traj_{:03d}".format(
            self.traj_cnt
        )

        json_path = os.path.join(
            complete_save_path,
            traj_name + ".json",
        )

        res_dict = {
            "program_status": "failed",

            "task_id": self.current_task_id,
            "traj_number": self.traj_cnt,

            "failure_stage": self.current_failure_stage,
            "failed_step": self.current_failure_step,
            "failed_action_index": self.current_failure_action_index,

            "error_type": type(exception).__name__,
            "error_message": str(exception),

            "controller_execution_status": getattr(
                self.controller,
                "execution_status",
                None,
            ),

            "controller_execution_error": getattr(
                self.controller,
                "execution_error",
                None,
            ),

            "traceback": traceback.format_exc(),
        }

        with open(
            json_path,
            "w",
            encoding="utf-8",
        ) as f:
            json.dump(
                res_dict,
                f,
                indent=4,
            )

        self.get_logger().error(
            f"Failure metadata saved to: {json_path}"
        )

    def save_rollout(self, traj=None, save_path=None, task_id=None, traj_number=None):
        """Save the current rollout Trajectory to a .pkl file, plus outcome metadata to a .json file."""
        complete_save_path = os.path.join(save_path, f'task_{task_id}')
        os.makedirs(complete_save_path,
                    exist_ok=True)

        traj_name = 'traj_{:03d}'.format(traj_number)

        if traj is not None:
            trajectory_path = os.path.join(
                complete_save_path,
                traj_name + '.pkl',
            )

            if self.ai_controller_target == 'seedo_controller':
                traj.save(
                    trajectory_path,
                    len=len(traj),
                    env_type=self.task_name,
                    task_id=task_id,
                )
            else:
                traj.save(
                    trajectory_path,
                    task_id=task_id,
                    traj_number=traj_number,
                    ai_controller_target=self.ai_controller_target,
                    task_name=self.task_name,
                )

            self.get_logger().info(
                f'Saved rollout trajectory to {trajectory_path}'
            )

        res_dict = dict()

        res_dict["program_status"] = "completed"
        res_dict["failure_stage"] = None
        res_dict["error_type"] = None
        res_dict["error_message"] = None
        res_dict["failed_step"] = None
        res_dict["failed_action_index"] = None

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

            self.current_task_id = enter_task_id

            self.current_failure_stage = "initialization"
            self.current_failure_step = None
            self.current_failure_action_index = None
            
            # create a new trajectory
            traj = Trajectory()

            # SeeDo runtime state
            seedo_runtime_input = None
            seedo_primitive_count = None
            seedo_obj_bb = None

            # Timing state for the current episode
            end_to_end_start = None
            robot_execution_start = None
            
            for step in range(self.max_step):

                self.current_failure_step = step
                self.current_failure_action_index = None

                if step == 0:
                    # resetting controller state for the new task
                    self.controller.reset()

                    if self.ai_controller_target == 'seedo_controller':

                        if self.move_robot:
                            self.get_logger().info(
                                f'Setting robot to home position for task ID: '
                                f'{enter_task_id}'
                            )

                            future = self.set_home_client.call_async(
                                GoHome.Request()
                            )

                            response = self._wait_for_future(
                                future,
                                timeout_sec=60.0,
                            )

                            if response is None:
                                raise RuntimeError(
                                    'Timed out waiting for GoHome service.'
                                )

                            if not response.success:
                                raise RuntimeError(
                                    f'Failed to set robot to home position: '
                                    f'{response.message}'
                                )

                            self.get_logger().info(
                                response.message
                            )

                            self.move_to_initial_pose()


                        # --------------------------------------------------
                        # START END-TO-END TIMING
                        # --------------------------------------------------

                        TIMING.reset()

                        end_to_end_start = TIMING.start()

                        self.get_logger().info(
                            '[TIMING] End-to-end timing started.'
                        )


                        self.get_logger().info(
                            f'Loading SeeDo demo data for task ID: {enter_task_id}'
                        )

                        self.pause_executor.set()

                        try:

                            self.current_failure_stage = "demonstration_understanding"

                            self.controller.load_command(
                                demo_path=self.demo_path,
                                task_id=enter_task_id,
                                artifacts_dir=self._get_seedo_artifacts_dir(),
                                precomputed_action_plan_path=(
                                    self.seedo_precomputed_action_plan_path
                                ),
                            )
                        finally:
                            self.pause_executor.clear()

                        self.get_logger().info(
                            'Waiting for SeeDo runtime RGB-D data...'
                        )

                        self._wait_for_seedo_runtime_data()

                        base_to_table_transform = (
                            self._get_seedo_base_to_table_transform()
                        )

                        seedo_runtime_input = self._get_seedo_runtime_input(
                            base_to_table_transform
                        )

                        self.get_logger().info(
                            'Running SeeDo runtime perception and planning...'
                        )

                        self.current_failure_stage = "scene_perception_and_planning"

                        result = self.controller.inference(
                            input_data=seedo_runtime_input,
                            t=0,
                        )

                        if result is not None:
                            raise RuntimeError(
                                'SeeDo inference(t=0) must return None.'
                            )

                        seedo_obj_bb = (
                            self._build_seedo_front_obj_bb()
                        )

                        self.get_logger().info(
                            "SeeDo runtime bounding boxes prepared "
                            f"for {len(seedo_obj_bb['camera_front'])} objects."
                        )

                        self.current_failure_stage = "lmp_generation"

                        if self.controller.primitive_plan is None:
                            raise RuntimeError(
                                'SeeDo did not generate a PrimitivePlan.'
                            )

                        seedo_primitive_count = len(
                            self.controller.primitive_plan.steps
                        )

                        if seedo_primitive_count == 0:
                            raise RuntimeError(
                                'SeeDo generated an empty PrimitivePlan.'
                            )

                        self.get_logger().info(
                            f'SeeDo generated {seedo_primitive_count} primitives.'
                        )

                        continue

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

                if self.ai_controller_target == 'seedo_controller':
                    self.current_failure_stage = "motion_generation"
                    out = self.controller.inference(
                        input_data={
                            "robot_state": robot_state,
                        },
                        t=step,
                    )
                else:
                    out = self.controller.inference(
                                                    input_data=[images, states],
                                                    t=step,
                                                    save_path=step_save_path)
                
                predicted_bb = None
                target_obj_prediction = None

                if self.ai_controller_target == 'seedo_controller':
                    if out is None:
                        raise RuntimeError(
                            f'SeeDo returned None before completing the '
                            f'PrimitivePlan at t={step}.'
                        )

                    actions = out

                    current_seedo_primitive = (
                        self.controller
                        .primitive_plan
                        .steps[step - 1]
                        .name
                    )
                elif self.ai_controller_target == 'cod_controller':
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


                primitive_execution_start = None

                if (
                    self.ai_controller_target == 'seedo_controller'
                    and self.move_robot
                ):
                    # Start the global robot execution timer only once,
                    # immediately before the first physical action.
                    if robot_execution_start is None:
                        robot_execution_start = TIMING.start()

                        self.get_logger().info(
                            '[TIMING] Robot execution timing started.'
                        )

                    # Start timing the current primitive.
                    primitive_execution_start = TIMING.start()

                for indx, action in enumerate(actions):
                    self.current_failure_action_index = indx
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

                        self.current_failure_stage = "robot_execution"
                        future = self.set_pose_client.call_async(
                            pose_request
                        )

                        if self.ai_controller_target == 'seedo_controller':
                            response = self._wait_for_future(
                                future,
                                timeout_sec=30.0,
                            )

                            if response is None:
                                raise RuntimeError(
                                    f'Timed out waiting for GoToPose '
                                    f'at step {step}, action {indx}.'
                                )

                            if not response.success:
                                raise RuntimeError(
                                    f'GoToPose failed at step {step}, '
                                    f'action {indx}: '
                                    f'{response.message}'
                                )

                            self.get_logger().info(
                                f'Robot set to desired pose '
                                f'at step {step}, action {indx}'
                            )

                            # Allow Isaac/ros2_control state feedback to settle
                            # before planning the next SeeDo micro-action.
                            time.sleep(0.5)

                        else:
                            rclpy.spin_until_future_complete(
                                self,
                                future,
                            )

                            if future.result() is not None:
                                self.get_logger().info(
                                    f'Robot set to desired pose at step {step}'
                                )
                            else:
                                self.get_logger().error(
                                    'Service call failed for setting robot '
                                    f'to desired pose: {future.exception()}'
                                )

                                raise RuntimeError(
                                    'Service call failed for setting robot '
                                    f'to desired pose: {future.exception()}'
                                )

                        
                        # 6. Control the gripper based on the predicted action
                        if (
                            self.ai_controller_target != 'seedo_controller'
                            or self.seedo_execute_gripper
                        ):
                            self.get_logger().info(
                                f'Controlling gripper at step {step}'
                            )

                            desired_gripper_position = float(action[-1])

                            # SeeDo sends a gripper command only when
                            # the desired position actually changes.
                            if self.ai_controller_target == 'seedo_controller':
                                gripper_changed = not np.isclose(
                                    desired_gripper_position,
                                    self.previous_gripper_position,
                                    atol=1e-6,
                                )
                            else:
                                gripper_changed = True

                            if gripper_changed:
                                gripper_goal = GripperCommand.Goal()

                                gripper_goal.command.position = (
                                    desired_gripper_position
                                )

                                gripper_goal.command.max_effort = 50.0

                                self.current_failure_stage = "gripper_execution"
                                future = (
                                    self.gripper_action_client
                                    .send_goal_async(gripper_goal)
                                )

                                if self.ai_controller_target == 'seedo_controller':
                                    goal_handle = self._wait_for_future(
                                        future,
                                        timeout_sec=10.0,
                                    )

                                    if goal_handle is None:
                                        raise RuntimeError(
                                            f'Timed out sending gripper command '
                                            f'at step {step}, action {indx}.'
                                        )

                                    if not goal_handle.accepted:
                                        raise RuntimeError(
                                            f'Gripper goal rejected at step {step}, '
                                            f'action {indx}.'
                                        )

                                    result_future = (
                                        goal_handle.get_result_async()
                                    )

                                    gripper_result = self._wait_for_future(
                                        result_future,
                                        timeout_sec=10.0,
                                    )

                                    if gripper_result is None:
                                        raise RuntimeError(
                                            f'Timed out waiting for gripper result '
                                            f'at step {step}, action {indx}.'
                                        )

                                    self.get_logger().info(
                                        f'Gripper command completed '
                                        f'at step {step}, action {indx}'
                                    )

                                    self.previous_gripper_position = (
                                        desired_gripper_position
                                    )

                                    # Allow the physical/simulated gripper state feedback
                                    # to settle before recording the rollout observation.
                                    time.sleep(0.5)

                                else:
                                    rclpy.spin_until_future_complete(
                                        self,
                                        future,
                                    )

                                    if future.result() is not None:
                                        self.get_logger().info(
                                            f'Gripper command sent at step {step}'
                                        )
                                    else:
                                        self.get_logger().error(
                                            f'Failed to send gripper command: '
                                            f'{future.exception()}'
                                        )

                                        raise RuntimeError(
                                            f'Failed to send gripper command: '
                                            f'{future.exception()}'
                                        )

                                self.get_logger().info(
                                    f'Gripper command position: '
                                    f'{gripper_goal.command.position}'
                                )

                                # Legacy gripper-state tracking only.
                                if (
                                    self.ai_controller_target != 'seedo_controller'
                                    and not self.gripper_closed
                                    and gripper_goal.command.position == 255.0
                                ):
                                    self.get_logger().info(
                                        f'Gripper is closing at step {step}'
                                    )
                                    self.gripper_closed = True

                            elif self.ai_controller_target == 'seedo_controller':
                                self.get_logger().debug(
                                    f'Gripper already at '
                                    f'{desired_gripper_position}; '
                                    f'skipping command.'
                                )

                        else:
                            self.get_logger().debug(
                                'Skipping SeeDo gripper command: '
                                'gripper execution disabled.'
                            )

                    if self.ai_controller_target == 'seedo_controller':
                        seedo_action_done = (
                            seedo_primitive_count is not None
                            and step == seedo_primitive_count
                            and indx == len(actions) - 1
                        )

                        seedo_status = (
                            self._get_seedo_dataset_status(
                                primitive_name=(
                                    current_seedo_primitive
                                ),
                                is_final_action=(
                                    seedo_action_done
                                ),
                            )
                        )

                        if self.move_robot:
                            rollout_robot_state = (
                                self._capture_robot_state()
                            )
                        else:
                            rollout_robot_state = robot_state

                        seedo_step_obs = dict(
                            rollout_robot_state
                        )

                        seedo_step_obs.pop(
                            GRIPPER_QVEL_NAME,
                            None,
                        )

                        if GRIPPER_QPOS_NAME in seedo_step_obs:
                            seedo_step_obs[
                                GRIPPER_QPOS_NAME
                            ] = self._gripper_joint_position_to_raw(
                                seedo_step_obs[
                                    GRIPPER_QPOS_NAME
                                ]
                            )

                        camera_data = (
                            self._get_seedo_record_camera_data()
                        )

                        camera_data = (
                            self._compress_seedo_dataset_rgb(
                                camera_data
                            )
                        )

                        seedo_step_obs.update(
                            camera_data
                        )

                        if seedo_obj_bb is None:
                            raise RuntimeError(
                                "SeeDo runtime bounding boxes are not available."
                            )

                        seedo_step_obs[
                            "obj_bb"
                        ] = seedo_obj_bb

                        if (
                            EEF_POS_NAME not in rollout_robot_state
                            or EEF_QUAT_NAME not in rollout_robot_state
                        ):
                            raise RuntimeError(
                                "Missing EEF pose while building SeeDo dataset action."
                            )

                        seedo_dataset_action = np.zeros(
                            8,
                            dtype=np.float64,
                        )

                        seedo_dataset_action[0:3] = np.asarray(
                            rollout_robot_state[EEF_POS_NAME],
                            dtype=np.float64,
                        )

                        seedo_dataset_action[3:7] = np.asarray(
                            rollout_robot_state[EEF_QUAT_NAME],
                            dtype=np.float64,
                        )

                        seedo_dataset_action[7] = (
                            1.0
                            if float(action[7]) > 0.5
                            else 0.0
                        )

                        traj.append(
                            obs=seedo_step_obs,
                            action=seedo_dataset_action,
                            done=seedo_action_done,
                            reward=1 if seedo_action_done else 0,
                            info={
                                "status": f'"{seedo_status}"',
                            },
                        )

                if primitive_execution_start is not None:
                    TIMING.stop(
                        f'robot.primitive_{step:02d}',
                        primitive_execution_start,
                    )

                # check if a transiction close->open has been made
                episode_done = False

                if self.ai_controller_target == 'seedo_controller':
                    episode_done = (
                        seedo_primitive_count is not None
                        and step == seedo_primitive_count
                    )
                else:
                    if self.gripper_closed and gripper_goal.command.position == 0.0:
                        self.get_logger().info(f'Gripper is opening at step {step}')
                        self.gripper_closed = False
                        episode_done = True

                if self.ai_controller_target != 'seedo_controller':
                    # 7. Record this step (observation image, cropped model input, predicted
                    # bounding boxes, computed action and robot state) into the rollout Trajectory
                    step_obs = dict(robot_state)
                    step_obs['camera_front_image'] = cv2.cvtColor(images[0], cv2.COLOR_RGB2BGR)

                    if self.ai_controller_target != 'seedo_controller':
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

                if (
                    self.ai_controller_target == 'seedo_controller'
                    and episode_done
                ):
                    completion_result = self.controller.inference(
                        input_data={},
                        t=step + 1,
                    )

                    if completion_result is not None:
                        raise RuntimeError(
                            'SeeDo must return None after the '
                            'PrimitivePlan has been consumed.'
                        )

                    if self.controller.execution_status != 'completed':
                        raise RuntimeError(
                            'SeeDo did not enter completed state. '
                            f'Current status: '
                            f'{self.controller.execution_status}'
                        )

                    self.get_logger().info(
                        'SeeDo PrimitivePlan consumed successfully.'
                    )

                    # --------------------------------------------------
                    # FINAL TIMINGS
                    # --------------------------------------------------

                    if robot_execution_start is not None:
                        TIMING.stop(
                            'robot_execution',
                            robot_execution_start,
                        )
                        robot_execution_start = None


                    if end_to_end_start is not None:
                        TIMING.stop(
                            'end_to_end_wall',
                            end_to_end_start,
                        )
                        end_to_end_start = None

                    if self.controller.artifacts_dir:
                        timing_path = (
                            self.controller.artifacts_dir
                            / 'timings.json'
                        )

                        TIMING.save(
                            timing_path
                        )

                        self.get_logger().info(
                            f'[TIMING] Results saved to: {timing_path}'
                        )
                    else:
                        self.get_logger().warning(
                            '[TIMING] No artifacts_dir set; '
                            'timing results not saved.'
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
                    
        
def spin_executor(node=None, executor=None):
    while rclpy.ok():
        if node.pause_executor.is_set():
            time.sleep(0.01)
            continue

        executor.spin_once(timeout_sec=0.01)

def main(args=None):
    rclpy.init()

    node = AIControllerNode()

    if node.ai_controller_target == 'seedo_controller':

        executor = MultiThreadedExecutor(num_threads=1)
        executor.add_node(node)

        # executor_thread = threading.Thread(
        #     target=executor.spin,
        #     daemon=True,
        # )

        executor_thread = threading.Thread(
            target=spin_executor, args=(node, executor),
            daemon=True,
        )

        executor_thread.start()

        try:
            node.get_logger().info(
                'Beginning SeeDo client, shut down with CTRL-C'
            )

            node.control_loop()

        except KeyboardInterrupt:
            node.get_logger().info(
                'Keyboard interrupt, shutting down.\n'
            )
        
        except Exception as exc:
            node.get_logger().error(
                f"SeeDo rollout failed: "
                f"{type(exc).__name__}: {exc}"
            )

            try:
                node.save_rollout_failure(
                    exception=exc,
                )
            except Exception as save_exc:
                node.get_logger().error(
                    "Failed to persist rollout failure metadata: "
                    f"{save_exc}"
                )

            raise

        finally:
            executor.shutdown()

            if executor_thread.is_alive():
                executor_thread.join(timeout=2.0)

            node.destroy_node()

            if rclpy.ok():
                rclpy.shutdown()

    else:
        try:
            node.get_logger().info(
                'Beginning client, shut down with CTRL-C'
            )

            node.control_loop()

            node.get_logger().info(
                'Shutting down AIControllerNode\n'
            )

        except KeyboardInterrupt:
            node.get_logger().info(
                'Keyboard interrupt, shutting down.\n'
            )

        finally:
            node.destroy_node()

            if rclpy.ok():
                rclpy.shutdown()