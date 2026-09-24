import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
import tf2_ros

from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from moveit_controller_srvs.srv import GoHome
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, Joy
from std_srvs.srv import Trigger
from ur5e_2f_85_teleoperation_msg.msg import TrajectoryState

from dataset_collector_pkg.utils import (
    EE_AA_NAME,
    EEF_POS_NAME,
    EEF_QUAT_NAME,
    GRIPPER_QPOS_NAME,
    GRIPPER_QVEL_NAME,
    JOINT_POS_NAME,
    JOINT_VEL_NAME,
    get_object_centers,
)

from scripts.savers import Trajectory


class DatasetCollector(Node):

    MODE_HUMAN = 'human'
    MODE_ROBOT = 'robot'

    def __init__(self):
        super().__init__('dataset_collector')

        self.get_logger().info(
            'Dataset Collector Node has been started.'
        )

        self.cv_bridge = CvBridge()

        # ============================================================
        # Runtime state
        # ============================================================

        self.current_t = 0
        self.is_recording = False
        self.is_moving_home = False

        self.trajectory_state = (
            TrajectoryState.TRAJECTORY_IDLE
        )

        self._trajectory = None
        self._active_traj_id = None

        self._video_writer = None
        self._video_path = None

        # Human recording button edge detection.
        self._previous_human_record_button_state = 0

        # ============================================================
        # Parameters
        # ============================================================

        self._declare_parameters()
        self._read_parameters()
        self._validate_configuration()
        self._configure_camera_names()

        # ============================================================
        # Debug windows
        # ============================================================

        if self.show_images:
            self._create_debug_windows()

        # ============================================================
        # Cameras
        # ============================================================

        self._create_camera_subscribers()

        # ============================================================
        # Mode-specific configuration
        # ============================================================

        if self.is_human_demo:
            self._configure_human_mode()

        else:
            self._configure_robot_mode()

        # ============================================================
        # Synchronization
        # ============================================================

        self._create_time_synchronizer()

        # ============================================================
        # Final configuration log
        # ============================================================

        self._log_configuration()

    # ================================================================
    # PARAMETERS
    # ================================================================

    def _declare_parameters(self):

        # ------------------------------------------------------------
        # Collector mode
        # ------------------------------------------------------------

        self.declare_parameter(
            'collector_mode',
            self.MODE_ROBOT,
        )

        # ------------------------------------------------------------
        # Cameras
        # ------------------------------------------------------------

        # Robot demonstrations keep the original four-camera setup.
        self.declare_parameter(
            'camera_names',
            [
                '/zed_front/zed_node',
                '/zed_left/zed_node',
                '/zed_right/zed_node',
                '/zed_gripper/zed_node',
            ],
        )

        # Human demonstrations use only the three external cameras.
        self.declare_parameter(
            'human_camera_names',
            [
                '/zed_front/zed_node',
                '/zed_left/zed_node',
                '/zed_right/zed_node',
            ],
        )

        # Topic exposed by the ZED setup currently used in this project.
        self.declare_parameter(
            'rgb_topic_suffix',
            '/rgb/color/rect/image',
        )

        self.declare_parameter(
            'depth_topic_suffix',
            '/depth/depth_registered',
        )

        # Robot demonstrations preserve the previous depth behavior.
        self.declare_parameter(
            'record_depth',
            True,
        )

        # Human samples mirror the reference dataset: RGB frames only.
        self.declare_parameter(
            'human_record_depth',
            False,
        )

        self.declare_parameter(
            'show_images',
            False,
        )

        # Human recording controls. Circle is button index 1 on the
        # joystick mapping already used by teleoperator_node.py.
        self.declare_parameter(
            'enable_human_joystick',
            True,
        )

        self.declare_parameter(
            'human_joy_topic',
            '/joy',
        )

        self.declare_parameter(
            'human_record_button_index',
            1,
        )

        # ------------------------------------------------------------
        # Synchronization
        # ------------------------------------------------------------

        self.declare_parameter(
            'sync_queue_size',
            10,
        )

        self.declare_parameter(
            'sync_slop_sec',
            0.1,
        )

        # ------------------------------------------------------------
        # Robot joints
        # ------------------------------------------------------------

        self.declare_parameter(
            'joint_robot_names',
            [
                'elbow_joint',
                'shoulder_lift_joint',
                'shoulder_pan_joint',
                'wrist_1_joint',
                'wrist_2_joint',
                'wrist_3_joint',
            ],
        )

        self.declare_parameter(
            'gripper_robot_names',
            [
                'robotiq_85_left_knuckle_joint',
            ],
        )

        # ------------------------------------------------------------
        # Robot frames
        # ------------------------------------------------------------

        self.declare_parameter(
            'eef_frame_name',
            'tcp_link',
        )

        self.declare_parameter(
            'base_frame_name',
            'base_link',
        )

        # ------------------------------------------------------------
        # Robot topics
        # ------------------------------------------------------------

        self.declare_parameter(
            'ur_topics_to_record',
            [
                '/joint_states',
            ],
        )

        self.declare_parameter(
            'teleop_state_topic',
            '/teleop_trajectory_state',
        )

        # ------------------------------------------------------------
        # Robot home
        # ------------------------------------------------------------

        self.declare_parameter(
            'set_robot_to_home_service',
            'set_robot_to_home',
        )

        self.declare_parameter(
            'move_home_on_start',
            True,
        )

        self.declare_parameter(
            'move_home_after_trajectory',
            True,
        )

        # ------------------------------------------------------------
        # Dataset metadata
        # ------------------------------------------------------------

        self.declare_parameter(
            'task_name',
            'pick_place',
        )

        self.declare_parameter(
            'variation_id',
            0,
        )

        self.declare_parameter(
            'traj_count_id',
            0,
        )

        self.declare_parameter(
            'saving_directory',
            '/home/saved_trajectories',
        )

        # ------------------------------------------------------------
        # Optional object annotation
        # ------------------------------------------------------------

        self.declare_parameter(
            'annotate_objects',
            False,
        )

        # ------------------------------------------------------------
        # Optional MP4 output
        # ------------------------------------------------------------

        self.declare_parameter(
            'save_video',
            False,
        )

        self.declare_parameter(
            'video_camera',
            'front_camera',
        )

        self.declare_parameter(
            'video_fps',
            30.0,
        )

        self.declare_parameter(
            'video_codec',
            'avc1',
        )

    def _read_parameters(self):

        # ------------------------------------------------------------
        # Mode
        # ------------------------------------------------------------

        self.collector_mode = str(
            self.get_parameter(
                'collector_mode'
            ).value
        ).lower()

        self.is_human_demo = (
            self.collector_mode
            == self.MODE_HUMAN
        )

        # ------------------------------------------------------------
        # Cameras
        # ------------------------------------------------------------

        self.robot_camera_names = list(
            self.get_parameter(
                'camera_names'
            ).value
        )

        self.human_camera_names = list(
            self.get_parameter(
                'human_camera_names'
            ).value
        )

        self.camera_names = (
            self.human_camera_names
            if self.is_human_demo
            else self.robot_camera_names
        )

        self.rgb_topic_suffix = str(
            self.get_parameter(
                'rgb_topic_suffix'
            ).value
        )

        self.depth_topic_suffix = str(
            self.get_parameter(
                'depth_topic_suffix'
            ).value
        )

        self.robot_record_depth = bool(
            self.get_parameter(
                'record_depth'
            ).value
        )

        self.human_record_depth = bool(
            self.get_parameter(
                'human_record_depth'
            ).value
        )

        self.record_depth = (
            self.human_record_depth
            if self.is_human_demo
            else self.robot_record_depth
        )

        self.show_images = bool(
            self.get_parameter(
                'show_images'
            ).value
        )

        self.enable_human_joystick = bool(
            self.get_parameter(
                'enable_human_joystick'
            ).value
        )

        self.human_joy_topic = str(
            self.get_parameter(
                'human_joy_topic'
            ).value
        )

        self.human_record_button_index = int(
            self.get_parameter(
                'human_record_button_index'
            ).value
        )

        # ------------------------------------------------------------
        # Synchronization
        # ------------------------------------------------------------

        self.sync_queue_size = int(
            self.get_parameter(
                'sync_queue_size'
            ).value
        )

        self.sync_slop_sec = float(
            self.get_parameter(
                'sync_slop_sec'
            ).value
        )

        # ------------------------------------------------------------
        # Robot
        # ------------------------------------------------------------

        self.joint_robot_names = list(
            self.get_parameter(
                'joint_robot_names'
            ).value
        )

        self.gripper_robot_names = list(
            self.get_parameter(
                'gripper_robot_names'
            ).value
        )

        self.eef_frame_name = str(
            self.get_parameter(
                'eef_frame_name'
            ).value
        )

        self.base_frame_name = str(
            self.get_parameter(
                'base_frame_name'
            ).value
        )

        self.ur_topics_to_record = list(
            self.get_parameter(
                'ur_topics_to_record'
            ).value
        )

        self.teleop_state_topic = str(
            self.get_parameter(
                'teleop_state_topic'
            ).value
        )

        self.set_robot_to_home_service_name = str(
            self.get_parameter(
                'set_robot_to_home_service'
            ).value
        )

        self.move_home_on_start = bool(
            self.get_parameter(
                'move_home_on_start'
            ).value
        )

        self.move_home_after_trajectory = bool(
            self.get_parameter(
                'move_home_after_trajectory'
            ).value
        )

        # ------------------------------------------------------------
        # Dataset
        # ------------------------------------------------------------

        self.task_name = str(
            self.get_parameter(
                'task_name'
            ).value
        )

        self.variation_id = int(
            self.get_parameter(
                'variation_id'
            ).value
        )

        self.traj_count_id = int(
            self.get_parameter(
                'traj_count_id'
            ).value
        )

        self.saving_directory = str(
            self.get_parameter(
                'saving_directory'
            ).value
        )

        # ------------------------------------------------------------
        # Annotation
        # ------------------------------------------------------------

        self.annotate_objects = bool(
            self.get_parameter(
                'annotate_objects'
            ).value
        )

        # ------------------------------------------------------------
        # Video
        # ------------------------------------------------------------

        self.save_video = bool(
            self.get_parameter(
                'save_video'
            ).value
        )

        self.video_camera = str(
            self.get_parameter(
                'video_camera'
            ).value
        )

        if self.is_human_demo:
            self.video_camera = {
                'front_camera': 'camera_front',
                'left_camera': 'camera_left',
                'right_camera': 'camera_right',
            }.get(self.video_camera, self.video_camera)

        self.video_fps = float(
            self.get_parameter(
                'video_fps'
            ).value
        )

        self.video_codec = str(
            self.get_parameter(
                'video_codec'
            ).value
        )

    def _validate_configuration(self):

        if self.collector_mode not in (
            self.MODE_HUMAN,
            self.MODE_ROBOT,
        ):
            raise ValueError(
                f'Unsupported collector_mode='
                f'{self.collector_mode!r}. '
                f'Expected "human" or "robot".'
            )

        if not self.camera_names:
            raise ValueError(
                'camera_names must contain '
                'at least one camera.'
            )

        if self.sync_queue_size <= 0:
            raise ValueError(
                'sync_queue_size must be > 0.'
            )

        if self.sync_slop_sec < 0.0:
            raise ValueError(
                'sync_slop_sec must be >= 0.'
            )

        if self.human_record_button_index < 0:
            raise ValueError(
                'human_record_button_index must be >= 0.'
            )

        if self.video_fps <= 0.0:
            raise ValueError(
                'video_fps must be > 0.'
            )

        if len(self.video_codec) != 4:
            raise ValueError(
                'video_codec must contain '
                'exactly 4 characters, '
                'e.g. "mp4v".'
            )

    # ================================================================
    # CAMERA CONFIGURATION
    # ================================================================

    def _configure_camera_names(self):

        self.camera_names_obs_name_map = {}

        self.front_camera_name = None

        for index, camera_name in enumerate(
            self.camera_names
        ):

            lower_name = camera_name.lower()

            if 'front' in lower_name:

                obs_name = (
                    'camera_front'
                    if self.is_human_demo
                    else 'front_camera'
                )
                self.front_camera_name = camera_name

            elif 'left' in lower_name:

                obs_name = (
                    'camera_left'
                    if self.is_human_demo
                    else 'left_camera'
                )

            elif 'right' in lower_name:

                obs_name = (
                    'camera_right'
                    if self.is_human_demo
                    else 'right_camera'
                )

            elif 'gripper' in lower_name:

                obs_name = (
                    'camera_gripper'
                    if self.is_human_demo
                    else 'gripper_camera'
                )

            else:

                obs_name = f'camera_{index}'

            if (
                obs_name
                in self.camera_names_obs_name_map.values()
            ):
                raise ValueError(
                    'Multiple cameras map to '
                    f'{obs_name!r}.'
                )

            self.camera_names_obs_name_map[
                camera_name
            ] = obs_name

        if (
            self.annotate_objects
            and self.front_camera_name is None
        ):
            raise ValueError(
                'annotate_objects=True requires '
                'a camera whose name contains '
                '"front".'
            )

        if (
            self.save_video
            and self.video_camera
            not in self.camera_names_obs_name_map.values()
        ):

            available = ', '.join(
                self.camera_names_obs_name_map.values()
            )

            raise ValueError(
                f'video_camera={self.video_camera!r} '
                'is unavailable. '
                f'Available cameras: {available}'
            )

    # ================================================================
    # DEBUG WINDOWS
    # ================================================================

    def _create_debug_windows(self):

        self.get_logger().info(
            'Creating debug OpenCV windows...'
        )

        for camera_name in self.camera_names:

            obs_name = (
                self.camera_names_obs_name_map[
                    camera_name
                ]
            )

            cv2.namedWindow(
                f'RGB {obs_name}',
                cv2.WINDOW_NORMAL,
            )

            if self.record_depth:

                cv2.namedWindow(
                    f'Depth {obs_name}',
                    cv2.WINDOW_NORMAL,
                )

    # ================================================================
    # SUBSCRIBERS
    # ================================================================

    def _create_camera_subscribers(self):

        self.camera_subscribers_rgb = []
        self.camera_subscribers_depth = []

        for camera_name in self.camera_names:

            # --------------------------------------------------------
            # RGB
            # --------------------------------------------------------

            rgb_topic = (
                f'{camera_name}'
                f'{self.rgb_topic_suffix}'
            )

            self.get_logger().info(
                f'Subscribing to RGB topic: '
                f'{rgb_topic}'
            )

            rgb_sub = Subscriber(
                self,
                Image,
                rgb_topic,
            )

            self.camera_subscribers_rgb.append(
                rgb_sub
            )

            # --------------------------------------------------------
            # Depth
            # --------------------------------------------------------

            if self.record_depth:

                depth_topic = (
                    f'{camera_name}'
                    f'{self.depth_topic_suffix}'
                )

                self.get_logger().info(
                    f'Subscribing to depth topic: '
                    f'{depth_topic}'
                )

                depth_sub = Subscriber(
                    self,
                    Image,
                    depth_topic,
                )

                self.camera_subscribers_depth.append(
                    depth_sub
                )

    # ================================================================
    # HUMAN MODE
    # ================================================================

    def _configure_human_mode(self):

        self.get_logger().info(
            'Configuring HUMAN mode.'
        )

        self.get_logger().info(
            'Robot state, TF, MoveIt and '
            'teleoperation are NOT required.'
        )

        # ------------------------------------------------------------
        # Manual START service
        #
        # Resolves to:
        #
        # /dataset_collector/start
        # ------------------------------------------------------------

        self.start_service = self.create_service(
            Trigger,
            '~/start',
            self._start_recording_service_callback,
        )

        # ------------------------------------------------------------
        # Manual STOP service
        #
        # Resolves to:
        #
        # /dataset_collector/stop
        # ------------------------------------------------------------

        self.stop_service = self.create_service(
            Trigger,
            '~/stop',
            self._stop_recording_service_callback,
        )

        # Robot-specific components do not exist
        # in human mode.

        self.ur_topics_record_subscribers = []

        # Human demonstrations do not need the teleoperator node or any
        # robot controller. The collector listens directly to /joy.
        self.teleop_state_subscription = None
        self.human_joy_subscription = None

        if self.enable_human_joystick:
            self.human_joy_subscription = self.create_subscription(
                Joy,
                self.human_joy_topic,
                self._human_joy_callback,
                10,
            )

            self.get_logger().info(
                f'Human recording joystick enabled: topic='
                f'{self.human_joy_topic}, button_index='
                f'{self.human_record_button_index}'
            )

        self.tf_buffer = None
        self.tf_listener = None

        self.go_home_service = None

    # ================================================================
    # ROBOT MODE
    # ================================================================

    def _configure_robot_mode(self):

        self.get_logger().info(
            'Configuring ROBOT mode.'
        )

        # ------------------------------------------------------------
        # Robot state subscribers
        # ------------------------------------------------------------

        self.ur_topics_record_subscribers = []

        for topic in self.ur_topics_to_record:

            if '/joint_states' not in topic:

                raise ValueError(
                    f'Unsupported UR topic '
                    f'to record: {topic}'
                )

            self.get_logger().info(
                f'Subscribing to robot topic: '
                f'{topic}'
            )

            subscriber = Subscriber(
                self,
                JointState,
                topic,
            )

            self.ur_topics_record_subscribers.append(
                subscriber
            )

        # ------------------------------------------------------------
        # Teleoperation trajectory state
        #
        # NOTE:
        # This is intentionally NOT part of the
        # ApproximateTimeSynchronizer.
        # ------------------------------------------------------------

        self.teleop_state_subscription = (
            self.create_subscription(
                TrajectoryState,
                self.teleop_state_topic,
                self._teleop_state_callback,
                10,
            )
        )

        # ------------------------------------------------------------
        # TF
        # ------------------------------------------------------------

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = (
            tf2_ros.TransformListener(
                self.tf_buffer,
                self,
            )
        )

        if not self.wait_for_tf(
            timeout=10.0
        ):

            self.get_logger().warn(
                f'TF '
                f'{self.base_frame_name} -> '
                f'{self.eef_frame_name} '
                f'is not available yet.'
            )

        # ------------------------------------------------------------
        # GoHome service
        # ------------------------------------------------------------

        self.go_home_service_callback_group = (
            ReentrantCallbackGroup()
        )

        self.go_home_service = self.create_client(
            GoHome,
            self.set_robot_to_home_service_name,
            callback_group=(
                self.go_home_service_callback_group
            ),
        )

        if self.move_home_on_start:

            while not (
                self.go_home_service.wait_for_service(
                    timeout_sec=1.0
                )
            ):

                self.get_logger().info(
                    f'Service '
                    f'{self.set_robot_to_home_service_name} '
                    f'not available, waiting...'
                )

            input(
                'Press Enter to set the robot '
                'to home position...'
            )

            self.set_robot_to_home_position()

    # ================================================================
    # TIME SYNCHRONIZER
    # ================================================================

    def _create_time_synchronizer(self):

        # ------------------------------------------------------------
        # Camera RGB
        # ------------------------------------------------------------

        self.list_of_subs = list(
            self.camera_subscribers_rgb
        )

        # ------------------------------------------------------------
        # Optional depth
        # ------------------------------------------------------------

        if self.record_depth:

            self.list_of_subs += (
                self.camera_subscribers_depth
            )

        # ------------------------------------------------------------
        # Robot states only in robot mode
        # ------------------------------------------------------------

        if not self.is_human_demo:

            self.list_of_subs += (
                self.ur_topics_record_subscribers
            )

        # ------------------------------------------------------------
        # Synchronizer
        # ------------------------------------------------------------

        self.ts = ApproximateTimeSynchronizer(
            self.list_of_subs,
            queue_size=self.sync_queue_size,
            slop=self.sync_slop_sec,
        )

        self.ts.registerCallback(
            self.synced_callback
        )

    # ================================================================
    # HUMAN JOYSTICK START / STOP
    # ================================================================

    def _human_joy_callback(
        self,
        msg,
    ):

        if not self.is_human_demo:
            return

        if self.human_record_button_index >= len(msg.buttons):
            self.get_logger().warn(
                f'Joystick message has only {len(msg.buttons)} buttons, '
                f'but human_record_button_index='
                f'{self.human_record_button_index}.'
            )
            return

        button_state = int(
            msg.buttons[self.human_record_button_index]
        )

        # Rising-edge detection: holding Circle does not repeatedly toggle.
        if (
            button_state == 1
            and self._previous_human_record_button_state == 0
        ):

            if not self.is_recording:
                self._start_new_trajectory(
                    TrajectoryState.TRAJECTORY_START
                )

                self.get_logger().info(
                    'Human recording STARTED from joystick.'
                )

            elif self.current_t > 0:
                saved_path = self._finish_trajectory()

                if saved_path is not None:
                    self.get_logger().info(
                        f'Human recording STOPPED from joystick. '
                        f'Saved to {saved_path}'
                    )

            else:
                self.get_logger().warn(
                    'Human recording stopped before any synchronized '
                    'camera frames were received; discarding sample.'
                )
                self._reset_active_trajectory()

        self._previous_human_record_button_state = button_state

    # ================================================================
    # HUMAN START / STOP SERVICES
    # ================================================================

    def _start_recording_service_callback(
        self,
        request,
        response,
    ):

        del request

        if not self.is_human_demo:

            response.success = False

            response.message = (
                'Manual start is available '
                'only in human mode.'
            )

            return response

        if self.is_recording:

            response.success = False

            response.message = (
                'A human demonstration is '
                'already being recorded.'
            )

            return response

        self._start_new_trajectory(
            TrajectoryState.TRAJECTORY_START
        )

        response.success = True

        response.message = (
            'Recording started for trajectory '
            f'{self._active_traj_id:03d}.'
        )

        return response

    def _stop_recording_service_callback(
        self,
        request,
        response,
    ):

        del request

        if not self.is_human_demo:

            response.success = False

            response.message = (
                'Manual stop is available '
                'only in human mode.'
            )

            return response

        if not self.is_recording:

            response.success = False

            response.message = (
                'No human demonstration '
                'is currently being recorded.'
            )

            return response

        # ------------------------------------------------------------
        # No frames received
        # ------------------------------------------------------------

        if self.current_t == 0:

            self._reset_active_trajectory()

            response.success = False

            response.message = (
                'Recording stopped, but no '
                'synchronized frames were received.'
            )

            return response

        # ------------------------------------------------------------
        # Save
        # ------------------------------------------------------------

        saved_path = (
            self._finish_trajectory()
        )

        response.success = (
            saved_path is not None
        )

        if saved_path is not None:

            response.message = (
                'Recording stopped. '
                f'Saved trajectory to '
                f'{saved_path}'
            )

        else:

            response.message = (
                'Recording stopped, but '
                'trajectory saving failed.'
            )

        return response

    # ================================================================
    # ROBOT TRAJECTORY STATE
    # ================================================================

    def _teleop_state_callback(
        self,
        msg,
    ):

        previous_state = (
            self.trajectory_state
        )

        new_state = (
            msg.trajectory_state
        )

        self.trajectory_state = new_state

        if new_state != previous_state:

            self.get_logger().info(
                f'Trajectory state: '
                f'{previous_state} -> '
                f'{new_state}'
            )

        active_states = {
            TrajectoryState.TRAJECTORY_START,
            TrajectoryState.TRAJECTORY_APPROACHING,
            TrajectoryState.TRAJECTORY_PICKING,
            TrajectoryState.TRAJECTORY_MOVING,
            TrajectoryState.TRAJECTORY_PLACING,
        }

        # ------------------------------------------------------------
        # Start
        # ------------------------------------------------------------

        if (
            new_state in active_states
            and not self.is_recording
        ):

            self._start_new_trajectory(
                new_state
            )

        # ------------------------------------------------------------
        # End
        # ------------------------------------------------------------

        if (
            new_state
            == TrajectoryState.TRAJECTORY_END
            and previous_state
            != TrajectoryState.TRAJECTORY_END
            and self.is_recording
        ):

            if self.current_t > 0:

                self._finish_trajectory()

            else:

                self.get_logger().warn(
                    'Trajectory reached END, '
                    'but no synchronized frames '
                    'were recorded.'
                )

                self._reset_active_trajectory()

    # ================================================================
    # SYNCHRONIZED CALLBACK
    # ================================================================

    def synced_callback(
        self,
        *args,
    ):

        # ------------------------------------------------------------
        # Ignore camera stream while no recording
        # is active.
        # ------------------------------------------------------------

        if not self.is_recording:
            return

        if self.is_moving_home:
            return

        num_cameras = len(
            self.camera_names
        )

        offset = 0

        # ------------------------------------------------------------
        # RGB
        # ------------------------------------------------------------

        rgb_messages = args[
            offset:
            offset + num_cameras
        ]

        offset += num_cameras

        # ------------------------------------------------------------
        # Depth
        # ------------------------------------------------------------

        if self.record_depth:

            depth_messages = args[
                offset:
                offset + num_cameras
            ]

            offset += num_cameras

        else:

            depth_messages = []

        # ------------------------------------------------------------
        # Robot state
        # ------------------------------------------------------------

        if not self.is_human_demo:

            ur_messages = args[offset:]

        else:

            ur_messages = []

        # ------------------------------------------------------------
        # Camera observation
        # ------------------------------------------------------------

        obs = (
            self._build_camera_observation(
                rgb_messages=rgb_messages,
                depth_messages=depth_messages,
            )
        )

        # ------------------------------------------------------------
        # Timestamp
        #
        # Keep robot samples unchanged. Human samples intentionally mirror
        # the reference dataset, whose observations contain only camera
        # frames and the per-step `done` flag.
        # ------------------------------------------------------------

        if not self.is_human_demo:

            if rgb_messages:

                stamp = (
                    rgb_messages[0]
                    .header
                    .stamp
                )

                obs['timestamp_ns'] = (
                    int(stamp.sec)
                    * 1_000_000_000
                    + int(stamp.nanosec)
                )

            else:

                obs['timestamp_ns'] = (
                    self.get_clock()
                    .now()
                    .nanoseconds
                )

        # ------------------------------------------------------------
        # Optional manual annotation on first frame
        # ------------------------------------------------------------

        if (
            self.current_t == 0
            and self.annotate_objects
        ):

            self._annotate_first_frame(
                obs
            )

        # ------------------------------------------------------------
        # Debug visualization
        # ------------------------------------------------------------

        if self.show_images:

            self._show_observation(
                obs
            )

        # ------------------------------------------------------------
        # Robot action/state
        # ------------------------------------------------------------

        action = None

        if not self.is_human_demo:

            action = (
                self._add_robot_observation(
                    obs,
                    ur_messages,
                )
            )

        # ------------------------------------------------------------
        # Optional MP4
        # ------------------------------------------------------------

        self._write_video_frame_if_enabled(
            obs
        )

        # ------------------------------------------------------------
        # Save trajectory step
        # ------------------------------------------------------------

        if self.is_human_demo:

            # Match the reference human sample: each step exposes only
            # `obs` and `done` through Trajectory.get(). Images are
            # compressed internally by Trajectory and decompressed on read.
            self._trajectory.append(
                obs=obs,
                done=False,
            )

        else:

            # Robot collection behavior is intentionally preserved.
            self._trajectory.append(

                obs=obs,

                reward=0,

                done=False,

                info=None,

                action=action,

                raw_state={

                    'teleop_state':
                        self.trajectory_state,

                    'trajectory_id':
                        self._active_traj_id,

                    'collector_mode':
                        self.collector_mode,
                },
            )

        self.current_t += 1

        # Avoid spamming one log per camera frame.

        if (
            self.current_t == 1
            or self.current_t % 30 == 0
        ):

            self.get_logger().info(
                f'Recording trajectory '
                f'{self._active_traj_id:03d}: '
                f'{self.current_t} '
                f'synchronized frames.'
            )

    # ================================================================
    # CAMERA OBSERVATION
    # ================================================================

    def _build_camera_observation(
        self,
        rgb_messages,
        depth_messages,
    ):

        obs = {}

        for index, camera_name in enumerate(
            self.camera_names
        ):

            obs_name = (
                self.camera_names_obs_name_map[
                    camera_name
                ]
            )

            # --------------------------------------------------------
            # RGB
            # --------------------------------------------------------

            obs[
                f'{obs_name}_image'
            ] = (
                self.cv_bridge.imgmsg_to_cv2(
                    rgb_messages[index],
                    desired_encoding='bgr8',
                )
            )

            # --------------------------------------------------------
            # Depth
            # --------------------------------------------------------

            if self.record_depth:

                obs[
                    f'{obs_name}_depth'
                ] = (
                    self.cv_bridge.imgmsg_to_cv2(
                        depth_messages[index],
                        desired_encoding='passthrough',
                    )
                )

        return obs

    # ================================================================
    # OPTIONAL FIRST-FRAME ANNOTATION
    # ================================================================

    def _annotate_first_frame(
        self,
        obs,
    ):

        front_obs_name = (
            self.camera_names_obs_name_map[
                self.front_camera_name
            ]
        )

        front_image_key = (
            f'{front_obs_name}_image'
        )

        front_depth_key = (
            f'{front_obs_name}_depth'
        )

        if front_image_key not in obs:

            self.get_logger().warn(
                'Object annotation requested, '
                'but front_camera_image '
                'is unavailable.'
            )

            return

        self.get_logger().info(
            'Annotating object centers '
            'on the first frame...'
        )

        obj_bb = get_object_centers(

            node=self,

            task_name=self.task_name,

            variation_id=(
                self.variation_id
            ),

            camera_name='front_camera',

            rgb_image=(
                obs[front_image_key]
            ),

            depth_image=(
                obs.get(front_depth_key)
            ),
        )

        obs['obj_bb'] = obj_bb

    # ================================================================
    # ROBOT OBSERVATION
    # ================================================================

    def _add_robot_observation(
        self,
        obs,
        ur_messages,
    ):

        self._add_joint_state_observation(
            obs,
            ur_messages,
        )

        self._add_eef_observation(
            obs
        )

        # ------------------------------------------------------------
        # Cannot build action without EEF pose
        # ------------------------------------------------------------

        if (
            EEF_POS_NAME not in obs
            or EEF_QUAT_NAME not in obs
        ):

            self.get_logger().warn(
                'EEF pose unavailable for '
                'this sample; action omitted.'
            )

            return None

        # ------------------------------------------------------------
        # Action:
        #
        # [x, y, z, qx, qy, qz, qw, gripper]
        # ------------------------------------------------------------

        action = np.zeros(
            8,
            dtype=np.float64,
        )

        action[0:3] = np.asarray(
            obs[EEF_POS_NAME],
            dtype=np.float64,
        )

        action[3:7] = np.asarray(
            obs[EEF_QUAT_NAME],
            dtype=np.float64,
        )

        gripper_qpos = obs.get(
            GRIPPER_QPOS_NAME
        )

        if (
            gripper_qpos is not None
            and len(gripper_qpos) > 0
        ):

            if float(
                gripper_qpos[0]
            ) < 0.01:

                action[7] = 0.0

            else:

                action[7] = 1.0

        # ------------------------------------------------------------
        # Axis-angle
        # ------------------------------------------------------------

        obs[EE_AA_NAME] = (
            self._quat2axisangle(
                np.asarray(
                    obs[EEF_QUAT_NAME],
                    dtype=np.float64,
                )
            )
        )

        return action

    def _add_joint_state_observation(
        self,
        obs,
        ur_messages,
    ):

        if (
            len(ur_messages) != 1
            or not isinstance(
                ur_messages[0],
                JointState,
            )
        ):

            self.get_logger().warn(
                'Expected exactly one '
                'JointState message.'
            )

            return

        msg = ur_messages[0]

        joint_name_to_index = {
            name: index
            for index, name
            in enumerate(msg.name)
        }

        required_names = (
            self.joint_robot_names
            + self.gripper_robot_names
        )

        missing_names = [

            name

            for name
            in required_names

            if name
            not in joint_name_to_index
        ]

        if missing_names:

            self.get_logger().warn(
                'JointState is missing: '
                + ', '.join(
                    missing_names
                )
            )

            return

        # ------------------------------------------------------------
        # Arm positions
        # ------------------------------------------------------------

        obs[JOINT_POS_NAME] = np.asarray(

            [
                msg.position[
                    joint_name_to_index[name]
                ]

                for name
                in self.joint_robot_names
            ],

            dtype=np.float64,
        )

        # ------------------------------------------------------------
        # Arm velocities
        # ------------------------------------------------------------

        obs[JOINT_VEL_NAME] = (
            self._extract_joint_vector(

                msg.velocity,

                self.joint_robot_names,

                joint_name_to_index,
            )
        )

        # ------------------------------------------------------------
        # Gripper positions
        # ------------------------------------------------------------

        obs[GRIPPER_QPOS_NAME] = np.asarray(

            [
                msg.position[
                    joint_name_to_index[name]
                ]

                for name
                in self.gripper_robot_names
            ],

            dtype=np.float64,
        )

        # ------------------------------------------------------------
        # Gripper velocities
        # ------------------------------------------------------------

        obs[GRIPPER_QVEL_NAME] = (
            self._extract_joint_vector(

                msg.velocity,

                self.gripper_robot_names,

                joint_name_to_index,
            )
        )

    @staticmethod
    def _extract_joint_vector(
        values,
        names,
        joint_name_to_index,
    ):

        if not values:

            return np.zeros(
                len(names),
                dtype=np.float64,
            )

        return np.asarray(

            [
                values[
                    joint_name_to_index[name]
                ]

                for name
                in names
            ],

            dtype=np.float64,
        )

    # ================================================================
    # EEF
    # ================================================================

    def _add_eef_observation(
        self,
        obs,
    ):

        try:

            transform = (
                self.tf_buffer.lookup_transform(

                    self.base_frame_name,

                    self.eef_frame_name,

                    rclpy.time.Time(),
                )
            )

        except Exception as exc:

            self.get_logger().warn(
                f'Could not look up TF '
                f'{self.base_frame_name} -> '
                f'{self.eef_frame_name}: '
                f'{exc}'
            )

            return

        obs[EEF_POS_NAME] = np.asarray(

            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],

            dtype=np.float64,
        )

        obs[EEF_QUAT_NAME] = np.asarray(

            [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ],

            dtype=np.float64,
        )

    # ================================================================
    # TRAJECTORY LIFECYCLE
    # ================================================================

    def _start_new_trajectory(
        self,
        trajectory_state,
    ):

        self._active_traj_id = (
            self._allocate_next_trajectory_id()
        )

        self._trajectory = (
            Trajectory(
                compress_camera_images=self.is_human_demo
            )
        )

        self.current_t = 0

        self.is_recording = True

        self.trajectory_state = (
            trajectory_state
        )

        self._video_writer = None
        self._video_path = None

        self.get_logger().info(

            f'Started '
            f'{self.collector_mode} '
            f'trajectory '
            f'{self._active_traj_id:03d} '

            f'for task='
            f'{self.task_name}, '

            f'variation='
            f'{self.variation_id}.'
        )

    def _finish_trajectory(self):

        if (
            self._trajectory is None
            or self._active_traj_id is None
        ):

            self.get_logger().warn(
                'No active trajectory to save.'
            )

            return None

        trajectory_path = (
            self._trajectory_path(
                self._active_traj_id
            )
        )

        video_path = self._video_path

        try:

            if self.is_human_demo:

                # The reference human dataset marks the final sample as done.
                self._trajectory.mark_last_done()

                # Match the reference top-level schema exactly:
                # ['traj', 'len', 'env_type', 'task_id']
                self._trajectory.save(
                    trajectory_path,
                    len=self.current_t,
                    env_type=self.task_name,
                    task_id=f'{self.variation_id:02d}',
                )

            else:

                # Preserve the current robot metadata and behavior.
                self._trajectory.save(

                    trajectory_path,

                    task_name=(
                        self.task_name
                    ),

                    variation_id=(
                        self.variation_id
                    ),

                    traj_count_id=(
                        self._active_traj_id
                    ),

                    human_demo=(
                        self.is_human_demo
                    ),

                    collector_mode=(
                        self.collector_mode
                    ),

                    num_steps=(
                        self.current_t
                    ),

                    video_path=(
                        str(video_path)
                        if video_path is not None
                        else None
                    ),
                )

        except Exception as exc:

            self.get_logger().error(
                f'Failed to save trajectory: '
                f'{exc}'
            )

            self._release_video_writer()

            self._reset_active_trajectory()

            return None

        # ------------------------------------------------------------
        # Close MP4
        # ------------------------------------------------------------

        self._release_video_writer()

        # ------------------------------------------------------------
        # Update last trajectory ID
        # ------------------------------------------------------------

        self.traj_count_id = (
            self._active_traj_id
        )

        self.get_logger().info(
            f'Saved trajectory with '
            f'{self.current_t} frames to '
            f'{trajectory_path}'
        )

        if video_path is not None:

            self.get_logger().info(
                f'Saved video to '
                f'{video_path}'
            )

        # ------------------------------------------------------------
        # Reset state
        # ------------------------------------------------------------

        self._reset_active_trajectory()

        # ------------------------------------------------------------
        # Robot home
        # ------------------------------------------------------------

        if (
            not self.is_human_demo
            and self.move_home_after_trajectory
        ):

            self.set_robot_to_home_position()

        return trajectory_path

    def _reset_active_trajectory(self):

        self._release_video_writer()

        self.current_t = 0

        self.is_recording = False

        self.trajectory_state = (
            TrajectoryState.TRAJECTORY_IDLE
        )

        self._trajectory = None

        self._active_traj_id = None

        self._video_path = None

    # ================================================================
    # TRAJECTORY PATHS
    # ================================================================

    def _allocate_next_trajectory_id(self):

        candidate = (
            self.traj_count_id + 1
        )

        while True:

            trajectory_path = (
                self._trajectory_path(
                    candidate
                )
            )

            video_path = (
                self._video_output_path(
                    candidate
                )
            )

            trajectory_exists = (
                trajectory_path.exists()
            )

            video_exists = (
                self.save_video
                and video_path.exists()
            )

            if (
                not trajectory_exists
                and not video_exists
            ):

                return candidate

            candidate += 1

    def _output_directory(self):

        return (

            Path(
                self.saving_directory
            ).expanduser()

            / self.collector_mode

            / self.task_name

            / f'task_'
              f'{self.variation_id:02d}'
        )

    def _trajectory_path(
        self,
        trajectory_id,
    ):

        return (

            self._output_directory()

            / f'traj'
              f'{trajectory_id:03d}'
              f'.pkl'
        )

    def _video_output_path(
        self,
        trajectory_id,
    ):

        return (

            self._output_directory()

            / f'traj'
              f'{trajectory_id:03d}'
              f'_{self.video_camera}'
              f'.mp4'
        )

    # ================================================================
    # VIDEO
    # ================================================================

    def _write_video_frame_if_enabled(
        self,
        obs,
    ):

        if not self.save_video:
            return

        image_key = (
            f'{self.video_camera}_image'
        )

        frame = obs.get(
            image_key
        )

        if frame is None:

            self.get_logger().warn(
                f'Video frame source '
                f'{image_key!r} '
                f'is unavailable.'
            )

            return

        # ------------------------------------------------------------
        # First frame -> initialize writer
        # ------------------------------------------------------------

        if self._video_writer is None:

            self._open_video_writer(
                frame
            )

        # ------------------------------------------------------------
        # Write
        # ------------------------------------------------------------

        if self._video_writer is not None:

            self._video_writer.write(
                frame
            )

    def _open_video_writer(
        self,
        frame,
    ):

        if self._active_traj_id is None:
            return

        output_path = (
            self._video_output_path(
                self._active_traj_id
            )
        )

        output_path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        height, width = (
            frame.shape[:2]
        )

        fourcc = (
            cv2.VideoWriter_fourcc(
                *self.video_codec
            )
        )

        writer = cv2.VideoWriter(

            str(output_path),

            fourcc,

            self.video_fps,

            (
                width,
                height,
            ),
        )

        if not writer.isOpened():

            writer.release()

            self.get_logger().error(
                f'Could not open video writer '
                f'for {output_path}. '
                f'Trajectory recording will '
                f'continue without MP4 output.'
            )

            # Avoid retrying on every frame.
            self.save_video = False

            return

        self._video_writer = writer

        self._video_path = (
            output_path
        )

        self.get_logger().info(
            f'Opened video writer: '
            f'{output_path}'
        )

    def _release_video_writer(self):

        if self._video_writer is None:
            return

        self._video_writer.release()

        self._video_writer = None

    # ================================================================
    # ROBOT HOME
    # ================================================================

    def set_robot_to_home_position(self):

        if self.is_human_demo:
            return

        if self.go_home_service is None:

            self.get_logger().warn(
                'GoHome service client '
                'is not configured.'
            )

            return

        if self.is_moving_home:

            self.get_logger().warn(
                'Robot is already '
                'moving home.'
            )

            return

        if not (
            self.go_home_service
            .service_is_ready()
        ):

            self.get_logger().warn(
                f'Service '
                f'{self.set_robot_to_home_service_name} '
                f'is not ready.'
            )

            return

        self.get_logger().info(
            'Setting robot to '
            'home position...'
        )

        self.is_moving_home = True

        request = GoHome.Request()

        future = (
            self.go_home_service.call_async(
                request
            )
        )

        future.add_done_callback(
            self._home_position_response_callback
        )

    def _home_position_response_callback(
        self,
        future,
    ):

        try:

            response = (
                future.result()
            )

            if response is None:

                self.get_logger().error(
                    'GoHome service returned '
                    'no response.'
                )

            elif response.success:

                self.get_logger().info(
                    f'Robot returned home: '
                    f'{response.message}'
                )

            else:

                self.get_logger().error(
                    'Failed to set robot '
                    'to home position: '
                    f'{response.message}'
                )

        except Exception as exc:

            self.get_logger().error(
                f'GoHome service call '
                f'failed: {exc}'
            )

        finally:

            self.is_moving_home = False

    # ================================================================
    # TF
    # ================================================================

    def wait_for_tf(
        self,
        timeout=5.0,
    ):

        start = (
            self.get_clock().now()
        )

        while rclpy.ok():

            # Needed here because this method is
            # called during __init__, before the
            # external executor starts spinning.

            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

            if self.tf_buffer.can_transform(

                self.base_frame_name,

                self.eef_frame_name,

                rclpy.time.Time(),
            ):

                self.get_logger().info(
                    f'TF available: '
                    f'{self.base_frame_name} -> '
                    f'{self.eef_frame_name}'
                )

                return True

            elapsed = (

                (
                    self.get_clock().now()
                    - start
                ).nanoseconds

                * 1e-9
            )

            if elapsed > timeout:

                return False

        return False

    # ================================================================
    # QUATERNION -> AXIS ANGLE
    # ================================================================

    @staticmethod
    def _quat2axisangle(
        quat,
    ):

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

            max(
                0.0,

                1.0
                - quat[3]
                * quat[3],
            )
        )

        if math.isclose(
            den,
            0.0,
        ):

            return np.zeros(
                3,
                dtype=np.float64,
            )

        return (

            quat[:3]

            * 2.0

            * math.acos(
                quat[3]
            )

            / den
        )

    # ================================================================
    # DEBUG VISUALIZATION
    # ================================================================

    def _show_observation(
        self,
        obs,
    ):

        for camera_name in self.camera_names:

            obs_name = (
                self.camera_names_obs_name_map[
                    camera_name
                ]
            )

            # --------------------------------------------------------
            # RGB
            # --------------------------------------------------------

            rgb = obs.get(
                f'{obs_name}_image'
            )

            if rgb is not None:

                cv2.imshow(
                    f'RGB {obs_name}',
                    rgb,
                )

            # --------------------------------------------------------
            # Depth
            # --------------------------------------------------------

            if self.record_depth:

                depth = obs.get(
                    f'{obs_name}_depth'
                )

                if depth is not None:

                    cv2.imshow(
                        f'Depth {obs_name}',
                        self._depth_for_display(
                            depth
                        ),
                    )

        cv2.waitKey(1)

    @staticmethod
    def _depth_for_display(
        depth,
    ):

        depth_array = np.asarray(
            depth
        )

        if depth_array.dtype == np.uint8:

            return depth_array

        finite_mask = np.isfinite(
            depth_array
        )

        if not np.any(
            finite_mask
        ):

            return np.zeros(
                depth_array.shape,
                dtype=np.uint8,
            )

        finite_values = (
            depth_array[
                finite_mask
            ]
        )

        minimum = float(
            np.min(
                finite_values
            )
        )

        maximum = float(
            np.max(
                finite_values
            )
        )

        if math.isclose(
            minimum,
            maximum,
        ):

            return np.zeros(
                depth_array.shape,
                dtype=np.uint8,
            )

        normalized = np.zeros(
            depth_array.shape,
            dtype=np.float32,
        )

        normalized[
            finite_mask
        ] = (

            (
                depth_array[
                    finite_mask
                ]
                - minimum
            )

            / (
                maximum
                - minimum
            )
        )

        return np.clip(

            normalized * 255.0,

            0,

            255,

        ).astype(
            np.uint8
        )

    # ================================================================
    # CONFIGURATION LOG
    # ================================================================

    def _log_configuration(self):

        rgb_topics = [

            (
                f'{camera_name}'
                f'{self.rgb_topic_suffix}'
            )

            for camera_name
            in self.camera_names
        ]

        if self.record_depth:

            depth_topics = [

                (
                    f'{camera_name}'
                    f'{self.depth_topic_suffix}'
                )

                for camera_name
                in self.camera_names
            ]

        else:

            depth_topics = []

        self.get_logger().info(

            'Dataset Collector configuration:\n'

            f'  collector_mode: '
            f'{self.collector_mode}\n'

            f'  camera_names: '
            f'{self.camera_names}\n'

            f'  rgb_topics: '
            f'{rgb_topics}\n'

            f'  record_depth: '
            f'{self.record_depth}\n'

            f'  depth_topics: '
            f'{depth_topics}\n'

            f'  sync_queue_size: '
            f'{self.sync_queue_size}\n'

            f'  sync_slop_sec: '
            f'{self.sync_slop_sec}\n'

            f'  show_images: '
            f'{self.show_images}\n'

            f'  annotate_objects: '
            f'{self.annotate_objects}\n'

            f'  task_name: '
            f'{self.task_name}\n'

            f'  variation_id: '
            f'{self.variation_id}\n'

            f'  traj_count_id: '
            f'{self.traj_count_id}\n'

            f'  saving_directory: '
            f'{self.saving_directory}\n'

            f'  save_video: '
            f'{self.save_video}\n'

            f'  video_camera: '
            f'{self.video_camera}\n'

            f'  video_fps: '
            f'{self.video_fps}'
        )

        if self.is_human_demo:

            self.get_logger().info(

                'Human recording controls:\n'

                f'  JOYSTICK TOGGLE: topic={self.human_joy_topic}, '
                f'button_index={self.human_record_button_index}\n'

                '  SERVICE START:\n'
                '    ros2 service call '
                '/dataset_collector/start '
                'std_srvs/srv/Trigger {}\n'

                '  SERVICE STOP:\n'
                '    ros2 service call '
                '/dataset_collector/stop '
                'std_srvs/srv/Trigger {}'
            )

    # ================================================================
    # SHUTDOWN
    # ================================================================

    def destroy_node(self):

        self._release_video_writer()

        if self.show_images:

            cv2.destroyAllWindows()

        return super().destroy_node()