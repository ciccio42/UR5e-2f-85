"""script_controller_node: a scripted (non-learned) pick-place controller.

At the start of every trajectory the frontal ZED camera frame is shown to the
user, who clicks the target object and the target placing location with the
mouse. Those two pixels are deprojected to 3D (using the clicked pixel's
depth and the camera intrinsics), transformed from the camera frame into the
raw ArUco marker frame using the extrinsics calibrated in
zed_camera/zed_camera_calibration/estimated_camera_positions.yaml, then into
the table_0 frame via a fixed offset (see script_controller.vision's
ARUCO_TO_TABLE0_ROTATION), and finally from table_0 into base_link via TF
(this node looks up base_link -> table_0, it does not broadcast it: something
else - e.g. a static_transform_publisher - must publish it). The resulting
object/bin poses drive six
motion primitives (reach, approaching, pick, lift_up, moving, placing, see
ai_controller.script_controller.primitives) that move the robot in linear
micro-steps of at least ``min_step`` meters. All 4 camera RGB+depth frames
are recorded at every micro-step into a rollout .pkl, following the same
Trajectory saving convention as ai_controller_node.py.
"""
import json
import sys
import threading
from pathlib import Path

import cv2
import message_filters
import numpy as np
import os
import rclpy
import rclpy.wait_for_message
import tf2_ros
from control_msgs.action import GripperCommand
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from moveit_controller_srvs.srv import GoHome, GoToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image as RosImage, JointState
from visualization_msgs.msg import Marker, MarkerArray

from ai_controller.script_controller.geometry_utils import GripperPose
from ai_controller.script_controller.primitives import MotionPrimitives
from ai_controller.script_controller.vision import (
    ClickCollector,
    aruco_point_to_table0,
    camera_point_to_aruco,
    deproject_pixel,
    get_camera_intrinsics,
    load_camera_calibration,
    robust_depth_at,
)
from ai_controller.utils.utils import (
    EEF_POS_NAME, EEF_QUAT_NAME, GRIPPER_QPOS_NAME, GRIPPER_QVEL_NAME,
    JOINT_POS_NAME, JOINT_VEL_NAME, _quat2mat,
)

_trajectory_cls = None


def _get_trajectory_cls(node):
    """Lazily resolve dataset_collector_pkg's savers.Trajectory class."""
    global _trajectory_cls
    if _trajectory_cls is None:
        node._add_dataset_collector_scripts_to_path()
        from savers import Trajectory as TrajectoryClass
        _trajectory_cls = TrajectoryClass
    return _trajectory_cls


class ScriptControllerNode(Node):

    PREVIEW_WINDOW_NAME = 'script_controller - front | gripper'

    def __init__(self):
        super().__init__('script_controller_node')
        self.get_logger().info('Script Controller Node has been started.')

        self._declare_parameters()
        self._get_parameters()

        # -- ROS2 interfaces ------------------------------------------------
        self.get_logger().info(f'Waiting for service {self.set_home_service}...')
        self.set_home_client = self.create_client(GoHome, self.set_home_service)
        self.set_home_client.wait_for_service()

        self.get_logger().info(f'Waiting for service {self.set_pose_service}...')
        self.set_pose_client = self.create_client(GoToPose, self.set_pose_service)
        self.set_pose_client.wait_for_service()

        self.gripper_action_client = ActionClient(self, GripperCommand, self.gripper_action_topic)

        self.latest_joint_state = None
        self.create_subscription(JointState, self.joint_states_topic, self._joint_state_callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # base_link -> table_0 is expected to already be published elsewhere (e.g. a
        # static_transform_publisher derived from the robot/table CAD geometry) -
        # this node only looks it up, it does not broadcast it itself.
        self._wait_for_table_tf()

        self.bridge = CvBridge()

        # RViz visualization: the full planned path (per primitive) is published here
        # the moment it is computed, independently of move_robot, so the whole intended
        # pick-place route can be inspected before/while anything actually executes.
        self.marker_pub = self.create_publisher(
            MarkerArray, 'script_controller/planned_waypoints',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._marker_array = []
        self._marker_id_counter = 0
        self._marker_colors = {
            'reach': (0.20, 0.40, 1.00),
            'approaching': (0.00, 0.80, 0.80),
            'lift_up': (0.20, 0.90, 0.20),
            'moving': (0.90, 0.90, 0.10),
            'placing': (0.90, 0.30, 0.00),
        }

        # Calibration: camera pose relative to table_0, per camera name.
        self.get_logger().info(f'Loading camera calibration from {self.camera_calibration_path}')
        self.camera_calibration = load_camera_calibration(self.camera_calibration_path)
        if self.click_camera_name not in self.camera_calibration:
            raise ValueError(
                f'click_camera_name={self.click_camera_name!r} has no entry in '
                f'{self.camera_calibration_path}; available cameras: {list(self.camera_calibration)}')

        # camera_name -> short obs key ('zed_front' -> 'front_camera'), mirrors
        # dataset_collector_pkg/dataset_collector.py's camera_names_obs_name_map.
        self.camera_obs_name_map = {}
        for camera_name in self.camera_names:
            for keyword, obs_name in (('front', 'front_camera'), ('left', 'left_camera'),
                                       ('right', 'right_camera'), ('gripper', 'gripper_camera')):
                if keyword in camera_name:
                    self.camera_obs_name_map[camera_name] = obs_name
                    break
            else:
                self.camera_obs_name_map[camera_name] = camera_name

        # Live preview window (front + gripper cameras side by side), updated at every
        # recorded micro-step while the controller is running.
        self.preview_camera_names = {}
        for camera_name, obs_name in self.camera_obs_name_map.items():
            if obs_name == 'front_camera':
                self.preview_camera_names['front'] = camera_name
            elif obs_name == 'gripper_camera':
                self.preview_camera_names['gripper'] = camera_name
        if len(self.preview_camera_names) == 2:
            cv2.namedWindow(self.PREVIEW_WINDOW_NAME, cv2.WINDOW_NORMAL)
        else:
            self.get_logger().warning(
                'Could not find both a front_camera and gripper_camera in camera_names; '
                'live front/gripper preview window disabled.')

        self._wait_for_camera_topics()
        self._setup_recording_subscribers()

        self.motion = MotionPrimitives(
            node=self,
            set_pose_client=self.set_pose_client,
            gripper_action_client=self.gripper_action_client,
            frame_id=self.frame_id,
            min_step=self.min_step,
            reach_hover_height=self.reach_hover_height,
            approach_z_offset=self.approach_z_offset,
            lift_height=self.lift_height,
            gripper_open_position=self.gripper_open_position,
            gripper_closed_position=self.gripper_closed_position,
            gripper_max_effort=self.gripper_max_effort,
            move_robot=self.move_robot,
        )

        os.makedirs(self.save_rollout_path, exist_ok=True)
        self.get_logger().info('Script Controller Node initialization complete.')

    # -- setup helpers --------------------------------------------------

    def _declare_parameters(self):
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('eef_frame_name', 'tcp_link')
        self.declare_parameter('table_frame_id', 'table_0')

        self.declare_parameter('set_home_service', 'set_robot_to_home')
        self.declare_parameter('set_pose_service', 'set_robot_to_pose')
        self.declare_parameter('gripper_action_topic', '/robotiq_gripper_controller/gripper_cmd')

        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('joint_robot_names', ['elbow_joint', 'shoulder_lift_joint',
                                                       'shoulder_pan_joint', 'wrist_1_joint',
                                                       'wrist_2_joint', 'wrist_3_joint'])
        self.declare_parameter('gripper_robot_names', ['robotiq_85_left_knuckle_joint'])

        self.declare_parameter('camera_names', ['zed_front', 'zed_left', 'zed_right', 'zed_gripper'])
        self.declare_parameter('camera_node_name', 'zed_node')
        self.declare_parameter('click_camera_name', 'zed_front')
        self.declare_parameter('sync_slop', 2.0)
        self.declare_parameter('depth_window', 5)

        self.declare_parameter('camera_calibration_path',
                                '/home/ros2_ws/src/zed_camera/zed_camera_calibration/estimated_camera_positions.yaml')

        self.declare_parameter('grasp_orientation', [0.9994452044624775,
                                                       0.03161651380119412,
                                                       0.0021438049655468088,
                                                       0.010251021036213035])

        self.declare_parameter('min_step', 0.02)
        self.declare_parameter('reach_hover_height', 0.15)
        self.declare_parameter('approach_z_offset', 0.0)
        self.declare_parameter('lift_height', 0.15)

        self.declare_parameter('gripper_open_position', 0.1)
        self.declare_parameter('gripper_closed_position', 0.8)
        self.declare_parameter('gripper_max_effort', 50.0)

        self.declare_parameter('move_robot', False)
        self.declare_parameter('move_to_initial_pose', True)
        self.declare_parameter('pose_before_first_inference', [-0.15552094619366708,
                                                                 0.34869994018501943,
                                                                 0.1532803451753288,
                                                                 0.9994452044624775,
                                                                 0.03161651380119412,
                                                                 0.0021438049655468088,
                                                                 0.010251021036213035])

        self.declare_parameter('task_name', 'pick_place')
        self.declare_parameter('save_rollout_path', '/home/ros2_ws/src/ai_controller/saved_rollouts/script_controller')

    def _get_parameters(self):
        gpv = self.get_parameter
        self.frame_id = gpv('frame_id').get_parameter_value().string_value
        self.eef_frame_name = gpv('eef_frame_name').get_parameter_value().string_value
        self.table_frame_id = gpv('table_frame_id').get_parameter_value().string_value

        self.set_home_service = gpv('set_home_service').get_parameter_value().string_value
        self.set_pose_service = gpv('set_pose_service').get_parameter_value().string_value
        self.gripper_action_topic = gpv('gripper_action_topic').get_parameter_value().string_value

        self.joint_states_topic = gpv('joint_states_topic').get_parameter_value().string_value
        self.joint_robot_names = gpv('joint_robot_names').get_parameter_value().string_array_value
        self.gripper_robot_names = gpv('gripper_robot_names').get_parameter_value().string_array_value

        self.camera_names = list(gpv('camera_names').get_parameter_value().string_array_value)
        self.camera_node_name = gpv('camera_node_name').get_parameter_value().string_value
        self.click_camera_name = gpv('click_camera_name').get_parameter_value().string_value
        self.sync_slop = gpv('sync_slop').get_parameter_value().double_value
        self.depth_window = gpv('depth_window').get_parameter_value().integer_value

        self.camera_calibration_path = gpv('camera_calibration_path').get_parameter_value().string_value

        self.grasp_orientation = np.array(
            gpv('grasp_orientation').get_parameter_value().double_array_value, dtype=np.float64)

        self.min_step = gpv('min_step').get_parameter_value().double_value
        self.reach_hover_height = gpv('reach_hover_height').get_parameter_value().double_value
        self.approach_z_offset = gpv('approach_z_offset').get_parameter_value().double_value
        self.lift_height = gpv('lift_height').get_parameter_value().double_value

        self.gripper_open_position = gpv('gripper_open_position').get_parameter_value().double_value
        self.gripper_closed_position = gpv('gripper_closed_position').get_parameter_value().double_value
        self.gripper_max_effort = gpv('gripper_max_effort').get_parameter_value().double_value

        self.move_robot = gpv('move_robot').get_parameter_value().bool_value
        self.move_to_initial_pose_enabled = gpv('move_to_initial_pose').get_parameter_value().bool_value
        self.pose_before_first_inference = gpv('pose_before_first_inference').get_parameter_value().double_array_value

        self.task_name = gpv('task_name').get_parameter_value().string_value
        self.save_rollout_path = gpv('save_rollout_path').get_parameter_value().string_value

    def _wait_for_table_tf(self, timeout_sec=10.0):
        """Wait for an externally-published base_link -> table_0 transform (e.g. a
        static_transform_publisher derived from the robot/table CAD geometry).
        This node does not broadcast table_0 itself."""
        start = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.tf_buffer.can_transform(self.frame_id, self.table_frame_id, rclpy.time.Time()):
                self.get_logger().info(f'TF {self.frame_id} -> {self.table_frame_id} is available.')
                return
            if (self.get_clock().now() - start).nanoseconds / 1e9 > timeout_sec:
                self.get_logger().error(
                    f'Timed out waiting for TF {self.frame_id} -> {self.table_frame_id}. Make sure '
                    f'something (e.g. a static_transform_publisher) is publishing this transform - '
                    'script_controller_node does not broadcast it itself.')
                return

    def _wait_for_camera_topics(self):
        topics = []
        for camera_name in self.camera_names:
            topics.append(f'/{camera_name}/{self.camera_node_name}/rgb/color/rect/image')
            topics.append(f'/{camera_name}/{self.camera_node_name}/depth/depth_registered')

        self.get_logger().info(f'Waiting for camera topics: {topics}')
        for topic in topics:
            found = False
            while not found:
                topics_info = self.get_topic_names_and_types()
                found = any(name == topic for name, _ in topics_info)
                if not found:
                    self.get_logger().info(f'Camera topic {topic} not available yet. Waiting...')
                    rclpy.spin_once(self, timeout_sec=1.0)
            self.get_logger().info(f'Camera topic {topic} is available.')

    def _setup_recording_subscribers(self):
        """Synchronized RGB + depth subscribers across all 4 cameras, used to
        record every micro-step of the executed rollout (mirrors
        dataset_collector_pkg/dataset_collector.py's camera subscriber setup)."""
        self.rgb_subs = [
            message_filters.Subscriber(self, RosImage,
                                        f'/{cam}/{self.camera_node_name}/rgb/color/rect/image',
                                        qos_profile=qos_profile_sensor_data)
            for cam in self.camera_names
        ]
        self.depth_subs = [
            message_filters.Subscriber(self, RosImage,
                                        f'/{cam}/{self.camera_node_name}/depth/depth_registered',
                                        qos_profile=qos_profile_sensor_data)
            for cam in self.camera_names
        ]

        self.latest_synced_frames = None
        self.synced_frames_event = threading.Event()

        self.camera_sync = message_filters.ApproximateTimeSynchronizer(
            self.rgb_subs + self.depth_subs, queue_size=10, slop=self.sync_slop)
        self.camera_sync.registerCallback(self._synced_frames_callback)

    def _synced_frames_callback(self, *msgs):
        num_cameras = len(self.camera_names)
        rgb_msgs = msgs[:num_cameras]
        depth_msgs = msgs[num_cameras:]
        frames = {}
        for i, camera_name in enumerate(self.camera_names):
            frames[camera_name] = {
                'rgb': self.bridge.imgmsg_to_cv2(rgb_msgs[i], desired_encoding='bgr8'),
                'depth': self.bridge.imgmsg_to_cv2(depth_msgs[i], desired_encoding='passthrough'),
            }
        self.latest_synced_frames = frames
        self.synced_frames_event.set()

    def get_synced_frames(self, timeout_sec=10.0):
        """Block (while spinning callbacks) until a fresh synchronized set of
        RGB+depth frames from all 4 cameras arrives."""
        self.synced_frames_event.clear()
        start = self.get_clock().now()
        while not self.synced_frames_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().error('Timed out waiting for synchronized camera frames.')
                return None
        return self.latest_synced_frames

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

    def _joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def _capture_joint_state(self, timeout_sec=1.0):
        """Spin briefly to receive a fresh /joint_states, returning joint/gripper
        qpos/qvel fields (same field names as ai_controller_node.py / dataset_collector.py)."""
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
        return state

    def _lookup_transform(self, target_frame, source_frame, max_attempts=100):
        """tf_buffer.lookup_transform, retrying (while spinning so new TF messages
        can arrive) up to max_attempts times before giving up."""
        last_exc = None
        for attempt in range(1, max_attempts + 1):
            try:
                return self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as exc:
                last_exc = exc
                self.get_logger().warning(
                    f'lookup_transform({target_frame} -> {source_frame}) failed '
                    f'(attempt {attempt}/{max_attempts}): {exc}')
                rclpy.spin_once(self, timeout_sec=0.1)

        raise RuntimeError(
            f'Failed to look up transform {target_frame} -> {source_frame} after '
            f'{max_attempts} attempts: {last_exc}')

    def get_current_gripper_pose(self):
        trans = self._lookup_transform(self.frame_id, self.eef_frame_name)
        pos = np.array([trans.transform.translation.x,
                         trans.transform.translation.y,
                         trans.transform.translation.z])
        quat = np.array([trans.transform.rotation.x,
                          trans.transform.rotation.y,
                          trans.transform.rotation.z,
                          trans.transform.rotation.w])
        return GripperPose(pos, quat)

    # -- RViz waypoint visualization --------------------------------------

    def _next_marker_id(self):
        marker_id = self._marker_id_counter
        self._marker_id_counter += 1
        return marker_id

    def _publish_marker_array(self):
        msg = MarkerArray()
        msg.markers = list(self._marker_array)
        self.marker_pub.publish(msg)

    def clear_waypoint_markers(self):
        """Delete everything previously plotted, ready for a fresh trajectory."""
        self._marker_array = []
        self._marker_id_counter = 0
        clear = MarkerArray()
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.action = Marker.DELETEALL
        clear.markers.append(m)
        self.marker_pub.publish(clear)

    def _publish_target_markers(self, target_object_pose, target_bin_pose):
        for ns, pose, color in (('target_object', target_object_pose, (0.1, 1.0, 0.1)),
                                 ('target_bin', target_bin_pose, (1.0, 0.1, 0.1))):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = ns
            m.id = self._next_marker_id()
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(pose.position[0])
            m.pose.position.y = float(pose.position[1])
            m.pose.position.z = float(pose.position[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.04
            m.color.r, m.color.g, m.color.b = color
            m.color.a = 0.9
            self._marker_array.append(m)
        self._publish_marker_array()

    def publish_waypoint_plan(self, label, start_pos, waypoints):
        """Plot one primitive's planned waypoints (line + points) in RViz. Called by
        MotionPrimitives as soon as a segment is computed, before it is executed."""
        color = self._marker_colors.get(label, (0.6, 0.6, 0.6))
        points = [Point(x=float(start_pos[0]), y=float(start_pos[1]), z=float(start_pos[2]))]
        points += [Point(x=float(wp.position[0]), y=float(wp.position[1]), z=float(wp.position[2]))
                   for wp in waypoints]

        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = label
        line.id = self._next_marker_id()
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.004
        line.color.r, line.color.g, line.color.b = color
        line.color.a = 0.9
        line.points = points
        self._marker_array.append(line)

        spheres = Marker()
        spheres.header.frame_id = self.frame_id
        spheres.header.stamp = line.header.stamp
        spheres.ns = label
        spheres.id = self._next_marker_id()
        spheres.type = Marker.SPHERE_LIST
        spheres.action = Marker.ADD
        spheres.scale.x = spheres.scale.y = spheres.scale.z = 0.015
        spheres.color.r, spheres.color.g, spheres.color.b = color
        spheres.color.a = 1.0
        spheres.points = points
        self._marker_array.append(spheres)

        self._publish_marker_array()

    # -- pixel -> base_link world point ---------------------------------

    def pixel_to_base_link(self, u, v, depth_image, camera_matrix):
        depth = robust_depth_at(depth_image, u, v, window=self.depth_window)
        if depth is None:
            raise RuntimeError(f'No valid depth found around pixel ({u}, {v}); click a different point.')

        point_cam = deproject_pixel(u, v, depth, camera_matrix)
        point_aruco = camera_point_to_aruco(point_cam, self.camera_calibration[self.click_camera_name])
        point_table0 = aruco_point_to_table0(point_aruco)

        trans = self._lookup_transform(self.frame_id, self.table_frame_id)
        R = _quat2mat([trans.transform.rotation.x, trans.transform.rotation.y,
                       trans.transform.rotation.z, trans.transform.rotation.w])
        t = np.array([trans.transform.translation.x,
                      trans.transform.translation.y,
                      trans.transform.translation.z])
        return R @ point_table0 + t

    def collect_target_poses(self):
        """Show the click camera's RGB frame, ask the user to click the target
        object then the target placing location, and return the corresponding
        (target_object_pose, target_bin_pose) GripperPose in base_link."""
        rgb_topic = f'/{self.click_camera_name}/{self.camera_node_name}/rgb/color/rect/image'
        depth_topic = f'/{self.click_camera_name}/{self.camera_node_name}/depth/depth_registered'
        info_topic = f'/{self.click_camera_name}/{self.camera_node_name}/rgb/color/rect/camera_info'

        ok_rgb, rgb_msg = rclpy.wait_for_message.wait_for_message(
            topic=rgb_topic, msg_type=RosImage, node=self, time_to_wait=5.0)
        ok_depth, depth_msg = rclpy.wait_for_message.wait_for_message(
            topic=depth_topic, msg_type=RosImage, node=self, time_to_wait=5.0)
        if not ok_rgb or not ok_depth:
            raise RuntimeError(f'Failed to receive RGB/depth from {self.click_camera_name}.')

        rgb_bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        camera_matrix = get_camera_intrinsics(self, info_topic)

        collector = ClickCollector()
        try:
            obj_uv = collector.collect(
                rgb_bgr, 'Click the TARGET OBJECT\nSPACE: confirm   ESC: abort')
            place_uv = collector.collect(
                rgb_bgr, 'Click the TARGET PLACING location\nSPACE: confirm   ESC: abort')
        finally:
            collector.close()

        obj_pos = self.pixel_to_base_link(obj_uv[0], obj_uv[1], depth_image, camera_matrix)
        bin_pos = self.pixel_to_base_link(place_uv[0], place_uv[1], depth_image, camera_matrix)

        self.get_logger().info(f'Target object position (base_link): {obj_pos}')
        self.get_logger().info(f'Target bin position (base_link): {bin_pos}')
        obj_pos[1] -= 0.04  # offset to move the object y in the object
        target_object_pose = GripperPose(obj_pos, self.grasp_orientation)
        target_bin_pose = GripperPose(bin_pos, self.grasp_orientation)
        return target_object_pose, target_bin_pose

    # -- robot motion helpers --------------------------------------------

    def go_home(self):
        future = self.set_home_client.call_async(GoHome.Request())
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None or not response.success:
            message = getattr(response, 'message', 'no response')
            raise RuntimeError(f'Failed to set robot to home position: {message}')
        self.get_logger().info(response.message)

    def move_to_initial_pose(self):
        self.get_logger().info('Moving robot to initial pose before showing the camera view...')
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

        if not self.move_robot:
            self.get_logger().info('[DRY RUN] would move to pose_before_first_inference.')
            return

        future = self.set_pose_client.call_async(pose_request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None or not response.success:
            message = getattr(response, 'message', 'no response')
            raise RuntimeError(f'Failed to move robot to initial pose: {message}')

        gripper_goal = GripperCommand.Goal()
        gripper_goal.command.position = self.gripper_open_position
        gripper_goal.command.max_effort = self.gripper_max_effort
        future = self.gripper_action_client.send_goal_async(gripper_goal)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            raise RuntimeError(f'Failed to open gripper before first inference: {future.exception()}')

    # -- rollout recording -------------------------------------------------

    def _show_camera_preview(self, frames):
        """Combine the front and gripper camera RGB frames into a single cv2 window."""
        if len(self.preview_camera_names) != 2:
            return

        front = frames[self.preview_camera_names['front']]['rgb']
        gripper = frames[self.preview_camera_names['gripper']]['rgb']

        height = min(front.shape[0], gripper.shape[0])

        def resize_to_height(img, height):
            if img.shape[0] == height:
                return img
            scale = height / img.shape[0]
            return cv2.resize(img, (int(img.shape[1] * scale), height))

        front = resize_to_height(front, height).copy()
        gripper = resize_to_height(gripper, height).copy()
        cv2.putText(front, 'front', (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(gripper, 'gripper', (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 0), 2, cv2.LINE_AA)

        combined = np.hstack([front, gripper])
        cv2.imshow(self.PREVIEW_WINDOW_NAME, combined)
        cv2.waitKey(1)

    def _record_step(self, traj, pos, quat, event, gripper_command, done=False):
        frames = self.get_synced_frames()
        if frames is None:
            self.get_logger().warning('Skipping step recording: no synchronized camera frames.')
            return

        self._show_camera_preview(frames)

        obs = dict(self._capture_joint_state())
        for camera_name in self.camera_names:
            obs_name = self.camera_obs_name_map[camera_name]
            obs[f'{obs_name}_image'] = frames[camera_name]['rgb']
            obs[f'{obs_name}_depth'] = frames[camera_name]['depth']

        obs[EEF_POS_NAME] = np.asarray(pos, dtype=np.float64)
        obs[EEF_QUAT_NAME] = np.asarray(quat, dtype=np.float64)

        gripper_normalized = 1.0 if abs(gripper_command - self.gripper_closed_position) < \
            abs(gripper_command - self.gripper_open_position) else 0.0
        action = np.concatenate([np.asarray(pos, dtype=np.float64),
                                  np.asarray(quat, dtype=np.float64),
                                  [gripper_normalized]])

        traj.append(obs=obs, action=action, done=done, reward=1 if done else 0)
        self.get_logger().info(f'Recorded step (event={event}, gripper={gripper_command}, done={done}).')

    def save_rollout(self, traj, task_id, traj_number):
        complete_save_path = os.path.join(self.save_rollout_path, f'task_{task_id}')
        os.makedirs(complete_save_path, exist_ok=True)
        traj_name = 'traj_{:03d}'.format(traj_number)

        trajectory_path = os.path.join(complete_save_path, traj_name + '.pkl')
        traj.save(trajectory_path, task_id=task_id, traj_number=traj_number,
                  ai_controller_target='script_controller', task_name=self.task_name)
        self.get_logger().info(f'Saved rollout trajectory to {trajectory_path}')

        res_dict = {}
        res_dict['object_reached'] = int(input('Did the robot successfully reach the target object? [1,0]: '))
        res_dict['object_picked'] = int(input('Did the robot successfully pick the target object? [1,0]: '))
        res_dict['object_placed'] = int(input('Did the robot successfully place the target object? [1,0]: '))
        with open(os.path.join(complete_save_path, traj_name + '.json'), 'w') as f:
            json.dump(res_dict, f)

    # -- main loop -----------------------------------------------------

    def control_loop(self):
        self.get_logger().info('Starting script controller loop...')
        Trajectory = _get_trajectory_cls(self)

        traj_cnt = int(input('Write the current trajectory count to the console: '))

        while rclpy.ok():
            input('Press Enter to start. Make sure the robot is in a safe position.')
            task_id = input('Enter task ID (e.g., 1, 2, 3): ').zfill(2)

            self.get_logger().info('Setting robot to home position...')
            self.go_home()
            if self.move_to_initial_pose_enabled:
                self.move_to_initial_pose()

            target_object_pose, target_bin_pose = self.collect_target_poses()

            self.clear_waypoint_markers()
            self._publish_target_markers(target_object_pose, target_bin_pose)

            input('Press Enter to execute the scripted pick-place with these targets '
                  '(Ctrl+C to abort). Check RViz -> MarkerArray on '
                  'script_controller/planned_waypoints for the full planned path as it '
                  'is computed...')

            traj = Trajectory()

            def on_step(pos, quat, event, gripper_command):
                self._record_step(traj, pos, quat, event, gripper_command, done=False)

            on_plan = self.publish_waypoint_plan

            current_pose = self.get_current_gripper_pose()
            current_pose = self.motion.reach(current_pose, target_object_pose, on_step, on_plan)
            current_pose = self.motion.approaching(current_pose, target_object_pose, on_step, on_plan)
            current_pose = self.motion.pick(current_pose, target_object_pose, on_step, on_plan)
            current_pose = self.motion.lift_up(current_pose, target_object_pose, on_step, on_plan)
            current_pose = self.motion.moving(current_pose, target_bin_pose, on_step, on_plan)
            current_pose = self.motion.placing(current_pose, target_bin_pose, on_step, on_plan)

            # Final frame explicitly marked done=True/reward=1, mirroring the
            # gripper close->open episode-end convention used in ai_controller_node.py.
            self._record_step(traj, current_pose.position, current_pose.orientation,
                               'episode_end', self.gripper_open_position, done=True)

            self.save_rollout(traj, task_id=task_id, traj_number=traj_cnt)
            traj_cnt += 1


def main(args=None):
    rclpy.init(args=args)
    node = ScriptControllerNode()
    executor = MultiThreadedExecutor(num_threads=1)
    executor.add_node(node)

    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        node.control_loop()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
