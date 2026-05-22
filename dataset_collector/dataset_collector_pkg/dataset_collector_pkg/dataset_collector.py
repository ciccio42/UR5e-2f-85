import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image, JointState
from controller_manager_msgs.srv import LoadController, SwitchController, ListControllers
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import tf2_ros
from ur5e_2f_85_teleoperation_msg.msg import TrajectoryState
import cv2
from cv_bridge import CvBridge 
from geometry_msgs.msg import Pose
import time
from moveit_controller_srvs.srv import GoHome
from dataset_collector_pkg.utils import *
import numpy as np
import math
from rclpy.executors import SingleThreadedExecutor
from scripts import Trajectory
class DatasetCollector(Node):

    def __init__(self):
        super().__init__('dataset_collector')
        self.get_logger().info('Dataset Collector Node has been started.')

        # get all the paramters
        # Camera names
        self.declare_parameter('camera_names', ['/zed_front/zed_node', 
                                                '/zed_left/zed_node', 
                                                '/zed_right/zed_node'])
        self.camera_names = self.get_parameter('camera_names').get_parameter_value().string_array_value
        self.camera_names_obs_name_map = dict()
        for camera_name in self.camera_names:
            if 'front' in camera_name:
                self.camera_names_obs_name_map[camera_name] = 'front_camera'
                self.front_camera_name = camera_name
            elif 'left' in camera_name:
                self.camera_names_obs_name_map[camera_name] = 'left_camera'
            elif 'right' in camera_name:
                self.camera_names_obs_name_map[camera_name] = 'right_camera'
        self.cv_bridge = CvBridge()


        # Internal executor for handling async calls
        self.internal_executer = SingleThreadedExecutor()
        self.internal_executer.add_node(self)

        # Debug cv2 window
        self.declare_parameter('show_images', False)
        self.show_images = self.get_parameter('show_images').get_parameter_value().bool_value
        if self.show_images:
            # create cv2 windows
            self.get_logger().info('Creating debug cv2 windows for camera images...')
            for camera_name in self.camera_names:
                cv2.namedWindow(f'RGB {camera_name}', cv2.WINDOW_NORMAL)
                cv2.namedWindow(f'Depth {camera_name}', cv2.WINDOW_NORMAL)

        # Joint robot names
        self.declare_parameter('joint_robot_names', ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'])
        self.joint_robot_names = self.get_parameter('joint_robot_names').get_parameter_value().string_array_value

        # Gripper robot names
        self.declare_parameter('gripper_robot_names', ['robotiq_85_left_knuckle_joint'])
        self.gripper_robot_names = self.get_parameter('gripper_robot_names').get_parameter_value().string_array_value

        # EEF frame name
        self.declare_parameter('eef_frame_name', 'tcp_link')
        self.eef_frame_name = self.get_parameter('eef_frame_name').get_parameter_value().string_value

        # Topics to record
        self.declare_parameter('ur_topics_to_record', ['/joint_states'])
        self.ur_topics_to_record = self.get_parameter('ur_topics_to_record').get_parameter_value().string_array_value

        # Teleop state topic
        self.declare_parameter('teleop_state_topic', '/teleop_trajectory_state')
        self.teleop_state_topic = self.get_parameter('teleop_state_topic').get_parameter_value().string_value

        # Human demos
        self.declare_parameter('human_demo', False)
        self.human_demo = self.get_parameter('human_demo').get_parameter_value().bool_value

        # Task name
        self.declare_parameter('task_name', 'pick_place')
        self.task_name = self.get_parameter('task_name').get_parameter_value().string_value

        # Variation id
        self.declare_parameter('variation_id', 0)
        self.variation_id = self.get_parameter('variation_id').get_parameter_value().integer_value

        # Traj count id
        self.declare_parameter('traj_count_id', 0)
        self.traj_count_id = self.get_parameter('traj_count_id').get_parameter_value().integer_value

        # Create subscribers
        self.camera_subscribers_rgb = []
        self.camera_subscribers_depth = []
        for camera_name in self.camera_names:
            # RGB Image Subscriber
            sub = Subscriber(self, Image, f'{camera_name}/left/color/rect/image')
            self.camera_subscribers_rgb.append(sub)

            # Depth Image Subscriber
            sub_depth = Subscriber(self, Image, f'{camera_name}/depth/depth_registered')
            self.camera_subscribers_depth.append(sub_depth)

        self.ur_topics_record_subscribers = []
        for topic in self.ur_topics_to_record:
            if '/joint_states' in topic:
                sub_ur = Subscriber(self, JointState, topic)
            else:
                raise ValueError(f'Unsupported UR topic to record: {topic}')
            self.ur_topics_record_subscribers.append(sub_ur)

        self.teleop_state_subscriber = Subscriber(self, TrajectoryState, self.teleop_state_topic)  
        self.current_t = 0

        # Set the robot to home position if not human demo
        if not self.human_demo:
            # Ignore everything while robot is moving home
            self.is_moving_home = False
            # Wait for user to press enter
            self.get_logger().info('Setting robot to home position before starting dataset collection...')
            self.go_home_service_callback_group = ReentrantCallbackGroup()
            self.go_home_service = self.create_client(GoHome, 
                                                      'set_robot_to_home',
                                                      callback_group=self.go_home_service_callback_group)
            while not self.go_home_service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('set_robot_to_home service not available, waiting again...')

            input('Press Enter to set the robot to home position...')
            self.set_robot_to_home_position()
            # if not self.set_robot_to_home_position():
            #     self.get_logger().error('Could not set robot to home position. Exiting...')
            #     return
            # self.get_logger().info('Robot set to home position successfully.')

        # Create Time Synchronizer
        self.list_of_subs = self.camera_subscribers_rgb + self.camera_subscribers_depth + self.ur_topics_record_subscribers + [self.teleop_state_subscriber]
        if self.show_images:
            self.ts = ApproximateTimeSynchronizer(self.list_of_subs, 
                                                queue_size=1, 
                                                slop=5.0)
        else:
            self.ts = ApproximateTimeSynchronizer(self.list_of_subs, 
                                                queue_size=1, 
                                                slop=5.0)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        while not self.wait_for_tf(timeout=10.0):
            self.get_logger().warn('Waiting for TF between base_link and EEF frame to become available...')

        self.ts.registerCallback(self.synced_callback)

        self.get_logger().info(f'Configuration: \n'
                               f'Camera Names: {self.camera_names}\n'
                               f'EEF Frame Name: {self.eef_frame_name}\n'
                               f'Show Images: {self.show_images}\n'
                               f'UR Topics to Record: {self.ur_topics_to_record}\n'
                               f'Teleop State Topic: {self.teleop_state_topic}\n'
                               f'Human Demo: {self.human_demo}\n'
                               f'Task Name: {self.task_name}\n'
                               f'Variation ID: {self.variation_id}\n'
                               f'Traj Count ID: {self.traj_count_id}\n')

    def wait_for_tf(self, timeout=5.0):
        
        start = self.get_clock().now()

        while rclpy.ok():
            # allow callbacks to be processed
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.tf_buffer.can_transform(
                'base_link',
                self.eef_frame_name,
                rclpy.time.Time()
            ):
                self.get_logger().info("TF available!")
                return True

            if (self.get_clock().now() - start).nanoseconds * 1e-9 > timeout:
                self.get_logger().error("Timeout waiting for TF")
                return False

    def _quat2axisangle(self, quat):
        """
        Converts quaternion to axis-angle format.
        Returns a unit vector direction scaled by its angle in radians.
        Args:
            quat (np.array): (x,y,z,w) vec4 float angles
        Returns:
            np.array: (ax,ay,az) axis-angle exponential coordinates
        """
        # clip quaternion
        if quat[3] > 1.0:
            quat[3] = 1.0
        elif quat[3] < -1.0:
            quat[3] = -1.0

        den = np.sqrt(1.0 - quat[3] * quat[3])
        if math.isclose(den, 0.0):
            # This is (close to) a zero degree rotation, immediately return
            return np.zeros(3)

        return (quat[:3] * 2.0 * math.acos(quat[3])) / den
       
    def _home_position_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Robot returned home: {response.message}')
            else:
                self.get_logger().error(
                    f'Failed to set robot to home position: {response.message}'
                )
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

        # Important: allow callbacks to resume normal behavior
        self.is_moving_home = False

    # def set_robot_to_home_position(self):
    #     # Implement the logic to set the robot to home position
    #     self.get_logger().info('Setting robot to home position')
        
    #     request = GoHome.Request()
    #     future = self.go_home_service.call_async(request)
    #     rclpy.spin_until_future_complete(self, future)
    #     if future.result() is not None:
    #         if future.result().success:
    #             self.get_logger().info(f'{future.result().message}')
    #             return True
    #         else:
    #             self.get_logger().error('Failed to set robot to home position: ' + future.result().message)
    #             return False
    def set_robot_to_home_position(self):
        self.get_logger().info('Setting robot to home position')

        if self.is_moving_home:
            self.get_logger().warn('Already moving home, skipping request.')
            return

        # self.is_moving_home = True

        request = GoHome.Request()
        future = self.go_home_service.call_async(request)
        self.internal_executer.spin_until_future_complete(future)
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f'{future.result().message}')
            else:
                self.get_logger().error('Failed to set robot to home position: ' + future.result().message)
        else:
            self.get_logger().error('Service call failed while setting robot to home position.')
        
        # future = self.go_home_service.call_async(request)
        # future.add_done_callback(self._home_position_response_callback)
 
    def synced_callback(self, *args):
        # self.get_logge    r().info('Synchronized messages received.')
        if self.is_moving_home:
            self.get_logger().info('Currently moving to home position, ignoring synchronized messages...')
            return

        # Get the messages
        num_cameras = len(self.camera_names)
        rgb_images = args[0:num_cameras]
        depth_images = args[num_cameras:2*num_cameras]
        ur_msgs = args[2*num_cameras:2*num_cameras + len(self.ur_topics_to_record)]
        # self.get_logger().info(f'{ur_msgs} UR messages.')
        teleop_state_msg = args[-1]

        obs = dict()

        if teleop_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_IDLE:
            self.get_logger().info('Teleop state is IDLE...')  
        
        elif teleop_state_msg.trajectory_state != TrajectoryState.TRAJECTORY_END:
            self.get_logger().info(f'Teleop state is {teleop_state_msg.trajectory_state} - recording data...')
            # self.get_logger().info(f'Teleop state is {teleop_state_msg.trajectory_state}...')
            # save in obs images
            for i in range(num_cameras):
                obs[f'{self.camera_names_obs_name_map[self.camera_names[i]]}_image'] = self.cv_bridge.imgmsg_to_cv2(rgb_images[i], desired_encoding='bgr8')
                obs[f'{self.camera_names_obs_name_map[self.camera_names[i]]}_depth'] = self.cv_bridge.imgmsg_to_cv2(depth_images[i], desired_encoding='passthrough')

            if self.current_t == 0:
                self._trajectory = Trajectory()
                # get the object centers in the first frame of the trajectory and save them in the obs for future use
                self.get_logger().info('Recording object centers in the first frame of the trajectory...')
                # get the index of the camera front image
                front_img = obs[f'{self.camera_names_obs_name_map[self.front_camera_name]}_image']
                front_depth = obs[f'{self.camera_names_obs_name_map[self.front_camera_name]}_depth']
                obj_bb = get_object_centers(node=self,
                                                    task_name=self.task_name,
                                                    variation_id=self.variation_id, 
                                                    camera_name='front_camera',
                                                    rgb_image=front_img, 
                                                    depth_image=front_depth)
                # save object centers in obs
                obs['obj_bb'] = obj_bb

            if self.show_images:
                bridge = CvBridge()
                for i in range(num_cameras):
                    # Convert ROS Image to OpenCV Image
                    # self.get_logger().info(f'Displaying images for camera: {self.camera_names[i]}')
                    cv_rgb_image = bridge.imgmsg_to_cv2(rgb_images[i], desired_encoding='bgr8')
                    cv_depth_image = bridge.imgmsg_to_cv2(depth_images[i], desired_encoding='passthrough')

                    # Show images in cv2 windows
                    cv2.imshow(f'RGB {self.camera_names[i]}', cv_rgb_image)
                    cv2.imshow(f'Depth {self.camera_names[i]}', cv_depth_image)

                cv2.waitKey(1)  # Needed to update the cv2 windows

            # prepare observations
            if len(ur_msgs) == 1 and isinstance(ur_msgs[0], JointState):
                # self.get_logger().info('Recording joint states from /joint_states topic...')
                # self.get_logger().info(f'Joint names: {ur_msgs[0].name}')
                
                # Create a mapping from joint names to their indices in the message
                joint_name_to_index = {name: idx for idx, name in enumerate(ur_msgs[0].name)}
                # Extract the joint positions and velocities for the specified robot joints
                obs[JOINT_POS_NAME] = np.array([ur_msgs[0].position[joint_name_to_index[joint]] for joint in self.joint_robot_names])
                obs[JOINT_VEL_NAME] = np.array([ur_msgs[0].velocity[joint_name_to_index[joint]] for joint in self.joint_robot_names])
                
                # Get gripper states
                obs[GRIPPER_QPOS_NAME] = np.array([ur_msgs[0].position[joint_name_to_index[joint]] for joint in self.gripper_robot_names])
                obs[GRIPPER_QVEL_NAME] = np.array([ur_msgs[0].velocity[joint_name_to_index[joint]] for joint in self.gripper_robot_names])                

            else:
                self.get_logger().warn('No joint states received or multiple UR messages received, skipping joint state recording for this step.')

            # eef pose
            try:
                now = rclpy.time.Time()
                trans = self.tf_buffer.lookup_transform('base_link', self.eef_frame_name, now)
                obs[EEF_POS_NAME] = [trans.transform.translation.x, 
                                     trans.transform.translation.y,
                                     trans.transform.translation.z]
                obs[EEF_QUAT_NAME] = [trans.transform.rotation.x,
                                      trans.transform.rotation.y,
                                      trans.transform.rotation.z,
                                      trans.transform.rotation.w]
            except Exception as e:
                self.get_logger().error(f'Error looking up TF for EEF pose: {e}')
                
            # prepare actions
            if not self.human_demo:
                action = np.zeros(8)  # placeholder action, replace with actual action if available
                action[0:3] = obs[EEF_POS_NAME]
                action[3:7] = obs[EEF_QUAT_NAME]
                if obs.get(GRIPPER_QPOS_NAME) is not None:
                    if obs[GRIPPER_QPOS_NAME].shape[0] < 0.01:
                        action[7] = 0.0 # gripper open
                    else:
                        action[7] = 1.0 # gripper closed

                # convert to axis-angle representation
                obs[EE_AA_NAME] = self._quat2axisangle(np.array(obs[EEF_QUAT_NAME]))     

            # Compute the done signal
            done = teleop_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_END

            # Compute reward
            reward = 1 if teleop_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_END else 0
            
            # save step data
            
            
            self.current_t += 1

        elif self.current_t != 0 and teleop_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_END:
            
            # save traj
            self.get_logger().info(f'Teleop state is END\nSaving trajectory data {self.traj_count_id} for task {self.task_name}, variation {self.variation_id}...')
            self.traj_count_id += 1

            # Close cv2 windows if open
            if self.show_images:
                cv2.destroyAllWindows()

            # save trajectory
            # raise NotImplementedError('Trajectory saving not implemented yet.')
            self.get_logger().info('Trajectory saving not implemented yet.')
            self.current_t = 0

            # Wait for user input before moving to home position
            self.get_logger().info('Trajectory ended. Press Enter to continue to home position...')
            char = input()
            while char != '':
                self.get_logger().info('Invalid input. Please press Enter to continue to home position...')
                char = input()
        
            # move robot to home position if not human demo
            if not self.human_demo:
                self.get_logger().info('Setting robot to home position after trajectory end...')
                # if not self.set_robot_to_home_position():
                #     self.get_logger().error('Could not set robot to home position after trajectory end.')
                self.set_robot_to_home_position()
        
        elif self.current_t == 0 and teleop_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_END:
            self.get_logger().info('Teleop state is END but current_t is 0, put the trajectory to IDLE')


            


        