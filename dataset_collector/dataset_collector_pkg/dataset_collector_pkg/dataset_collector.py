import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image, JointState
from controller_manager_msgs.srv import LoadController, SwitchController, ListControllers
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from ur5e_2f_85_teleoperation_msg.msg import TrajectoryState
import cv2
from cv_bridge import CvBridge 
from geometry_msgs.msg import Pose
import time
from moveit_controller_srvs.srv import GoHome

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

        # Debug cv2 window
        self.declare_parameter('show_images', False)
        self.show_images = self.get_parameter('show_images').get_parameter_value().bool_value
        if self.show_images:
            # create cv2 windows
            self.get_logger().info('Creating debug cv2 windows for camera images...')
            for camera_name in self.camera_names:
                cv2.namedWindow(f'RGB {camera_name}', cv2.WINDOW_NORMAL)
                cv2.namedWindow(f'Depth {camera_name}', cv2.WINDOW_NORMAL)

        # EEF frame name
        self.declare_parameter('eef_frame_name', '/tcp_link')
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
        self.declare_parameter('task_name', 'pick_and_place')
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


        # Set the robot to home position if not human demo
        if not self.human_demo:
            # Wait for user to press enter
            self.get_logger().info('Setting robot to home position before starting dataset collection...')
            self.go_home_service_callback_group = ReentrantCallbackGroup()
            self.go_home_service = self.create_client(GoHome, 
                                                      'set_robot_to_home',
                                                      callback_group=self.go_home_service_callback_group)
            while not self.go_home_service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('set_robot_to_home service not available, waiting again...')

            input('Press Enter to set the robot to home position...')
            if not self.set_robot_to_home_position():
                self.get_logger().error('Could not set robot to home position. Exiting...')
                return
            self.get_logger().info('Robot set to home position successfully.')

        # # Create Time Synchronizer
        # self.list_of_subs = self.camera_subscribers_rgb + self.camera_subscribers_depth + self.ur_topics_record_subscribers + [self.teleop_state_subscriber]
        # if self.show_images:
        #     self.ts = ApproximateTimeSynchronizer(self.list_of_subs, 
        #                                         queue_size=10, 
        #                                         slop=5.0)
        # else:
        #     self.ts = ApproximateTimeSynchronizer(self.list_of_subs, 
        #                                         queue_size=10, 
        #                                         slop=1.0)
                
        # self.ts.registerCallback(self.synced_callback)

        # self.get_logger().info(f'Configuration: \n'
        #                        f'Camera Names: {self.camera_names}\n'
        #                        f'EEF Frame Name: {self.eef_frame_name}\n'
        #                        f'Show Images: {self.show_images}\n'
        #                        f'UR Topics to Record: {self.ur_topics_to_record}\n'
        #                        f'Teleop State Topic: {self.teleop_state_topic}\n'
        #                        f'Human Demo: {self.human_demo}\n'
        #                        f'Task Name: {self.task_name}\n'
        #                        f'Variation ID: {self.variation_id}\n'
        #                        f'Traj Count ID: {self.traj_count_id}\n')
    
    def set_robot_to_home_position(self):
        # Implement the logic to set the robot to home position
        self.get_logger().info('Setting robot to home position')
        
        request = GoHome.Request()
        future = self.go_home_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f'{future.result().message}')
                return True
            else:
                self.get_logger().error('Failed to set robot to home position: ' + future.result().message)
                return False

        
    def synced_callback(self, *args):
        # self.get_logger().info('Synchronized messages received.')

        # Get the messages
        num_cameras = len(self.camera_names)
        rgb_images = args[0:num_cameras]
        depth_images = args[num_cameras:2*num_cameras]
        ur_msgs = args[2*num_cameras:2*num_cameras + len(self.ur_topics_to_record)]
        teleop_state_msg = args[-1]

        if teleop_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_IDLE:
            self.get_logger().info('Teleop state is IDLE...')  
        elif teleop_state_msg.trajectory_state != TrajectoryState.TRAJECTORY_END:
            # self.get_logger().info(f'Teleop state is {teleop_state_msg.trajectory_state}...')
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

            # prepare actions

            # save step data
                
        elif teleop_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_END:
            # save traj
            self.get_logger().info(f'Teleop state is END\nSaving trajectory data {self.traj_count_id} for task {self.task_name}, variation {self.variation_id}...')
            self.traj_count_id += 1

            # Close cv2 windows if open
            if self.show_images:
                cv2.destroyAllWindows()
            

            # sav trajectory
            raise NotImplementedError('Trajectory saving not implemented yet.')
