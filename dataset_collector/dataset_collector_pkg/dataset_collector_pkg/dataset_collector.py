import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image, JointState
from controller_manager_msgs.srv import LoadController, SwitchController, ListControllers
from ur5e_2f_85_teleoperation_msg.msg import TrajectoryState
import cv2
from cv_bridge import CvBridge 
from geometry_msgs.msg import Pose
import time
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

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
        self.declare_parameter('show_images', True)
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

        # Controller names
        self.declare_parameter('controller_to_run', 'scaled_joint_trajectory_controller')
        self.controller_to_run = self.get_parameter('controller_to_run').get_parameter_value().string_value
        self.declare_parameter('controller_to_stop', 'forward_position_controller')
        self.controller_to_stop = self.get_parameter('controller_to_stop').get_parameter_value().string_value


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
        # if not self.human_demo:
        #     # Wait for user to press enter
        #     input('Press Enter to set the robot to home position...')
        #     self.set_robot_to_home_position()

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
    
    def load_and_start_controller(self, controller_to_run=None, controller_to_stop=None):
        # check whether the controller is already loaded
        client_controller_state = self.create_client(ListControllers,
                                                     '/controller_manager/list_controllers')
        while not client_controller_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /controller_manager/list_controllers not available, waiting again...')
        client_controller_state_request = ListControllers.Request()
        future_controller_state = client_controller_state.call_async(client_controller_state_request)
        rclpy.spin_until_future_complete(self, future_controller_state)
        controllers = future_controller_state.result().controller
        for controller in controllers:
            self.get_logger().info(f'Found controller: {controller.name} with state {controller.state}')
            if controller.name == controller_to_run and controller.state == 'active':
                self.get_logger().info(f'Controller {controller_to_run} is already running.')
                return True  # No need to load and start

        # Load the controller to run
        client_switch = self.create_client(SwitchController, '/controller_manager/switch_controller')
        while not client_switch.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /controller_manager/switch_controller not available, waiting again...')
        
        request_switch = SwitchController.Request()
        request_switch.deactivate_controllers = [str(controller_to_stop)]
        request_switch.activate_controllers = [str(controller_to_run)]
        request_switch.strictness = SwitchController.Request.STRICT
        request_switch.activate_asap = True

        future_switch = client_switch.call_async(request_switch) 
        rclpy.spin_until_future_complete(self, future_switch)
        if future_switch.result() is not None:
            if future_switch.result().success:
                self.get_logger().info(f'Switched controllers: stopped {controller_to_stop}, started {controller_to_run}')
                return True
            else:
                self.get_logger().error('Failed to switch controllers')
                return False
        else:
            self.get_logger().error('Failed to switch controllers')
            return False
    
    
    def set_robot_to_home_position(self):
        # Implement the logic to set the robot to home position
        self.get_logger().info('Setting robot to home position')
        
        # Change controller to scaled joint trajectory controller
        if not self.load_and_start_controller(controller_to_run=self.controller_to_run, 
                                              controller_to_stop=self.controller_to_stop):
            self.get_logger().error('Could not start the required controller to set robot to home position.')
            raise RuntimeError('Could not start the required controller to set robot to home position.')
        else:
            # Wait a bit for connection
            time.sleep(2.0)

            # ---- Check available named targets ----
            targets = self.move_group.get_named_targets()
            self.get_logger().info(f"Available named targets: {targets}")

            if "start_scene_arm_tcp" not in targets:
                self.get_logger().error(
                    "NO 'start_scene_arm_tcp' named target found in SRDF! "
                    "Check ur5e_2f_85.srdf group_state definitions."
                )
                raise RuntimeError("Missing start_scene_arm_tcp position in MoveIt config")

            # ---- Plan to start_scene_arm_tcp ----
            self.move_group.set_named_target("start_scene_arm_tcp")
            self.get_logger().info("Planning motion to start_scene_arm_tcp...")
            plan = self.move_group.plan()

            # plan() in ROS2 returns tuple (success, plan, planning_time, error_code)
            if isinstance(plan, tuple):
                success = plan[0]
            else:
                success = plan

            if not success:
                self.get_logger().error("MoveIt planning to start_scene_arm_tcp failed")
                raise RuntimeError("Planning failed")

            # ---- Execute ----
            self.get_logger().info("Executing motion to start_scene_arm_tcp...")
            result = self.move_group.go(wait=True)

            # Stop residual motion
            self.move_group.stop()
            self.move_group.clear_pose_targets()

            if result:
                self.get_logger().info("Robot successfully reached start_scene_arm_tcp position")
            else:
                self.get_logger().error("Execution to start_scene_arm_tcp failed")
                raise RuntimeError("Execution failed")



        
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
        elif teleop_state_msg.trajectory_state != TrajectoryState.TRAJECTORY_START:
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
                
        elif teleop_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_END:
            # save traj
            pass
            
