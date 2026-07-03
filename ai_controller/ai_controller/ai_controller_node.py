import threading

import message_filters
import rclpy
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as RosImage
from PIL import Image
import os
from moveit_controller_srvs.srv import GoHome, GoToPose
from control_msgs.action import GripperCommand


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
                                            '/zed_right/zed_node/rgb/color/rect/image'])
        self.declare_parameter('task_name', 
                                            "pick_place")
        self.declare_parameter('demo_path', 
                                            "/dataset/pick_place/human_rgb_pick_place")
        
        

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
        
        # 1. Initialize the AI controller
        self.get_logger().info(f'Initializing AI Controller: {self.ai_controller_target}')
        if self.ai_controller_target == 'cod_controller':
            from ai_controller.models.cod_controller.cod_controller import CODController
            self.controller = CODController(self.model_config_path, self.task_name)
        elif self.ai_controller_target == 'openvla_controller':
            from ai_controller.models.openvla_controller.openvla_controller import OpenVLAController
            self.controller = OpenVLAController(self.model_config_path, self.task_name)
        else:
            self.get_logger().error(f'Unknown AI Controller target: {self.ai_controller_target}')
            raise ValueError(f'Unknown AI Controller target: {self.ai_controller_target}')
        
        
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
        self.synced_images_event.clear()
        start = self.get_clock().now()
        while not self.synced_images_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().error('Timed out waiting for synchronized camera images.')
                return None
        return self.latest_synced_images
        
    def control_loop(self):
        """Main control loop for the AI controller."""
        
        self.get_logger().info('Starting control loop...')
        # create a directory to save the images
        save_path = f'/home/ros2_ws/src/ai_controller/saved_images/task_{self.task_name}'
        os.makedirs(save_path, exist_ok=True)
        
        while rclpy.ok():
        
            input("Press Enter to start the control loop. Make sure the robot is in a safe position.")
            
            enter_task_id = input("Enter task ID (e.g., 1, 2, 3): ")
            self.get_logger().info(f'Starting control loop for task ID: {enter_task_id}')
            # make task_id like XX
            enter_task_id = enter_task_id.zfill(2)
            
            for step in range(self.max_step):
                
                if step == 0:
                    self.get_logger().info(f'Setting robot to home position for task ID: {enter_task_id}')
                    # call service to set robot to home position
                    # wait for the service to complete
                    future = self.set_home_client.call_async(GoHome.Request())
                    rclpy.spin_until_future_complete(self, future)
                    if future.result() is not None:
                        self.get_logger().info(f'Robot set to home position for task ID: {enter_task_id}')
                    else:
                        self.get_logger().error(f'Service call failed for setting robot to home position: {future.exception()}')
                        return                
                    
                    # load the demo data for the given task_id
                    self.get_logger().info(f'Loading demo data for task ID: {enter_task_id}')
                    self.controller.load_demo_dataset(self.demo_path, enter_task_id)
                
            
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

                # 2. Get joint-states or other relevant robot states (if needed for inference)
                states = None
                
                # 3. Perform inference using the AI controller
                out = self.controller.inference(
                                                    input_data=[images, states],
                                                   t=step,
                                                   save_path=f'{save_path}/step_{step}')
                if len(out) > 1:
                    action, predicted_bb, target_obj_prediction = out
                else:
                    action = out[0]                
                self.get_logger().info(f'Computed Action at step {step}: {action}')
                
                # 5. Send commands to the robot (e.g., set pose, control gripper)
                # call service to set robot to the desired pose
                self.get_logger().info(f'Setting robot to desired pose at step {step}')
                pose_request = GoToPose.Request()
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
                gripper_goal.command.max_effort = 50.0
                future = self.gripper_action_client.send_goal_async(gripper_goal)
                rclpy.spin_until_future_complete(self, future)
                if future.result() is not None:
                    self.get_logger().info(f'Gripper command sent at step {step}')
                else:
                    self.get_logger().error(f'Failed to send gripper command: {future.exception()}')
                    raise RuntimeError(f'Failed to send gripper command: {future.exception()}')
                
                
        

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