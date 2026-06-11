import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient


class AIControllerNode(Node):
    
    def __init__(self):
        super().__init__('ai_controller_node')
        self.get_logger().info('AI Controller Node has been started.')
        
        # define parameters
        self.declare_parameter('ai_controller_target', 
                                                    'cod_controller')
        self.declare_parameter('model_config_path', 
                                                '/home/ros2_ws/src/ai_controller/checkpoint_folder/1Task-pick_place-Simulated-Agent-Human-Demonstration-UR5e-Agent-MOSAIC-COD-SKIP-0-5-10-15-Batch24/config.yaml')
        self.declare_parameter('frame_id', 
                                        'base_link')
        self.declare_parameter('set_home_service', 
                                                'set_robot_to_home')
        self.declare_parameter('set_pose_service', 
                                                'set_robot_to_pose')
        self.declare_parameter('gripper_action_topic', 
                                                    '/robotiq_gripper_controller/gripper_cmd')
        self.declare_parameter('camera_topic', 
                                            ['zed_front/rgb/color/rect/image', 
                                            'zed_left/rgb/color/rect/image',
                                            'zed_right/rgb/color/rect/image'])
        

        # get parameters
        self.ai_controller_target = self.get_parameter('ai_controller_target').get_parameter_value().string_value
        self.model_config_path = self.get_parameter('model_config_path').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.set_home_service = self.get_parameter('set_home_service').get_parameter_value().string_value
        self.set_pose_service = self.get_parameter('set_pose_service').get_parameter_value().string_value
        self.gripper_action_topic = self.get_parameter('gripper_action_topic').get_parameter_value().string_value
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_array_value
        
        
        # 1. Initialize the AI controller
        self.get_logger().info(f'Initializing AI Controller: {self.ai_controller_target}')
        if self.ai_controller_target == 'cod_controller':
            from ai_controller.models.cod_controller.cod_controller import CODController
            self.controller = CODController(self.model_config_path)
        else:
            self.get_logger().error(f'Unknown AI Controller target: {self.ai_controller_target}')
            raise ValueError(f'Unknown AI Controller target: {self.ai_controller_target}')
        
        
        # 2. Set up ROS2 interfaces (publishers, subscribers, services)
        self.get_logger().info(f'Waiting for service {self.set_home_service}...')
        self.set_home_client = self.create_client(SetHome, self.set_home_service)
        self.set_home_client.wait_for_service()
        self.get_logger().info(f'Service {self.set_home_service} is available.')
        
        self.get_logger().info(f'Waiting for service {self.set_pose_service}...')
        self.set_pose_client = self.create_client(SetPose, self.set_pose_service)
        self.set_pose_client.wait_for_service()
        self.get_logger().info(f'Service {self.set_pose_service} is available.')
        
        self.get_logger().info(f"Creating publisher for gripper action on topic {self.gripper_action_topic}...")
        self.gripper_action_client = ActionClient(
            self,
            GripperCommand,
            self.gripper_action_topic,
        )
        self.get_logger().info(f"Publisher for gripper action created on topic {self.gripper_action_topic}.")
        
        
    def control_loop(self):
        """Main control loop for the AI controller."""
        # 1. Get sensor data (e.g., camera images)
        # 2. Pre-process the sensor data
        # 3. Perform inference using the AI controller
        # 4. Post-process the output from the AI controller
        # 5. Send commands to the robot (e.g., set pose, control gripper)
        pass
        

def main(args=None):
    rclpy.init()
    node = AIControllerNode()
    executor = MultiThreadedExecutor(num_threads=1)
    executor.add_node(node)
    
    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
        node.get_logger().info('Shutting down AIControllerNode\n')
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    node.destroy_node()
    rclpy.shutdown()