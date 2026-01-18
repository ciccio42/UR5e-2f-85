import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from controller_manager_msgs.srv import LoadController, SwitchController
from rclpy.duration import Duration
from geometry_msgs.msg import TwistStamped

class Teleoperator(Node):
    
    def __init__(self):
        super().__init__('teleoperator')

        # Declare Parameters
        self.declare_parameter('controller_to_stop', 'scaled_joint_trajectory_controller')
        self.declare_parameter('controller_to_run', 'forward_position_controller')
        self.declare_parameter('topic_servo', '/delta_twist_cmds')
        self.declare_parameter('topic_name', '/joy')
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)

        # Get Parameters
        self.controller_to_stop = self.get_parameter('controller_to_stop').get_parameter_value().string_value
        self.controller_to_run = self.get_parameter('controller_to_run').get_parameter_value().string_value
        self.topic_servo = self.get_parameter('topic_servo').get_parameter_value().string_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.linear_scale = self.get_parameter('linear_scale').get_parameter_value().double_value
        self.angular_scale = self.get_parameter('angular_scale').get_parameter_value().double_value

        self.load_and_start_controller()

        # Subscribe to joy topic
        self.get_logger().info(f"Subscribing to topic: {self.topic_name}")
        self.subscription = self.create_subscription(
            Joy,
            self.topic_name,
            self.joy_callback,
            10)
        

        # Create a publisher to send commands on the servo topic
        self.servo_publisher = self.create_publisher(
            TwistStamped,
            self.topic_servo,
            10)


        self.get_logger().info(f"Teleoperator Init ok.Parameters: \
                                \nStopped Controller Name: {self.controller_to_stop},\
                                \nRunning Controller Name: {self.controller_to_run},\
                                \ntopic_name: {self.topic_name},\
                                \nlinear_scale: {self.linear_scale},\
                                \nangular_scale: {self.angular_scale}")
        

    def load_and_start_controller(self):
        client_switch = self.create_client(SwitchController, '/controller_manager/switch_controller')
        while not client_switch.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /controller_manager/switch_controller not available, waiting again...')
        
        request_switch = SwitchController.Request()
        request_switch.deactivate_controllers = [str(self.controller_to_stop)]
        request_switch.activate_controllers = [str(self.controller_to_run)]
        request_switch.strictness = SwitchController.Request.STRICT
        request_switch.activate_asap = True

        future_switch = client_switch.call_async(request_switch)
        rclpy.spin_until_future_complete(self, future_switch)
        
        future_switch = client_switch.call_async(request_switch)
        
        rclpy.spin_until_future_complete(self, future_switch)
        if future_switch.result() is not None:
            self.get_logger().info(f'Switched controllers: stopped {self.controller_to_stop}, started {self.controller_to_run}')
        else:
            self.get_logger().error('Failed to switch controllers')

    def joy_callback(self, msg):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        
        # Left joystick controls linear x and y
        linear_x = msg.axes[1] * self.linear_scale
        linear_y = msg.axes[0] * self.linear_scale
        
        
        

        message.twist.linear.x = linear_x
        message.twist.linear.y = linear_y

        # Publish the command
        self.servo_publisher.publish(message)

        self.get_logger().info(f'Linear X: {linear_x}, Linear Y: {linear_y}')