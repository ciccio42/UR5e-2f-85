import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from controller_manager_msgs.srv import LoadController, SwitchController, ListControllers
from rclpy.duration import Duration
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from ur5e_2f_85_teleoperation_msg.msg import TrajectoryState
from ur5e_2f_85_teleoperation.utils import *
from sensor_msgs.msg import JoyFeedback
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

        # Create Service Client to Change Command Type
        self.servo_command_type_client = self.create_client(ServoCommandType,
                                                            '/servo_node/switch_command_type')
        while not self.servo_command_type_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /servo_node/switch_command_type not available, waiting again...')

        request = ServoCommandType.Request()
        request.command_type = ServoCommandType.Request.TWIST
        future = self.servo_command_type_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Successfully changed servo command type to TWIST')
            else:
                self.get_logger().error('Failed to change servo command type')

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
        self._teleoperator_state = INACTIVE
        self._previous_teleoperator_activation_button_state = 0
        self._previous_trajectory_state_button_state = 0

        # Create a timer to periodically status
        self.telop_state = self.create_publisher(
            TrajectoryState,
            '/teleop_trajectory_state',
            10)
        self._trajectory_state_msg = TrajectoryState()
        self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_IDLE
        self._previous_trajectory_state_button_state = 0
        self._previous_trj_state = TrajectoryState.TRAJECTORY_IDLE
        self._rumble_counter = 0

        self._hid_path = find_dualshock_hidraw()

        # create publisher for rumble feedback
        self._joy_feedback_publisher = self.create_publisher(
            JoyFeedback,
            '/joy/set_feedback',
            10)
        

        self.get_logger().info(f"Teleoperator Init ok.Parameters: \
                                \nStopped Controller Name: {self.controller_to_stop},\
                                \nRunning Controller Name: {self.controller_to_run},\
                                \ntopic_name: {self.topic_name},\
                                \nlinear_scale: {self.linear_scale},\
                                \nangular_scale: {self.angular_scale} \
                                \nHID Path: {self._hid_path}")
        

    def load_and_start_controller(self):
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
            if controller.name == self.controller_to_run and controller.state == 'active':
                self.get_logger().info(f'Controller {self.controller_to_run} is already running.')
                return

        # Load the controller to run
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
        if future_switch.result() is not None:
            if future_switch.result():
                self.get_logger().info(f'Switched controllers: stopped {self.controller_to_stop}, started {self.controller_to_run}')
            else:
                self.get_logger().error('Failed to switch controllers')
        else:
            self.get_logger().error('Failed to switch controllers')

    def joy_callback(self, msg: Joy):

        command_msg = TwistStamped()
        command_msg.header.stamp = self.get_clock().now().to_msg()
        command_msg.header.frame_id = 'base_link'

        # =========================
        # Linear velocity (m/s)
        # =========================

        # Left joystick
        command_msg.twist.linear.x = msg.axes[0] * self.linear_scale
        command_msg.twist.linear.y = -msg.axes[1] * self.linear_scale  # keep your sign convention

        # Triggers for Z motion
        # R2 → +Z, L2 → -Z
        z_vel = 0.0
        # Triggers usually report 1.0 (released) → -1.0 (pressed)
        l2_raw = msg.axes[2]
        r2_raw = msg.axes[5]

        # mainly at the beginning
        if l2_raw == 0.0:
            l2_raw = 1.0
        if r2_raw == 0.0:
            r2_raw = 1.0

        # Normalize to [0.0, 1.0]
        l2 = (1.0 - l2_raw) * 0.5
        r2 = (1.0 - r2_raw) * 0.5
        # self.get_logger().info(f"L2: {l2}, R2: {r2}")
        if l2 != 0.0 and r2 == 0.0:
            z_vel = l2 * self.linear_scale
        elif r2 != 0.0 and l2 == 0.0:
            z_vel = -r2 * self.linear_scale

        command_msg.twist.linear.z = z_vel

        # =========================
        # Angular velocity (rad/s)
        # =========================

        # Right joystick
        command_msg.twist.angular.x = msg.axes[4] * self.angular_scale
        command_msg.twist.angular.y = msg.axes[3] * self.angular_scale

        # Buttons for Z rotation
        angular_z = 0.0
        if msg.buttons[4] == 1:
            angular_z = +4.0 * self.angular_scale   # CCW
        elif msg.buttons[5] == 1:
            angular_z = -4.0 * self.angular_scale   # CW

        command_msg.twist.angular.z = angular_z

        # publish rumble feedback
        rumble_msg = JoyFeedback()
        rumble_msg.type = JoyFeedback.TYPE_RUMBLE
        rumble_msg.id = 0
        rumble_msg.intensity = 0.5
        
        # =========================
        # Check the Button to Activate/Deactivate Teleoperation
        # press the cross to activate/inactive teleoperator
        # =========================
        if msg.buttons[0] == 1 and self._previous_teleoperator_activation_button_state == 0 and self._teleoperator_state == INACTIVE:
            self.get_logger().info("---- Teleoperator is active ----")
            self._teleoperator_state = ACTIVE
            self._previous_teleoperator_activation_button_state = 1
            self._joy_feedback_publisher.publish(rumble_msg)
        elif msg.buttons[0] == 1 and self._previous_teleoperator_activation_button_state == 0 and self._teleoperator_state == ACTIVE:
            self.get_logger().info("---- Teleoperator is inactive ----")
            self._teleoperator_state = INACTIVE
            self._previous_teleoperator_activation_button_state = 1
            self._joy_feedback_publisher.publish(rumble_msg)
        elif msg.buttons[0] == 0 and self._previous_teleoperator_activation_button_state == 1:
            self._previous_teleoperator_activation_button_state = 0

        # =========================
        # Check the Button to Change Trajectory State
        # press circle button to change trajectory state
        # =========================
        if msg.buttons[1] == 1 and self._previous_trajectory_state_button_state == 0:
            self._previous_trajectory_state_button_state = 1
            if self._trajectory_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_IDLE:
                self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_START
            elif self._trajectory_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_START:
                self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_APPROACHING
            elif self._trajectory_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_APPROACHING:
                self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_PICKING
            elif self._trajectory_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_PICKING:
                self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_MOVING
            elif self._trajectory_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_MOVING:
                self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_PLACING
            elif self._trajectory_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_PLACING:
                self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_END
            elif self._trajectory_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_END:
                self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_IDLE
        elif msg.buttons[1] == 0 and self._previous_trajectory_state_button_state == 1:
            self._previous_trajectory_state_button_state = 0

        # Publish the trajectory state
        self._trajectory_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.telop_state.publish(self._trajectory_state_msg)


        # =========================
        # Publish to MoveIt Servo
        # Only if the teleoperator is active
        # =========================
        if self._teleoperator_state:
            self.servo_publisher.publish(command_msg)
