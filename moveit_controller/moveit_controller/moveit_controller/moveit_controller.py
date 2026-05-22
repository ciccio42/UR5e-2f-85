import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetPositionIK
from moveit_controller_srvs.srv import GoHome, GoToPose
from controller_manager_msgs.srv import LoadController, SwitchController, ListControllers
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from rclpy.wait_for_message import wait_for_message
from builtin_interfaces.msg import Duration

class MoveItControllerNode(Node):
    def __init__(self):
        super().__init__('moveit_controller_node')

        # Declare parameters
        self.declare_parameter('planning_component', 'arm_tcp')
        self.planning_component = self.get_parameter('planning_component').get_parameter_value().string_value

        self.declare_parameter('home_joint_names',  ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'])
        self.home_joint_names = self.get_parameter('home_joint_names').get_parameter_value().string_array_value

        self.declare_parameter('home_joint_positions', [2.35, -2.35, 1.57, -1.57, -1.57, 0.0])
        self.home_joint_positions = self.get_parameter('home_joint_positions').get_parameter_value().double_array_value

        # Controller names
        self.declare_parameter('controller_to_run', 'scaled_joint_trajectory_controller')
        self.controller_to_run = self.get_parameter('controller_to_run').get_parameter_value().string_value
        self.declare_parameter('controller_to_stop', 'forward_position_controller')
        self.controller_to_stop = self.get_parameter('controller_to_stop').get_parameter_value().string_value

        self.internal_executer = SingleThreadedExecutor()
        self.internal_executer.add_node(self)

        # Service Server to set robot to home position
        self.get_logger().info('Running service server for setting robot to home position...')
        self.go_home_service_callback_group = MutuallyExclusiveCallbackGroup()
        self._home_service = self.create_service(GoHome, 
                                                 'set_robot_to_home', 
                                                 self.handle_set_robot_to_home,
                                                 callback_group=self.go_home_service_callback_group
                                                 )
        
        # Service Server to set robot to a specific pose
        self.get_logger().info('Running service server for setting robot to a specific pose...')
        self.go_to_pose_service_callback_group = MutuallyExclusiveCallbackGroup()
        self._go_to_pose_service = self.create_service(GoToPose, 
                                                 'set_robot_to_pose', 
                                                 self.handle_set_robot_to_pose,
                                                 callback_group=self.go_to_pose_service_callback_group
                                                 )
        
        # check whether the controller is already loaded
        self.client_controller_callback_group = ReentrantCallbackGroup()
        self.client_controller_state = self.create_client(ListControllers,
                                                     '/controller_manager/list_controllers',
                                                     callback_group=self.client_controller_callback_group)
        while not self.client_controller_state.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service /controller_manager/list_controllers not available, waiting again...')


        # Load the controller to run
        self.client_load_callback_group = self.client_controller_callback_group
        self.client_switch = self.create_client(SwitchController, 
                                                '/controller_manager/switch_controller', 
                                                callback_group=self.client_load_callback_group)
        while not self.client_switch.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /controller_manager/switch_controller not available, waiting again...')

        # Create Client for IK service
        self.client_ik_callback_group = ReentrantCallbackGroup()
        self.client_ik = self.create_client(GetPositionIK, 
                                            '/compute_ik', 
                                            callback_group=self.client_ik_callback_group)
        while not self.client_ik.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /compute_ik not available, waiting again...')

        # Create Action Client for MoveGroup action
        self.action_client_callback_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(self, 
                                           MoveGroup, 
                                           '/move_action',
                                           callback_group=self.action_client_callback_group )


        self.get_logger().info('MoveItControllerNode initialized.\nPlanning component: {}\
                               \nHome joint names: {}\
                               \nHome joint positions: {}'.format(self.planning_component,
                                                                  self.home_joint_names,
                                                                  self.home_joint_positions))

        
    def _switch_controllers(self, controller_to_run, controller_to_stop):
        
        client_controller_state_request = ListControllers.Request()
        
        self.get_logger().info(f'Checking if controller {controller_to_run} is already running...')
        future_controller_state = self.client_controller_state.call_async(client_controller_state_request)
        # rclpy.spin_until_future_complete(self, future_controller_state)
        # while not future_controller_state.done():
        #     rclpy.spin_once(self)
        self.internal_executer.spin_until_future_complete(future_controller_state)
        controllers = future_controller_state.result().controller
        for controller in controllers:
            # self.get_logger().info(f'Found controller: {controller.name} with state {controller.state}')
            if controller.name == controller_to_run and controller.state == 'active':
                self.get_logger().info(f'Controller {controller_to_run} is already running.')
                return True  # No need to load and start
        
        
        self.get_logger().info(f'Loading controller {controller_to_run}...')
        request_switch = SwitchController.Request()
        request_switch.deactivate_controllers = [str(controller_to_stop)]
        request_switch.activate_controllers = [str(controller_to_run)]
        request_switch.strictness = SwitchController.Request.STRICT
        request_switch.activate_asap = True

        future_switch = self.client_switch.call_async(request_switch) 
        #rclpy.spin_until_future_complete(self, future_switch)
        # while not future_switch.done():
        #     rclpy.spin_once(self)
        self.internal_executer.spin_until_future_complete(future_switch)
        if future_switch.result().ok:
            self.get_logger().info(f'Switched controllers: stopped {controller_to_stop}, started {controller_to_run}')
            return True
        else:
            self.get_logger().error('Failed to switch controllers')
            return False
    
    def send_moveit_goal_with_joint_positions(self, joint_positions):
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('MoveGroup action server not available!')
            return False
        
        # Switch to the appropriate controller for moving to joint positions
        self.get_logger().info('Switching controllers to prepare for moving to specified joint positions...')
        if not self._switch_controllers(controller_to_run=self.controller_to_run, 
                                       controller_to_stop=self.controller_to_stop):
            self.get_logger().error('Could not start the required controller to move to specified joint positions.')
            return False
        
        # Create and send goal to move to specified joint positions
        goal = MoveGroup.Goal()
        goal.request.group_name = self.planning_component
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 30.0
        goal.request.max_velocity_scaling_factor = 0.25
        goal.request.max_acceleration_scaling_factor = 0.25

        c = Constraints()

        for joint_name, joint_value in zip(self.home_joint_names, joint_positions):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = joint_value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        
        goal.request.goal_constraints.append(c)
        self.get_logger().info('Sending goal to move to specified joint positions...')

        send_goal_future = self._action_client.send_goal_async(goal)
        
        self.internal_executer.spin_until_future_complete(send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Move to specified joint positions goal rejected!')
            return False
        self.get_logger().info('Move to specified joint positions goal accepted.')

        result_future = goal_handle.get_result_async()
        
        self.internal_executer.spin_until_future_complete(result_future)
        result = result_future.result().result

        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info('Robot moved to specified joint positions successfully.')
            return True
        else:
            self.get_logger().error(
                f'Failed to move robot. Error code: {result.error_code.val}'
            )
            return False

    def set_robot_to_home_position(self):
        self.get_logger().info('Setting robot to home position')
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('MoveGroup action server not available!')
            return False
        
        return self.send_moveit_goal_with_joint_positions(self.home_joint_positions)
    
    def get_current_joint_positions(self):
        self.get_logger().info('Waiting for joint_states message for IK seed...')

        try:
            msg = wait_for_message(
                JointState,
                self,
                '/joint_states',
                timeout_sec=3.0
            )
        except TimeoutError:
            self.get_logger().error('Timeout while waiting for joint_states.')
            return None

        name_to_index = {name: i for i, name in enumerate(msg.name)}

        try:
            joint_positions = [
                msg.position[name_to_index[joint]]
                for joint in self.home_joint_names
            ]
        except KeyError:
            self.get_logger().error('Required joints not found in joint_states.')
            return None

        self.get_logger().info(f'Joint positions received: {joint_positions}')
        return joint_positions
        
    def send_moveit_goal(self, pose):
        self.get_logger().info('Sending MoveIt goal to set robot pose {}...'.format(pose))

        # compute IK for the desired pose and send the goal to MoveIt
        request_ik = GetPositionIK.Request()
        request_ik.ik_request.group_name = self.planning_component
        
        # Set the pose in the IK request
        request_ik.ik_request.pose_stamped.header = self.get_clock().now().to_msg()
        request_ik.ik_request.pose_stamped.pose.position.x = pose.position.x
        request_ik.ik_request.pose_stamped.pose.position.y = pose.position.y
        request_ik.ik_request.pose_stamped.pose.position.z = pose.position.z
        request_ik.ik_request.pose_stamped.pose.orientation.x = pose.orientation.x
        request_ik.ik_request.pose_stamped.pose.orientation.y = pose.orientation.y
        request_ik.ik_request.pose_stamped.pose.orientation.z = pose.orientation.z
        request_ik.ik_request.pose_stamped.pose.orientation.w = pose.orientation.w

        # Set the IK seed
        request_ik.ik_request.robot_state.joint_state.name = self.home_joint_names
        # get the current joint positions to use as seed for IK
        current_joint_positions = self.get_current_joint_positions()
        request_ik.ik_request.robot_state.joint_state.position = current_joint_positions
        
        
        future_ik = self.client_ik.call_async(request_ik)
        self.internal_executer.spin_until_future_complete(future_ik)
        if future_ik.result() is None:
            self.get_logger().error('Failed to call IK service')
            return False
        if future_ik.result().error_code.val != future_ik.result().error_code.SUCCESS:
            self.get_logger().error(f'IK service failed to find a solution. Error code: {future_ik.result().error_code.val}')
            return False
        joint_positions = future_ik.result().solution.joint_state.position
        self.get_logger().info(f'IK solution found: {joint_positions}')
        
        return self.send_moveit_goal_with_joint_positions(joint_positions)

    def handle_set_robot_to_home(self, request, response):
        self.get_logger().info('Received request to set robot to home position.')
        success = self.set_robot_to_home_position()
        if success:
            response.success = True
            response.message = 'Robot moved to home position successfully.'
        else:
            response.success = False
            response.message = 'Failed to move robot to home position.'
        
        # wait a bit to ensure the robot has stopped moving before switching controllers back
        # self.get_logger().info('Waiting for robot to finish moving...')
        # self.get_clock().sleep_for(Duration(seconds=2.0))

        # reset controllers to original state
        self.get_logger().info('Resetting controllers to original state...')
        if self._switch_controllers(controller_to_run=self.controller_to_stop,
                                    controller_to_stop=self.controller_to_run):
            self.get_logger().info('Controllers reset successfully.')
        else:
            self.get_logger().error('Failed to reset controllers to original state.')
        
        return response
    
    def handle_set_robot_to_pose(self, request, response):
        self.get_logger().info('Received request to set robot pose.')
        # Extract pose from request
        pose = request.pose
        # Call send_moveit_goal with the pose
        success = self.send_moveit_goal(pose)
        if success:
            response.success = True
            response.message = 'Robot pose set successfully.'
        else:
            response.success = False
            response.message = 'Failed to set robot pose.'
        
        # reset controllers to original state
        self.get_logger().info('Resetting controllers to original state...')
        if self._switch_controllers(controller_to_run=self.controller_to_stop,
                                    controller_to_stop=self.controller_to_run):
            self.get_logger().info('Controllers reset successfully.')
        else:
            self.get_logger().error('Failed to reset controllers to original state.')

        return response
    
