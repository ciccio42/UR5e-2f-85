import rclpy
import threading
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, DisplayTrajectory, JointConstraint
from moveit_msgs.srv import GetPositionIK
from moveit_controller_srvs.srv import GoHome, GoToPose
from controller_manager_msgs.srv import LoadController, SwitchController, ListControllers
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from sensor_msgs.msg import JointState


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

        self.declare_parameter('moveit_result_timeout_sec', 120.0)
        self.moveit_result_timeout_sec = self.get_parameter('moveit_result_timeout_sec').value
        self.declare_parameter('joint_goal_tolerance', 0.03)
        self.joint_goal_tolerance = self.get_parameter('joint_goal_tolerance').value
        self.declare_parameter('manual_goal_completion', False)
        self.manual_goal_completion = self.get_parameter('manual_goal_completion').value

        # When False: plan every requested motion, publish it to /display_planned_path
        # for RViz (MotionPlanning display -> Planned Path), and return without ever
        # calling ExecuteTrajectory or touching the controller_manager. Lets you preview
        # a full pick-place run (e.g. via script_controller_node) against fake/real
        # hardware without the robot actually moving.
        self.declare_parameter('execute_trajectory', True)
        self.execute_trajectory = self.get_parameter('execute_trajectory').value
        if not self.execute_trajectory:
            self.get_logger().warning(
                'execute_trajectory:=False - PLAN-ONLY MODE. Planned trajectories will be '
                'published to /display_planned_path for RViz but NOT executed on the robot.')

        self._latest_joint_state = None
        self._joint_state_event = threading.Event()
        self.joint_state_callback_group = ReentrantCallbackGroup()
        self._joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_state,
            10,
            callback_group=self.joint_state_callback_group,
        )

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
        self._execute_trajectory_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory',
            callback_group=self.action_client_callback_group,
        )
        self._display_trajectory_publisher = self.create_publisher(
            DisplayTrajectory,
            '/display_planned_path',
            10,
        )


        self.get_logger().info('MoveItControllerNode initialized.\nPlanning component: {}\
                               \nHome joint names: {}\
                               \nHome joint positions: {}'.format(self.planning_component,
                                                                  self.home_joint_names,
                                                                  self.home_joint_positions))


    def _wait_for_future(self, future, timeout_sec=None):
        done_event = threading.Event()
        future.add_done_callback(lambda _: done_event.set())
        if not done_event.wait(timeout=timeout_sec):
            return None
        return future.result()

    def _on_joint_state(self, msg):
        self._latest_joint_state = msg
        self._joint_state_event.set()

    def _wait_for_joint_positions(self, joint_positions):
        deadline = self.get_clock().now().nanoseconds + int(self.moveit_result_timeout_sec * 1e9)

        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            self._joint_state_event.wait(timeout=0.2)
            self._joint_state_event.clear()

            msg = self._latest_joint_state
            if msg is None:
                continue

            name_to_index = {name: i for i, name in enumerate(msg.name)}
            try:
                current_positions = [
                    msg.position[name_to_index[joint]]
                    for joint in self.home_joint_names
                ]
            except KeyError:
                self.get_logger().error('Required joints not found in joint_states.')
                return False

            if all(
                abs(current - target) <= self.joint_goal_tolerance
                for current, target in zip(current_positions, joint_positions)
            ):
                self.get_logger().info('Requested joint positions reached.')
                return True

        self.get_logger().error('Timed out while waiting for requested joint positions.')
        return False
        
    def _switch_controllers(self, controller_to_run, controller_to_stop):
        
        client_controller_state_request = ListControllers.Request()
        
        self.get_logger().info(f'Checking if controller {controller_to_run} is already running...')
        future_controller_state = self.client_controller_state.call_async(client_controller_state_request)
        # rclpy.spin_until_future_complete(self, future_controller_state)
        # while not future_controller_state.done():
        #     rclpy.spin_once(self)
        controller_state = self._wait_for_future(future_controller_state, timeout_sec=5.0)
        if controller_state is None:
            self.get_logger().error('Timed out while checking controller state.')
            return False
        controllers = controller_state.controller
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
        switch_result = self._wait_for_future(future_switch, timeout_sec=5.0)
        if switch_result is None:
            self.get_logger().error('Timed out while switching controllers.')
            return False
        if switch_result.ok:
            self.get_logger().info(f'Switched controllers: stopped {controller_to_stop}, started {controller_to_run}')
            return True
        else:
            self.get_logger().error('Failed to switch controllers')
            return False
    
    def send_moveit_goal_with_joint_positions(self, joint_positions):
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('MoveGroup action server not available!')
            return False

        if self.execute_trajectory and not self._execute_trajectory_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('ExecuteTrajectory action server not available!')
            return False

        if self.execute_trajectory:
            self.get_logger().info('Switching controllers to prepare for moving to specified joint positions...')
            if not self._switch_controllers(controller_to_run=self.controller_to_run,
                                           controller_to_stop=self.controller_to_stop):
                self.get_logger().error('Could not start the required controller to move to specified joint positions.')
                return False
        else:
            self.get_logger().info('execute_trajectory:=False - skipping controller switch (plan-only).')

        goal = MoveGroup.Goal()
        goal.request.group_name = self.planning_component
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 30.0
        goal.request.max_velocity_scaling_factor = 0.25
        goal.request.max_acceleration_scaling_factor = 0.25
        goal.planning_options.plan_only = True

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
        self.get_logger().info('Planning trajectory to specified joint positions...')

        send_goal_future = self._action_client.send_goal_async(goal)
        goal_handle = self._wait_for_future(send_goal_future, timeout_sec=10.0)
        if goal_handle is None:
            self.get_logger().error('Timed out while sending MoveIt planning goal.')
            return False
        if not goal_handle.accepted:
            self.get_logger().error('MoveIt planning goal rejected!')
            return False

        result_future = goal_handle.get_result_async()
        result_response = self._wait_for_future(
            result_future,
            timeout_sec=self.moveit_result_timeout_sec,
        )
        if result_response is None:
            self.get_logger().error('Timed out while waiting for MoveIt planning result.')
            return False

        result = result_response.result
        if result.error_code.val != result.error_code.SUCCESS:
            self.get_logger().error(
                f'Failed to plan robot motion. Error code: {result.error_code.val}'
            )
            return False

        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = result.trajectory_start
        display_trajectory.trajectory.append(result.planned_trajectory)
        self._display_trajectory_publisher.publish(display_trajectory)
        self.get_logger().info('Published planned trajectory to /display_planned_path.')

        if not self.execute_trajectory:
            self.get_logger().info(
                'execute_trajectory:=False - plan published for RViz, skipping execution.')
            return True

        # try:
        #     answer = input('Execute this planned trajectory? [y/N]: ').strip().lower()
        # except KeyboardInterrupt:
        #     self.get_logger().warn('MoveIt trajectory execution cancelled by user input.')
        #     return False

        # if answer not in ('y', 'yes'):
        #     self.get_logger().info('MoveIt trajectory execution skipped by user.')
        #     return False

        execute_goal = ExecuteTrajectory.Goal()
        execute_goal.trajectory = result.planned_trajectory
        self.get_logger().info('Executing planned trajectory...')
        execute_future = self._execute_trajectory_client.send_goal_async(execute_goal)
        execute_handle = self._wait_for_future(execute_future, timeout_sec=10.0)
        if execute_handle is None:
            self.get_logger().error('Timed out while sending ExecuteTrajectory goal.')
            return False
        if not execute_handle.accepted:
            self.get_logger().error('ExecuteTrajectory goal rejected!')
            return False

        if self.manual_goal_completion:
            try:
                input('Press Enter after the robot has finished moving...')
            except KeyboardInterrupt:
                self.get_logger().warn('MoveIt goal completion cancelled by user input.')
                cancel_future = execute_handle.cancel_goal_async()
                self._wait_for_future(cancel_future, timeout_sec=5.0)
                return False

            self.get_logger().info('Robot movement confirmed by user.')
            return True

        execute_result_future = execute_handle.get_result_async()
        execute_result_response = self._wait_for_future(
            execute_result_future,
            timeout_sec=self.moveit_result_timeout_sec,
        )
        if execute_result_response is None:
            self.get_logger().error('Timed out while waiting for trajectory execution result.')
            cancel_future = execute_handle.cancel_goal_async()
            self._wait_for_future(cancel_future, timeout_sec=5.0)
            return False

        execute_result = execute_result_response.result
        if execute_result.error_code.val == execute_result.error_code.SUCCESS:
            self.get_logger().info('Robot moved to specified joint positions successfully.')
            return True

        self.get_logger().error(
            f'Failed to execute robot trajectory. Error code: {execute_result.error_code.val}'
        )
        return False

    def set_robot_to_home_position(self):
        self.get_logger().info('Setting robot to home position')
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('MoveGroup action server not available!')
            return False
        
        return self.send_moveit_goal_with_joint_positions(self.home_joint_positions)
    
    def get_current_joint_positions(self):
        self.get_logger().info('Reading latest joint_states message for IK seed...')

        if self._latest_joint_state is None:
            self._joint_state_event.wait(timeout=3.0)
            self._joint_state_event.clear()

        msg = self._latest_joint_state
        if msg is None:
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
        
        # Set the pose in the IK request. The GoToPose service uses PoseStamped.
        if hasattr(pose, 'pose'):
            request_ik.ik_request.pose_stamped = pose
            request_ik.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        else:
            request_ik.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
            request_ik.ik_request.pose_stamped.pose = pose

        self.get_logger().info(
            f'IK target frame: {request_ik.ik_request.pose_stamped.header.frame_id}'
        )
        self.get_logger().info(
            f'IK target pose: {request_ik.ik_request.pose_stamped.pose}'
        )

        # Set the IK seed
        request_ik.ik_request.robot_state.joint_state.name = self.home_joint_names
        # get the current joint positions to use as seed for IK
        current_joint_positions = self.get_current_joint_positions()
        if current_joint_positions is None:
            return False
        request_ik.ik_request.robot_state.joint_state.position = current_joint_positions
        self.get_logger().info(
            f'IK seed joint names: {list(request_ik.ik_request.robot_state.joint_state.name)}'
        )
        self.get_logger().info(
            f'IK seed joint positions: {list(request_ik.ik_request.robot_state.joint_state.position)}'
        )
        
        future_ik = self.client_ik.call_async(request_ik)
        ik_result = self._wait_for_future(future_ik, timeout_sec=10.0)
        if ik_result is None:
            self.get_logger().error('Failed to call IK service')
            return False
        if ik_result.error_code.val != ik_result.error_code.SUCCESS:
            self.get_logger().error(f'IK service failed to find a solution. Error code: {ik_result.error_code.val}')
            return False
        joint_names = ik_result.solution.joint_state.name
        joint_positions = ik_result.solution.joint_state.position
        self.get_logger().info(f'IK solution joint names: {list(joint_names)}')
        self.get_logger().info(f'IK solution joint positions: {list(joint_positions)}')

        solution_by_name = dict(zip(joint_names, joint_positions))
        try:
            arm_joint_positions = [
                solution_by_name[joint_name]
                for joint_name in self.home_joint_names
            ]
        except KeyError as exc:
            self.get_logger().error(f'IK solution is missing required joint: {exc}')
            return False

        self.get_logger().info(f'Ordered arm joint names: {list(self.home_joint_names)}')
        self.get_logger().info(f'Ordered arm joint positions: {arm_joint_positions}')
        
        return self.send_moveit_goal_with_joint_positions(arm_joint_positions)

    def handle_set_robot_to_home(self, request, response):
        self.get_logger().info('Received request to set robot to home position.')
        success = self.set_robot_to_home_position()
        if success:
            response.success = True
            response.message = ('Home trajectory planned (not executed).' if not self.execute_trajectory
                                 else 'Robot moved to home position successfully.')
        else:
            response.success = False
            response.message = 'Failed to move robot to home position.'

        # wait a bit to ensure the robot has stopped moving before switching controllers back
        # self.get_logger().info('Waiting for robot to finish moving...')
        # self.get_clock().sleep_for(Duration(seconds=2.0))

        # reset controllers to original state (only needed if we actually switched to
        # controller_to_run to execute; plan-only mode never touched the controllers)
        if self.execute_trajectory:
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
            response.message = ('Pose trajectory planned (not executed).' if not self.execute_trajectory
                                 else 'Robot pose set successfully.')
        else:
            response.success = False
            response.message = 'Failed to set robot pose.'

        # reset controllers to original state (only needed if we actually switched to
        # controller_to_run to execute; plan-only mode never touched the controllers)
        if self.execute_trajectory:
            self.get_logger().info('Resetting controllers to original state...')
            if self._switch_controllers(controller_to_run=self.controller_to_stop,
                                        controller_to_stop=self.controller_to_run):
                self.get_logger().info('Controllers reset successfully.')
            else:
                self.get_logger().error('Failed to reset controllers to original state.')

        return response
    
