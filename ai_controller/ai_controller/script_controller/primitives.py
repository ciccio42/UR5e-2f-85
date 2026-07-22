"""Scripted pick-place motion primitives: reach, approaching, pick, lift_up,
moving, placing.

Every primitive takes the current gripper (tcp_link) pose and a target pose
(the clicked target-object pose for reach/approaching/pick/lift_up, the
clicked target-placing pose for moving/placing) and drives the robot there
through linearly-interpolated micro-steps of at least ``min_step`` meters,
calling ``on_step(position, orientation, event, gripper_command)`` after each
executed waypoint/gripper action so the caller can record it into a rollout,
and ``on_plan(label, start_pos, waypoints)`` once per primitive with the full
computed waypoint list *before* execution starts, so the caller can visualize
the whole intended path (e.g. as RViz markers) independently of whether the
waypoints are actually sent to the robot (``move_robot``).
"""
import numpy as np
import rclpy
from control_msgs.action import GripperCommand
from moveit_controller_srvs.srv import GoToPose

from ai_controller.script_controller.geometry_utils import build_linear_waypoints


class MotionPrimitives:

    def __init__(self, node, set_pose_client, gripper_action_client, frame_id,
                 min_step, reach_hover_height, approach_z_offset, release_height_offset, lift_height,
                 gripper_open_position, gripper_closed_position, gripper_max_effort,
                 move_robot=True):
        self.node = node
        self.set_pose_client = set_pose_client
        self.gripper_action_client = gripper_action_client
        self.frame_id = frame_id
        self.min_step = min_step
        self.reach_hover_height = reach_hover_height
        self.approach_z_offset = approach_z_offset
        self.release_height_offset = release_height_offset
        self.lift_height = lift_height
        self.gripper_open_position = gripper_open_position
        self.gripper_closed_position = gripper_closed_position
        self.gripper_max_effort = gripper_max_effort
        self.move_robot = move_robot
        self.last_gripper_command = gripper_open_position

    # -- low level -----------------------------------------------------

    def _send_pose(self, pos, quat):
        request = GoToPose.Request()
        request.pose.header.stamp = self.node.get_clock().now().to_msg()
        request.pose.header.frame_id = self.frame_id
        request.pose.pose.position.x = float(pos[0])
        request.pose.pose.position.y = float(pos[1])
        request.pose.pose.position.z = float(pos[2])
        request.pose.pose.orientation.x = float(quat[0])
        request.pose.pose.orientation.y = float(quat[1])
        request.pose.pose.orientation.z = float(quat[2])
        request.pose.pose.orientation.w = float(quat[3])

        if not self.move_robot:
            self.node.get_logger().info(f'[DRY RUN] would move TCP to pos={pos}, quat={quat}')
            return

        future = self.set_pose_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        if response is None or not response.success:
            message = getattr(response, 'message', 'no response from set_pose_client')
            raise RuntimeError(f'GoToPose service call failed: {message}')

    def _move_linear(self, current_pose, target_pos, target_quat, on_step=None,
                      on_plan=None, label=None):
        waypoints = build_linear_waypoints(
            current_pose.position, current_pose.orientation,
            target_pos, target_quat, self.min_step)

        if on_plan is not None:
            on_plan(label, current_pose.position, waypoints)

        last_pose = current_pose
        for wp in waypoints:
            self._send_pose(wp.position, wp.orientation)
            last_pose = wp
            if on_step is not None:
                on_step(wp.position, wp.orientation, 'move', self.last_gripper_command)
        return last_pose

    def _set_gripper(self, position, current_pose, on_step=None):
        self.last_gripper_command = position

        if not self.move_robot:
            self.node.get_logger().info(f'[DRY RUN] would set gripper to {position}')
        else:
            goal = GripperCommand.Goal()
            goal.command.position = float(position)
            goal.command.max_effort = self.gripper_max_effort

            future = self.gripper_action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, future)
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError('Gripper goal was rejected or the action call failed.')

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future)

        if on_step is not None:
            on_step(current_pose.position, current_pose.orientation, 'gripper', self.last_gripper_command)

    # -- primitives ------------------------------------------------------
    # Each primitive's docstring states the primitive's semantics as specified;
    # the implementation follows from ``target_pose``'s meaning per primitive.

    def reach(self, current_pose, target_object_pose, on_step=None, on_plan=None):
        """Move above the target object: same (x, y), hover height z."""
        target_pos = np.array([
            target_object_pose.position[0],
            target_object_pose.position[1],
            target_object_pose.position[2] + self.reach_hover_height,
        ])
        return self._move_linear(current_pose, target_pos, target_object_pose.orientation,
                                  on_step, on_plan, label='reach')

    def approaching(self, current_pose, target_object_pose, on_step=None, on_plan=None):
        """Descend the TCP to the same height as the clicked target object."""
        target_pos = np.array([
            target_object_pose.position[0],
            target_object_pose.position[1],
            target_object_pose.position[2] + self.approach_z_offset,
        ])
        return self._move_linear(current_pose, target_pos, target_object_pose.orientation,
                                  on_step, on_plan, label='approaching')

    def pick(self, current_pose, target_object_pose, on_step=None, on_plan=None):
        """Close the gripper on the target object (no TCP motion)."""
        self._set_gripper(self.gripper_closed_position, current_pose, on_step)
        return current_pose

    def lift_up(self, current_pose, target_object_pose, on_step=None, on_plan=None):
        """Increase the gripper height, keeping (x, y) fixed."""
        target_pos = np.array([
            current_pose.position[0],
            current_pose.position[1],
            current_pose.position[2] + self.lift_height,
        ])
        return self._move_linear(current_pose, target_pos, target_object_pose.orientation,
                                  on_step, on_plan, label='lift_up')

    def moving(self, current_pose, target_bin_pose, on_step=None, on_plan=None):
        """Move toward the target bin, keeping the current (post-lift) height."""
        target_pos = np.array([
            target_bin_pose.position[0],
            target_bin_pose.position[1],
            current_pose.position[2],
        ])
        return self._move_linear(current_pose, target_pos, target_bin_pose.orientation,
                                  on_step, on_plan, label='moving')

    def placing(self, current_pose, target_bin_pose, on_step=None, on_plan=None):
        """Descend into the bin and open the gripper to release the object."""
        target_pos = np.array([
            target_bin_pose.position[0],
            target_bin_pose.position[1],
            target_bin_pose.position[2] + self.release_height_offset,
        ])
        new_pose = self._move_linear(current_pose, target_pos, target_bin_pose.orientation,
                                      on_step, on_plan, label='placing')
        self._set_gripper(self.gripper_open_position, new_pose, on_step)
        return new_pose
