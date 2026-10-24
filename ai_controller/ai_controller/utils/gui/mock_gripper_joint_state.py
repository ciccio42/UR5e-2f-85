#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from sensor_msgs.msg import JointState


SOURCE_TOPIC = "/joint_states"
MOCK_TOPIC = "/seedo_mock_joint_states"

GRIPPER_JOINT = "robotiq_85_left_knuckle_joint"

# Initial open position expected by the current SeeDo motion layer.
MOCK_GRIPPER_POSITION = 0.1


class MockGripperJointState(Node):

    def __init__(self):
        super().__init__(
            "seedo_mock_gripper_joint_state"
        )

        self.publisher = self.create_publisher(
            JointState,
            MOCK_TOPIC,
            10,
        )

        self.subscription = self.create_subscription(
            JointState,
            SOURCE_TOPIC,
            self._joint_state_callback,
            10,
        )

        self._seen_gripper = False

        self.get_logger().warning(
            "SEEDO GRIPPER STATE MOCK ENABLED"
        )

        self.get_logger().warning(
            f"Input:  {SOURCE_TOPIC}"
        )

        self.get_logger().warning(
            f"Output: {MOCK_TOPIC}"
        )

        self.get_logger().warning(
            f"Mocking {GRIPPER_JOINT} "
            f"at position {MOCK_GRIPPER_POSITION:.4f}"
        )


    def _joint_state_callback(
        self,
        msg: JointState,
    ):
        output = JointState()

        # Preserve the real timestamp/frame information.
        output.header = msg.header

        # Copy the complete robot state.
        output.name = list(
            msg.name
        )

        output.position = list(
            msg.position
        )

        output.velocity = list(
            msg.velocity
        )

        output.effort = list(
            msg.effort
        )


        try:
            index = output.name.index(
                GRIPPER_JOINT
            )

        except ValueError:

            if not self._seen_gripper:
                self.get_logger().error(
                    f"Joint {GRIPPER_JOINT!r} "
                    "is not present in /joint_states."
                )

            return


        if index >= len(
            output.position
        ):
            self.get_logger().error(
                f"Joint {GRIPPER_JOINT!r} exists "
                "but has no position value."
            )

            return


        self._seen_gripper = True


        # ---------------------------------------------------------
        # ONLY modification performed by the mock.
        # ---------------------------------------------------------

        output.position[index] = (
            MOCK_GRIPPER_POSITION
        )


        if index < len(
            output.velocity
        ):
            output.velocity[index] = 0.0


        self.publisher.publish(
            output
        )


def main():
    rclpy.init()

    node = MockGripperJointState()

    try:
        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()