
#!/usr/bin/env python3

"""
Simulate one press and release of the joystick Circle button.

Each invocation generates one rising edge on /joy.
The DatasetCollector human mode uses successive rising edges
to start and stop recording.
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class MockJoyCircle(Node):

    def __init__(self, topic: str, button_index: int):
        super().__init__("mock_joy_circle")

        self.button_index = button_index
        self.button_count = max(12, button_index + 1)

        self.publisher = self.create_publisher(
            Joy,
            topic,
            10,
        )

        self.get_logger().info(
            f"Mock joystick ready: topic={topic}, "
            f"button_index={button_index}"
        )

    def wait_for_subscriber(self, timeout: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            if self.publisher.get_subscription_count() > 0:
                return True

            time.sleep(0.1)

        return False

    def publish_state(self, pressed: bool) -> None:
        msg = Joy()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [0.0] * 8
        msg.buttons = [0] * self.button_count

        msg.buttons[self.button_index] = int(pressed)

        self.publisher.publish(msg)

    def hold_state(self, pressed: bool, duration: float) -> None:
        deadline = time.monotonic() + duration

        while time.monotonic() < deadline:
            self.publish_state(pressed)
            time.sleep(0.05)

    def pulse(self) -> None:
        # Ensure the collector first observes a released button.
        self.hold_state(False, 0.4)

        # Simulate pressing Circle.
        self.get_logger().info("CIRCLE PRESSED")
        self.hold_state(True, 0.4)

        # Release the button.
        self.hold_state(False, 0.4)
        self.get_logger().info("CIRCLE RELEASED")


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Simulate one Circle button press on a ROS 2 Joy topic."
    )

    parser.add_argument(
        "--topic",
        default="/joy",
    )

    parser.add_argument(
        "--button-index",
        type=int,
        default=1,
    )

    parsed = parser.parse_args(args)

    if parsed.button_index < 0:
        parser.error("--button-index must be >= 0")

    rclpy.init()

    node = MockJoyCircle(
        topic=parsed.topic,
        button_index=parsed.button_index,
    )

    try:
        if not node.wait_for_subscriber():
            node.get_logger().error(
                "No joystick subscriber detected. "
                "Start dataset_collector first."
            )
            return 1

        node.pulse()
        return 0

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
