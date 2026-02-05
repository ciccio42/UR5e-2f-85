
import rclpy
from moveit_controller.moveit_controller import MoveItControllerNode
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


def main(args=None):
    # rclpy.init(args=args)
    
    # moveit_controller_node = MoveItControllerNode()

    # # rclpy.spin(moveit_controller_node)

    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # moveit_controller_node.destroy_node()
    # rclpy.shutdown()

    rclpy.init()
    node = MoveItControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Add 

    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
        node.get_logger().info('Shutting down moveit_controller_node.\n')
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()