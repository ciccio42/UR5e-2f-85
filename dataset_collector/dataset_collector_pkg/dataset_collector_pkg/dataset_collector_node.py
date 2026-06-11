import rclpy
from rclpy.node import Node
from dataset_collector_pkg.dataset_collector import DatasetCollector
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


def main(args=None):
    # rclpy.init(args=args)
    
    # dataset_collector = DatasetCollector()

    # rclpy.spin(dataset_collector)

    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # dataset_collector.destroy_node()
    # rclpy.shutdown()

    rclpy.init()
    node = DatasetCollector()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
