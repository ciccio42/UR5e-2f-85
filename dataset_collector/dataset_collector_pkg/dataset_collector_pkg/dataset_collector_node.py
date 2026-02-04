import rclpy
from rclpy.node import Node
from dataset_collector_pkg.dataset_collector import DatasetCollector

def main(args=None):
    rclpy.init(args=args)
    
    dataset_collector = DatasetCollector()

    rclpy.spin(dataset_collector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dataset_collector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
