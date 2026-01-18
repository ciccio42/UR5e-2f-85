import rclpy
from rclpy.node import Node
from ur5e_2f_85_teleoperation.teleoperator_node import Teleoperator


def main(args=None):
    rclpy.init(args=args)

    teleoperator = Teleoperator()

    rclpy.spin(teleoperator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    teleoperator.destroy_node()
    rclpy.shutdown()

    


if __name__ == '__main__':
    main()