from pathlib import Path

import numpy as np
import rclpy
import yaml

from cv_bridge import CvBridge
from PIL import Image
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image as RosImage


class MockScenePublisher(Node):

    def __init__(self):
        super().__init__('mock_scene_publisher')

        self.declare_parameter(
            'scene_capture_path',
            '/scene_capture',
        )

        scene_capture_path = Path(
            self.get_parameter('scene_capture_path')
            .get_parameter_value()
            .string_value
        )

        rgb_path = scene_capture_path / 'rgb.png'
        depth_path = scene_capture_path / 'depth.npy'
        camera_info_path = scene_capture_path / 'camera_info.yaml'

        if not rgb_path.is_file():
            raise FileNotFoundError(rgb_path)

        if not depth_path.is_file():
            raise FileNotFoundError(depth_path)

        if not camera_info_path.is_file():
            raise FileNotFoundError(camera_info_path)

        # PIL mantiene direttamente l'ordine RGB.
        self.rgb = np.asarray(
            Image.open(rgb_path).convert('RGB'),
            dtype=np.uint8,
        )

        self.depth = np.load(depth_path).astype(
            np.float32,
            copy=False,
        )

        with camera_info_path.open(
            'r',
            encoding='utf-8',
        ) as stream:
            self.camera_info_data = yaml.safe_load(stream)

        self.bridge = CvBridge()

        self.rgb_pub = self.create_publisher(
            RosImage,
            '/zed_front/zed_node/rgb/color/rect/image',
            qos_profile_sensor_data,
        )

        self.depth_pub = self.create_publisher(
            RosImage,
            '/zed_front/zed_node/depth/depth_registered',
            qos_profile_sensor_data,
        )

        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/zed_front/zed_node/rgb/color/rect/camera_info',
            qos_profile_sensor_data,
        )

        self.timer = self.create_timer(
            0.2,
            self.publish_scene,
        )

        self.get_logger().info(
            f'Loaded mock RGB: shape={self.rgb.shape}, '
            f'dtype={self.rgb.dtype}'
        )
        self.get_logger().info(
            f'Loaded mock depth: shape={self.depth.shape}, '
            f'dtype={self.depth.dtype}'
        )
        self.get_logger().info(
            f'Publishing static scene from {scene_capture_path}'
        )

    def publish_scene(self):
        stamp = self.get_clock().now().to_msg()

        frame_id = self.camera_info_data.get(
            'frame_id',
            'zed_front_left_camera_frame_optical',
        )

        rgb_msg = self.bridge.cv2_to_imgmsg(
            self.rgb,
            encoding='rgb8',
        )

        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = frame_id

        depth_msg = self.bridge.cv2_to_imgmsg(
            self.depth,
            encoding='32FC1',
        )

        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = frame_id

        camera_info_msg = CameraInfo()

        camera_info_msg.header.stamp = stamp
        camera_info_msg.header.frame_id = frame_id

        camera_info_msg.width = int(
            self.camera_info_data['width']
        )
        camera_info_msg.height = int(
            self.camera_info_data['height']
        )

        camera_info_msg.distortion_model = (
            self.camera_info_data['distortion_model']
        )

        camera_info_msg.d = [
            float(v)
            for v in self.camera_info_data['d']
        ]

        camera_info_msg.k = [
            float(v)
            for v in self.camera_info_data['k']
        ]

        camera_info_msg.r = [
            float(v)
            for v in self.camera_info_data['r']
        ]

        camera_info_msg.p = [
            float(v)
            for v in self.camera_info_data['p']
        ]

        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.camera_info_pub.publish(camera_info_msg)


def main(args=None):
    rclpy.init(args=args)

    node = MockScenePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()