#!/usr/bin/env python3

from pathlib import Path
import argparse

import cv2
import message_filters
import numpy as np
import rclpy
import yaml

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo


class FrontSceneCapture(Node):

    def __init__(
        self,
        output_dir: Path,
        rgb_topic: str,
        depth_topic: str,
        camera_info_topic: str,
    ):
        super().__init__("front_scene_capture")

        self.output_dir = output_dir
        self.output_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.bridge = CvBridge()

        self.camera_info_msg = None
        self.saved = False

        self.get_logger().info(
            f"RGB topic: {rgb_topic}"
        )
        self.get_logger().info(
            f"Depth topic: {depth_topic}"
        )
        self.get_logger().info(
            f"CameraInfo topic: {camera_info_topic}"
        )
        self.get_logger().info(
            f"Output directory: {output_dir}"
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )

        self.rgb_sub = message_filters.Subscriber(
            self,
            Image,
            rgb_topic,
            qos_profile=qos_profile_sensor_data,
        )

        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            depth_topic,
            qos_profile=qos_profile_sensor_data,
        )

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [
                self.rgb_sub,
                self.depth_sub,
            ],
            queue_size=20,
            slop=0.1,
        )

        self.sync.registerCallback(
            self._rgb_depth_callback
        )

    def _camera_info_callback(
        self,
        msg: CameraInfo,
    ):
        self.camera_info_msg = msg

    def _rgb_depth_callback(
        self,
        rgb_msg: Image,
        depth_msg: Image,
    ):
        if self.saved:
            return

        if self.camera_info_msg is None:
            self.get_logger().info(
                "Waiting for CameraInfo..."
            )
            return

        self.get_logger().info(
            "Synchronized RGB + depth received."
        )

        rgb = self.bridge.imgmsg_to_cv2(
            rgb_msg,
            desired_encoding="bgr8",
        )

        depth = self.bridge.imgmsg_to_cv2(
            depth_msg,
            desired_encoding="passthrough",
        )

        rgb = np.asarray(rgb)
        depth = np.asarray(
            depth,
            dtype=np.float32,
        )

        self._save_rgb(rgb)
        self._save_depth(depth)
        self._save_camera_info(
            self.camera_info_msg
        )

        self.saved = True

        self.get_logger().info(
            "Capture completed successfully."
        )

        self.get_logger().info(
            f"RGB: {self.output_dir / 'rgb.png'}"
        )
        self.get_logger().info(
            f"Depth: {self.output_dir / 'depth.npy'}"
        )
        self.get_logger().info(
            f"Depth preview: "
            f"{self.output_dir / 'depth_preview.png'}"
        )
        self.get_logger().info(
            f"CameraInfo: "
            f"{self.output_dir / 'camera_info.yaml'}"
        )

        rclpy.shutdown()

    def _save_rgb(
        self,
        rgb: np.ndarray,
    ):
        output_path = (
            self.output_dir
            / "rgb.png"
        )

        if not cv2.imwrite(
            str(output_path),
            rgb,
        ):
            raise RuntimeError(
                f"Could not save RGB image: "
                f"{output_path}"
            )

    def _save_depth(
        self,
        depth: np.ndarray,
    ):
        np.save(
            self.output_dir / "depth.npy",
            depth,
        )

        valid_mask = (
            np.isfinite(depth)
            & (depth > 0)
        )

        if not np.any(valid_mask):
            raise RuntimeError(
                "Depth image contains no valid values."
            )

        valid_depth = depth[
            valid_mask
        ]

        near = float(
            np.percentile(
                valid_depth,
                2,
            )
        )

        far = float(
            np.percentile(
                valid_depth,
                98,
            )
        )

        if far <= near:
            far = near + 1e-6

        normalized = np.zeros(
            depth.shape,
            dtype=np.float32,
        )

        normalized[valid_mask] = (
            depth[valid_mask] - near
        ) / (
            far - near
        )

        normalized = np.clip(
            normalized,
            0.0,
            1.0,
        )

        # Near objects are brighter.
        preview_gray = (
            (1.0 - normalized)
            * 255.0
        ).astype(np.uint8)

        preview_gray[
            ~valid_mask
        ] = 0

        output_path = (
            self.output_dir
            / "depth_preview.png"
        )

        if not cv2.imwrite(
            str(output_path),
            preview_gray,
        ):
            raise RuntimeError(
                f"Could not save depth preview: "
                f"{output_path}"
            )

        print(
            f"Depth shape: {depth.shape}"
        )
        print(
            f"Depth dtype: {depth.dtype}"
        )
        print(
            f"Valid depth range: "
            f"{valid_depth.min():.6f} - "
            f"{valid_depth.max():.6f}"
        )
        print(
            f"Preview percentile range: "
            f"{near:.6f} - {far:.6f}"
        )

    def _save_camera_info(
        self,
        msg: CameraInfo,
    ):
        camera_info = {
            "height": int(msg.height),
            "width": int(msg.width),
            "distortion_model": (
                msg.distortion_model
            ),
            "d": [
                float(value)
                for value in msg.d
            ],
            "k": [
                float(value)
                for value in msg.k
            ],
            "r": [
                float(value)
                for value in msg.r
            ],
            "p": [
                float(value)
                for value in msg.p
            ],
            "binning_x": int(
                msg.binning_x
            ),
            "binning_y": int(
                msg.binning_y
            ),
            "roi": {
                "x_offset": int(
                    msg.roi.x_offset
                ),
                "y_offset": int(
                    msg.roi.y_offset
                ),
                "height": int(
                    msg.roi.height
                ),
                "width": int(
                    msg.roi.width
                ),
                "do_rectify": bool(
                    msg.roi.do_rectify
                ),
            },
        }

        output_path = (
            self.output_dir
            / "camera_info.yaml"
        )

        with output_path.open(
            "w",
            encoding="utf-8",
        ) as stream:
            yaml.safe_dump(
                camera_info,
                stream,
                sort_keys=False,
            )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--output-dir",
        default="/scene_capture",
    )

    parser.add_argument(
        "--rgb-topic",
        default=(
            "/zed_front/zed_node/"
            "rgb/color/rect/image"
        ),
    )

    parser.add_argument(
        "--depth-topic",
        default=(
            "/zed_front/zed_node/"
            "depth/depth_registered"
        ),
    )

    parser.add_argument(
        "--camera-info-topic",
        default=(
            "/zed_front/zed_node/"
            "rgb/color/rect/camera_info"
        ),
    )

    args = parser.parse_args()

    rclpy.init()

    node = FrontSceneCapture(
        output_dir=Path(
            args.output_dir
        ).expanduser().resolve(),
        rgb_topic=args.rgb_topic,
        depth_topic=args.depth_topic,
        camera_info_topic=(
            args.camera_info_topic
        ),
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

        node.destroy_node()


if __name__ == "__main__":
    main()