#!/usr/bin/env python3

from pathlib import Path

import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


# ================================================================
# CONFIG
# ================================================================

DEPTH_TOPIC = "/zed_gripper/zed_node/depth/depth_registered"

OUTPUT_DIR = Path("/host_debug/depth_test")

MIN_DEPTH = 0.01
MAX_DEPTH = 1.0

# Horizontal crop on ORIGINAL depth image.
CROP_LEFT = 281
CROP_RIGHT = 431  # inclusive

# Same parameters used by _find_depth_object_centroid().
CANNY_LOW = 5
CANNY_HIGH = 15

BLUR_KERNEL = 5

CLOSE_KERNEL_SIZE = 7
CLOSE_ITERATIONS = 1

MIN_CONTOUR_AREA = 200.0

# Margins applied AFTER cropping.
IGNORE_TOP = 70
IGNORE_BOTTOM = 50
IGNORE_LEFT = 0
IGNORE_RIGHT = 0


class DepthCapture(Node):

    def __init__(self):
        super().__init__("depth_capture")

        self.bridge = CvBridge()

        self.depth_m = None
        self.depth_gray = None

        self.debug_images = {}
        self.result = None

        self.subscription = self.create_subscription(
            Image,
            DEPTH_TOPIC,
            self.callback,
            qos_profile_sensor_data,
        )

    # ============================================================
    # ROS DEPTH CALLBACK
    # ============================================================

    def callback(self, msg):

        if self.depth_gray is not None:
            return

        depth = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="passthrough",
        ).astype(np.float32)

        # Keep RAW metric depth.
        self.depth_m = depth.copy()

        # --------------------------------------------------------
        # Full grayscale visualization
        # --------------------------------------------------------

        depth_safe = depth.copy()

        invalid = (
            ~np.isfinite(depth_safe)
            | (depth_safe <= 0.0)
        )

        depth_safe[invalid] = MIN_DEPTH

        depth_clipped = np.clip(
            depth_safe,
            MIN_DEPTH,
            MAX_DEPTH,
        )

        depth_normalized = (
            (depth_clipped - MIN_DEPTH)
            / (MAX_DEPTH - MIN_DEPTH)
        )

        self.depth_gray = (
            depth_normalized * 255.0
        ).astype(np.uint8)

        # Run exactly the processing pipeline.
        self.process_depth()

    # ============================================================
    # DEPTH PIPELINE
    # ============================================================

    def process_depth(self):

        depth_m = np.asarray(
            self.depth_m,
            dtype=np.float64,
        )

        full_height, full_width = depth_m.shape[:2]

        # --------------------------------------------------------
        # CROP
        # --------------------------------------------------------

        crop_left = int(
            np.clip(
                CROP_LEFT,
                0,
                full_width - 1,
            )
        )

        crop_right = int(
            np.clip(
                CROP_RIGHT,
                crop_left,
                full_width - 1,
            )
        )

        depth_crop = depth_m[
            :,
            crop_left:crop_right + 1,
        ]

        height, width = depth_crop.shape[:2]

        print()
        print("==============================================")
        print("DEPTH DEBUG")
        print("==============================================")
        print(
            f"Full image: {full_width} x {full_height}"
        )
        print(
            f"Crop: x={crop_left} .. {crop_right}"
        )
        print(
            f"Crop size: {width} x {height}"
        )

        # --------------------------------------------------------
        # KEEP MASK
        # --------------------------------------------------------

        keep_mask = np.ones(
            (height, width),
            dtype=np.uint8,
        )

        top = max(0, IGNORE_TOP)
        bottom = max(0, IGNORE_BOTTOM)
        left = max(0, IGNORE_LEFT)
        right = max(0, IGNORE_RIGHT)

        if top > 0:
            keep_mask[
                :min(top, height),
                :
            ] = 0

        if bottom > 0:
            keep_mask[
                max(0, height - bottom):,
                :
            ] = 0

        if left > 0:
            keep_mask[
                :,
                :min(left, width)
            ] = 0

        if right > 0:
            keep_mask[
                :,
                max(0, width - right):
            ] = 0

        # --------------------------------------------------------
        # METRIC DEPTH -> GRAYSCALE
        # --------------------------------------------------------

        depth_safe = depth_crop.copy()

        invalid = (
            ~np.isfinite(depth_safe)
            | (depth_safe <= 0.0)
        )

        depth_safe[invalid] = MIN_DEPTH

        depth_clipped = np.clip(
            depth_safe,
            MIN_DEPTH,
            MAX_DEPTH,
        )

        depth_normalized = (
            (depth_clipped - MIN_DEPTH)
            / (MAX_DEPTH - MIN_DEPTH)
        )

        depth_gray_crop = (
            depth_normalized * 255.0
        ).astype(np.uint8)

        # --------------------------------------------------------
        # GAUSSIAN BLUR
        # --------------------------------------------------------

        blur_kernel = max(
            1,
            BLUR_KERNEL,
        )

        if blur_kernel % 2 == 0:
            blur_kernel += 1

        depth_blurred = cv2.GaussianBlur(
            depth_gray_crop,
            (
                blur_kernel,
                blur_kernel,
            ),
            0,
        )

        # --------------------------------------------------------
        # CANNY
        # --------------------------------------------------------

        edges = cv2.Canny(
            depth_blurred,
            CANNY_LOW,
            CANNY_HIGH,
        )

        edges_masked = (
            (edges > 0)
            & (keep_mask > 0)
        ).astype(np.uint8) * 255

        # --------------------------------------------------------
        # MORPHOLOGICAL CLOSING
        # --------------------------------------------------------

        close_kernel_size = max(
            1,
            CLOSE_KERNEL_SIZE,
        )

        if close_kernel_size % 2 == 0:
            close_kernel_size += 1

        close_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (
                close_kernel_size,
                close_kernel_size,
            ),
        )

        edges_closed = cv2.morphologyEx(
            edges_masked,
            cv2.MORPH_CLOSE,
            close_kernel,
            iterations=CLOSE_ITERATIONS,
        )

        edges_closed = (
            (edges_closed > 0)
            & (keep_mask > 0)
        ).astype(np.uint8) * 255

        # --------------------------------------------------------
        # FIND CONTOURS
        # --------------------------------------------------------

        contours, _ = cv2.findContours(
            edges_closed,
            cv2.RETR_LIST,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        print(
            f"Contours found before filtering: {len(contours)}"
        )

        best_contour = None
        best_centroid = None
        best_depth = None
        best_area = None

        valid_contours = []

        for index, contour in enumerate(contours):

            area = float(
                cv2.contourArea(
                    contour
                )
            )

            if area < MIN_CONTOUR_AREA:
                continue

            moments = cv2.moments(
                contour
            )

            if moments["m00"] == 0.0:
                continue

            u = float(
                moments["m10"]
                / moments["m00"]
            )

            v = float(
                moments["m01"]
                / moments["m00"]
            )

            centroid = np.asarray(
                [
                    u,
                    v,
                ],
                dtype=np.float64,
            )

            # ----------------------------------------------------
            # DEPTH INSIDE CONTOUR
            # ----------------------------------------------------

            contour_mask = np.zeros(
                (height, width),
                dtype=np.uint8,
            )

            cv2.drawContours(
                contour_mask,
                [contour],
                contourIdx=-1,
                color=255,
                thickness=cv2.FILLED,
            )

            valid_pixels = (
                (contour_mask > 0)
                & (keep_mask > 0)
                & np.isfinite(depth_crop)
                & (depth_crop > 0.0)
                & (depth_crop >= MIN_DEPTH)
                & (depth_crop <= MAX_DEPTH)
            )

            contour_depth_values = depth_crop[
                valid_pixels
            ]

            if contour_depth_values.size == 0:
                continue

            contour_min_depth = float(
                np.min(
                    contour_depth_values
                )
            )

            valid_contours.append(
                (
                    contour,
                    centroid,
                    contour_min_depth,
                    area,
                )
            )

            print(
                f"Contour {index}: "
                f"area={area:.1f} px²  "
                f"centroid_crop=({u:.1f}, {v:.1f})  "
                f"centroid_full=({crop_left + u:.1f}, {v:.1f})  "
                f"min_depth={contour_min_depth:.4f} m"
            )

            # Closest contour to camera.
            if (
                best_depth is None
                or contour_min_depth < best_depth
            ):
                best_depth = contour_min_depth
                best_contour = contour
                best_centroid = centroid
                best_area = area

        print(
            f"Valid contours: {len(valid_contours)}"
        )

        # --------------------------------------------------------
        # VISUALIZATIONS
        # --------------------------------------------------------

        # Full grayscale image converted to BGR for annotations.
        full_vis = cv2.cvtColor(
            self.depth_gray,
            cv2.COLOR_GRAY2BGR,
        )

        # Show crop boundaries.
        cv2.rectangle(
            full_vis,
            (crop_left, 0),
            (crop_right, full_height - 1),
            (255, 255, 255),
            2,
        )

        # Crop visualization.
        crop_vis = cv2.cvtColor(
            depth_gray_crop,
            cv2.COLOR_GRAY2BGR,
        )

        # Show excluded areas.
        crop_mask_vis = crop_vis.copy()

        excluded = keep_mask == 0

        crop_mask_vis[
            excluded
        ] = (
            crop_mask_vis[
                excluded
            ] * 0.25
        ).astype(np.uint8)

        # --------------------------------------------------------
        # ALL VALID CONTOURS
        # --------------------------------------------------------

        contours_vis = crop_vis.copy()

        for contour, centroid, min_depth, area in valid_contours:

            cv2.drawContours(
                contours_vis,
                [contour],
                -1,
                (255, 255, 255),
                1,
            )

            cu = int(
                round(
                    centroid[0]
                )
            )

            cv = int(
                round(
                    centroid[1]
                )
            )

            cv2.circle(
                contours_vis,
                (cu, cv),
                3,
                (255, 255, 255),
                -1,
            )

            cv2.putText(
                contours_vis,
                f"{min_depth:.3f}m",
                (
                    cu + 5,
                    cv - 5,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        # --------------------------------------------------------
        # SELECTED CONTOUR
        # --------------------------------------------------------

        selected_vis = crop_vis.copy()

        if best_contour is not None:

            cv2.drawContours(
                selected_vis,
                [best_contour],
                -1,
                (255, 255, 255),
                2,
            )

            u_crop = int(
                np.clip(
                    round(best_centroid[0]),
                    0,
                    width - 1,
                )
            )

            v = int(
                np.clip(
                    round(best_centroid[1]),
                    0,
                    height - 1,
                )
            )

            u_full = crop_left + u_crop

            cv2.circle(
                selected_vis,
                (u_crop, v),
                5,
                (255, 255, 255),
                -1,
            )

            cv2.drawMarker(
                selected_vis,
                (u_crop, v),
                (255, 255, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=15,
                thickness=2,
            )

            cv2.putText(
                selected_vis,
                f"depth={best_depth:.4f}m",
                (
                    max(0, u_crop - 50),
                    max(15, v - 10),
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

            # Same selected point on full image.
            cv2.circle(
                full_vis,
                (u_full, v),
                5,
                (255, 255, 255),
                -1,
            )

            cv2.drawMarker(
                full_vis,
                (u_full, v),
                (255, 255, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=20,
                thickness=2,
            )

            print()
            print("----------------------------------------------")
            print("SELECTED CONTOUR")
            print("----------------------------------------------")
            print(
                f"area = {best_area:.1f} px²"
            )
            print(
                f"minimum depth = {best_depth:.4f} m"
            )
            print(
                f"centroid crop = ({u_crop}, {v})"
            )
            print(
                f"centroid original = ({u_full}, {v})"
            )
            print("----------------------------------------------")

            self.result = (
                u_full,
                v,
                best_depth,
            )

        else:

            print()
            print("NO VALID CONTOUR SELECTED")

            self.result = None

        # --------------------------------------------------------
        # STORE DEBUG IMAGES
        # --------------------------------------------------------

        self.debug_images = {
            "01_depth_full": full_vis,
            "02_depth_crop": depth_gray_crop,
            "03_crop_keep_mask": crop_mask_vis,
            "04_blurred": depth_blurred,
            "05_canny": edges,
            "06_canny_masked": edges_masked,
            "07_edges_closed": edges_closed,
            "08_all_contours": contours_vis,
            "09_selected_contour": selected_vis,
        }

    # ============================================================
    # MOUSE CALLBACK
    # ============================================================

    @staticmethod
    def mouse_callback(event, x, y, flags, param):

        node = param

        if node.depth_m is None:
            return

        if event == cv2.EVENT_MOUSEMOVE:

            if (
                y < 0
                or y >= node.depth_m.shape[0]
                or x < 0
                or x >= node.depth_m.shape[1]
            ):
                return

            depth_m = node.depth_m[
                y,
                x,
            ]

            gray_value = node.depth_gray[
                y,
                x,
            ]

            print(
                f"\r"
                f"(x={x:4d}, y={y:4d})  "
                f"depth={depth_m:.4f} m  "
                f"gray={gray_value:3d}",
                end="",
                flush=True,
            )


def main():

    rclpy.init()

    node = DepthCapture()

    try:

        # --------------------------------------------------------
        # WAIT FOR FIRST DEPTH FRAME
        # --------------------------------------------------------

        while (
            rclpy.ok()
            and node.depth_gray is None
        ):
            rclpy.spin_once(
                node,
                timeout_sec=0.1,
            )

        if node.depth_gray is None:
            return

        # --------------------------------------------------------
        # SAVE DEBUG IMAGES
        # --------------------------------------------------------

        OUTPUT_DIR.mkdir(
            parents=True,
            exist_ok=True,
        )

        for name, image in node.debug_images.items():

            path = OUTPUT_DIR / f"{name}.png"

            cv2.imwrite(
                str(path),
                image,
            )

            print(
                f"Saved: {path}"
            )

        # --------------------------------------------------------
        # WINDOWS
        # --------------------------------------------------------

        cv2.namedWindow(
            "Depth Full",
            cv2.WINDOW_NORMAL,
        )

        cv2.namedWindow(
            "Depth Crop",
            cv2.WINDOW_NORMAL,
        )

        cv2.namedWindow(
            "Canny Masked",
            cv2.WINDOW_NORMAL,
        )

        cv2.namedWindow(
            "Edges Closed",
            cv2.WINDOW_NORMAL,
        )

        cv2.namedWindow(
            "All Contours",
            cv2.WINDOW_NORMAL,
        )

        cv2.namedWindow(
            "Selected Contour",
            cv2.WINDOW_NORMAL,
        )

        # Mouse coordinates are relative to the FULL image.
        cv2.setMouseCallback(
            "Depth Full",
            node.mouse_callback,
            node,
        )

        print()
        print("Move mouse over 'Depth Full' to inspect metric depth.")
        print("Press q or ESC to exit.")

        # --------------------------------------------------------
        # DISPLAY LOOP
        # --------------------------------------------------------

        while rclpy.ok():

            cv2.imshow(
                "Depth Full",
                node.debug_images[
                    "01_depth_full"
                ],
            )

            cv2.imshow(
                "Depth Crop",
                node.debug_images[
                    "02_depth_crop"
                ],
            )

            cv2.imshow(
                "Canny Masked",
                node.debug_images[
                    "06_canny_masked"
                ],
            )

            cv2.imshow(
                "Edges Closed",
                node.debug_images[
                    "07_edges_closed"
                ],
            )

            cv2.imshow(
                "All Contours",
                node.debug_images[
                    "08_all_contours"
                ],
            )

            cv2.imshow(
                "Selected Contour",
                node.debug_images[
                    "09_selected_contour"
                ],
            )

            key = cv2.waitKey(50) & 0xFF

            if (
                key == ord("q")
                or key == 27
            ):
                break

    except KeyboardInterrupt:
        pass

    finally:

        print()

        cv2.destroyAllWindows()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()