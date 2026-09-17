#!/usr/bin/env python3
"""Standalone debug tool for OSVI-AWDA wrist/gripper depth segmentation.

This script DOES NOT command or move the robot.
It only reads one RGB frame and one registered depth frame from the gripper
camera, reproduces the current OSVI-AWDA foreground/connected-component logic,
and saves all intermediate outputs for inspection.

Pipeline reproduced from the controller:
    depth image
      -> border exclusion mask
      -> valid depth mask
      -> floor_depth = median(valid depth)
      -> above_floor = (floor_depth - depth) > floor_margin
      -> foreground mask
      -> connected components
      -> reject components smaller than min_component_area_px
      -> choose centroid closest to image center

Example:
    python3 osvi_awda_depth_segmentation_debug.py \
        --config path/to/osvi_awda.yaml \
        --output-dir /tmp/osvi_depth_debug

Or without a YAML:
    python3 osvi_awda_depth_segmentation_debug.py \
        --depth-topic /zed_gripper/zed_node/depth/depth_registered \
        --rgb-topic /zed_gripper/zed_node/rgb/color/rect/image \
        --depth-max-range-m 1.0 \
        --floor-margin-m 0.01 \
        --min-component-area-px 20 \
        --bottom 20 --left 10 --right 10
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np
import yaml

import rclpy
import rclpy.wait_for_message
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage


DEFAULT_DEPTH_TOPIC = "/zed_gripper/zed_node/depth/depth_registered"
DEFAULT_RGB_TOPIC = "/zed_gripper/zed_node/rgb/color/rect/image"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture one gripper RGB/depth frame and save every intermediate "
            "stage of the OSVI-AWDA depth segmentation pipeline."
        )
    )

    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help=(
            "Optional OSVI-AWDA runtime YAML. Values under grasp_refinement "
            "are used unless overridden on the command line."
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output directory. Default: ./osvi_depth_debug_<timestamp>",
    )

    parser.add_argument("--depth-topic", default=None)
    parser.add_argument("--rgb-topic", default=None)
    parser.add_argument("--timeout-sec", type=float, default=2.0)

    parser.add_argument("--depth-scale", type=float, default=None)
    parser.add_argument("--depth-max-range-m", type=float, default=None)
    parser.add_argument("--floor-margin-m", type=float, default=None)
    parser.add_argument("--min-component-area-px", type=int, default=None)

    parser.add_argument("--top", type=int, default=None)
    parser.add_argument("--bottom", type=int, default=None)
    parser.add_argument("--left", type=int, default=None)
    parser.add_argument("--right", type=int, default=None)

    parser.add_argument(
        "--show",
        action="store_true",
        help="Also show a final debug mosaic with OpenCV.",
    )

    return parser.parse_args()


def load_grasp_refinement_config(config_path: Optional[Path]) -> Dict[str, Any]:
    if config_path is None:
        return {}

    config_path = config_path.expanduser().resolve()
    if not config_path.is_file():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    with config_path.open("r", encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream) or {}

    section = cfg.get("grasp_refinement", {})
    if not isinstance(section, dict):
        raise ValueError("grasp_refinement in YAML must be a mapping.")

    return section


def choose(cli_value, config: Dict[str, Any], key: str, default):
    if cli_value is not None:
        return cli_value
    return config.get(key, default)


def ros_stamp_to_float(msg: RosImage) -> Optional[float]:
    try:
        stamp = msg.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except Exception:
        return None


def wait_for_image(
    node: Node,
    topic: str,
    timeout_sec: float,
) -> RosImage:
    ok, msg = rclpy.wait_for_message.wait_for_message(
        topic=topic,
        msg_type=RosImage,
        node=node,
        time_to_wait=timeout_sec,
    )
    if not ok:
        raise TimeoutError(f"No image received on {topic!r} within {timeout_sec:.2f} s")
    return msg


def depth_to_meters(raw_depth: np.ndarray, configured_scale: float) -> Tuple[np.ndarray, float]:
    raw = np.asarray(raw_depth)
    if raw.ndim == 3:
        raw = raw[:, :, 0]

    scale = float(configured_scale)
    # Same behavior as the controller: integer depth with scale=1.0 is assumed mm.
    if np.issubdtype(raw.dtype, np.integer) and scale == 1.0:
        scale = 0.001

    return raw.astype(np.float64) * scale, scale


def normalize_depth_for_display(depth_m: np.ndarray, valid_mask: np.ndarray) -> np.ndarray:
    """Create a human-readable color map without modifying the actual depth data."""
    display = np.zeros(depth_m.shape, dtype=np.uint8)

    values = depth_m[valid_mask]
    if values.size > 0:
        lo = float(np.percentile(values, 1.0))
        hi = float(np.percentile(values, 99.0))
        if hi <= lo:
            hi = lo + 1e-6

        scaled = (depth_m - lo) / (hi - lo)
        scaled = np.clip(scaled, 0.0, 1.0)
        display = (scaled * 255.0).astype(np.uint8)
        display[~valid_mask] = 0

    return cv2.applyColorMap(display, cv2.COLORMAP_TURBO)


def mask_to_png(mask: np.ndarray) -> np.ndarray:
    return (np.asarray(mask, dtype=np.uint8) * 255)


def make_keep_mask(
    height: int,
    width: int,
    top: int,
    bottom: int,
    left: int,
    right: int,
) -> np.ndarray:
    keep = np.ones((height, width), dtype=bool)

    top = max(0, min(int(top), height))
    bottom = max(0, min(int(bottom), height))
    left = max(0, min(int(left), width))
    right = max(0, min(int(right), width))

    if top > 0:
        keep[:top, :] = False
    if bottom > 0:
        keep[height - bottom :, :] = False
    if left > 0:
        keep[:, :left] = False
    if right > 0:
        keep[:, width - right :] = False

    return keep


def component_color(label: int) -> Tuple[int, int, int]:
    """Deterministic pseudo-color for connected-component visualization."""
    # Avoid black for foreground labels.
    return (
        int((53 * label) % 205 + 50),
        int((97 * label) % 205 + 50),
        int((151 * label) % 205 + 50),
    )


def scale_depth_point_to_rgb(
    u_depth: int,
    v_depth: int,
    depth_shape: Tuple[int, int],
    rgb_shape: Tuple[int, int],
) -> Tuple[int, int]:
    depth_h, depth_w = depth_shape
    rgb_h, rgb_w = rgb_shape

    if depth_w > 1:
        u_rgb = int(round(float(u_depth) * (rgb_w - 1) / (depth_w - 1)))
    else:
        u_rgb = 0

    if depth_h > 1:
        v_rgb = int(round(float(v_depth) * (rgb_h - 1) / (depth_h - 1)))
    else:
        v_rgb = 0

    u_rgb = int(np.clip(u_rgb, 0, rgb_w - 1))
    v_rgb = int(np.clip(v_rgb, 0, rgb_h - 1))
    return u_rgb, v_rgb


def draw_excluded_margins(
    image_bgr: np.ndarray,
    depth_shape: Tuple[int, int],
    top: int,
    bottom: int,
    left: int,
    right: int,
) -> np.ndarray:
    """Darken the RGB regions corresponding to ignored depth-image margins."""
    out = image_bgr.copy()
    rgb_h, rgb_w = out.shape[:2]
    depth_h, depth_w = depth_shape

    sx = rgb_w / float(depth_w)
    sy = rgb_h / float(depth_h)

    top_rgb = int(round(top * sy))
    bottom_rgb = int(round(bottom * sy))
    left_rgb = int(round(left * sx))
    right_rgb = int(round(right * sx))

    overlay = out.copy()
    if top_rgb > 0:
        overlay[:top_rgb, :] = 0
    if bottom_rgb > 0:
        overlay[rgb_h - bottom_rgb :, :] = 0
    if left_rgb > 0:
        overlay[:, :left_rgb] = 0
    if right_rgb > 0:
        overlay[:, rgb_w - right_rgb :] = 0

    return cv2.addWeighted(out, 0.45, overlay, 0.55, 0.0)


def resize_to_height(image: np.ndarray, target_height: int) -> np.ndarray:
    h, w = image.shape[:2]
    if h == target_height:
        return image
    scale = float(target_height) / float(h)
    return cv2.resize(
        image,
        (max(1, int(round(w * scale))), target_height),
        interpolation=cv2.INTER_NEAREST,
    )


def main() -> None:
    args = parse_args()
    cfg = load_grasp_refinement_config(args.config)

    margins_cfg = cfg.get("depth_ignore_margins_px", {}) or {}

    depth_topic = str(
        choose(args.depth_topic, cfg, "depth_topic", DEFAULT_DEPTH_TOPIC)
    )
    rgb_topic = str(
        choose(args.rgb_topic, cfg, "rgb_topic", DEFAULT_RGB_TOPIC)
    )

    depth_scale = float(choose(args.depth_scale, cfg, "depth_scale", 1.0))
    max_depth = float(
        choose(args.depth_max_range_m, cfg, "depth_max_range_m", 1.0)
    )
    floor_margin = float(
        choose(args.floor_margin_m, cfg, "floor_margin_m", 0.01)
    )
    min_area = int(
        choose(args.min_component_area_px, cfg, "min_component_area_px", 20)
    )

    top = int(args.top if args.top is not None else margins_cfg.get("top", 0))
    bottom = int(args.bottom if args.bottom is not None else margins_cfg.get("bottom", 0))
    left = int(args.left if args.left is not None else margins_cfg.get("left", 0))
    right = int(args.right if args.right is not None else margins_cfg.get("right", 0))

    if max_depth <= 0.0:
        raise ValueError("depth_max_range_m must be > 0")
    if floor_margin < 0.0:
        raise ValueError("floor_margin_m must be >= 0")
    if min_area < 1:
        raise ValueError("min_component_area_px must be >= 1")

    if args.output_dir is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = Path.cwd() / f"osvi_depth_debug_{timestamp}"
    else:
        output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 80)
    print("OSVI-AWDA GRIPPER DEPTH SEGMENTATION DEBUG")
    print("NO robot commands are published by this script.")
    print(f"depth_topic                = {depth_topic}")
    print(f"rgb_topic                  = {rgb_topic}")
    print(f"depth_scale config         = {depth_scale}")
    print(f"depth_max_range_m          = {max_depth}")
    print(f"floor_margin_m             = {floor_margin}")
    print(f"min_component_area_px      = {min_area}")
    print(f"ignored margins [T,B,L,R] = [{top}, {bottom}, {left}, {right}]")
    print(f"output_dir                 = {output_dir}")
    print("=" * 80)

    rclpy.init()
    node = Node("osvi_awda_depth_segmentation_debug")
    bridge = CvBridge()

    try:
        # Robot remains stationary; exact message synchronization is not required
        # for this diagnostic. Timestamps are saved so the offset can be inspected.
        print("[1/2] Waiting for registered depth image...")
        depth_msg = wait_for_image(node, depth_topic, args.timeout_sec)
        print("[2/2] Waiting for RGB image...")
        rgb_msg = wait_for_image(node, rgb_topic, args.timeout_sec)

        raw_depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        rgb_bgr = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        rgb_bgr = np.asarray(rgb_bgr, dtype=np.uint8)

        depth_m, effective_scale = depth_to_meters(raw_depth, depth_scale)
        if depth_m.ndim != 2:
            raise ValueError(f"Expected 2D depth image, got shape {depth_m.shape}")

        height, width = depth_m.shape
        keep_mask = make_keep_mask(height, width, top, bottom, left, right)

        finite_positive = np.isfinite(depth_m) & (depth_m > 0.0)
        range_mask = depth_m < max_depth
        valid_mask = finite_positive & range_mask & keep_mask

        if not np.any(valid_mask):
            raise RuntimeError("No valid depth pixels remain after filtering.")

        floor_depth = float(np.median(depth_m[valid_mask]))

        above_floor_mask = (floor_depth - depth_m) > floor_margin
        foreground_mask = valid_mask & above_floor_mask

        label_count, labels = cv2.connectedComponents(
            foreground_mask.astype(np.uint8)
        )

        image_center_vu = np.asarray(
            [height / 2.0, width / 2.0], dtype=np.float64
        )

        component_records = []
        selected_label = None
        selected_centroid_vu = None
        selected_distance = None

        all_components_vis = np.zeros((height, width, 3), dtype=np.uint8)
        accepted_components_vis = np.zeros((height, width, 3), dtype=np.uint8)

        for label in range(1, label_count):
            rows, cols = np.where(labels == label)
            area = int(len(rows))
            if area == 0:
                continue

            centroid_vu = np.asarray(
                [float(rows.mean()), float(cols.mean())], dtype=np.float64
            )
            distance = float(np.linalg.norm(centroid_vu - image_center_vu))
            accepted = area >= min_area
            color = component_color(label)

            all_components_vis[labels == label] = color
            if accepted:
                accepted_components_vis[labels == label] = color

            record = {
                "label": int(label),
                "area_px": area,
                "accepted_by_min_area": bool(accepted),
                "centroid_u": float(centroid_vu[1]),
                "centroid_v": float(centroid_vu[0]),
                "distance_to_image_center_px": distance,
            }
            component_records.append(record)

            if accepted and (
                selected_distance is None or distance < selected_distance
            ):
                selected_label = int(label)
                selected_centroid_vu = centroid_vu
                selected_distance = distance

        selected_mask = np.zeros((height, width), dtype=np.uint8)
        selected_u = None
        selected_v = None
        selected_depth = None

        if selected_label is not None and selected_centroid_vu is not None:
            selected_mask[labels == selected_label] = 255
            selected_v = int(np.clip(round(selected_centroid_vu[0]), 0, height - 1))
            selected_u = int(np.clip(round(selected_centroid_vu[1]), 0, width - 1))
            selected_depth = float(depth_m[selected_v, selected_u])

        # ------------------------------------------------------------------
        # Save intermediate images/data
        # ------------------------------------------------------------------
        cv2.imwrite(str(output_dir / "00_rgb_raw.png"), rgb_bgr)
        np.save(output_dir / "01_depth_meters.npy", depth_m)

        # For display, use finite positive depths, even if > max_depth, so the
        # user can see what the sensor returned before range filtering.
        raw_depth_display_mask = finite_positive
        cv2.imwrite(
            str(output_dir / "02_depth_colormap_raw.png"),
            normalize_depth_for_display(depth_m, raw_depth_display_mask),
        )

        cv2.imwrite(
            str(output_dir / "03_keep_mask.png"),
            mask_to_png(keep_mask),
        )
        cv2.imwrite(
            str(output_dir / "04_finite_positive_mask.png"),
            mask_to_png(finite_positive),
        )
        cv2.imwrite(
            str(output_dir / "05_within_max_range_mask.png"),
            mask_to_png(finite_positive & range_mask),
        )
        cv2.imwrite(
            str(output_dir / "06_valid_mask_after_all_filters.png"),
            mask_to_png(valid_mask),
        )

        valid_depth_colormap = normalize_depth_for_display(depth_m, valid_mask)
        cv2.imwrite(
            str(output_dir / "07_depth_colormap_valid_only.png"),
            valid_depth_colormap,
        )

        cv2.imwrite(
            str(output_dir / "08_above_floor_mask_before_valid_intersection.png"),
            mask_to_png(above_floor_mask),
        )
        cv2.imwrite(
            str(output_dir / "09_foreground_mask.png"),
            mask_to_png(foreground_mask),
        )
        cv2.imwrite(
            str(output_dir / "10_connected_components_all.png"),
            all_components_vis,
        )
        cv2.imwrite(
            str(output_dir / "11_connected_components_min_area_filtered.png"),
            accepted_components_vis,
        )
        cv2.imwrite(
            str(output_dir / "12_selected_component_mask.png"),
            selected_mask,
        )

        rgb_margins = draw_excluded_margins(
            rgb_bgr,
            depth_shape=(height, width),
            top=top,
            bottom=bottom,
            left=left,
            right=right,
        )
        cv2.imwrite(
            str(output_dir / "13_rgb_with_ignored_margins.png"),
            rgb_margins,
        )

        centroid_overlay = rgb_margins.copy()
        if selected_u is not None and selected_v is not None:
            u_rgb, v_rgb = scale_depth_point_to_rgb(
                selected_u,
                selected_v,
                depth_shape=(height, width),
                rgb_shape=rgb_bgr.shape[:2],
            )
            cross = max(10, int(min(rgb_bgr.shape[:2]) * 0.03))
            cv2.line(
                centroid_overlay,
                (u_rgb - cross, v_rgb - cross),
                (u_rgb + cross, v_rgb + cross),
                (0, 0, 255),
                3,
                cv2.LINE_AA,
            )
            cv2.line(
                centroid_overlay,
                (u_rgb - cross, v_rgb + cross),
                (u_rgb + cross, v_rgb - cross),
                (0, 0, 255),
                3,
                cv2.LINE_AA,
            )
            cv2.circle(
                centroid_overlay,
                (u_rgb, v_rgb),
                5,
                (0, 0, 255),
                -1,
                cv2.LINE_AA,
            )
            cv2.putText(
                centroid_overlay,
                f"selected centroid depth=(u={selected_u}, v={selected_v})",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
        else:
            cv2.putText(
                centroid_overlay,
                "NO COMPONENT PASSED THE FILTERS",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        cv2.imwrite(
            str(output_dir / "14_rgb_selected_centroid.png"),
            centroid_overlay,
        )

        # Annotate connected components with label / area / centroid.
        component_overlay = all_components_vis.copy()
        for record in component_records:
            u = int(round(record["centroid_u"]))
            v = int(round(record["centroid_v"]))
            accepted = record["accepted_by_min_area"]
            text = (
                f"L{record['label']} A={record['area_px']} "
                f"{'OK' if accepted else 'REJECT'}"
            )
            cv2.circle(
                component_overlay,
                (u, v),
                4,
                (255, 255, 255),
                -1,
                cv2.LINE_AA,
            )
            cv2.putText(
                component_overlay,
                text,
                (min(u + 5, width - 1), max(v - 5, 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        center_u = int(round(width / 2.0))
        center_v = int(round(height / 2.0))
        cv2.drawMarker(
            component_overlay,
            (center_u, center_v),
            (255, 255, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=18,
            thickness=2,
        )
        cv2.imwrite(
            str(output_dir / "15_components_annotated.png"),
            component_overlay,
        )

        # Final mosaic.
        panel_h = 360
        mosaic_panels = [
            resize_to_height(rgb_bgr, panel_h),
            resize_to_height(valid_depth_colormap, panel_h),
            resize_to_height(cv2.cvtColor(mask_to_png(foreground_mask), cv2.COLOR_GRAY2BGR), panel_h),
            resize_to_height(component_overlay, panel_h),
            resize_to_height(centroid_overlay, panel_h),
        ]
        mosaic = np.hstack(mosaic_panels)
        cv2.imwrite(str(output_dir / "16_debug_mosaic.png"), mosaic)

        depth_stamp = ros_stamp_to_float(depth_msg)
        rgb_stamp = ros_stamp_to_float(rgb_msg)
        stamp_delta = None
        if depth_stamp is not None and rgb_stamp is not None:
            stamp_delta = float(rgb_stamp - depth_stamp)

        summary = {
            "topics": {
                "depth": depth_topic,
                "rgb": rgb_topic,
            },
            "timestamps_sec": {
                "depth": depth_stamp,
                "rgb": rgb_stamp,
                "rgb_minus_depth": stamp_delta,
            },
            "image_shapes": {
                "depth_hw": [int(height), int(width)],
                "rgb_hw": [int(rgb_bgr.shape[0]), int(rgb_bgr.shape[1])],
            },
            "parameters": {
                "configured_depth_scale": depth_scale,
                "effective_depth_scale": effective_scale,
                "depth_max_range_m": max_depth,
                "floor_margin_m": floor_margin,
                "min_component_area_px": min_area,
                "depth_ignore_margins_px": {
                    "top": top,
                    "bottom": bottom,
                    "left": left,
                    "right": right,
                },
            },
            "statistics": {
                "valid_pixel_count": int(np.count_nonzero(valid_mask)),
                "valid_fraction": float(np.mean(valid_mask)),
                "floor_depth_m": floor_depth,
                "foreground_pixel_count": int(np.count_nonzero(foreground_mask)),
                "foreground_fraction": float(np.mean(foreground_mask)),
                "connected_component_count_excluding_background": int(label_count - 1),
                "accepted_component_count": int(
                    sum(1 for record in component_records if record["accepted_by_min_area"])
                ),
            },
            "components": component_records,
            "selected": {
                "label": selected_label,
                "centroid_u": selected_u,
                "centroid_v": selected_v,
                "distance_to_image_center_px": selected_distance,
                "depth_at_centroid_m": selected_depth,
            },
        }

        with (output_dir / "summary.json").open("w", encoding="utf-8") as stream:
            json.dump(summary, stream, indent=2)

        print("\nRESULT")
        print(f"floor_depth_m              = {floor_depth:.6f}")
        print(f"foreground pixels          = {np.count_nonzero(foreground_mask)}")
        print(f"components found           = {label_count - 1}")
        print(
            "components >= min area    = "
            f"{sum(1 for record in component_records if record['accepted_by_min_area'])}"
        )

        if selected_label is None:
            print("selected component         = NONE")
        else:
            print(f"selected component label   = {selected_label}")
            print(f"selected centroid (u,v)    = ({selected_u}, {selected_v})")
            print(f"distance to image center   = {selected_distance:.3f} px")
            print(f"depth at centroid          = {selected_depth:.6f} m")

        print(f"\nSaved all outputs to: {output_dir}")

        if args.show:
            cv2.namedWindow("OSVI-AWDA depth segmentation debug", cv2.WINDOW_NORMAL)
            cv2.imshow("OSVI-AWDA depth segmentation debug", mosaic)
            print("Press any key in the OpenCV window to close.")
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
