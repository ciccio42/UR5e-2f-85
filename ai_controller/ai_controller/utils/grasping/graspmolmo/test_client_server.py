#!/usr/bin/env python3
"""
Integration test for GraspMolmoClient <-> server.py.

Requires the GraspMolmo server to be already running in the dedicated
virtual environment.

Run:

    python3 test_client_server.py

Optionally:

    python3 test_client_server.py \
        --image /scene_capture/nut/scena_1/rgb.png
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from ai_controller.utils.grasping.graspmolmo.client import (
    GraspMolmoClient,
)


THIS_DIR = Path(__file__).resolve().parent


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "--config",
        default=str(
            THIS_DIR
            / "config"
            / "graspmolmo_config.yaml"
        ),
    )

    parser.add_argument(
        "--image",
        default="/scene_capture/nut/scena_1/rgb.png",
    )

    parser.add_argument(
        "--task",
        default=(
            "Grasp the protruding handle of the blue ring. "
            "Do not grasp the circular ring body or through the hole. "
            "Then place the ring onto the right peg."
        ),
    )

    parser.add_argument(
        "--verbosity",
        type=int,
        default=1,
    )

    parser.add_argument(
        "--output",
        default=(
            "/home/ros2_ws/src/UR5e-2f-85/"
            ".runtime/graspmolmo_tests/"
            "test_client_server_prediction.png"
        ),
    )

    args = parser.parse_args()

    print(
        f"[TEST] Connecting to GraspMolmo server via "
        f"{args.config} ..."
    )

    client = GraspMolmoClient(
        config_path=args.config,
    )

    print(
        f"[TEST] Loading image: {args.image}"
    )

    image = Image.open(
        args.image
    ).convert("RGB")

    rgb = np.asarray(
        image,
        dtype=np.uint8,
    )

    print(
        f"[TEST] Image shape: {rgb.shape}"
    )

    print(
        f"[TEST] Task: {args.task}"
    )

    t0 = time.perf_counter()

    point = client.predict_point(
        rgb=rgb,
        task=args.task,
        verbosity=args.verbosity,
    )

    elapsed = time.perf_counter() - t0

    assert point is not None, (
        "Expected a valid grasp point, got None."
    )

    point = np.asarray(
        point,
        dtype=np.float32,
    )

    assert point.shape == (2,), (
        f"Expected point shape (2,), got {point.shape}"
    )

    x = float(point[0])
    y = float(point[1])

    width, height = image.size

    assert 0.0 <= x < width, (
        f"Predicted x coordinate out of bounds: {x}"
    )

    assert 0.0 <= y < height, (
        f"Predicted y coordinate out of bounds: {y}"
    )

    print(
        f"[TEST] Predicted point: "
        f"x={x:.2f}, y={y:.2f}"
    )

    print(
        f"[TEST] Inference time: "
        f"{elapsed:.2f} s"
    )

    # ---------------------------------------------------------------------
    # Visualization
    # ---------------------------------------------------------------------

    output_path = Path(
        args.output
    )

    output_path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    vis = image.copy()
    draw = ImageDraw.Draw(vis)

    r = 10

    draw.ellipse(
        (
            x - r,
            y - r,
            x + r,
            y + r,
        ),
        fill="red",
        outline="white",
        width=2,
    )

    draw.line(
        (
            x - 20,
            y,
            x + 20,
            y,
        ),
        fill="red",
        width=3,
    )

    draw.line(
        (
            x,
            y - 20,
            x,
            y + 20,
        ),
        fill="red",
        width=3,
    )

    vis.save(
        output_path
    )

    print(
        f"[TEST] Visualization saved to: "
        f"{output_path}"
    )

    print(
        "[PASS] GraspMolmoClient <-> "
        "server.py round-trip works correctly."
    )


if __name__ == "__main__":
    main()