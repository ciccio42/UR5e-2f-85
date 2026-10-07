#!/usr/bin/env python3

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from PIL import Image


REPO_ROOT = Path(
    "/home/ros2_ws/src/UR5e-2f-85"
)

SEEDO_DIR = (
    REPO_ROOT
    / "ai_controller"
    / "ai_controller"
    / "models"
    / "seedo_controller"
)

if str(SEEDO_DIR) not in sys.path:
    sys.path.insert(
        0,
        str(SEEDO_DIR),
    )

from inspect_pkl import (
    load_pickle,
    get_trajectory_step,
)


PKL_PATH = Path(
    "/scene_capture/traj_000.pkl"
)

OUTPUT_DIR = (
    REPO_ROOT
    / ".runtime"
    / "eye_in_hand_tests"
)

STEPS = [
    19,
    # 20,
    # 23,
    # 25,
    # 26,
]


def save_depth_preview(
    depth: np.ndarray,
    output_path: Path,
) -> None:

    depth = np.asarray(
        depth,
        dtype=np.float32,
    )

    valid = (
        np.isfinite(depth)
        & (depth > 0.0)
    )

    preview = np.zeros(
        depth.shape,
        dtype=np.uint8,
    )

    if np.any(valid):

        values = depth[
            valid
        ]

        near = np.percentile(
            values,
            2,
        )

        far = np.percentile(
            values,
            98,
        )

        normalized = (
            depth - near
        ) / max(
            far - near,
            1e-6,
        )

        normalized = np.clip(
            normalized,
            0.0,
            1.0,
        )

        # Near = bright, far = dark.
        preview[
            valid
        ] = (
            255.0
            * (
                1.0
                - normalized[valid]
            )
        ).astype(
            np.uint8
        )

    Image.fromarray(
        preview,
        mode="L",
    ).save(
        output_path
    )


def main() -> None:

    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    print(
        f"[EXTRACT] Loading {PKL_PATH}"
    )

    data = load_pickle(
        PKL_PATH
    )

    traj = data[
        "traj"
    ]

    print(
        f"[EXTRACT] Trajectory length: {len(traj)}"
    )

    for step_index in STEPS:

        step = get_trajectory_step(
            traj,
            step_index,
        )

        obs = step[
            "obs"
        ]

        bgr = np.asarray(
            obs[
                "eye_in_hand_image"
            ],
            dtype=np.uint8,
        )

        depth = np.asarray(
            obs[
                "eye_in_hand_depth"
            ],
            dtype=np.float32,
        )

        if bgr.shape != (
            376,
            672,
            3,
        ):
            raise ValueError(
                f"Unexpected eye-in-hand image shape "
                f"at step {step_index}: {bgr.shape}"
            )

        if depth.shape != (
            376,
            672,
        ):
            raise ValueError(
                f"Unexpected eye-in-hand depth shape "
                f"at step {step_index}: {depth.shape}"
            )

        # Dataset stores camera images in BGR/OpenCV format.
        rgb = bgr[
            ...,
            ::-1
        ].copy()

        step_dir = (
            OUTPUT_DIR
            / f"step_{step_index:03d}"
        )

        step_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        rgb_path = (
            step_dir
            / "eye_in_hand_rgb.png"
        )

        depth_path = (
            step_dir
            / "eye_in_hand_depth.npy"
        )

        preview_path = (
            step_dir
            / "eye_in_hand_depth_preview.png"
        )

        Image.fromarray(
            rgb,
            mode="RGB",
        ).save(
            rgb_path
        )

        np.save(
            depth_path,
            depth,
        )

        save_depth_preview(
            depth,
            preview_path,
        )

        valid = (
            np.isfinite(depth)
            & (depth > 0.0)
        )

        valid_depth = depth[
            valid
        ]

        status = step.get(
            "info",
            {},
        ).get(
            "status"
        )

        print()
        print(
            f"[STEP {step_index}]"
        )

        print(
            "  status:",
            status,
        )

        print(
            "  gripper_qpos:",
            obs.get(
                "gripper_qpos"
            ),
        )

        print(
            "  RGB:",
            rgb_path,
        )

        print(
            "  depth:",
            depth_path,
        )

        print(
            "  preview:",
            preview_path,
        )

        print(
            "  valid depth:",
            f"{valid.sum()} / {depth.size}",
        )

        if valid_depth.size:
            print(
                "  depth range:",
                f"{valid_depth.min():.4f} -> "
                f"{valid_depth.max():.4f} m",
            )

            print(
                "  median depth:",
                f"{np.median(valid_depth):.4f} m",
            )

    print()
    print(
        "[PASS] Eye-in-hand frames extracted."
    )


if __name__ == "__main__":
    main()