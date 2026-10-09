#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml


# =====================================================================
# Paths
# =====================================================================

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

from ai_controller.utils.grasping.grasp_planner import (
    GraspPlanner,
)


# =====================================================================
# Eye-in-hand calibration used by the validated grasp pipeline test
# =====================================================================

DEFAULT_K = np.array(
    [
        [
            363.8071594238281,
            0.0,
            335.86553955078125,
        ],
        [
            0.0,
            363.8071594238281,
            183.81210327148438,
        ],
        [
            0.0,
            0.0,
            1.0,
        ],
    ],
    dtype=np.float64,
)


# =====================================================================
# TCP -> eye-in-hand optical frame
#
# Same transform used by test_grasp_pipeline.py.
# This is only for the offline PKL test.
# At runtime this will come from TF.
# =====================================================================

T_TCP_CAMERA = np.array(
    [
        [
            0.999262529,
            -0.0383979045,
            0.0,
            -0.0209004103,
        ],
        [
            0.0383979045,
            0.999262529,
            0.0,
            -0.0681027558,
        ],
        [
            0.0,
            0.0,
            1.0,
            -0.1455,
        ],
        [
            0.0,
            0.0,
            0.0,
            1.0,
        ],
    ],
    dtype=np.float64,
)


# =====================================================================
# Helpers
# =====================================================================

def quaternion_to_rotation(
    quaternion: np.ndarray,
) -> np.ndarray:
    quaternion = np.asarray(
        quaternion,
        dtype=np.float64,
    )

    if quaternion.shape != (4,):
        raise ValueError(
            "Quaternion must have shape (4,)."
        )

    x, y, z, w = quaternion

    norm = np.linalg.norm(
        quaternion
    )

    if norm <= 1e-12:
        raise ValueError(
            "Invalid zero quaternion."
        )

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    return np.array(
        [
            [
                1 - 2 * (y * y + z * z),
                2 * (x * y - z * w),
                2 * (x * z + y * w),
            ],
            [
                2 * (x * y + z * w),
                1 - 2 * (x * x + z * z),
                2 * (y * z - x * w),
            ],
            [
                2 * (x * z - y * w),
                2 * (y * z + x * w),
                1 - 2 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def pose_to_transform(
    position: np.ndarray,
    quaternion: np.ndarray,
) -> np.ndarray:
    transform = np.eye(
        4,
        dtype=np.float64,
    )

    transform[
        :3,
        :3,
    ] = quaternion_to_rotation(
        quaternion
    )

    transform[
        :3,
        3,
    ] = np.asarray(
        position,
        dtype=np.float64,
    )

    return transform


def load_base_to_table(
    path: Path,
) -> np.ndarray:
    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        data = yaml.safe_load(
            stream
        )

    transform = np.eye(
        4,
        dtype=np.float64,
    )

    transform[
        :3,
        :3,
    ] = np.asarray(
        data["rotation"],
        dtype=np.float64,
    )

    transform[
        :3,
        3,
    ] = np.asarray(
        data["translation"],
        dtype=np.float64,
    )

    return transform


def load_camera_matrix(
    path: Path | None,
) -> np.ndarray:
    if path is None:
        print(
            "[TEST] Using built-in eye-in-hand intrinsics."
        )

        return DEFAULT_K.copy()

    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        data = yaml.safe_load(
            stream
        )

    return np.asarray(
        data["k"],
        dtype=np.float64,
    ).reshape(
        3,
        3,
    )


def get_gripper_rgbd(
    obs: dict,
) -> tuple[
    np.ndarray,
    np.ndarray,
]:
    """
    Support both rollout schemas already handled by
    test_grasp_pipeline.py.
    """

    candidates = [
        (
            "eye_in_hand_image",
            "eye_in_hand_depth",
        ),
        (
            "camera_gripper_image",
            "camera_gripper_depth",
        ),
    ]

    for (
        image_key,
        depth_key,
    ) in candidates:

        if (
            image_key in obs
            and depth_key in obs
        ):
            print(
                "[TEST] RGB key:",
                image_key,
            )

            print(
                "[TEST] Depth key:",
                depth_key,
            )

            return (
                np.asarray(
                    obs[
                        image_key
                    ]
                ),
                np.asarray(
                    obs[
                        depth_key
                    ],
                    dtype=np.float32,
                ),
            )

    raise RuntimeError(
        "No eye-in-hand RGB-D pair found."
    )


# =====================================================================
# Main
# =====================================================================

def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Offline PKL test for the SeeDo GraspPlanner."
        )
    )

    parser.add_argument(
        "--pkl",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--step",
        type=int,
        required=True,
    )

    parser.add_argument(
        "--base-to-table",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--camera-info",
        type=Path,
        default=None,
    )

    parser.add_argument(
        "--task",
        type=str,
        required=True,
        help=(
            "Grasp instruction passed directly "
            "to GraspMolmo."
        ),
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=(
            REPO_ROOT
            / ".runtime"
            / "grasp_planner_test"
        ),
    )

    # -------------------------------------------------------------
    # Same validated defaults used with test_grasp_pipeline.py
    # -------------------------------------------------------------

    parser.add_argument(
        "--num-runs",
        type=int,
        default=20,
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=42,
    )

    parser.add_argument(
        "--confidence-threshold",
        type=float,
        default=0.4,
    )

    parser.add_argument(
        "--semantic-radius-px",
        type=float,
        default=50.0,
    )

    parser.add_argument(
        "--max-approach-tilt-deg",
        type=float,
        default=45.0,
    )

    parser.add_argument(
        "--max-wrist-rotation-deg",
        type=float,
        default=45.0,
    )

    parser.add_argument(
        "--disable-finger-collision-check",
        action="store_true",
    )

    args = parser.parse_args()

    # =============================================================
    # Paths
    # =============================================================

    pkl_path = (
        args.pkl
        .expanduser()
        .resolve()
    )

    base_to_table_path = (
        args.base_to_table
        .expanduser()
        .resolve()
    )

    output_dir = (
        args.output_dir
        .expanduser()
        .resolve()
    )

    if not pkl_path.is_file():
        raise FileNotFoundError(
            f"PKL not found: {pkl_path}"
        )

    if not base_to_table_path.is_file():
        raise FileNotFoundError(
            "Base-to-table transform not found: "
            f"{base_to_table_path}"
        )

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    print()
    print(
        "=" * 78
    )

    print(
        "GRASP PLANNER OFFLINE TEST"
    )

    print(
        "=" * 78
    )

    print(
        "[TEST] PKL:",
        pkl_path,
    )

    print(
        "[TEST] Step:",
        args.step,
    )

    print(
        "[TEST] Grasp instruction:",
        args.task,
    )

    # =============================================================
    # Load trajectory
    # =============================================================

    data = load_pickle(
        pkl_path
    )

    trajectory = data[
        "traj"
    ]

    print(
        "[TEST] Trajectory length:",
        len(
            trajectory
        ),
    )

    step = get_trajectory_step(
        trajectory,
        args.step,
    )

    obs = step[
        "obs"
    ]

    # =============================================================
    # RGB-D
    # =============================================================

    (
        image_bgr,
        depth,
    ) = get_gripper_rgbd(
        obs
    )

    if (
        image_bgr.ndim != 3
        or image_bgr.shape[2] != 3
    ):
        raise RuntimeError(
            "Invalid eye-in-hand RGB shape: "
            f"{image_bgr.shape}"
        )

    if depth.ndim != 2:
        raise RuntimeError(
            "Invalid eye-in-hand depth shape: "
            f"{depth.shape}"
        )

    # Rollout images are BGR/OpenCV.
    image_rgb = cv2.cvtColor(
        image_bgr,
        cv2.COLOR_BGR2RGB,
    )

    cv2.imwrite(
        str(
            output_dir
            / "input_rgb.png"
        ),
        image_bgr,
    )

    np.save(
        output_dir
        / "input_depth.npy",
        depth,
    )

    print(
        "[TEST] RGB shape:",
        image_rgb.shape,
    )

    print(
        "[TEST] Depth shape:",
        depth.shape,
    )

    # =============================================================
    # TCP pose from rollout
    # =============================================================

    if "eef_pos" not in obs:
        raise RuntimeError(
            "Observation does not contain 'eef_pos'."
        )

    if "eef_quat" not in obs:
        raise RuntimeError(
            "Observation does not contain 'eef_quat'."
        )

    eef_position = np.asarray(
        obs[
            "eef_pos"
        ],
        dtype=np.float64,
    )

    eef_quaternion = np.asarray(
        obs[
            "eef_quat"
        ],
        dtype=np.float64,
    )

    print(
        "[TEST] EEF position:",
        eef_position,
    )

    print(
        "[TEST] EEF quaternion:",
        eef_quaternion,
    )

    T_base_tcp = pose_to_transform(
        eef_position,
        eef_quaternion,
    )

    T_base_camera = (
        T_base_tcp
        @ T_TCP_CAMERA
    )

    # =============================================================
    # Calibration
    # =============================================================

    camera_info_path = (
        None
        if args.camera_info is None
        else (
            args.camera_info
            .expanduser()
            .resolve()
        )
    )

    K = load_camera_matrix(
        camera_info_path
    )

    T_base_table = (
        load_base_to_table(
            base_to_table_path
        )
    )

    print()
    print(
        "[TEST] Camera matrix:"
    )

    print(
        K
    )

    print()
    print(
        "[TEST] T_base_camera:"
    )

    print(
        T_base_camera
    )

    print()
    print(
        "[TEST] T_base_table:"
    )

    print(
        T_base_table
    )

    # =============================================================
    # GraspPlanner
    # =============================================================

    print()
    print(
        "[TEST] Initializing GraspPlanner..."
    )

    planner = GraspPlanner(
        num_runs=args.num_runs,
        seed=args.seed,
        confidence_threshold=(
            args.confidence_threshold
        ),
        semantic_radius_px=(
            args.semantic_radius_px
        ),
        max_approach_tilt_deg=(
            args.max_approach_tilt_deg
        ),
        max_wrist_rotation_deg=(
            args.max_wrist_rotation_deg
        ),
        finger_collision_check=(
            not args.disable_finger_collision_check
        ),
    )

    print()
    print(
        "[TEST] Running grasp planning..."
    )

    result = planner.plan(
        rgb_image=image_rgb,
        depth_image=depth,
        camera_matrix=K,
        T_base_camera=T_base_camera,
        T_base_table=T_base_table,
        current_tcp_position=(
            eef_position
        ),
        current_tcp_orientation=(
            eef_quaternion
        ),
        grasp_instruction=(
            args.task
        ),
        artifacts_dir=(
            output_dir
        ),
    )

    # =============================================================
    # Validate result
    # =============================================================

    if result is None:
        raise RuntimeError(
            "GraspPlanner returned None."
        )

    if (
        np.asarray(
            result.grasp_pose_base
        ).shape
        != (4, 4)
    ):
        raise RuntimeError(
            "Invalid grasp_pose_base shape: "
            f"{np.asarray(result.grasp_pose_base).shape}"
        )

    if (
        np.asarray(
            result.grasp_position_base
        ).shape
        != (3,)
    ):
        raise RuntimeError(
            "Invalid grasp_position_base shape."
        )

    if (
        np.asarray(
            result.grasp_orientation_base
        ).shape
        != (4,)
    ):
        raise RuntimeError(
            "Invalid grasp_orientation_base shape."
        )

    if (
        np.asarray(
            result.semantic_point_px
        ).shape
        != (2,)
    ):
        raise RuntimeError(
            "Invalid semantic_point_px shape."
        )

    if not np.isfinite(
        result.grasp_pose_base
    ).all():
        raise RuntimeError(
            "grasp_pose_base contains non-finite values."
        )

    # =============================================================
    # Output
    # =============================================================

    print()
    print(
        "=" * 78
    )

    print(
        "[RESULT] GraspPlanner succeeded."
    )

    print(
        "[RESULT] Instruction:"
    )

    print(
        "        ",
        result.grasp_instruction,
    )

    print(
        "[RESULT] Semantic point:",
        result.semantic_point_px,
    )

    print(
        "[RESULT] Selected M2T2 index:",
        result.selected_m2t2_index,
    )

    print(
        "[RESULT] Confidence:",
        result.confidence,
    )

    print(
        "[RESULT] Semantic distance:",
        f"{result.semantic_distance_px:.3f} px",
    )

    print(
        "[RESULT] Approach tilt:",
        f"{result.approach_tilt_deg:.3f} deg",
    )

    print(
        "[RESULT] Wrist rotation:",
        f"{result.wrist_rotation_deg:.3f} deg",
    )

    print()
    print(
        "[RESULT] Grasp position in base_link:"
    )

    print(
        result.grasp_position_base
    )

    print()
    print(
        "[RESULT] Grasp orientation in base_link "
        "[qx, qy, qz, qw]:"
    )

    print(
        result.grasp_orientation_base
    )

    print()
    print(
        "[RESULT] Grasp pose in base_link:"
    )

    print(
        result.grasp_pose_base
    )

    print()

    artifact_path = (
        output_dir
        / "grasp_plan.json"
    )

    if not artifact_path.is_file():
        raise RuntimeError(
            "GraspPlanner did not generate "
            f"{artifact_path}"
        )

    with artifact_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        artifact = json.load(
            stream
        )

    if not artifact:
        raise RuntimeError(
            "grasp_plan.json is empty."
        )

    print(
        "[TEST] Artifact:",
        artifact_path,
    )

    print(
        "=" * 78
    )

    print(
        "[PASS] test_grasp_planner completed successfully."
    )

    print(
        "=" * 78
    )


if __name__ == "__main__":
    main()