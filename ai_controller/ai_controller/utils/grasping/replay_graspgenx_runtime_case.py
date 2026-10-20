#!/usr/bin/env python3
"""
Replay a real GraspGen-X runtime case offline.

The script uses:
- input_rgb.png / input_depth.npy saved by the real grasp run;
- the rollout PKL only to recover the current TCP pose;
- the production base_link -> table_0 transform by default;
- the CURRENT local GraspGenXPlanner implementation, so new fixes can be
  regression-tested without moving the robot.

By default, if <case-dir>/grasp_plan.json exists, the exact GraspMolmo
semantic point saved during the real run is reused. This isolates downstream
changes (SAM, candidate filtering/ranking, table safety, symmetry, etc.).
Use --live-molmo to query GraspMolmo again instead.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


DEFAULT_REPO_ROOT = Path("/home/ros2_ws/src/UR5e-2f-85")

DEFAULT_K = np.array(
    [
        [363.8071594238281, 0.0, 335.86553955078125],
        [0.0, 363.8071594238281, 183.81210327148438],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)

# Validated eye-in-hand rigid transform used by the existing offline tests:
# T_base_camera = T_base_tcp @ T_TCP_CAMERA
T_TCP_CAMERA = np.array(
    [
        [0.999262529, -0.0383979045, 0.0, -0.0209004103],
        [0.0383979045, 0.999262529, 0.0, -0.0681027558],
        [0.0, 0.0, 1.0, -0.1455],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)


def install_project_imports(repo_root: Path) -> None:
    ai_controller_root = repo_root / "ai_controller"
    seedo_dir = (
        repo_root
        / "ai_controller"
        / "ai_controller"
        / "models"
        / "seedo_controller"
    )

    for path in (ai_controller_root, seedo_dir):
        path_str = str(path)
        if path_str not in sys.path:
            sys.path.insert(0, path_str)


def get_gripper_rgbd(obs: dict) -> tuple[np.ndarray, np.ndarray, str, str]:
    candidates = [
        ("eye_in_hand_image", "eye_in_hand_depth"),
        ("camera_gripper_image", "camera_gripper_depth"),
    ]

    for image_key, depth_key in candidates:
        if image_key in obs and depth_key in obs:
            return (
                np.asarray(obs[image_key]),
                np.asarray(obs[depth_key], dtype=np.float32),
                image_key,
                depth_key,
            )

    raise RuntimeError(
        "No supported eye-in-hand RGB-D pair found in the rollout observation."
    )


def pose_to_transform(
    position: np.ndarray,
    quaternion_xyzw: np.ndarray,
) -> np.ndarray:
    position = np.asarray(position, dtype=np.float64)
    quaternion_xyzw = np.asarray(quaternion_xyzw, dtype=np.float64)

    if position.shape != (3,):
        raise ValueError(f"position must have shape (3,), got {position.shape}")

    if quaternion_xyzw.shape != (4,):
        raise ValueError(
            "quaternion_xyzw must have shape (4,), "
            f"got {quaternion_xyzw.shape}"
        )

    norm = float(np.linalg.norm(quaternion_xyzw))
    if norm <= 0.0:
        raise ValueError("Quaternion has zero norm.")

    quaternion_xyzw = quaternion_xyzw / norm

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_quat(quaternion_xyzw).as_matrix()
    T[:3, 3] = position
    return T


def make_base_to_table(
    x: float,
    y: float,
    z: float,
    yaw_deg: float,
) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_euler(
        "z",
        float(yaw_deg),
        degrees=True,
    ).as_matrix()
    T[:3, 3] = [float(x), float(y), float(z)]
    return T


def load_artifact_rgb(path: Path) -> np.ndarray:
    image_bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)

    if image_bgr is None:
        raise RuntimeError(f"Could not read RGB artifact: {path}")

    return cv2.cvtColor(
        image_bgr,
        cv2.COLOR_BGR2RGB,
    )


def rgb_mae(a: np.ndarray, b: np.ndarray) -> float:
    if a.shape != b.shape:
        return float("inf")

    return float(
        np.mean(
            np.abs(
                a.astype(np.float32)
                - b.astype(np.float32)
            )
        )
    )


def find_matching_rollout_step(
    trajectory,
    artifact_rgb: np.ndarray,
    get_trajectory_step,
) -> tuple[int, float]:
    matches: list[tuple[float, int]] = []

    try:
        n_steps = len(trajectory)
    except TypeError as exc:
        raise RuntimeError(
            "Could not determine rollout trajectory length. "
            "Pass --step explicitly."
        ) from exc

    for step_index in range(n_steps):
        try:
            step = get_trajectory_step(
                trajectory,
                step_index,
            )
            obs = step["obs"]
            image_bgr, _, _, _ = get_gripper_rgbd(obs)

            candidate_rgb = cv2.cvtColor(
                np.asarray(image_bgr, dtype=np.uint8),
                cv2.COLOR_BGR2RGB,
            )

            matches.append(
                (
                    rgb_mae(
                        candidate_rgb,
                        artifact_rgb,
                    ),
                    step_index,
                )
            )
        except Exception:
            continue

    if not matches:
        raise RuntimeError(
            "Could not find any rollout step containing a supported "
            "eye-in-hand RGB image. Pass --step explicitly."
        )

    matches.sort()

    print("[PKL] Best RGB matches:")
    for error, index in matches[:5]:
        print(
            f"      step={index:3d}  RGB_MAE={error:.6f}"
        )

    best_error, best_step = matches[0]
    return int(best_step), float(best_error)


def load_saved_semantic_point(
    case_dir: Path,
    live_molmo: bool,
) -> np.ndarray | None:
    """
    Load the semantic point recorded during the real robot run.

    When available, this lets the replay isolate the downstream
    GraspGen-X pipeline without requiring a live GraspMolmo server.
    """

    if live_molmo:
        print(
            "[MOLMO] Live GraspMolmo inference enabled."
        )
        return None

    plan_json = (
        case_dir
        / "grasp_plan.json"
    )

    if not plan_json.exists():
        print(
            "[MOLMO] No grasp_plan.json found; "
            "falling back to live GraspMolmo."
        )
        return None

    payload = json.loads(
        plan_json.read_text(
            encoding="utf-8"
        )
    )

    point = payload.get(
        "semantic_point_px"
    )

    if point is None:
        print(
            "[MOLMO] grasp_plan.json has no semantic_point_px; "
            "falling back to live GraspMolmo."
        )
        return None

    point = np.asarray(
        point,
        dtype=np.float32,
    )

    if point.shape != (2,):
        raise RuntimeError(
            "semantic_point_px in grasp_plan.json "
            f"has invalid shape: {point.shape}"
        )

    if not np.isfinite(
        point
    ).all():
        raise RuntimeError(
            "semantic_point_px contains non-finite values."
        )

    print(
        "[MOLMO] Reusing real-run semantic point:",
        point,
    )

    return point


def build_current_planner(
    args,
    saved_semantic_point: np.ndarray | None,
):
    """
    Construct the current production GraspGenXPlanner.

    All shared inference arguments are passed directly because the
    final planner exposes them through **v1_kwargs.

    If a semantic point from the real run is available, GraspMolmo
    construction is bypassed for this offline replay only.
    """

    from ai_controller.utils.grasping.graspgenx.planner import (
        GraspGenXPlanner,
    )

    planner_kwargs = {
        "server_host": args.server_host,
        "server_port": args.server_port,
        "gripper_name": args.gripper_name,
        "num_grasps": args.num_grasps,
        "grasp_threshold": args.grasp_threshold,
        "topk_num_grasps": -1,
        "seed": args.seed,
        "semantic_radius_px": args.semantic_radius_px,
    }

    if saved_semantic_point is None:
        # Full replay, including live GraspMolmo.
        planner = GraspGenXPlanner(
            **planner_kwargs
        )

    else:
        # GraspMolmoClient currently performs a server check during
        # construction. For a replay using the already-recorded
        # semantic point, there is no reason to require that server.
        from unittest.mock import patch

        from ai_controller.utils.grasping.graspgenx import (
            inference_core,
        )

        class OfflineGraspMolmoClient:
            def __init__(
                self,
                *args,
                **kwargs,
            ):
                del args, kwargs

        with patch.object(
            inference_core,
            "GraspMolmoClient",
            OfflineGraspMolmoClient,
        ):
            planner = GraspGenXPlanner(
                **planner_kwargs
            )

        point = np.asarray(
            saved_semantic_point,
            dtype=np.float32,
        ).copy()

        def fixed_predict_semantic_point(
            *,
            rgb_image,
            grasp_instruction,
        ):
            del rgb_image
            del grasp_instruction

            return point.copy()

        planner._predict_semantic_point = (
            fixed_predict_semantic_point
        )

    print(
        "[PLANNER] server:",
        f"{args.server_host}:{args.server_port}",
    )

    print(
        "[PLANNER] num grasps:",
        planner.num_grasps,
    )

    print(
        "[PLANNER] grasp threshold:",
        planner.grasp_threshold,
    )

    print(
        "[PLANNER] seed:",
        planner.seed,
    )

    print(
        "[PLANNER] semantic radius:",
        planner.semantic_radius_px,
    )

    print(
        "[PLANNER] semantic source:",
        (
            "saved real-run point"
            if saved_semantic_point is not None
            else "live GraspMolmo"
        ),
    )

    return planner


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Replay an exact real GraspGen-X RGB-D case offline "
            "using the current local planner implementation."
        )
    )

    parser.add_argument(
        "--case-dir",
        type=Path,
        required=True,
        help=(
            "Directory containing input_rgb.png and input_depth.npy "
            "from the real grasp artifact."
        ),
    )

    parser.add_argument(
        "--pkl",
        type=Path,
        required=True,
        help="Rollout PKL used to recover the current TCP pose.",
    )

    parser.add_argument(
        "--step",
        type=int,
        default=None,
        help=(
            "Rollout step. If omitted, the script automatically "
            "matches input_rgb.png against the eye-in-hand images in the PKL."
        ),
    )

    parser.add_argument(
        "--instruction",
        type=str,
        required=True,
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
    )

    parser.add_argument(
        "--repo-root",
        type=Path,
        default=DEFAULT_REPO_ROOT,
    )

    parser.add_argument(
        "--semantic-radius-px",
        type=float,
        default=50.0,
    )

    # Production table_0 used in the current real setup:
    # base_link -> table_0 = [0, 0.612, -0.120], yaw=180 deg.
    parser.add_argument(
        "--table-x",
        type=float,
        default=0.0,
    )
    parser.add_argument(
        "--table-y",
        type=float,
        default=0.612,
    )
    parser.add_argument(
        "--table-z",
        type=float,
        default=-0.120,
    )
    parser.add_argument(
        "--table-yaw-deg",
        type=float,
        default=180.0,
    )

    parser.add_argument(
        "--server-host",
        type=str,
        default="127.0.0.1",
    )
    parser.add_argument(
        "--server-port",
        type=int,
        default=8782,
    )
    parser.add_argument(
        "--gripper-name",
        type=str,
        default="robotiq_2f_85",
    )
    parser.add_argument(
        "--num-grasps",
        type=int,
        default=200,
    )
    parser.add_argument(
        "--grasp-threshold",
        type=float,
        default=-1.0,
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
    )

    parser.add_argument(
        "--live-molmo",
        action="store_true",
        help=(
            "Call GraspMolmo again instead of reusing semantic_point_px "
            "from the real run's grasp_plan.json."
        ),
    )

    args = parser.parse_args()

    args.case_dir = args.case_dir.expanduser().resolve()
    args.pkl = args.pkl.expanduser().resolve()
    args.repo_root = args.repo_root.expanduser().resolve()

    if args.output_dir is None:
        args.output_dir = (
            args.case_dir
            / (
                "offline_replay_"
                f"r{args.semantic_radius_px:g}px"
            )
        )

    args.output_dir = (
        args.output_dir
        .expanduser()
        .resolve()
    )

    args.output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    rgb_path = (
        args.case_dir
        / "input_rgb.png"
    )

    depth_path = (
        args.case_dir
        / "input_depth.npy"
    )

    if not rgb_path.exists():
        raise FileNotFoundError(rgb_path)

    if not depth_path.exists():
        raise FileNotFoundError(depth_path)

    if not args.pkl.exists():
        raise FileNotFoundError(args.pkl)

    install_project_imports(
        args.repo_root
    )

    from inspect_pkl import (  # noqa: E402
        get_trajectory_step,
        load_pickle,
    )

    print("=" * 78)
    print("GRASPGEN-X REAL CASE OFFLINE REPLAY")
    print("=" * 78)
    print("[CASE]       ", args.case_dir)
    print("[PKL]        ", args.pkl)
    print("[INSTRUCTION]", args.instruction)
    print("[OUTPUT]     ", args.output_dir)

    image_rgb = load_artifact_rgb(
        rgb_path
    )

    depth = np.asarray(
        np.load(
            depth_path
        ),
        dtype=np.float32,
    )

    if depth.shape != image_rgb.shape[:2]:
        raise RuntimeError(
            "Artifact RGB/depth dimensions do not match: "
            f"RGB={image_rgb.shape[:2]}, depth={depth.shape}"
        )

    print(
        "[RGB-D] shape:",
        image_rgb.shape,
        depth.shape,
    )

    data = load_pickle(
        args.pkl
    )

    trajectory = data[
        "traj"
    ]

    if args.step is None:
        step_index, match_error = (
            find_matching_rollout_step(
                trajectory=trajectory,
                artifact_rgb=image_rgb,
                get_trajectory_step=(
                    get_trajectory_step
                ),
            )
        )

        print(
            "[PKL] Auto-selected step:",
            step_index,
            f"(RGB MAE={match_error:.6f})",
        )
    else:
        step_index = int(
            args.step
        )
        print(
            "[PKL] Forced step:",
            step_index,
        )

    step = get_trajectory_step(
        trajectory,
        step_index,
    )

    obs = step[
        "obs"
    ]

    if "eef_pos" not in obs:
        raise RuntimeError(
            "Selected rollout observation has no 'eef_pos'."
        )

    if "eef_quat" not in obs:
        raise RuntimeError(
            "Selected rollout observation has no 'eef_quat'."
        )

    current_tcp_position = np.asarray(
        obs[
            "eef_pos"
        ],
        dtype=np.float64,
    )

    current_tcp_orientation = np.asarray(
        obs[
            "eef_quat"
        ],
        dtype=np.float64,
    )

    T_base_tcp = pose_to_transform(
        current_tcp_position,
        current_tcp_orientation,
    )

    T_base_camera = (
        T_base_tcp
        @ T_TCP_CAMERA
    )

    T_base_table = make_base_to_table(
        x=args.table_x,
        y=args.table_y,
        z=args.table_z,
        yaw_deg=args.table_yaw_deg,
    )

    print(
        "[TF] current TCP position:",
        current_tcp_position,
    )
    print(
        "[TF] current TCP quaternion xyzw:",
        current_tcp_orientation,
    )
    print(
        "[TF] table_0 translation:",
        T_base_table[
            :3,
            3,
        ],
    )

    reused_semantic_point = (
        load_saved_semantic_point(
            case_dir=args.case_dir,
            live_molmo=args.live_molmo,
        )
    )

    planner = build_current_planner(
        args=args,
        saved_semantic_point=(
            reused_semantic_point
        ),
    )

    result = planner.plan(
        rgb_image=image_rgb,
        depth_image=depth,
        camera_matrix=DEFAULT_K,
        T_base_camera=T_base_camera,
        T_base_table=T_base_table,
        current_tcp_position=(
            current_tcp_position
        ),
        current_tcp_orientation=(
            current_tcp_orientation
        ),
        grasp_instruction=(
            args.instruction
        ),
        artifacts_dir=(
            args.output_dir
        ),
    )

    summary = {
        "source_case_dir": str(
            args.case_dir
        ),
        "source_pkl": str(
            args.pkl
        ),
        "rollout_step": int(
            step_index
        ),
        "instruction": str(
            args.instruction
        ),
        "semantic_radius_px": float(
            args.semantic_radius_px
        ),
        "T_base_table": (
            T_base_table.tolist()
        ),
        "reused_semantic_point": (
            None
            if reused_semantic_point is None
            else reused_semantic_point.tolist()
        ),
        "result": {
            "semantic_point_px": (
                np.asarray(
                    result.semantic_point_px
                ).tolist()
            ),
            "tcp_position_base": (
                np.asarray(
                    result.tcp_position_base
                ).tolist()
            ),
            "tcp_orientation_base": (
                np.asarray(
                    result.tcp_orientation_base
                ).tolist()
            ),
            "approach_direction_base": (
                np.asarray(
                    result.approach_direction_base
                ).tolist()
            ),
            "confidence": float(
                result.confidence
            ),
        },
    }

    summary_path = (
        args.output_dir
        / "replay_summary.json"
    )

    summary_path.write_text(
        json.dumps(
            summary,
            indent=2,
        ),
        encoding="utf-8",
    )

    print()
    print("=" * 78)
    print("REPLAY RESULT")
    print("=" * 78)
    print(
        "Semantic point:",
        result.semantic_point_px,
    )
    print(
        "TCP position:",
        result.tcp_position_base,
    )
    print(
        "TCP orientation:",
        result.tcp_orientation_base,
    )
    print(
        "Approach:",
        result.approach_direction_base,
    )
    print(
        "Confidence:",
        result.confidence,
    )
    print(
        "Artifacts:",
        args.output_dir,
    )
    print(
        "Summary:",
        summary_path,
    )
    print("=" * 78)


if __name__ == "__main__":
    main()
