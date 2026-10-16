
"""GraspGen-X V2: real model inference on a previously recorded RGB-D frame.

Runs GraspMolmo -> SAM -> GraspGen-X -> V2 geometry -> artifacts.
No ROS node, MoveIt, gripper command, or physical robot execution.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess

import cv2
import numpy as np
import yaml

from ai_controller.utils.grasping.grasp_planner_factory import (
    build_grasp_planner,
)
from ai_controller.utils.grasping.graspgenx import (
    test_graspgenx_plan_integration as v1_helpers,
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__
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
        "--instruction",
        required=True,
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(
            "ai_controller/ai_controller/models/seedo_controller/"
            "config/seedo_controller.yaml"
        ),
    )
    parser.add_argument(
        "--base-to-table-transform",
        type=Path,
        default=None,
        help=(
            "Optional real base-to-table YAML; "
            "otherwise fit from saved depth"
        ),
    )
    parser.add_argument(
        "--skip-visualization",
        action="store_true",
    )

    args = parser.parse_args()

    pkl = args.pkl.expanduser().resolve()
    output = args.output_dir.expanduser().resolve()

    if not pkl.is_file():
        raise FileNotFoundError(pkl)

    if not args.config.is_file():
        raise FileNotFoundError(args.config)

    if output.exists() and any(output.iterdir()):
        raise FileExistsError(
            f"Output folder is not empty: {output}. "
            "Use a new run directory."
        )

    output.mkdir(
        parents=True,
        exist_ok=True,
    )

    artifacts = (
        output
        / "grasp_planner"
        / "action_00"
    )
    artifacts.mkdir(
        parents=True,
        exist_ok=True,
    )

    # ----------------------------------------------------------
    # 1. Load recorded observation
    # ----------------------------------------------------------
    data = v1_helpers.load_pickle(pkl)

    step = v1_helpers.get_trajectory_step(
        data["traj"],
        args.step,
    )

    observation = step["obs"]

    bgr, depth, rgb_key, depth_key = (
        v1_helpers.get_gripper_rgbd(
            observation
        )
    )

    rgb = cv2.cvtColor(
        np.asarray(bgr),
        cv2.COLOR_BGR2RGB,
    )

    depth = np.asarray(
        depth,
        dtype=np.float32,
    )

    pos = np.asarray(
        observation["eef_pos"],
        dtype=float,
    ).reshape(3)

    quat = np.asarray(
        observation["eef_quat"],
        dtype=float,
    ).reshape(4)

    T_base_tcp = (
        v1_helpers.pose_to_transform(
            pos,
            quat,
        )
    )

    T_base_camera = (
        T_base_tcp @ v1_helpers.T_TCP_CAMERA
    )

    K = v1_helpers.DEFAULT_K.copy()

    # ----------------------------------------------------------
    # 2. Table geometry
    # ----------------------------------------------------------
    if args.base_to_table_transform is not None:
        T_base_table = (
            v1_helpers.load_transform_yaml(
                args.base_to_table_transform
            )
        )
        table_source = "provided_production_transform"

    else:
        points_camera = (
            v1_helpers.depth_to_camera_point_cloud(
                depth=depth,
                K=K,
                min_depth=0.15,
                max_depth=0.60,
                border_fraction=0.05,
            )
        )

        points_base = (
            v1_helpers.transform_points(
                points_camera,
                T_base_camera,
            )
        )

        normal, _, table_origin, _ = (
            v1_helpers.estimate_horizontal_table_plane(
                points_base,
                seed=42,
                iterations=1200,
                distance_threshold_m=0.003,
                min_abs_normal_z=0.80,
            )
        )

        T_base_table = (
            v1_helpers.table_transform_from_plane(
                normal,
                table_origin,
            )
        )

        table_source = "depth_plane_fit"

    # ----------------------------------------------------------
    # 3. Construct the real V2 planner
    # ----------------------------------------------------------
    config = yaml.safe_load(
        args.config.read_text(encoding="utf-8")
    )

    settings = dict(
        config["grasp_planner"]
    )

    settings["backend"] = "graspgenx_v2"

    # Explicitly disabled. The test uses plan_offline(),
    # never the normal SeeDo execution entry point.
    settings[
        "graspgenx_v2_unsafe_enable_robot_commands"
    ] = False

    planner = build_grasp_planner(settings)

    if not hasattr(planner, "plan_offline"):
        raise TypeError(
            "Factory did not construct the V2 offline planner"
        )

    if planner.v2_unsafe_enable_robot_commands:
        raise RuntimeError(
            "Robot-command override must remain disabled"
        )

    print(
        "[V2 FULL] Source:",
        pkl,
        "step:",
        args.step,
        flush=True,
    )
    print(
        "[V2 FULL] Target instruction:",
        args.instruction,
        flush=True,
    )
    print(
        "[V2 FULL] Table source:",
        table_source,
        flush=True,
    )
    print(
        "[V2 FULL] Artifacts:",
        artifacts,
        flush=True,
    )
    print(
        "[V2 FULL] Re-running GraspMolmo, SAM and GraspGen-X",
        flush=True,
    )

    # ----------------------------------------------------------
    # 4. Full new inference, without robot execution
    # ----------------------------------------------------------
    result = planner.plan_offline(
        rgb_image=rgb,
        depth_image=depth,
        camera_matrix=K,
        T_base_camera=T_base_camera,
        T_base_table=T_base_table,
        current_tcp_position=pos,
        current_tcp_orientation=quat,
        grasp_instruction=args.instruction,
        artifacts_dir=artifacts,
    )

    info = {
        "source_pkl": str(pkl),
        "source_step": args.step,
        "image_key": rgb_key,
        "depth_key": depth_key,
        "grasp_instruction": args.instruction,
        "table_source": table_source,
        "camera_matrix": K.tolist(),
        "T_base_camera": T_base_camera.tolist(),
        "T_base_table": T_base_table.tolist(),
        "tcp_before_position_base": pos.tolist(),
        "tcp_before_quaternion_xyzw": quat.tolist(),
        "predicted_tcp_position_base": (
            np.asarray(
                result.tcp_position_base
            ).tolist()
        ),
        "predicted_tcp_quaternion_xyzw": (
            np.asarray(
                result.tcp_orientation_base
            ).tolist()
        ),
        "confidence": float(
            result.confidence
        ),
        "status": "OFFLINE_PLAN_RETURNED_NOT_EXECUTABLE",
    }

    (
        output / "inference_run.json"
    ).write_text(
        json.dumps(info, indent=2),
        encoding="utf-8",
    )

    print(
        "[V2 FULL] TCP pose:",
        info["predicted_tcp_position_base"],
    )
    print(
        "[V2 FULL] TCP quaternion:",
        info["predicted_tcp_quaternion_xyzw"],
    )

    # ----------------------------------------------------------
    # 5. Render the new prediction
    # ----------------------------------------------------------
    if not args.skip_visualization:
        command = [
            str(planner.v2_worker_python),
            "-m",
            "ai_controller.utils.grasping.graspgenx_v2.offline_replay",
            "--case-dir", str(output),
            "--urdf", str(planner.v2_driver_urdf),
            "--mesh-share", str(planner.v2_robotiq_mesh_share),
            "--open-q", str(planner.v2_gripper_open_q),
            "--max-expensive-checks",
            str(planner.v2_max_expensive_checks),
            "--current-quat",
            *(str(x) for x in quat),
            "--out-dir",
            str(output / "visualization"),
        ]

        env = os.environ.copy()
        env["PYTHONDONTWRITEBYTECODE"] = "1"

        subprocess.run(
            command,
            env=env,
            check=True,
        )

    print(
        "[PASS] V2 full offline inference finished; "
        "no robot commands sent."
    )
    print(
        "[V2 FULL] Output:",
        output,
    )


if __name__ == "__main__":
    main()
