from __future__ import annotations

import argparse
import importlib
import json
import pickle
import shutil
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


# =====================================================================
# Basic helpers
# =====================================================================


def _require_file(
    path: Path,
    description: str,
) -> Path:

    path = (
        Path(path)
        .expanduser()
        .resolve()
    )

    if not path.is_file():
        raise FileNotFoundError(
            f"Missing {description}: {path}"
        )

    return path


def _write_json(
    path: Path,
    payload: Any,
) -> None:

    def jsonable(
        value: Any,
    ) -> Any:

        if isinstance(
            value,
            np.ndarray,
        ):
            return value.tolist()

        if isinstance(
            value,
            (
                np.integer,
                np.floating,
                np.bool_,
            ),
        ):
            return value.item()

        if isinstance(
            value,
            Path,
        ):
            return str(
                value
            )

        if isinstance(
            value,
            dict,
        ):
            return {
                str(key): jsonable(
                    item
                )
                for key, item
                in value.items()
            }

        if isinstance(
            value,
            (
                list,
                tuple,
            ),
        ):
            return [
                jsonable(
                    item
                )
                for item in value
            ]

        return value

    path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    with path.open(
        "w",
        encoding="utf-8",
    ) as stream:
        json.dump(
            jsonable(
                payload
            ),
            stream,
            indent=2,
            ensure_ascii=False,
            allow_nan=False,
        )


# =====================================================================
# Rollout loading
# =====================================================================


def _add_dataset_collector_scripts_to_path() -> None:

    try:
        from ament_index_python.packages import (
            get_package_share_directory,
        )

        scripts_dir = (
            Path(
                get_package_share_directory(
                    "dataset_collector_pkg"
                )
            )
            / "scripts"
        )

    except Exception as exc:
        raise RuntimeError(
            "Could not locate dataset_collector_pkg. "
            "Run this script inside the normal SeeDo ROS2 container."
        ) from exc

    if not scripts_dir.is_dir():
        raise RuntimeError(
            "dataset_collector_pkg scripts directory "
            f"does not exist: {scripts_dir}"
        )

    scripts_str = str(
        scripts_dir
    )

    if scripts_str not in sys.path:
        sys.path.append(
            scripts_str
        )


class _TrajectoryUnpickler(
    pickle.Unpickler
):

    def find_class(
        self,
        module,
        name,
    ):

        if (
            module.startswith(
                "multi_task_il"
            )
            and name == "Trajectory"
        ):
            return importlib.import_module(
                "savers"
            ).Trajectory

        return super().find_class(
            module,
            name,
        )


def _load_first_rollout_observation(
    path: Path,
) -> dict[str, Any]:

    _add_dataset_collector_scripts_to_path()

    with path.open(
        "rb"
    ) as stream:
        payload = _TrajectoryUnpickler(
            stream
        ).load()

    trajectory = payload.get(
        "traj"
    )

    if trajectory is None:
        raise RuntimeError(
            "Rollout does not contain 'traj'."
        )

    if len(
        trajectory
    ) == 0:
        raise RuntimeError(
            "Rollout trajectory is empty."
        )

    step = trajectory[
        0
    ]

    observation = step.get(
        "obs"
    )

    if not isinstance(
        observation,
        dict,
    ):
        raise RuntimeError(
            "First rollout step does not contain 'obs'."
        )

    return observation


def _first_existing(
    mapping: dict[str, Any],
    keys: tuple[str, ...],
):

    for key in keys:

        if key in mapping:
            return (
                key,
                mapping[
                    key
                ],
            )

    return (
        None,
        None,
    )


# =====================================================================
# Front scene extraction
# =====================================================================


def _extract_front_bgr(
    observation: dict[str, Any],
) -> tuple[
    str,
    np.ndarray,
]:

    key, value = _first_existing(
        observation,
        (
            "camera_front_image",
            "front_camera_image",
        ),
    )

    if value is None:
        raise RuntimeError(
            "Rollout does not contain front RGB."
        )

    image = np.asarray(
        value
    )

    if image.ndim == 1:

        image_bgr = cv2.imdecode(
            image.astype(
                np.uint8,
                copy=False,
            ),
            cv2.IMREAD_COLOR,
        )

        if image_bgr is None:
            raise RuntimeError(
                "Could not decode rollout front RGB."
            )

    elif (
        image.ndim == 3
        and image.shape[2] == 3
    ):

        # Dataset Collector stores OpenCV/BGR frames.
        image_bgr = image.astype(
            np.uint8,
            copy=False,
        )

    else:

        raise RuntimeError(
            "Unsupported rollout front RGB representation: "
            f"{image.shape}"
        )

    return (
        str(key),
        image_bgr,
    )


def _extract_front_depth(
    observation: dict[str, Any],
) -> tuple[
    str,
    np.ndarray,
]:

    key, value = _first_existing(
        observation,
        (
            "camera_front_depth",
            "front_camera_depth",
        ),
    )

    if value is None:
        raise RuntimeError(
            "Rollout does not contain front depth."
        )

    depth = np.asarray(
        value,
        dtype=np.float32,
    )

    if depth.ndim != 2:
        raise RuntimeError(
            "Front depth must have shape (H, W)."
        )

    return (
        str(key),
        depth,
    )


def _extract_robot_state(
    observation: dict[str, Any],
) -> dict[str, list[float]]:

    position = np.asarray(
        observation[
            "eef_pos"
        ],
        dtype=np.float64,
    )

    orientation = np.asarray(
        observation[
            "eef_quat"
        ],
        dtype=np.float64,
    )

    if position.shape != (3,):
        raise RuntimeError(
            "Saved eef_pos must have shape (3,)."
        )

    if orientation.shape != (4,):
        raise RuntimeError(
            "Saved eef_quat must have shape (4,)."
        )

    orientation_norm = float(
        np.linalg.norm(
            orientation
        )
    )

    if orientation_norm <= 1e-12:
        raise RuntimeError(
            "Saved EEF quaternion is invalid."
        )

    orientation = (
        orientation
        / orientation_norm
    )

    return {
        "eef_pos": (
            position.tolist()
        ),

        "eef_quat": (
            orientation.tolist()
        ),
    }


# =====================================================================
# Front K from the already-validated derived artifact
# =====================================================================


def _load_front_K(
    path: Path,
) -> np.ndarray:

    path = _require_file(
        path,
        "derived front camera info",
    )

    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        payload = json.load(
            stream
        )

    if "matrix" in payload:

        K = np.asarray(
            payload[
                "matrix"
            ],
            dtype=np.float64,
        )

    elif "k" in payload:

        K = np.asarray(
            payload[
                "k"
            ],
            dtype=np.float64,
        ).reshape(
            3,
            3,
        )

    else:

        raise RuntimeError(
            "Could not find 'matrix' or 'k' in "
            f"{path}"
        )

    if K.shape != (
        3,
        3,
    ):
        raise RuntimeError(
            f"Invalid front K shape: {K.shape}"
        )

    return K


# =====================================================================
# Recover eye-in-hand K from the frozen GraspGenX input
# =====================================================================


def _recover_gripper_K(
    diagnostics,
    depth: np.ndarray,
) -> tuple[
    np.ndarray,
    float,
]:

    mask = np.asarray(
        diagnostics[
            "target_mask"
        ]
    ).astype(bool)

    cloud = np.asarray(
        diagnostics[
            "target_point_cloud_camera"
        ],
        dtype=np.float64,
    )

    valid = (
        mask
        & np.isfinite(
            depth
        )
        & (
            depth >= 0.15
        )
        & (
            depth <= 0.60
        )
    )

    v, u = np.where(
        valid
    )

    if len(
        u
    ) != len(
        cloud
    ):
        raise RuntimeError(
            "Cannot recover eye-in-hand K: "
            "mask/depth/cloud point counts differ."
        )

    z = cloud[
        :,
        2
    ]

    valid_z = (
        np.abs(
            z
        )
        > 1e-9
    )

    cloud = cloud[
        valid_z
    ]

    u = u[
        valid_z
    ].astype(
        np.float64
    )

    v = v[
        valid_z
    ].astype(
        np.float64
    )

    z = cloud[
        :,
        2
    ]

    xn = (
        cloud[
            :,
            0
        ]
        / z
    )

    yn = (
        cloud[
            :,
            1
        ]
        / z
    )

    fx, cx = np.linalg.lstsq(
        np.column_stack(
            (
                xn,
                np.ones_like(
                    xn
                ),
            )
        ),
        u,
        rcond=None,
    )[0]

    fy, cy = np.linalg.lstsq(
        np.column_stack(
            (
                yn,
                np.ones_like(
                    yn
                ),
            )
        ),
        v,
        rcond=None,
    )[0]

    K = np.array(
        [
            [
                fx,
                0.0,
                cx,
            ],
            [
                0.0,
                fy,
                cy,
            ],
            [
                0.0,
                0.0,
                1.0,
            ],
        ],
        dtype=np.float64,
    )

    predicted_u = (
        fx
        * xn
        + cx
    )

    predicted_v = (
        fy
        * yn
        + cy
    )

    rmse_px = float(
        np.sqrt(
            np.mean(
                (
                    predicted_u
                    - u
                ) ** 2
                + (
                    predicted_v
                    - v
                ) ** 2
            )
        )
    )

    return (
        K,
        rmse_px,
    )


# =====================================================================
# Recover the TCP pose at eye-in-hand capture time
# =====================================================================


def _make_transform(
    *,
    translation: np.ndarray,
    rotation_matrix: np.ndarray,
) -> np.ndarray:

    transform = np.eye(
        4,
        dtype=np.float64,
    )

    transform[
        :3,
        :3,
    ] = rotation_matrix

    transform[
        :3,
        3,
    ] = translation

    return transform


def _recover_gripper_capture_robot_state(
    *,
    T_base_camera: np.ndarray,
    tcp_camera_translation_m: np.ndarray,
    tcp_camera_rpy_deg: np.ndarray,
) -> tuple[
    dict[str, list[float]],
    np.ndarray,
]:

    # Static transform measured on the real robot:
    #
    # tcp_link -> zed_mini_left_camera_frame_optical
    #
    # T_base_camera =
    #     T_base_tcp @ T_tcp_camera
    #
    # therefore:
    #
    # T_base_tcp =
    #     T_base_camera @ inv(T_tcp_camera)

    tcp_camera_rotation = (
        Rotation.from_euler(
            "xyz",
            tcp_camera_rpy_deg,
            degrees=True,
        )
        .as_matrix()
    )

    T_tcp_camera = _make_transform(
        translation=(
            tcp_camera_translation_m
        ),

        rotation_matrix=(
            tcp_camera_rotation
        ),
    )

    T_base_tcp = (
        T_base_camera
        @ np.linalg.inv(
            T_tcp_camera
        )
    )

    tcp_position = (
        T_base_tcp[
            :3,
            3,
        ]
    )

    tcp_quaternion = (
        Rotation.from_matrix(
            T_base_tcp[
                :3,
                :3,
            ]
        )
        .as_quat()
    )

    tcp_quaternion /= np.linalg.norm(
        tcp_quaternion
    )

    return (
        {
            "eef_pos": (
                tcp_position.tolist()
            ),

            "eef_quat": (
                tcp_quaternion.tolist()
            ),
        },
        T_tcp_camera,
    )


# =====================================================================
# Main preparation
# =====================================================================


def prepare_case(
    *,
    source_case: Path,
    output_dir: Path,
    demo_path: Path | None,
    tcp_camera_translation_m: np.ndarray,
    tcp_camera_rpy_deg: np.ndarray,
    force: bool,
) -> None:

    source_case = (
        Path(source_case)
        .expanduser()
        .resolve()
    )

    output_dir = (
        Path(output_dir)
        .expanduser()
        .resolve()
    )

    rollout_path = _require_file(
        source_case
        / "rollouts"
        / "seedo_controller"
        / "pick_place"
        / "task_00"
        / "traj_000.pkl",
        "test38 rollout",
    )

    front_camera_info_source = _require_file(
        source_case
        / "derived_artifacts"
        / "front_camera_info.json",
        "derived front camera info",
    )

    grasp_dir = (
        source_case
        / "grasp_planner"
        / "action_00"
    )

    grasp_rgb_path = _require_file(
        grasp_dir
        / "input_rgb.png",
        "GraspGenX RGB input",
    )

    grasp_depth_path = _require_file(
        grasp_dir
        / "input_depth.npy",
        "GraspGenX depth input",
    )

    grasp_diagnostics_path = _require_file(
        grasp_dir
        / "graspgenx_diagnostics.npz",
        "GraspGenX diagnostics",
    )

    grasp_plan_path = _require_file(
        grasp_dir
        / "grasp_plan.json",
        "reference GraspGenX plan",
    )

    if output_dir.exists():

        existing_entries = list(
            output_dir.iterdir()
        )

        if existing_entries and not force:
            raise RuntimeError(
                f"Output directory is not empty: {output_dir}\n"
                "Use --force to rebuild it."
            )

        if force:
            shutil.rmtree(
                output_dir
            )

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    front_dir = (
        output_dir
        / "front"
    )

    gripper_dir = (
        output_dir
        / "gripper"
    )

    reference_dir = (
        output_dir
        / "reference"
    )

    front_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    gripper_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    reference_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    # -------------------------------------------------------------
    # Front runtime scene from the first saved rollout observation.
    # -------------------------------------------------------------

    observation = (
        _load_first_rollout_observation(
            rollout_path
        )
    )

    (
        front_rgb_key,
        front_bgr,
    ) = _extract_front_bgr(
        observation
    )

    (
        front_depth_key,
        front_depth,
    ) = _extract_front_depth(
        observation
    )

    if (
        front_bgr.shape[:2]
        != front_depth.shape
    ):
        raise RuntimeError(
            "Front RGB/depth shapes differ: "
            f"rgb={front_bgr.shape[:2]}, "
            f"depth={front_depth.shape}."
        )

    front_rgb_output = (
        front_dir
        / "rgb.png"
    )

    saved = cv2.imwrite(
        str(
            front_rgb_output
        ),
        front_bgr,
    )

    if not saved:
        raise RuntimeError(
            "Could not save front RGB."
        )

    np.save(
        front_dir
        / "depth.npy",
        front_depth,
    )

    initial_robot_state = (
        _extract_robot_state(
            observation
        )
    )

    _write_json(
        output_dir
        / "initial_robot_state.json",
        initial_robot_state,
    )

    # -------------------------------------------------------------
    # Front intrinsics.
    # -------------------------------------------------------------

    front_K = _load_front_K(
        front_camera_info_source
    )

    _write_json(
        front_dir
        / "camera_info.json",
        {
            "k": (
                front_K
                .reshape(-1)
                .tolist()
            ),
        },
    )

    # -------------------------------------------------------------
    # Frozen GraspGenX artifacts.
    # -------------------------------------------------------------

    diagnostics = np.load(
        grasp_diagnostics_path
    )

    T_base_camera = np.asarray(
        diagnostics[
            "T_base_camera"
        ],
        dtype=np.float64,
    )

    T_base_table = np.asarray(
        diagnostics[
            "T_base_table"
        ],
        dtype=np.float64,
    )

    if T_base_camera.shape != (
        4,
        4,
    ):
        raise RuntimeError(
            "Invalid saved T_base_camera."
        )

    if T_base_table.shape != (
        4,
        4,
    ):
        raise RuntimeError(
            "Invalid saved T_base_table."
        )

    # Use the same table transform for initial front perception.
    _write_json(
        front_dir
        / "base_to_table_transform.json",
        {
            "rotation": (
                T_base_table[
                    :3,
                    :3,
                ]
            ),

            "translation": (
                T_base_table[
                    :3,
                    3,
                ]
            ),
        },
    )

    # -------------------------------------------------------------
    # Eye-in-hand scene.
    # -------------------------------------------------------------

    shutil.copy2(
        grasp_rgb_path,
        gripper_dir
        / "rgb.png",
    )

    shutil.copy2(
        grasp_depth_path,
        gripper_dir
        / "depth.npy",
    )

    gripper_depth = np.asarray(
        np.load(
            grasp_depth_path
        ),
        dtype=np.float32,
    )

    (
        gripper_K,
        gripper_K_rmse_px,
    ) = _recover_gripper_K(
        diagnostics,
        gripper_depth,
    )

    if gripper_K_rmse_px > 1e-3:
        raise RuntimeError(
            "Recovered eye-in-hand K is not consistent "
            "with the frozen GraspGenX artifact: "
            f"RMSE={gripper_K_rmse_px:.6f}px"
        )

    _write_json(
        gripper_dir
        / "camera_info.json",
        {
            "k": (
                gripper_K
                .reshape(-1)
                .tolist()
            ),
        },
    )

    np.save(
        gripper_dir
        / "T_base_camera.npy",
        T_base_camera,
    )

    np.save(
        gripper_dir
        / "T_base_table.npy",
        T_base_table,
    )

    # -------------------------------------------------------------
    # TCP pose associated with the eye-in-hand capture.
    # -------------------------------------------------------------

    (
        gripper_robot_state,
        T_tcp_camera,
    ) = _recover_gripper_capture_robot_state(
        T_base_camera=(
            T_base_camera
        ),

        tcp_camera_translation_m=(
            tcp_camera_translation_m
        ),

        tcp_camera_rpy_deg=(
            tcp_camera_rpy_deg
        ),
    )

    _write_json(
        gripper_dir
        / "robot_state.json",
        gripper_robot_state,
    )

    np.save(
        gripper_dir
        / "T_tcp_camera.npy",
        T_tcp_camera,
    )

    # -------------------------------------------------------------
    # Keep old outputs only as REFERENCE.
    #
    # The full-pipeline runner must never consume these.
    # -------------------------------------------------------------

    shutil.copy2(
        grasp_diagnostics_path,
        reference_dir
        / "test38_graspgenx_diagnostics.npz",
    )

    shutil.copy2(
        grasp_plan_path,
        reference_dir
        / "test38_grasp_plan.json",
    )

    source_primitive_plan = (
        source_case
        / "lmp_generator"
        / "primitive_plan.json"
    )

    if source_primitive_plan.is_file():

        shutil.copy2(
            source_primitive_plan,
            reference_dir
            / "test38_primitive_plan.json",
        )

    source_raw_scene = (
        source_case
        / "scene_perceiver"
        / "raw_scene_state.json"
    )

    if source_raw_scene.is_file():

        shutil.copy2(
            source_raw_scene,
            reference_dir
            / "test38_raw_scene_state.json",
        )

    source_pick_refinement = (
        source_case
        / "derived_artifacts"
        / "nut_pick_refinements.json"
    )

    if source_pick_refinement.is_file():

        shutil.copy2(
            source_pick_refinement,
            reference_dir
            / "test38_nut_pick_refinements.json",
        )

    source_place_refinement = (
        source_case
        / "derived_artifacts"
        / "nut_place_refinements.json"
    )

    if source_place_refinement.is_file():

        shutil.copy2(
            source_place_refinement,
            reference_dir
            / "test38_nut_place_refinements.json",
        )

    # -------------------------------------------------------------
    # Demo video.
    # -------------------------------------------------------------

    if demo_path is not None:

        demo_path = _require_file(
            demo_path,
            "demonstration video",
        )

        shutil.copy2(
            demo_path,
            output_dir
            / "demo.mp4",
        )

    # -------------------------------------------------------------
    # Manifest.
    # -------------------------------------------------------------

    manifest = {
        "source_case": (
            source_case
        ),

        "source_rollout": (
            rollout_path
        ),

        "front": {
            "rgb_rollout_key": (
                front_rgb_key
            ),

            "depth_rollout_key": (
                front_depth_key
            ),

            "rgb_shape": (
                front_bgr.shape
            ),

            "depth_shape": (
                front_depth.shape
            ),

            "camera_matrix": (
                front_K
            ),
        },

        "gripper": {
            "rgb_source": (
                grasp_rgb_path
            ),

            "depth_source": (
                grasp_depth_path
            ),

            "camera_matrix": (
                gripper_K
            ),

            "camera_matrix_recovery_rmse_px": (
                gripper_K_rmse_px
            ),

            "T_base_camera": (
                T_base_camera
            ),

            "T_base_table": (
                T_base_table
            ),

            "T_tcp_camera": (
                T_tcp_camera
            ),

            "tcp_camera_translation_m": (
                tcp_camera_translation_m
            ),

            "tcp_camera_rpy_deg": (
                tcp_camera_rpy_deg
            ),

            "recovered_robot_state": (
                gripper_robot_state
            ),
        },

        "important": (
            "Files under reference/ are old test38 outputs "
            "for comparison only. They must never be used as "
            "inputs by the full pipeline."
        ),
    }

    _write_json(
        output_dir
        / "case_manifest.json",
        manifest,
    )

    print()
    print("=" * 90)
    print("FULL NUT CASE PREPARED")
    print("=" * 90)

    print(
        "Source:",
        source_case,
    )

    print(
        "Output:",
        output_dir,
    )

    print()

    print(
        "Front RGB:",
        front_bgr.shape,
        f"(rollout key={front_rgb_key})",
    )

    print(
        "Front depth:",
        front_depth.shape,
        f"(rollout key={front_depth_key})",
    )

    print()

    print(
        "Front K:"
    )
    print(
        front_K
    )

    print()

    print(
        "Eye-in-hand K:"
    )
    print(
        gripper_K
    )

    print(
        "K recovery RMSE:",
        f"{gripper_K_rmse_px:.9f} px",
    )

    print()

    print(
        "Recovered eye-in-hand capture TCP:"
    )

    print(
        "  position:",
        gripper_robot_state[
            "eef_pos"
        ],
    )

    print(
        "  quaternion [x,y,z,w]:",
        gripper_robot_state[
            "eef_quat"
        ],
    )

    print()

    if demo_path is None:
        print(
            "WARNING: demo.mp4 has not been provided."
        )

        print(
            "Copy the desired Nut-Assembly demonstration to:"
        )

        print(
            output_dir
            / "demo.mp4"
        )

    else:
        print(
            "Demo:",
            output_dir
            / "demo.mp4",
        )

    print("=" * 90)


def main() -> None:

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--source-case",
        type=Path,
        default=Path(
            "/test_real/test_task00_38"
        ),
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(
            "/full_nut_case"
        ),
    )

    parser.add_argument(
        "--demo",
        type=Path,
        default=None,
        help=(
            "Optional Nut-Assembly demonstration video. "
            "If omitted, copy demo.mp4 manually afterwards."
        ),
    )

    parser.add_argument(
        "--tcp-camera-translation",
        type=float,
        nargs=3,
        default=(
            -0.021,
            -0.068,
            -0.145,
        ),
        metavar=(
            "X",
            "Y",
            "Z",
        ),
        help=(
            "tcp_link -> eye-in-hand optical-frame "
            "translation in metres."
        ),
    )

    parser.add_argument(
        "--tcp-camera-rpy-deg",
        type=float,
        nargs=3,
        default=(
            0.0,
            0.0,
            2.2,
        ),
        metavar=(
            "R",
            "P",
            "Y",
        ),
        help=(
            "tcp_link -> eye-in-hand optical-frame "
            "RPY in degrees."
        ),
    )

    parser.add_argument(
        "--force",
        action="store_true",
    )

    args = parser.parse_args()

    prepare_case(
        source_case=(
            args.source_case
        ),

        output_dir=(
            args.output_dir
        ),

        demo_path=(
            args.demo
        ),

        tcp_camera_translation_m=np.asarray(
            args.tcp_camera_translation,
            dtype=np.float64,
        ),

        tcp_camera_rpy_deg=np.asarray(
            args.tcp_camera_rpy_deg,
            dtype=np.float64,
        ),

        force=(
            args.force
        ),
    )


if __name__ == "__main__":
    main()