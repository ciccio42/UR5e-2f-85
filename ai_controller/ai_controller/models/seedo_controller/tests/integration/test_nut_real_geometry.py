from __future__ import annotations

import importlib
import json
import pickle
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

from scipy.spatial.transform import Rotation

from ai_controller.models.seedo_controller.motion_layer import (
    SeeDoMotionLayer,
)
from ai_controller.models.seedo_controller.scene_perceiver import (
    ScenePerceiver,
)

from ai_controller.script_controller.vision import (
    ARUCO_TO_TABLE0_ROTATION,
    ARUCO_TO_TABLE0_TRANSLATION,
)


# =====================================================================
# Artifact discovery
# =====================================================================


def _find_unique(
    root: Path,
    filename: str,
) -> Path:

    matches = sorted(
        root.rglob(
            filename
        )
    )

    if len(matches) != 1:
        raise RuntimeError(
            f"Expected exactly one {filename!r} inside "
            f"{root}, found {len(matches)}:\n"
            + "\n".join(
                str(path)
                for path in matches
            )
        )

    return matches[0]


def _find_optional_unique(
    root: Path,
    filename: str,
) -> Path | None:

    matches = sorted(
        root.rglob(
            filename
        )
    )

    if not matches:
        return None

    if len(matches) != 1:
        raise RuntimeError(
            f"Expected at most one {filename!r} inside "
            f"{root}, found {len(matches)}:\n"
            + "\n".join(
                str(path)
                for path in matches
            )
        )

    return matches[0]


def _find_rollout_pkl(
    case_dir: Path,
) -> Path:

    candidates = sorted(
        case_dir.rglob(
            "*.pkl"
        )
    )

    if not candidates:
        raise RuntimeError(
            "No saved rollout .pkl exists inside "
            f"{case_dir}."
        )

    direct = [
        path
        for path in candidates
        if path.parent == case_dir
    ]

    if len(direct) == 1:
        return direct[0]

    if len(candidates) == 1:
        return candidates[0]

    raise RuntimeError(
        "Could not uniquely determine the rollout PKL "
        f"inside {case_dir}:\n"
        + "\n".join(
            str(path)
            for path in candidates
        )
    )


# =====================================================================
# Generic readers
# =====================================================================


def _load_json(
    path: Path,
) -> dict:

    payload = json.loads(
        path.read_text(
            encoding="utf-8"
        )
    )

    if not isinstance(
        payload,
        dict,
    ):
        raise RuntimeError(
            f"Expected JSON object in {path}."
        )

    return payload


def _load_rgb(
    path: Path,
) -> np.ndarray:

    bgr = cv2.imread(
        str(path),
        cv2.IMREAD_COLOR,
    )

    if bgr is None:
        raise RuntimeError(
            f"Could not read image: {path}"
        )

    return cv2.cvtColor(
        bgr,
        cv2.COLOR_BGR2RGB,
    )


def _load_binary_mask(
    path: Path,
) -> np.ndarray:

    image = cv2.imread(
        str(path),
        cv2.IMREAD_GRAYSCALE,
    )

    if image is None:
        raise RuntimeError(
            f"Could not read mask: {path}"
        )

    return (
        image
        > 0
    )


# =====================================================================
# Rollout PKL
# =====================================================================


def _add_dataset_collector_scripts_to_path():
    """
    Make the Trajectory class used by dataset_collector_pkg
    available to pickle.
    """

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
            "Could not locate dataset_collector_pkg."
        ) from exc

    if not scripts_dir.is_dir():
        raise RuntimeError(
            "dataset_collector_pkg scripts directory "
            f"does not exist: {scripts_dir}"
        )

    scripts_dir_str = str(
        scripts_dir
    )

    if scripts_dir_str not in sys.path:
        sys.path.append(
            scripts_dir_str
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
    pkl_path: Path,
) -> dict:

    _add_dataset_collector_scripts_to_path()

    with pkl_path.open(
        "rb"
    ) as stream:

        payload = _TrajectoryUnpickler(
            stream
        ).load()

    if not isinstance(
        payload,
        dict,
    ):
        raise RuntimeError(
            "Saved rollout root must be a dictionary."
        )

    trajectory = payload.get(
        "traj"
    )

    if trajectory is None:
        raise RuntimeError(
            f"{pkl_path} does not contain 'traj'."
        )

    if len(trajectory) == 0:
        raise RuntimeError(
            "Saved trajectory contains no steps."
        )

    first_step = trajectory[
        0
    ]

    if not isinstance(
        first_step,
        dict,
    ):
        raise RuntimeError(
            "Trajectory step 0 is not a dictionary."
        )

    observation = first_step.get(
        "obs"
    )

    if not isinstance(
        observation,
        dict,
    ):
        raise RuntimeError(
            "Trajectory step 0 does not contain "
            "an observation dictionary."
        )

    return observation


def _get_first_existing(
    mapping: dict,
    keys: tuple[str, ...],
):

    for key in keys:

        value = mapping.get(
            key
        )

        if value is not None:
            return (
                key,
                value,
            )

    return (
        None,
        None,
    )


def _decode_front_rgb(
    observation: dict,
) -> tuple[
    str,
    np.ndarray,
]:

    key, value = _get_first_existing(
        observation,
        (
            "camera_front_image",
            "front_camera_image",
        ),
    )

    if value is None:
        raise RuntimeError(
            "First rollout observation does not contain "
            "front-camera RGB."
        )

    image = np.asarray(
        value
    )

    if image.ndim == 1:

        bgr = cv2.imdecode(
            image.astype(
                np.uint8,
                copy=False,
            ),
            cv2.IMREAD_COLOR,
        )

        if bgr is None:
            raise RuntimeError(
                "Could not decode compressed front RGB."
            )

        image = cv2.cvtColor(
            bgr,
            cv2.COLOR_BGR2RGB,
        )

    elif (
        image.ndim == 3
        and image.shape[2] == 3
    ):
        image = image.astype(
            np.uint8,
            copy=False,
        )

    else:
        raise RuntimeError(
            "Unsupported front RGB representation: "
            f"{image.shape}"
        )

    return (
        str(key),
        image,
    )


def _decode_front_depth(
    observation: dict,
) -> tuple[
    str,
    np.ndarray,
]:

    key, value = _get_first_existing(
        observation,
        (
            "camera_front_depth",
            "front_camera_depth",
        ),
    )

    if value is None:
        raise RuntimeError(
            "First rollout observation does not contain "
            "front-camera depth."
        )

    depth = np.asarray(
        value,
        dtype=np.float64,
    )

    if depth.ndim != 2:
        raise RuntimeError(
            "Front depth must be 2D, "
            f"received {depth.shape}."
        )

    return (
        str(key),
        depth,
    )


# =====================================================================
# Primitive-plan target resolution
# =====================================================================


def _resolve_nut_targets(
    primitive_plan_payload: dict,
) -> tuple[
    str,
    str,
]:

    steps = primitive_plan_payload.get(
        "steps"
    )

    if not isinstance(
        steps,
        list,
    ):
        raise RuntimeError(
            "primitive_plan.json does not contain "
            "a valid steps list."
        )

    pick_targets = []
    place_targets = []

    for step in steps:

        if not isinstance(
            step,
            dict,
        ):
            continue

        name = str(
            step.get(
                "name",
                "",
            )
        ).strip().lower()

        arguments = step.get(
            "arguments",
            {},
        )

        if not isinstance(
            arguments,
            dict,
        ):
            continue

        target = str(
            arguments.get(
                "target",
                "",
            )
        ).strip()

        if not target:
            continue

        if name in {
            "reach",
            "approaching",
            "pick",
            "lift_up",
        }:
            pick_targets.append(
                target
            )

        if name in {
            "moving",
            "aligning",
            "inserting",
        }:
            place_targets.append(
                target
            )

    pick_unique = {
        target.casefold(): target
        for target in pick_targets
    }

    place_unique = {
        target.casefold(): target
        for target in place_targets
    }

    if len(
        pick_unique
    ) != 1:
        raise RuntimeError(
            "Could not uniquely resolve Nut-Assembly "
            f"pick target: {pick_targets}"
        )

    if len(
        place_unique
    ) != 1:
        raise RuntimeError(
            "Could not uniquely resolve Nut-Assembly "
            f"place target: {place_targets}"
        )

    return (
        next(
            iter(
                pick_unique.values()
            )
        ),
        next(
            iter(
                place_unique.values()
            )
        ),
    )


def _get_saved_object(
    raw_scene_payload: dict,
    object_id: str,
) -> dict:

    objects = raw_scene_payload.get(
        "objects"
    )

    if not isinstance(
        objects,
        list,
    ):
        raise RuntimeError(
            "raw_scene_state.json does not contain "
            "an objects list."
        )

    normalized = (
        object_id
        .strip()
        .casefold()
    )

    matches = [
        obj
        for obj in objects
        if (
            str(
                obj.get(
                    "object_id",
                    "",
                )
            )
            .strip()
            .casefold()
            == normalized
        )
    ]

    if len(matches) != 1:
        raise RuntimeError(
            f"Could not uniquely resolve saved object "
            f"{object_id!r}. Matches={len(matches)}."
        )

    return matches[
        0
    ]


# =====================================================================
# Front-camera artifacts
# =====================================================================


def _recover_front_camera_matrix(
    raw_scene_payload: dict,
) -> tuple[
    np.ndarray,
    float,
]:

    objects = raw_scene_payload.get(
        "objects"
    )

    if (
        not isinstance(
            objects,
            list,
        )
        or len(objects) < 3
    ):
        raise RuntimeError(
            "At least three saved ScenePerceiver objects "
            "are required to recover the front-camera matrix."
        )

    x_rows = []
    x_targets = []

    y_rows = []
    y_targets = []

    for obj in objects:

        pixel = np.asarray(
            obj.get(
                "pixel_coordinates"
            ),
            dtype=np.float64,
        )

        point = np.asarray(
            obj.get(
                "position_camera"
            ),
            dtype=np.float64,
        )

        if (
            pixel.shape != (2,)
            or point.shape != (3,)
        ):
            continue

        z = float(
            point[2]
        )

        if abs(
            z
        ) <= 1e-9:
            continue

        x_rows.append(
            [
                point[0] / z,
                1.0,
            ]
        )

        x_targets.append(
            pixel[0]
        )

        y_rows.append(
            [
                point[1] / z,
                1.0,
            ]
        )

        y_targets.append(
            pixel[1]
        )

    if len(
        x_rows
    ) < 3:
        raise RuntimeError(
            "Insufficient saved objects to recover "
            "front-camera intrinsics."
        )

    fx, cx = np.linalg.lstsq(
        np.asarray(
            x_rows,
            dtype=np.float64,
        ),
        np.asarray(
            x_targets,
            dtype=np.float64,
        ),
        rcond=None,
    )[0]

    fy, cy = np.linalg.lstsq(
        np.asarray(
            y_rows,
            dtype=np.float64,
        ),
        np.asarray(
            y_targets,
            dtype=np.float64,
        ),
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

    if (
        not np.isfinite(
            K
        ).all()
        or fx <= 0.0
        or fy <= 0.0
    ):
        raise RuntimeError(
            f"Recovered invalid front K:\n{K}"
        )

    errors = []

    for obj in objects:

        point = np.asarray(
            obj[
                "position_camera"
            ],
            dtype=np.float64,
        )

        pixel = np.asarray(
            obj[
                "pixel_coordinates"
            ],
            dtype=np.float64,
        )

        z = float(
            point[2]
        )

        if abs(
            z
        ) <= 1e-9:
            continue

        predicted = np.array(
            [
                fx * point[0] / z + cx,
                fy * point[1] / z + cy,
            ],
            dtype=np.float64,
        )

        errors.append(
            float(
                np.linalg.norm(
                    predicted
                    - pixel
                )
            )
        )

    rmse = float(
        np.sqrt(
            np.mean(
                np.square(
                    errors
                )
            )
        )
    )

    return (
        K,
        rmse,
    )


def _load_saved_front_calibration(
    path: Path,
) -> dict[str, np.ndarray]:

    payload = _load_json(
        path
    )

    calibration = payload.get(
        "perturbed_calibration"
    )

    if not isinstance(
        calibration,
        dict,
    ):
        raise RuntimeError(
            "camera_pose_noise.json does not contain "
            "'perturbed_calibration'."
        )

    position = np.asarray(
        calibration[
            "position_m"
        ],
        dtype=np.float64,
    )

    orientation = np.asarray(
        calibration[
            "orientation_matrix"
        ],
        dtype=np.float64,
    )

    if position.shape != (3,):
        raise RuntimeError(
            "Invalid saved front-camera position."
        )

    if orientation.shape != (
        3,
        3,
    ):
        raise RuntimeError(
            "Invalid saved front-camera rotation."
        )

    return {
        "position": position,
        "orientation_matrix": orientation,
    }


def _validate_front_transform_chain(
    *,
    raw_scene_payload: dict,
    camera_calibration: dict,
    T_base_table: np.ndarray,
) -> float:

    R_camera_aruco = np.asarray(
        camera_calibration[
            "orientation_matrix"
        ],
        dtype=np.float64,
    )

    t_camera_aruco = np.asarray(
        camera_calibration[
            "position"
        ],
        dtype=np.float64,
    )

    R_table_base = np.asarray(
        T_base_table[
            :3,
            :3,
        ],
        dtype=np.float64,
    )

    t_table_base = np.asarray(
        T_base_table[
            :3,
            3,
        ],
        dtype=np.float64,
    )

    errors = []

    for obj in raw_scene_payload[
        "objects"
    ]:

        camera_point = np.asarray(
            obj[
                "position_camera"
            ],
            dtype=np.float64,
        )

        saved_base = np.asarray(
            obj[
                "position_base"
            ],
            dtype=np.float64,
        )

        aruco_point = (
            R_camera_aruco
            @ camera_point
            + t_camera_aruco
        )

        table_point = (
            ARUCO_TO_TABLE0_ROTATION
            @ aruco_point
            + ARUCO_TO_TABLE0_TRANSLATION
        )

        reconstructed_base = (
            R_table_base
            @ table_point
            + t_table_base
        )

        errors.append(
            float(
                np.linalg.norm(
                    reconstructed_base
                    - saved_base
                )
            )
        )

    return float(
        max(
            errors
        )
    )


# =====================================================================
# Recover eye-in-hand intrinsics directly from GraspGen-X artifacts
# =====================================================================


def _recover_grasp_camera_matrix(
    *,
    target_mask: np.ndarray,
    depth: np.ndarray,
    target_point_cloud_camera: np.ndarray,
) -> tuple[
    np.ndarray,
    float,
    float,
]:

    mask = np.asarray(
        target_mask,
        dtype=bool,
    )

    depth = np.asarray(
        depth,
        dtype=np.float64,
    )

    point_cloud = np.asarray(
        target_point_cloud_camera,
        dtype=np.float64,
    )

    if mask.shape != depth.shape:
        raise RuntimeError(
            "Saved GraspGen-X mask/depth shapes differ: "
            f"mask={mask.shape}, depth={depth.shape}"
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
        point_cloud
    ):
        raise RuntimeError(
            "Saved target_point_cloud_camera length does not "
            "match the valid target-mask depth pixels. "
            f"pixels={len(u)}, points={len(point_cloud)}"
        )

    if (
        point_cloud.ndim != 2
        or point_cloud.shape[1] != 3
    ):
        raise RuntimeError(
            "target_point_cloud_camera must have "
            "shape (N, 3)."
        )

    depth_z = depth[
        v,
        u
    ]

    cloud_z = point_cloud[
        :,
        2,
    ]

    z_rmse = float(
        np.sqrt(
            np.mean(
                (
                    depth_z
                    - cloud_z
                ) ** 2
            )
        )
    )

    if z_rmse > 1e-5:
        raise RuntimeError(
            "Saved target point cloud ordering is not "
            "consistent with target_mask/input_depth. "
            f"Z RMSE={z_rmse:.9f} m."
        )

    z = point_cloud[
        :,
        2,
    ]

    valid_z = (
        np.abs(
            z
        )
        > 1e-9
    )

    point_cloud = point_cloud[
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

    z = point_cloud[
        :,
        2,
    ]

    normalized_x = (
        point_cloud[
            :,
            0,
        ]
        / z
    )

    normalized_y = (
        point_cloud[
            :,
            1,
        ]
        / z
    )

    fx, cx = np.linalg.lstsq(
        np.column_stack(
            (
                normalized_x,
                np.ones_like(
                    normalized_x
                ),
            )
        ),
        u,
        rcond=None,
    )[0]

    fy, cy = np.linalg.lstsq(
        np.column_stack(
            (
                normalized_y,
                np.ones_like(
                    normalized_y
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
        * normalized_x
        + cx
    )

    predicted_v = (
        fy
        * normalized_y
        + cy
    )

    reprojection_errors = np.sqrt(
        (
            predicted_u
            - u
        ) ** 2
        + (
            predicted_v
            - v
        ) ** 2
    )

    reprojection_rmse = float(
        np.sqrt(
            np.mean(
                reprojection_errors ** 2
            )
        )
    )

    if (
        not np.isfinite(
            K
        ).all()
        or fx <= 0.0
        or fy <= 0.0
    ):
        raise RuntimeError(
            f"Recovered invalid eye-in-hand K:\n{K}"
        )

    return (
        K,
        reprojection_rmse,
        z_rmse,
    )


# =====================================================================
# Saved GraspGen-X consistency
# =====================================================================


def _validate_saved_grasp_point_cloud(
    *,
    point_cloud_camera: np.ndarray,
    point_cloud_base: np.ndarray,
    T_base_camera: np.ndarray,
) -> float:

    camera_points = np.asarray(
        point_cloud_camera,
        dtype=np.float64,
    )

    saved_base = np.asarray(
        point_cloud_base,
        dtype=np.float64,
    )

    reconstructed_base = (
        camera_points
        @ T_base_camera[
            :3,
            :3,
        ].T
        + T_base_camera[
            :3,
            3,
        ]
    )

    if (
        reconstructed_base.shape
        != saved_base.shape
    ):
        raise RuntimeError(
            "Saved GraspGen-X camera/base point clouds "
            "have inconsistent shapes."
        )

    errors = np.linalg.norm(
        reconstructed_base
        - saved_base,
        axis=1,
    )

    return float(
        np.sqrt(
            np.mean(
                errors ** 2
            )
        )
    )


def _camera_to_aruco_from_saved_transforms(
    *,
    T_base_camera: np.ndarray,
    T_base_table: np.ndarray,
) -> np.ndarray:
    """
    Recover the camera -> raw ArUco transform expected by
    ScenePerceiver.refine_nut_hole().

    Saved GraspGen-X artifacts provide:
        camera -> base
        table_0 -> base

    while ScenePerceiver internally uses:
        camera -> ArUco -> table_0 -> base
    """

    T_base_camera = np.asarray(
        T_base_camera,
        dtype=np.float64,
    )

    T_base_table = np.asarray(
        T_base_table,
        dtype=np.float64,
    )

    T_table_camera = (
        np.linalg.inv(
            T_base_table
        )
        @ T_base_camera
    )

    T_table_aruco = np.eye(
        4,
        dtype=np.float64,
    )

    T_table_aruco[
        :3,
        :3,
    ] = (
        ARUCO_TO_TABLE0_ROTATION
    )

    T_table_aruco[
        :3,
        3,
    ] = (
        ARUCO_TO_TABLE0_TRANSLATION
    )

    T_aruco_camera = (
        np.linalg.inv(
            T_table_aruco
        )
        @ T_table_camera
    )

    return T_aruco_camera


# =====================================================================
# Scene geometry helpers
# =====================================================================


def _height_above_table(
    point_base: np.ndarray,
    T_base_table: np.ndarray,
) -> float:

    point = np.asarray(
        point_base,
        dtype=np.float64,
    )

    table_origin = np.asarray(
        T_base_table[
            :3,
            3,
        ],
        dtype=np.float64,
    )

    table_axis = np.asarray(
        T_base_table[
            :3,
            2,
        ],
        dtype=np.float64,
    )

    table_axis /= np.linalg.norm(
        table_axis
    )

    if table_axis[
        2
    ] < 0.0:
        table_axis = (
            -table_axis
        )

    return float(
        np.dot(
            point
            - table_origin,
            table_axis,
        )
    )


# =====================================================================
# Artifact-only Nut-hole refinement
# =====================================================================


def _build_artifact_replay_perceiver(
    *,
    T_base_camera: np.ndarray,
    T_base_table: np.ndarray,
) -> ScenePerceiver:
    """
    Construct only the geometric state needed by refine_nut_hole().

    No GroundingDINO, SAM, VLM, checkpoint or API is initialized.
    """

    perceiver = ScenePerceiver.__new__(
        ScenePerceiver
    )

    T_aruco_camera = (
        _camera_to_aruco_from_saved_transforms(
            T_base_camera=(
                T_base_camera
            ),
            T_base_table=(
                T_base_table
            ),
        )
    )

    perceiver._last_runtime_camera_calibration = {
        "position": np.asarray(
            T_aruco_camera[
                :3,
                3,
            ],
            dtype=np.float64,
        ).copy(),

        "orientation_matrix": np.asarray(
            T_aruco_camera[
                :3,
                :3,
            ],
            dtype=np.float64,
        ).copy(),
    }

    perceiver._last_base_to_table_rotation = (
        np.asarray(
            T_base_table[
                :3,
                :3,
            ],
            dtype=np.float64,
        ).copy()
    )

    perceiver._last_base_to_table_translation = (
        np.asarray(
            T_base_table[
                :3,
                3,
            ],
            dtype=np.float64,
        ).copy()
    )

    return perceiver


# =====================================================================
# Motion-layer artifact handoff
# =====================================================================


def _build_motion_layer() -> SeeDoMotionLayer:

    return SeeDoMotionLayer(
        grasp_orientation=(
            0.0,
            0.0,
            0.0,
            1.0,
        ),

        min_step=0.02,
        reach_hover_height=0.18,
        approach_z_offset=0.0,
        release_height_offset=0.10,
        lift_height=0.15,

        gripper_open_position=0.1,
        gripper_closed_position=0.8,

        object_y_offset=-0.04,

        grasp_planning_enabled=True,

        pregrasp_distance=0.08,
        grasp_depth_offset=0.005,
        retreat_distance=0.04,

        nut_assembly_config={
            "peg_clearance_m": 0.020,
            "initial_entry_m": 0.005,
            "insertion_step_m": 0.005,
            "release_bottom_height_m": 0.010,
            "rotation_step_deg": 5.0,
            "auto_release": False,
        },
    )


# =====================================================================
# Main test
# =====================================================================


def test_nut_real_geometry_from_saved_case(
    case_dir: Path,
):

    # =============================================================
    # Discover artifacts
    # =============================================================

    rollout_pkl = (
        _find_rollout_pkl(
            case_dir
        )
    )

    raw_scene_path = (
        _find_unique(
            case_dir,
            "raw_scene_state.json",
        )
    )

    camera_noise_path = (
        _find_unique(
            case_dir,
            "camera_pose_noise.json",
        )
    )

    primitive_plan_path = (
        _find_unique(
            case_dir,
            "primitive_plan.json",
        )
    )

    diagnostics_path = (
        _find_unique(
            case_dir,
            "graspgenx_diagnostics.npz",
        )
    )

    grasp_plan_path = (
        _find_unique(
            case_dir,
            "grasp_plan.json",
        )
    )

    grasp_rgb_path = (
        _find_unique(
            case_dir,
            "input_rgb.png",
        )
    )

    grasp_depth_path = (
        _find_unique(
            case_dir,
            "input_depth.npy",
        )
    )

    grasp_mask_path = (
        _find_unique(
            case_dir,
            "target_mask.png",
        )
    )

    saved_place_refinement_path = (
        _find_optional_unique(
            case_dir,
            "nut_place_refinements.json",
        )
    )

    # =============================================================
    # Load saved symbolic artifacts
    # =============================================================

    raw_scene_payload = (
        _load_json(
            raw_scene_path
        )
    )

    primitive_plan_payload = (
        _load_json(
            primitive_plan_path
        )
    )

    grasp_plan_payload = (
        _load_json(
            grasp_plan_path
        )
    )

    (
        pick_target,
        place_target,
    ) = _resolve_nut_targets(
        primitive_plan_payload
    )

    saved_pick_object = (
        _get_saved_object(
            raw_scene_payload,
            pick_target,
        )
    )

    saved_place_object = (
        _get_saved_object(
            raw_scene_payload,
            place_target,
        )
    )

    # =============================================================
    # Load the FIRST front RGB-D actually recorded by this test.
    #
    # We do NOT rerun perception on it.
    # =============================================================

    first_observation = (
        _load_first_rollout_observation(
            rollout_pkl
        )
    )

    (
        front_rgb_key,
        front_rgb,
    ) = _decode_front_rgb(
        first_observation
    )

    (
        front_depth_key,
        front_depth,
    ) = _decode_front_depth(
        first_observation
    )

    if (
        front_rgb.shape[
            :2
        ]
        != front_depth.shape
    ):
        raise RuntimeError(
            "First saved front RGB/depth pair has "
            "inconsistent dimensions. "
            f"rgb={front_rgb.shape[:2]}, "
            f"depth={front_depth.shape}"
        )

    # =============================================================
    # Recover front-camera K from raw_scene_state.json.
    # No model inference.
    # =============================================================

    (
        front_K,
        front_reprojection_rmse_px,
    ) = _recover_front_camera_matrix(
        raw_scene_payload
    )

    assert (
        front_reprojection_rmse_px
        < 0.25
    ), (
        "Saved ScenePerceiver points/pixels do not define "
        "a consistent front-camera matrix. "
        f"RMSE={front_reprojection_rmse_px:.4f}px."
    )

    front_calibration = (
        _load_saved_front_calibration(
            camera_noise_path
        )
    )

    # =============================================================
    # Load real GraspGen-X artifacts
    # =============================================================

    diagnostics = np.load(
        diagnostics_path
    )

    required_npz_keys = {
        "semantic_point_px",
        "target_mask",
        "target_point_cloud_camera",
        "target_point_cloud_base",
        "tcp_position_base",
        "tcp_orientation_base",
        "approach_direction_base",
        "T_base_camera",
        "T_base_table",
        "confidences",
        "selected_index",
    }

    missing_npz_keys = (
        required_npz_keys
        - set(
            diagnostics.files
        )
    )

    if missing_npz_keys:
        raise RuntimeError(
            "GraspGen-X diagnostics are missing keys: "
            f"{sorted(missing_npz_keys)}"
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

    if (
        T_base_camera.shape
        != (
            4,
            4,
        )
    ):
        raise RuntimeError(
            "Invalid saved T_base_camera."
        )

    if (
        T_base_table.shape
        != (
            4,
            4,
        )
    ):
        raise RuntimeError(
            "Invalid saved T_base_table."
        )

    real_tcp_position = np.asarray(
        diagnostics[
            "tcp_position_base"
        ],
        dtype=np.float64,
    )

    real_tcp_orientation = np.asarray(
        diagnostics[
            "tcp_orientation_base"
        ],
        dtype=np.float64,
    )

    real_tcp_orientation /= np.linalg.norm(
        real_tcp_orientation
    )

    real_approach = np.asarray(
        diagnostics[
            "approach_direction_base"
        ],
        dtype=np.float64,
    )

    selected_index = int(
        diagnostics[
            "selected_index"
        ]
    )

    confidences = np.asarray(
        diagnostics[
            "confidences"
        ],
        dtype=np.float64,
    )

    real_confidence = float(
        confidences[
            selected_index
        ]
    )

    # =============================================================
    # Validate grasp_plan.json against diagnostics.npz
    # =============================================================

    np.testing.assert_allclose(
        np.asarray(
            grasp_plan_payload[
                "tcp_position_base"
            ],
            dtype=np.float64,
        ),
        real_tcp_position,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        np.asarray(
            grasp_plan_payload[
                "tcp_orientation_base"
            ],
            dtype=np.float64,
        ),
        real_tcp_orientation,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        np.asarray(
            grasp_plan_payload[
                "approach_direction_base"
            ],
            dtype=np.float64,
        ),
        real_approach,
        atol=1e-9,
    )

    assert float(
        grasp_plan_payload[
            "confidence"
        ]
    ) == pytest.approx(
        real_confidence,
        abs=1e-9,
    )

    # =============================================================
    # Validate FRONT-camera transform chain
    # =============================================================

    front_transform_error_m = (
        _validate_front_transform_chain(
            raw_scene_payload=(
                raw_scene_payload
            ),
            camera_calibration=(
                front_calibration
            ),
            T_base_table=(
                T_base_table
            ),
        )
    )

    assert (
        front_transform_error_m
        < 0.005
    ), (
        "Saved front-camera/table transforms do not "
        "reconstruct raw_scene_state positions. "
        f"Maximum error="
        f"{front_transform_error_m * 1000.0:.3f} mm."
    )

    # =============================================================
    # Load exact eye-in-hand RGB-D + SAM target mask
    # already saved by the REAL GraspGen-X run.
    # =============================================================

    grasp_rgb = (
        _load_rgb(
            grasp_rgb_path
        )
    )

    grasp_depth = np.asarray(
        np.load(
            grasp_depth_path
        ),
        dtype=np.float64,
    )

    mask_png = (
        _load_binary_mask(
            grasp_mask_path
        )
    )

    mask_npz = (
        np.asarray(
            diagnostics[
                "target_mask"
            ]
        )
        .astype(bool)
    )

    if (
        grasp_rgb.shape[
            :2
        ]
        != grasp_depth.shape
    ):
        raise RuntimeError(
            "Saved eye-in-hand RGB/depth dimensions "
            "do not match."
        )

    if (
        mask_npz.shape
        != grasp_depth.shape
    ):
        raise RuntimeError(
            "Saved GraspGen-X mask/depth dimensions "
            "do not match."
        )

    np.testing.assert_array_equal(
        mask_png,
        mask_npz,
    )

    # =============================================================
    # Recover the EXACT eye-in-hand camera matrix from:
    #
    # mask + depth + saved camera-frame point cloud.
    #
    # This works because GraspGen-X generated the saved point cloud
    # deterministically using np.where(mask & valid_depth).
    # =============================================================

    target_cloud_camera = np.asarray(
        diagnostics[
            "target_point_cloud_camera"
        ],
        dtype=np.float64,
    )

    target_cloud_base = np.asarray(
        diagnostics[
            "target_point_cloud_base"
        ],
        dtype=np.float64,
    )

    (
        grasp_K,
        grasp_reprojection_rmse_px,
        grasp_depth_cloud_z_rmse_m,
    ) = _recover_grasp_camera_matrix(
        target_mask=mask_npz,
        depth=grasp_depth,
        target_point_cloud_camera=(
            target_cloud_camera
        ),
    )

    assert (
        grasp_reprojection_rmse_px
        < 1e-3
    ), (
        "Could not recover eye-in-hand intrinsics "
        "accurately from saved artifacts. "
        f"RMSE={grasp_reprojection_rmse_px:.6f}px."
    )

    assert (
        grasp_depth_cloud_z_rmse_m
        < 1e-5
    )

    # =============================================================
    # Validate camera -> base point-cloud transform
    # =============================================================

    grasp_cloud_transform_rmse_m = (
        _validate_saved_grasp_point_cloud(
            point_cloud_camera=(
                target_cloud_camera
            ),
            point_cloud_base=(
                target_cloud_base
            ),
            T_base_camera=(
                T_base_camera
            ),
        )
    )

    assert (
        grasp_cloud_transform_rmse_m
        < 1e-5
    ), (
        "Saved GraspGen-X point clouds are not "
        "consistent with T_base_camera. "
        f"RMSE={grasp_cloud_transform_rmse_m:.9f}m."
    )

    # =============================================================
    # ARTIFACT-ONLY REAL NUT-HOLE REFINEMENT
    #
    # No DINO.
    # No SAM.
    # No VLM.
    # No API.
    #
    # refine_nut_hole() receives exactly the mask/depth that the
    # actual test saved before executing the grasp.
    # =============================================================

    perceiver = (
        _build_artifact_replay_perceiver(
            T_base_camera=(
                T_base_camera
            ),
            T_base_table=(
                T_base_table
            ),
        )
    )

    hole = perceiver.refine_nut_hole(
        mask=mask_npz,
        depth_image=grasp_depth,
        camera_matrix=grasp_K,
    )

    assert hole[
        "accepted"
    ] is True

    hole_top = np.asarray(
        hole[
            "hole_center_base"
        ],
        dtype=np.float64,
    )

    hole_bottom = np.asarray(
        hole[
            "hole_bottom_center_base"
        ],
        dtype=np.float64,
    )

    hole_axis = np.asarray(
        hole[
            "hole_axis_base"
        ],
        dtype=np.float64,
    )

    hole_axis /= np.linalg.norm(
        hole_axis
    )

    ring_thickness_m = float(
        np.dot(
            hole_top
            - hole_bottom,
            hole_axis,
        )
    )

    assert (
        ring_thickness_m
        > 0.0
    )

    assert ring_thickness_m == pytest.approx(
        float(
            hole[
                "nut_thickness_estimate_m"
            ]
        ),
        abs=0.003,
    )

    # =============================================================
    # REAL Controller/MotionLayer-style grasp-relative handoff
    # =============================================================

    motion_layer = (
        _build_motion_layer()
    )

    motion_layer.reset(
        current_position=(
            real_tcp_position
        ),
        current_orientation=(
            real_tcp_orientation
        ),
        artifacts_dir=None,
        task_type="nut_assembly",
    )

    motion_layer.set_grasp_pose(
        position=(
            real_tcp_position
        ),
        orientation=(
            real_tcp_orientation
        ),
        approach_direction=(
            real_approach
        ),
        backend_name="graspgenx",
    )

    motion_layer.set_nut_grasp_geometry(
        target_id=pick_target,

        hole_center_base=(
            hole_top
        ),

        hole_bottom_center_base=(
            hole_bottom
        ),

        hole_axis_base=(
            hole_axis
        ),

        object_radius_from_hole_m=float(
            hole[
                "object_radius_from_hole_m"
            ]
        ),
    )

    # =============================================================
    # Verify base -> TCP-local -> base round trip using the REAL grasp
    # =============================================================

    (
        reconstructed_hole_top,
        reconstructed_hole_bottom,
        reconstructed_hole_axis,
    ) = (
        motion_layer
        ._nut_current_hole_geometry()
    )

    hole_top_roundtrip_error_m = float(
        np.linalg.norm(
            reconstructed_hole_top
            - hole_top
        )
    )

    hole_bottom_roundtrip_error_m = float(
        np.linalg.norm(
            reconstructed_hole_bottom
            - hole_bottom
        )
    )

    hole_axis_roundtrip_dot = float(
        np.dot(
            reconstructed_hole_axis,
            hole_axis,
        )
    )

    assert (
        hole_top_roundtrip_error_m
        < 1e-9
    )

    assert (
        hole_bottom_roundtrip_error_m
        < 1e-9
    )

    assert (
        hole_axis_roundtrip_dot
        > 1.0
        - 1e-10
    )

    # =============================================================
    # Information about the real destination that IS available
    # in legacy artifacts.
    #
    # IMPORTANT:
    # raw_scene_state stores only one representative point.
    # It does NOT contain the SAM mask required to replay
    # refine_peg_axis().
    # =============================================================

    saved_pick_position_base = np.asarray(
        saved_pick_object[
            "position_base"
        ],
        dtype=np.float64,
    )

    saved_place_position_base = np.asarray(
        saved_place_object[
            "position_base"
        ],
        dtype=np.float64,
    )

    pick_saved_height_m = (
        _height_above_table(
            saved_pick_position_base,
            T_base_table,
        )
    )

    place_saved_height_m = (
        _height_above_table(
            saved_place_position_base,
            T_base_table,
        )
    )

    peg_artifact_status = {
        "available": False,

        "reason": (
            "This legacy test case does not contain the "
            "ScenePerceiver SAM mask for the destination peg. "
            "raw_scene_state.json stores only the representative "
            "pixel/3D point, which is insufficient to replay "
            "refine_peg_axis() without generating new perception "
            "data."
        ),

        "target_id": (
            place_target
        ),

        "saved_pixel": (
            saved_place_object[
                "pixel_coordinates"
            ]
        ),

        "saved_position_camera": (
            saved_place_object[
                "position_camera"
            ]
        ),

        "saved_position_base": (
            saved_place_object[
                "position_base"
            ]
        ),

        "saved_point_height_above_table_m": (
            place_saved_height_m
        ),
    }

    # If a future run contains the new artifact, simply record it.
    # Still no inference is performed.
    saved_place_refinement = None

    if (
        saved_place_refinement_path
        is not None
    ):
        saved_place_refinement = (
            _load_json(
                saved_place_refinement_path
            )
        )

        peg_artifact_status = {
            "available": True,

            "source": str(
                saved_place_refinement_path
            ),

            "target_id": (
                place_target
            ),

            "saved_refinement": (
                saved_place_refinement
            ),
        }

    # =============================================================
    # Report
    # =============================================================

    report_dir = (
        case_dir
        / "offline_artifact_geometry_validation"
    )

    report_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    report = {
        "case_dir": str(
            case_dir
        ),

        "mode": (
            "artifact_only_no_inference"
        ),

        "sources": {
            "rollout_pkl": str(
                rollout_pkl
            ),

            "raw_scene_state": str(
                raw_scene_path
            ),

            "camera_pose_noise": str(
                camera_noise_path
            ),

            "primitive_plan": str(
                primitive_plan_path
            ),

            "grasp_plan": str(
                grasp_plan_path
            ),

            "graspgenx_diagnostics": str(
                diagnostics_path
            ),

            "grasp_rgb": str(
                grasp_rgb_path
            ),

            "grasp_depth": str(
                grasp_depth_path
            ),

            "grasp_target_mask": str(
                grasp_mask_path
            ),
        },

        "targets": {
            "pick": (
                pick_target
            ),

            "place": (
                place_target
            ),
        },

        "rollout_first_front_frame": {
            "rgb_key": (
                front_rgb_key
            ),

            "depth_key": (
                front_depth_key
            ),

            "rgb_shape": list(
                front_rgb.shape
            ),

            "depth_shape": list(
                front_depth.shape
            ),
        },

        "front_camera": {
            "recovered_K": (
                front_K.tolist()
            ),

            "reprojection_rmse_px": (
                front_reprojection_rmse_px
            ),

            "transform_chain_max_error_m": (
                front_transform_error_m
            ),
        },

        "grasp_camera": {
            "recovered_K": (
                grasp_K.tolist()
            ),

            "reprojection_rmse_px": (
                grasp_reprojection_rmse_px
            ),

            "depth_point_cloud_z_rmse_m": (
                grasp_depth_cloud_z_rmse_m
            ),

            "camera_to_base_point_cloud_rmse_m": (
                grasp_cloud_transform_rmse_m
            ),
        },

        "saved_scene_points": {
            "pick_position_base": (
                saved_pick_position_base.tolist()
            ),

            "pick_height_above_table_m": (
                pick_saved_height_m
            ),

            "place_position_base": (
                saved_place_position_base.tolist()
            ),

            "place_height_above_table_m": (
                place_saved_height_m
            ),
        },

        "real_graspgenx": {
            "tcp_position_base": (
                real_tcp_position.tolist()
            ),

            "tcp_orientation_base": (
                real_tcp_orientation.tolist()
            ),

            "approach_direction_base": (
                real_approach.tolist()
            ),

            "confidence": (
                real_confidence
            ),
        },

        "real_ring_hole_refinement": (
            hole
        ),

        "grasp_relative_geometry": {
            "hole_top_local": (
                motion_layer
                ._nut_hole_top_local
                .tolist()
            ),

            "hole_bottom_local": (
                motion_layer
                ._nut_hole_bottom_local
                .tolist()
            ),

            "hole_axis_local": (
                motion_layer
                ._nut_hole_axis_local
                .tolist()
            ),

            "object_sweep_radius_m": float(
                motion_layer
                ._nut_object_sweep_radius_m
            ),

            "top_roundtrip_error_m": (
                hole_top_roundtrip_error_m
            ),

            "bottom_roundtrip_error_m": (
                hole_bottom_roundtrip_error_m
            ),

            "axis_roundtrip_dot": (
                hole_axis_roundtrip_dot
            ),
        },

        "destination_peg_geometry": (
            peg_artifact_status
        ),
    }

    report_path = (
        report_dir
        / "artifact_geometry_report.json"
    )

    report_path.write_text(
        json.dumps(
            report,
            indent=2,
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )

    # =============================================================
    # Human-readable output
    # =============================================================

    print()
    print("=" * 90)
    print("REAL NUT-ASSEMBLY ARTIFACT-ONLY REPLAY")
    print("=" * 90)

    print(
        "Case:",
        case_dir,
    )

    print(
        "Mode: ARTIFACT ONLY - NO NEW INFERENCE"
    )

    print()

    print(
        "Pick target:",
        pick_target,
    )

    print(
        "Place target:",
        place_target,
    )

    print()

    print(
        "First saved front RGB:",
        front_rgb_key,
        front_rgb.shape,
    )

    print(
        "First saved front depth:",
        front_depth_key,
        front_depth.shape,
    )

    print()

    print(
        "Recovered FRONT K:"
    )

    print(
        front_K
    )

    print(
        "Front reprojection RMSE:",
        f"{front_reprojection_rmse_px:.6f} px",
    )

    print(
        "Front transform max error:",
        f"{front_transform_error_m * 1000.0:.3f} mm",
    )

    print()

    print(
        "Recovered EYE-IN-HAND K:"
    )

    print(
        grasp_K
    )

    print(
        "Eye reprojection RMSE:",
        f"{grasp_reprojection_rmse_px:.9f} px",
    )

    print(
        "Depth/cloud Z RMSE:",
        f"{grasp_depth_cloud_z_rmse_m * 1000.0:.6f} mm",
    )

    print(
        "Camera/base cloud RMSE:",
        f"{grasp_cloud_transform_rmse_m * 1000.0:.6f} mm",
    )

    print()

    print(
        "REAL GraspGen-X TCP:",
        real_tcp_position,
    )

    print(
        "REAL GraspGen-X quaternion:",
        real_tcp_orientation,
    )

    print(
        "REAL GraspGen-X approach:",
        real_approach,
    )

    print(
        "REAL GraspGen-X confidence:",
        f"{real_confidence:.6f}",
    )

    print()

    print(
        "REAL hole top:",
        hole_top,
    )

    print(
        "REAL hole bottom:",
        hole_bottom,
    )

    print(
        "REAL hole axis:",
        hole_axis,
    )

    print(
        "REAL ring thickness:",
        f"{ring_thickness_m * 1000.0:.2f} mm",
    )

    print(
        "REAL observed radius from hole:",
        f"{hole['object_radius_from_hole_m'] * 1000.0:.2f} mm",
    )

    print(
        "Top plane RMS:",
        f"{hole['top_plane_rms_m'] * 1000.0:.3f} mm",
    )

    print(
        "Support MAD:",
        f"{hole['support_mad_m'] * 1000.0:.3f} mm",
    )

    print()

    print(
        "Hole top in REAL grasp TCP:",
        motion_layer
        ._nut_hole_top_local,
    )

    print(
        "Hole bottom in REAL grasp TCP:",
        motion_layer
        ._nut_hole_bottom_local,
    )

    print(
        "Hole axis in REAL grasp TCP:",
        motion_layer
        ._nut_hole_axis_local,
    )

    print(
        "Conservative object sweep radius:",
        f"{motion_layer._nut_object_sweep_radius_m * 1000.0:.2f} mm",
    )

    print(
        "Hole top round-trip error:",
        f"{hole_top_roundtrip_error_m * 1000.0:.9f} mm",
    )

    print(
        "Hole bottom round-trip error:",
        f"{hole_bottom_roundtrip_error_m * 1000.0:.9f} mm",
    )

    print(
        "Hole axis round-trip dot:",
        f"{hole_axis_roundtrip_dot:.12f}",
    )

    print()

    print(
        "Saved pick ScenePerceiver point:",
        saved_pick_position_base,
    )

    print(
        "Saved pick point height above table:",
        f"{pick_saved_height_m * 1000.0:.2f} mm",
    )

    print(
        "Saved destination point:",
        saved_place_position_base,
    )

    print(
        "Saved destination point height above table:",
        f"{place_saved_height_m * 1000.0:.2f} mm",
    )

    print()

    if peg_artifact_status[
        "available"
    ]:

        print(
            "Destination peg refinement: "
            "AVAILABLE FROM SAVED ARTIFACT"
        )

        print(
            "Source:",
            saved_place_refinement_path,
        )

    else:

        print(
            "Destination peg refinement: "
            "NOT AVAILABLE IN LEGACY ARTIFACTS"
        )

        print(
            "Reason:",
            peg_artifact_status[
                "reason"
            ],
        )

    print()

    print(
        "Report:",
        report_path,
    )

    print("=" * 90)