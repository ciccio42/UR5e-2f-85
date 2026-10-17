from __future__ import annotations

import hashlib
import importlib
import json
import pickle
import sys
from pathlib import Path
from typing import Any

import cv2
import matplotlib.pyplot as plt
import numpy as np
import pytest

from scipy.spatial.transform import Rotation

from ai_controller.models.seedo_controller.motion_layer import (
    SeeDoMotionLayer,
)
from ai_controller.models.seedo_controller.seedo_controller import (
    SeeDoController,
)
from ai_controller.utils.grasping.graspgenx.planner import (
    GraspGenXPlanResult,
)

from ai_controller.script_controller.vision import (
    ARUCO_TO_TABLE0_ROTATION,
    ARUCO_TO_TABLE0_TRANSLATION,
)

from results import (
    ActionPlanningResult,
    ActionStep,
    PrimitivePlan,
    PrimitiveStep,
    SceneObject,
    SceneState,
)


EXPECTED_PRIMITIVES = (
    "reach",
    "approaching",
    "pick",
    "lift_up",
    "moving",
    "aligning",
    "inserting",
)


# =====================================================================
# Basic IO
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


def _load_json(
    path: Path,
) -> dict[str, Any]:

    path = _require_file(
        path,
        "JSON artifact",
    )

    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        payload = json.load(
            stream
        )

    if not isinstance(
        payload,
        dict,
    ):
        raise RuntimeError(
            f"Expected JSON object in {path}."
        )

    return payload


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


def _sha256(
    path: Path,
) -> str:

    digest = hashlib.sha256()

    with path.open(
        "rb"
    ) as stream:

        while True:

            chunk = stream.read(
                1024 * 1024
            )

            if not chunk:
                break

            digest.update(
                chunk
            )

    return digest.hexdigest()


# =====================================================================
# Canonical real-case paths
# =====================================================================


def _case_paths(
    case_dir: Path,
) -> dict[str, Path]:

    case_dir = (
        Path(case_dir)
        .expanduser()
        .resolve()
    )

    paths = {
        "case_dir": case_dir,

        "rollout": (
            case_dir
            / "rollouts"
            / "seedo_controller"
            / "pick_place"
            / "task_00"
            / "traj_000.pkl"
        ),

        "raw_scene": (
            case_dir
            / "scene_perceiver"
            / "raw_scene_state.json"
        ),

        "camera_noise": (
            case_dir
            / "scene_perceiver"
            / "camera_pose_noise.json"
        ),

        "primitive_plan": (
            case_dir
            / "lmp_generator"
            / "primitive_plan.json"
        ),

        "grasp_plan": (
            case_dir
            / "grasp_planner"
            / "action_00"
            / "grasp_plan.json"
        ),

        "grasp_diagnostics": (
            case_dir
            / "grasp_planner"
            / "action_00"
            / "graspgenx_diagnostics.npz"
        ),

        "grasp_rgb": (
            case_dir
            / "grasp_planner"
            / "action_00"
            / "input_rgb.png"
        ),

        "grasp_depth": (
            case_dir
            / "grasp_planner"
            / "action_00"
            / "input_depth.npy"
        ),

        "grasp_mask": (
            case_dir
            / "grasp_planner"
            / "action_00"
            / "target_mask.png"
        ),

        "derived_dir": (
            case_dir
            / "derived_artifacts"
        ),

        "front_camera_info": (
            case_dir
            / "derived_artifacts"
            / "front_camera_info.json"
        ),

        "pick_refinements": (
            case_dir
            / "derived_artifacts"
            / "nut_pick_refinements.json"
        ),

        "place_refinements": (
            case_dir
            / "derived_artifacts"
            / "nut_place_refinements.json"
        ),

        "manifest": (
            case_dir
            / "derived_artifacts"
            / "derivation_manifest.json"
        ),

        "visual_debug": (
            case_dir
            / "derived_artifacts"
            / "visual_debug"
        ),
    }

    required = (
        "rollout",
        "raw_scene",
        "camera_noise",
        "primitive_plan",
        "grasp_plan",
        "grasp_diagnostics",
        "grasp_rgb",
        "grasp_depth",
        "grasp_mask",
        "front_camera_info",
        "pick_refinements",
        "place_refinements",
        "manifest",
    )

    for key in required:

        _require_file(
            paths[
                key
            ],
            key,
        )

    return paths


# =====================================================================
# Validate derived-artifact provenance
# =====================================================================


def _validate_manifest(
    paths: dict[str, Path],
) -> dict[str, Any]:

    manifest = _load_json(
        paths[
            "manifest"
        ]
    )

    if (
        manifest.get(
            "case_dir"
        )
        != str(
            paths[
                "case_dir"
            ]
        )
    ):
        raise RuntimeError(
            "Derived-artifact manifest belongs to "
            "a different real test case."
        )

    sources = manifest.get(
        "sources"
    )

    if not isinstance(
        sources,
        dict,
    ):
        raise RuntimeError(
            "Invalid derivation manifest: "
            "missing sources."
        )

    for name, entry in sources.items():

        if not isinstance(
            entry,
            dict,
        ):
            raise RuntimeError(
                f"Invalid manifest source {name!r}."
            )

        source_path = Path(
            entry[
                "path"
            ]
        )

        _require_file(
            source_path,
            f"manifest source {name}",
        )

        current_hash = _sha256(
            source_path
        )

        expected_hash = str(
            entry[
                "sha256"
            ]
        )

        if current_hash != expected_hash:
            raise RuntimeError(
                "Original source artifact changed after "
                "derived artifacts were generated: "
                f"{source_path}"
            )

    derived = manifest.get(
        "derived_artifacts"
    )

    if not isinstance(
        derived,
        dict,
    ):
        raise RuntimeError(
            "Invalid derivation manifest: "
            "missing derived_artifacts."
        )

    for name, entry in derived.items():

        artifact_path = Path(
            entry[
                "path"
            ]
        )

        _require_file(
            artifact_path,
            f"derived artifact {name}",
        )

        current_hash = _sha256(
            artifact_path
        )

        if (
            current_hash
            != str(
                entry[
                    "sha256"
                ]
            )
        ):
            raise RuntimeError(
                "Derived artifact changed after "
                "manifest generation: "
                f"{artifact_path}"
            )

    return manifest


# =====================================================================
# Rollout loader
# =====================================================================


def _add_dataset_collector_scripts_to_path():

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
            "First rollout step does not contain obs."
        )

    return observation


def _first_existing(
    mapping: dict[str, Any],
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

        bgr = cv2.imdecode(
            image.astype(
                np.uint8,
                copy=False,
            ),
            cv2.IMREAD_COLOR,
        )

        if bgr is None:
            raise RuntimeError(
                "Could not decode front RGB."
            )

        image = cv2.cvtColor(
            bgr,
            cv2.COLOR_BGR2RGB,
        )

    elif (
        image.ndim == 3
        and image.shape[2] == 3
    ):

        # Rollout dataset frames are stored in BGR/OpenCV format.
        image = cv2.cvtColor(
            image.astype(
                np.uint8,
                copy=False,
            ),
            cv2.COLOR_BGR2RGB,
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
        dtype=np.float64,
    )

    if depth.ndim != 2:
        raise RuntimeError(
            "Front depth must be 2D."
        )

    return (
        str(key),
        depth,
    )


def _artifact_robot_state(
    observation: dict[str, Any],
) -> dict[str, np.ndarray]:

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

    orientation /= np.linalg.norm(
        orientation
    )

    return {
        "eef_pos": position,
        "eef_quat": orientation,
    }


# =====================================================================
# Front-camera geometry
# =====================================================================


def _load_front_camera_matrix(
    path: Path,
) -> np.ndarray:

    payload = _load_json(
        path
    )

    K = np.asarray(
        payload[
            "matrix"
        ],
        dtype=np.float64,
    )

    if K.shape != (
        3,
        3,
    ):
        raise RuntimeError(
            "Derived front camera matrix has invalid shape."
        )

    return K


def _load_front_camera_calibration(
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
            "Invalid front-camera translation."
        )

    if orientation.shape != (
        3,
        3,
    ):
        raise RuntimeError(
            "Invalid front-camera orientation."
        )

    return {
        "position": position,
        "orientation_matrix": orientation,
    }


# =====================================================================
# Saved semantic scene
# =====================================================================


def _scene_object_from_json(
    payload: dict[str, Any],
) -> SceneObject:

    label = str(
        payload[
            "label"
        ]
    )

    attributes = payload.get(
        "attributes",
        {},
    )

    if not isinstance(
        attributes,
        dict,
    ):
        attributes = {}

    return SceneObject(
        object_id=str(
            payload[
                "object_id"
            ]
        ),

        label=label,

        pixel_coordinates=tuple(
            payload[
                "pixel_coordinates"
            ]
        ),

        position_camera=tuple(
            payload[
                "position_camera"
            ]
        ),

        position_base=tuple(
            payload[
                "position_base"
            ]
        ),

        category=str(
            payload.get(
                "category",
                label,
            )
        ),

        attributes=attributes,
    )


def _load_scene_state(
    raw_scene_payload: dict[str, Any],
) -> SceneState:

    objects = raw_scene_payload.get(
        "objects"
    )

    if not isinstance(
        objects,
        list,
    ):
        raise RuntimeError(
            "raw_scene_state.json has no objects."
        )

    return SceneState(
        objects=tuple(
            _scene_object_from_json(
                obj
            )
            for obj in objects
        )
    )


def _saved_scene_object(
    raw_scene_payload: dict[str, Any],
    object_id: str,
) -> dict[str, Any]:

    normalized = (
        object_id
        .strip()
        .casefold()
    )

    matches = [
        obj
        for obj
        in raw_scene_payload[
            "objects"
        ]
        if (
            str(
                obj[
                    "object_id"
                ]
            )
            .strip()
            .casefold()
            == normalized
        )
    ]

    if len(matches) != 1:
        raise RuntimeError(
            f"Could not uniquely resolve "
            f"{object_id!r} in raw scene."
        )

    return matches[
        0
    ]


# =====================================================================
# Primitive plan
# =====================================================================


def _extract_steps(
    payload: dict[str, Any],
) -> list[dict[str, Any]]:

    steps = payload.get(
        "steps"
    )

    if isinstance(
        steps,
        list,
    ):
        return steps

    nested = payload.get(
        "primitive_plan"
    )

    if isinstance(
        nested,
        dict,
    ):

        steps = nested.get(
            "steps"
        )

        if isinstance(
            steps,
            list,
        ):
            return steps

    raise RuntimeError(
        "Could not find primitive plan steps."
    )


def _load_primitive_plan(
    payload: dict[str, Any],
) -> PrimitivePlan:

    raw_steps = _extract_steps(
        payload
    )

    names = tuple(
        str(
            step[
                "name"
            ]
        )
        .strip()
        .lower()
        for step in raw_steps
    )

    if names != EXPECTED_PRIMITIVES:
        raise RuntimeError(
            "Unexpected Nut-Assembly primitive sequence.\n"
            f"Expected: {EXPECTED_PRIMITIVES}\n"
            f"Actual:   {names}"
        )

    return PrimitivePlan(
        steps=tuple(
            PrimitiveStep(
                name=str(
                    step[
                        "name"
                    ]
                ),

                arguments=dict(
                    step.get(
                        "arguments",
                        {},
                    )
                ),
            )
            for step in raw_steps
        ),

        source_code=str(
            payload.get(
                "source_code",
                "",
            )
        ),
    )


def _resolve_targets(
    primitive_plan: PrimitivePlan,
) -> tuple[
    str,
    str,
]:

    pick_targets = []
    place_targets = []

    for step in primitive_plan.steps:

        target = str(
            step.arguments.get(
                "target",
                "",
            )
        ).strip()

        if not target:
            continue

        name = str(
            step.name
        ).strip().lower()

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

    pick = {
        target.casefold(): target
        for target in pick_targets
    }

    place = {
        target.casefold(): target
        for target in place_targets
    }

    if len(pick) != 1:
        raise RuntimeError(
            f"Invalid pick targets: {pick_targets}"
        )

    if len(place) != 1:
        raise RuntimeError(
            f"Invalid place targets: {place_targets}"
        )

    return (
        next(
            iter(
                pick.values()
            )
        ),
        next(
            iter(
                place.values()
            )
        ),
    )


# =====================================================================
# Minimal action plan
# =====================================================================


def _make_action_plan(
    *,
    pick_target: str,
    place_target: str,
    pick_object: dict[str, Any],
    place_object: dict[str, Any],
    grasp_instruction: str,
) -> ActionPlanningResult:

    label = str(
        pick_object[
            "label"
        ]
    )

    color = (
        label.split()[0]
        if " " in label
        else label
    )

    step = ActionStep(
        pick_keyframe=0,
        place_keyframe=1,

        picked_track_id=0,
        picked_category=label,
        picked_color=color,

        destination_track_id=1,

        destination_category=str(
            place_object[
                "label"
            ]
        ),

        destination_ordinal_from_left=0,

        relation="on",

        action=(
            f"Pick {pick_target} and insert it "
            f"onto {place_target}."
        ),

        picked_detector_label=label,

        grasp_instruction=(
            grasp_instruction
        ),
    )

    return ActionPlanningResult(
        steps=(
            step,
        ),

        status="completed",

        ambiguities=(),

        natural_language_plan=(
            step.action
        ),

        task_type="nut_assembly",
    )


# =====================================================================
# Derived refinements
# =====================================================================


def _load_refinement_map(
    path: Path,
) -> dict[str, dict[str, Any]]:

    payload = _load_json(
        path
    )

    output = {}

    for object_id, refinement in payload.items():

        if not isinstance(
            refinement,
            dict,
        ):
            raise RuntimeError(
                f"Invalid refinement for {object_id!r}."
            )

        if not bool(
            refinement.get(
                "accepted",
                False,
            )
        ):
            raise RuntimeError(
                f"Refinement {object_id!r} "
                "is not accepted."
            )

        output[
            str(
                object_id
            )
        ] = refinement

    return output


def _resolve_refinement(
    refinements: dict[str, dict[str, Any]],
    target_id: str,
) -> dict[str, Any]:

    normalized = (
        target_id
        .strip()
        .casefold()
    )

    matches = [
        refinement
        for object_id, refinement
        in refinements.items()
        if (
            object_id
            .strip()
            .casefold()
            == normalized
        )
    ]

    if len(matches) != 1:
        raise RuntimeError(
            f"Could not resolve refinement for "
            f"{target_id!r}."
        )

    return matches[
        0
    ]


# =====================================================================
# GraspGen-X artifact loading
# =====================================================================


def _load_grasp_rgb(
    path: Path,
) -> np.ndarray:

    bgr = cv2.imread(
        str(
            path
        ),
        cv2.IMREAD_COLOR,
    )

    if bgr is None:
        raise RuntimeError(
            f"Could not read {path}"
        )

    return cv2.cvtColor(
        bgr,
        cv2.COLOR_BGR2RGB,
    )


def _recover_grasp_camera_matrix(
    *,
    diagnostics,
    depth: np.ndarray,
) -> np.ndarray:

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

    if len(u) != len(cloud):
        raise RuntimeError(
            "Cannot reconstruct GraspGen-X camera matrix: "
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

    rmse = float(
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

    if rmse > 1e-3:
        raise RuntimeError(
            "Recovered eye-in-hand K is inconsistent "
            f"with saved artifacts: RMSE={rmse:.6f}px."
        )

    return K


class ArtifactGraspPlanner:
    """
    No inference.

    Returns the GraspGen-X result already saved by
    the real robot test.
    """

    backend_name = "graspgenx"

    def __init__(
        self,
        result: GraspGenXPlanResult,
    ):

        self.result = result
        self.calls = []

    def plan(
        self,
        **kwargs,
    ) -> GraspGenXPlanResult:

        self.calls.append(
            kwargs
        )

        return self.result


def _load_grasp_artifacts(
    paths: dict[str, Path],
):

    diagnostics = np.load(
        paths[
            "grasp_diagnostics"
        ]
    )

    grasp_plan = _load_json(
        paths[
            "grasp_plan"
        ]
    )

    rgb = _load_grasp_rgb(
        paths[
            "grasp_rgb"
        ]
    )

    depth = np.asarray(
        np.load(
            paths[
                "grasp_depth"
            ]
        ),
        dtype=np.float32,
    )

    if (
        rgb.shape[
            :2
        ]
        != depth.shape
    ):
        raise RuntimeError(
            "Saved GraspGen-X RGB/depth dimensions differ."
        )

    K = _recover_grasp_camera_matrix(
        diagnostics=diagnostics,
        depth=depth,
    )

    tcp_position = np.asarray(
        diagnostics[
            "tcp_position_base"
        ],
        dtype=np.float64,
    )

    tcp_orientation = np.asarray(
        diagnostics[
            "tcp_orientation_base"
        ],
        dtype=np.float64,
    )

    tcp_orientation /= np.linalg.norm(
        tcp_orientation
    )

    approach = np.asarray(
        diagnostics[
            "approach_direction_base"
        ],
        dtype=np.float64,
    )

    semantic_point = np.asarray(
        diagnostics[
            "semantic_point_px"
        ],
        dtype=np.float32,
    )

    selected_index = int(
        diagnostics[
            "selected_index"
        ]
    )

    confidence = float(
        np.asarray(
            diagnostics[
                "confidences"
            ]
        )[
            selected_index
        ]
    )

    np.testing.assert_allclose(
        np.asarray(
            grasp_plan[
                "tcp_position_base"
            ],
            dtype=np.float64,
        ),
        tcp_position,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        np.asarray(
            grasp_plan[
                "tcp_orientation_base"
            ],
            dtype=np.float64,
        ),
        tcp_orientation,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        np.asarray(
            grasp_plan[
                "approach_direction_base"
            ],
            dtype=np.float64,
        ),
        approach,
        atol=1e-9,
    )

    assert float(
        grasp_plan[
            "confidence"
        ]
    ) == pytest.approx(
        confidence,
        abs=1e-9,
    )

    result = GraspGenXPlanResult(
        grasp_instruction=str(
            grasp_plan[
                "grasp_instruction"
            ]
        ),

        semantic_point_px=(
            semantic_point
        ),

        tcp_position_base=(
            tcp_position
        ),

        tcp_orientation_base=(
            tcp_orientation
        ),

        approach_direction_base=(
            approach
        ),

        confidence=(
            confidence
        ),
    )

    grasp_input = {
        "rgb_image": (
            rgb
        ),

        "depth_image": (
            depth
        ),

        "camera_matrix": (
            K
        ),

        "T_base_camera": np.asarray(
            diagnostics[
                "T_base_camera"
            ],
            dtype=np.float64,
        ),

        "T_base_table": np.asarray(
            diagnostics[
                "T_base_table"
            ],
            dtype=np.float64,
        ),
    }

    return (
        diagnostics,
        grasp_plan,
        result,
        grasp_input,
    )


# =====================================================================
# Motion layer
# =====================================================================


def _make_motion_layer() -> SeeDoMotionLayer:

    return SeeDoMotionLayer(
        grasp_orientation=np.array(
            [
                0.9994452044624775,
                0.03161651380119412,
                0.0021438049655468088,
                0.010251021036213035,
            ],
            dtype=np.float64,
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
# Geometry helpers
# =====================================================================


def _normalized(
    vector: np.ndarray,
) -> np.ndarray:

    vector = np.asarray(
        vector,
        dtype=np.float64,
    )

    norm = float(
        np.linalg.norm(
            vector
        )
    )

    if norm <= 1e-12:
        raise RuntimeError(
            "Cannot normalize zero vector."
        )

    return (
        vector
        / norm
    )


def _hole_from_pose(
    *,
    position: np.ndarray,
    orientation: np.ndarray,
    hole_top_local: np.ndarray,
    hole_bottom_local: np.ndarray,
    hole_axis_local: np.ndarray,
):

    rotation = Rotation.from_quat(
        orientation
    )

    top = (
        position
        + rotation.apply(
            hole_top_local
        )
    )

    bottom = (
        position
        + rotation.apply(
            hole_bottom_local
        )
    )

    axis = _normalized(
        rotation.apply(
            hole_axis_local
        )
    )

    return (
        top,
        bottom,
        axis,
    )


def _hole_from_action(
    action: np.ndarray,
    *,
    hole_top_local: np.ndarray,
    hole_bottom_local: np.ndarray,
    hole_axis_local: np.ndarray,
):

    action = np.asarray(
        action,
        dtype=np.float64,
    )

    return _hole_from_pose(
        position=(
            action[
                :3
            ]
        ),

        orientation=(
            action[
                3:7
            ]
        ),

        hole_top_local=(
            hole_top_local
        ),

        hole_bottom_local=(
            hole_bottom_local
        ),

        hole_axis_local=(
            hole_axis_local
        ),
    )


def _lateral_distance_to_axis(
    point: np.ndarray,
    axis_point: np.ndarray,
    axis_direction: np.ndarray,
) -> float:

    delta = (
        np.asarray(
            point,
            dtype=np.float64,
        )
        - np.asarray(
            axis_point,
            dtype=np.float64,
        )
    )

    axis = _normalized(
        axis_direction
    )

    lateral = (
        delta
        - np.dot(
            delta,
            axis,
        )
        * axis
    )

    return float(
        np.linalg.norm(
            lateral
        )
    )


def _validate_action_vectors(
    groups: dict[str, list],
) -> None:

    for primitive, actions in groups.items():

        for index, action in enumerate(
            actions
        ):

            action = np.asarray(
                action,
                dtype=np.float64,
            )

            if not np.isfinite(
                action
            ).all():
                raise AssertionError(
                    f"{primitive}[{index}] contains "
                    "non-finite values."
                )

            if action.size >= 7:

                quat_norm = float(
                    np.linalg.norm(
                        action[
                            3:7
                        ]
                    )
                )

                assert quat_norm == pytest.approx(
                    1.0,
                    abs=1e-6,
                )


# =====================================================================
# Visual-debug geometry
# =====================================================================


def _load_binary_mask(
    path: Path,
) -> np.ndarray:

    image = cv2.imread(
        str(
            path
        ),
        cv2.IMREAD_GRAYSCALE,
    )

    if image is None:
        raise RuntimeError(
            f"Could not load mask: {path}"
        )

    return (
        image
        > 0
    )


def _camera_points_to_base(
    points_camera: np.ndarray,
    *,
    camera_calibration: dict[str, np.ndarray],
    T_base_table: np.ndarray,
) -> np.ndarray:

    points_camera = np.asarray(
        points_camera,
        dtype=np.float64,
    )

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

    points_aruco = (
        points_camera
        @ R_camera_aruco.T
        + t_camera_aruco
    )

    points_table = (
        points_aruco
        @ ARUCO_TO_TABLE0_ROTATION.T
        + ARUCO_TO_TABLE0_TRANSLATION
    )

    points_base = (
        points_table
        @ T_base_table[
            :3,
            :3,
        ].T
        + T_base_table[
            :3,
            3,
        ]
    )

    return points_base


def _base_points_to_camera(
    points_base: np.ndarray,
    *,
    camera_calibration: dict[str, np.ndarray],
    T_base_table: np.ndarray,
) -> np.ndarray:

    points_base = np.asarray(
        points_base,
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

    points_table = (
        points_base
        - t_table_base
    ) @ R_table_base

    points_aruco = (
        points_table
        - ARUCO_TO_TABLE0_TRANSLATION
    ) @ ARUCO_TO_TABLE0_ROTATION

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

    points_camera = (
        points_aruco
        - t_camera_aruco
    ) @ R_camera_aruco

    return points_camera


def _project_base_points(
    points_base: np.ndarray,
    *,
    K: np.ndarray,
    camera_calibration: dict[str, np.ndarray],
    T_base_table: np.ndarray,
) -> tuple[
    np.ndarray,
    np.ndarray,
]:

    points_camera = _base_points_to_camera(
        points_base,

        camera_calibration=(
            camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

    z = points_camera[
        :,
        2
    ]

    valid = (
        np.isfinite(
            points_camera
        ).all(
            axis=1
        )
        & (
            z
            > 1e-6
        )
    )

    pixels = np.full(
        (
            len(
                points_camera
            ),
            2,
        ),
        np.nan,
        dtype=np.float64,
    )

    valid_points = points_camera[
        valid
    ]

    pixels[
        valid,
        0,
    ] = (
        K[
            0,
            0
        ]
        * valid_points[
            :,
            0
        ]
        / valid_points[
            :,
            2
        ]
        + K[
            0,
            2
        ]
    )

    pixels[
        valid,
        1,
    ] = (
        K[
            1,
            1
        ]
        * valid_points[
            :,
            1
        ]
        / valid_points[
            :,
            2
        ]
        + K[
            1,
            2
        ]
    )

    return (
        pixels,
        valid,
    )


def _mask_depth_to_base_cloud(
    *,
    mask: np.ndarray,
    depth: np.ndarray,
    K: np.ndarray,
    camera_calibration: dict[str, np.ndarray],
    T_base_table: np.ndarray,
    max_points: int = 6000,
) -> np.ndarray:

    mask = np.asarray(
        mask,
        dtype=bool,
    )

    depth = np.asarray(
        depth,
        dtype=np.float64,
    )

    if mask.shape != depth.shape:
        raise RuntimeError(
            "Front mask and depth dimensions differ."
        )

    valid = (
        mask
        & np.isfinite(
            depth
        )
        & (
            depth > 0.05
        )
        & (
            depth < 2.0
        )
    )

    v, u = np.where(
        valid
    )

    if len(
        u
    ) == 0:
        raise RuntimeError(
            "Ring mask contains no valid depth pixels."
        )

    if len(
        u
    ) > max_points:

        indices = np.linspace(
            0,
            len(
                u
            )
            - 1,
            max_points,
            dtype=int,
        )

        u = u[
            indices
        ]

        v = v[
            indices
        ]

    z = depth[
        v,
        u
    ]

    x = (
        (
            u
            - K[
                0,
                2
            ]
        )
        * z
        / K[
            0,
            0
        ]
    )

    y = (
        (
            v
            - K[
                1,
                2
            ]
        )
        * z
        / K[
            1,
            1
        ]
    )

    points_camera = np.column_stack(
        (
            x,
            y,
            z,
        )
    )

    return _camera_points_to_base(
        points_camera,

        camera_calibration=(
            camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )


def _transform_object_via_tcp(
    points_base: np.ndarray,
    *,
    source_tcp_position: np.ndarray,
    source_tcp_orientation: np.ndarray,
    destination_tcp_position: np.ndarray,
    destination_tcp_orientation: np.ndarray,
) -> np.ndarray:

    points_base = np.asarray(
        points_base,
        dtype=np.float64,
    )

    source_rotation = Rotation.from_quat(
        source_tcp_orientation
    )

    destination_rotation = Rotation.from_quat(
        destination_tcp_orientation
    )

    points_tcp_local = (
        source_rotation
        .inv()
        .apply(
            points_base
            - source_tcp_position
        )
    )

    predicted_points_base = (
        destination_tcp_position
        + destination_rotation.apply(
            points_tcp_local
        )
    )

    return predicted_points_base


def _draw_projected_point(
    image_bgr: np.ndarray,
    *,
    point_base: np.ndarray,
    label: str,
    color: tuple[int, int, int],
    K: np.ndarray,
    camera_calibration: dict[str, np.ndarray],
    T_base_table: np.ndarray,
    radius: int = 7,
) -> tuple[int, int] | None:

    pixels, valid = _project_base_points(
        np.asarray(
            point_base,
            dtype=np.float64,
        ).reshape(
            1,
            3,
        ),

        K=K,

        camera_calibration=(
            camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

    if not bool(
        valid[
            0
        ]
    ):
        return None

    u = int(
        round(
            float(
                pixels[
                    0,
                    0
                ]
            )
        )
    )

    v = int(
        round(
            float(
                pixels[
                    0,
                    1
                ]
            )
        )
    )

    height, width = (
        image_bgr.shape[
            :2
        ]
    )

    if (
        u < 0
        or u >= width
        or v < 0
        or v >= height
    ):
        return None

    cv2.circle(
        image_bgr,
        (
            u,
            v,
        ),
        radius,
        color,
        -1,
    )

    cv2.putText(
        image_bgr,
        label,
        (
            u + 8,
            v - 8,
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        color,
        1,
        cv2.LINE_AA,
    )

    return (
        u,
        v,
    )


def _draw_projected_segment(
    image_bgr: np.ndarray,
    *,
    point_a: np.ndarray,
    point_b: np.ndarray,
    color: tuple[int, int, int],
    K: np.ndarray,
    camera_calibration: dict[str, np.ndarray],
    T_base_table: np.ndarray,
    thickness: int = 2,
) -> None:

    points = np.asarray(
        [
            point_a,
            point_b,
        ],
        dtype=np.float64,
    )

    pixels, valid = _project_base_points(
        points,

        K=K,

        camera_calibration=(
            camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

    if not bool(
        np.all(
            valid
        )
    ):
        return

    p1 = tuple(
        np.round(
            pixels[
                0
            ]
        )
        .astype(int)
        .tolist()
    )

    p2 = tuple(
        np.round(
            pixels[
                1
            ]
        )
        .astype(int)
        .tolist()
    )

    cv2.line(
        image_bgr,
        p1,
        p2,
        color,
        thickness,
        cv2.LINE_AA,
    )


def _collect_hole_bottom_trajectory(
    action_groups: dict[str, list],
    *,
    hole_top_local: np.ndarray,
    hole_bottom_local: np.ndarray,
    hole_axis_local: np.ndarray,
) -> dict[str, np.ndarray]:

    output = {}

    for primitive, actions in action_groups.items():

        bottoms = []

        for action in actions:

            action = np.asarray(
                action,
                dtype=np.float64,
            )

            if action.size < 7:
                continue

            (
                _,
                bottom,
                _,
            ) = _hole_from_action(
                action,

                hole_top_local=(
                    hole_top_local
                ),

                hole_bottom_local=(
                    hole_bottom_local
                ),

                hole_axis_local=(
                    hole_axis_local
                ),
            )

            bottoms.append(
                bottom
            )

        output[
            primitive
        ] = np.asarray(
            bottoms,
            dtype=np.float64,
        )

    return output


# =====================================================================
# Visual artifact exporter
# =====================================================================


def _export_nut_visual_debug_artifacts(
    *,
    output_dir: Path,
    front_rgb: np.ndarray,
    front_depth: np.ndarray,
    front_K: np.ndarray,
    front_camera_calibration: dict[str, np.ndarray],
    T_base_table: np.ndarray,
    ring_mask: np.ndarray,
    pick_target: str,
    place_target: str,
    real_grasp_tcp: np.ndarray,
    real_grasp_quaternion: np.ndarray,
    final_tcp_position: np.ndarray,
    final_tcp_orientation: np.ndarray,
    hole_top_initial: np.ndarray,
    hole_bottom_initial: np.ndarray,
    hole_axis_initial: np.ndarray,
    peg_support: np.ndarray,
    peg_tip: np.ndarray,
    peg_axis: np.ndarray,
    align_target_hole_bottom: np.ndarray,
    initial_entry_hole_bottom: np.ndarray,
    final_release_hole_bottom: np.ndarray,
    final_hole_top: np.ndarray,
    final_hole_axis: np.ndarray,
    lift_tcp: np.ndarray,
    object_sweep_radius_m: float,
    ring_thickness_m: float,
    peg_height_m: float,
    action_groups: dict[str, list],
    hole_top_local: np.ndarray,
    hole_bottom_local: np.ndarray,
    hole_axis_local: np.ndarray,
    auto_release: bool,
) -> dict[str, Path]:

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    # -------------------------------------------------------------
    # Reconstruct the actual ring point cloud from the saved t=0
    # front RGB-D + derived front mask.
    # -------------------------------------------------------------

    ring_points_initial_base = (
        _mask_depth_to_base_cloud(
            mask=ring_mask,
            depth=front_depth,
            K=front_K,
            camera_calibration=(
                front_camera_calibration
            ),
            T_base_table=(
                T_base_table
            ),
        )
    )

    # -------------------------------------------------------------
    # Predict where the entire observed ring geometry will end up
    # after applying the final TCP pose.
    #
    # This uses the rigid transform actually established by the
    # real GraspGen-X grasp.
    # -------------------------------------------------------------

    ring_points_final_base = (
        _transform_object_via_tcp(
            ring_points_initial_base,

            source_tcp_position=(
                real_grasp_tcp
            ),

            source_tcp_orientation=(
                real_grasp_quaternion
            ),

            destination_tcp_position=(
                final_tcp_position
            ),

            destination_tcp_orientation=(
                final_tcp_orientation
            ),
        )
    )

    predicted_ring_cloud_path = (
        output_dir
        / "predicted_final_ring_points_base.npy"
    )

    np.save(
        predicted_ring_cloud_path,
        ring_points_final_base,
    )

    # -------------------------------------------------------------
    # Hole-bottom trajectory for every primitive
    # -------------------------------------------------------------

    hole_trajectories = (
        _collect_hole_bottom_trajectory(
            action_groups,

            hole_top_local=(
                hole_top_local
            ),

            hole_bottom_local=(
                hole_bottom_local
            ),

            hole_axis_local=(
                hole_axis_local
            ),
        )
    )

    # -------------------------------------------------------------
    # JSON artifact
    # -------------------------------------------------------------

    points_json_path = (
        output_dir
        / "nut_visual_debug_points.json"
    )

    predicted_bbox_min = np.min(
        ring_points_final_base,
        axis=0,
    )

    predicted_bbox_max = np.max(
        ring_points_final_base,
        axis=0,
    )

    points_payload = {
        "frame": (
            "base_link"
        ),

        "pick_target": (
            pick_target
        ),

        "place_target": (
            place_target
        ),

        "important_note": (
            "final_release_hole_bottom_base is the final "
            "placement/release geometry target. "
            "auto_release is false in this test, therefore "
            "the gripper is not automatically opened."
        ),

        "auto_release": (
            auto_release
        ),

        "initial_ring_geometry": {
            "hole_top_base": (
                hole_top_initial
            ),

            "hole_bottom_base": (
                hole_bottom_initial
            ),

            "hole_axis_base": (
                hole_axis_initial
            ),

            "ring_thickness_m": (
                ring_thickness_m
            ),
        },

        "peg_geometry": {
            "support_base": (
                peg_support
            ),

            "tip_base": (
                peg_tip
            ),

            "axis_base": (
                peg_axis
            ),

            "height_m": (
                peg_height_m
            ),
        },

        "planned_targets": {
            "align_hole_bottom_base": (
                align_target_hole_bottom
            ),

            "initial_entry_hole_bottom_base": (
                initial_entry_hole_bottom
            ),

            "final_release_hole_bottom_base": (
                final_release_hole_bottom
            ),
        },

        "predicted_final_ring_pose": {
            "hole_top_base": (
                final_hole_top
            ),

            "hole_bottom_base": (
                final_release_hole_bottom
            ),

            "hole_axis_base": (
                final_hole_axis
            ),

            "tcp_position_base": (
                final_tcp_position
            ),

            "tcp_orientation_base": (
                final_tcp_orientation
            ),

            "point_cloud_bbox_min_base": (
                predicted_bbox_min
            ),

            "point_cloud_bbox_max_base": (
                predicted_bbox_max
            ),

            "point_count": int(
                len(
                    ring_points_final_base
                )
            ),
        },

        "real_grasp": {
            "tcp_position_base": (
                real_grasp_tcp
            ),

            "tcp_orientation_base": (
                real_grasp_quaternion
            ),
        },

        "lift_tcp_base": (
            lift_tcp
        ),

        "object_sweep_radius_m": (
            object_sweep_radius_m
        ),

        "hole_bottom_trajectories": {
            primitive: points
            for primitive, points
            in hole_trajectories.items()
        },
    }

    _write_json(
        points_json_path,
        points_payload,
    )

    # -------------------------------------------------------------
    # FRONT CAMERA OVERLAY
    # -------------------------------------------------------------

    overlay = cv2.cvtColor(
        front_rgb,
        cv2.COLOR_RGB2BGR,
    ).copy()

    # Initial observed ring cloud.
    initial_pixels, initial_valid = (
        _project_base_points(
            ring_points_initial_base,

            K=front_K,

            camera_calibration=(
                front_camera_calibration
            ),

            T_base_table=(
                T_base_table
            ),
        )
    )

    for pixel, valid in zip(
        initial_pixels,
        initial_valid,
    ):

        if not valid:
            continue

        u, v = (
            np.round(
                pixel
            )
            .astype(int)
        )

        if (
            0 <= u < overlay.shape[1]
            and 0 <= v < overlay.shape[0]
        ):
            cv2.circle(
                overlay,
                (
                    int(u),
                    int(v),
                ),
                1,
                (
                    255,
                    160,
                    0,
                ),
                -1,
            )

    # Predicted final ring cloud.
    final_pixels, final_valid = (
        _project_base_points(
            ring_points_final_base,

            K=front_K,

            camera_calibration=(
                front_camera_calibration
            ),

            T_base_table=(
                T_base_table
            ),
        )
    )

    for pixel, valid in zip(
        final_pixels,
        final_valid,
    ):

        if not valid:
            continue

        u, v = (
            np.round(
                pixel
            )
            .astype(int)
        )

        if (
            0 <= u < overlay.shape[1]
            and 0 <= v < overlay.shape[0]
        ):
            cv2.circle(
                overlay,
                (
                    int(u),
                    int(v),
                ),
                1,
                (
                    0,
                    255,
                    255,
                ),
                -1,
            )

    # Peg axis.
    _draw_projected_segment(
        overlay,

        point_a=(
            peg_support
        ),

        point_b=(
            peg_tip
        ),

        color=(
            0,
            255,
            0,
        ),

        K=front_K,

        camera_calibration=(
            front_camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),

        thickness=3,
    )

    # Planned insertion path.
    _draw_projected_segment(
        overlay,

        point_a=(
            align_target_hole_bottom
        ),

        point_b=(
            initial_entry_hole_bottom
        ),

        color=(
            255,
            255,
            0,
        ),

        K=front_K,

        camera_calibration=(
            front_camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

    _draw_projected_segment(
        overlay,

        point_a=(
            initial_entry_hole_bottom
        ),

        point_b=(
            final_release_hole_bottom
        ),

        color=(
            255,
            0,
            255,
        ),

        K=front_K,

        camera_calibration=(
            front_camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

    _draw_projected_point(
        overlay,

        point_base=(
            hole_bottom_initial
        ),

        label="HOLE initial",

        color=(
            255,
            0,
            0,
        ),

        K=front_K,

        camera_calibration=(
            front_camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

    _draw_projected_point(
        overlay,

        point_base=(
            peg_support
        ),

        label="PEG support",

        color=(
            0,
            180,
            0,
        ),

        K=front_K,

        camera_calibration=(
            front_camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

    _draw_projected_point(
        overlay,

        point_base=(
            peg_tip
        ),

        label="PEG tip",

        color=(
            0,
            0,
            255,
        ),

        K=front_K,

        camera_calibration=(
            front_camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

    _draw_projected_point(
        overlay,

        point_base=(
            align_target_hole_bottom
        ),

        label="ALIGN",

        color=(
            255,
            255,
            0,
        ),

        K=front_K,

        camera_calibration=(
            front_camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

    _draw_projected_point(
        overlay,

        point_base=(
            initial_entry_hole_bottom
        ),

        label="ENTRY -5mm",

        color=(
            0,
            165,
            255,
        ),

        K=front_K,

        camera_calibration=(
            front_camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

    _draw_projected_point(
        overlay,

        point_base=(
            final_release_hole_bottom
        ),

        label="FINAL +10mm",

        color=(
            255,
            0,
            255,
        ),

        K=front_K,

        camera_calibration=(
            front_camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

    cv2.putText(
        overlay,
        "BLUE CLOUD: initial ring",
        (
            15,
            22,
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (
            255,
            160,
            0,
        ),
        1,
        cv2.LINE_AA,
    )

    cv2.putText(
        overlay,
        "YELLOW CLOUD: predicted final ring",
        (
            15,
            42,
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (
            0,
            255,
            255,
        ),
        1,
        cv2.LINE_AA,
    )

    front_overlay_path = (
        output_dir
        / "nut_place_targets_front_overlay.png"
    )

    if not cv2.imwrite(
        str(
            front_overlay_path
        ),
        overlay,
    ):
        raise RuntimeError(
            "Could not save front visual-debug overlay."
        )

    # -------------------------------------------------------------
    # TOP-DOWN X-Y PLOT
    # -------------------------------------------------------------

    topdown_path = (
        output_dir
        / "nut_place_targets_topdown.png"
    )

    fig = plt.figure(
        figsize=(
            10,
            8,
        )
    )

    ax = fig.add_subplot(
        111
    )

    initial_plot_points = (
        ring_points_initial_base[
            ::max(
                1,
                len(
                    ring_points_initial_base
                )
                // 1200,
            )
        ]
    )

    final_plot_points = (
        ring_points_final_base[
            ::max(
                1,
                len(
                    ring_points_final_base
                )
                // 1200,
            )
        ]
    )

    ax.scatter(
        initial_plot_points[
            :,
            0
        ],
        initial_plot_points[
            :,
            1
        ],
        s=3,
        alpha=0.25,
        label="Initial observed ring",
    )

    ax.scatter(
        final_plot_points[
            :,
            0
        ],
        final_plot_points[
            :,
            1
        ],
        s=3,
        alpha=0.35,
        label="Predicted final ring",
    )

    ax.scatter(
        hole_bottom_initial[
            0
        ],
        hole_bottom_initial[
            1
        ],
        s=90,
        marker="x",
        label="Initial hole bottom",
    )

    ax.scatter(
        peg_tip[
            0
        ],
        peg_tip[
            1
        ],
        s=100,
        marker="^",
        label="Peg axis XY",
    )

    ax.scatter(
        align_target_hole_bottom[
            0
        ],
        align_target_hole_bottom[
            1
        ],
        s=85,
        marker="o",
        label="Align target",
    )

    ax.scatter(
        final_release_hole_bottom[
            0
        ],
        final_release_hole_bottom[
            1
        ],
        s=100,
        marker="*",
        label="Final placement target",
    )

    path_xy = np.asarray(
        [
            hole_bottom_initial,
            align_target_hole_bottom,
            initial_entry_hole_bottom,
            final_release_hole_bottom,
        ],
        dtype=np.float64,
    )

    ax.plot(
        path_xy[
            :,
            0
        ],
        path_xy[
            :,
            1
        ],
        "--",
        linewidth=1.5,
        label="Hole target progression",
    )

    ax.set_xlabel(
        "base_link X [m]"
    )

    ax.set_ylabel(
        "base_link Y [m]"
    )

    ax.set_title(
        "Nut Assembly - Top Down (X-Y)"
    )

    ax.axis(
        "equal"
    )

    ax.grid(
        True,
        alpha=0.25,
    )

    ax.legend(
        loc="best"
    )

    fig.tight_layout()

    fig.savefig(
        topdown_path,
        dpi=180,
    )

    plt.close(
        fig
    )

    # -------------------------------------------------------------
    # SIDE Y-Z PLOT
    # -------------------------------------------------------------

    sideview_path = (
        output_dir
        / "nut_place_targets_sideview.png"
    )

    fig = plt.figure(
        figsize=(
            10,
            8,
        )
    )

    ax = fig.add_subplot(
        111
    )

    ax.scatter(
        initial_plot_points[
            :,
            1
        ],
        initial_plot_points[
            :,
            2
        ],
        s=3,
        alpha=0.25,
        label="Initial observed ring",
    )

    ax.scatter(
        final_plot_points[
            :,
            1
        ],
        final_plot_points[
            :,
            2
        ],
        s=3,
        alpha=0.35,
        label="Predicted final ring",
    )

    ax.plot(
        [
            peg_support[
                1
            ],
            peg_tip[
                1
            ],
        ],
        [
            peg_support[
                2
            ],
            peg_tip[
                2
            ],
        ],
        linewidth=4,
        label="Refined peg axis",
    )

    ax.scatter(
        peg_support[
            1
        ],
        peg_support[
            2
        ],
        s=100,
        marker="s",
        label="Peg support",
    )

    ax.scatter(
        peg_tip[
            1
        ],
        peg_tip[
            2
        ],
        s=100,
        marker="^",
        label="Peg tip",
    )

    ax.scatter(
        align_target_hole_bottom[
            1
        ],
        align_target_hole_bottom[
            2
        ],
        s=90,
        marker="o",
        label="Align: +20 mm",
    )

    ax.scatter(
        initial_entry_hole_bottom[
            1
        ],
        initial_entry_hole_bottom[
            2
        ],
        s=90,
        marker="v",
        label="Initial entry: -5 mm",
    )

    ax.scatter(
        final_release_hole_bottom[
            1
        ],
        final_release_hole_bottom[
            2
        ],
        s=120,
        marker="*",
        label="Final: support +10 mm",
    )

    insertion_path = np.asarray(
        [
            align_target_hole_bottom,
            initial_entry_hole_bottom,
            final_release_hole_bottom,
        ],
        dtype=np.float64,
    )

    ax.plot(
        insertion_path[
            :,
            1
        ],
        insertion_path[
            :,
            2
        ],
        "--",
        linewidth=2,
        label="Insertion target path",
    )

    ax.set_xlabel(
        "base_link Y [m]"
    )

    ax.set_ylabel(
        "base_link Z [m]"
    )

    ax.set_title(
        "Nut Assembly - Side View (Y-Z)"
    )

    ax.grid(
        True,
        alpha=0.25,
    )

    ax.legend(
        loc="best"
    )

    fig.tight_layout()

    fig.savefig(
        sideview_path,
        dpi=180,
    )

    plt.close(
        fig
    )

    return {
        "points_json": (
            points_json_path
        ),

        "front_overlay": (
            front_overlay_path
        ),

        "topdown": (
            topdown_path
        ),

        "sideview": (
            sideview_path
        ),

        "predicted_ring_cloud": (
            predicted_ring_cloud_path
        ),
    }


# =====================================================================
# Complete test
# =====================================================================


def test_real_nut_assembly_pipeline(
    case_dir: Path,
    tmp_path: Path,
):

    paths = _case_paths(
        case_dir
    )

    manifest = _validate_manifest(
        paths
    )

    # =============================================================
    # Load frozen artifacts
    # =============================================================

    raw_scene_payload = _load_json(
        paths[
            "raw_scene"
        ]
    )

    primitive_payload = _load_json(
        paths[
            "primitive_plan"
        ]
    )

    primitive_plan = _load_primitive_plan(
        primitive_payload
    )

    (
        pick_target,
        place_target,
    ) = _resolve_targets(
        primitive_plan
    )

    pick_object = _saved_scene_object(
        raw_scene_payload,
        pick_target,
    )

    place_object = _saved_scene_object(
        raw_scene_payload,
        place_target,
    )

    scene_state = _load_scene_state(
        raw_scene_payload
    )

    pick_refinements = _load_refinement_map(
        paths[
            "pick_refinements"
        ]
    )

    place_refinements = _load_refinement_map(
        paths[
            "place_refinements"
        ]
    )

    pick_geometry = _resolve_refinement(
        pick_refinements,
        pick_target,
    )

    place_geometry = _resolve_refinement(
        place_refinements,
        place_target,
    )

    (
        diagnostics,
        grasp_plan_payload,
        real_grasp_result,
        grasp_input,
    ) = _load_grasp_artifacts(
        paths
    )

    first_observation = (
        _load_first_rollout_observation(
            paths[
                "rollout"
            ]
        )
    )

    robot_state = (
        _artifact_robot_state(
            first_observation
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

    front_K = _load_front_camera_matrix(
        paths[
            "front_camera_info"
        ]
    )

    front_camera_calibration = (
        _load_front_camera_calibration(
            paths[
                "camera_noise"
            ]
        )
    )

    T_base_table = np.asarray(
        grasp_input[
            "T_base_table"
        ],
        dtype=np.float64,
    )

    # Pick-mask path generated by prepare_nut_real_case.py.
    pick_mask_path = (
        paths[
            "derived_dir"
        ]
        / "scene_masks"
        / f"{pick_target}.png"
    )

    pick_mask = _load_binary_mask(
        _require_file(
            pick_mask_path,
            "derived pick mask",
        )
    )

    # =============================================================
    # Geometry from frozen refinements
    # =============================================================

    peg_axis = _normalized(
        np.asarray(
            place_geometry[
                "axis_direction_base"
            ],
            dtype=np.float64,
        )
    )

    peg_tip = np.asarray(
        place_geometry[
            "peg_tip_point_base"
        ],
        dtype=np.float64,
    )

    peg_support = np.asarray(
        place_geometry[
            "peg_support_point_base"
        ],
        dtype=np.float64,
    )

    observed_peg_height = float(
        np.dot(
            peg_tip
            - peg_support,
            peg_axis,
        )
    )

    hole_top = np.asarray(
        pick_geometry[
            "hole_center_base"
        ],
        dtype=np.float64,
    )

    hole_bottom = np.asarray(
        pick_geometry[
            "hole_bottom_center_base"
        ],
        dtype=np.float64,
    )

    hole_axis = _normalized(
        np.asarray(
            pick_geometry[
                "hole_axis_base"
            ],
            dtype=np.float64,
        )
    )

    observed_ring_thickness = float(
        np.dot(
            hole_top
            - hole_bottom,
            hole_axis,
        )
    )

    initial_hole_peg_angle_deg = float(
        np.degrees(
            np.arccos(
                np.clip(
                    float(
                        np.dot(
                            hole_axis,
                            peg_axis,
                        )
                    ),
                    -1.0,
                    1.0,
                )
            )
        )
    )

    # =============================================================
    # Frozen real grasp
    # =============================================================

    planner = ArtifactGraspPlanner(
        real_grasp_result
    )

    # =============================================================
    # Runtime Controller without model inference
    # =============================================================

    controller = SeeDoController.__new__(
        SeeDoController
    )

    controller.action_plan = (
        _make_action_plan(
            pick_target=(
                pick_target
            ),

            place_target=(
                place_target
            ),

            pick_object=(
                pick_object
            ),

            place_object=(
                place_object
            ),

            grasp_instruction=str(
                grasp_plan_payload[
                    "grasp_instruction"
                ]
            ),
        )
    )

    controller.scene_state = (
        scene_state
    )

    controller.primitive_plan = (
        primitive_plan
    )

    controller.perception_mode = (
        "artifact_replay"
    )

    controller.grasp_planning_enabled = True
    controller.grasp_planner = planner

    controller.motion_layer = (
        _make_motion_layer()
    )

    controller.nut_pick_refinements = (
        pick_refinements
    )

    controller.nut_place_refinements = (
        place_refinements
    )

    controller.artifacts_dir = (
        tmp_path
        / "real_nut_pipeline"
    )

    controller.artifacts_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    controller.execution_status = (
        "plan_ready"
    )

    controller.execution_error = None

    controller.grasp_plan = None
    controller.grasp_plan_target = None

    # =============================================================
    # t=1 REACH
    # =============================================================

    reach_actions = (
        controller.inference(
            input_data={
                "robot_state": (
                    robot_state
                ),
            },
            t=1,
        )
    )

    assert reach_actions
    assert len(
        planner.calls
    ) == 0

    # =============================================================
    # t=2 APPROACHING
    # =============================================================

    approaching_actions = (
        controller.inference(
            input_data={
                "robot_state": (
                    robot_state
                ),

                "grasp_input": (
                    grasp_input
                ),
            },
            t=2,
        )
    )

    assert approaching_actions
    assert len(
        planner.calls
    ) == 1

    planner_call = (
        planner.calls[
            0
        ]
    )

    np.testing.assert_allclose(
        planner_call[
            "camera_matrix"
        ],
        grasp_input[
            "camera_matrix"
        ],
        atol=1e-9,
    )

    np.testing.assert_allclose(
        planner_call[
            "T_base_camera"
        ],
        grasp_input[
            "T_base_camera"
        ],
        atol=1e-9,
    )

    np.testing.assert_allclose(
        planner_call[
            "T_base_table"
        ],
        grasp_input[
            "T_base_table"
        ],
        atol=1e-9,
    )

    real_tcp = np.asarray(
        real_grasp_result
        .tcp_position_base,
        dtype=np.float64,
    )

    real_quaternion = _normalized(
        np.asarray(
            real_grasp_result
            .tcp_orientation_base,
            dtype=np.float64,
        )
    )

    np.testing.assert_allclose(
        approaching_actions[
            -1
        ][
            :3
        ],
        real_tcp,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        approaching_actions[
            -1
        ][
            3:7
        ],
        real_quaternion,
        atol=1e-8,
    )

    # =============================================================
    # Frozen grasp-relative ring geometry
    # =============================================================

    hole_top_local = np.asarray(
        controller
        .motion_layer
        ._nut_hole_top_local,
        dtype=np.float64,
    )

    hole_bottom_local = np.asarray(
        controller
        .motion_layer
        ._nut_hole_bottom_local,
        dtype=np.float64,
    )

    hole_axis_local = _normalized(
        np.asarray(
            controller
            .motion_layer
            ._nut_hole_axis_local,
            dtype=np.float64,
        )
    )

    (
        reconstructed_top,
        reconstructed_bottom,
        reconstructed_axis,
    ) = _hole_from_pose(
        position=(
            real_tcp
        ),

        orientation=(
            real_quaternion
        ),

        hole_top_local=(
            hole_top_local
        ),

        hole_bottom_local=(
            hole_bottom_local
        ),

        hole_axis_local=(
            hole_axis_local
        ),
    )

    handoff_top_error = float(
        np.linalg.norm(
            reconstructed_top
            - hole_top
        )
    )

    handoff_bottom_error = float(
        np.linalg.norm(
            reconstructed_bottom
            - hole_bottom
        )
    )

    handoff_axis_dot = float(
        np.dot(
            reconstructed_axis,
            hole_axis,
        )
    )

    assert handoff_top_error < 1e-8
    assert handoff_bottom_error < 1e-8
    assert handoff_axis_dot > 0.999999999

    # =============================================================
    # t=3 PICK
    # =============================================================

    pick_actions = (
        controller.inference(
            input_data={},
            t=3,
        )
    )

    assert len(
        pick_actions
    ) == 1

    np.testing.assert_allclose(
        pick_actions[
            0
        ][
            :3
        ],
        real_tcp,
        atol=1e-9,
    )

    # =============================================================
    # t=4 LIFT_UP
    # =============================================================

    lift_actions = (
        controller.inference(
            input_data={},
            t=4,
        )
    )

    assert lift_actions

    lift_tcp = (
        controller
        .motion_layer
        .current_pose
        .position
        .copy()
    )

    # =============================================================
    # t=5 MOVING
    # =============================================================

    moving_actions = (
        controller.inference(
            input_data={},
            t=5,
        )
    )

    assert moving_actions

    (
        moving_hole_top,
        moving_hole_bottom,
        moving_hole_axis,
    ) = (
        controller
        .motion_layer
        ._nut_current_hole_geometry()
    )

    moving_lateral_error = (
        _lateral_distance_to_axis(
            moving_hole_bottom,
            peg_tip,
            peg_axis,
        )
    )

    moving_axial_clearance = float(
        np.dot(
            moving_hole_bottom
            - peg_tip,
            peg_axis,
        )
    )

    moving_axis_dot = float(
        np.dot(
            moving_hole_axis,
            peg_axis,
        )
    )

    assert moving_lateral_error < 1e-6
    assert moving_axial_clearance > 0.020
    assert moving_axis_dot > 0.999

    # =============================================================
    # t=6 ALIGNING
    # =============================================================

    aligning_actions = (
        controller.inference(
            input_data={},
            t=6,
        )
    )

    assert aligning_actions

    (
        aligned_hole_top,
        aligned_hole_bottom,
        aligned_hole_axis,
    ) = (
        controller
        .motion_layer
        ._nut_current_hole_geometry()
    )

    expected_aligned_bottom = (
        peg_tip
        + 0.020
        * peg_axis
    )

    aligning_error = float(
        np.linalg.norm(
            aligned_hole_bottom
            - expected_aligned_bottom
        )
    )

    aligned_axis_dot = float(
        np.dot(
            aligned_hole_axis,
            peg_axis,
        )
    )

    assert aligning_error < 1e-6
    assert aligned_axis_dot > 0.999

    # =============================================================
    # t=7 INSERTING
    # =============================================================

    inserting_actions = (
        controller.inference(
            input_data={},
            t=7,
        )
    )

    assert inserting_actions

    (
        final_hole_top,
        final_hole_bottom,
        final_hole_axis,
    ) = (
        controller
        .motion_layer
        ._nut_current_hole_geometry()
    )

    expected_final_bottom = (
        peg_support
        + 0.010
        * peg_axis
    )

    expected_initial_entry = (
        peg_tip
        - 0.005
        * peg_axis
    )

    final_position_error = float(
        np.linalg.norm(
            final_hole_bottom
            - expected_final_bottom
        )
    )

    final_axis_dot = float(
        np.dot(
            final_hole_axis,
            peg_axis,
        )
    )

    final_support_height = float(
        np.dot(
            final_hole_bottom
            - peg_support,
            peg_axis,
        )
    )

    final_lateral_error = (
        _lateral_distance_to_axis(
            final_hole_bottom,
            peg_support,
            peg_axis,
        )
    )

    assert final_position_error < 1e-6
    assert final_lateral_error < 1e-6

    assert final_support_height == pytest.approx(
        0.010,
        abs=1e-6,
    )

    assert final_axis_dot > 0.999

    # =============================================================
    # Verify initial-entry waypoint
    # =============================================================

    insertion_axial_coordinates = []
    insertion_lateral_errors = []

    for action in inserting_actions:

        (
            _,
            action_hole_bottom,
            action_hole_axis,
        ) = _hole_from_action(
            action,

            hole_top_local=(
                hole_top_local
            ),

            hole_bottom_local=(
                hole_bottom_local
            ),

            hole_axis_local=(
                hole_axis_local
            ),
        )

        axial = float(
            np.dot(
                action_hole_bottom
                - peg_tip,
                peg_axis,
            )
        )

        lateral = (
            _lateral_distance_to_axis(
                action_hole_bottom,
                peg_tip,
                peg_axis,
            )
        )

        insertion_axial_coordinates.append(
            axial
        )

        insertion_lateral_errors.append(
            lateral
        )

        assert float(
            np.dot(
                action_hole_axis,
                peg_axis,
            )
        ) > 0.999

    insertion_axial_coordinates = np.asarray(
        insertion_axial_coordinates,
        dtype=np.float64,
    )

    insertion_lateral_errors = np.asarray(
        insertion_lateral_errors,
        dtype=np.float64,
    )

    initial_entry_error = float(
        np.min(
            np.abs(
                insertion_axial_coordinates
                - (
                    -0.005
                )
            )
        )
    )

    assert initial_entry_error < 1e-6

    assert float(
        np.max(
            insertion_lateral_errors
        )
    ) < 1e-6

    # =============================================================
    # Prove old centroid is not suitable
    # =============================================================

    saved_place_position = np.asarray(
        place_object[
            "position_base"
        ],
        dtype=np.float64,
    )

    scene_centroid_lateral_bias = (
        _lateral_distance_to_axis(
            saved_place_position,
            peg_tip,
            peg_axis,
        )
    )

    assert scene_centroid_lateral_bias > 0.001

    # =============================================================
    # Global action sanity
    # =============================================================

    action_groups = {
        "reach": (
            reach_actions
        ),

        "approaching": (
            approaching_actions
        ),

        "pick": (
            pick_actions
        ),

        "lift_up": (
            lift_actions
        ),

        "moving": (
            moving_actions
        ),

        "aligning": (
            aligning_actions
        ),

        "inserting": (
            inserting_actions
        ),
    }

    _validate_action_vectors(
        action_groups
    )

    assert (
        controller
        .motion_layer
        ._nut_phase
        == "inserted"
    )

    assert controller.execution_error is None

    assert (
        controller.grasp_plan_target
        == pick_target
    )

    # =============================================================
    # FINAL TCP pose
    # =============================================================

    final_tcp_position = (
        controller
        .motion_layer
        .current_pose
        .position
        .copy()
    )

    final_tcp_orientation = (
        controller
        .motion_layer
        .current_pose
        .orientation
        .copy()
    )

    # =============================================================
    # VISUAL DEBUG ARTIFACTS
    #
    # Persistent:
    # /test_real/test_task00_38/
    #     derived_artifacts/visual_debug/
    # =============================================================

    visual_artifacts = (
        _export_nut_visual_debug_artifacts(
            output_dir=(
                paths[
                    "visual_debug"
                ]
            ),

            front_rgb=(
                front_rgb
            ),

            front_depth=(
                front_depth
            ),

            front_K=(
                front_K
            ),

            front_camera_calibration=(
                front_camera_calibration
            ),

            T_base_table=(
                T_base_table
            ),

            ring_mask=(
                pick_mask
            ),

            pick_target=(
                pick_target
            ),

            place_target=(
                place_target
            ),

            real_grasp_tcp=(
                real_tcp
            ),

            real_grasp_quaternion=(
                real_quaternion
            ),

            final_tcp_position=(
                final_tcp_position
            ),

            final_tcp_orientation=(
                final_tcp_orientation
            ),

            hole_top_initial=(
                hole_top
            ),

            hole_bottom_initial=(
                hole_bottom
            ),

            hole_axis_initial=(
                hole_axis
            ),

            peg_support=(
                peg_support
            ),

            peg_tip=(
                peg_tip
            ),

            peg_axis=(
                peg_axis
            ),

            align_target_hole_bottom=(
                expected_aligned_bottom
            ),

            initial_entry_hole_bottom=(
                expected_initial_entry
            ),

            final_release_hole_bottom=(
                expected_final_bottom
            ),

            final_hole_top=(
                final_hole_top
            ),

            final_hole_axis=(
                final_hole_axis
            ),

            lift_tcp=(
                lift_tcp
            ),

            object_sweep_radius_m=float(
                controller
                .motion_layer
                ._nut_object_sweep_radius_m
            ),

            ring_thickness_m=(
                observed_ring_thickness
            ),

            peg_height_m=(
                observed_peg_height
            ),

            action_groups=(
                action_groups
            ),

            hole_top_local=(
                hole_top_local
            ),

            hole_bottom_local=(
                hole_bottom_local
            ),

            hole_axis_local=(
                hole_axis_local
            ),

            auto_release=False,
        )
    )

    # =============================================================
    # Numerical report
    # =============================================================

    report = {
        "case_dir": str(
            case_dir
        ),

        "mode": (
            "artifact_only_runtime_pipeline"
        ),

        "targets": {
            "pick": (
                pick_target
            ),

            "place": (
                place_target
            ),
        },

        "perception_geometry": {
            "ring_thickness_mm": (
                observed_ring_thickness
                * 1000.0
            ),

            "peg_height_mm": (
                observed_peg_height
                * 1000.0
            ),

            "initial_hole_peg_axis_angle_deg": (
                initial_hole_peg_angle_deg
            ),

            "peg_fit_rms_mm": (
                float(
                    place_geometry[
                        "rms_m"
                    ]
                )
                * 1000.0
            ),

            "peg_inlier_fraction": float(
                place_geometry[
                    "inlier_fraction"
                ]
            ),

            "peg_arc_coverage_deg": float(
                place_geometry[
                    "arc_coverage_deg"
                ]
            ),

            "support_mad_mm": (
                float(
                    place_geometry[
                        "support_mad_m"
                    ]
                )
                * 1000.0
            ),
        },

        "grasp": {
            "tcp_position_base": (
                real_tcp.tolist()
            ),

            "tcp_orientation_base": (
                real_quaternion.tolist()
            ),

            "approach_direction_base": (
                np.asarray(
                    real_grasp_result
                    .approach_direction_base,
                    dtype=np.float64,
                ).tolist()
            ),

            "confidence": float(
                real_grasp_result
                .confidence
            ),
        },

        "handoff": {
            "hole_top_roundtrip_error_mm": (
                handoff_top_error
                * 1000.0
            ),

            "hole_bottom_roundtrip_error_mm": (
                handoff_bottom_error
                * 1000.0
            ),

            "hole_axis_roundtrip_dot": (
                handoff_axis_dot
            ),

            "hole_top_local_m": (
                hole_top_local.tolist()
            ),

            "hole_bottom_local_m": (
                hole_bottom_local.tolist()
            ),

            "hole_axis_local": (
                hole_axis_local.tolist()
            ),

            "object_sweep_radius_mm": (
                float(
                    controller
                    .motion_layer
                    ._nut_object_sweep_radius_m
                )
                * 1000.0
            ),
        },

        "moving": {
            "hole_bottom_base": (
                moving_hole_bottom.tolist()
            ),

            "lateral_error_mm": (
                moving_lateral_error
                * 1000.0
            ),

            "axial_clearance_mm": (
                moving_axial_clearance
                * 1000.0
            ),

            "axis_dot": (
                moving_axis_dot
            ),
        },

        "aligning": {
            "hole_bottom_base": (
                aligned_hole_bottom.tolist()
            ),

            "position_error_mm": (
                aligning_error
                * 1000.0
            ),

            "axis_dot": (
                aligned_axis_dot
            ),

            "requested_clearance_mm": (
                20.0
            ),
        },

        "inserting": {
            "initial_entry_target_base": (
                expected_initial_entry.tolist()
            ),

            "final_hole_bottom_base": (
                final_hole_bottom.tolist()
            ),

            "final_position_error_mm": (
                final_position_error
                * 1000.0
            ),

            "final_lateral_error_mm": (
                final_lateral_error
                * 1000.0
            ),

            "final_height_above_support_mm": (
                final_support_height
                * 1000.0
            ),

            "axis_dot": (
                final_axis_dot
            ),

            "initial_entry_waypoint_error_mm": (
                initial_entry_error
                * 1000.0
            ),

            "max_insertion_lateral_error_mm": (
                float(
                    np.max(
                        insertion_lateral_errors
                    )
                )
                * 1000.0
            ),

            "auto_release": False,
        },

        "legacy_scene_state": {
            "destination_centroid_base": (
                saved_place_position.tolist()
            ),

            "centroid_to_refined_axis_lateral_bias_mm": (
                scene_centroid_lateral_bias
                * 1000.0
            ),
        },

        "action_counts": {
            primitive: len(
                actions
            )
            for primitive, actions
            in action_groups.items()
        },

        "visual_artifacts": {
            name: str(
                path
            )
            for name, path
            in visual_artifacts.items()
        },
    }

    report_path = (
        paths[
            "visual_debug"
        ]
        / "real_nut_pipeline_report.json"
    )

    _write_json(
        report_path,
        report,
    )

    # =============================================================
    # Human-readable report
    # =============================================================

    print()
    print("=" * 90)
    print("REAL NUT-ASSEMBLY COMPLETE ARTIFACT PIPELINE")
    print("=" * 90)

    print(
        "Case:",
        case_dir,
    )

    print(
        "Mode: artifact-only, no perception/planning inference"
    )

    print()

    print(
        "Front RGB:",
        front_rgb_key,
        front_rgb.shape,
    )

    print(
        "Front depth:",
        front_depth_key,
        front_depth.shape,
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
        "Ring thickness:",
        f"{observed_ring_thickness * 1000.0:.2f} mm",
    )

    print(
        "Peg height:",
        f"{observed_peg_height * 1000.0:.2f} mm",
    )

    print(
        "Initial hole/peg axis angle:",
        f"{initial_hole_peg_angle_deg:.3f} deg",
    )

    print(
        "Peg fit RMS:",
        f"{float(place_geometry['rms_m']) * 1000.0:.3f} mm",
    )

    print(
        "Peg inlier fraction:",
        f"{float(place_geometry['inlier_fraction']):.6f}",
    )

    print(
        "Peg arc coverage:",
        f"{float(place_geometry['arc_coverage_deg']):.2f} deg",
    )

    print()

    print(
        "REAL GraspGen-X TCP:",
        real_tcp,
    )

    print(
        "REAL GraspGen-X quaternion:",
        real_quaternion,
    )

    print(
        "REAL GraspGen-X confidence:",
        f"{float(real_grasp_result.confidence):.6f}",
    )

    print()

    print(
        "Initial hole bottom:",
        hole_bottom,
    )

    print(
        "Refined peg tip:",
        peg_tip,
    )

    print(
        "Refined peg support:",
        peg_support,
    )

    print()

    print(
        "ALIGN target hole bottom:",
        expected_aligned_bottom,
    )

    print(
        "INITIAL ENTRY target:",
        expected_initial_entry,
    )

    print(
        "FINAL placement/release target:",
        expected_final_bottom,
    )

    print()

    print(
        "Hole bottom local in REAL TCP:",
        hole_bottom_local,
    )

    print(
        "Hole axis local in REAL TCP:",
        hole_axis_local,
    )

    print(
        "Object sweep radius:",
        f"{controller.motion_layer._nut_object_sweep_radius_m * 1000.0:.2f} mm",
    )

    print(
        "Grasp handoff bottom error:",
        f"{handoff_bottom_error * 1000.0:.9f} mm",
    )

    print()

    print(
        "LIFT TCP:",
        lift_tcp,
    )

    print()

    print(
        "MOVING hole bottom:",
        moving_hole_bottom,
    )

    print(
        "MOVING lateral error:",
        f"{moving_lateral_error * 1000.0:.6f} mm",
    )

    print(
        "MOVING axial clearance:",
        f"{moving_axial_clearance * 1000.0:.2f} mm",
    )

    print(
        "MOVING axis dot:",
        f"{moving_axis_dot:.9f}",
    )

    print()

    print(
        "ALIGNING hole bottom:",
        aligned_hole_bottom,
    )

    print(
        "ALIGNING position error:",
        f"{aligning_error * 1000.0:.6f} mm",
    )

    print(
        "ALIGNING axis dot:",
        f"{aligned_axis_dot:.9f}",
    )

    print()

    print(
        "FINAL hole bottom:",
        final_hole_bottom,
    )

    print(
        "FINAL hole top:",
        final_hole_top,
    )

    print(
        "FINAL position error:",
        f"{final_position_error * 1000.0:.6f} mm",
    )

    print(
        "FINAL lateral error:",
        f"{final_lateral_error * 1000.0:.6f} mm",
    )

    print(
        "FINAL height above support:",
        f"{final_support_height * 1000.0:.2f} mm",
    )

    print(
        "FINAL axis dot:",
        f"{final_axis_dot:.9f}",
    )

    print(
        "Initial-entry waypoint error:",
        f"{initial_entry_error * 1000.0:.6f} mm",
    )

    print()

    print(
        "Old SceneState centroid lateral bias:",
        f"{scene_centroid_lateral_bias * 1000.0:.2f} mm",
    )

    print()

    print(
        "Primitive action counts:",
        {
            primitive: len(
                actions
            )
            for primitive, actions
            in action_groups.items()
        },
    )

    print()

    print(
        "Visual debug artifacts:"
    )

    for name, path in (
        visual_artifacts.items()
    ):

        print(
            f"  {name}: {path}"
        )

    print()

    print(
        "Pipeline report:",
        report_path,
    )

    print("=" * 90)