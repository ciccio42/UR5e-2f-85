from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import pickle
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
import yaml

from GroundingDINO.groundingdino.util import box_ops
from GroundingDINO.groundingdino.util.inference import (
    load_image_from_array,
    predict,
)

from ai_controller.models.seedo_controller.scene_perceiver import (
    ScenePerceiver,
)

from ai_controller.models.seedo_controller.utils import (
    crop_image_for_grounding_dino,
    remap_grounding_dino_boxes_to_full_frame,
)

from ai_controller.script_controller.vision import (
    ARUCO_TO_TABLE0_ROTATION,
    ARUCO_TO_TABLE0_TRANSLATION,
)


# =====================================================================
# Configuration
# =====================================================================


SCRIPT_VERSION = 1

DEFAULT_MODEL_CONFIG = (
    Path(__file__)
    .resolve()
    .parents[2]
    / "config"
    / "seedo_controller.yaml"
)


# =====================================================================
# Generic utilities
# =====================================================================


def _require_file(
    path: Path,
    *,
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
        description="JSON artifact",
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


def _load_yaml(
    path: Path,
) -> dict[str, Any]:

    path = _require_file(
        path,
        description="YAML configuration",
    )

    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        payload = yaml.safe_load(
            stream
        )

    if not isinstance(
        payload,
        dict,
    ):
        raise RuntimeError(
            f"Expected YAML mapping in {path}."
        )

    return payload


def _jsonable(
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
        dict,
    ):
        return {
            str(key): _jsonable(
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
            _jsonable(
                item
            )
            for item in value
        ]

    if isinstance(
        value,
        Path,
    ):
        return str(
            value
        )

    return value


def _write_json(
    path: Path,
    payload: Any,
) -> None:

    path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    with path.open(
        "w",
        encoding="utf-8",
    ) as stream:
        json.dump(
            _jsonable(
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


def _safe_filename(
    value: str,
) -> str:

    return (
        str(value)
        .strip()
        .replace(
            "/",
            "_",
        )
        .replace(
            "\\",
            "_",
        )
    )


# =====================================================================
# Canonical real-test layout
# =====================================================================


def _resolve_case_paths(
    case_dir: Path,
    model_config: Path,
) -> dict[str, Path]:

    case_dir = (
        Path(case_dir)
        .expanduser()
        .resolve()
    )

    if not case_dir.is_dir():
        raise FileNotFoundError(
            f"Real test case does not exist: {case_dir}"
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

        "grasp_dir": (
            case_dir
            / "grasp_planner"
            / "action_00"
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

        "model_config": (
            Path(model_config)
            .expanduser()
            .resolve()
        ),

        "derived_dir": (
            case_dir
            / "derived_artifacts"
        ),
    }

    for key in (
        "rollout",
        "raw_scene",
        "camera_noise",
        "primitive_plan",
        "grasp_plan",
        "grasp_diagnostics",
        "model_config",
    ):
        _require_file(
            paths[key],
            description=key,
        )

    return paths


# =====================================================================
# Dataset rollout loader
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
    rollout_path: Path,
) -> dict[str, Any]:

    _add_dataset_collector_scripts_to_path()

    with rollout_path.open(
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
            "Rollout root must be a dictionary."
        )

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
    mapping: dict[str, Any],
    keys: tuple[str, ...],
) -> tuple[
    str | None,
    Any,
]:

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
            "Unsupported front-camera RGB representation: "
            f"shape={image.shape}"
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
            "Front-camera depth must be 2D, "
            f"received {depth.shape}."
        )

    return (
        str(key),
        depth,
    )


# =====================================================================
# Primitive plan
# =====================================================================


def _extract_primitive_steps(
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
        "Could not find primitive-plan steps in "
        "primitive_plan.json."
    )


def _resolve_nut_targets(
    primitive_plan_payload: dict[str, Any],
) -> tuple[
    str,
    str,
]:

    steps = _extract_primitive_steps(
        primitive_plan_payload
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
            "Could not uniquely determine Nut-Assembly "
            f"pick target: {pick_targets}"
        )

    if len(
        place_unique
    ) != 1:
        raise RuntimeError(
            "Could not uniquely determine Nut-Assembly "
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
    raw_scene_payload: dict[str, Any],
    object_id: str,
) -> dict[str, Any]:

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

    if len(
        matches
    ) != 1:
        raise RuntimeError(
            "Could not uniquely find object "
            f"{object_id!r} in raw_scene_state.json. "
            f"Matches={len(matches)}."
        )

    return matches[
        0
    ]


# =====================================================================
# Recover front-camera intrinsics
# =====================================================================


def _recover_front_camera_matrix(
    raw_scene_payload: dict[str, Any],
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
            "At least three ScenePerceiver objects are "
            "required to recover front-camera intrinsics."
        )

    horizontal_rows = []
    horizontal_targets = []

    vertical_rows = []
    vertical_targets = []

    for obj in objects:

        pixel = np.asarray(
            obj.get(
                "pixel_coordinates"
            ),
            dtype=np.float64,
        )

        point_camera = np.asarray(
            obj.get(
                "position_camera"
            ),
            dtype=np.float64,
        )

        if (
            pixel.shape != (2,)
            or point_camera.shape != (3,)
        ):
            continue

        z = float(
            point_camera[
                2
            ]
        )

        if abs(
            z
        ) <= 1e-9:
            continue

        horizontal_rows.append(
            [
                point_camera[0] / z,
                1.0,
            ]
        )

        horizontal_targets.append(
            pixel[
                0
            ]
        )

        vertical_rows.append(
            [
                point_camera[1] / z,
                1.0,
            ]
        )

        vertical_targets.append(
            pixel[
                1
            ]
        )

    if len(
        horizontal_rows
    ) < 3:
        raise RuntimeError(
            "Insufficient valid points to recover "
            "the front-camera intrinsic matrix."
        )

    fx, cx = np.linalg.lstsq(
        np.asarray(
            horizontal_rows,
            dtype=np.float64,
        ),
        np.asarray(
            horizontal_targets,
            dtype=np.float64,
        ),
        rcond=None,
    )[0]

    fy, cy = np.linalg.lstsq(
        np.asarray(
            vertical_rows,
            dtype=np.float64,
        ),
        np.asarray(
            vertical_targets,
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
            f"Recovered invalid front-camera K:\n{K}"
        )

    errors = []

    for obj in objects:

        pixel = np.asarray(
            obj[
                "pixel_coordinates"
            ],
            dtype=np.float64,
        )

        point_camera = np.asarray(
            obj[
                "position_camera"
            ],
            dtype=np.float64,
        )

        z = float(
            point_camera[
                2
            ]
        )

        if abs(
            z
        ) <= 1e-9:
            continue

        projected = np.asarray(
            [
                fx
                * point_camera[0]
                / z
                + cx,

                fy
                * point_camera[1]
                / z
                + cy,
            ],
            dtype=np.float64,
        )

        errors.append(
            float(
                np.linalg.norm(
                    projected
                    - pixel
                )
            )
        )

    reprojection_rmse_px = float(
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
        reprojection_rmse_px,
    )


# =====================================================================
# Saved runtime transforms
# =====================================================================


def _load_runtime_camera_calibration(
    camera_noise_path: Path,
) -> dict[str, np.ndarray]:

    payload = _load_json(
        camera_noise_path
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
            "Saved camera position must have shape (3,)."
        )

    if orientation.shape != (
        3,
        3,
    ):
        raise RuntimeError(
            "Saved camera orientation must have shape (3,3)."
        )

    return {
        "position": position,
        "orientation_matrix": orientation,
    }


def _validate_transform_chain(
    *,
    raw_scene_payload: dict[str, Any],
    camera_calibration: dict[str, np.ndarray],
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

        point_camera = np.asarray(
            obj[
                "position_camera"
            ],
            dtype=np.float64,
        )

        expected_base = np.asarray(
            obj[
                "position_base"
            ],
            dtype=np.float64,
        )

        point_aruco = (
            R_camera_aruco
            @ point_camera
            + t_camera_aruco
        )

        point_table = (
            ARUCO_TO_TABLE0_ROTATION
            @ point_aruco
            + ARUCO_TO_TABLE0_TRANSLATION
        )

        reconstructed_base = (
            R_table_base
            @ point_table
            + t_table_base
        )

        errors.append(
            float(
                np.linalg.norm(
                    reconstructed_base
                    - expected_base
                )
            )
        )

    return float(
        max(
            errors
        )
    )


# =====================================================================
# Segmentation model
# =====================================================================


def _build_segmentation_perceiver(
    *,
    config: dict[str, Any],
    detector_labels: list[str],
) -> ScenePerceiver:

    visual = config[
        "visual_prompter"
    ]

    perception = config[
        "scene_perceiver"
    ]

    crop = (
        config
        .get(
            "grounding_dino",
            {},
        )
        .get(
            "crop",
            {},
        )
    )

    return ScenePerceiver(
        camera_calibration_path=(
            perception[
                "camera_calibration_path"
            ]
        ),

        camera_name=(
            perception.get(
                "camera_name",
                "zed_front",
            )
        ),

        grounding_config=(
            visual[
                "grounding_config"
            ]
        ),

        grounding_checkpoint=(
            visual[
                "grounding_checkpoint"
            ]
        ),

        bert_model=(
            visual[
                "bert_model"
            ]
        ),

        sam_checkpoint=(
            visual[
                "sam_checkpoint"
            ]
        ),

        detector_labels=(
            detector_labels
        ),

        camera_pose_noise_level="baseline",

        translation_noise_std_mm=0.0,
        rotation_noise_std_deg=0.0,

        perception_mode="prior_guided",

        grounding_dino_crop={
            "top_px": crop.get(
                "top_px",
                80,
            ),

            "bottom_px": crop.get(
                "bottom_px",
                0,
            ),

            "left_px": crop.get(
                "left_px",
                0,
            ),

            "right_px": crop.get(
                "right_px",
                0,
            ),
        },
    )


# =====================================================================
# Target-specific mask derivation
# =====================================================================


def _derive_target_mask(
    *,
    perceiver: ScenePerceiver,
    rgb: np.ndarray,
    saved_object: dict[str, Any],
    output_mask_path: Path,
    output_overlay_path: Path,
    output_diagnostics_path: Path,
    max_box_center_distance_px: float = 60.0,
    max_mask_centroid_distance_px: float = 40.0,
) -> tuple[
    np.ndarray,
    dict[str, Any],
]:

    object_id = str(
        saved_object[
            "object_id"
        ]
    ).strip()

    label = str(
        saved_object[
            "label"
        ]
    ).strip()

    saved_pixel = np.asarray(
        saved_object[
            "pixel_coordinates"
        ],
        dtype=np.float64,
    )

    if saved_pixel.shape != (2,):
        raise RuntimeError(
            f"Invalid saved pixel for {object_id!r}: "
            f"{saved_pixel}"
        )

    perceiver._ensure_models_loaded()

    (
        dino_rgb,
        crop_info,
    ) = crop_image_for_grounding_dino(
        rgb,

        top_px=(
            perceiver
            .grounding_dino_crop[
                "top_px"
            ]
        ),

        bottom_px=(
            perceiver
            .grounding_dino_crop[
                "bottom_px"
            ]
        ),

        left_px=(
            perceiver
            .grounding_dino_crop[
                "left_px"
            ]
        ),

        right_px=(
            perceiver
            .grounding_dino_crop[
                "right_px"
            ]
        ),
    )

    _, dino_image = (
        load_image_from_array(
            dino_rgb
        )
    )

    boxes, logits, phrases = predict(
        model=perceiver.grounding_model,
        image=dino_image,
        caption=label,
        box_threshold=0.20,
        text_threshold=0.25,
        device=perceiver.device,
    )

    if len(
        boxes
    ) == 0:
        raise RuntimeError(
            f"GroundingDINO returned no candidates for "
            f"{object_id!r} using label {label!r}."
        )

    # Same small-object sanity rule used by ScenePerceiver.
    box_areas = (
        boxes[
            :,
            2
        ]
        * boxes[
            :,
            3
        ]
    )

    keep = (
        box_areas
        < 0.1
    )

    boxes = boxes[
        keep
    ]

    logits = logits[
        keep
    ]

    keep_values = (
        keep
        .detach()
        .cpu()
        .tolist()
    )

    phrases = [
        phrase
        for phrase, accepted
        in zip(
            phrases,
            keep_values,
        )
        if accepted
    ]

    if len(
        boxes
    ) == 0:
        raise RuntimeError(
            f"Every DINO candidate for {object_id!r} "
            "was rejected by the small-object filter."
        )

    boxes_full = (
        remap_grounding_dino_boxes_to_full_frame(
            boxes,
            crop_info=crop_info,
        )
    )

    height, width = (
        rgb.shape[
            :2
        ]
    )

    boxes_np = (
        boxes_full
        .detach()
        .cpu()
        .numpy()
    )

    candidate_centers = np.column_stack(
        (
            boxes_np[
                :,
                0
            ]
            * width,

            boxes_np[
                :,
                1
            ]
            * height,
        )
    )

    center_distances = np.linalg.norm(
        candidate_centers
        - saved_pixel.reshape(
            1,
            2,
        ),
        axis=1,
    )

    selected_index = int(
        np.argmin(
            center_distances
        )
    )

    selected_distance = float(
        center_distances[
            selected_index
        ]
    )

    candidates = []

    for index in range(
        len(
            boxes_full
        )
    ):

        candidates.append(
            {
                "index": index,

                "center_px": (
                    candidate_centers[
                        index
                    ].tolist()
                ),

                "center_distance_px": float(
                    center_distances[
                        index
                    ]
                ),

                "confidence": float(
                    logits[
                        index
                    ]
                    .detach()
                    .cpu()
                    .item()
                ),

                "box_cxcywh_normalized_full": (
                    boxes_np[
                        index
                    ].tolist()
                ),
            }
        )

    if (
        selected_distance
        > max_box_center_distance_px
    ):
        raise RuntimeError(
            f"Closest DINO candidate for {object_id!r} "
            f"is {selected_distance:.2f}px from the "
            f"saved target pixel {saved_pixel.tolist()}. "
            f"Candidates={candidates}"
        )

    selected_box = (
        boxes_full[
            selected_index:
            selected_index + 1
        ]
    )

    selected_score = float(
        logits[
            selected_index
        ]
        .detach()
        .cpu()
        .item()
    )

    boxes_xyxy = (
        box_ops
        .box_cxcywh_to_xyxy(
            selected_box
        )
        * torch.tensor(
            [
                width,
                height,
                width,
                height,
            ],
            dtype=(
                selected_box.dtype
            ),
            device=(
                selected_box.device
            ),
        )
    )

    perceiver.sam_predictor.set_image(
        rgb
    )

    transformed_box = (
        perceiver
        .sam_predictor
        .transform
        .apply_boxes_torch(
            boxes_xyxy,
            rgb.shape[
                :2
            ],
        )
        .to(
            perceiver.device
        )
    )

    masks, _, _ = (
        perceiver
        .sam_predictor
        .predict_torch(
            point_coords=None,
            point_labels=None,
            boxes=transformed_box,
            multimask_output=False,
        )
    )

    mask = (
        masks[
            0,
            0,
        ]
        .detach()
        .cpu()
        .numpy()
        .astype(bool)
    )

    mask_area = int(
        np.count_nonzero(
            mask
        )
    )

    if mask_area == 0:
        raise RuntimeError(
            f"SAM produced an empty mask for {object_id!r}."
        )

    if (
        mask_area
        > 0.30
        * height
        * width
    ):
        raise RuntimeError(
            f"SAM mask for {object_id!r} covers "
            f"{100.0 * mask_area / (height * width):.2f}% "
            "of the image."
        )

    mask_pixels = np.argwhere(
        mask
    )

    avg_v, avg_u = np.mean(
        mask_pixels,
        axis=0,
    )

    mask_centroid = np.asarray(
        [
            avg_u,
            avg_v,
        ],
        dtype=np.float64,
    )

    mask_centroid_distance = float(
        np.linalg.norm(
            mask_centroid
            - saved_pixel
        )
    )

    if (
        mask_centroid_distance
        > max_mask_centroid_distance_px
    ):
        raise RuntimeError(
            f"Derived SAM mask for {object_id!r} is not "
            "consistent with the original ScenePerceiver "
            f"object. Saved pixel={saved_pixel.tolist()}, "
            f"derived centroid={mask_centroid.tolist()}, "
            f"distance={mask_centroid_distance:.2f}px."
        )

    output_mask_path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    saved = cv2.imwrite(
        str(
            output_mask_path
        ),
        (
            mask.astype(
                np.uint8
            )
            * 255
        ),
    )

    if not saved:
        raise RuntimeError(
            f"Could not save derived mask: "
            f"{output_mask_path}"
        )

    # -------------------------------------------------------------
    # Diagnostic overlay
    # -------------------------------------------------------------

    overlay = cv2.cvtColor(
        rgb,
        cv2.COLOR_RGB2BGR,
    )

    mask_color = np.zeros_like(
        overlay
    )

    mask_color[
        ...,
        1,
    ] = 255

    overlay = np.where(
        mask[
            ...,
            None
        ],
        (
            0.70
            * overlay
            + 0.30
            * mask_color
        ).astype(
            np.uint8
        ),
        overlay,
    )

    selected_xyxy = (
        boxes_xyxy[
            0
        ]
        .detach()
        .cpu()
        .numpy()
    )

    x1, y1, x2, y2 = [
        int(
            round(
                float(value)
            )
        )
        for value
        in selected_xyxy
    ]

    cv2.rectangle(
        overlay,
        (
            x1,
            y1,
        ),
        (
            x2,
            y2,
        ),
        (
            255,
            255,
            255,
        ),
        2,
    )

    saved_pixel_int = tuple(
        np.round(
            saved_pixel
        )
        .astype(int)
        .tolist()
    )

    selected_center_int = tuple(
        np.round(
            candidate_centers[
                selected_index
            ]
        )
        .astype(int)
        .tolist()
    )

    mask_centroid_int = tuple(
        np.round(
            mask_centroid
        )
        .astype(int)
        .tolist()
    )

    # Red: original pixel from raw_scene_state.
    cv2.circle(
        overlay,
        saved_pixel_int,
        7,
        (
            0,
            0,
            255,
        ),
        -1,
    )

    # Blue: selected DINO box center.
    cv2.circle(
        overlay,
        selected_center_int,
        6,
        (
            255,
            0,
            0,
        ),
        -1,
    )

    # Yellow: centroid of newly derived SAM mask.
    cv2.circle(
        overlay,
        mask_centroid_int,
        5,
        (
            0,
            255,
            255,
        ),
        -1,
    )

    output_overlay_path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    if not cv2.imwrite(
        str(
            output_overlay_path
        ),
        overlay,
    ):
        raise RuntimeError(
            "Could not save segmentation overlay: "
            f"{output_overlay_path}"
        )

    diagnostics = {
        "object_id": (
            object_id
        ),

        "label": (
            label
        ),

        "saved_scene_pixel": (
            saved_pixel.tolist()
        ),

        "candidate_count": int(
            len(
                boxes_full
            )
        ),

        "candidates": (
            candidates
        ),

        "selected_candidate_index": (
            selected_index
        ),

        "selected_confidence": (
            selected_score
        ),

        "selected_box_center_px": (
            candidate_centers[
                selected_index
            ].tolist()
        ),

        "selected_box_center_distance_px": (
            selected_distance
        ),

        "derived_mask_centroid_px": (
            mask_centroid.tolist()
        ),

        "derived_mask_centroid_distance_px": (
            mask_centroid_distance
        ),

        "mask_area_px": (
            mask_area
        ),

        "mask_area_fraction": float(
            mask_area
            / (
                height
                * width
            )
        ),
    }

    _write_json(
        output_diagnostics_path,
        diagnostics,
    )

    print(
        "[DERIVE MASK] "
        f"{object_id!r} | "
        f"label={label!r} | "
        f"saved_pixel="
        f"{saved_pixel.tolist()} | "
        f"dino_center="
        f"{candidate_centers[selected_index].tolist()} | "
        f"dino_error="
        f"{selected_distance:.2f}px | "
        f"mask_centroid="
        f"{mask_centroid.tolist()} | "
        f"mask_error="
        f"{mask_centroid_distance:.2f}px | "
        f"score={selected_score:.4f} | "
        f"area={mask_area}px",
        flush=True,
    )

    return (
        mask,
        diagnostics,
    )


def _load_mask(
    path: Path,
    *,
    expected_shape: tuple[int, int],
) -> np.ndarray:

    image = cv2.imread(
        str(
            path
        ),
        cv2.IMREAD_GRAYSCALE,
    )

    if image is None:
        raise RuntimeError(
            f"Could not read derived mask: {path}"
        )

    mask = (
        image
        > 0
    )

    if mask.shape != (
        expected_shape
    ):
        raise RuntimeError(
            f"Derived mask has wrong shape: "
            f"{path}: {mask.shape}, "
            f"expected={expected_shape}"
        )

    if not np.any(
        mask
    ):
        raise RuntimeError(
            f"Derived mask is empty: {path}"
        )

    return mask


# =====================================================================
# Geometry-only ScenePerceiver
# =====================================================================


def _build_geometry_perceiver(
    *,
    runtime_camera_calibration: dict[str, np.ndarray],
    T_base_table: np.ndarray,
) -> ScenePerceiver:

    perceiver = ScenePerceiver.__new__(
        ScenePerceiver
    )

    perceiver._last_runtime_camera_calibration = {
        "position": np.asarray(
            runtime_camera_calibration[
                "position"
            ],
            dtype=np.float64,
        ).copy(),

        "orientation_matrix": np.asarray(
            runtime_camera_calibration[
                "orientation_matrix"
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
# Refinement artifact helpers
# =====================================================================


def _load_target_refinement(
    path: Path,
    target_id: str,
) -> dict[str, Any]:

    payload = _load_json(
        path
    )

    normalized = (
        target_id
        .strip()
        .casefold()
    )

    matches = [
        refinement
        for object_id, refinement
        in payload.items()
        if (
            str(
                object_id
            )
            .strip()
            .casefold()
            == normalized
        )
    ]

    if len(
        matches
    ) != 1:
        raise RuntimeError(
            f"Could not uniquely load refinement "
            f"for {target_id!r} from {path}."
        )

    refinement = matches[
        0
    ]

    if not isinstance(
        refinement,
        dict,
    ):
        raise RuntimeError(
            f"Invalid refinement for {target_id!r}."
        )

    if not bool(
        refinement.get(
            "accepted",
            False,
        )
    ):
        raise RuntimeError(
            f"Saved refinement for {target_id!r} "
            "is not accepted."
        )

    return refinement


# =====================================================================
# Provenance
# =====================================================================


def _source_fingerprints(
    paths: dict[str, Path],
) -> dict[str, dict[str, str]]:

    names = (
        "rollout",
        "raw_scene",
        "camera_noise",
        "primitive_plan",
        "grasp_plan",
        "grasp_diagnostics",
        "model_config",
    )

    return {
        name: {
            "path": str(
                paths[
                    name
                ]
            ),

            "sha256": _sha256(
                paths[
                    name
                ]
            ),
        }
        for name in names
    }


def _validate_existing_manifest(
    *,
    manifest_path: Path,
    current_sources: dict[str, dict[str, str]],
    force: bool,
) -> None:

    if (
        not manifest_path.is_file()
        or force
    ):
        return

    manifest = _load_json(
        manifest_path
    )

    previous_sources = manifest.get(
        "sources"
    )

    if not isinstance(
        previous_sources,
        dict,
    ):
        raise RuntimeError(
            "Existing derivation manifest is invalid. "
            "Use --force to rebuild derived artifacts."
        )

    changed = []

    for name, current in current_sources.items():

        previous = previous_sources.get(
            name
        )

        if not isinstance(
            previous,
            dict,
        ):
            changed.append(
                name
            )
            continue

        if (
            previous.get(
                "sha256"
            )
            != current[
                "sha256"
            ]
        ):
            changed.append(
                name
            )

    if changed:
        raise RuntimeError(
            "Source artifacts changed since "
            "derived_artifacts were generated: "
            f"{changed}. "
            "Run again with --force to regenerate them."
        )


# =====================================================================
# Main preparation
# =====================================================================


def prepare_case(
    *,
    case_dir: Path,
    model_config: Path,
    force: bool,
) -> dict[str, Any]:

    paths = _resolve_case_paths(
        case_dir,
        model_config,
    )

    case_dir = paths[
        "case_dir"
    ]

    derived_dir = paths[
        "derived_dir"
    ]

    masks_dir = (
        derived_dir
        / "scene_masks"
    )

    segmentation_dir = (
        derived_dir
        / "segmentation_diagnostics"
    )

    pick_refinement_path = (
        derived_dir
        / "nut_pick_refinements.json"
    )

    place_refinement_path = (
        derived_dir
        / "nut_place_refinements.json"
    )

    camera_info_path = (
        derived_dir
        / "front_camera_info.json"
    )

    manifest_path = (
        derived_dir
        / "derivation_manifest.json"
    )

    derived_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    # -------------------------------------------------------------
    # Source provenance
    # -------------------------------------------------------------

    current_sources = (
        _source_fingerprints(
            paths
        )
    )

    _validate_existing_manifest(
        manifest_path=(
            manifest_path
        ),
        current_sources=(
            current_sources
        ),
        force=force,
    )

    # -------------------------------------------------------------
    # Load original run artifacts
    # -------------------------------------------------------------

    raw_scene_payload = (
        _load_json(
            paths[
                "raw_scene"
            ]
        )
    )

    primitive_plan_payload = (
        _load_json(
            paths[
                "primitive_plan"
            ]
        )
    )

    config = _load_yaml(
        paths[
            "model_config"
        ]
    )

    (
        pick_target,
        place_target,
    ) = _resolve_nut_targets(
        primitive_plan_payload
    )

    saved_pick = _get_saved_object(
        raw_scene_payload,
        pick_target,
    )

    saved_place = _get_saved_object(
        raw_scene_payload,
        place_target,
    )

    print()
    print("=" * 90)
    print("PREPARE REAL NUT-ASSEMBLY CASE")
    print("=" * 90)

    print(
        "Case:",
        case_dir,
    )

    print(
        "Pick target:",
        pick_target,
        "| label=",
        repr(
            saved_pick[
                "label"
            ]
        ),
        "| saved pixel=",
        saved_pick[
            "pixel_coordinates"
        ],
    )

    print(
        "Place target:",
        place_target,
        "| label=",
        repr(
            saved_place[
                "label"
            ]
        ),
        "| saved pixel=",
        saved_place[
            "pixel_coordinates"
        ],
    )

    # -------------------------------------------------------------
    # Exact t=0 front RGB-D from THIS rollout
    # -------------------------------------------------------------

    first_observation = (
        _load_first_rollout_observation(
            paths[
                "rollout"
            ]
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
            "Saved t=0 front RGB/depth dimensions differ: "
            f"rgb={front_rgb.shape[:2]}, "
            f"depth={front_depth.shape}"
        )

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

    # -------------------------------------------------------------
    # Recover K from original ScenePerceiver artifacts
    # -------------------------------------------------------------

    (
        front_K,
        reprojection_rmse_px,
    ) = _recover_front_camera_matrix(
        raw_scene_payload
    )

    if (
        reprojection_rmse_px
        >= 0.25
    ):
        raise RuntimeError(
            "Recovered front-camera K is not "
            "sufficiently consistent with the original "
            "ScenePerceiver output. "
            f"RMSE={reprojection_rmse_px:.6f}px."
        )

    camera_info_payload = {
        "k": (
            front_K
            .reshape(-1)
            .tolist()
        ),

        "matrix": (
            front_K.tolist()
        ),

        "reprojection_rmse_px": (
            reprojection_rmse_px
        ),

        "source": (
            "Recovered from raw_scene_state.json "
            "pixel_coordinates + position_camera."
        ),
    }

    if (
        force
        or not camera_info_path.is_file()
    ):
        _write_json(
            camera_info_path,
            camera_info_payload,
        )

        camera_info_status = (
            "generated"
        )

    else:

        existing_camera_info = (
            _load_json(
                camera_info_path
            )
        )

        existing_K = np.asarray(
            existing_camera_info[
                "matrix"
            ],
            dtype=np.float64,
        )

        if not np.allclose(
            existing_K,
            front_K,
            atol=1e-6,
            rtol=0.0,
        ):
            raise RuntimeError(
                "Existing derived front_camera_info.json "
                "does not match current source artifacts. "
                "Use --force."
            )

        camera_info_status = (
            "reused"
        )

    print()
    print(
        "Recovered front K:"
    )
    print(
        front_K
    )

    print(
        "Front K reprojection RMSE:",
        f"{reprojection_rmse_px:.6f} px",
    )

    # -------------------------------------------------------------
    # Exact saved camera pose
    # -------------------------------------------------------------

    runtime_camera_calibration = (
        _load_runtime_camera_calibration(
            paths[
                "camera_noise"
            ]
        )
    )

    # -------------------------------------------------------------
    # T_base_table from real GraspGen-X artifact
    # -------------------------------------------------------------

    diagnostics = np.load(
        paths[
            "grasp_diagnostics"
        ]
    )

    if (
        "T_base_table"
        not in diagnostics.files
    ):
        raise RuntimeError(
            "graspgenx_diagnostics.npz does not contain "
            "T_base_table."
        )

    T_base_table = np.asarray(
        diagnostics[
            "T_base_table"
        ],
        dtype=np.float64,
    )

    if T_base_table.shape != (
        4,
        4,
    ):
        raise RuntimeError(
            "Saved T_base_table must have shape (4,4)."
        )

    # -------------------------------------------------------------
    # Validate that front calibration + T_base_table really belong
    # to this recorded run.
    # -------------------------------------------------------------

    transform_error_m = (
        _validate_transform_chain(
            raw_scene_payload=(
                raw_scene_payload
            ),

            camera_calibration=(
                runtime_camera_calibration
            ),

            T_base_table=(
                T_base_table
            ),
        )
    )

    if (
        transform_error_m
        >= 0.005
    ):
        raise RuntimeError(
            "Saved calibration/T_base_table do not "
            "reconstruct the original ScenePerceiver "
            "positions closely enough. "
            f"Maximum error="
            f"{transform_error_m * 1000.0:.3f} mm."
        )

    print(
        "Saved transform-chain max error:",
        f"{transform_error_m * 1000.0:.3f} mm",
    )

    # -------------------------------------------------------------
    # Derived target mask paths
    # -------------------------------------------------------------

    pick_mask_path = (
        masks_dir
        / (
            _safe_filename(
                pick_target
            )
            + ".png"
        )
    )

    place_mask_path = (
        masks_dir
        / (
            _safe_filename(
                place_target
            )
            + ".png"
        )
    )

    # -------------------------------------------------------------
    # Load model ONLY if one of the masks is missing.
    # -------------------------------------------------------------

    must_generate_pick_mask = (
        force
        or not pick_mask_path.is_file()
    )

    must_generate_place_mask = (
        force
        or not place_mask_path.is_file()
    )

    segmentation_perceiver = None

    if (
        must_generate_pick_mask
        or must_generate_place_mask
    ):

        print()
        print(
            "At least one target mask is missing."
        )

        print(
            "Loading GroundingDINO + SAM ONLY to derive "
            "the missing artifact(s)."
        )

        segmentation_perceiver = (
            _build_segmentation_perceiver(
                config=config,

                detector_labels=[
                    str(
                        saved_pick[
                            "label"
                        ]
                    ),

                    str(
                        saved_place[
                            "label"
                        ]
                    ),
                ],
            )
        )

    # -------------------------------------------------------------
    # PICK mask
    # -------------------------------------------------------------

    if must_generate_pick_mask:

        (
            pick_mask,
            pick_mask_diagnostics,
        ) = _derive_target_mask(
            perceiver=(
                segmentation_perceiver
            ),

            rgb=(
                front_rgb
            ),

            saved_object=(
                saved_pick
            ),

            output_mask_path=(
                pick_mask_path
            ),

            output_overlay_path=(
                segmentation_dir
                / (
                    _safe_filename(
                        pick_target
                    )
                    + "_overlay.png"
                )
            ),

            output_diagnostics_path=(
                segmentation_dir
                / (
                    _safe_filename(
                        pick_target
                    )
                    + ".json"
                )
            ),
        )

        pick_mask_status = (
            "generated"
        )

    else:

        pick_mask = _load_mask(
            pick_mask_path,

            expected_shape=(
                front_depth.shape
            ),
        )

        pick_mask_diagnostics_path = (
            segmentation_dir
            / (
                _safe_filename(
                    pick_target
                )
                + ".json"
            )
        )

        pick_mask_diagnostics = (
            _load_json(
                pick_mask_diagnostics_path
            )
            if pick_mask_diagnostics_path.is_file()
            else {
                "status": (
                    "existing mask reused"
                )
            }
        )

        pick_mask_status = (
            "reused"
        )

        print()
        print(
            "[REUSE MASK]",
            pick_target,
            "->",
            pick_mask_path,
        )

    # -------------------------------------------------------------
    # PLACE mask
    # -------------------------------------------------------------

    if must_generate_place_mask:

        (
            place_mask,
            place_mask_diagnostics,
        ) = _derive_target_mask(
            perceiver=(
                segmentation_perceiver
            ),

            rgb=(
                front_rgb
            ),

            saved_object=(
                saved_place
            ),

            output_mask_path=(
                place_mask_path
            ),

            output_overlay_path=(
                segmentation_dir
                / (
                    _safe_filename(
                        place_target
                    )
                    + "_overlay.png"
                )
            ),

            output_diagnostics_path=(
                segmentation_dir
                / (
                    _safe_filename(
                        place_target
                    )
                    + ".json"
                )
            ),
        )

        place_mask_status = (
            "generated"
        )

    else:

        place_mask = _load_mask(
            place_mask_path,

            expected_shape=(
                front_depth.shape
            ),
        )

        place_mask_diagnostics_path = (
            segmentation_dir
            / (
                _safe_filename(
                    place_target
                )
                + ".json"
            )
        )

        place_mask_diagnostics = (
            _load_json(
                place_mask_diagnostics_path
            )
            if place_mask_diagnostics_path.is_file()
            else {
                "status": (
                    "existing mask reused"
                )
            }
        )

        place_mask_status = (
            "reused"
        )

        print(
            "[REUSE MASK]",
            place_target,
            "->",
            place_mask_path,
        )

    # Models no longer needed.
    segmentation_perceiver = None

    # -------------------------------------------------------------
    # Geometry-only ScenePerceiver.
    #
    # No inference occurs below this point.
    # -------------------------------------------------------------

    geometry_perceiver = (
        _build_geometry_perceiver(
            runtime_camera_calibration=(
                runtime_camera_calibration
            ),

            T_base_table=(
                T_base_table
            ),
        )
    )

    # -------------------------------------------------------------
    # Pick refinement
    # -------------------------------------------------------------

    if (
        not force
        and pick_refinement_path.is_file()
    ):

        pick_refinement = (
            _load_target_refinement(
                pick_refinement_path,
                pick_target,
            )
        )

        pick_refinement_status = (
            "reused"
        )

        print()
        print(
            "[REUSE REFINEMENT]",
            pick_target,
            "->",
            pick_refinement_path,
        )

    else:

        print()
        print(
            "[REFINE PICK]",
            pick_target,
        )

        pick_refinement = (
            geometry_perceiver
            .refine_nut_hole(
                mask=(
                    pick_mask
                ),

                depth_image=(
                    front_depth
                ),

                camera_matrix=(
                    front_K
                ),
            )
        )

        if not bool(
            pick_refinement.get(
                "accepted",
                False,
            )
        ):
            raise RuntimeError(
                "Nut-hole refinement returned "
                "accepted=False."
            )

        _write_json(
            pick_refinement_path,
            {
                pick_target: (
                    pick_refinement
                )
            },
        )

        pick_refinement_status = (
            "generated"
        )

    # -------------------------------------------------------------
    # Place refinement
    # -------------------------------------------------------------

    if (
        not force
        and place_refinement_path.is_file()
    ):

        place_refinement = (
            _load_target_refinement(
                place_refinement_path,
                place_target,
            )
        )

        place_refinement_status = (
            "reused"
        )

        print(
            "[REUSE REFINEMENT]",
            place_target,
            "->",
            place_refinement_path,
        )

    else:

        print()
        print(
            "[REFINE PLACE]",
            place_target,
        )

        place_refinement = (
            geometry_perceiver
            .refine_peg_axis(
                mask=(
                    place_mask
                ),

                depth_image=(
                    front_depth
                ),

                camera_matrix=(
                    front_K
                ),
            )
        )

        if not bool(
            place_refinement.get(
                "accepted",
                False,
            )
        ):
            raise RuntimeError(
                "Peg refinement returned accepted=False."
            )

        _write_json(
            place_refinement_path,
            {
                place_target: (
                    place_refinement
                )
            },
        )

        place_refinement_status = (
            "generated"
        )

    # -------------------------------------------------------------
    # Numerical cross-checks
    # -------------------------------------------------------------

    hole_top = np.asarray(
        pick_refinement[
            "hole_center_base"
        ],
        dtype=np.float64,
    )

    hole_bottom = np.asarray(
        pick_refinement[
            "hole_bottom_center_base"
        ],
        dtype=np.float64,
    )

    hole_axis = np.asarray(
        pick_refinement[
            "hole_axis_base"
        ],
        dtype=np.float64,
    )

    hole_axis /= np.linalg.norm(
        hole_axis
    )

    peg_tip = np.asarray(
        place_refinement[
            "peg_tip_point_base"
        ],
        dtype=np.float64,
    )

    peg_support = np.asarray(
        place_refinement[
            "peg_support_point_base"
        ],
        dtype=np.float64,
    )

    peg_axis = np.asarray(
        place_refinement[
            "axis_direction_base"
        ],
        dtype=np.float64,
    )

    peg_axis /= np.linalg.norm(
        peg_axis
    )

    measured_ring_thickness_m = float(
        np.dot(
            hole_top
            - hole_bottom,
            hole_axis,
        )
    )

    measured_peg_height_m = float(
        np.dot(
            peg_tip
            - peg_support,
            peg_axis,
        )
    )

    hole_peg_axis_dot = float(
        np.dot(
            hole_axis,
            peg_axis,
        )
    )

    hole_peg_axis_angle_deg = float(
        np.degrees(
            np.arccos(
                np.clip(
                    hole_peg_axis_dot,
                    -1.0,
                    1.0,
                )
            )
        )
    )

    if measured_ring_thickness_m <= 0.0:
        raise RuntimeError(
            "Derived ring thickness is not positive."
        )

    if measured_peg_height_m <= 0.0:
        raise RuntimeError(
            "Derived peg height is not positive."
        )

    # -------------------------------------------------------------
    # Output fingerprints
    # -------------------------------------------------------------

    generated_paths = {
        "front_camera_info": (
            camera_info_path
        ),

        "pick_mask": (
            pick_mask_path
        ),

        "place_mask": (
            place_mask_path
        ),

        "pick_refinement": (
            pick_refinement_path
        ),

        "place_refinement": (
            place_refinement_path
        ),
    }

    derived_fingerprints = {
        name: {
            "path": str(
                path
            ),

            "sha256": _sha256(
                path
            ),
        }
        for name, path
        in generated_paths.items()
    }

    # -------------------------------------------------------------
    # Manifest
    # -------------------------------------------------------------

    manifest = {
        "schema_version": (
            SCRIPT_VERSION
        ),

        "case_dir": str(
            case_dir
        ),

        "policy": {
            "use_existing_artifacts_first": True,

            "generate_only_missing_artifacts": (
                not force
            ),

            "force_regeneration": (
                bool(
                    force
                )
            ),

            "new_vlm_inference": False,

            "target_mask_derivation": (
                "GroundingDINO query using saved semantic "
                "label -> candidate nearest saved "
                "ScenePerceiver pixel -> SAM."
            ),
        },

        "sources": (
            current_sources
        ),

        "runtime_input": {
            "front_rgb_key": (
                front_rgb_key
            ),

            "front_depth_key": (
                front_depth_key
            ),

            "rgb_shape": list(
                front_rgb.shape
            ),

            "depth_shape": list(
                front_depth.shape
            ),
        },

        "targets": {
            "pick": {
                "object_id": (
                    pick_target
                ),

                "label": (
                    saved_pick[
                        "label"
                    ]
                ),

                "saved_pixel": (
                    saved_pick[
                        "pixel_coordinates"
                    ]
                ),
            },

            "place": {
                "object_id": (
                    place_target
                ),

                "label": (
                    saved_place[
                        "label"
                    ]
                ),

                "saved_pixel": (
                    saved_place[
                        "pixel_coordinates"
                    ]
                ),
            },
        },

        "front_camera": {
            "K": (
                front_K.tolist()
            ),

            "reprojection_rmse_px": (
                reprojection_rmse_px
            ),

            "transform_chain_max_error_m": (
                transform_error_m
            ),
        },

        "artifact_status": {
            "front_camera_info": (
                camera_info_status
            ),

            "pick_mask": (
                pick_mask_status
            ),

            "place_mask": (
                place_mask_status
            ),

            "pick_refinement": (
                pick_refinement_status
            ),

            "place_refinement": (
                place_refinement_status
            ),
        },

        "segmentation_diagnostics": {
            "pick": (
                pick_mask_diagnostics
            ),

            "place": (
                place_mask_diagnostics
            ),
        },

        "derived_geometry_summary": {
            "ring_thickness_m": (
                measured_ring_thickness_m
            ),

            "peg_height_m": (
                measured_peg_height_m
            ),

            "hole_peg_axis_dot": (
                hole_peg_axis_dot
            ),

            "hole_peg_axis_angle_deg": (
                hole_peg_axis_angle_deg
            ),

            "object_radius_from_hole_m": (
                pick_refinement.get(
                    "object_radius_from_hole_m"
                )
            ),

            "peg_fit_rms_m": (
                place_refinement.get(
                    "rms_m"
                )
            ),

            "peg_inlier_fraction": (
                place_refinement.get(
                    "inlier_fraction"
                )
            ),

            "peg_arc_coverage_deg": (
                place_refinement.get(
                    "arc_coverage_deg"
                )
            ),
        },

        "derived_artifacts": (
            derived_fingerprints
        ),
    }

    _write_json(
        manifest_path,
        manifest,
    )

    # -------------------------------------------------------------
    # Final report
    # -------------------------------------------------------------

    print()
    print("=" * 90)
    print("REAL CASE PREPARATION COMPLETE")
    print("=" * 90)

    print(
        "Case:",
        case_dir,
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
        "Front K reprojection RMSE:",
        f"{reprojection_rmse_px:.6f} px",
    )

    print(
        "Transform-chain max error:",
        f"{transform_error_m * 1000.0:.3f} mm",
    )

    print()

    print(
        "Pick mask:",
        pick_mask_status,
        "->",
        pick_mask_path,
    )

    print(
        "Place mask:",
        place_mask_status,
        "->",
        place_mask_path,
    )

    print()

    print(
        "Ring hole top:",
        hole_top,
    )

    print(
        "Ring hole bottom:",
        hole_bottom,
    )

    print(
        "Ring hole axis:",
        hole_axis,
    )

    print(
        "Ring thickness:",
        f"{measured_ring_thickness_m * 1000.0:.2f} mm",
    )

    print(
        "Ring radius from hole:",
        f"{float(pick_refinement['object_radius_from_hole_m']) * 1000.0:.2f} mm",
    )

    print()

    print(
        "Peg support:",
        peg_support,
    )

    print(
        "Peg tip:",
        peg_tip,
    )

    print(
        "Peg axis:",
        peg_axis,
    )

    print(
        "Peg height:",
        f"{measured_peg_height_m * 1000.0:.2f} mm",
    )

    if (
        place_refinement.get(
            "rms_m"
        )
        is not None
    ):
        print(
            "Peg fit RMS:",
            f"{float(place_refinement['rms_m']) * 1000.0:.3f} mm",
        )

    if (
        place_refinement.get(
            "inlier_fraction"
        )
        is not None
    ):
        print(
            "Peg inlier fraction:",
            f"{float(place_refinement['inlier_fraction']):.6f}",
        )

    if (
        place_refinement.get(
            "arc_coverage_deg"
        )
        is not None
    ):
        print(
            "Peg arc coverage:",
            f"{float(place_refinement['arc_coverage_deg']):.2f} deg",
        )

    print()

    print(
        "Hole/Peg axis dot:",
        f"{hole_peg_axis_dot:.9f}",
    )

    print(
        "Hole/Peg axis angle:",
        f"{hole_peg_axis_angle_deg:.3f} deg",
    )

    print()

    print(
        "Pick refinement:",
        pick_refinement_status,
        "->",
        pick_refinement_path,
    )

    print(
        "Place refinement:",
        place_refinement_status,
        "->",
        place_refinement_path,
    )

    print()

    print(
        "Manifest:",
        manifest_path,
    )

    print("=" * 90)

    return manifest


# =====================================================================
# CLI
# =====================================================================


def _parse_args() -> argparse.Namespace:

    parser = argparse.ArgumentParser(
        description=(
            "Prepare missing Nut-Assembly derived artifacts "
            "for a saved real SeeDo test case."
        )
    )

    parser.add_argument(
        "--case-dir",
        type=Path,
        required=True,
        help=(
            "Saved real test directory, for example "
            "/test_real/test_task00_38"
        ),
    )

    parser.add_argument(
        "--model-config",
        type=Path,
        default=(
            DEFAULT_MODEL_CONFIG
        ),
        help=(
            "SeeDo controller YAML. Used only when a "
            "missing mask must be derived."
        ),
    )

    parser.add_argument(
        "--force",
        action="store_true",
        help=(
            "Regenerate derived masks/refinements even "
            "when they already exist."
        ),
    )

    return parser.parse_args()


def main() -> int:

    args = _parse_args()

    prepare_case(
        case_dir=(
            args.case_dir
        ),

        model_config=(
            args.model_config
        ),

        force=bool(
            args.force
        ),
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(
        main()
    )