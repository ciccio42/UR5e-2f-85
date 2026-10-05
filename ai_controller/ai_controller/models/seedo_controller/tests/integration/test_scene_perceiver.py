from __future__ import annotations

import argparse
import json
import math
import yaml
from pathlib import Path

import cv2
import numpy as np

from ..common import build_scene_perception_result


EXPECTED_SCENE_DIR = Path(
    "/scene_capture/without_distractors/scene_1_no_distractors"
)

EXPECTED_TOTAL_OBJECTS = 8
EXPECTED_STORAGE_BINS = 4
EXPECTED_MANIPULABLE_OBJECTS = 4
EXPECTED_COLORS = {
    "red",
    "green",
    "blue",
    "yellow",
}


def _load_json(path: Path) -> dict:
    if not path.is_file():
        raise AssertionError(
            f"Missing ScenePerceiver artifact: {path}"
        )

    if path.stat().st_size == 0:
        raise AssertionError(
            f"ScenePerceiver artifact is empty: {path}"
        )

    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        value = json.load(stream)

    if not isinstance(value, dict):
        raise AssertionError(
            f"Expected a JSON object in {path}."
        )

    return value


def _assert_finite_vector(
    values,
    *,
    length: int,
    name: str,
) -> tuple[float, ...]:
    if len(values) != length:
        raise AssertionError(
            f"{name} must contain exactly {length} values: "
            f"{values!r}"
        )

    normalized = tuple(
        float(value)
        for value in values
    )

    if not all(
        math.isfinite(value)
        for value in normalized
    ):
        raise AssertionError(
            f"{name} contains non-finite values: {normalized}"
        )

    return normalized


def _expected_serialized_object(obj) -> dict:
    return {
        "object_id": obj.object_id,
        "label": obj.label,
        "pixel_coordinates": list(
            obj.pixel_coordinates
        ),
        "confidence": obj.confidence,
        "position_camera": list(
            obj.position_camera
        ),
        "position_base": list(
            obj.position_base
        ),
        "category": obj.category,
        "attributes": dict(
            obj.attributes
        ),
    }


def run_scene_perceiver_test(
    args: argparse.Namespace,
) -> int:
    """Run the real generalized ScenePerceiver integration test."""

    if args.scene_dir is None:
        raise ValueError(
            "--scene-dir is required for the scene_perceiver test."
        )

    if args.base_to_table_transform is None:
        raise ValueError(
            "--base-to-table-transform is required for the "
            "scene_perceiver test."
        )

    if not args.model_config:
        raise ValueError(
            "--model-config is required for the scene_perceiver test."
        )

    if args.artifacts_dir is None:
        raise ValueError(
            "--artifacts-dir is required for this integration test so "
            "the ScenePerceiver handoff artifacts are preserved."
        )

    scene_dir = (
        Path(args.scene_dir)
        .expanduser()
        .resolve()
    )

    expected_scene_dir = (
        EXPECTED_SCENE_DIR
        .expanduser()
        .resolve()
    )

    if scene_dir != expected_scene_dir:
        raise ValueError(
            "This integration test is pinned to the standard "
            "no-distractors scene. "
            f"Expected {expected_scene_dir}, received {scene_dir}."
        )

    expected_transform = (
        scene_dir
        / "base_to_table_transform.yaml"
    ).resolve()

    provided_transform = (
        Path(args.base_to_table_transform)
        .expanduser()
        .resolve()
    )

    if provided_transform != expected_transform:
        raise ValueError(
            "The base-to-table transform must belong to the standard "
            "test scene. "
            f"Expected {expected_transform}, received {provided_transform}."
        )

    rgb_path = scene_dir / "rgb.png"
    depth_path = scene_dir / "depth.npy"
    camera_info_path = (
        scene_dir
        / "camera_info.yaml"
    )

    for required_path in (
        rgb_path,
        depth_path,
        camera_info_path,
        provided_transform,
    ):
        if not required_path.is_file():
            raise FileNotFoundError(
                "Missing standard runtime-scene input: "
                f"{required_path}"
            )

    rgb_bgr = cv2.imread(
        str(rgb_path),
        cv2.IMREAD_COLOR,
    )

    if rgb_bgr is None:
        raise RuntimeError(
            f"Could not read RGB image: {rgb_path}"
        )

    depth = np.load(
        depth_path
    )

    if depth.ndim != 2:
        raise AssertionError(
            "Standard runtime depth image must be 2D, "
            f"received shape={depth.shape}."
        )

    height, width = rgb_bgr.shape[:2]

    if depth.shape != (
        height,
        width,
    ):
        raise AssertionError(
            "RGB/depth shape mismatch in standard runtime scene: "
            f"rgb={(height, width)}, depth={depth.shape}"
        )

    perception_result = (
        build_scene_perception_result(
            args
        )
    )

    raw_scene = (
        perception_result.raw_scene
    )

    # ---------------------------------------------------------
    # Validate complete runtime scene.
    # ---------------------------------------------------------

    if len(raw_scene.objects) != EXPECTED_TOTAL_OBJECTS:
        raise AssertionError(
            "Unexpected number of perceived objects in the standard "
            "no-distractors scene: "
            f"expected={EXPECTED_TOTAL_OBJECTS}, "
            f"received={len(raw_scene.objects)}"
        )

    raw_ids = [
        obj.object_id
        for obj
        in raw_scene.objects
    ]

    if len(raw_ids) != len(
        set(raw_ids)
    ):
        raise AssertionError(
            "ScenePerceiver produced duplicate object IDs."
        )

    storage_bins = []
    manipulable_objects = []
    observed_colors: set[str] = set()

    expected_id_counts: dict[str, int] = {}

    for obj in raw_scene.objects:
        if not obj.object_id.strip():
            raise AssertionError(
                "Detected object has an empty object ID."
            )

        if not obj.label.strip():
            raise AssertionError(
                f"{obj.object_id} has an empty detector label."
            )

        normalized_label = (
            obj.label
            .strip()
            .lower()
        )

        expected_prefix = (
            normalized_label
            .replace(" ", "_")
        )

        occurrence_index = (
            expected_id_counts.get(
                normalized_label,
                0,
            )
        )

        expected_object_id = (
            f"{expected_prefix}_{occurrence_index}"
        )

        if obj.object_id != expected_object_id:
            raise AssertionError(
                "ScenePerceiver object ID is inconsistent with its "
                "detector label/order: "
                f"expected={expected_object_id!r}, "
                f"received={obj.object_id!r}"
            )

        expected_id_counts[
            normalized_label
        ] = occurrence_index + 1

        if len(obj.pixel_coordinates) != 2:
            raise AssertionError(
                f"{obj.object_id} has invalid pixel coordinates."
            )

        pixel_x = int(
            obj.pixel_coordinates[0]
        )
        pixel_y = int(
            obj.pixel_coordinates[1]
        )

        if not (
            0 <= pixel_x < width
            and 0 <= pixel_y < height
        ):
            raise AssertionError(
                f"{obj.object_id} centroid is outside the RGB image: "
                f"pixel={obj.pixel_coordinates}, image={width}x{height}"
            )

        position_camera = _assert_finite_vector(
            obj.position_camera,
            length=3,
            name=(
                f"{obj.object_id}.position_camera"
            ),
        )

        _assert_finite_vector(
            obj.position_base,
            length=3,
            name=(
                f"{obj.object_id}.position_base"
            ),
        )

        if position_camera[2] <= 0.0:
            raise AssertionError(
                f"{obj.object_id} has non-positive camera depth: "
                f"{position_camera[2]}"
            )

        if obj.mask is None:
            raise AssertionError(
                f"{obj.object_id} has no segmentation mask."
            )

        mask = np.asarray(
            obj.mask
        )

        if mask.shape != (
            height,
            width,
        ):
            raise AssertionError(
                f"{obj.object_id} mask shape does not match RGB: "
                f"mask={mask.shape}, rgb={(height, width)}"
            )

        mask_area = int(
            np.count_nonzero(mask)
        )

        if mask_area <= 0:
            raise AssertionError(
                f"{obj.object_id} has an empty segmentation mask."
            )

        if mask_area > (
            height * width * 0.3
        ):
            raise AssertionError(
                f"{obj.object_id} mask exceeds the production sanity "
                f"threshold: area={mask_area}."
            )

        if obj.confidence is None:
            raise AssertionError(
                f"{obj.object_id} has no GroundingDINO confidence."
            )

        confidence = float(
            obj.confidence
        )

        if not math.isfinite(
            confidence
        ):
            raise AssertionError(
                f"{obj.object_id} has non-finite confidence: "
                f"{confidence}"
            )

        category = str(
            obj.category
            if obj.category is not None
            else ""
        ).strip().lower()

        if not category:
            raise AssertionError(
                f"{obj.object_id} is missing generalized category metadata."
            )

        if not isinstance(
            obj.attributes,
            dict,
        ):
            raise AssertionError(
                f"{obj.object_id} attributes are not a dictionary."
            )

        if category == "bin":
            storage_bins.append(
                obj
            )

            if normalized_label != "storage bin":
                raise AssertionError(
                    "Generalized storage-bin detector label was not "
                    "normalized: "
                    f"object={obj.object_id}, label={obj.label!r}"
                )

            if obj.attributes:
                raise AssertionError(
                    f"{obj.object_id} storage-bin attributes should be empty: "
                    f"{obj.attributes}"
                )

        else:
            manipulable_objects.append(
                obj
            )

            color = str(
                obj.attributes.get(
                    "color",
                    "",
                )
            ).strip().lower()

            if not color:
                raise AssertionError(
                    f"{obj.object_id} is missing the visible color attribute."
                )

            if color in observed_colors:
                raise AssertionError(
                    "Duplicate manipulable-object color in standard scene: "
                    f"{color!r}"
                )

            if color not in normalized_label.split():
                raise AssertionError(
                    "Detector label does not contain the authoritative "
                    "visible color: "
                    f"object={obj.object_id}, color={color!r}, "
                    f"label={obj.label!r}"
                )

            observed_colors.add(
                color
            )

    if len(storage_bins) != EXPECTED_STORAGE_BINS:
        raise AssertionError(
            "Unexpected number of runtime storage bins: "
            f"expected={EXPECTED_STORAGE_BINS}, "
            f"received={len(storage_bins)}"
        )

    if (
        len(manipulable_objects)
        != EXPECTED_MANIPULABLE_OBJECTS
    ):
        raise AssertionError(
            "Unexpected number of manipulable runtime objects: "
            f"expected={EXPECTED_MANIPULABLE_OBJECTS}, "
            f"received={len(manipulable_objects)}"
        )

    if observed_colors != EXPECTED_COLORS:
        raise AssertionError(
            "Unexpected manipulable-object colors in standard scene: "
            f"expected={sorted(EXPECTED_COLORS)}, "
            f"received={sorted(observed_colors)}"
        )

    expected_bin_ids = {
        f"storage_bin_{index}"
        for index in range(
            EXPECTED_STORAGE_BINS
        )
    }

    returned_bin_ids = {
        obj.object_id
        for obj
        in storage_bins
    }

    if returned_bin_ids != expected_bin_ids:
        raise AssertionError(
            "Unexpected storage-bin object IDs: "
            f"expected={sorted(expected_bin_ids)}, "
            f"received={sorted(returned_bin_ids)}"
        )

    # ---------------------------------------------------------
    # Validate persistent artifacts.
    # ---------------------------------------------------------

    artifacts_dir = (
        Path(args.artifacts_dir)
        .expanduser()
        .resolve()
    )

    dino_input_path = (
        artifacts_dir
        / "groundingdino_input.png"
    )

    if not dino_input_path.is_file():
        raise AssertionError(
            "ScenePerceiver did not persist the cropped "
            "GroundingDINO input image: "
            f"{dino_input_path}"
        )

    dino_input = cv2.imread(
        str(dino_input_path),
        cv2.IMREAD_COLOR,
    )

    if dino_input is None:
        raise AssertionError(
            "GroundingDINO input artifact is not readable: "
            f"{dino_input_path}"
        )

    with Path(
        args.model_config
    ).expanduser().resolve().open(
        "r",
        encoding="utf-8",
    ) as stream:
        model_config = (
            yaml.safe_load(stream)
            or {}
        )

    crop_config = (
        model_config
        .get(
            "grounding_dino",
            {},
        )
        .get(
            "crop",
            {},
        )
    )

    crop_top = int(
        crop_config.get(
            "top_px",
            80,
        )
    )
    crop_bottom = int(
        crop_config.get(
            "bottom_px",
            0,
        )
    )
    crop_left = int(
        crop_config.get(
            "left_px",
            0,
        )
    )
    crop_right = int(
        crop_config.get(
            "right_px",
            0,
        )
    )

    expected_dino_shape = (
        height
        - crop_top
        - crop_bottom,
        width
        - crop_left
        - crop_right,
    )

    if dino_input.shape[:2] != expected_dino_shape:
        raise AssertionError(
            "GroundingDINO runtime input does not match the "
            "configured crop: "
            f"expected={expected_dino_shape}, "
            f"received={dino_input.shape[:2]}"
        )

    overlay_path = (
        perception_result
        .overlay_image_path
    )

    if overlay_path is None:
        raise AssertionError(
            "ScenePerceiver did not return an overlay path."
        )

    overlay_path = (
        Path(overlay_path)
        .expanduser()
        .resolve()
    )

    expected_overlay_path = (
        artifacts_dir
        / "raw_scene_overlay.png"
    )

    if overlay_path != expected_overlay_path:
        raise AssertionError(
            "Unexpected raw-scene overlay path: "
            f"expected={expected_overlay_path}, received={overlay_path}"
        )

    overlay = cv2.imread(
        str(overlay_path),
        cv2.IMREAD_COLOR,
    )

    if overlay is None:
        raise AssertionError(
            f"Raw-scene overlay is not readable: {overlay_path}"
        )

    if overlay.shape[:2] != (
        height,
        width,
    ):
        raise AssertionError(
            "Raw-scene overlay shape differs from source RGB: "
            f"overlay={overlay.shape[:2]}, rgb={(height, width)}"
        )

    raw_scene_json_path = (
        perception_result
        .raw_scene_json_path
    )

    if raw_scene_json_path is None:
        raise AssertionError(
            "ScenePerceiver did not return a raw-scene JSON path."
        )

    raw_scene_json_path = (
        Path(raw_scene_json_path)
        .expanduser()
        .resolve()
    )

    expected_raw_scene_path = (
        artifacts_dir
        / "raw_scene_state.json"
    )

    if raw_scene_json_path != expected_raw_scene_path:
        raise AssertionError(
            "Unexpected raw-scene JSON path: "
            f"expected={expected_raw_scene_path}, "
            f"received={raw_scene_json_path}"
        )

    raw_scene_json = _load_json(
        raw_scene_json_path
    )

    serialized_objects = (
        raw_scene_json.get(
            "objects"
        )
    )

    if not isinstance(
        serialized_objects,
        list,
    ):
        raise AssertionError(
            "raw_scene_state.json does not contain an objects list."
        )

    expected_serialized_objects = [
        _expected_serialized_object(
            obj
        )
        for obj
        in raw_scene.objects
    ]

    if serialized_objects != expected_serialized_objects:
        raise AssertionError(
            "raw_scene_state.json does not match the returned RawSceneState."
        )

    camera_pose_noise_path = (
        artifacts_dir
        / "camera_pose_noise.json"
    )

    camera_pose_noise = _load_json(
        camera_pose_noise_path
    )

    distribution = (
        camera_pose_noise.get(
            "distribution"
        )
    )

    if not isinstance(
        distribution,
        dict,
    ):
        raise AssertionError(
            "camera_pose_noise.json is missing distribution metadata."
        )

    translation_std = float(
        distribution.get(
            "translation_std_mm_per_axis",
            -1.0,
        )
    )

    rotation_std = float(
        distribution.get(
            "rotation_std_deg_per_axis",
            -1.0,
        )
    )

    if (
        translation_std < 0.0
        or rotation_std < 0.0
    ):
        raise AssertionError(
            "Invalid camera-pose noise standard deviation metadata."
        )

    sampled_translation = _assert_finite_vector(
        camera_pose_noise.get(
            "sampled_translation_mm",
            [],
        ),
        length=3,
        name="sampled_translation_mm",
    )

    sampled_rotation = _assert_finite_vector(
        camera_pose_noise.get(
            "sampled_rotation_deg",
            [],
        ),
        length=3,
        name="sampled_rotation_deg",
    )

    if (
        translation_std == 0.0
        and any(
            value != 0.0
            for value
            in sampled_translation
        )
    ):
        raise AssertionError(
            "Zero translation-noise configuration produced a non-zero sample."
        )

    if (
        rotation_std == 0.0
        and any(
            value != 0.0
            for value
            in sampled_rotation
        )
    ):
        raise AssertionError(
            "Zero rotation-noise configuration produced a non-zero sample."
        )

    # ---------------------------------------------------------
    # Report.
    # ---------------------------------------------------------

    print(
        "Scene perception completed"
    )
    print(
        "Scene directory: "
        f"{scene_dir}"
    )
    print(
        "Detected objects: "
        f"{len(raw_scene.objects)}"
    )
    print(
        "Storage bins: "
        f"{len(storage_bins)}"
    )
    print(
        "Manipulable objects: "
        f"{len(manipulable_objects)}"
    )
    print(
        "Observed colors: "
        f"{sorted(observed_colors)}"
    )

    print(
        "\nRaw scene objects:"
    )

    for obj in raw_scene.objects:
        print(
            f"  {obj.object_id}: "
            f"label={obj.label!r}, "
            f"category={obj.category!r}, "
            f"attributes={obj.attributes}, "
            f"pixel={obj.pixel_coordinates}, "
            f"camera={obj.position_camera}, "
            f"base={obj.position_base}, "
            f"confidence={obj.confidence}"
        )

    print(
        "\nOverlay: "
        f"{overlay_path}"
    )
    print(
        "Raw scene JSON: "
        f"{raw_scene_json_path}"
    )
    print(
        "Camera-pose noise: "
        f"{camera_pose_noise_path}"
    )

    print(
        "\nTEST PASSED"
    )

    return 0
