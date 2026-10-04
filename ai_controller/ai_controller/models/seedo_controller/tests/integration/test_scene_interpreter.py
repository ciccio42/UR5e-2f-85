from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import yaml

from results import (
    RawSceneObject,
    RawSceneState,
    ScenePerceptionResult,
)

from ai_controller.models.seedo_controller.scene_interpreter import (
    SceneInterpreter,
)


DEFAULT_RAW_SCENE_STATE = Path(
    "/seedo_tests/scene_perceiver/raw_scene_state.json"
)

DEFAULT_RAW_SCENE_OVERLAY = Path(
    "/seedo_tests/scene_perceiver/raw_scene_overlay.png"
)

EXPECTED_OBJECT_IDS = {
    "storage_bin_0",
    "storage_bin_1",
    "storage_bin_2",
    "storage_bin_3",
    "green_block_0",
    "yellow_block_0",
    "blue_block_0",
    "red_block_0",
}

EXPECTED_CATEGORIES = {
    "storage_bin_0": "bin",
    "storage_bin_1": "bin",
    "storage_bin_2": "bin",
    "storage_bin_3": "bin",
    "green_block_0": "block",
    "yellow_block_0": "block",
    "blue_block_0": "block",
    "red_block_0": "block",
}

EXPECTED_COLORS = {
    "green_block_0": "green",
    "yellow_block_0": "yellow",
    "blue_block_0": "blue",
    "red_block_0": "red",
}


def _load_json(path: Path) -> dict:
    normalized_path = (
        Path(path)
        .expanduser()
        .resolve()
    )

    if not normalized_path.is_file():
        raise FileNotFoundError(
            f"Required artifact does not exist: {normalized_path}"
        )

    if normalized_path.stat().st_size == 0:
        raise ValueError(
            f"Required artifact is empty: {normalized_path}"
        )

    with normalized_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        value = json.load(stream)

    if not isinstance(value, dict):
        raise ValueError(
            f"Expected a JSON object in {normalized_path}."
        )

    return value


def _finite_tuple(
    values,
    *,
    length: int,
    field_name: str,
) -> tuple[float, ...]:
    if (
        not isinstance(values, (list, tuple))
        or len(values) != length
    ):
        raise ValueError(
            f"{field_name} must contain exactly {length} values: "
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
        raise ValueError(
            f"{field_name} contains non-finite values: "
            f"{normalized}"
        )

    return normalized


def _build_perception_result_from_artifacts(
    *,
    raw_scene_state_path: Path,
    raw_scene_overlay_path: Path,
) -> ScenePerceptionResult:
    """Reconstruct ScenePerceptionResult from the validated ScenePerceiver handoff."""

    raw_scene_data = _load_json(
        raw_scene_state_path
    )

    raw_objects_data = raw_scene_data.get(
        "objects"
    )

    if (
        not isinstance(raw_objects_data, list)
        or not raw_objects_data
    ):
        raise ValueError(
            "raw_scene_state.json must contain a non-empty "
            "'objects' list."
        )

    raw_objects: list[RawSceneObject] = []

    for index, item in enumerate(
        raw_objects_data
    ):
        if not isinstance(item, dict):
            raise ValueError(
                "Invalid raw scene object at index "
                f"{index}: {item!r}"
            )

        object_id = str(
            item.get(
                "object_id",
                "",
            )
        ).strip()

        label = str(
            item.get(
                "label",
                "",
            )
        ).strip()

        if not object_id:
            raise ValueError(
                f"Raw scene object at index {index} has no object_id."
            )

        if not label:
            raise ValueError(
                f"Raw scene object {object_id!r} has no label."
            )

        pixel_coordinates_raw = item.get(
            "pixel_coordinates"
        )

        if (
            not isinstance(
                pixel_coordinates_raw,
                (list, tuple),
            )
            or len(pixel_coordinates_raw) != 2
        ):
            raise ValueError(
                f"{object_id} has invalid pixel_coordinates: "
                f"{pixel_coordinates_raw!r}"
            )

        pixel_coordinates = (
            int(pixel_coordinates_raw[0]),
            int(pixel_coordinates_raw[1]),
        )

        position_camera = _finite_tuple(
            item.get(
                "position_camera"
            ),
            length=3,
            field_name=(
                f"{object_id}.position_camera"
            ),
        )

        position_base = _finite_tuple(
            item.get(
                "position_base"
            ),
            length=3,
            field_name=(
                f"{object_id}.position_base"
            ),
        )

        confidence_raw = item.get(
            "confidence"
        )

        confidence = (
            None
            if confidence_raw is None
            else float(confidence_raw)
        )

        if (
            confidence is not None
            and not math.isfinite(confidence)
        ):
            raise ValueError(
                f"{object_id} has non-finite confidence."
            )

        category_raw = item.get(
            "category"
        )

        category = (
            None
            if category_raw is None
            else str(category_raw)
        )

        attributes = item.get(
            "attributes",
            {},
        )

        if not isinstance(
            attributes,
            dict,
        ):
            raise ValueError(
                f"{object_id} attributes must be a dictionary."
            )

        raw_objects.append(
            RawSceneObject(
                object_id=object_id,
                label=label,
                pixel_coordinates=pixel_coordinates,
                position_camera=position_camera,
                position_base=position_base,
                mask=None,
                confidence=confidence,
                category=category,
                attributes=dict(attributes),
            )
        )

    overlay_path = (
        Path(raw_scene_overlay_path)
        .expanduser()
        .resolve()
    )

    if not overlay_path.is_file():
        raise FileNotFoundError(
            "Raw scene overlay does not exist: "
            f"{overlay_path}"
        )

    if overlay_path.stat().st_size == 0:
        raise ValueError(
            "Raw scene overlay is empty: "
            f"{overlay_path}"
        )

    overlay = cv2.imread(
        str(overlay_path),
        cv2.IMREAD_COLOR,
    )

    if overlay is None:
        raise ValueError(
            "Raw scene overlay is not readable: "
            f"{overlay_path}"
        )

    return ScenePerceptionResult(
        raw_scene=RawSceneState(
            objects=tuple(raw_objects)
        ),
        overlay_image_path=overlay_path,
        raw_scene_json_path=(
            Path(raw_scene_state_path)
            .expanduser()
            .resolve()
        ),
    )


def _expected_scene_state_json(
    scene_state,
) -> dict:
    return {
        "objects": [
            {
                "object_id": obj.object_id,
                "label": obj.label,
                "category": obj.category,
                "attributes": dict(
                    obj.attributes
                ),
                "pixel_coordinates": list(
                    obj.pixel_coordinates
                ),
                "position_camera": list(
                    obj.position_camera
                ),
                "position_base": list(
                    obj.position_base
                ),
            }
            for obj in scene_state.objects
        ]
    }


def run_scene_interpreter_test(
    args: argparse.Namespace,
) -> int:
    """Run generalized SceneInterpreter from persisted ScenePerceiver artifacts."""

    if args.artifacts_dir is None:
        raise ValueError(
            "--artifacts-dir is required for the "
            "scene_interpreter integration test."
        )

    artifacts_dir = (
        Path(args.artifacts_dir)
        .expanduser()
        .resolve()
    )

    artifacts_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    raw_scene_state_path = (
        DEFAULT_RAW_SCENE_STATE
    )

    raw_scene_overlay_path = (
        DEFAULT_RAW_SCENE_OVERLAY
    )

    perception_result = (
        _build_perception_result_from_artifacts(
            raw_scene_state_path=(
                raw_scene_state_path
            ),
            raw_scene_overlay_path=(
                raw_scene_overlay_path
            ),
        )
    )

    raw_scene = (
        perception_result.raw_scene
    )

    # ---------------------------------------------------------
    # Validate the ScenePerceiver handoff before interpretation.
    # ---------------------------------------------------------

    raw_ids = {
        obj.object_id
        for obj
        in raw_scene.objects
    }

    if raw_ids != EXPECTED_OBJECT_IDS:
        raise AssertionError(
            "Unexpected raw runtime object IDs: "
            f"expected={sorted(EXPECTED_OBJECT_IDS)}, "
            f"received={sorted(raw_ids)}"
        )

    if len(raw_scene.objects) != 8:
        raise AssertionError(
            "Expected exactly 8 raw runtime objects, "
            f"received {len(raw_scene.objects)}."
        )

    for raw_obj in raw_scene.objects:
        expected_category = (
            EXPECTED_CATEGORIES[
                raw_obj.object_id
            ]
        )

        if (
            str(
                raw_obj.category
            ).strip().lower()
            != expected_category
        ):
            raise AssertionError(
                "Unexpected generalized category for "
                f"{raw_obj.object_id}: "
                f"expected={expected_category!r}, "
                f"received={raw_obj.category!r}"
            )

        if (
            raw_obj.object_id
            in EXPECTED_COLORS
        ):
            expected_color = (
                EXPECTED_COLORS[
                    raw_obj.object_id
                ]
            )

            received_color = str(
                raw_obj.attributes.get(
                    "color",
                    "",
                )
            ).strip().lower()

            if (
                received_color
                != expected_color
            ):
                raise AssertionError(
                    "Unexpected color attribute for "
                    f"{raw_obj.object_id}: "
                    f"expected={expected_color!r}, "
                    f"received={received_color!r}"
                )

        else:
            if raw_obj.attributes:
                raise AssertionError(
                    f"{raw_obj.object_id} bin attributes "
                    "should be empty."
                )

    # ---------------------------------------------------------
    # Mirror the configured model name when available.
    # Generalized mode itself performs no VLM call.
    # ---------------------------------------------------------

    model = "gpt-4o-2024-08-06"

    if args.model_config:
        model_config_path = (
            Path(args.model_config)
            .expanduser()
            .resolve()
        )

        if not model_config_path.is_file():
            raise FileNotFoundError(
                "SeeDo controller configuration does not exist: "
                f"{model_config_path}"
            )

        with model_config_path.open(
            "r",
            encoding="utf-8",
        ) as stream:
            config = yaml.safe_load(
                stream
            )

        if not isinstance(
            config,
            dict,
        ):
            raise ValueError(
                "SeeDo controller configuration must contain a YAML mapping."
            )

        interpreter_config = (
            config.get(
                "scene_interpreter",
                {},
            )
        )

        if isinstance(
            interpreter_config,
            dict,
        ):
            model = str(
                interpreter_config.get(
                    "model",
                    model,
                )
            )

    interpreter = SceneInterpreter(
        model=model,
        perception_mode="generalized",
    )

    scene_state = interpreter.run(
        perception_result=perception_result,
        artifacts_dir=artifacts_dir,
    )

    # ---------------------------------------------------------
    # Structural and semantic checks.
    # ---------------------------------------------------------

    if (
        len(scene_state.objects)
        != len(raw_scene.objects)
    ):
        raise AssertionError(
            "SceneInterpreter changed the number of objects. "
            f"Raw={len(raw_scene.objects)}, "
            f"semantic={len(scene_state.objects)}."
        )

    scene_ids = {
        obj.object_id
        for obj
        in scene_state.objects
    }

    if scene_ids != EXPECTED_OBJECT_IDS:
        raise AssertionError(
            "Generalized SceneInterpreter changed runtime object identity: "
            f"expected={sorted(EXPECTED_OBJECT_IDS)}, "
            f"received={sorted(scene_ids)}"
        )

    if len(scene_ids) != len(
        scene_state.objects
    ):
        raise AssertionError(
            "SceneInterpreter produced duplicate object IDs."
        )

    for raw_obj, scene_obj in zip(
        raw_scene.objects,
        scene_state.objects,
        strict=True,
    ):
        if (
            scene_obj.object_id
            != raw_obj.object_id
        ):
            raise AssertionError(
                "Generalized SceneInterpreter must preserve "
                "the neutral runtime object ID: "
                f"raw={raw_obj.object_id!r}, "
                f"scene={scene_obj.object_id!r}"
            )

        if (
            scene_obj.label
            != raw_obj.label
        ):
            raise AssertionError(
                f"{raw_obj.object_id}: detector label changed "
                f"from {raw_obj.label!r} "
                f"to {scene_obj.label!r}."
            )

        if (
            scene_obj.category
            != raw_obj.category
        ):
            raise AssertionError(
                f"{raw_obj.object_id}: category changed during "
                "generalized interpretation."
            )

        if (
            scene_obj.attributes
            != raw_obj.attributes
        ):
            raise AssertionError(
                f"{raw_obj.object_id}: attributes changed during "
                "generalized interpretation."
            )

        if (
            scene_obj.pixel_coordinates
            != raw_obj.pixel_coordinates
        ):
            raise AssertionError(
                f"{raw_obj.object_id}: pixel coordinates changed "
                "during interpretation."
            )

        if (
            scene_obj.position_camera
            != raw_obj.position_camera
        ):
            raise AssertionError(
                f"{raw_obj.object_id}: camera-space position changed "
                "during interpretation."
            )

        if (
            scene_obj.position_base
            != raw_obj.position_base
        ):
            raise AssertionError(
                f"{raw_obj.object_id}: base-space position changed "
                "during interpretation."
            )

    # ---------------------------------------------------------
    # scene_interpretation.json
    # ---------------------------------------------------------

    interpretation_path = (
        artifacts_dir
        / "scene_interpretation.json"
    )

    interpretation_json = _load_json(
        interpretation_path
    )

    interpretation_objects = (
        interpretation_json.get(
            "objects"
        )
    )

    if not isinstance(
        interpretation_objects,
        list,
    ):
        raise AssertionError(
            "scene_interpretation.json does not contain an objects list."
        )

    expected_interpretation = {
        "objects": [
            {
                "raw_object_id": (
                    raw_obj.object_id
                ),
                "semantic_name": (
                    raw_obj.object_id
                ),
            }
            for raw_obj
            in raw_scene.objects
        ]
    }

    if (
        interpretation_json
        != expected_interpretation
    ):
        raise AssertionError(
            "Generalized scene_interpretation.json does not "
            "preserve neutral runtime IDs exactly."
        )

    # ---------------------------------------------------------
    # scene_state.json
    # ---------------------------------------------------------

    state_path = (
        artifacts_dir
        / "scene_state.json"
    )

    state_json = _load_json(
        state_path
    )

    expected_state_json = (
        _expected_scene_state_json(
            scene_state
        )
    )

    if (
        state_json
        != expected_state_json
    ):
        raise AssertionError(
            "scene_state.json does not match the returned SceneState."
        )

    # ---------------------------------------------------------
    # Report.
    # ---------------------------------------------------------

    print(
        "Scene interpretation completed"
    )
    print(
        "Raw scene artifact: "
        f"{raw_scene_state_path}"
    )
    print(
        "Raw overlay artifact: "
        f"{raw_scene_overlay_path}"
    )
    print(
        "Perception mode: generalized"
    )
    print(
        "Objects: "
        f"{len(scene_state.objects)}"
    )

    print(
        "\nScene objects:"
    )

    for obj in scene_state.objects:
        print(
            f"  {obj.object_id}: "
            f"label={obj.label!r}, "
            f"category={obj.category!r}, "
            f"attributes={obj.attributes}, "
            f"pixel={obj.pixel_coordinates}, "
            f"camera={obj.position_camera}, "
            f"base={obj.position_base}"
        )

    print(
        "\nScene interpretation artifact: "
        f"{interpretation_path}"
    )
    print(
        "Scene state artifact: "
        f"{state_path}"
    )

    print(
        "\nTEST PASSED"
    )

    return 0
