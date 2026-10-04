from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

from results import (
    SceneObject,
    SceneState,
)

from ai_controller.models.seedo_controller.runtime_structured_scene_builder import (
    RuntimeStructuredSceneBuilder,
)


DEFAULT_SCENE_STATE = Path(
    "/seedo_tests/scene_interpreter/scene_state.json"
)

EXPECTED_OBJECTS = {
    "storage_bin_0": ("bin", (194.0, 316.0)),
    "storage_bin_1": ("bin", (304.0, 314.0)),
    "storage_bin_2": ("bin", (412.0, 311.0)),
    "storage_bin_3": ("bin", (521.0, 309.0)),
}

EXPECTED_RELATIONS = {
    ("storage_bin_0", "storage_bin_1", "LEFT"),
    ("storage_bin_0", "storage_bin_2", "LEFT"),
    ("storage_bin_0", "storage_bin_3", "LEFT"),
    ("storage_bin_1", "storage_bin_0", "RIGHT"),
    ("storage_bin_1", "storage_bin_2", "LEFT"),
    ("storage_bin_1", "storage_bin_3", "LEFT"),
    ("storage_bin_2", "storage_bin_0", "RIGHT"),
    ("storage_bin_2", "storage_bin_1", "RIGHT"),
    ("storage_bin_2", "storage_bin_3", "LEFT"),
    ("storage_bin_3", "storage_bin_0", "RIGHT"),
    ("storage_bin_3", "storage_bin_1", "RIGHT"),
    ("storage_bin_3", "storage_bin_2", "RIGHT"),
}


def _load_scene_state(
    path: Path,
) -> SceneState:
    normalized_path = (
        Path(path)
        .expanduser()
        .resolve()
    )

    if not normalized_path.is_file():
        raise FileNotFoundError(
            "SceneState artifact does not exist: "
            f"{normalized_path}"
        )

    if normalized_path.stat().st_size == 0:
        raise ValueError(
            "SceneState artifact is empty: "
            f"{normalized_path}"
        )

    with normalized_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        data = json.load(
            stream
        )

    objects_data = data.get(
        "objects"
    )

    if (
        not isinstance(objects_data, list)
        or not objects_data
    ):
        raise ValueError(
            "scene_state.json must contain a non-empty "
            "'objects' list."
        )

    objects: list[SceneObject] = []

    for index, item in enumerate(
        objects_data
    ):
        if not isinstance(item, dict):
            raise ValueError(
                "Invalid scene object at index "
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

        pixel_coordinates = item.get(
            "pixel_coordinates"
        )

        position_camera = item.get(
            "position_camera"
        )

        position_base = item.get(
            "position_base"
        )

        if not object_id:
            raise ValueError(
                f"Scene object at index {index} has no object_id."
            )

        if not label:
            raise ValueError(
                f"Scene object {object_id!r} has no label."
            )

        if (
            not isinstance(pixel_coordinates, list)
            or len(pixel_coordinates) != 2
        ):
            raise ValueError(
                f"{object_id} has invalid pixel_coordinates: "
                f"{pixel_coordinates!r}"
            )

        if (
            not isinstance(position_camera, list)
            or len(position_camera) != 3
        ):
            raise ValueError(
                f"{object_id} has invalid position_camera: "
                f"{position_camera!r}"
            )

        if (
            not isinstance(position_base, list)
            or len(position_base) != 3
        ):
            raise ValueError(
                f"{object_id} has invalid position_base: "
                f"{position_base!r}"
            )

        if not isinstance(
            attributes,
            dict,
        ):
            raise ValueError(
                f"{object_id} attributes must be a dictionary."
            )

        objects.append(
            SceneObject(
                object_id=object_id,
                label=label,
                pixel_coordinates=(
                    int(pixel_coordinates[0]),
                    int(pixel_coordinates[1]),
                ),
                position_camera=tuple(
                    float(value)
                    for value
                    in position_camera
                ),
                position_base=tuple(
                    float(value)
                    for value
                    in position_base
                ),
                category=category,
                attributes=dict(
                    attributes
                ),
            )
        )

    return SceneState(
        objects=tuple(objects)
    )


def run_runtime_structured_scene_builder_test(
    args: argparse.Namespace,
) -> int:
    """Run RuntimeStructuredSceneBuilder on the real SceneInterpreter artifact."""

    scene_state_path = Path(
        getattr(
            args,
            "scene_state",
            None,
        )
        or DEFAULT_SCENE_STATE
    )

    scene_state = _load_scene_state(
        scene_state_path
    )

    if args.artifacts_dir is None:
        artifacts_dir = Path(
            "/seedo_tests/runtime_structured_scene_builder"
        )
    else:
        artifacts_dir = (
            Path(args.artifacts_dir)
            .expanduser()
            .resolve()
        )

    builder = RuntimeStructuredSceneBuilder(
        directions=8,
    )

    structured_scene = builder.run(
        scene_state=scene_state,
        artifacts_dir=artifacts_dir,
    )

    # ---------------------------------------------------------
    # Validate top-level result.
    # ---------------------------------------------------------

    if structured_scene.directions != 8:
        raise AssertionError(
            "Unexpected direction mode: "
            f"{structured_scene.directions}"
        )

    if len(structured_scene.objects) != 4:
        raise AssertionError(
            "Expected exactly four runtime place objects, "
            f"received {len(structured_scene.objects)}."
        )

    if len(structured_scene.relations) != 12:
        raise AssertionError(
            "Expected 4 * 3 = 12 directed spatial relations, "
            f"received {len(structured_scene.relations)}."
        )

    # ---------------------------------------------------------
    # Validate destination filtering and centers.
    # ---------------------------------------------------------

    returned_objects = {
        obj.object_id: obj
        for obj
        in structured_scene.objects
    }

    if set(returned_objects) != set(
        EXPECTED_OBJECTS
    ):
        raise AssertionError(
            "Unexpected runtime structured-scene object IDs: "
            f"expected={sorted(EXPECTED_OBJECTS)}, "
            f"received={sorted(returned_objects)}"
        )

    for (
        object_id,
        (
            expected_category,
            expected_center,
        ),
    ) in EXPECTED_OBJECTS.items():
        obj = returned_objects[
            object_id
        ]

        if obj.category != expected_category:
            raise AssertionError(
                "Unexpected category for runtime object "
                f"{object_id}: "
                f"expected={expected_category!r}, "
                f"received={obj.category!r}"
            )

        if obj.center != expected_center:
            raise AssertionError(
                "Unexpected runtime SAM centroid for object "
                f"{object_id}: "
                f"expected={expected_center}, "
                f"received={obj.center}"
            )

    excluded_ids = {
        "green_block_0",
        "yellow_block_0",
        "blue_block_0",
        "red_block_0",
    }

    if set(returned_objects) & excluded_ids:
        raise AssertionError(
            "Non-place runtime objects were included in "
            "the structured scene."
        )

    # ---------------------------------------------------------
    # Validate qualitative spatial relations.
    # ---------------------------------------------------------

    returned_relations = {
        (
            relation.subject_object_id,
            relation.reference_object_id,
            relation.relation,
        )
        for relation
        in structured_scene.relations
    }

    if len(returned_relations) != len(
        structured_scene.relations
    ):
        raise AssertionError(
            "Runtime structured scene contains duplicate relations."
        )

    if returned_relations != EXPECTED_RELATIONS:
        raise AssertionError(
            "Unexpected runtime qualitative relation graph. "
            f"missing={sorted(EXPECTED_RELATIONS - returned_relations)}, "
            f"unexpected={sorted(returned_relations - EXPECTED_RELATIONS)}"
        )

    for relation in (
        structured_scene.relations
    ):
        if (
            relation.subject_object_id
            == relation.reference_object_id
        ):
            raise AssertionError(
                "Runtime structured scene contains a self-relation: "
                f"{relation}"
            )

    # ---------------------------------------------------------
    # Validate persistent artifact.
    # ---------------------------------------------------------

    artifact_path = (
        artifacts_dir
        / "runtime_structured_scene.json"
    )

    if not artifact_path.is_file():
        raise AssertionError(
            "Runtime structured-scene artifact was not created: "
            f"{artifact_path}"
        )

    if artifact_path.stat().st_size == 0:
        raise AssertionError(
            "Runtime structured-scene artifact is empty: "
            f"{artifact_path}"
        )

    with artifact_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        artifact = json.load(
            stream
        )

    expected_artifact = json.loads(
        json.dumps(
            asdict(
                structured_scene
            ),
            ensure_ascii=False,
        )
    )

    if artifact != expected_artifact:
        raise AssertionError(
            "runtime_structured_scene.json does not match "
            "the returned StructuredScene."
        )

    # ---------------------------------------------------------
    # Report.
    # ---------------------------------------------------------

    print(
        "Runtime structured scene completed"
    )
    print(
        "SceneState artifact: "
        f"{scene_state_path.expanduser().resolve()}"
    )
    print(
        "Structured-scene artifact: "
        f"{artifact_path}"
    )
    print(
        "Direction mode: "
        f"{structured_scene.directions}"
    )
    print(
        "Place objects: "
        f"{len(structured_scene.objects)}"
    )
    print(
        "Directed relations: "
        f"{len(structured_scene.relations)}"
    )

    print(
        "\nStructured objects:"
    )

    for obj in (
        structured_scene.objects
    ):
        print(
            f"  {obj.object_id}: "
            f"category={obj.category}, "
            f"center={obj.center}"
        )

    print(
        "\nSpatial relations:"
    )

    for relation in (
        structured_scene.relations
    ):
        print(
            "  "
            f"{relation.subject_object_id} "
            f"{relation.relation} "
            f"{relation.reference_object_id}"
        )

    print(
        "\nTEST PASSED"
    )

    return 0
