from __future__ import annotations

import json
from types import SimpleNamespace

import pytest

from ai_controller.models.seedo_controller.runtime_structured_scene_builder import (
    RuntimeStructuredSceneBuilder,
)
from ai_controller.models.seedo_controller.task_types import (
    all_place_categories,
)


def _scene_object(
    object_id,
    category,
    pixel_coordinates=None,
):
    return SimpleNamespace(
        object_id=object_id,
        category=category,
        pixel_coordinates=pixel_coordinates,
    )


def _scene_state(objects):
    return SimpleNamespace(
        objects=objects,
    )


@pytest.mark.parametrize(
    "directions",
    [0, 1, 3, 5, 6, 7, 9],
)
def test_constructor_rejects_invalid_direction_count(
    directions,
):
    with pytest.raises(
        ValueError,
        match="Unsupported number of directions",
    ):
        RuntimeStructuredSceneBuilder(
            directions=directions,
        )


@pytest.mark.parametrize(
    "directions",
    [4, 8],
)
def test_constructor_accepts_supported_direction_counts(
    directions,
):
    builder = RuntimeStructuredSceneBuilder(
        directions=directions,
    )

    assert builder.directions == directions
    assert builder.place_categories == all_place_categories()


@pytest.mark.parametrize(
    "objects",
    [
        (),
        [],
    ],
)
def test_run_rejects_empty_scene_state(
    objects,
):
    builder = RuntimeStructuredSceneBuilder()

    scene_state = _scene_state(
        objects=objects,
    )

    with pytest.raises(
        ValueError,
        match="SceneState contains no objects",
    ):
        builder.run(
            scene_state=scene_state,
        )


@pytest.mark.parametrize(
    "category",
    [
        None,
        "",
        "   ",
    ],
)
def test_run_rejects_missing_category(
    category,
):
    builder = RuntimeStructuredSceneBuilder()

    scene_state = _scene_state(
        objects=(
            _scene_object(
                object_id="obj_1",
                category=category,
                pixel_coordinates=(10.0, 20.0),
            ),
        ),
    )

    with pytest.raises(
        ValueError,
        match="Missing category",
    ):
        builder.run(
            scene_state=scene_state,
        )


def test_run_ignores_non_place_categories():
    builder = RuntimeStructuredSceneBuilder()

    scene_state = _scene_state(
        objects=(
            _scene_object(
                object_id="cube_1",
                category="cube",
                pixel_coordinates=(10.0, 20.0),
            ),
            _scene_object(
                object_id="object_1",
                category="object",
                pixel_coordinates=(30.0, 40.0),
            ),
        ),
    )

    with pytest.raises(
        ValueError,
        match="No possible place destinations",
    ):
        builder.run(
            scene_state=scene_state,
        )


def test_run_does_not_require_pixel_coordinates_for_ignored_objects():
    builder = RuntimeStructuredSceneBuilder()

    place_category = sorted(
        all_place_categories()
    )[0]

    scene_state = _scene_state(
        objects=(
            _scene_object(
                object_id="cube_1",
                category="cube",
            ),
            _scene_object(
                object_id="place_1",
                category=place_category,
                pixel_coordinates=(100.0, 200.0),
            ),
        ),
    )

    scene = builder.run(
        scene_state=scene_state,
    )

    assert len(scene.objects) == 1

    assert scene.objects[0].object_id == "place_1"
    assert scene.objects[0].category == place_category
    assert scene.objects[0].center == (100.0, 200.0)


def test_run_normalizes_place_category():
    builder = RuntimeStructuredSceneBuilder()

    place_category = sorted(
        all_place_categories()
    )[0]

    scene_state = _scene_state(
        objects=(
            _scene_object(
                object_id="place_1",
                category=f"  {place_category.upper()}  ",
                pixel_coordinates=(100, 200),
            ),
        ),
    )

    scene = builder.run(
        scene_state=scene_state,
    )

    assert len(scene.objects) == 1
    assert scene.objects[0].category == place_category


def test_run_preserves_runtime_object_id():
    builder = RuntimeStructuredSceneBuilder()

    place_category = sorted(
        all_place_categories()
    )[0]

    scene_state = _scene_state(
        objects=(
            _scene_object(
                object_id="runtime_bin_42",
                category=place_category,
                pixel_coordinates=(10, 20),
            ),
        ),
    )

    scene = builder.run(
        scene_state=scene_state,
    )

    assert scene.objects[0].object_id == "runtime_bin_42"


def test_run_converts_pixel_coordinates_to_float():
    builder = RuntimeStructuredSceneBuilder()

    place_category = sorted(
        all_place_categories()
    )[0]

    scene_state = _scene_state(
        objects=(
            _scene_object(
                object_id="place_1",
                category=place_category,
                pixel_coordinates=(10, 20),
            ),
        ),
    )

    scene = builder.run(
        scene_state=scene_state,
    )

    assert scene.objects[0].center == (
        10.0,
        20.0,
    )

    assert isinstance(
        scene.objects[0].center[0],
        float,
    )

    assert isinstance(
        scene.objects[0].center[1],
        float,
    )


def test_run_builds_only_place_destination_objects():
    builder = RuntimeStructuredSceneBuilder(
        directions=8,
    )

    place_categories = sorted(
        all_place_categories()
    )

    first_category = place_categories[0]
    second_category = place_categories[-1]

    scene_state = _scene_state(
        objects=(
            _scene_object(
                object_id="place_a",
                category=first_category,
                pixel_coordinates=(100.0, 200.0),
            ),
            _scene_object(
                object_id="place_b",
                category=second_category,
                pixel_coordinates=(300.5, 200.0),
            ),
            _scene_object(
                object_id="cube_1",
                category="cube",
                pixel_coordinates=(200.0, 300.0),
            ),
        ),
    )

    scene = builder.run(
        scene_state=scene_state,
    )

    assert scene.directions == 8

    assert len(scene.objects) == 2

    assert scene.objects[0].object_id == "place_a"
    assert scene.objects[0].category == first_category
    assert scene.objects[0].center == (100.0, 200.0)

    assert scene.objects[1].object_id == "place_b"
    assert scene.objects[1].category == second_category
    assert scene.objects[1].center == (300.5, 200.0)


def test_run_builds_directed_spatial_relations():
    builder = RuntimeStructuredSceneBuilder(
        directions=8,
    )

    place_category = sorted(
        all_place_categories()
    )[0]

    scene_state = _scene_state(
        objects=(
            _scene_object(
                object_id="left",
                category=place_category,
                pixel_coordinates=(100.0, 100.0),
            ),
            _scene_object(
                object_id="right",
                category=place_category,
                pixel_coordinates=(200.0, 100.0),
            ),
        ),
    )

    scene = builder.run(
        scene_state=scene_state,
    )

    actual_relations = {
        (
            relation.subject_object_id,
            relation.reference_object_id,
            relation.relation,
        )
        for relation in scene.relations
    }

    assert actual_relations == {
        ("left", "right", "LEFT"),
        ("right", "left", "RIGHT"),
    }


def test_run_respects_four_direction_mode():
    builder = RuntimeStructuredSceneBuilder(
        directions=4,
    )

    place_category = sorted(
        all_place_categories()
    )[0]

    scene_state = _scene_state(
        objects=(
            _scene_object(
                object_id="a",
                category=place_category,
                pixel_coordinates=(0.0, 0.0),
            ),
            _scene_object(
                object_id="b",
                category=place_category,
                pixel_coordinates=(20.0, 10.0),
            ),
        ),
    )

    scene = builder.run(
        scene_state=scene_state,
    )

    actual_relations = {
        (
            relation.subject_object_id,
            relation.reference_object_id,
            relation.relation,
        )
        for relation in scene.relations
    }

    assert scene.directions == 4

    assert actual_relations == {
        ("a", "b", "LEFT"),
        ("b", "a", "RIGHT"),
    }


def test_run_rejects_identical_place_centers():
    builder = RuntimeStructuredSceneBuilder()

    place_category = sorted(
        all_place_categories()
    )[0]

    scene_state = _scene_state(
        objects=(
            _scene_object(
                object_id="a",
                category=place_category,
                pixel_coordinates=(100.0, 200.0),
            ),
            _scene_object(
                object_id="b",
                category=place_category,
                pixel_coordinates=(100.0, 200.0),
            ),
        ),
    )

    with pytest.raises(
        ValueError,
        match="identical centers",
    ):
        builder.run(
            scene_state=scene_state,
        )


def test_run_writes_structured_scene_artifact(
    tmp_path,
):
    builder = RuntimeStructuredSceneBuilder(
        directions=8,
    )

    place_category = sorted(
        all_place_categories()
    )[0]

    scene_state = _scene_state(
        objects=(
            _scene_object(
                object_id="runtime_1",
                category=place_category,
                pixel_coordinates=(10.0, 20.0),
            ),
            _scene_object(
                object_id="runtime_2",
                category=place_category,
                pixel_coordinates=(30.0, 20.0),
            ),
        ),
    )

    artifacts_dir = (
        tmp_path
        / "nested"
        / "runtime_structured_scene"
    )

    scene = builder.run(
        scene_state=scene_state,
        artifacts_dir=artifacts_dir,
    )

    artifact_path = (
        artifacts_dir
        / "runtime_structured_scene.json"
    )

    assert artifact_path.is_file()

    with artifact_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        data = json.load(stream)

    assert data["directions"] == 8

    assert data["objects"] == [
        {
            "object_id": "runtime_1",
            "category": place_category,
            "center": [10.0, 20.0],
        },
        {
            "object_id": "runtime_2",
            "category": place_category,
            "center": [30.0, 20.0],
        },
    ]

    assert data["relations"] == [
        {
            "subject_object_id": "runtime_1",
            "reference_object_id": "runtime_2",
            "relation": "LEFT",
        },
        {
            "subject_object_id": "runtime_2",
            "reference_object_id": "runtime_1",
            "relation": "RIGHT",
        },
    ]

    assert len(scene.objects) == 2
    assert len(scene.relations) == 2