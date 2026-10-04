from __future__ import annotations

import json
from types import SimpleNamespace

import pytest

from ai_controller.models.seedo_controller.demo_structured_scene_builder import (
    DemoStructuredSceneBuilder,
)
from ai_controller.models.seedo_controller.task_types import (
    all_place_categories,
)


def _visual_result(track_id_map):
    return SimpleNamespace(
        track_id_map=track_id_map,
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
        DemoStructuredSceneBuilder(
            directions=directions,
        )


@pytest.mark.parametrize(
    "directions",
    [4, 8],
)
def test_constructor_accepts_supported_direction_counts(
    directions,
):
    builder = DemoStructuredSceneBuilder(
        directions=directions,
    )

    assert builder.directions == directions
    assert builder.place_categories == all_place_categories()


def test_run_rejects_empty_track_id_map():
    builder = DemoStructuredSceneBuilder()

    visual_result = _visual_result({})

    with pytest.raises(
        ValueError,
        match="empty track_id_map",
    ):
        builder.run(
            visual_prompting_result=visual_result,
        )


@pytest.mark.parametrize(
    "track_info",
    [
        {},
        {"category": ""},
        {"category": "   "},
    ],
)
def test_run_rejects_missing_category(
    track_info,
):
    builder = DemoStructuredSceneBuilder()

    visual_result = _visual_result(
        {
            1: track_info,
        }
    )

    with pytest.raises(
        ValueError,
        match="Missing category",
    ):
        builder.run(
            visual_prompting_result=visual_result,
        )


def test_run_ignores_non_place_categories():
    builder = DemoStructuredSceneBuilder()

    visual_result = _visual_result(
        {
            1: {
                "category": "cube",
                "initial_center": (10.0, 20.0),
            },
            2: {
                "category": "object",
                "initial_center": (30.0, 40.0),
            },
        }
    )

    with pytest.raises(
        ValueError,
        match="No possible place destinations",
    ):
        builder.run(
            visual_prompting_result=visual_result,
        )


def test_run_does_not_require_center_for_ignored_objects():
    builder = DemoStructuredSceneBuilder()

    place_category = sorted(
        all_place_categories()
    )[0]

    visual_result = _visual_result(
        {
            1: {
                "category": "cube",
            },
            2: {
                "category": place_category,
                "initial_center": (100.0, 200.0),
            },
        }
    )

    scene = builder.run(
        visual_prompting_result=visual_result,
    )

    assert len(scene.objects) == 1
    assert scene.objects[0].object_id == "2"
    assert scene.objects[0].category == place_category
    assert scene.objects[0].center == (100.0, 200.0)


def test_run_rejects_missing_center_for_place_destination():
    builder = DemoStructuredSceneBuilder()

    place_category = sorted(
        all_place_categories()
    )[0]

    visual_result = _visual_result(
        {
            1: {
                "category": place_category,
            },
        }
    )

    with pytest.raises(
        ValueError,
        match="Missing SAM centroid",
    ):
        builder.run(
            visual_prompting_result=visual_result,
        )


@pytest.mark.parametrize(
    "center",
    [
        10,
        "10,20",
        [],
        [10],
        [10, 20, 30],
        (10,),
        (10, 20, 30),
    ],
)
def test_run_rejects_invalid_center_shape(
    center,
):
    builder = DemoStructuredSceneBuilder()

    place_category = sorted(
        all_place_categories()
    )[0]

    visual_result = _visual_result(
        {
            1: {
                "category": place_category,
                "initial_center": center,
            },
        }
    )

    with pytest.raises(
        ValueError,
        match="Invalid SAM centroid",
    ):
        builder.run(
            visual_prompting_result=visual_result,
        )


def test_run_builds_only_place_destination_objects():
    builder = DemoStructuredSceneBuilder(
        directions=8,
    )

    place_categories = sorted(
        all_place_categories()
    )

    first_category = place_categories[0]
    second_category = place_categories[-1]

    visual_result = _visual_result(
        {
            10: {
                "category": f"  {first_category.upper()}  ",
                "initial_center": [100, 200],
            },
            20: {
                "category": second_category,
                "initial_center": (300.5, 200.0),
            },
            30: {
                "category": "cube",
                "initial_center": (200.0, 300.0),
            },
        }
    )

    scene = builder.run(
        visual_prompting_result=visual_result,
    )

    assert scene.directions == 8

    assert len(scene.objects) == 2

    assert scene.objects[0].object_id == "10"
    assert scene.objects[0].category == first_category
    assert scene.objects[0].center == (100.0, 200.0)

    assert scene.objects[1].object_id == "20"
    assert scene.objects[1].category == second_category
    assert scene.objects[1].center == (300.5, 200.0)


def test_run_builds_directed_spatial_relations():
    builder = DemoStructuredSceneBuilder(
        directions=8,
    )

    place_category = sorted(
        all_place_categories()
    )[0]

    visual_result = _visual_result(
        {
            "left": {
                "category": place_category,
                "initial_center": (100.0, 100.0),
            },
            "right": {
                "category": place_category,
                "initial_center": (200.0, 100.0),
            },
        }
    )

    scene = builder.run(
        visual_prompting_result=visual_result,
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
    builder = DemoStructuredSceneBuilder(
        directions=4,
    )

    place_category = sorted(
        all_place_categories()
    )[0]

    visual_result = _visual_result(
        {
            "a": {
                "category": place_category,
                "initial_center": (0.0, 0.0),
            },
            "b": {
                "category": place_category,
                "initial_center": (20.0, 10.0),
            },
        }
    )

    scene = builder.run(
        visual_prompting_result=visual_result,
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
    builder = DemoStructuredSceneBuilder()

    place_category = sorted(
        all_place_categories()
    )[0]

    visual_result = _visual_result(
        {
            1: {
                "category": place_category,
                "initial_center": (100.0, 200.0),
            },
            2: {
                "category": place_category,
                "initial_center": (100.0, 200.0),
            },
        }
    )

    with pytest.raises(
        ValueError,
        match="identical centers",
    ):
        builder.run(
            visual_prompting_result=visual_result,
        )


def test_run_writes_structured_scene_artifact(
    tmp_path,
):
    builder = DemoStructuredSceneBuilder(
        directions=8,
    )

    place_category = sorted(
        all_place_categories()
    )[0]

    visual_result = _visual_result(
        {
            1: {
                "category": place_category,
                "initial_center": (10.0, 20.0),
            },
            2: {
                "category": place_category,
                "initial_center": (30.0, 20.0),
            },
        }
    )

    artifacts_dir = (
        tmp_path
        / "nested"
        / "demo_structured_scene"
    )

    scene = builder.run(
        visual_prompting_result=visual_result,
        artifacts_dir=artifacts_dir,
    )

    artifact_path = (
        artifacts_dir
        / "demo_structured_scene.json"
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
            "object_id": "1",
            "category": place_category,
            "center": [10.0, 20.0],
        },
        {
            "object_id": "2",
            "category": place_category,
            "center": [30.0, 20.0],
        },
    ]

    assert data["relations"] == [
        {
            "subject_object_id": "1",
            "reference_object_id": "2",
            "relation": "LEFT",
        },
        {
            "subject_object_id": "2",
            "reference_object_id": "1",
            "relation": "RIGHT",
        },
    ]

    assert len(scene.objects) == 2
    assert len(scene.relations) == 2