from __future__ import annotations

import pytest

from ai_controller.models.seedo_controller import utils as spatial_utils


@pytest.mark.parametrize(
    ("center_a", "center_b", "expected"),
    [
        ((1.0, 0.0), (0.0, 0.0), "RIGHT"),
        ((1.0, 1.0), (0.0, 0.0), "DOWN_RIGHT"),
        ((0.0, 1.0), (0.0, 0.0), "DOWN"),
        ((-1.0, 1.0), (0.0, 0.0), "DOWN_LEFT"),
        ((-1.0, 0.0), (0.0, 0.0), "LEFT"),
        ((-1.0, -1.0), (0.0, 0.0), "UP_LEFT"),
        ((0.0, -1.0), (0.0, 0.0), "UP"),
        ((1.0, -1.0), (0.0, 0.0), "UP_RIGHT"),
    ],
)
def test_spatial_relation_eight_directions(
    center_a,
    center_b,
    expected,
):
    relation = spatial_utils.spatial_relation(
        center_a=center_a,
        center_b=center_b,
        directions=8,
    )

    assert relation == expected


@pytest.mark.parametrize(
    ("center_a", "center_b", "expected"),
    [
        ((1.0, 0.0), (0.0, 0.0), "RIGHT"),
        ((0.0, 1.0), (0.0, 0.0), "DOWN"),
        ((-1.0, 0.0), (0.0, 0.0), "LEFT"),
        ((0.0, -1.0), (0.0, 0.0), "UP"),
    ],
)
def test_spatial_relation_four_cardinal_directions(
    center_a,
    center_b,
    expected,
):
    relation = spatial_utils.spatial_relation(
        center_a=center_a,
        center_b=center_b,
        directions=4,
    )

    assert relation == expected


@pytest.mark.parametrize(
    ("center_a", "expected"),
    [
        ((2.0, 1.0), "RIGHT"),
        ((1.0, 2.0), "DOWN"),
        ((-1.0, 2.0), "DOWN"),
        ((-2.0, 1.0), "LEFT"),
        ((-2.0, -1.0), "LEFT"),
        ((-1.0, -2.0), "UP"),
        ((1.0, -2.0), "UP"),
        ((2.0, -1.0), "RIGHT"),
    ],
)
def test_spatial_relation_four_directions_collapses_diagonals(
    center_a,
    expected,
):
    relation = spatial_utils.spatial_relation(
        center_a=center_a,
        center_b=(0.0, 0.0),
        directions=4,
    )

    assert relation == expected


@pytest.mark.parametrize(
    "directions",
    [0, 1, 3, 5, 6, 7, 9],
)
def test_spatial_relation_rejects_invalid_direction_count(
    directions,
):
    with pytest.raises(
        ValueError,
        match="Unsupported number of directions",
    ):
        spatial_utils.spatial_relation(
            center_a=(1.0, 0.0),
            center_b=(0.0, 0.0),
            directions=directions,
        )


def test_spatial_relation_rejects_identical_centers():
    with pytest.raises(
        ValueError,
        match="identical centers",
    ):
        spatial_utils.spatial_relation(
            center_a=(10.0, 20.0),
            center_b=(10.0, 20.0),
            directions=8,
        )


def test_build_spatial_relations_builds_all_directed_pairs():
    objects = (
        spatial_utils.StructuredSceneObject(
            object_id="a",
            category="bin",
            center=(0.0, 0.0),
        ),
        spatial_utils.StructuredSceneObject(
            object_id="b",
            category="bin",
            center=(10.0, 0.0),
        ),
        spatial_utils.StructuredSceneObject(
            object_id="c",
            category="bin",
            center=(0.0, 10.0),
        ),
    )

    relations = spatial_utils.build_spatial_relations(
        objects=objects,
        directions=8,
    )

    assert len(relations) == 6

    actual_relations = {
        (
            relation.subject_object_id,
            relation.reference_object_id,
            relation.relation,
        )
        for relation in relations
    }

    expected_relations = {
        ("a", "b", "LEFT"),
        ("a", "c", "UP"),
        ("b", "a", "RIGHT"),
        ("b", "c", "UP_RIGHT"),
        ("c", "a", "DOWN"),
        ("c", "b", "DOWN_LEFT"),
    }

    assert actual_relations == expected_relations


def test_build_spatial_relations_respects_four_direction_mode():
    objects = (
        spatial_utils.StructuredSceneObject(
            object_id="a",
            category="bin",
            center=(0.0, 0.0),
        ),
        spatial_utils.StructuredSceneObject(
            object_id="b",
            category="bin",
            center=(20.0, 10.0),
        ),
    )

    relations = spatial_utils.build_spatial_relations(
        objects=objects,
        directions=4,
    )

    actual_relations = {
        (
            relation.subject_object_id,
            relation.reference_object_id,
            relation.relation,
        )
        for relation in relations
    }

    assert actual_relations == {
        ("a", "b", "LEFT"),
        ("b", "a", "RIGHT"),
    }


@pytest.mark.parametrize(
    "objects",
    [
        (),
        (
            spatial_utils.StructuredSceneObject(
                object_id="a",
                category="bin",
                center=(0.0, 0.0),
            ),
        ),
    ],
)
def test_build_spatial_relations_returns_empty_without_pairs(
    objects,
):
    relations = spatial_utils.build_spatial_relations(
        objects=objects,
        directions=8,
    )

    assert relations == ()


def test_build_spatial_relations_rejects_duplicate_object_ids():
    objects = (
        spatial_utils.StructuredSceneObject(
            object_id="duplicate",
            category="bin",
            center=(0.0, 0.0),
        ),
        spatial_utils.StructuredSceneObject(
            object_id="duplicate",
            category="bin",
            center=(10.0, 0.0),
        ),
    )

    with pytest.raises(
        ValueError,
        match="unique object IDs",
    ):
        spatial_utils.build_spatial_relations(
            objects=objects,
            directions=8,
        )


def test_build_spatial_relations_rejects_identical_centers():
    objects = (
        spatial_utils.StructuredSceneObject(
            object_id="a",
            category="bin",
            center=(10.0, 20.0),
        ),
        spatial_utils.StructuredSceneObject(
            object_id="b",
            category="bin",
            center=(10.0, 20.0),
        ),
    )

    with pytest.raises(
        ValueError,
        match="identical centers",
    ):
        spatial_utils.build_spatial_relations(
            objects=objects,
            directions=8,
        )


def test_build_spatial_relations_rejects_invalid_direction_count():
    with pytest.raises(
        ValueError,
        match="Unsupported number of directions",
    ):
        spatial_utils.build_spatial_relations(
            objects=(),
            directions=6,
        )