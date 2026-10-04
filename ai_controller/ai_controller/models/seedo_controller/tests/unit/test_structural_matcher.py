from __future__ import annotations

import json

import pytest

from results import (
    StructuredScene,
    StructuredSceneObject,
    StructuredSceneRelation,
)

from ai_controller.models.seedo_controller.structural_matcher import (
    StructuralMatcher,
)


def _object(
    object_id: str,
    category: str = "bin",
) -> StructuredSceneObject:
    return StructuredSceneObject(
        object_id=object_id,
        category=category,
        center=(0.0, 0.0),
    )


def _relation(
    subject: str,
    reference: str,
    relation: str,
) -> StructuredSceneRelation:
    return StructuredSceneRelation(
        subject_object_id=subject,
        reference_object_id=reference,
        relation=relation,
    )


def _unique_demo_scene(
    directions: int = 8,
) -> StructuredScene:
    return StructuredScene(
        objects=(
            _object("demo_a", "bin"),
            _object("demo_b", "bin"),
            _object("demo_c", "bin"),
        ),
        relations=(
            _relation("demo_a", "demo_b", "LEFT"),
            _relation("demo_a", "demo_c", "UP"),
            _relation("demo_b", "demo_a", "RIGHT"),
            _relation("demo_b", "demo_c", "UP_RIGHT"),
            _relation("demo_c", "demo_a", "DOWN"),
            _relation("demo_c", "demo_b", "DOWN_LEFT"),
        ),
        directions=directions,
    )


def _unique_runtime_scene(
    directions: int = 8,
) -> StructuredScene:
    # Deliberately use a different object order and different categories.
    #
    # Structural equivalence is:
    # demo_a -> runtime_x
    # demo_b -> runtime_y
    # demo_c -> runtime_z
    return StructuredScene(
        objects=(
            _object("runtime_z", "peg"),
            _object("runtime_x", "peg"),
            _object("runtime_y", "peg"),
        ),
        relations=(
            _relation("runtime_x", "runtime_y", "LEFT"),
            _relation("runtime_x", "runtime_z", "UP"),
            _relation("runtime_y", "runtime_x", "RIGHT"),
            _relation("runtime_y", "runtime_z", "UP_RIGHT"),
            _relation("runtime_z", "runtime_x", "DOWN"),
            _relation("runtime_z", "runtime_y", "DOWN_LEFT"),
        ),
        directions=directions,
    )


def test_run_rejects_different_direction_modes():
    matcher = StructuralMatcher()

    demo_scene = _unique_demo_scene(
        directions=8,
    )

    runtime_scene = _unique_runtime_scene(
        directions=4,
    )

    with pytest.raises(
        ValueError,
        match="different direction modes",
    ):
        matcher.run(
            demo_scene=demo_scene,
            runtime_scene=runtime_scene,
        )


def test_run_returns_no_mapping_for_different_object_counts():
    matcher = StructuralMatcher()

    demo_scene = StructuredScene(
        objects=(
            _object("demo_a"),
            _object("demo_b"),
        ),
        relations=(
            _relation("demo_a", "demo_b", "LEFT"),
            _relation("demo_b", "demo_a", "RIGHT"),
        ),
        directions=8,
    )

    runtime_scene = StructuredScene(
        objects=(
            _object("runtime_a"),
        ),
        relations=(),
        directions=8,
    )

    result = matcher.run(
        demo_scene=demo_scene,
        runtime_scene=runtime_scene,
    )

    assert result.valid_mappings == ()
    assert result.is_valid is False
    assert result.is_unique is False


def test_run_finds_unique_structure_preserving_mapping():
    matcher = StructuralMatcher()

    result = matcher.run(
        demo_scene=_unique_demo_scene(),
        runtime_scene=_unique_runtime_scene(),
    )

    assert result.is_valid is True
    assert result.is_unique is True

    assert len(result.valid_mappings) == 1

    mapping = result.valid_mappings[0]

    actual_matches = {
        match.demo_object_id:
            match.runtime_object_id
        for match in mapping.matches
    }

    assert actual_matches == {
        "demo_a": "runtime_x",
        "demo_b": "runtime_y",
        "demo_c": "runtime_z",
    }


def test_run_preserves_demo_object_order_in_mapping():
    matcher = StructuralMatcher()

    result = matcher.run(
        demo_scene=_unique_demo_scene(),
        runtime_scene=_unique_runtime_scene(),
    )

    mapping = result.valid_mappings[0]

    assert tuple(
        match.demo_object_id
        for match in mapping.matches
    ) == (
        "demo_a",
        "demo_b",
        "demo_c",
    )


def test_run_ignores_object_categories():
    matcher = StructuralMatcher()

    demo_scene = _unique_demo_scene()
    runtime_scene = _unique_runtime_scene()

    assert {
        obj.category
        for obj in demo_scene.objects
    } == {"bin"}

    assert {
        obj.category
        for obj in runtime_scene.objects
    } == {"peg"}

    result = matcher.run(
        demo_scene=demo_scene,
        runtime_scene=runtime_scene,
    )

    assert result.is_valid is True
    assert result.is_unique is True


def test_run_returns_no_mapping_when_structure_differs():
    matcher = StructuralMatcher()

    demo_scene = _unique_demo_scene()

    runtime_scene = StructuredScene(
        objects=(
            _object("runtime_a"),
            _object("runtime_b"),
            _object("runtime_c"),
        ),
        relations=(
            _relation("runtime_a", "runtime_b", "RIGHT"),
            _relation("runtime_a", "runtime_c", "RIGHT"),
            _relation("runtime_b", "runtime_a", "RIGHT"),
            _relation("runtime_b", "runtime_c", "RIGHT"),
            _relation("runtime_c", "runtime_a", "RIGHT"),
            _relation("runtime_c", "runtime_b", "RIGHT"),
        ),
        directions=8,
    )

    result = matcher.run(
        demo_scene=demo_scene,
        runtime_scene=runtime_scene,
    )

    assert result.valid_mappings == ()
    assert result.is_valid is False
    assert result.is_unique is False


def test_run_returns_all_valid_mappings_for_symmetric_structure():
    matcher = StructuralMatcher()

    demo_scene = StructuredScene(
        objects=(
            _object("demo_a"),
            _object("demo_b"),
        ),
        relations=(
            _relation("demo_a", "demo_b", "LEFT"),
            _relation("demo_b", "demo_a", "LEFT"),
        ),
        directions=8,
    )

    runtime_scene = StructuredScene(
        objects=(
            _object("runtime_x"),
            _object("runtime_y"),
        ),
        relations=(
            _relation("runtime_x", "runtime_y", "LEFT"),
            _relation("runtime_y", "runtime_x", "LEFT"),
        ),
        directions=8,
    )

    result = matcher.run(
        demo_scene=demo_scene,
        runtime_scene=runtime_scene,
    )

    assert result.is_valid is True
    assert result.is_unique is False

    assert len(result.valid_mappings) == 2

    actual_mappings = {
        tuple(
            (
                match.demo_object_id,
                match.runtime_object_id,
            )
            for match in mapping.matches
        )
        for mapping in result.valid_mappings
    }

    assert actual_mappings == {
        (
            ("demo_a", "runtime_x"),
            ("demo_b", "runtime_y"),
        ),
        (
            ("demo_a", "runtime_y"),
            ("demo_b", "runtime_x"),
        ),
    }


def test_run_accepts_four_direction_scenes():
    matcher = StructuralMatcher()

    demo_scene = StructuredScene(
        objects=(
            _object("demo_left"),
            _object("demo_right"),
        ),
        relations=(
            _relation(
                "demo_left",
                "demo_right",
                "LEFT",
            ),
            _relation(
                "demo_right",
                "demo_left",
                "RIGHT",
            ),
        ),
        directions=4,
    )

    runtime_scene = StructuredScene(
        objects=(
            _object("runtime_right"),
            _object("runtime_left"),
        ),
        relations=(
            _relation(
                "runtime_left",
                "runtime_right",
                "LEFT",
            ),
            _relation(
                "runtime_right",
                "runtime_left",
                "RIGHT",
            ),
        ),
        directions=4,
    )

    result = matcher.run(
        demo_scene=demo_scene,
        runtime_scene=runtime_scene,
    )

    assert result.is_valid is True
    assert result.is_unique is True

    actual_matches = {
        match.demo_object_id:
            match.runtime_object_id
        for match in result.valid_mappings[0].matches
    }

    assert actual_matches == {
        "demo_left": "runtime_left",
        "demo_right": "runtime_right",
    }


def test_run_rejects_incomplete_demo_relation_graph():
    matcher = StructuralMatcher()

    demo_scene = StructuredScene(
        objects=(
            _object("a"),
            _object("b"),
        ),
        relations=(
            _relation("a", "b", "LEFT"),
        ),
        directions=8,
    )

    runtime_scene = StructuredScene(
        objects=(
            _object("x"),
            _object("y"),
        ),
        relations=(
            _relation("x", "y", "LEFT"),
            _relation("y", "x", "RIGHT"),
        ),
        directions=8,
    )

    with pytest.raises(
        ValueError,
        match="expected number of directed relations",
    ):
        matcher.run(
            demo_scene=demo_scene,
            runtime_scene=runtime_scene,
        )


def test_run_rejects_incomplete_runtime_relation_graph():
    matcher = StructuralMatcher()

    demo_scene = StructuredScene(
        objects=(
            _object("a"),
            _object("b"),
        ),
        relations=(
            _relation("a", "b", "LEFT"),
            _relation("b", "a", "RIGHT"),
        ),
        directions=8,
    )

    runtime_scene = StructuredScene(
        objects=(
            _object("x"),
            _object("y"),
        ),
        relations=(
            _relation("x", "y", "LEFT"),
        ),
        directions=8,
    )

    with pytest.raises(
        ValueError,
        match="expected number of directed relations",
    ):
        matcher.run(
            demo_scene=demo_scene,
            runtime_scene=runtime_scene,
        )


def test_run_rejects_unknown_subject_object():
    matcher = StructuralMatcher()

    demo_scene = StructuredScene(
        objects=(
            _object("a"),
            _object("b"),
        ),
        relations=(
            _relation("unknown", "b", "LEFT"),
            _relation("b", "a", "RIGHT"),
        ),
        directions=8,
    )

    runtime_scene = StructuredScene(
        objects=(
            _object("x"),
            _object("y"),
        ),
        relations=(
            _relation("x", "y", "LEFT"),
            _relation("y", "x", "RIGHT"),
        ),
        directions=8,
    )

    with pytest.raises(
        ValueError,
        match="unknown subject object",
    ):
        matcher.run(
            demo_scene=demo_scene,
            runtime_scene=runtime_scene,
        )


def test_run_rejects_unknown_reference_object():
    matcher = StructuralMatcher()

    demo_scene = StructuredScene(
        objects=(
            _object("a"),
            _object("b"),
        ),
        relations=(
            _relation("a", "unknown", "LEFT"),
            _relation("b", "a", "RIGHT"),
        ),
        directions=8,
    )

    runtime_scene = StructuredScene(
        objects=(
            _object("x"),
            _object("y"),
        ),
        relations=(
            _relation("x", "y", "LEFT"),
            _relation("y", "x", "RIGHT"),
        ),
        directions=8,
    )

    with pytest.raises(
        ValueError,
        match="unknown reference object",
    ):
        matcher.run(
            demo_scene=demo_scene,
            runtime_scene=runtime_scene,
        )


def test_run_rejects_self_relation():
    matcher = StructuralMatcher()

    demo_scene = StructuredScene(
        objects=(
            _object("a"),
            _object("b"),
        ),
        relations=(
            _relation("a", "a", "LEFT"),
            _relation("b", "a", "RIGHT"),
        ),
        directions=8,
    )

    runtime_scene = StructuredScene(
        objects=(
            _object("x"),
            _object("y"),
        ),
        relations=(
            _relation("x", "y", "LEFT"),
            _relation("y", "x", "RIGHT"),
        ),
        directions=8,
    )

    with pytest.raises(
        ValueError,
        match="same object as both subject and reference",
    ):
        matcher.run(
            demo_scene=demo_scene,
            runtime_scene=runtime_scene,
        )


def test_run_rejects_duplicate_relation_pair():
    matcher = StructuralMatcher()

    demo_scene = StructuredScene(
        objects=(
            _object("a"),
            _object("b"),
        ),
        relations=(
            _relation("a", "b", "LEFT"),
            _relation("a", "b", "RIGHT"),
        ),
        directions=8,
    )

    runtime_scene = StructuredScene(
        objects=(
            _object("x"),
            _object("y"),
        ),
        relations=(
            _relation("x", "y", "LEFT"),
            _relation("y", "x", "RIGHT"),
        ),
        directions=8,
    )

    with pytest.raises(
        ValueError,
        match="Duplicate structured relation",
    ):
        matcher.run(
            demo_scene=demo_scene,
            runtime_scene=runtime_scene,
        )


def test_run_writes_unique_matching_artifact(
    tmp_path,
):
    matcher = StructuralMatcher()

    artifacts_dir = (
        tmp_path
        / "nested"
        / "structural_matching"
    )

    result = matcher.run(
        demo_scene=_unique_demo_scene(),
        runtime_scene=_unique_runtime_scene(),
        artifacts_dir=artifacts_dir,
    )

    artifact_path = (
        artifacts_dir
        / "structural_matching_result.json"
    )

    assert artifact_path.is_file()

    with artifact_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        data = json.load(stream)

    assert data == {
        "valid_mappings": [
            {
                "matches": [
                    {
                        "demo_object_id": "demo_a",
                        "runtime_object_id": "runtime_x",
                    },
                    {
                        "demo_object_id": "demo_b",
                        "runtime_object_id": "runtime_y",
                    },
                    {
                        "demo_object_id": "demo_c",
                        "runtime_object_id": "runtime_z",
                    },
                ]
            }
        ]
    }

    assert result.is_unique is True


def test_run_writes_empty_artifact_for_object_count_mismatch(
    tmp_path,
):
    matcher = StructuralMatcher()

    demo_scene = StructuredScene(
        objects=(
            _object("demo_a"),
            _object("demo_b"),
        ),
        relations=(
            _relation("demo_a", "demo_b", "LEFT"),
            _relation("demo_b", "demo_a", "RIGHT"),
        ),
        directions=8,
    )

    runtime_scene = StructuredScene(
        objects=(
            _object("runtime_a"),
        ),
        relations=(),
        directions=8,
    )

    result = matcher.run(
        demo_scene=demo_scene,
        runtime_scene=runtime_scene,
        artifacts_dir=tmp_path,
    )

    artifact_path = (
        tmp_path
        / "structural_matching_result.json"
    )

    assert artifact_path.is_file()

    with artifact_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        data = json.load(stream)

    assert data == {
        "valid_mappings": []
    }

    assert result.is_valid is False