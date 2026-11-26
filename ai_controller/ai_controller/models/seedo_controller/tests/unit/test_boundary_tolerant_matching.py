from __future__ import annotations

import json
import math

from types import SimpleNamespace

import pytest

from results import (
    StructuredScene,
    StructuredSceneObject,
)

from ai_controller.models.seedo_controller.utils import (
    spatial_relation,
    spatial_boundary_relation,
    build_spatial_relations,
    structured_scene_to_dict,
    validate_boundary_tolerance,
)

from ai_controller.models.seedo_controller.structural_matcher import (
    StructuralMatcher,
)

from ai_controller.models.seedo_controller.demo_structured_scene_builder import (
    DemoStructuredSceneBuilder,
)

from ai_controller.models.seedo_controller.runtime_structured_scene_builder import (
    RuntimeStructuredSceneBuilder,
)


# ============================================================
# HELPERS
# ============================================================

def _point(angle_deg):

    angle_rad = math.radians(angle_deg)

    return (
        100.0 * math.cos(angle_rad),
        100.0 * math.sin(angle_rad),
    )


def _scene(centers, prefix, directions=4):

    objects = tuple(
        StructuredSceneObject(
            object_id=f"{prefix}_{index}",
            category="bin",
            center=center,
        )
        for index, center in enumerate(centers)
    )

    return StructuredScene(
        objects=objects,
        relations=build_spatial_relations(
            objects=objects,
            directions=directions,
        ),
        directions=directions,
    )


def _angular_scene(angle_deg, prefix, directions=4):

    return _scene(
        centers=(
            _point(angle_deg),
            (0.0, 0.0),
        ),
        prefix=prefix,
        directions=directions,
    )


def _mapping_dict(mapping):

    return {
        match.demo_object_id: match.runtime_object_id
        for match in mapping.matches
    }


# ============================================================
# 1. BOUNDARY REGIONS
# ============================================================

@pytest.mark.parametrize(
    "angle, expected",
    [
        (0.0, None),
        (90.0, None),
        (180.0, None),
        (270.0, None),

        (45.0, "DOWN_RIGHT"),
        (135.0, "DOWN_LEFT"),
        (225.0, "UP_LEFT"),
        (315.0, "UP_RIGHT"),

        (35.0, "DOWN_RIGHT"),
        (55.0, "DOWN_RIGHT"),

        (34.9, None),
        (55.1, None),

        (215.0, "UP_LEFT"),
        (235.0, "UP_LEFT"),

        (214.9, None),
        (235.1, None),
    ],
)
def test_boundary_region_classification(angle, expected):

    result = spatial_boundary_relation(
        center_a=_point(angle),
        center_b=(0.0, 0.0),
        tolerance_deg=10.0,
    )

    assert result == expected


@pytest.mark.parametrize(
    "angle",
    [45.0, 135.0, 225.0, 315.0],
)
def test_zero_tolerance_disables_boundary(angle):

    assert spatial_boundary_relation(
        center_a=_point(angle),
        center_b=(0.0, 0.0),
        tolerance_deg=0.0,
    ) is None


@pytest.mark.parametrize(
    "tolerance",
    [
        -1.0,
        45.0,
        90.0,
        float("inf"),
        float("nan"),
    ],
)
def test_invalid_boundary_tolerance(tolerance):

    with pytest.raises(ValueError):

        validate_boundary_tolerance(tolerance)


# ============================================================
# 2. BOUNDARY-AWARE MATCHING
# ============================================================

@pytest.mark.parametrize(
    "demo_angle, runtime_angle, expected",
    [
        # Same cardinal direction.
        (180.0, 190.0, True),

        # Same cardinal: only one vector near boundary.
        (180.0, 220.0, True),

        # Same cardinal: different boundary regions.
        (138.0, 222.0, True),

        # Different cardinals, same boundary.
        (42.0, 48.0, True),
        (132.0, 138.0, True),
        (222.0, 228.0, True),
        (312.0, 318.0, True),

        # Inclusive tolerance boundaries.
        (35.0, 55.0, True),

        # Only runtime inside the boundary.
        (210.0, 228.0, False),

        # Only demo inside the boundary.
        (222.0, 240.0, False),

        # Both inside different boundaries.
        (222.0, 310.0, False),

        # Both outside the boundary.
        (180.0, 270.0, False),

        # Just outside tolerance.
        (34.9, 55.0, False),
    ],
)
def test_boundary_matching(
    demo_angle,
    runtime_angle,
    expected,
):

    demo = _angular_scene(
        demo_angle,
        "demo",
    )

    runtime = _angular_scene(
        runtime_angle,
        "runtime",
    )

    matcher = StructuralMatcher(
        boundary_tolerance_deg=10.0,
    )

    result = matcher.run(
        demo_scene=demo,
        runtime_scene=runtime,
    )

    assert result.is_valid == expected

    if expected:

        assert result.is_unique

        assert _mapping_dict(
            result.valid_mappings[0]
        ) == {
            "demo_0": "runtime_0",
            "demo_1": "runtime_1",
        }

    else:

        assert len(result.valid_mappings) == 0


# ============================================================
# 3. BACKWARD COMPATIBILITY
# ============================================================

@pytest.mark.parametrize(
    "angle, expected",
    [
        (0.0, "RIGHT"),
        (44.0, "RIGHT"),
        (46.0, "DOWN"),
        (90.0, "DOWN"),
        (134.0, "DOWN"),
        (136.0, "LEFT"),
        (180.0, "LEFT"),
        (224.0, "LEFT"),
        (226.0, "UP"),
        (270.0, "UP"),
        (314.0, "UP"),
        (316.0, "RIGHT"),
    ],
)
def test_original_four_direction_classification(
    angle,
    expected,
):

    assert spatial_relation(
        center_a=_point(angle),
        center_b=(0.0, 0.0),
        directions=4,
    ) == expected


@pytest.mark.parametrize(
    "demo_angle, runtime_angle, expected",
    [
        (180.0, 220.0, True),
        (222.0, 228.0, False),
        (42.0, 48.0, False),
        (0.0, 90.0, False),
    ],
)
def test_original_matching_with_disabled_tolerance(
    demo_angle,
    runtime_angle,
    expected,
):

    demo = _angular_scene(
        demo_angle,
        "demo",
    )

    runtime = _angular_scene(
        runtime_angle,
        "runtime",
    )

    result = StructuralMatcher(
        boundary_tolerance_deg=0.0,
    ).run(
        demo_scene=demo,
        runtime_scene=runtime,
    )

    assert result.is_valid == expected


@pytest.mark.parametrize(
    "tolerance",
    [0.0, 10.0, 20.0],
)
def test_eight_direction_matching_is_unchanged(tolerance):

    demo = _angular_scene(
        180.0,
        "demo",
        directions=8,
    )

    runtime = _angular_scene(
        220.0,
        "runtime",
        directions=8,
    )

    result = StructuralMatcher(
        boundary_tolerance_deg=tolerance,
    ).run(
        demo_scene=demo,
        runtime_scene=runtime,
    )

    # LEFT and UP_LEFT must remain incompatible
    # in the original eight-direction mode.
    assert not result.is_valid


def test_different_object_counts_remain_invalid():

    demo = _scene(
        [(0.0, 0.0), (100.0, 100.0)],
        "demo",
    )

    runtime = _scene(
        [(0.0, 0.0)],
        "runtime",
    )

    result = StructuralMatcher(
        boundary_tolerance_deg=10.0,
    ).run(
        demo_scene=demo,
        runtime_scene=runtime,
    )

    assert not result.is_valid
    assert len(result.valid_mappings) == 0


# ============================================================
# 4. JSON SERIALIZATION
# ============================================================

def test_boundary_metadata_serialization():

    scene = _angular_scene(
        222.0,
        "demo",
    )

    data = structured_scene_to_dict(
        scene=scene,
        tolerance_deg=10.0,
    )

    assert data["directions"] == 4
    assert data["boundary_tolerance_deg"] == 10.0

    forward = data["relations"][0]
    reverse = data["relations"][1]

    # Original cardinal relations must remain unchanged.
    assert forward["relation"] == "LEFT"
    assert reverse["relation"] == "RIGHT"

    # Boundary relations are additional metadata.
    assert forward["boundary_relation"] == "UP_LEFT"
    assert reverse["boundary_relation"] == "DOWN_RIGHT"


def test_boundary_metadata_outside_boundary():

    scene = _angular_scene(
        180.0,
        "demo",
    )

    data = structured_scene_to_dict(
        scene=scene,
        tolerance_deg=10.0,
    )

    assert data["relations"][0]["boundary_relation"] is None
    assert data["relations"][1]["boundary_relation"] is None


@pytest.mark.parametrize(
    "directions, tolerance",
    [
        (4, 0.0),
        (8, 0.0),
        (8, 10.0),
    ],
)
def test_original_json_format_is_preserved(
    directions,
    tolerance,
):

    scene = _angular_scene(
        222.0,
        "demo",
        directions=directions,
    )

    data = structured_scene_to_dict(
        scene=scene,
        tolerance_deg=tolerance,
    )

    assert set(data.keys()) == {
        "objects",
        "relations",
        "directions",
    }

    for relation in data["relations"]:

        assert set(relation.keys()) == {
            "subject_object_id",
            "reference_object_id",
            "relation",
        }


# ============================================================
# 5. NEW BUILDER FUNCTIONALITY
# ============================================================

@pytest.mark.parametrize(
    "builder_type",
    ["demo", "runtime"],
)
@pytest.mark.parametrize(
    "directions, tolerance, expected_metadata",
    [
        (4, 10.0, True),
        (4, 0.0, False),
        (8, 10.0, False),
    ],
)
def test_builder_boundary_metadata(
    tmp_path,
    builder_type,
    directions,
    tolerance,
    expected_metadata,
):

    center_a = _point(222.0)
    center_b = (0.0, 0.0)

    if builder_type == "demo":

        builder = DemoStructuredSceneBuilder(
            directions=directions,
            boundary_tolerance_deg=tolerance,
        )

        input_data = SimpleNamespace(
            track_id_map={
                0: {
                    "category": "bin",
                    "initial_center": center_a,
                },
                1: {
                    "category": "bin",
                    "initial_center": center_b,
                },
            }
        )

        scene = builder.run(
            visual_prompting_result=input_data,
            artifacts_dir=tmp_path,
        )

        path = tmp_path / "demo_structured_scene.json"

    else:

        builder = RuntimeStructuredSceneBuilder(
            directions=directions,
            boundary_tolerance_deg=tolerance,
        )

        input_data = SimpleNamespace(
            objects=(
                SimpleNamespace(
                    object_id="0",
                    category="bin",
                    pixel_coordinates=center_a,
                ),
                SimpleNamespace(
                    object_id="1",
                    category="bin",
                    pixel_coordinates=center_b,
                ),
            )
        )

        scene = builder.run(
            scene_state=input_data,
            artifacts_dir=tmp_path,
        )

        path = tmp_path / "runtime_structured_scene.json"

    assert path.is_file()

    data = json.loads(
        path.read_text(encoding="utf-8")
    )

    assert len(scene.objects) == 2
    assert len(scene.relations) == 2

    if expected_metadata:

        assert data["boundary_tolerance_deg"] == tolerance

        assert data["relations"][0]["relation"] == "LEFT"

        assert (
            data["relations"][0]["boundary_relation"]
            == "UP_LEFT"
        )

    else:

        assert "boundary_tolerance_deg" not in data
        assert "boundary_relation" not in data["relations"][0]


# ============================================================
# 6. REAL EXPERIMENTAL SCENES
# ============================================================

DEMO_LAYOUT_1 = (
    (267.0, 243.0),
    (169.0, 244.0),
    (255.0, 331.0),
    (139.0, 331.0),
)

RUNTIME_LAYOUT_1 = (
    (412.0, 331.0),
    (527.0, 329.0),
    (398.0, 240.0),
    (494.0, 237.0),
)

DEMO_LAYOUT_2 = (
    (279.0, 226.0),
    (186.0, 225.0),
    (266.0, 322.0),
    (152.0, 321.0),
)

RUNTIME_LAYOUT_2 = (
    (122.0, 334.0),
    (236.0, 331.0),
    (150.0, 248.0),
    (250.0, 242.0),
)


@pytest.mark.parametrize(
    "layout, directions, tolerance, expected_mappings",
    [
        (1, 4, 0.0, 0),
        (1, 4, 10.0, 0),
        (1, 4, 11.0, 1),
        (1, 8, 10.0, 1),

        (2, 4, 0.0, 0),
        (2, 4, 10.0, 1),
        (2, 4, 11.0, 1),
        (2, 8, 10.0, 1),
    ],
)
def test_real_experimental_layouts(
    layout,
    directions,
    tolerance,
    expected_mappings,
):

    if layout == 1:

        demo_centers = DEMO_LAYOUT_1
        runtime_centers = RUNTIME_LAYOUT_1

    else:

        demo_centers = DEMO_LAYOUT_2
        runtime_centers = RUNTIME_LAYOUT_2

    demo = _scene(
        demo_centers,
        "demo",
        directions,
    )

    runtime = _scene(
        runtime_centers,
        "runtime",
        directions,
    )

    assert len(demo.relations) == 12
    assert len(runtime.relations) == 12

    result = StructuralMatcher(
        boundary_tolerance_deg=tolerance,
    ).run(
        demo_scene=demo,
        runtime_scene=runtime,
    )

    assert len(
        result.valid_mappings
    ) == expected_mappings

    if expected_mappings == 1:

        assert result.is_unique

        assert _mapping_dict(
            result.valid_mappings[0]
        ) == {
            "demo_0": "runtime_3",
            "demo_1": "runtime_2",
            "demo_2": "runtime_1",
            "demo_3": "runtime_0",
        }