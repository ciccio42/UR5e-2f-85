from __future__ import annotations

import json
from types import SimpleNamespace

import numpy as np
import pytest

from results import (
    PrimitiveStep,
    SceneObject,
    SceneState,
)

from ai_controller.models.seedo_controller import (
    motion_layer as motion_layer_module,
)
from ai_controller.models.seedo_controller.motion_layer import (
    SeeDoMotionLayer,
)


def _motion_layer(
    **overrides,
) -> SeeDoMotionLayer:
    kwargs = {
        "grasp_orientation": (
            0.0,
            0.0,
            0.0,
            1.0,
        ),
        "min_step": 0.01,
        "reach_hover_height": 0.10,
        "approach_z_offset": 0.02,
        "release_height_offset": 0.03,
        "lift_height": 0.15,
        "gripper_open_position": 0.8,
        "gripper_closed_position": 0.2,
        "object_y_offset": -0.06,
        "assembly_alignment_z_offset": 0.135,
        "assembly_insertion_z_offset": -0.11,
    }

    kwargs.update(
        overrides
    )

    return SeeDoMotionLayer(
        **kwargs
    )


def _scene_object(
    object_id: str,
    *,
    position_base=(0.4, 0.5, 0.6),
) -> SceneObject:
    return SceneObject(
        object_id=object_id,
        label=object_id,
        pixel_coordinates=(10, 20),
        position_camera=(0.1, 0.2, 0.3),
        position_base=position_base,
        category="object",
    )


def _scene_state(
    *objects: SceneObject,
) -> SceneState:
    return SceneState(
        objects=tuple(objects),
    )


def _primitive(
    name: str,
    target="target",
    argument_name: str = "target",
) -> PrimitiveStep:
    return PrimitiveStep(
        name=name,
        arguments={
            argument_name: target,
        },
    )


def _reset(
    layer: SeeDoMotionLayer,
    *,
    position=(0.1, 0.2, 0.3),
    orientation=(0.0, 0.0, 0.0, 1.0),
    artifacts_dir=None,
):
    layer.reset(
        current_position=position,
        current_orientation=orientation,
        artifacts_dir=artifacts_dir,
    )


def _install_fake_move_linear(
    monkeypatch,
    layer,
    captured,
):
    def fake_move_linear(
        *,
        target_position,
        target_orientation,
    ):
        position = np.asarray(
            target_position,
            dtype=np.float64,
        )

        orientation = np.asarray(
            target_orientation,
            dtype=np.float64,
        )

        captured["target_position"] = (
            position.copy()
        )

        captured["target_orientation"] = (
            orientation.copy()
        )

        layer.current_pose = (
            SimpleNamespace(
                position=position.copy(),
                orientation=orientation.copy(),
            )
        )

        return [
            layer._build_action(
                position=position,
                orientation=orientation,
                gripper_position=(
                    layer.current_gripper_position
                ),
            )
        ]

    monkeypatch.setattr(
        layer,
        "_move_linear",
        fake_move_linear,
    )


# ---------------------------------------------------------------------
# Construction and reset
# ---------------------------------------------------------------------


@pytest.mark.parametrize(
    "orientation",
    [
        (),
        (0.0,),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0, 2.0),
    ],
)
def test_constructor_rejects_invalid_grasp_orientation(
    orientation,
):
    with pytest.raises(
        ValueError,
        match="grasp_orientation must contain exactly 4 values",
    ):
        _motion_layer(
            grasp_orientation=orientation,
        )


def test_constructor_initializes_motion_state():
    layer = _motion_layer()

    assert layer.current_pose is None
    assert (
        layer.current_gripper_position
        == pytest.approx(0.8)
    )
    assert layer.active_grasp_position is None
    assert layer.active_grasp_orientation is None
    assert layer.motion_history == []
    assert layer.artifacts_dir is None


@pytest.mark.parametrize(
    "position",
    [
        (),
        (0.0,),
        (0.0, 0.0),
        (0.0, 0.0, 0.0, 0.0),
    ],
)
def test_reset_rejects_invalid_position(
    position,
):
    layer = _motion_layer()

    with pytest.raises(
        ValueError,
        match="current_position must contain exactly 3 values",
    ):
        layer.reset(
            current_position=position,
            current_orientation=(
                0.0,
                0.0,
                0.0,
                1.0,
            ),
        )


@pytest.mark.parametrize(
    "orientation",
    [
        (),
        (0.0,),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0, 2.0),
    ],
)
def test_reset_rejects_invalid_orientation(
    orientation,
):
    layer = _motion_layer()

    with pytest.raises(
        ValueError,
        match="current_orientation must contain exactly 4 values",
    ):
        layer.reset(
            current_position=(
                0.0,
                0.0,
                0.0,
            ),
            current_orientation=orientation,
        )


def test_reset_initializes_pose_and_open_gripper():
    layer = _motion_layer()

    _reset(
        layer,
        position=(0.1, 0.2, 0.3),
        orientation=(0.0, 0.0, 1.0, 0.0),
    )

    assert np.allclose(
        layer.current_pose.position,
        np.array(
            [0.1, 0.2, 0.3]
        ),
    )

    assert np.allclose(
        layer.current_pose.orientation,
        np.array(
            [0.0, 0.0, 1.0, 0.0]
        ),
    )

    assert (
        layer.current_gripper_position
        == pytest.approx(0.8)
    )


def test_reset_clears_previous_motion_history():
    layer = _motion_layer()

    _reset(layer)

    layer.motion_history.append(
        {
            "test": True,
        }
    )

    _reset(layer)

    assert layer.motion_history == []

def test_reset_clears_active_grasp_pose():
    layer = _motion_layer()

    _reset(layer)

    layer.set_grasp_pose(
        position=(
            0.4,
            0.5,
            0.6,
        ),
        orientation=(
            0.0,
            0.0,
            0.0,
            1.0,
        ),
    )

    assert (
        layer.active_grasp_position
        is not None
    )

    assert (
        layer.active_grasp_orientation
        is not None
    )

    _reset(layer)

    assert (
        layer.active_grasp_position
        is None
    )

    assert (
        layer.active_grasp_orientation
        is None
    )


# ---------------------------------------------------------------------
# Translation validation
# ---------------------------------------------------------------------


def test_translate_requires_reset():
    layer = _motion_layer()

    with pytest.raises(
        RuntimeError,
        match="has not been initialized",
    ):
        layer.translate(
            primitive_step=_primitive(
                "pick"
            ),
            scene_state=_scene_state(
                _scene_object(
                    "target"
                )
            ),
        )


def test_translate_requires_primitive_step():
    layer = _motion_layer()
    _reset(layer)

    with pytest.raises(
        TypeError,
        match="primitive_step must be a PrimitiveStep",
    ):
        layer.translate(
            primitive_step="pick",
            scene_state=_scene_state(),
        )


def test_translate_requires_scene_state():
    layer = _motion_layer()
    _reset(layer)

    with pytest.raises(
        TypeError,
        match="scene_state must be a SceneState",
    ):
        layer.translate(
            primitive_step=_primitive(
                "pick"
            ),
            scene_state=[],
        )


def test_translate_rejects_unsupported_primitive():
    layer = _motion_layer()
    _reset(layer)

    with pytest.raises(
        ValueError,
        match="Unsupported SeeDo primitive",
    ):
        layer.translate(
            primitive_step=_primitive(
                "dance"
            ),
            scene_state=_scene_state(
                _scene_object(
                    "target"
                )
            ),
        )


def test_translate_strips_primitive_name():
    layer = _motion_layer()
    _reset(layer)

    actions = layer.translate(
        primitive_step=_primitive(
            "  pick  "
        ),
        scene_state=_scene_state(
            _scene_object(
                "target"
            )
        ),
    )

    assert len(actions) == 1
    assert actions[0][7] == pytest.approx(
        0.2
    )


# ---------------------------------------------------------------------
# Primitive geometry
# ---------------------------------------------------------------------


def test_reach_builds_hover_target(
    monkeypatch,
):
    layer = _motion_layer()

    _reset(layer)

    captured = {}

    _install_fake_move_linear(
        monkeypatch,
        layer,
        captured,
    )

    layer.translate(
        primitive_step=_primitive(
            "reach"
        ),
        scene_state=_scene_state(
            _scene_object(
                "target",
                position_base=(
                    0.4,
                    0.5,
                    0.6,
                ),
            )
        ),
    )

    assert np.allclose(
        captured["target_position"],
        np.array(
            [
                0.4,
                0.44,
                0.70,
            ]
        ),
    )

    assert np.allclose(
        captured["target_orientation"],
        np.array(
            [
                0.0,
                0.0,
                0.0,
                1.0,
            ]
        ),
    )


def test_approaching_uses_active_grasp_pose(
    monkeypatch,
):
    layer = _motion_layer()
    _reset(layer)

    grasp_position = np.array(
        [
            0.18,
            0.47,
            -0.10,
        ],
        dtype=np.float64,
    )

    grasp_orientation = np.array(
        [
            0.97719848,
            0.17864325,
            -0.03462727,
            -0.10941056,
        ],
        dtype=np.float64,
    )

    layer.set_grasp_pose(
        position=grasp_position,
        orientation=grasp_orientation,
    )

    captured = {}

    _install_fake_move_linear(
        monkeypatch,
        layer,
        captured,
    )

    layer.translate(
        primitive_step=_primitive(
            "approaching"
        ),
        scene_state=_scene_state(
            _scene_object(
                "target",
                # Deliberately unrelated to the grasp pose.
                position_base=(
                    9.0,
                    9.0,
                    9.0,
                ),
            )
        ),
    )

    np.testing.assert_allclose(
        captured[
            "target_position"
        ],
        grasp_position,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        captured[
            "target_orientation"
        ],
        grasp_orientation,
        atol=1e-9,
    )

def test_approaching_requires_active_grasp_pose():
    layer = _motion_layer()
    _reset(layer)

    with pytest.raises(
        RuntimeError,
        match="No active grasp pose",
    ):
        layer.translate(
            primitive_step=_primitive(
                "approaching"
            ),
            scene_state=_scene_state(
                _scene_object(
                    "target"
                )
            ),
        )


def test_pick_closes_gripper_without_moving():
    layer = _motion_layer()

    _reset(
        layer,
        position=(
            0.1,
            0.2,
            0.3,
        ),
    )

    actions = layer.translate(
        primitive_step=_primitive(
            "pick"
        ),
        scene_state=_scene_state(
            _scene_object(
                "target"
            )
        ),
    )

    assert len(actions) == 1

    action = actions[0]

    assert np.allclose(
        action[:3],
        np.array(
            [0.1, 0.2, 0.3]
        ),
    )

    assert action[7] == pytest.approx(
        0.2
    )

    assert (
        layer.current_gripper_position
        == pytest.approx(0.2)
    )


def test_lift_up_preserves_xy_and_adds_lift_height(
    monkeypatch,
):
    layer = _motion_layer(
        lift_height=0.15,
    )

    _reset(
        layer,
        position=(
            0.1,
            0.2,
            0.3,
        ),
    )

    captured = {}

    _install_fake_move_linear(
        monkeypatch,
        layer,
        captured,
    )

    layer.translate(
        primitive_step=_primitive(
            "lift_up"
        ),
        scene_state=_scene_state(
            _scene_object(
                "target"
            )
        ),
    )

    assert np.allclose(
        captured["target_position"],
        np.array(
            [
                0.1,
                0.2,
                0.45,
            ]
        ),
    )


def test_moving_uses_destination_xy_and_current_z(
    monkeypatch,
):
    layer = _motion_layer()

    _reset(
        layer,
        position=(
            0.1,
            0.2,
            0.9,
        ),
    )

    captured = {}

    _install_fake_move_linear(
        monkeypatch,
        layer,
        captured,
    )

    layer.translate(
        primitive_step=_primitive(
            "moving"
        ),
        scene_state=_scene_state(
            _scene_object(
                "target",
                position_base=(
                    0.7,
                    0.8,
                    0.2,
                ),
            )
        ),
    )

    assert np.allclose(
        captured["target_position"],
        np.array(
            [
                0.7,
                0.8,
                0.9,
            ]
        ),
    )


def test_placing_moves_above_destination_and_opens_gripper(
    monkeypatch,
):
    layer = _motion_layer(
        release_height_offset=0.03,
    )

    _reset(layer)

    layer.current_gripper_position = (
        layer.gripper_closed_position
    )

    captured = {}

    _install_fake_move_linear(
        monkeypatch,
        layer,
        captured,
    )

    actions = layer.translate(
        primitive_step=_primitive(
            "placing"
        ),
        scene_state=_scene_state(
            _scene_object(
                "target",
                position_base=(
                    0.4,
                    0.5,
                    0.6,
                ),
            )
        ),
    )

    assert np.allclose(
        captured["target_position"],
        np.array(
            [
                0.4,
                0.5,
                0.63,
            ]
        ),
    )

    assert len(actions) == 2

    assert actions[0][7] == pytest.approx(
        0.2
    )

    assert actions[1][7] == pytest.approx(
        0.8
    )

    assert (
        layer.current_gripper_position
        == pytest.approx(0.8)
    )


def test_aligning_uses_assembly_alignment_offset(
    monkeypatch,
):
    layer = _motion_layer(
        assembly_alignment_z_offset=0.135,
    )

    _reset(layer)

    captured = {}

    _install_fake_move_linear(
        monkeypatch,
        layer,
        captured,
    )

    layer.translate(
        primitive_step=_primitive(
            "aligning"
        ),
        scene_state=_scene_state(
            _scene_object(
                "target",
                position_base=(
                    0.4,
                    0.5,
                    0.6,
                ),
            )
        ),
    )

    assert np.allclose(
        captured["target_position"],
        np.array(
            [
                0.4,
                0.5,
                0.735,
            ]
        ),
    )


def test_inserting_uses_insertion_offset_and_opens_gripper(
    monkeypatch,
):
    layer = _motion_layer(
        assembly_insertion_z_offset=-0.11,
    )

    _reset(layer)

    layer.current_gripper_position = (
        layer.gripper_closed_position
    )

    captured = {}

    _install_fake_move_linear(
        monkeypatch,
        layer,
        captured,
    )

    actions = layer.translate(
        primitive_step=_primitive(
            "inserting"
        ),
        scene_state=_scene_state(
            _scene_object(
                "target",
                position_base=(
                    0.4,
                    0.5,
                    0.6,
                ),
            )
        ),
    )

    assert np.allclose(
        captured["target_position"],
        np.array(
            [
                0.4,
                0.5,
                0.49,
            ]
        ),
    )

    assert len(actions) == 2

    assert actions[0][7] == pytest.approx(
        0.2
    )

    assert actions[1][7] == pytest.approx(
        0.8
    )

    assert (
        layer.current_gripper_position
        == pytest.approx(0.8)
    )


# ---------------------------------------------------------------------
# Linear motion and action format
# ---------------------------------------------------------------------


def test_move_linear_builds_actions_and_updates_pose(
    monkeypatch,
):
    layer = _motion_layer()

    _reset(
        layer,
        position=(
            0.0,
            0.0,
            0.0,
        ),
    )

    layer.current_gripper_position = 0.25

    waypoints = [
        SimpleNamespace(
            position=np.array(
                [0.1, 0.0, 0.0]
            ),
            orientation=np.array(
                [0.0, 0.0, 0.0, 1.0]
            ),
        ),
        SimpleNamespace(
            position=np.array(
                [0.2, 0.0, 0.0]
            ),
            orientation=np.array(
                [0.0, 0.0, 0.0, 1.0]
            ),
        ),
    ]

    captured = {}

    def fake_build_linear_waypoints(
        start_position,
        start_orientation,
        target_position,
        target_orientation,
        min_step,
    ):
        captured["start_position"] = (
            np.asarray(
                start_position
            ).copy()
        )

        captured["start_orientation"] = (
            np.asarray(
                start_orientation
            ).copy()
        )

        captured["target_position"] = (
            np.asarray(
                target_position
            ).copy()
        )

        captured["target_orientation"] = (
            np.asarray(
                target_orientation
            ).copy()
        )

        captured["min_step"] = min_step

        return waypoints

    monkeypatch.setattr(
        motion_layer_module,
        "build_linear_waypoints",
        fake_build_linear_waypoints,
    )

    actions = layer._move_linear(
        target_position=(
            0.2,
            0.0,
            0.0,
        ),
        target_orientation=(
            0.0,
            0.0,
            0.0,
            1.0,
        ),
    )

    assert len(actions) == 2

    assert all(
        action.shape == (8,)
        for action in actions
    )

    assert np.allclose(
        actions[0][:3],
        np.array(
            [0.1, 0.0, 0.0]
        ),
    )

    assert np.allclose(
        actions[1][:3],
        np.array(
            [0.2, 0.0, 0.0]
        ),
    )

    assert actions[0][7] == pytest.approx(
        0.25
    )

    assert actions[1][7] == pytest.approx(
        0.25
    )

    assert (
        layer.current_pose
        is waypoints[-1]
    )

    assert captured["min_step"] == pytest.approx(
        0.01
    )


def test_move_linear_keeps_pose_when_no_waypoints(
    monkeypatch,
):
    layer = _motion_layer()

    _reset(
        layer,
        position=(
            0.1,
            0.2,
            0.3,
        ),
    )

    previous_pose = (
        layer.current_pose
    )

    monkeypatch.setattr(
        motion_layer_module,
        "build_linear_waypoints",
        lambda *args, **kwargs: [],
    )

    actions = layer._move_linear(
        target_position=(
            0.4,
            0.5,
            0.6,
        ),
        target_orientation=(
            0.0,
            0.0,
            0.0,
            1.0,
        ),
    )

    assert actions == []
    assert layer.current_pose is previous_pose


def test_move_linear_requires_current_pose():
    layer = _motion_layer()

    with pytest.raises(
        RuntimeError,
        match="Cannot generate motion without a current pose",
    ):
        layer._move_linear(
            target_position=(
                0.0,
                0.0,
                0.0,
            ),
            target_orientation=(
                0.0,
                0.0,
                0.0,
                1.0,
            ),
        )


def test_build_action_has_expected_eight_value_format():
    action = SeeDoMotionLayer._build_action(
        position=(
            1.0,
            2.0,
            3.0,
        ),
        orientation=(
            0.1,
            0.2,
            0.3,
            0.4,
        ),
        gripper_position=0.5,
    )

    assert action.shape == (
        8,
    )

    assert np.allclose(
        action,
        np.array(
            [
                1.0,
                2.0,
                3.0,
                0.1,
                0.2,
                0.3,
                0.4,
                0.5,
            ]
        ),
    )


# ---------------------------------------------------------------------
# Target resolution
# ---------------------------------------------------------------------


@pytest.mark.parametrize(
    "argument_name",
    [
        "target",
        "object",
        "destination",
    ],
)
def test_resolve_target_accepts_supported_argument_names(
    argument_name,
):
    layer = _motion_layer()

    target = layer._resolve_target(
        primitive_step=_primitive(
            "pick",
            target="object_1",
            argument_name=argument_name,
        ),
        scene_state=_scene_state(
            _scene_object(
                "object_1"
            )
        ),
    )

    assert target.object_id == (
        "object_1"
    )


def test_resolve_target_is_case_insensitive_and_strips_whitespace():
    layer = _motion_layer()

    target = layer._resolve_target(
        primitive_step=_primitive(
            "pick",
            target="  OBJECT_1  ",
        ),
        scene_state=_scene_state(
            _scene_object(
                "object_1"
            )
        ),
    )

    assert target.object_id == (
        "object_1"
    )


def test_resolve_target_rejects_missing_object():
    layer = _motion_layer()

    with pytest.raises(
        ValueError,
        match="could not resolve target",
    ):
        layer._resolve_target(
            primitive_step=_primitive(
                "pick",
                target="missing",
            ),
            scene_state=_scene_state(
                _scene_object(
                    "available"
                )
            ),
        )


def test_resolve_target_rejects_ambiguous_object_ids():
    layer = _motion_layer()

    with pytest.raises(
        ValueError,
        match="found multiple SceneState objects",
    ):
        layer._resolve_target(
            primitive_step=_primitive(
                "pick",
                target="cube",
            ),
            scene_state=_scene_state(
                _scene_object(
                    "cube"
                ),
                _scene_object(
                    " CUBE "
                ),
            ),
        )


def test_extract_target_rejects_missing_target_argument():
    with pytest.raises(
        ValueError,
        match="does not contain a recognized target argument",
    ):
        SeeDoMotionLayer._extract_target_name(
            {}
        )


@pytest.mark.parametrize(
    "target",
    [
        1,
        1.5,
        None,
        [],
    ],
)
def test_extract_target_rejects_non_string_target(
    target,
):
    with pytest.raises(
        TypeError,
        match="Primitive target must be a string",
    ):
        SeeDoMotionLayer._extract_target_name(
            {
                "target": target,
            }
        )


@pytest.mark.parametrize(
    "target",
    [
        "",
        "   ",
    ],
)
def test_extract_target_rejects_empty_target(
    target,
):
    with pytest.raises(
        ValueError,
        match="Primitive target cannot be empty",
    ):
        SeeDoMotionLayer._extract_target_name(
            {
                "target": target,
            }
        )


def test_extract_target_prefers_target_over_other_aliases():
    target = SeeDoMotionLayer._extract_target_name(
        {
            "target": "a",
            "object": "b",
            "destination": "c",
        }
    )

    assert target == "a"


# ---------------------------------------------------------------------
# Motion history and artifacts
# ---------------------------------------------------------------------


def test_translate_records_motion_history():
    layer = _motion_layer()

    _reset(layer)

    actions = layer.translate(
        primitive_step=_primitive(
            "pick",
            target="target",
        ),
        scene_state=_scene_state(
            _scene_object(
                "target"
            )
        ),
    )

    assert len(
        layer.motion_history
    ) == 1

    history = (
        layer.motion_history[0]
    )

    assert history["name"] == (
        "pick"
    )

    assert history["arguments"] == {
        "target": "target",
    }

    assert len(
        history["actions"]
    ) == len(actions)

    assert history["actions"][0][
        "gripper_position"
    ] == pytest.approx(
        0.2
    )


def test_reset_with_artifacts_writes_initial_motion_plan(
    tmp_path,
):
    layer = _motion_layer()

    artifacts_dir = (
        tmp_path
        / "nested"
        / "motion"
    )

    _reset(
        layer,
        position=(
            0.1,
            0.2,
            0.3,
        ),
        artifacts_dir=artifacts_dir,
    )

    artifact_path = (
        artifacts_dir
        / "motion_plan.json"
    )

    assert artifact_path.is_file()

    with artifact_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        data = json.load(
            stream
        )

    assert data == {
        "primitives": [],
        "total_actions": 0,
        "final_planned_state": {
            "position": [
                0.1,
                0.2,
                0.3,
            ],
            "orientation": [
                0.0,
                0.0,
                0.0,
                1.0,
            ],
            "gripper_position": 0.8,
        },
    }


def test_translate_updates_motion_artifact(
    tmp_path,
):
    layer = _motion_layer()

    artifacts_dir = (
        tmp_path
        / "motion"
    )

    _reset(
        layer,
        artifacts_dir=artifacts_dir,
    )

    layer.translate(
        primitive_step=_primitive(
            "pick",
            target="target",
        ),
        scene_state=_scene_state(
            _scene_object(
                "target"
            )
        ),
    )

    artifact_path = (
        artifacts_dir
        / "motion_plan.json"
    )

    with artifact_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        data = json.load(
            stream
        )

    assert data["total_actions"] == 1

    assert len(
        data["primitives"]
    ) == 1

    assert data["primitives"][0][
        "name"
    ] == "pick"

    assert data["primitives"][0][
        "arguments"
    ] == {
        "target": "target",
    }

    assert data["final_planned_state"][
        "gripper_position"
    ] == pytest.approx(
        0.2
    )