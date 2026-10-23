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
        "nut_assembly_config": {
            "peg_clearance_m": 0.020,
            "transport_clearance_margin_m": 0.050,
            "initial_entry_m": 0.005,
            "insertion_step_m": 0.005,
            "release_bottom_height_m": 0.010,
            "rotation_step_deg": 5.0,
            "auto_release": False,
        },
    }

    kwargs.update(
        overrides
    )

    return SeeDoMotionLayer(
        **kwargs
    )

NUT_TARGET = "gray_ring_0"
PEG_TARGET = "wooden_peg_2"


def _nut_scene_state() -> SceneState:
    """
    SceneState positions are intentionally NOT equal to the refined
    assembly geometry.

    This verifies that Nut-Assembly placement never falls back to
    SceneObject.position_base.
    """

    nut = _scene_object(
        NUT_TARGET,
        position_base=(
            0.20,
            0.40,
            0.12,
        ),
    )

    peg = _scene_object(
        PEG_TARGET,
        position_base=(
            0.62,
            0.38,
            0.25,
        ),
    )

    return _scene_state(
        nut,
        peg,
    )


def _install_test_nut_geometry(
    layer: SeeDoMotionLayer,
) -> None:
    """
    Install a fully deterministic assembly geometry.

    Planned grasp TCP:
        [0.20, 0.40, 0.12]

    Hole bottom at grasp:
        [0.23, 0.40, 0.07]

    Therefore:
        TCP -> hole_bottom = [0.03, 0.00, -0.05]

    Peg:
        support = [0.50, 0.30, 0.10]
        tip     = [0.50, 0.30, 0.30]
        axis    = +Z
    """

    layer.set_grasp_pose(
        position=(
            0.20,
            0.40,
            0.12,
        ),
        orientation=(
            0.0,
            0.0,
            0.0,
            1.0,
        ),
        approach_direction=(
            0.0,
            0.0,
            -1.0,
        ),
        backend_name="graspgenx",
    )

    layer.set_nut_place_refinements(
        {
            PEG_TARGET: {
                "accepted": True,

                "axis_xy_base": (
                    0.50,
                    0.30,
                ),

                "axis_direction_base": (
                    0.0,
                    0.0,
                    1.0,
                ),

                "peg_tip_point_base": (
                    0.50,
                    0.30,
                    0.30,
                ),

                "peg_support_point_base": (
                    0.50,
                    0.30,
                    0.10,
                ),
            }
        }
    )

    layer.set_nut_grasp_geometry(
        target_id=NUT_TARGET,

        hole_center_base=(
            0.23,
            0.40,
            0.09,
        ),

        hole_bottom_center_base=(
            0.23,
            0.40,
            0.07,
        ),

        hole_axis_base=(
            0.0,
            0.0,
            1.0,
        ),

        object_radius_from_hole_m=0.040,
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
    task_type="pick_and_place",
):
    layer.reset(
        current_position=position,
        current_orientation=orientation,
        artifacts_dir=artifacts_dir,
        task_type=task_type
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
        match="grasp_orientation must contain",
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
        approach_direction=(
            0.0,
            0.0,
            -1.0,
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

    assert (
        layer.active_approach_direction
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

    assert (
        layer.active_approach_direction
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
    layer = _motion_layer(
        grasp_planning_enabled=False,
    )
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


def test_approaching_uses_pregrasp_and_final_grasp(
    monkeypatch,
):
    layer = _motion_layer(
        pregrasp_distance=0.08,
        grasp_depth_offset=0.005,
    )

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

    approach_direction = np.array(
        [
            0.0,
            0.0,
            -1.0,
        ],
        dtype=np.float64,
    )

    layer.set_grasp_pose(
        position=grasp_position,
        orientation=grasp_orientation,
        approach_direction=approach_direction,
    )

    calls = []

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

        calls.append(
            {
                "position": position.copy(),
                "orientation": orientation.copy(),
            }
        )

        layer.current_pose = SimpleNamespace(
            position=position.copy(),
            orientation=orientation.copy(),
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

    actions = layer.translate(
        primitive_step=_primitive(
            "approaching"
        ),
        scene_state=_scene_state(
            _scene_object(
                "target"
            )
        ),
    )

    expected_final_grasp = (
        grasp_position
        + 0.005 * approach_direction
    )

    expected_pregrasp = (
        expected_final_grasp
        - 0.08 * approach_direction
    )

    # ---------------------------------------------------------
    # Two distinct motion phases
    # ---------------------------------------------------------

    assert len(calls) == 2

    # Phase A:
    # current pose -> pregrasp
    np.testing.assert_allclose(
        calls[0]["position"],
        expected_pregrasp,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        calls[0]["orientation"],
        grasp_orientation,
        atol=1e-9,
    )

    # Phase B:
    # pregrasp -> final grasp
    np.testing.assert_allclose(
        calls[1]["position"],
        expected_final_grasp,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        calls[1]["orientation"],
        grasp_orientation,
        atol=1e-9,
    )

    # Final approach must be exactly 8 cm along
    # the M2T2 approach direction.
    np.testing.assert_allclose(
        (
            calls[1]["position"]
            - calls[0]["position"]
        ),
        (
            0.08
            * approach_direction
        ),
        atol=1e-9,
    )

    # The fake generates one action per phase.
    assert len(actions) == 2

    # Motion Layer must finish at the deeper final grasp pose.
    np.testing.assert_allclose(
        layer.current_pose.position,
        expected_final_grasp,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        layer.current_pose.orientation,
        grasp_orientation,
        atol=1e-9,
    )

def test_approaching_uses_legacy_target_when_grasp_disabled(
    monkeypatch,
):
    layer = _motion_layer(
        grasp_planning_enabled=False,
    )

    _reset(layer)

    captured = {}

    _install_fake_move_linear(
        monkeypatch,
        layer,
        captured,
    )

    target_position = (
        0.4,
        0.5,
        0.6,
    )

    layer.translate(
        primitive_step=_primitive(
            "approaching"
        ),
        scene_state=_scene_state(
            _scene_object(
                "target",
                position_base=target_position,
            )
        ),
    )

    expected_position = np.array(
        [
            0.4,
            0.44,
            0.62,
        ],
        dtype=np.float64,
    )

    expected_orientation = np.array(
        [
            0.0,
            0.0,
            0.0,
            1.0,
        ],
        dtype=np.float64,
    )

    np.testing.assert_allclose(
        captured["target_position"],
        expected_position,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        captured["target_orientation"],
        expected_orientation,
        atol=1e-9,
    )

    assert layer.active_grasp_position is None
    assert layer.active_grasp_orientation is None

def test_approaching_requires_active_grasp_pose():
    layer = _motion_layer()
    _reset(layer)

    with pytest.raises(
        RuntimeError,
        match="active grasp pose or approach direction",
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
    layer = _motion_layer(
        grasp_planning_enabled=False,
    )

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

def test_pick_closes_gripper_and_retreats_when_grasp_enabled(
    monkeypatch,
):
    layer = _motion_layer(
        grasp_planning_enabled=True,
        retreat_distance=0.04,
    )

    _reset(
        layer,
        position=(
            0.2,
            0.4,
            0.1,
        ),
    )

    layer.active_approach_direction = np.array(
        [
            0.0,
            0.0,
            -1.0,
        ],
        dtype=np.float64,
    )

    captured = {}

    _install_fake_move_linear(
        monkeypatch,
        layer,
        captured,
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

    expected_retreat_position = np.array(
        [
            0.2,
            0.4,
            0.14,
        ],
        dtype=np.float64,
    )

    # First action: close gripper at grasp pose.
    assert len(actions) == 2

    np.testing.assert_allclose(
        actions[0][:3],
        np.array(
            [
                0.2,
                0.4,
                0.1,
            ]
        ),
        atol=1e-9,
    )

    assert actions[0][7] == pytest.approx(
        0.2
    )

    # Second action: retreat 4 cm along -approach_direction.
    np.testing.assert_allclose(
        captured["target_position"],
        expected_retreat_position,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        layer.current_pose.position,
        expected_retreat_position,
        atol=1e-9,
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
    layer = _motion_layer(
        grasp_planning_enabled=False,
    )

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
    layer = _motion_layer(
        grasp_planning_enabled=False,
    )

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

def test_graspgenx_approaching_uses_exact_tcp_pose(
    monkeypatch,
):
    layer = _motion_layer(
        grasp_planning_enabled=True,
        pregrasp_distance=0.08,
        grasp_depth_offset=0.005,
    )

    _reset(layer)

    tcp_position = np.array(
        [
            -0.10693327,
            0.46925381,
            -0.05868659,
        ],
        dtype=np.float64,
    )

    tcp_orientation = np.array(
        [
            0.81915205,
            -0.57357643,
            0.0,
            0.0,
        ],
        dtype=np.float64,
    )

    approach_direction = np.array(
        [
            0.0,
            0.0,
            -1.0,
        ],
        dtype=np.float64,
    )

    layer.set_grasp_pose(
        position=tcp_position,
        orientation=tcp_orientation,
        approach_direction=(
            approach_direction
        ),
        backend_name="graspgenx",
    )

    calls = []

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

        calls.append(
            {
                "position": position.copy(),
                "orientation": orientation.copy(),
            }
        )

        layer.current_pose = SimpleNamespace(
            position=position.copy(),
            orientation=orientation.copy(),
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

    actions = layer.translate(
        primitive_step=_primitive(
            "approaching"
        ),
        scene_state=_scene_state(
            _scene_object(
                "target"
            )
        ),
    )

    # GraspGen-X must perform exactly one motion:
    # current pose -> planner TCP.
    assert len(calls) == 1
    assert len(actions) == 1

    np.testing.assert_allclose(
        calls[0]["position"],
        tcp_position,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        calls[0]["orientation"],
        (
            tcp_orientation
            / np.linalg.norm(
                tcp_orientation
            )
        ),
        atol=1e-8,
    )

    np.testing.assert_allclose(
        layer.current_pose.position,
        tcp_position,
        atol=1e-9,
    )

    # Explicitly verify that the old M2T2
    # +5 mm depth offset was NOT applied.
    old_m2t2_final = (
        tcp_position
        + 0.005
        * approach_direction
    )

    assert not np.allclose(
        layer.current_pose.position,
        old_m2t2_final,
    )

    assert (
        layer.active_grasp_backend
        == "graspgenx"
    )


def test_graspgenx_pick_closes_without_retreat(
    monkeypatch,
):
    layer = _motion_layer(
        grasp_planning_enabled=True,
        retreat_distance=0.04,
    )

    tcp_position = np.array(
        [
            -0.10693327,
            0.46925381,
            -0.05868659,
        ],
        dtype=np.float64,
    )

    tcp_orientation = np.array(
        [
            0.81915205,
            -0.57357643,
            0.0,
            0.0,
        ],
        dtype=np.float64,
    )

    approach_direction = np.array(
        [
            0.0,
            0.0,
            -1.0,
        ],
        dtype=np.float64,
    )

    _reset(
        layer,
        position=tcp_position,
        orientation=tcp_orientation,
    )

    layer.set_grasp_pose(
        position=tcp_position,
        orientation=tcp_orientation,
        approach_direction=(
            approach_direction
        ),
        backend_name="graspgenx",
    )

    def forbidden_move_linear(
        *,
        target_position,
        target_orientation,
    ):
        raise AssertionError(
            "GraspGen-X PICK must not perform "
            "the M2T2 retreat motion."
        )

    monkeypatch.setattr(
        layer,
        "_move_linear",
        forbidden_move_linear,
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

    # PICK must only close the gripper.
    assert len(actions) == 1

    np.testing.assert_allclose(
        actions[0][:3],
        tcp_position,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        actions[0][3:7],
        (
            tcp_orientation
            / np.linalg.norm(
                tcp_orientation
            )
        ),
        atol=1e-8,
    )

    assert (
        actions[0][7]
        == pytest.approx(
            layer.gripper_closed_position
        )
    )

    np.testing.assert_allclose(
        layer.current_pose.position,
        tcp_position,
        atol=1e-9,
    )

def test_nut_grasp_geometry_is_frozen_in_tcp_frame():
    layer = _motion_layer()

    _reset(
        layer,
        position=(
            0.20,
            0.40,
            0.12,
        ),
        orientation=(
            0.0,
            0.0,
            0.0,
            1.0,
        ),
        task_type="nut_assembly",
    )

    _install_test_nut_geometry(
        layer
    )

    np.testing.assert_allclose(
        layer._nut_hole_top_local,
        np.array(
            [
                0.03,
                0.00,
                -0.03,
            ]
        ),
        atol=1e-9,
    )

    np.testing.assert_allclose(
        layer._nut_hole_bottom_local,
        np.array(
            [
                0.03,
                0.00,
                -0.05,
            ]
        ),
        atol=1e-9,
    )

    np.testing.assert_allclose(
        layer._nut_hole_axis_local,
        np.array(
            [
                0.0,
                0.0,
                1.0,
            ]
        ),
        atol=1e-9,
    )

    expected_sweep_radius = (
        max(
            np.linalg.norm(
                np.array(
                    [
                        0.03,
                        0.0,
                        -0.03,
                    ]
                )
            ),
            np.linalg.norm(
                np.array(
                    [
                        0.03,
                        0.0,
                        -0.05,
                    ]
                )
            ),
        )
        + 0.040
    )

    assert (
        layer._nut_object_sweep_radius_m
        == pytest.approx(
            expected_sweep_radius,
            abs=1e-9,
        )
    )

    # Simulate reaching the planned grasp TCP.
    layer.current_pose = SimpleNamespace(
        position=np.array(
            [
                0.20,
                0.40,
                0.12,
            ],
            dtype=np.float64,
        ),
        orientation=np.array(
            [
                0.0,
                0.0,
                0.0,
                1.0,
            ],
            dtype=np.float64,
        ),
    )

    (
        hole_top,
        hole_bottom,
        hole_axis,
    ) = layer._nut_current_hole_geometry()

    np.testing.assert_allclose(
        hole_top,
        np.array(
            [
                0.23,
                0.40,
                0.09,
            ]
        ),
        atol=1e-9,
    )

    np.testing.assert_allclose(
        hole_bottom,
        np.array(
            [
                0.23,
                0.40,
                0.07,
            ]
        ),
        atol=1e-9,
    )

    np.testing.assert_allclose(
        hole_axis,
        np.array(
            [
                0.0,
                0.0,
                1.0,
            ]
        ),
        atol=1e-9,
    )

def test_legacy_nut_grasp_geometry_uses_measured_tcp_and_allows_lift():
    layer = _motion_layer(
        grasp_planning_enabled=False,
        lift_height=0.15,
    )

    scene_state = _nut_scene_state()

    # Initial Motion Layer state before the legacy grasp.
    _reset(
        layer,
        position=(
            0.20,
            0.40,
            0.20,
        ),
        orientation=(
            0.0,
            0.0,
            0.0,
            1.0,
        ),
        task_type="nut_assembly",
    )

    # Nut-Assembly still needs the refined destination geometry.
    layer.set_nut_place_refinements(
        {
            PEG_TARGET: {
                "accepted": True,

                "axis_xy_base": (
                    0.50,
                    0.30,
                ),

                "axis_direction_base": (
                    0.0,
                    0.0,
                    1.0,
                ),

                "peg_tip_point_base": (
                    0.50,
                    0.30,
                    0.30,
                ),

                "peg_support_point_base": (
                    0.50,
                    0.30,
                    0.10,
                ),
            }
        }
    )

    # This represents the REAL TCP measured after the legacy
    # APPROACHING primitive has physically completed.
    measured_tcp_position = np.array(
        [
            0.20,
            0.40,
            0.12,
        ],
        dtype=np.float64,
    )

    measured_tcp_orientation = np.array(
        [
            0.0,
            0.0,
            0.0,
            1.0,
        ],
        dtype=np.float64,
    )

    # No GraspPlanner pose is installed.
    assert layer.active_grasp_position is None
    assert layer.active_grasp_orientation is None
    assert layer.active_approach_direction is None

    layer.set_nut_grasp_geometry(
        target_id=NUT_TARGET,

        hole_center_base=(
            0.23,
            0.40,
            0.09,
        ),

        hole_bottom_center_base=(
            0.23,
            0.40,
            0.07,
        ),

        hole_axis_base=(
            0.0,
            0.0,
            1.0,
        ),

        object_radius_from_hole_m=0.040,

        reference_tcp_position=(
            measured_tcp_position
        ),

        reference_tcp_orientation=(
            measured_tcp_orientation
        ),

        synchronize_current_pose=True,
    )

    # The legacy path must not fabricate a planned grasp.
    assert layer.active_grasp_position is None
    assert layer.active_grasp_orientation is None
    assert layer.active_approach_direction is None

    # Hole geometry must be frozen in the measured TCP frame.
    np.testing.assert_allclose(
        layer._nut_hole_top_local,
        np.array(
            [
                0.03,
                0.00,
                -0.03,
            ]
        ),
        atol=1e-9,
    )

    np.testing.assert_allclose(
        layer._nut_hole_bottom_local,
        np.array(
            [
                0.03,
                0.00,
                -0.05,
            ]
        ),
        atol=1e-9,
    )

    np.testing.assert_allclose(
        layer._nut_hole_axis_local,
        np.array(
            [
                0.0,
                0.0,
                1.0,
            ]
        ),
        atol=1e-9,
    )

    # Motion Layer must also start subsequent primitives from
    # the REAL measured TCP pose.
    np.testing.assert_allclose(
        layer.current_pose.position,
        measured_tcp_position,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        layer.current_pose.orientation,
        measured_tcp_orientation,
        atol=1e-9,
    )

    # Simulate PICK completion.
    layer.current_gripper_position = (
        layer.gripper_closed_position
    )

    # Most importantly: LIFT_UP must now work even though
    # grasp_planning_enabled=False.
    lift_actions = layer.translate(
        primitive_step=_primitive(
            "lift_up",
            target=NUT_TARGET,
        ),
        scene_state=scene_state,
    )

    assert lift_actions

    assert (
        layer._nut_phase
        == "lifted"
    )

    assert (
        layer.current_pose.position[2]
        > measured_tcp_position[2]
    )

def test_rotation_between_vectors_aligns_source_with_target():
    source = np.array(
        [
            1.0,
            0.0,
            0.0,
        ],
        dtype=np.float64,
    )

    target = np.array(
        [
            0.0,
            0.0,
            1.0,
        ],
        dtype=np.float64,
    )

    rotation = (
        SeeDoMotionLayer
        ._rotation_between_vectors(
            source,
            target,
        )
    )

    rotated = rotation.apply(
        source
    )

    np.testing.assert_allclose(
        rotated,
        target,
        atol=1e-9,
    )

    assert (
        rotation.magnitude()
        == pytest.approx(
            np.pi / 2.0,
            abs=1e-9,
        )
    )

def test_rotation_between_vectors_handles_antiparallel_axes():
    source = np.array(
        [
            0.0,
            0.0,
            -1.0,
        ],
        dtype=np.float64,
    )

    target = np.array(
        [
            0.0,
            0.0,
            1.0,
        ],
        dtype=np.float64,
    )

    rotation = (
        SeeDoMotionLayer
        ._rotation_between_vectors(
            source,
            target,
        )
    )

    np.testing.assert_allclose(
        rotation.apply(source),
        target,
        atol=1e-9,
    )

    assert (
        rotation.magnitude()
        == pytest.approx(
            np.pi,
            abs=1e-9,
        )
    )

def test_complete_nut_assembly_geometry_is_numerically_consistent():
    layer = _motion_layer(
        min_step=0.01,
        lift_height=0.15,
    )

    scene_state = (
        _nut_scene_state()
    )

    _reset(
        layer,
        position=(
            0.20,
            0.40,
            0.12,
        ),
        orientation=(
            0.0,
            0.0,
            0.0,
            1.0,
        ),
        task_type="nut_assembly",
    )

    _install_test_nut_geometry(
        layer
    )

    # The test starts immediately after a successful PICK.
    layer.current_gripper_position = (
        layer.gripper_closed_position
    )

    # ============================================================
    # LIFT_UP
    # ============================================================

    lift_actions = layer.translate(
        primitive_step=_primitive(
            "lift_up",
            target=NUT_TARGET,
        ),
        scene_state=scene_state,
    )

    assert lift_actions

    expected_hole_bottom_offset_z = -0.050

    expected_required_clearance = (
        0.020  # peg_clearance_m
        + 0.050  # transport_clearance_margin_m
    )

    expected_safe_tcp_z = (
        0.30  # highest peg tip Z
        + expected_required_clearance
        - expected_hole_bottom_offset_z
    )

    expected_nominal_lift_z = (
        0.12  # TCP Z before LIFT_UP
        + 0.15  # lift_height
    )

    expected_lift_z = max(
        expected_nominal_lift_z,
        expected_safe_tcp_z,
    )

    assert (
        layer.current_pose.position[2]
        == pytest.approx(
            expected_lift_z,
            abs=1e-9,
        )
    )

    # ============================================================
    # MOVING
    # ============================================================

    moving_actions = layer.translate(
        primitive_step=_primitive(
            "moving",
            target=PEG_TARGET,
        ),
        scene_state=scene_state,
    )

    assert moving_actions

    expected_moving_tcp = np.array(
        [
            0.47,
            0.30,
            expected_lift_z,
        ],
        dtype=np.float64,
    )

    np.testing.assert_allclose(
        layer.current_pose.position,
        expected_moving_tcp,
        atol=1e-9,
    )

    (
        _,
        moving_hole_bottom,
        moving_hole_axis,
    ) = layer._nut_current_hole_geometry()

    # Hole must be exactly on the refined peg axis.
    np.testing.assert_allclose(
        moving_hole_bottom[:2],
        np.array(
            [
                0.50,
                0.30,
            ]
        ),
        atol=1e-9,
    )

    np.testing.assert_allclose(
        moving_hole_axis,
        np.array(
            [
                0.0,
                0.0,
                1.0,
            ]
        ),
        atol=1e-9,
    )

    # IMPORTANT:
    # SceneState deliberately says that the peg is elsewhere.
    scene_peg = next(
        obj
        for obj in scene_state.objects
        if obj.object_id == PEG_TARGET
    )

    assert not np.allclose(
        moving_hole_bottom[:2],
        np.asarray(
            scene_peg.position_base[:2]
        ),
        atol=1e-6,
    )

    # ============================================================
    # ALIGNING
    # ============================================================

    aligning_actions = layer.translate(
        primitive_step=_primitive(
            "aligning",
            target=PEG_TARGET,
        ),
        scene_state=scene_state,
    )

    assert aligning_actions

    (
        _,
        aligned_hole_bottom,
        aligned_hole_axis,
    ) = layer._nut_current_hole_geometry()

    expected_aligned_hole_bottom = (
        np.array(
            [
                0.50,
                0.30,
                0.30,
            ],
            dtype=np.float64,
        )
        + np.array(
            [
                0.0,
                0.0,
                0.020,
            ],
            dtype=np.float64,
        )
    )

    np.testing.assert_allclose(
        aligned_hole_bottom,
        expected_aligned_hole_bottom,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        aligned_hole_axis,
        np.array(
            [
                0.0,
                0.0,
                1.0,
            ]
        ),
        atol=1e-9,
    )

    np.testing.assert_allclose(
        layer.current_pose.position,
        np.array(
            [
                0.47,
                0.30,
                0.37,
            ]
        ),
        atol=1e-9,
    )

    # ============================================================
    # INSERTING
    # ============================================================

    inserting_actions = layer.translate(
        primitive_step=_primitive(
            "inserting",
            target=PEG_TARGET,
        ),
        scene_state=scene_state,
    )

    assert inserting_actions

    (
        _,
        final_hole_bottom,
        final_hole_axis,
    ) = layer._nut_current_hole_geometry()

    expected_final_hole_bottom = np.array(
        [
            0.50,
            0.30,
            0.11,
        ],
        dtype=np.float64,
    )

    np.testing.assert_allclose(
        final_hole_bottom,
        expected_final_hole_bottom,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        final_hole_axis,
        np.array(
            [
                0.0,
                0.0,
                1.0,
            ]
        ),
        atol=1e-9,
    )

    np.testing.assert_allclose(
        layer.current_pose.position,
        np.array(
            [
                0.47,
                0.30,
                0.16,
            ]
        ),
        atol=1e-9,
    )

    # auto_release=False in the test configuration.
    assert (
        layer.current_gripper_position
        == pytest.approx(
            layer.gripper_closed_position
        )
    )

    assert (
        layer._nut_phase
        == "inserted"
    )