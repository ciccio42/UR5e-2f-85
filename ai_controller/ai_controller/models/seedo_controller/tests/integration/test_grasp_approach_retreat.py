from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from ai_controller.models.seedo_controller.motion_layer import (
    SeeDoMotionLayer,
)

from results import (
    PrimitiveStep,
    SceneObject,
    SceneState,
)


# =====================================================================
# Configuration
# =====================================================================

GRASP_PLAN_PATH = Path(
    "/home/ros2_ws/src/UR5e-2f-85/"
    ".runtime/grasp_planner_test/grasp_plan.json"
)

PREGRASP_DISTANCE = 0.08
GRASP_DEPTH_OFFSET = 0.005
RETREAT_DISTANCE = 0.04
LIFT_HEIGHT = 0.15

MIN_STEP = 0.02

GRIPPER_OPEN_POSITION = 0.1
GRIPPER_CLOSED_POSITION = 0.8

LEGACY_GRASP_ORIENTATION = np.array(
    [
        0.9994452044624775,
        0.03161651380119412,
        0.0021438049655468088,
        0.010251021036213035,
    ],
    dtype=np.float64,
)


# =====================================================================
# Helpers
# =====================================================================

def _primitive(
    name: str,
) -> PrimitiveStep:
    return PrimitiveStep(
        name=name,
        arguments={
            "target": "target",
        },
        source_code=None,
    )


def _scene_state(
    position,
) -> SceneState:
    return SceneState(
        objects=(
            SceneObject(
                object_id="target",
                label="target",
                pixel_coordinates=(0, 0),
                position_camera=(0.0, 0.0, 0.0),
                position_base=tuple(position),
                category="object",
                attributes={},
            ),
        ),
    )


def _print_vector(
    name: str,
    value,
) -> None:
    value = np.asarray(
        value,
        dtype=np.float64,
    )

    print(
        f"{name:24s}: "
        f"[{value[0]: .6f}, "
        f"{value[1]: .6f}, "
        f"{value[2]: .6f}]"
    )


# =====================================================================
# Test
# =====================================================================

def test_grasp_approach_retreat_geometry():

    with GRASP_PLAN_PATH.open(
        "r",
        encoding="utf-8",
    ) as stream:
        grasp_plan = json.load(
            stream
        )

    tcp_position = np.asarray(
        grasp_plan["tcp_position_base"],
        dtype=np.float64,
    )

    tcp_orientation = np.asarray(
        grasp_plan["tcp_orientation_base"],
        dtype=np.float64,
    )

    approach_direction = np.asarray(
        grasp_plan["approach_direction_base"],
        dtype=np.float64,
    )

    # Motion Layer normalizes the direction, therefore compute
    # the expected geometry using the same normalized vector.
    approach_direction = (
        approach_direction
        / np.linalg.norm(
            approach_direction
        )
    )

    # -------------------------------------------------------------
    # Expected geometry
    # -------------------------------------------------------------

    expected_final_grasp = (
        tcp_position
        + GRASP_DEPTH_OFFSET
        * approach_direction
    )

    expected_pregrasp = (
        expected_final_grasp
        - PREGRASP_DISTANCE
        * approach_direction
    )

    expected_retreat = (
        expected_final_grasp
        - RETREAT_DISTANCE
        * approach_direction
    )

    expected_lift = (
        expected_retreat
        + np.array(
            [
                0.0,
                0.0,
                LIFT_HEIGHT,
            ],
            dtype=np.float64,
        )
    )

    # -------------------------------------------------------------
    # Motion Layer
    # -------------------------------------------------------------

    layer = SeeDoMotionLayer(
        grasp_orientation=(
            LEGACY_GRASP_ORIENTATION
        ),
        min_step=MIN_STEP,
        reach_hover_height=0.15,
        approach_z_offset=0.0,
        release_height_offset=0.10,
        lift_height=LIFT_HEIGHT,
        gripper_open_position=(
            GRIPPER_OPEN_POSITION
        ),
        gripper_closed_position=(
            GRIPPER_CLOSED_POSITION
        ),
        object_y_offset=-0.04,
        assembly_alignment_z_offset=0.135,
        assembly_insertion_z_offset=-0.11,
        grasp_planning_enabled=True,
        pregrasp_distance=(
            PREGRASP_DISTANCE
        ),
        grasp_depth_offset=(
            GRASP_DEPTH_OFFSET
        ),
        retreat_distance=(
            RETREAT_DISTANCE
        ),
    )

    # Start somewhere safely above and behind the grasp.
    initial_position = (
        expected_pregrasp
        + np.array(
            [
                -0.10,
                -0.10,
                0.20,
            ],
            dtype=np.float64,
        )
    )

    layer.reset(
        current_position=(
            initial_position
        ),
        current_orientation=(
            LEGACY_GRASP_ORIENTATION
        ),
    )

    layer.set_grasp_pose(
        position=tcp_position,
        orientation=tcp_orientation,
        approach_direction=(
            approach_direction
        ),
    )

    scene_state = _scene_state(
        tcp_position
    )

    # =============================================================
    # APPROACHING
    # =============================================================

    approaching_actions = layer.translate(
        primitive_step=_primitive(
            "approaching"
        ),
        scene_state=scene_state,
    )

    approaching_positions = np.asarray(
        [
            action[:3]
            for action in approaching_actions
        ]
    )

    # Pregrasp must actually appear as one of the generated
    # waypoints: it is the endpoint of phase A.
    pregrasp_distances = np.linalg.norm(
        approaching_positions
        - expected_pregrasp,
        axis=1,
    )

    pregrasp_index = int(
        np.argmin(
            pregrasp_distances
        )
    )

    assert (
        pregrasp_distances[
            pregrasp_index
        ]
        < 1e-9
    )

    # Final APPROACHING action must be the deeper grasp pose.
    np.testing.assert_allclose(
        approaching_actions[-1][:3],
        expected_final_grasp,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        approaching_actions[-1][3:7],
        tcp_orientation,
        atol=1e-9,
    )

    # Distance pregrasp -> final grasp must be exactly 8 cm.
    actual_approach_distance = np.linalg.norm(
        expected_final_grasp
        - expected_pregrasp
    )

    np.testing.assert_allclose(
        actual_approach_distance,
        PREGRASP_DISTANCE,
        atol=1e-9,
    )

    # Direction must be +approach_direction.
    actual_approach_direction = (
        expected_final_grasp
        - expected_pregrasp
    )

    actual_approach_direction /= (
        np.linalg.norm(
            actual_approach_direction
        )
    )

    np.testing.assert_allclose(
        actual_approach_direction,
        approach_direction,
        atol=1e-9,
    )

    # =============================================================
    # PICK + RETREAT
    # =============================================================

    pick_actions = layer.translate(
        primitive_step=_primitive(
            "pick"
        ),
        scene_state=scene_state,
    )

    # First PICK action closes the gripper at final grasp.
    np.testing.assert_allclose(
        pick_actions[0][:3],
        expected_final_grasp,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        pick_actions[0][7],
        GRIPPER_CLOSED_POSITION,
        atol=1e-9,
    )

    # Last PICK action is the end of the retreat.
    np.testing.assert_allclose(
        pick_actions[-1][:3],
        expected_retreat,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        pick_actions[-1][7],
        GRIPPER_CLOSED_POSITION,
        atol=1e-9,
    )

    actual_retreat_vector = (
        expected_retreat
        - expected_final_grasp
    )

    actual_retreat_distance = (
        np.linalg.norm(
            actual_retreat_vector
        )
    )

    np.testing.assert_allclose(
        actual_retreat_distance,
        RETREAT_DISTANCE,
        atol=1e-9,
    )

    actual_retreat_direction = (
        actual_retreat_vector
        / actual_retreat_distance
    )

    np.testing.assert_allclose(
        actual_retreat_direction,
        -approach_direction,
        atol=1e-9,
    )

    # =============================================================
    # LIFT UP
    # =============================================================

    lift_actions = layer.translate(
        primitive_step=_primitive(
            "lift_up"
        ),
        scene_state=scene_state,
    )

    np.testing.assert_allclose(
        lift_actions[-1][:3],
        expected_lift,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        lift_actions[-1][3:7],
        tcp_orientation,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        lift_actions[-1][7],
        GRIPPER_CLOSED_POSITION,
        atol=1e-9,
    )

    # =============================================================
    # Numerical report
    # =============================================================

    print()
    print(
        "=" * 78
    )
    print(
        "GRASP APPROACH / RETREAT GEOMETRY"
    )
    print(
        "=" * 78
    )

    _print_vector(
        "M2T2 TCP",
        tcp_position,
    )

    _print_vector(
        "Approach direction",
        approach_direction,
    )

    _print_vector(
        "Pregrasp",
        expected_pregrasp,
    )

    _print_vector(
        "Final grasp",
        expected_final_grasp,
    )

    _print_vector(
        "Retreat",
        expected_retreat,
    )

    _print_vector(
        "Lift",
        expected_lift,
    )

    print()
    print(
        "TCP -> final depth:",
        np.linalg.norm(
            expected_final_grasp
            - tcp_position
        ),
        "m",
    )

    print(
        "Pregrasp -> final:",
        actual_approach_distance,
        "m",
    )

    print(
        "Final -> retreat:",
        actual_retreat_distance,
        "m",
    )

    print(
        "Retreat -> lift:",
        np.linalg.norm(
            expected_lift
            - expected_retreat
        ),
        "m",
    )

    print(
        "=" * 78
    )