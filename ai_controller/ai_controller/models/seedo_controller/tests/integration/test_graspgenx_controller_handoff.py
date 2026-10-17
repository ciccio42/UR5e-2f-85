from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from ai_controller.models.seedo_controller.motion_layer import (
    SeeDoMotionLayer,
)
from ai_controller.models.seedo_controller.seedo_controller import (
    SeeDoController,
)
from ai_controller.utils.grasping.graspgenx.planner import (
    GraspGenXPlanResult,
)

from results import (
    ActionPlanningResult,
    ActionStep,
    PrimitivePlan,
    PrimitiveStep,
    SceneObject,
    SceneState,
)


TARGET_ID = "gray_ring_0"

TCP_POSITION = np.array(
    [
        -0.10693327,
        0.46925381,
        -0.05868659,
    ],
    dtype=np.float64,
)

TCP_ORIENTATION = np.array(
    [
        0.81915205,
        -0.57357643,
        0.0,
        0.0,
    ],
    dtype=np.float64,
)

APPROACH_DIRECTION = np.array(
    [
        0.0,
        0.0,
        -1.0,
    ],
    dtype=np.float64,
)

SEMANTIC_POINT = np.array(
    [
        269.47198486,
        216.57600403,
    ],
    dtype=np.float32,
)

CONFIDENCE = 0.6277044415473938

CURRENT_POSITION = np.array(
    [
        -0.04707974,
        0.43916608,
        0.08374979,
    ],
    dtype=np.float64,
)

CURRENT_ORIENTATION = np.array(
    [
        0.99923116,
        0.03764141,
        -0.00260986,
        0.01064924,
    ],
    dtype=np.float64,
)

NUT_TARGET_ID = "gray_ring_0"
PEG_TARGET_ID = "wooden_peg_2"

PEG_AXIS_BASE = np.array(
    [
        0.0,
        0.0,
        1.0,
    ],
    dtype=np.float64,
)

# Coherent with the real setup:
# table_0 around z=-0.12 m, peg approximately 20 cm high.
#
# IMPORTANT:
# these are NOT configuration values used by production.
# They are frozen refinement outputs used only by this test.
PEG_SUPPORT_BASE = np.array(
    [
        0.06170433,
        0.38406133,
        -0.12000000,
    ],
    dtype=np.float64,
)

PEG_TIP_BASE = np.array(
    [
        0.06170433,
        0.38406133,
        0.08000000,
    ],
    dtype=np.float64,
)

# Geometry of the ring relative to the REAL GraspGen-X TCP.
#
# The current real grasp quaternion turns local -Z into base +Z.
HOLE_BOTTOM_LOCAL = np.array(
    [
        0.030,
        0.000,
        -0.030,
    ],
    dtype=np.float64,
)

HOLE_TOP_LOCAL = np.array(
    [
        0.030,
        0.000,
        -0.050,
    ],
    dtype=np.float64,
)

HOLE_AXIS_LOCAL = np.array(
    [
        0.0,
        0.0,
        -1.0,
    ],
    dtype=np.float64,
)

OBJECT_RADIUS_FROM_HOLE_M = 0.040

class FakeGraspGenXPlanner:
    backend_name = "graspgenx"

    def __init__(
        self,
        result: GraspGenXPlanResult,
    ) -> None:
        self.result = result
        self.calls = []

    def plan(
        self,
        **kwargs,
    ) -> GraspGenXPlanResult:
        self.calls.append(
            kwargs
        )

        return self.result

def _make_nut_action_plan() -> ActionPlanningResult:
    step = ActionStep(
        pick_keyframe=0,
        place_keyframe=1,

        picked_track_id=0,
        picked_category="ring",
        picked_color="gray",

        destination_track_id=1,
        destination_category="peg",
        destination_ordinal_from_left=0,

        relation="on",

        action=(
            "Pick the gray ring and insert it "
            "onto the destination peg."
        ),

        picked_detector_label="ring",

        grasp_instruction=(
            "Pick up the gray ring by grasping "
            "its handle, not the circular ring body."
        ),
    )

    return ActionPlanningResult(
        steps=(
            step,
        ),
        status="completed",
        ambiguities=(),
        natural_language_plan=step.action,
        task_type="nut_assembly",
    )

def _make_nut_scene_state() -> SceneState:

    ring = SceneObject(
        object_id=NUT_TARGET_ID,
        label="ring",
        pixel_coordinates=(
            269,
            217,
        ),
        position_camera=(
            -0.0347,
            0.0317,
            0.2889,
        ),
        position_base=tuple(
            TCP_POSITION
        ),
        category="ring",
        attributes={
            "color": "gray",
        },
    )

    # Deliberately WRONG geometric center.
    #
    # Placement must use the refinement below,
    # never this SceneState centroid.
    peg = SceneObject(
        object_id=PEG_TARGET_ID,
        label="peg",
        pixel_coordinates=(
            400,
            190,
        ),
        position_camera=(
            0.0,
            0.0,
            0.4,
        ),
        position_base=(
            0.30,
            0.60,
            0.20,
        ),
        category="peg",
        attributes={},
    )

    return SceneState(
        objects=(
            ring,
            peg,
        )
    )

def _make_nut_primitive_plan() -> PrimitivePlan:
    return PrimitivePlan(
        steps=(
            PrimitiveStep(
                name="reach",
                arguments={
                    "target": NUT_TARGET_ID,
                },
            ),
            PrimitiveStep(
                name="approaching",
                arguments={
                    "target": NUT_TARGET_ID,
                },
            ),
            PrimitiveStep(
                name="pick",
                arguments={
                    "target": NUT_TARGET_ID,
                },
            ),
            PrimitiveStep(
                name="lift_up",
                arguments={
                    "target": NUT_TARGET_ID,
                },
            ),
            PrimitiveStep(
                name="moving",
                arguments={
                    "target": PEG_TARGET_ID,
                },
            ),
            PrimitiveStep(
                name="aligning",
                arguments={
                    "target": PEG_TARGET_ID,
                },
            ),
            PrimitiveStep(
                name="inserting",
                arguments={
                    "target": PEG_TARGET_ID,
                },
            ),
        ),
        source_code="",
    )

def _make_nut_place_refinements():
    return {
        PEG_TARGET_ID: {
            "accepted": True,

            "axis_xy_base": tuple(
                PEG_TIP_BASE[:2]
            ),

            "axis_point_base": tuple(
                PEG_TIP_BASE
            ),

            "axis_direction_base": tuple(
                PEG_AXIS_BASE
            ),

            "peg_tip_point_base": tuple(
                PEG_TIP_BASE
            ),

            "peg_support_point_base": tuple(
                PEG_SUPPORT_BASE
            ),

            "peg_height_estimate_m": float(
                np.linalg.norm(
                    PEG_TIP_BASE
                    - PEG_SUPPORT_BASE
                )
            ),
        }
    }

def _make_nut_pick_refinements():

    grasp_rotation = Rotation.from_quat(
        TCP_ORIENTATION
        / np.linalg.norm(
            TCP_ORIENTATION
        )
    )

    hole_bottom_base = (
        TCP_POSITION
        + grasp_rotation.apply(
            HOLE_BOTTOM_LOCAL
        )
    )

    hole_top_base = (
        TCP_POSITION
        + grasp_rotation.apply(
            HOLE_TOP_LOCAL
        )
    )

    hole_axis_base = (
        grasp_rotation.apply(
            HOLE_AXIS_LOCAL
        )
    )

    return {
        NUT_TARGET_ID: {
            "accepted": True,

            "hole_center_base": tuple(
                hole_top_base
            ),

            "hole_bottom_center_base": tuple(
                hole_bottom_base
            ),

            "hole_axis_base": tuple(
                hole_axis_base
            ),

            "nut_thickness_estimate_m": 0.020,

            "object_radius_from_hole_m": (
                OBJECT_RADIUS_FROM_HOLE_M
            ),
        }
    }

def _make_nut_motion_layer() -> SeeDoMotionLayer:
    return SeeDoMotionLayer(
        grasp_orientation=np.array(
            [
                0.9994452044624775,
                0.03161651380119412,
                0.0021438049655468088,
                0.010251021036213035,
            ],
            dtype=np.float64,
        ),

        min_step=0.02,
        reach_hover_height=0.18,
        approach_z_offset=0.0,
        release_height_offset=0.10,
        lift_height=0.15,

        gripper_open_position=0.1,
        gripper_closed_position=0.8,

        object_y_offset=-0.04,

        grasp_planning_enabled=True,

        pregrasp_distance=0.08,
        grasp_depth_offset=0.005,
        retreat_distance=0.04,

        nut_assembly_config={
            "peg_clearance_m": 0.020,
            "initial_entry_m": 0.005,
            "insertion_step_m": 0.005,
            "release_bottom_height_m": 0.010,
            "rotation_step_deg": 5.0,
            "auto_release": False,
        },
    )

def _make_nut_controller(
    tmp_path: Path,
) -> tuple[
    SeeDoController,
    FakeGraspGenXPlanner,
]:

    result = GraspGenXPlanResult(
        grasp_instruction=(
            "Pick up the gray ring by grasping "
            "its handle, not the circular ring body."
        ),

        semantic_point_px=(
            SEMANTIC_POINT.copy()
        ),

        tcp_position_base=(
            TCP_POSITION.copy()
        ),

        tcp_orientation_base=(
            TCP_ORIENTATION.copy()
        ),

        approach_direction_base=(
            APPROACH_DIRECTION.copy()
        ),

        confidence=CONFIDENCE,
    )

    planner = FakeGraspGenXPlanner(
        result
    )

    controller = SeeDoController.__new__(
        SeeDoController
    )

    controller.action_plan = (
        _make_nut_action_plan()
    )

    controller.scene_state = (
        _make_nut_scene_state()
    )

    controller.primitive_plan = (
        _make_nut_primitive_plan()
    )

    controller.perception_mode = (
        "prior_guided"
    )

    controller.grasp_planning_enabled = True
    controller.grasp_planner = planner

    controller.motion_layer = (
        _make_nut_motion_layer()
    )

    controller.nut_place_refinements = (
        _make_nut_place_refinements()
    )

    controller.nut_pick_refinements = (
        _make_nut_pick_refinements()
    )

    controller.artifacts_dir = (
        tmp_path
        / "nut_handoff"
    )

    controller.artifacts_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    controller.execution_status = (
        "plan_ready"
    )

    controller.execution_error = None

    controller.grasp_plan = None
    controller.grasp_plan_target = None

    return (
        controller,
        planner,
    )

def _make_action_plan() -> ActionPlanningResult:
    step = ActionStep(
        pick_keyframe=0,
        place_keyframe=1,
        picked_track_id=0,
        picked_category="ring",
        picked_color="gray",
        destination_track_id=1,
        destination_category="storage bin",
        destination_ordinal_from_left=0,
        relation="inside",
        action=(
            "Pick the gray ring and place it "
            "in the storage bin."
        ),
        picked_detector_label="ring",
        grasp_instruction=(
            "Pick up the gray ring by grasping "
            "its handle, not the circular ring body."
        ),
    )

    return ActionPlanningResult(
        steps=(
            step,
        ),
        status="completed",
        ambiguities=(),
        natural_language_plan=step.action,
        task_type="pick_and_place",
    )


def _make_scene_state() -> SceneState:
    target = SceneObject(
        object_id=TARGET_ID,
        label="ring",
        pixel_coordinates=(
            269,
            217,
        ),
        position_camera=(
            -0.0347,
            0.0317,
            0.2889,
        ),
        position_base=tuple(
            TCP_POSITION
        ),
        category="ring",
        attributes={
            "color": "gray",
        },
    )

    return SceneState(
        objects=(
            target,
        )
    )


def _make_primitive_plan() -> PrimitivePlan:
    return PrimitivePlan(
        steps=(
            PrimitiveStep(
                name="reach",
                arguments={
                    "target": TARGET_ID,
                },
            ),
            PrimitiveStep(
                name="approaching",
                arguments={
                    "target": TARGET_ID,
                },
            ),
            PrimitiveStep(
                name="pick",
                arguments={
                    "target": TARGET_ID,
                },
            ),
            PrimitiveStep(
                name="lift_up",
                arguments={
                    "target": TARGET_ID,
                },
            ),
        ),
        source_code="",
    )


def _make_motion_layer() -> SeeDoMotionLayer:
    return SeeDoMotionLayer(
        grasp_orientation=np.array(
            [
                0.9994452044624775,
                0.03161651380119412,
                0.0021438049655468088,
                0.010251021036213035,
            ],
            dtype=np.float64,
        ),
        min_step=0.02,
        reach_hover_height=0.18,
        approach_z_offset=0.0,
        release_height_offset=0.10,
        lift_height=0.15,
        gripper_open_position=0.1,
        gripper_closed_position=0.8,
        object_y_offset=-0.04,
        grasp_planning_enabled=True,

        # These M2T2 values deliberately remain non-zero.
        # The test verifies that GraspGen-X ignores them.
        pregrasp_distance=0.08,
        grasp_depth_offset=0.005,
        retreat_distance=0.04,
    )


def _make_controller(
    tmp_path: Path,
) -> tuple[
    SeeDoController,
    FakeGraspGenXPlanner,
]:

    result = GraspGenXPlanResult(
        grasp_instruction=(
            "Pick up the gray ring by grasping "
            "its handle, not the circular ring body."
        ),
        semantic_point_px=(
            SEMANTIC_POINT.copy()
        ),
        tcp_position_base=(
            TCP_POSITION.copy()
        ),
        tcp_orientation_base=(
            TCP_ORIENTATION.copy()
        ),
        approach_direction_base=(
            APPROACH_DIRECTION.copy()
        ),
        confidence=CONFIDENCE,
    )

    planner = FakeGraspGenXPlanner(
        result
    )

    # We intentionally bypass heavyweight model initialization.
    # This test targets only the runtime controller handoff.
    controller = SeeDoController.__new__(
        SeeDoController
    )

    controller.action_plan = (
        _make_action_plan()
    )

    controller.scene_state = (
        _make_scene_state()
    )

    controller.primitive_plan = (
        _make_primitive_plan()
    )

    controller.perception_mode = (
        "prior_guided"
    )

    controller.grasp_planning_enabled = True
    controller.grasp_planner = planner

    controller.motion_layer = (
        _make_motion_layer()
    )

    controller.artifacts_dir = (
        tmp_path
        / "handoff"
    )

    controller.artifacts_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    controller.execution_status = "plan_ready"
    controller.execution_error = None

    controller.grasp_plan = None
    controller.grasp_plan_target = None

    return (
        controller,
        planner,
    )


def _robot_state() -> dict:
    return {
        "eef_pos": (
            CURRENT_POSITION.copy()
        ),
        "eef_quat": (
            CURRENT_ORIENTATION.copy()
        ),
    }


def _grasp_input() -> dict:
    return {
        "rgb_image": np.zeros(
            (
                376,
                672,
                3,
            ),
            dtype=np.uint8,
        ),
        "depth_image": np.full(
            (
                376,
                672,
            ),
            0.29,
            dtype=np.float32,
        ),
        "camera_matrix": np.array(
            [
                [
                    363.8071594238281,
                    0.0,
                    335.86553955078125,
                ],
                [
                    0.0,
                    363.8071594238281,
                    183.81210327148438,
                ],
                [
                    0.0,
                    0.0,
                    1.0,
                ],
            ],
            dtype=np.float64,
        ),
        "T_base_camera": np.eye(
            4,
            dtype=np.float64,
        ),
        "T_base_table": np.eye(
            4,
            dtype=np.float64,
        ),
    }


def test_graspgenx_controller_motion_handoff(
    tmp_path,
):
    (
        controller,
        planner,
    ) = _make_controller(
        tmp_path
    )

    # ============================================================
    # t=1: REACH
    #
    # Initializes the real Motion Layer exactly as production does.
    # ============================================================

    reach_actions = controller.inference(
        input_data={
            "robot_state": _robot_state(),
        },
        t=1,
    )

    assert reach_actions

    assert (
        controller.motion_layer.current_pose
        is not None
    )

    # Planner must not run during REACH.
    assert len(
        planner.calls
    ) == 0

    # ============================================================
    # t=2: APPROACHING
    #
    # This is the actual SeeDoController grasp-planning handoff.
    # ============================================================

    grasp_input = _grasp_input()

    approaching_actions = (
        controller.inference(
            input_data={
                "robot_state": (
                    _robot_state()
                ),
                "grasp_input": (
                    grasp_input
                ),
            },
            t=2,
        )
    )

    # Planner called exactly once.
    assert len(
        planner.calls
    ) == 1

    call = planner.calls[0]

    assert (
        call["grasp_instruction"]
        ==
        "Pick up the gray ring by grasping "
        "its handle, not the circular ring body."
    )

    np.testing.assert_allclose(
        call[
            "current_tcp_position"
        ],
        CURRENT_POSITION,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        call[
            "current_tcp_orientation"
        ],
        CURRENT_ORIENTATION,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        call["camera_matrix"],
        grasp_input[
            "camera_matrix"
        ],
    )

    # ============================================================
    # Controller -> Motion Layer handoff
    # ============================================================

    assert (
        controller.motion_layer
        .active_grasp_backend
        == "graspgenx"
    )

    np.testing.assert_allclose(
        controller.motion_layer
        .active_grasp_position,
        TCP_POSITION,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        controller.motion_layer
        .active_approach_direction,
        APPROACH_DIRECTION,
        atol=1e-9,
    )

    normalized_orientation = (
        TCP_ORIENTATION
        / np.linalg.norm(
            TCP_ORIENTATION
        )
    )

    np.testing.assert_allclose(
        controller.motion_layer
        .active_grasp_orientation,
        normalized_orientation,
        atol=1e-8,
    )

    assert (
        controller.grasp_plan_target
        == TARGET_ID
    )

    # ============================================================
    # GraspGen-X APPROACHING behavior
    #
    # Final waypoint must be EXACT planner TCP.
    # ============================================================

    assert approaching_actions

    approaching_final = (
        approaching_actions[-1]
    )

    np.testing.assert_allclose(
        approaching_final[:3],
        TCP_POSITION,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        approaching_final[3:7],
        normalized_orientation,
        atol=1e-8,
    )

    np.testing.assert_allclose(
        approaching_final[7],
        0.1,
        atol=1e-9,
    )

    # Explicitly prove that M2T2's +5 mm depth offset
    # was NOT applied.
    m2t2_final = (
        TCP_POSITION
        + 0.005
        * APPROACH_DIRECTION
    )

    assert not np.allclose(
        approaching_final[:3],
        m2t2_final,
        atol=1e-9,
    )

    # ============================================================
    # t=3: PICK
    #
    # GraspGen-X must close exactly at TCP and NOT retreat.
    # ============================================================

    pick_actions = controller.inference(
        input_data={},
        t=3,
    )

    assert len(
        pick_actions
    ) == 1

    pick_action = pick_actions[0]

    np.testing.assert_allclose(
        pick_action[:3],
        TCP_POSITION,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        pick_action[3:7],
        normalized_orientation,
        atol=1e-8,
    )

    np.testing.assert_allclose(
        pick_action[7],
        0.8,
        atol=1e-9,
    )

    # No M2T2 retreat.
    np.testing.assert_allclose(
        controller.motion_layer
        .current_pose.position,
        TCP_POSITION,
        atol=1e-9,
    )

    # ============================================================
    # t=4: LIFT_UP
    # ============================================================

    lift_actions = controller.inference(
        input_data={},
        t=4,
    )

    assert lift_actions

    expected_lift = (
        TCP_POSITION
        + np.array(
            [
                0.0,
                0.0,
                0.15,
            ],
            dtype=np.float64,
        )
    )

    lift_final = (
        lift_actions[-1]
    )

    np.testing.assert_allclose(
        lift_final[:3],
        expected_lift,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        lift_final[3:7],
        normalized_orientation,
        atol=1e-8,
    )

    np.testing.assert_allclose(
        lift_final[7],
        0.8,
        atol=1e-9,
    )

    # ============================================================
    # Final controller state
    # ============================================================

    assert (
        controller.grasp_plan
        is planner.result
    )

    assert (
        controller.execution_error
        is None
    )

    print()
    print("=" * 78)
    print("GRASPGEN-X CONTROLLER HANDOFF PASSED")
    print("=" * 78)
    print(
        "Planner backend:",
        controller.motion_layer.active_grasp_backend,
    )
    print(
        "Planner TCP:",
        TCP_POSITION,
    )
    print(
        "APPROACHING final:",
        approaching_final[:3],
    )
    print(
        "PICK actions:",
        len(pick_actions),
    )
    print(
        "LIFT final:",
        lift_final[:3],
    )
    print("=" * 78)

def test_nut_assembly_complete_controller_motion_handoff(
    tmp_path,
):

    (
        controller,
        planner,
    ) = _make_nut_controller(
        tmp_path
    )

    # ------------------------------------------------------------
    # t=1 REACH
    # ------------------------------------------------------------

    reach_actions = controller.inference(
        input_data={
            "robot_state": _robot_state(),
        },
        t=1,
    )

    assert reach_actions
    assert len(planner.calls) == 0

    # ------------------------------------------------------------
    # t=2 APPROACHING + real GraspGen-X result handoff
    # ------------------------------------------------------------

    approaching_actions = (
        controller.inference(
            input_data={
                "robot_state": (
                    _robot_state()
                ),
                "grasp_input": (
                    _grasp_input()
                ),
            },
            t=2,
        )
    )

    assert approaching_actions
    assert len(planner.calls) == 1

    np.testing.assert_allclose(
        approaching_actions[-1][:3],
        TCP_POSITION,
        atol=1e-9,
    )

    # Controller must have reconstructed the original
    # grasp-relative geometry.
    np.testing.assert_allclose(
        controller.motion_layer
        ._nut_hole_bottom_local,
        HOLE_BOTTOM_LOCAL,
        atol=1e-8,
    )

    np.testing.assert_allclose(
        controller.motion_layer
        ._nut_hole_top_local,
        HOLE_TOP_LOCAL,
        atol=1e-8,
    )

    np.testing.assert_allclose(
        controller.motion_layer
        ._nut_hole_axis_local,
        HOLE_AXIS_LOCAL,
        atol=1e-8,
    )

    # ------------------------------------------------------------
    # t=3 PICK
    # ------------------------------------------------------------

    pick_actions = controller.inference(
        input_data={},
        t=3,
    )

    assert len(pick_actions) == 1

    # ------------------------------------------------------------
    # t=4 LIFT_UP
    # ------------------------------------------------------------

    lift_actions = controller.inference(
        input_data={},
        t=4,
    )

    assert lift_actions

    lift_tcp = (
        controller.motion_layer
        .current_pose.position.copy()
    )

    # ------------------------------------------------------------
    # t=5 MOVING
    # ------------------------------------------------------------

    moving_actions = controller.inference(
        input_data={},
        t=5,
    )

    assert moving_actions

    (
        _,
        moving_hole_bottom,
        moving_hole_axis,
    ) = (
        controller.motion_layer
        ._nut_current_hole_geometry()
    )

    moving_delta = (
        moving_hole_bottom
        - PEG_TIP_BASE
    )

    moving_lateral_error = float(
        np.linalg.norm(
            moving_delta
            - np.dot(
                moving_delta,
                PEG_AXIS_BASE,
            )
            * PEG_AXIS_BASE
        )
    )

    moving_axis_dot = float(
        np.dot(
            moving_hole_axis,
            PEG_AXIS_BASE,
        )
    )

    assert moving_lateral_error < 1e-8
    assert moving_axis_dot > 0.999999

    # ------------------------------------------------------------
    # t=6 ALIGNING
    # ------------------------------------------------------------

    aligning_actions = controller.inference(
        input_data={},
        t=6,
    )

    assert aligning_actions

    (
        _,
        aligned_hole_bottom,
        aligned_hole_axis,
    ) = (
        controller.motion_layer
        ._nut_current_hole_geometry()
    )

    expected_aligned_bottom = (
        PEG_TIP_BASE
        + 0.020
        * PEG_AXIS_BASE
    )

    np.testing.assert_allclose(
        aligned_hole_bottom,
        expected_aligned_bottom,
        atol=1e-8,
    )

    aligned_axis_dot = float(
        np.dot(
            aligned_hole_axis,
            PEG_AXIS_BASE,
        )
    )

    assert aligned_axis_dot > 0.999999

    # ------------------------------------------------------------
    # t=7 INSERTING
    # ------------------------------------------------------------

    inserting_actions = controller.inference(
        input_data={},
        t=7,
    )

    assert inserting_actions

    (
        _,
        final_hole_bottom,
        final_hole_axis,
    ) = (
        controller.motion_layer
        ._nut_current_hole_geometry()
    )

    expected_final_bottom = (
        PEG_SUPPORT_BASE
        + 0.010
        * PEG_AXIS_BASE
    )

    np.testing.assert_allclose(
        final_hole_bottom,
        expected_final_bottom,
        atol=1e-8,
    )

    final_axis_dot = float(
        np.dot(
            final_hole_axis,
            PEG_AXIS_BASE,
        )
    )

    assert final_axis_dot > 0.999999

    # SceneState centroid must NOT have been used.
    scene_peg = next(
        obj
        for obj in controller.scene_state.objects
        if obj.object_id == PEG_TARGET_ID
    )

    assert not np.allclose(
        final_hole_bottom[:2],
        np.asarray(
            scene_peg.position_base[:2]
        ),
        atol=1e-4,
    )

    assert (
        controller.motion_layer._nut_phase
        == "inserted"
    )

    assert (
        controller.execution_error
        is None
    )

    # ------------------------------------------------------------
    # NUMERICAL REPORT
    # ------------------------------------------------------------

    peg_height = float(
        np.dot(
            PEG_TIP_BASE
            - PEG_SUPPORT_BASE,
            PEG_AXIS_BASE,
        )
    )

    moving_clearance = float(
        np.dot(
            moving_hole_bottom
            - PEG_TIP_BASE,
            PEG_AXIS_BASE,
        )
    )

    final_support_height = float(
        np.dot(
            final_hole_bottom
            - PEG_SUPPORT_BASE,
            PEG_AXIS_BASE,
        )
    )

    print()
    print("=" * 78)
    print("NUT-ASSEMBLY COMPLETE CONTROLLER HANDOFF")
    print("=" * 78)

    print(
        "GraspGen-X TCP:",
        TCP_POSITION,
    )

    print(
        "Peg support:",
        PEG_SUPPORT_BASE,
    )

    print(
        "Peg tip:",
        PEG_TIP_BASE,
    )

    print(
        "Observed peg height:",
        f"{peg_height * 1000.0:.2f} mm",
    )

    print(
        "LIFT TCP:",
        lift_tcp,
    )

    print(
        "MOVING hole bottom:",
        moving_hole_bottom,
    )

    print(
        "MOVING lateral error:",
        f"{moving_lateral_error * 1000.0:.6f} mm",
    )

    print(
        "MOVING axial clearance:",
        f"{moving_clearance * 1000.0:.2f} mm",
    )

    print(
        "ALIGNING hole bottom:",
        aligned_hole_bottom,
    )

    print(
        "ALIGNING axis dot:",
        f"{aligned_axis_dot:.9f}",
    )

    print(
        "FINAL hole bottom:",
        final_hole_bottom,
    )

    print(
        "FINAL height above support:",
        f"{final_support_height * 1000.0:.2f} mm",
    )

    print(
        "FINAL axis dot:",
        f"{final_axis_dot:.9f}",
    )

    print(
        "Primitive action counts:",
        {
            "reach": len(reach_actions),
            "approaching": len(
                approaching_actions
            ),
            "pick": len(pick_actions),
            "lift_up": len(lift_actions),
            "moving": len(moving_actions),
            "aligning": len(
                aligning_actions
            ),
            "inserting": len(
                inserting_actions
            ),
        },
    )

    print("=" * 78)