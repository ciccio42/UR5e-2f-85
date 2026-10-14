from __future__ import annotations

from pathlib import Path

import numpy as np

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
        task_type=None,
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