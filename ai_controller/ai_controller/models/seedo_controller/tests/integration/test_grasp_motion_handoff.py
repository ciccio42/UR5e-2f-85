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
# Paths
# =====================================================================

DEFAULT_GRASP_PLAN = Path(
    "/home/ros2_ws/src/UR5e-2f-85/"
    ".runtime/grasp_planner_test/grasp_plan.json"
)


# =====================================================================
# IDs
# =====================================================================

TARGET_ID = "gray_ring_0"

PICK_PLACE_DESTINATION_ID = (
    "storage_bin_0"
)

ASSEMBLY_DESTINATION_ID = (
    "peg_0"
)


# =====================================================================
# Motion Layer configuration
# =====================================================================

LEGACY_GRASP_ORIENTATION = np.array(
    [
        0.9994452044624775,
        0.03161651380119412,
        0.0021438049655468088,
        0.010251021036213035,
    ],
    dtype=np.float64,
)

MIN_STEP = 0.02

REACH_HOVER_HEIGHT = 0.15
APPROACH_Z_OFFSET = 0.0

RELEASE_HEIGHT_OFFSET = 0.10
LIFT_HEIGHT = 0.15

OBJECT_Y_OFFSET = -0.04

ASSEMBLY_ALIGNMENT_Z_OFFSET = 0.135
ASSEMBLY_INSERTION_Z_OFFSET = -0.11

GRIPPER_OPEN_POSITION = 0.1
GRIPPER_CLOSED_POSITION = 0.8


# =====================================================================
# Helpers
# =====================================================================

def _load_grasp_plan(
    path: str | Path,
) -> dict:
    path = (
        Path(path)
        .expanduser()
        .resolve()
    )

    if not path.is_file():
        raise FileNotFoundError(
            f"Grasp plan not found: {path}"
        )

    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        data = json.load(
            stream
        )

    for key in (
        "tcp_position_base",
        "tcp_orientation_base",
    ):
        if key not in data:
            raise RuntimeError(
                "grasp_plan.json does not contain "
                f"{key!r}."
            )

    return data


def _primitive(
    name: str,
    target: str,
) -> PrimitiveStep:
    return PrimitiveStep(
        name=name,
        arguments={
            "target": target,
        },
        source_code=None,
    )


def _scene_object(
    object_id: str,
    position_base,
    *,
    label: str,
    category: str,
) -> SceneObject:
    return SceneObject(
        object_id=object_id,
        label=label,
        pixel_coordinates=(
            0,
            0,
        ),
        position_camera=(
            0.0,
            0.0,
            0.0,
        ),
        position_base=tuple(
            np.asarray(
                position_base,
                dtype=np.float64,
            )
        ),
        category=category,
        attributes={},
    )


def _new_motion_layer() -> SeeDoMotionLayer:
    return SeeDoMotionLayer(
        grasp_orientation=(
            LEGACY_GRASP_ORIENTATION
        ),
        min_step=MIN_STEP,
        reach_hover_height=(
            REACH_HOVER_HEIGHT
        ),
        approach_z_offset=(
            APPROACH_Z_OFFSET
        ),
        release_height_offset=(
            RELEASE_HEIGHT_OFFSET
        ),
        lift_height=LIFT_HEIGHT,
        gripper_open_position=(
            GRIPPER_OPEN_POSITION
        ),
        gripper_closed_position=(
            GRIPPER_CLOSED_POSITION
        ),
        object_y_offset=(
            OBJECT_Y_OFFSET
        ),
        assembly_alignment_z_offset=(
            ASSEMBLY_ALIGNMENT_Z_OFFSET
        ),
        assembly_insertion_z_offset=(
            ASSEMBLY_INSERTION_Z_OFFSET
        ),
    )


def _assert_quaternion_equivalent(
    actual,
    expected,
    atol: float = 1e-9,
) -> None:
    actual = np.asarray(
        actual,
        dtype=np.float64,
    )

    expected = np.asarray(
        expected,
        dtype=np.float64,
    )

    if actual.shape != (4,):
        raise AssertionError(
            "Actual quaternion must have shape (4,)."
        )

    if expected.shape != (4,):
        raise AssertionError(
            "Expected quaternion must have shape (4,)."
        )

    if np.dot(
        actual,
        expected,
    ) < 0.0:
        actual = -actual

    np.testing.assert_allclose(
        actual,
        expected,
        atol=atol,
    )


def _assert_action_format(
    actions: list[np.ndarray],
    primitive_name: str,
) -> None:
    if not actions:
        raise AssertionError(
            f"{primitive_name} generated no actions."
        )

    for index, action in enumerate(
        actions
    ):
        if not isinstance(
            action,
            np.ndarray,
        ):
            raise AssertionError(
                f"{primitive_name} action {index} "
                "is not a numpy array."
            )

        if action.shape != (8,):
            raise AssertionError(
                f"{primitive_name} action {index} "
                f"has shape {action.shape}; "
                "expected (8,)."
            )

        if not np.isfinite(
            action
        ).all():
            raise AssertionError(
                f"{primitive_name} action {index} "
                "contains non-finite values."
            )

        quaternion = action[
            3:7
        ]

        np.testing.assert_allclose(
            np.linalg.norm(
                quaternion
            ),
            1.0,
            atol=1e-6,
        )


def _assert_actions_orientation(
    actions,
    expected_orientation,
) -> None:
    for action in actions:
        _assert_quaternion_equivalent(
            action[
                3:7
            ],
            expected_orientation,
        )


def _assert_actions_gripper(
    actions,
    expected_gripper,
) -> None:
    for action in actions:
        np.testing.assert_allclose(
            action[
                7
            ],
            expected_gripper,
            atol=1e-9,
        )


def _print_state(
    name: str,
    action: np.ndarray,
) -> None:
    print()
    print(
        f"[{name}]"
    )
    print(
        "position:",
        action[
            :3
        ],
    )
    print(
        "orientation:",
        action[
            3:7
        ],
    )
    print(
        "gripper:",
        action[
            7
        ],
    )


# =====================================================================
# Main test
# =====================================================================

def run_test(
    grasp_plan_path: str | Path = DEFAULT_GRASP_PLAN,
) -> int:
    print()
    print(
        "=" * 78
    )
    print(
        "COMPLETE GRASP-AWARE MOTION LAYER TEST"
    )
    print(
        "=" * 78
    )

    # =============================================================
    # GraspPlanner output
    # =============================================================

    grasp_plan = _load_grasp_plan(
        grasp_plan_path
    )

    tcp_position = np.asarray(
        grasp_plan[
            "tcp_position_base"
        ],
        dtype=np.float64,
    )

    tcp_orientation = np.asarray(
        grasp_plan[
            "tcp_orientation_base"
        ],
        dtype=np.float64,
    )

    if tcp_position.shape != (3,):
        raise AssertionError(
            "tcp_position_base must have shape (3,)."
        )

    if tcp_orientation.shape != (4,):
        raise AssertionError(
            "tcp_orientation_base must have shape (4,)."
        )

    if not np.isfinite(
        tcp_position
    ).all():
        raise AssertionError(
            "TCP position contains non-finite values."
        )

    if not np.isfinite(
        tcp_orientation
    ).all():
        raise AssertionError(
            "TCP orientation contains non-finite values."
        )

    np.testing.assert_allclose(
        np.linalg.norm(
            tcp_orientation
        ),
        1.0,
        atol=1e-6,
    )

    print()
    print(
        "[INPUT] TCP position:"
    )
    print(
        tcp_position
    )

    print()
    print(
        "[INPUT] TCP orientation:"
    )
    print(
        tcp_orientation
    )

    print()
    print(
        "[INPUT] symmetry flipped:",
        grasp_plan.get(
            "tcp_symmetry_flipped"
        ),
    )

    # =============================================================
    # Synthetic but geometrically sensible SceneState
    # =============================================================

    object_position = (
        tcp_position
        + np.array(
            [
                0.03,
                0.02,
                0.02,
            ],
            dtype=np.float64,
        )
    )

    bin_position = (
        tcp_position
        + np.array(
            [
                0.18,
                -0.12,
                0.03,
            ],
            dtype=np.float64,
        )
    )

    peg_position = (
        tcp_position
        + np.array(
            [
                -0.12,
                -0.08,
                0.05,
            ],
            dtype=np.float64,
        )
    )

    target_object = _scene_object(
        TARGET_ID,
        object_position,
        label="ring",
        category="ring",
    )

    destination_bin = _scene_object(
        PICK_PLACE_DESTINATION_ID,
        bin_position,
        label="storage_bin",
        category="container",
    )

    destination_peg = _scene_object(
        ASSEMBLY_DESTINATION_ID,
        peg_position,
        label="peg",
        category="peg",
    )

    scene_state = SceneState(
        objects=(
            target_object,
            destination_bin,
            destination_peg,
        )
    )

    # =============================================================
    # Motion Layer initial state
    # =============================================================

    motion_layer = (
        _new_motion_layer()
    )

    initial_position = (
        tcp_position
        + np.array(
            [
                -0.10,
                -0.10,
                0.30,
            ],
            dtype=np.float64,
        )
    )

    motion_layer.reset(
        current_position=(
            initial_position
        ),
        current_orientation=(
            LEGACY_GRASP_ORIENTATION
        ),
    )

    # =============================================================
    # REACH
    # =============================================================

    reach_actions = (
        motion_layer.translate(
            primitive_step=_primitive(
                "reach",
                TARGET_ID,
            ),
            scene_state=scene_state,
        )
    )

    _assert_action_format(
        reach_actions,
        "REACH",
    )

    reach_final = (
        reach_actions[
            -1
        ]
    )

    expected_reach_position = (
        object_position.copy()
    )

    expected_reach_position[
        1
    ] += OBJECT_Y_OFFSET

    expected_reach_position[
        2
    ] += REACH_HOVER_HEIGHT

    np.testing.assert_allclose(
        reach_final[
            :3
        ],
        expected_reach_position,
        atol=1e-9,
    )

    _assert_quaternion_equivalent(
        reach_final[
            3:7
        ],
        LEGACY_GRASP_ORIENTATION,
    )

    np.testing.assert_allclose(
        reach_final[
            7
        ],
        GRIPPER_OPEN_POSITION,
        atol=1e-9,
    )

    print()
    print(
        "[PASS] REACH uses scene geometry and legacy orientation."
    )

    # =============================================================
    # GraspPlanner -> Motion Layer handoff
    # =============================================================

    motion_layer.set_grasp_pose(
        position=(
            tcp_position
        ),
        orientation=(
            tcp_orientation
        ),
    )

    # =============================================================
    # APPROACHING
    # =============================================================

    approaching_actions = (
        motion_layer.translate(
            primitive_step=_primitive(
                "approaching",
                TARGET_ID,
            ),
            scene_state=scene_state,
        )
    )

    _assert_action_format(
        approaching_actions,
        "APPROACHING",
    )

    approaching_final = (
        approaching_actions[
            -1
        ]
    )

    np.testing.assert_allclose(
        approaching_final[
            :3
        ],
        tcp_position,
        atol=1e-9,
    )

    _assert_quaternion_equivalent(
        approaching_final[
            3:7
        ],
        tcp_orientation,
    )

    _assert_actions_gripper(
        approaching_actions,
        GRIPPER_OPEN_POSITION,
    )

    # Ensure that old SceneState geometry was not used.
    legacy_approach_position = (
        object_position.copy()
    )

    legacy_approach_position[
        1
    ] += OBJECT_Y_OFFSET

    legacy_approach_position[
        2
    ] += APPROACH_Z_OFFSET

    if np.allclose(
        approaching_final[
            :3
        ],
        legacy_approach_position,
    ):
        raise AssertionError(
            "APPROACHING used legacy SceneState geometry."
        )

    print(
        "[PASS] APPROACHING uses GraspPlan TCP pose."
    )

    # =============================================================
    # PICK
    # =============================================================

    pick_actions = (
        motion_layer.translate(
            primitive_step=_primitive(
                "pick",
                TARGET_ID,
            ),
            scene_state=scene_state,
        )
    )

    _assert_action_format(
        pick_actions,
        "PICK",
    )

    if len(
        pick_actions
    ) != 1:
        raise AssertionError(
            "PICK must generate exactly one action."
        )

    pick_final = (
        pick_actions[
            0
        ]
    )

    np.testing.assert_allclose(
        pick_final[
            :3
        ],
        tcp_position,
        atol=1e-9,
    )

    _assert_quaternion_equivalent(
        pick_final[
            3:7
        ],
        tcp_orientation,
    )

    np.testing.assert_allclose(
        pick_final[
            7
        ],
        GRIPPER_CLOSED_POSITION,
        atol=1e-9,
    )

    print(
        "[PASS] PICK preserves grasp pose and closes gripper."
    )

    # =============================================================
    # LIFT UP
    # =============================================================

    lift_actions = (
        motion_layer.translate(
            primitive_step=_primitive(
                "lift_up",
                TARGET_ID,
            ),
            scene_state=scene_state,
        )
    )

    _assert_action_format(
        lift_actions,
        "LIFT_UP",
    )

    lift_final = (
        lift_actions[
            -1
        ]
    )

    expected_lift_position = (
        tcp_position.copy()
    )

    expected_lift_position[
        2
    ] += LIFT_HEIGHT

    np.testing.assert_allclose(
        lift_final[
            :3
        ],
        expected_lift_position,
        atol=1e-9,
    )

    _assert_actions_orientation(
        lift_actions,
        tcp_orientation,
    )

    _assert_actions_gripper(
        lift_actions,
        GRIPPER_CLOSED_POSITION,
    )

    print(
        "[PASS] LIFT_UP preserves grasp orientation and closed gripper."
    )

    # =============================================================
    # PICK-AND-PLACE BRANCH
    # =============================================================

    print()
    print(
        "-" * 78
    )
    print(
        "PICK-AND-PLACE BRANCH"
    )
    print(
        "-" * 78
    )

    # -------------------------------------------------------------
    # MOVING
    # -------------------------------------------------------------

    moving_actions = (
        motion_layer.translate(
            primitive_step=_primitive(
                "moving",
                PICK_PLACE_DESTINATION_ID,
            ),
            scene_state=scene_state,
        )
    )

    _assert_action_format(
        moving_actions,
        "MOVING",
    )

    moving_final = (
        moving_actions[
            -1
        ]
    )

    expected_moving_position = np.array(
        [
            bin_position[
                0
            ],
            bin_position[
                1
            ],
            expected_lift_position[
                2
            ],
        ],
        dtype=np.float64,
    )

    np.testing.assert_allclose(
        moving_final[
            :3
        ],
        expected_moving_position,
        atol=1e-9,
    )

    _assert_actions_orientation(
        moving_actions,
        tcp_orientation,
    )

    _assert_actions_gripper(
        moving_actions,
        GRIPPER_CLOSED_POSITION,
    )

    print(
        "[PASS] MOVING preserves lift height, grasp orientation "
        "and closed gripper."
    )

    # -------------------------------------------------------------
    # PLACING
    # -------------------------------------------------------------

    placing_actions = (
        motion_layer.translate(
            primitive_step=_primitive(
                "placing",
                PICK_PLACE_DESTINATION_ID,
            ),
            scene_state=scene_state,
        )
    )

    _assert_action_format(
        placing_actions,
        "PLACING",
    )

    if len(
        placing_actions
    ) < 2:
        raise AssertionError(
            "PLACING must contain motion actions "
            "and a final gripper-open action."
        )

    expected_placing_position = (
        bin_position.copy()
    )

    expected_placing_position[
        2
    ] += RELEASE_HEIGHT_OFFSET

    placing_final = (
        placing_actions[
            -1
        ]
    )

    np.testing.assert_allclose(
        placing_final[
            :3
        ],
        expected_placing_position,
        atol=1e-9,
    )

    _assert_quaternion_equivalent(
        placing_final[
            3:7
        ],
        tcp_orientation,
    )

    # All movement actions happen while holding the object.
    _assert_actions_gripper(
        placing_actions[
            :-1
        ],
        GRIPPER_CLOSED_POSITION,
    )

    # Last action releases it.
    np.testing.assert_allclose(
        placing_final[
            7
        ],
        GRIPPER_OPEN_POSITION,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        motion_layer.current_pose.position,
        expected_placing_position,
        atol=1e-9,
    )

    _assert_quaternion_equivalent(
        motion_layer.current_pose.orientation,
        tcp_orientation,
    )

    print(
        "[PASS] PLACING preserves orientation and opens gripper "
        "only at destination."
    )

    # =============================================================
    # NUT-ASSEMBLY BRANCH
    # =============================================================
    #
    # Restore the already validated post-LIFT_UP state and test the
    # alternative assembly path independently.
    # =============================================================

    print()
    print(
        "-" * 78
    )
    print(
        "NUT-ASSEMBLY BRANCH"
    )
    print(
        "-" * 78
    )

    assembly_layer = (
        _new_motion_layer()
    )

    assembly_layer.reset(
        current_position=(
            expected_lift_position
        ),
        current_orientation=(
            tcp_orientation
        ),
    )

    assembly_layer.current_gripper_position = (
        GRIPPER_CLOSED_POSITION
    )

    assembly_layer.set_grasp_pose(
        position=(
            tcp_position
        ),
        orientation=(
            tcp_orientation
        ),
    )

    # -------------------------------------------------------------
    # MOVING TO ASSEMBLY TARGET
    # -------------------------------------------------------------

    assembly_moving_actions = (
        assembly_layer.translate(
            primitive_step=_primitive(
                "moving",
                ASSEMBLY_DESTINATION_ID,
            ),
            scene_state=scene_state,
        )
    )

    _assert_action_format(
        assembly_moving_actions,
        "ASSEMBLY MOVING",
    )

    assembly_moving_final = (
        assembly_moving_actions[
            -1
        ]
    )

    expected_assembly_moving_position = (
        np.array(
            [
                peg_position[
                    0
                ],
                peg_position[
                    1
                ],
                expected_lift_position[
                    2
                ],
            ],
            dtype=np.float64,
        )
    )

    np.testing.assert_allclose(
        assembly_moving_final[
            :3
        ],
        expected_assembly_moving_position,
        atol=1e-9,
    )

    _assert_actions_orientation(
        assembly_moving_actions,
        tcp_orientation,
    )

    _assert_actions_gripper(
        assembly_moving_actions,
        GRIPPER_CLOSED_POSITION,
    )

    print(
        "[PASS] Assembly MOVING preserves grasp orientation."
    )

    # -------------------------------------------------------------
    # ALIGNING
    # -------------------------------------------------------------

    aligning_actions = (
        assembly_layer.translate(
            primitive_step=_primitive(
                "aligning",
                ASSEMBLY_DESTINATION_ID,
            ),
            scene_state=scene_state,
        )
    )

    _assert_action_format(
        aligning_actions,
        "ALIGNING",
    )

    aligning_final = (
        aligning_actions[
            -1
        ]
    )

    expected_alignment_position = (
        peg_position.copy()
    )

    expected_alignment_position[
        2
    ] += ASSEMBLY_ALIGNMENT_Z_OFFSET

    np.testing.assert_allclose(
        aligning_final[
            :3
        ],
        expected_alignment_position,
        atol=1e-9,
    )

    _assert_actions_orientation(
        aligning_actions,
        tcp_orientation,
    )

    _assert_actions_gripper(
        aligning_actions,
        GRIPPER_CLOSED_POSITION,
    )

    print(
        "[PASS] ALIGNING preserves grasp orientation "
        "and closed gripper."
    )

    # -------------------------------------------------------------
    # INSERTING
    # -------------------------------------------------------------

    inserting_actions = (
        assembly_layer.translate(
            primitive_step=_primitive(
                "inserting",
                ASSEMBLY_DESTINATION_ID,
            ),
            scene_state=scene_state,
        )
    )

    _assert_action_format(
        inserting_actions,
        "INSERTING",
    )

    if len(
        inserting_actions
    ) < 2:
        raise AssertionError(
            "INSERTING must contain motion actions "
            "and a final gripper-open action."
        )

    expected_insertion_position = (
        peg_position.copy()
    )

    expected_insertion_position[
        2
    ] += ASSEMBLY_INSERTION_Z_OFFSET

    inserting_final = (
        inserting_actions[
            -1
        ]
    )

    np.testing.assert_allclose(
        inserting_final[
            :3
        ],
        expected_insertion_position,
        atol=1e-9,
    )

    _assert_quaternion_equivalent(
        inserting_final[
            3:7
        ],
        tcp_orientation,
    )

    # Motion part happens while grasping the ring.
    _assert_actions_gripper(
        inserting_actions[
            :-1
        ],
        GRIPPER_CLOSED_POSITION,
    )

    # Final action releases the ring.
    np.testing.assert_allclose(
        inserting_final[
            7
        ],
        GRIPPER_OPEN_POSITION,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        assembly_layer.current_pose.position,
        expected_insertion_position,
        atol=1e-9,
    )

    _assert_quaternion_equivalent(
        assembly_layer.current_pose.orientation,
        tcp_orientation,
    )

    print(
        "[PASS] INSERTING preserves grasp orientation and "
        "opens gripper only after insertion."
    )

    # =============================================================
    # Summary
    # =============================================================

    print()
    print(
        "=" * 78
    )
    print(
        "MOTION SUMMARY"
    )
    print(
        "=" * 78
    )

    _print_state(
        "REACH",
        reach_final,
    )

    _print_state(
        "APPROACHING",
        approaching_final,
    )

    _print_state(
        "PICK",
        pick_final,
    )

    _print_state(
        "LIFT_UP",
        lift_final,
    )

    _print_state(
        "PICK-PLACE MOVING",
        moving_final,
    )

    _print_state(
        "PLACING",
        placing_final,
    )

    _print_state(
        "ASSEMBLY MOVING",
        assembly_moving_final,
    )

    _print_state(
        "ALIGNING",
        aligning_final,
    )

    _print_state(
        "INSERTING",
        inserting_final,
    )

    print()
    print(
        "Waypoint counts:"
    )

    print(
        "  reach:",
        len(
            reach_actions
        ),
    )

    print(
        "  approaching:",
        len(
            approaching_actions
        ),
    )

    print(
        "  pick:",
        len(
            pick_actions
        ),
    )

    print(
        "  lift_up:",
        len(
            lift_actions
        ),
    )

    print(
        "  moving (pick-place):",
        len(
            moving_actions
        ),
    )

    print(
        "  placing:",
        len(
            placing_actions
        ),
    )

    print(
        "  moving (assembly):",
        len(
            assembly_moving_actions
        ),
    )

    print(
        "  aligning:",
        len(
            aligning_actions
        ),
    )

    print(
        "  inserting:",
        len(
            inserting_actions
        ),
    )

    print()
    print(
        "=" * 78
    )
    print(
        "COMPLETE GRASP-AWARE MOTION LAYER TEST PASSED"
    )
    print(
        "=" * 78
    )

    return 0


def main() -> int:
    return run_test()


if __name__ == "__main__":
    raise SystemExit(
        main()
    )