from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from ai_controller.models.seedo_controller.motion_layer import (
    SeeDoMotionLayer,
)

from results import (
    PrimitivePlan,
    PrimitiveStep,
    SceneObject,
    SceneState,
)


GRASP_ORIENTATION = np.array(
    [
        0.9994452044624775,
        0.03161651380119412,
        0.0021438049655468088,
        0.010251021036213035,
    ],
    dtype=np.float64,
)

INITIAL_POSITION = np.array(
    [
        -0.15552094619366708,
        0.34869994018501943,
        0.1532803451753288,
    ],
    dtype=np.float64,
)

INITIAL_ORIENTATION = GRASP_ORIENTATION.copy()

MIN_STEP = 0.02
REACH_HOVER_HEIGHT = 0.15
APPROACH_Z_OFFSET = 0.0
RELEASE_HEIGHT_OFFSET = 0.10
LIFT_HEIGHT = 0.15

GRIPPER_OPEN_POSITION = 0.1
GRIPPER_CLOSED_POSITION = 0.8

OBJECT_Y_OFFSET = -0.06


def load_scene_state(
    path: str | Path,
) -> SceneState:
    path = Path(path).expanduser().resolve()

    with path.open("r", encoding="utf-8") as stream:
        payload = json.load(stream)

    objects = tuple(
        SceneObject(
            object_id=str(obj["object_id"]),
            label=str(obj["label"]),
            pixel_coordinates=tuple(
                int(value)
                for value in obj["pixel_coordinates"]
            ),
            position_camera=tuple(
                float(value)
                for value in obj["position_camera"]
            ),
            position_base=tuple(
                float(value)
                for value in obj["position_base"]
            ),
        )
        for obj in payload["objects"]
    )

    return SceneState(
        objects=objects,
    )


def load_primitive_plan(
    path: str | Path,
) -> PrimitivePlan:
    path = Path(path).expanduser().resolve()

    with path.open("r", encoding="utf-8") as stream:
        payload = json.load(stream)

    steps = tuple(
        PrimitiveStep(
            name=str(step["name"]),
            arguments=dict(step["arguments"]),
            source_code=step.get("source_code"),
        )
        for step in payload["steps"]
    )

    return PrimitivePlan(
        steps=steps,
        source_code=str(
            payload.get("source_code", "")
        ),
    )


def get_object(
    scene_state: SceneState,
    object_id: str,
) -> SceneObject:
    matches = [
        obj
        for obj in scene_state.objects
        if obj.object_id == object_id
    ]

    if len(matches) != 1:
        raise AssertionError(
            f"Expected exactly one object {object_id!r}, "
            f"found {len(matches)}."
        )

    return matches[0]


def assert_action_format(
    actions: list[np.ndarray],
) -> None:
    if not actions:
        raise AssertionError(
            "Motion Layer returned an empty action list."
        )

    for index, action in enumerate(actions):
        if not isinstance(action, np.ndarray):
            raise AssertionError(
                f"Action {index} is not a numpy array."
            )

        if action.shape != (8,):
            raise AssertionError(
                f"Action {index} has shape {action.shape}; "
                "expected (8,)."
            )

        if not np.all(np.isfinite(action)):
            raise AssertionError(
                f"Action {index} contains non-finite values: "
                f"{action}"
            )


def print_actions(
    primitive: PrimitiveStep,
    actions: list[np.ndarray],
) -> None:
    print()
    print(
        f"=== {primitive.name}"
        f"({primitive.arguments}) ==="
    )

    print(
        f"Generated actions: {len(actions)}"
    )

    for index, action in enumerate(actions):
        print(
            f"[{index:02d}] "
            f"pos={action[:3]} "
            f"quat={action[3:7]} "
            f"gripper={action[7]:.4f}"
        )


def run_test(
    scene_state_path: str | Path,
    primitive_plan_path: str | Path,
) -> int:
    print(
        "=== LOADING MOTION LAYER TEST DATA ==="
    )

    scene_state = load_scene_state(
        scene_state_path
    )

    primitive_plan = load_primitive_plan(
        primitive_plan_path
    )

    print(
        f"Scene objects: {len(scene_state.objects)}"
    )

    print(
        f"Primitive steps: {len(primitive_plan.steps)}"
    )

    if not primitive_plan.steps:
        raise AssertionError(
            "PrimitivePlan is empty."
        )

    motion_layer = SeeDoMotionLayer(
        grasp_orientation=GRASP_ORIENTATION,
        min_step=MIN_STEP,
        reach_hover_height=REACH_HOVER_HEIGHT,
        approach_z_offset=APPROACH_Z_OFFSET,
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
        object_y_offset=OBJECT_Y_OFFSET,
    )

    motion_layer.reset(
        current_position=INITIAL_POSITION,
        current_orientation=INITIAL_ORIENTATION,
        artifacts_dir="/seedo_tests/motion_layer",
    )

    all_actions: list[np.ndarray] = []

    actions_by_primitive: dict[
        str,
        list[np.ndarray],
    ] = {}

    for primitive in primitive_plan.steps:
        actions = motion_layer.translate(
            primitive_step=primitive,
            scene_state=scene_state,
        )

        assert_action_format(actions)

        print_actions(
            primitive,
            actions,
        )

        actions_by_primitive[
            primitive.name
        ] = actions

        all_actions.extend(actions)

    print()
    print(
        "=== VALIDATING MOTION SEMANTICS ==="
    )

    green_cube = get_object(
        scene_state,
        "green cube",
    )

    destination_bin = get_object(
        scene_state,
        "first bin from the left",
    )

    #
    # reach
    #
    reach_actions = actions_by_primitive[
        "reach"
    ]

    expected_reach_position = np.array(
        green_cube.position_base,
        dtype=np.float64,
    )

    expected_reach_position[1] += (
        OBJECT_Y_OFFSET
    )

    expected_reach_position[2] += (
        REACH_HOVER_HEIGHT
    )

    np.testing.assert_allclose(
        reach_actions[-1][:3],
        expected_reach_position,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        reach_actions[-1][3:7],
        GRASP_ORIENTATION,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        reach_actions[-1][7],
        GRIPPER_OPEN_POSITION,
        atol=1e-9,
    )

    print("[PASS] reach")

    #
    # approaching
    #
    approaching_actions = (
        actions_by_primitive[
            "approaching"
        ]
    )

    expected_approach_position = np.array(
        green_cube.position_base,
        dtype=np.float64,
    )

    expected_approach_position[1] += (
        OBJECT_Y_OFFSET
    )

    expected_approach_position[2] += (
        APPROACH_Z_OFFSET
    )

    np.testing.assert_allclose(
        approaching_actions[-1][:3],
        expected_approach_position,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        approaching_actions[-1][7],
        GRIPPER_OPEN_POSITION,
        atol=1e-9,
    )

    print("[PASS] approaching")

    #
    # pick
    #
    pick_actions = actions_by_primitive[
        "pick"
    ]

    if len(pick_actions) != 1:
        raise AssertionError(
            "pick must generate exactly one "
            "gripper action."
        )

    np.testing.assert_allclose(
        pick_actions[0][:3],
        expected_approach_position,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        pick_actions[0][7],
        GRIPPER_CLOSED_POSITION,
        atol=1e-9,
    )

    print("[PASS] pick")

    #
    # lift_up
    #
    lift_actions = actions_by_primitive[
        "lift_up"
    ]

    expected_lift_position = (
        expected_approach_position.copy()
    )

    expected_lift_position[2] += (
        LIFT_HEIGHT
    )

    np.testing.assert_allclose(
        lift_actions[-1][:3],
        expected_lift_position,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        lift_actions[-1][7],
        GRIPPER_CLOSED_POSITION,
        atol=1e-9,
    )

    print("[PASS] lift_up")

    #
    # moving
    #
    moving_actions = actions_by_primitive[
        "moving"
    ]

    expected_moving_position = np.array(
        [
            destination_bin.position_base[0],
            destination_bin.position_base[1],
            expected_lift_position[2],
        ],
        dtype=np.float64,
    )

    np.testing.assert_allclose(
        moving_actions[-1][:3],
        expected_moving_position,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        moving_actions[-1][7],
        GRIPPER_CLOSED_POSITION,
        atol=1e-9,
    )

    print("[PASS] moving")

    #
    # placing
    #
    placing_actions = actions_by_primitive[
        "placing"
    ]

    if len(placing_actions) < 2:
        raise AssertionError(
            "placing must contain at least one "
            "motion action followed by the "
            "gripper-open action."
        )

    expected_placing_position = np.array(
        destination_bin.position_base,
        dtype=np.float64,
    )

    expected_placing_position[2] += (
        RELEASE_HEIGHT_OFFSET
    )

    # Last movement waypoint:
    np.testing.assert_allclose(
        placing_actions[-2][:3],
        expected_placing_position,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        placing_actions[-2][7],
        GRIPPER_CLOSED_POSITION,
        atol=1e-9,
    )

    # Final action opens the gripper without
    # changing TCP pose.
    np.testing.assert_allclose(
        placing_actions[-1][:3],
        expected_placing_position,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        placing_actions[-1][3:7],
        placing_actions[-2][3:7],
        atol=1e-9,
    )

    np.testing.assert_allclose(
        placing_actions[-1][7],
        GRIPPER_OPEN_POSITION,
        atol=1e-9,
    )

    print("[PASS] placing")

    motion_artifact_path = Path(
        "/seedo_tests/motion_layer/motion_plan.json"
    )

    if not motion_artifact_path.is_file():
        raise AssertionError(
            "Motion Layer artifact was not generated: "
            f"{motion_artifact_path}"
        )

    with motion_artifact_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        motion_artifact = json.load(stream)

    if motion_artifact.get("total_actions") != len(all_actions):
        raise AssertionError(
            "Motion Layer artifact contains an unexpected "
            "number of low-level actions. "
            f"Expected {len(all_actions)}, "
            f"found {motion_artifact.get('total_actions')}."
        )

    if len(
        motion_artifact.get(
            "primitives",
            [],
        )
    ) != len(primitive_plan.steps):
        raise AssertionError(
            "Motion Layer artifact contains an unexpected "
            "number of primitives."
        )

    print(
        "[PASS] motion_plan.json artifact"
    )

    print()
    print(
        "=== MOTION LAYER TEST PASSED ==="
    )

    print(
        f"Total generated low-level actions: "
        f"{len(all_actions)}"
    )

    print(
        "Final planned position:",
        motion_layer.current_pose.position,
    )

    print(
        "Final planned orientation:",
        motion_layer.current_pose.orientation,
    )

    print(
        "Final gripper position:",
        motion_layer.current_gripper_position,
    )

    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Offline unit/integration test for "
            "SeeDoMotionLayer."
        )
    )

    parser.add_argument(
        "--scene-state",
        required=True,
        help="Path to scene_state.json.",
    )

    parser.add_argument(
        "--primitive-plan",
        required=True,
        help="Path to primitive_plan.json.",
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    return run_test(
        scene_state_path=args.scene_state,
        primitive_plan_path=args.primitive_plan,
    )


if __name__ == "__main__":
    raise SystemExit(main())