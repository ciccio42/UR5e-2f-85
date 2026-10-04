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


DEFAULT_SCENE_STATE = Path(
    "/seedo_tests/scene_interpreter/scene_state.json"
)

DEFAULT_PRIMITIVE_PLAN = Path(
    "/seedo_tests/lmp_generator/primitive_plan.json"
)

DEFAULT_ARTIFACTS_DIR = Path(
    "/seedo_tests/motion_layer"
)

EXPECTED_PRIMITIVES = (
    ("reach", "green_block_0"),
    ("approaching", "green_block_0"),
    ("pick", "green_block_0"),
    ("lift_up", "green_block_0"),
    ("moving", "storage_bin_0"),
    ("placing", "storage_bin_0"),
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

INITIAL_ORIENTATION = (
    GRASP_ORIENTATION.copy()
)

MIN_STEP = 0.02
REACH_HOVER_HEIGHT = 0.15
APPROACH_Z_OFFSET = 0.0
RELEASE_HEIGHT_OFFSET = 0.10
LIFT_HEIGHT = 0.15

GRIPPER_OPEN_POSITION = 0.1
GRIPPER_CLOSED_POSITION = 0.8

OBJECT_Y_OFFSET = -0.06


def _load_json(
    path: str | Path,
) -> dict:
    normalized_path = (
        Path(path)
        .expanduser()
        .resolve()
    )

    if not normalized_path.is_file():
        raise FileNotFoundError(
            "Required integration artifact does not exist: "
            f"{normalized_path}"
        )

    if normalized_path.stat().st_size == 0:
        raise ValueError(
            "Required integration artifact is empty: "
            f"{normalized_path}"
        )

    with normalized_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        payload = json.load(
            stream
        )

    if not isinstance(
        payload,
        dict,
    ):
        raise ValueError(
            f"Expected a JSON object in {normalized_path}."
        )

    return payload


def load_scene_state(
    path: str | Path,
) -> SceneState:
    payload = _load_json(
        path
    )

    objects_data = payload.get(
        "objects"
    )

    if (
        not isinstance(
            objects_data,
            list,
        )
        or not objects_data
    ):
        raise ValueError(
            "scene_state.json must contain a non-empty "
            "'objects' list."
        )

    objects: list[
        SceneObject
    ] = []

    for index, obj in enumerate(
        objects_data
    ):
        if not isinstance(
            obj,
            dict,
        ):
            raise ValueError(
                "Invalid SceneObject entry at index "
                f"{index}: {obj!r}"
            )

        pixel_coordinates = obj.get(
            "pixel_coordinates"
        )
        position_camera = obj.get(
            "position_camera"
        )
        position_base = obj.get(
            "position_base"
        )
        attributes = obj.get(
            "attributes",
            {},
        )

        if (
            not isinstance(
                pixel_coordinates,
                list,
            )
            or len(pixel_coordinates) != 2
        ):
            raise ValueError(
                f"Invalid pixel_coordinates at index {index}: "
                f"{pixel_coordinates!r}"
            )

        if (
            not isinstance(
                position_camera,
                list,
            )
            or len(position_camera) != 3
        ):
            raise ValueError(
                f"Invalid position_camera at index {index}: "
                f"{position_camera!r}"
            )

        if (
            not isinstance(
                position_base,
                list,
            )
            or len(position_base) != 3
        ):
            raise ValueError(
                f"Invalid position_base at index {index}: "
                f"{position_base!r}"
            )

        if not isinstance(
            attributes,
            dict,
        ):
            raise ValueError(
                f"Invalid attributes at index {index}: "
                f"{attributes!r}"
            )

        objects.append(
            SceneObject(
                object_id=str(
                    obj["object_id"]
                ),
                label=str(
                    obj["label"]
                ),
                pixel_coordinates=(
                    int(
                        pixel_coordinates[0]
                    ),
                    int(
                        pixel_coordinates[1]
                    ),
                ),
                position_camera=tuple(
                    float(value)
                    for value
                    in position_camera
                ),
                position_base=tuple(
                    float(value)
                    for value
                    in position_base
                ),
                category=(
                    None
                    if obj.get(
                        "category"
                    ) is None
                    else str(
                        obj[
                            "category"
                        ]
                    )
                ),
                attributes=dict(
                    attributes
                ),
            )
        )

    return SceneState(
        objects=tuple(
            objects
        )
    )


def load_primitive_plan(
    path: str | Path,
) -> PrimitivePlan:
    payload = _load_json(
        path
    )

    steps_data = payload.get(
        "steps"
    )

    if (
        not isinstance(
            steps_data,
            list,
        )
        or not steps_data
    ):
        raise ValueError(
            "primitive_plan.json must contain a non-empty "
            "'steps' list."
        )

    steps: list[
        PrimitiveStep
    ] = []

    for index, step in enumerate(
        steps_data
    ):
        if not isinstance(
            step,
            dict,
        ):
            raise ValueError(
                "Invalid PrimitiveStep entry at index "
                f"{index}: {step!r}"
            )

        arguments = step.get(
            "arguments"
        )

        if not isinstance(
            arguments,
            dict,
        ):
            raise ValueError(
                f"PrimitiveStep {index} has invalid arguments: "
                f"{arguments!r}"
            )

        steps.append(
            PrimitiveStep(
                name=str(
                    step["name"]
                ),
                arguments=dict(
                    arguments
                ),
                source_code=step.get(
                    "source_code"
                ),
            )
        )

    return PrimitivePlan(
        steps=tuple(
            steps
        ),
        source_code=str(
            payload.get(
                "source_code",
                "",
            )
        ),
    )


def get_object(
    scene_state: SceneState,
    object_id: str,
) -> SceneObject:
    matches = [
        obj
        for obj
        in scene_state.objects
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

    for index, action in enumerate(
        actions
    ):
        if not isinstance(
            action,
            np.ndarray,
        ):
            raise AssertionError(
                f"Action {index} is not a numpy array."
            )

        if action.shape != (
            8,
        ):
            raise AssertionError(
                f"Action {index} has shape {action.shape}; "
                "expected (8,)."
            )

        if not np.all(
            np.isfinite(
                action
            )
        ):
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
        f"Generated actions: "
        f"{len(actions)}"
    )

    for index, action in enumerate(
        actions
    ):
        print(
            f"[{index:02d}] "
            f"pos={action[:3]} "
            f"quat={action[3:7]} "
            f"gripper={action[7]:.4f}"
        )


def _assert_upstream_handoffs(
    scene_state: SceneState,
    primitive_plan: PrimitivePlan,
) -> None:
    scene_object_ids = {
        obj.object_id
        for obj
        in scene_state.objects
    }

    for required_id in (
        "green_block_0",
        "storage_bin_0",
    ):
        if required_id not in scene_object_ids:
            raise AssertionError(
                "Expected runtime object is missing from "
                f"SceneState: {required_id}"
            )

    if len(
        primitive_plan.steps
    ) != len(
        EXPECTED_PRIMITIVES
    ):
        raise AssertionError(
            "Unexpected PrimitivePlan length: "
            f"expected={len(EXPECTED_PRIMITIVES)}, "
            f"received={len(primitive_plan.steps)}"
        )

    for index, (
        step,
        (
            expected_name,
            expected_target,
        ),
    ) in enumerate(
        zip(
            primitive_plan.steps,
            EXPECTED_PRIMITIVES,
            strict=True,
        ),
        start=1,
    ):
        if step.name != expected_name:
            raise AssertionError(
                f"Primitive {index}: expected name "
                f"{expected_name!r}, received {step.name!r}."
            )

        if (
            step.arguments
            != {
                "target": expected_target,
            }
        ):
            raise AssertionError(
                f"Primitive {index}: expected target "
                f"{expected_target!r}, received "
                f"{step.arguments!r}."
            )


def _assert_motion_artifact(
    *,
    artifact_path: Path,
    primitive_plan: PrimitivePlan,
    actions_by_primitive: dict[
        str,
        list[np.ndarray],
    ],
    total_actions: int,
    motion_layer: SeeDoMotionLayer,
) -> None:
    payload = _load_json(
        artifact_path
    )

    if (
        payload.get(
            "total_actions"
        )
        != total_actions
    ):
        raise AssertionError(
            "motion_plan.json contains an unexpected "
            "total_actions value. "
            f"Expected {total_actions}, "
            f"received {payload.get('total_actions')}."
        )

    primitives = payload.get(
        "primitives"
    )

    if not isinstance(
        primitives,
        list,
    ):
        raise AssertionError(
            "motion_plan.json does not contain a primitives list."
        )

    if len(
        primitives
    ) != len(
        primitive_plan.steps
    ):
        raise AssertionError(
            "motion_plan.json contains an unexpected "
            "number of primitives."
        )

    for primitive_step, artifact_primitive in zip(
        primitive_plan.steps,
        primitives,
        strict=True,
    ):
        if (
            artifact_primitive.get(
                "name"
            )
            != primitive_step.name
        ):
            raise AssertionError(
                "motion_plan.json changed primitive order or name."
            )

        if (
            artifact_primitive.get(
                "arguments"
            )
            != primitive_step.arguments
        ):
            raise AssertionError(
                "motion_plan.json changed primitive arguments."
            )

        artifact_actions = (
            artifact_primitive.get(
                "actions"
            )
        )

        if not isinstance(
            artifact_actions,
            list,
        ):
            raise AssertionError(
                "motion_plan.json primitive does not contain "
                "an actions list."
            )

        produced_actions = (
            actions_by_primitive[
                primitive_step.name
            ]
        )

        if len(
            artifact_actions
        ) != len(
            produced_actions
        ):
            raise AssertionError(
                "motion_plan.json action count does not match "
                f"the translated primitive {primitive_step.name!r}."
            )

        for (
            produced_action,
            artifact_action,
        ) in zip(
            produced_actions,
            artifact_actions,
            strict=True,
        ):
            np.testing.assert_allclose(
                np.asarray(
                    artifact_action[
                        "position"
                    ],
                    dtype=np.float64,
                ),
                produced_action[:3],
                atol=1e-12,
            )

            np.testing.assert_allclose(
                np.asarray(
                    artifact_action[
                        "orientation"
                    ],
                    dtype=np.float64,
                ),
                produced_action[3:7],
                atol=1e-12,
            )

            np.testing.assert_allclose(
                float(
                    artifact_action[
                        "gripper_position"
                    ]
                ),
                produced_action[7],
                atol=1e-12,
            )

    final_state = payload.get(
        "final_planned_state"
    )

    if not isinstance(
        final_state,
        dict,
    ):
        raise AssertionError(
            "motion_plan.json does not contain final_planned_state."
        )

    np.testing.assert_allclose(
        np.asarray(
            final_state[
                "position"
            ],
            dtype=np.float64,
        ),
        motion_layer.current_pose.position,
        atol=1e-12,
    )

    np.testing.assert_allclose(
        np.asarray(
            final_state[
                "orientation"
            ],
            dtype=np.float64,
        ),
        motion_layer.current_pose.orientation,
        atol=1e-12,
    )

    np.testing.assert_allclose(
        float(
            final_state[
                "gripper_position"
            ]
        ),
        motion_layer.current_gripper_position,
        atol=1e-12,
    )


def run_test(
    scene_state_path: str | Path = DEFAULT_SCENE_STATE,
    primitive_plan_path: str | Path = DEFAULT_PRIMITIVE_PLAN,
    artifacts_dir: str | Path = DEFAULT_ARTIFACTS_DIR,
) -> int:
    """Translate the validated generalized PrimitivePlan into low-level actions."""

    scene_state_path = (
        Path(
            scene_state_path
        )
        .expanduser()
        .resolve()
    )

    primitive_plan_path = (
        Path(
            primitive_plan_path
        )
        .expanduser()
        .resolve()
    )

    artifacts_dir = (
        Path(
            artifacts_dir
        )
        .expanduser()
        .resolve()
    )

    print(
        "=== LOADING MOTION LAYER HANDOFFS ==="
    )

    scene_state = load_scene_state(
        scene_state_path
    )

    primitive_plan = load_primitive_plan(
        primitive_plan_path
    )

    _assert_upstream_handoffs(
        scene_state,
        primitive_plan,
    )

    print(
        f"SceneState: {scene_state_path}"
    )
    print(
        f"PrimitivePlan: {primitive_plan_path}"
    )
    print(
        f"Scene objects: "
        f"{len(scene_state.objects)}"
    )
    print(
        f"Primitive steps: "
        f"{len(primitive_plan.steps)}"
    )

    motion_layer = SeeDoMotionLayer(
        grasp_orientation=(
            GRASP_ORIENTATION
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
    )

    motion_layer.reset(
        current_position=(
            INITIAL_POSITION
        ),
        current_orientation=(
            INITIAL_ORIENTATION
        ),
        artifacts_dir=(
            artifacts_dir
        ),
    )

    all_actions: list[
        np.ndarray
    ] = []

    actions_by_primitive: dict[
        str,
        list[np.ndarray],
    ] = {}

    print()
    print(
        "=== TRANSLATING PRIMITIVE PLAN ==="
    )

    for primitive in (
        primitive_plan.steps
    ):
        actions = (
            motion_layer.translate(
                primitive_step=primitive,
                scene_state=scene_state,
            )
        )

        assert_action_format(
            actions
        )

        print_actions(
            primitive,
            actions,
        )

        actions_by_primitive[
            primitive.name
        ] = actions

        all_actions.extend(
            actions
        )

    print()
    print(
        "=== VALIDATING MOTION SEMANTICS ==="
    )

    green_block = get_object(
        scene_state,
        "green_block_0",
    )

    destination_bin = get_object(
        scene_state,
        "storage_bin_0",
    )

    # ---------------------------------------------------------
    # reach
    # ---------------------------------------------------------

    reach_actions = (
        actions_by_primitive[
            "reach"
        ]
    )

    expected_reach_position = np.array(
        green_block.position_base,
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

    print(
        "[PASS] reach -> green_block_0"
    )

    # ---------------------------------------------------------
    # approaching
    # ---------------------------------------------------------

    approaching_actions = (
        actions_by_primitive[
            "approaching"
        ]
    )

    expected_approach_position = np.array(
        green_block.position_base,
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
        approaching_actions[-1][3:7],
        GRASP_ORIENTATION,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        approaching_actions[-1][7],
        GRIPPER_OPEN_POSITION,
        atol=1e-9,
    )

    print(
        "[PASS] approaching -> green_block_0"
    )

    # ---------------------------------------------------------
    # pick
    # ---------------------------------------------------------

    pick_actions = (
        actions_by_primitive[
            "pick"
        ]
    )

    if len(
        pick_actions
    ) != 1:
        raise AssertionError(
            "pick must generate exactly one gripper action."
        )

    np.testing.assert_allclose(
        pick_actions[0][:3],
        expected_approach_position,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        pick_actions[0][3:7],
        GRASP_ORIENTATION,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        pick_actions[0][7],
        GRIPPER_CLOSED_POSITION,
        atol=1e-9,
    )

    print(
        "[PASS] pick -> green_block_0"
    )

    # ---------------------------------------------------------
    # lift_up
    # ---------------------------------------------------------

    lift_actions = (
        actions_by_primitive[
            "lift_up"
        ]
    )

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
        lift_actions[-1][3:7],
        GRASP_ORIENTATION,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        lift_actions[-1][7],
        GRIPPER_CLOSED_POSITION,
        atol=1e-9,
    )

    print(
        "[PASS] lift_up -> green_block_0"
    )

    # ---------------------------------------------------------
    # moving
    # ---------------------------------------------------------

    moving_actions = (
        actions_by_primitive[
            "moving"
        ]
    )

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
        moving_actions[-1][3:7],
        GRASP_ORIENTATION,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        moving_actions[-1][7],
        GRIPPER_CLOSED_POSITION,
        atol=1e-9,
    )

    print(
        "[PASS] moving -> storage_bin_0"
    )

    # ---------------------------------------------------------
    # placing
    # ---------------------------------------------------------

    placing_actions = (
        actions_by_primitive[
            "placing"
        ]
    )

    if len(
        placing_actions
    ) < 2:
        raise AssertionError(
            "placing must contain at least one motion action "
            "followed by the gripper-open action."
        )

    expected_placing_position = np.array(
        destination_bin.position_base,
        dtype=np.float64,
    )

    expected_placing_position[2] += (
        RELEASE_HEIGHT_OFFSET
    )

    np.testing.assert_allclose(
        placing_actions[-2][:3],
        expected_placing_position,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        placing_actions[-2][3:7],
        GRASP_ORIENTATION,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        placing_actions[-2][7],
        GRIPPER_CLOSED_POSITION,
        atol=1e-9,
    )

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

    print(
        "[PASS] placing -> storage_bin_0"
    )

    # ---------------------------------------------------------
    # Validate accumulated artifact.
    # ---------------------------------------------------------

    motion_artifact_path = (
        artifacts_dir
        / "motion_plan.json"
    )

    if not motion_artifact_path.is_file():
        raise AssertionError(
            "Motion Layer artifact was not generated: "
            f"{motion_artifact_path}"
        )

    _assert_motion_artifact(
        artifact_path=(
            motion_artifact_path
        ),
        primitive_plan=(
            primitive_plan
        ),
        actions_by_primitive=(
            actions_by_primitive
        ),
        total_actions=len(
            all_actions
        ),
        motion_layer=(
            motion_layer
        ),
    )

    print(
        "[PASS] motion_plan.json artifact"
    )

    expected_final_position = (
        expected_placing_position
    )

    np.testing.assert_allclose(
        motion_layer.current_pose.position,
        expected_final_position,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        motion_layer.current_pose.orientation,
        GRASP_ORIENTATION,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        motion_layer.current_gripper_position,
        GRIPPER_OPEN_POSITION,
        atol=1e-9,
    )

    print()
    print(
        "MOTION LAYER TEST PASSED"
    )
    print(
        "Total generated low-level actions: "
        f"{len(all_actions)}"
    )
    print(
        "Final planned position: "
        f"{motion_layer.current_pose.position}"
    )
    print(
        "Final planned orientation: "
        f"{motion_layer.current_pose.orientation}"
    )
    print(
        "Final gripper position: "
        f"{motion_layer.current_gripper_position}"
    )
    print(
        "Motion artifact: "
        f"{motion_artifact_path}"
    )

    return 0


def run_motion_layer_test(
    args: argparse.Namespace,
) -> int:
    """CLI-stage adapter used by tests/cli.py."""

    return run_test(
        scene_state_path=(
            getattr(
                args,
                "scene_state",
                None,
            )
            or DEFAULT_SCENE_STATE
        ),
        primitive_plan_path=(
            getattr(
                args,
                "primitive_plan",
                None,
            )
            or DEFAULT_PRIMITIVE_PLAN
        ),
        artifacts_dir=(
            getattr(
                args,
                "artifacts_dir",
                None,
            )
            or DEFAULT_ARTIFACTS_DIR
        ),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Offline integration test for "
            "SeeDoMotionLayer."
        )
    )

    parser.add_argument(
        "--scene-state",
        type=Path,
        default=(
            DEFAULT_SCENE_STATE
        ),
    )

    parser.add_argument(
        "--primitive-plan",
        type=Path,
        default=(
            DEFAULT_PRIMITIVE_PLAN
        ),
    )

    parser.add_argument(
        "--artifacts-dir",
        type=Path,
        default=(
            DEFAULT_ARTIFACTS_DIR
        ),
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    return run_motion_layer_test(
        args
    )


if __name__ == "__main__":
    raise SystemExit(
        main()
    )
