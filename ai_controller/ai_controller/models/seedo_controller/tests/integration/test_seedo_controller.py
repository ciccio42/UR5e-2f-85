from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from ai_controller.utils.utils import (
    EEF_POS_NAME,
    EEF_QUAT_NAME,
)
from ..common import load_scene_runtime_input
from ai_controller.models.seedo_controller.seedo_controller import (
    SeeDoController,
)


EXPECTED_VIDEO = Path(
    "/test_dataset/pick_place/human_rgb_pick_place/"
    "task_00/traj000/converted/traj000-h264-30fps.mp4"
)

EXPECTED_SCENE_DIR = Path(
    "/scene_capture/without_distractors/"
    "scene_1_no_distractors"
)

EXPECTED_RUNTIME_OBJECT_IDS = {
    "storage_bin_0",
    "storage_bin_1",
    "storage_bin_2",
    "storage_bin_3",
    "green_block_0",
    "yellow_block_0",
    "blue_block_0",
    "red_block_0",
}

EXPECTED_DEMO_PLACE_IDS = {
    "0",
    "1",
    "2",
    "3",
}

EXPECTED_RUNTIME_PLACE_IDS = {
    "storage_bin_0",
    "storage_bin_1",
    "storage_bin_2",
    "storage_bin_3",
}

EXPECTED_STRUCTURAL_MAPPING = {
    "0": "storage_bin_0",
    "1": "storage_bin_1",
    "2": "storage_bin_2",
    "3": "storage_bin_3",
}

EXPECTED_PRIMITIVES = (
    ("reach", "green_block_0"),
    ("approaching", "green_block_0"),
    ("pick", "green_block_0"),
    ("lift_up", "green_block_0"),
    ("moving", "storage_bin_0"),
    ("placing", "storage_bin_0"),
)

INITIAL_POSITION = np.array(
    [
        -0.15552094619366708,
        0.34869994018501943,
        0.1532803451753288,
    ],
    dtype=np.float64,
)

INITIAL_ORIENTATION = np.array(
    [
        0.9994452044624775,
        0.03161651380119412,
        0.0021438049655468088,
        0.010251021036213035,
    ],
    dtype=np.float64,
)


def _normalize_task_type(
    value,
) -> str:
    raw_value = getattr(
        value,
        "value",
        value,
    )

    return str(
        raw_value
    ).strip().lower()


def _require_file(
    path: Path,
) -> None:
    if not path.is_file():
        raise AssertionError(
            f"Expected artifact was not created: {path}"
        )

    if path.stat().st_size == 0:
        raise AssertionError(
            f"Expected artifact is empty: {path}"
        )


def _load_json(
    path: Path,
) -> dict:
    _require_file(
        path
    )

    with path.open(
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
        raise AssertionError(
            f"Expected JSON object in artifact: {path}"
        )

    return payload


def _validate_canonical_inputs(
    args: argparse.Namespace,
) -> None:
    if args.video is None:
        raise ValueError(
            "--video is required for the seedo_controller test."
        )

    if not args.model_config:
        raise ValueError(
            "--model-config is required for the seedo_controller test."
        )

    if args.scene_dir is None:
        raise ValueError(
            "--scene-dir is required for the seedo_controller test."
        )

    if args.base_to_table_transform is None:
        raise ValueError(
            "--base-to-table-transform is required for the "
            "seedo_controller test."
        )

    video_path = (
        Path(args.video)
        .expanduser()
        .resolve()
    )

    expected_video = (
        EXPECTED_VIDEO
        .expanduser()
        .resolve()
    )

    if video_path != expected_video:
        raise ValueError(
            "This end-to-end integration test is pinned to the "
            "canonical pick-and-place demonstration. "
            f"Expected {expected_video}, received {video_path}."
        )

    scene_dir = (
        Path(args.scene_dir)
        .expanduser()
        .resolve()
    )

    expected_scene_dir = (
        EXPECTED_SCENE_DIR
        .expanduser()
        .resolve()
    )

    if scene_dir != expected_scene_dir:
        raise ValueError(
            "This end-to-end integration test is pinned to the "
            "canonical no-distractors runtime scene. "
            f"Expected {expected_scene_dir}, received {scene_dir}."
        )

    expected_transform = (
        expected_scene_dir
        / "base_to_table_transform.yaml"
    ).resolve()

    provided_transform = (
        Path(
            args.base_to_table_transform
        )
        .expanduser()
        .resolve()
    )

    if provided_transform != expected_transform:
        raise ValueError(
            "The runtime transform must belong to the canonical "
            "runtime scene. "
            f"Expected {expected_transform}, "
            f"received {provided_transform}."
        )


def _validate_demo_pipeline(
    controller: SeeDoController,
) -> None:
    if controller.action_plan is None:
        raise AssertionError(
            "SeeDoController did not generate an ActionPlanningResult."
        )

    if controller.demo_structured_scene is None:
        raise AssertionError(
            "Generalized SeeDoController did not generate the "
            "demonstration StructuredScene."
        )

    if controller.execution_status != "plan_ready":
        raise AssertionError(
            "Unexpected controller status after load_command(): "
            f"{controller.execution_status}"
        )

    action_plan = (
        controller.action_plan
    )

    if action_plan.status != "completed":
        raise AssertionError(
            "Expected a completed demonstration action plan, "
            f"received {action_plan.status!r}."
        )

    if _normalize_task_type(
        action_plan.task_type
    ) != "pick_and_place":
        raise AssertionError(
            "Unexpected demonstration task type: "
            f"{action_plan.task_type!r}"
        )

    if len(
        action_plan.steps
    ) != 1:
        raise AssertionError(
            "Expected exactly one demonstrated action step, "
            f"received {len(action_plan.steps)}."
        )

    step = (
        action_plan.steps[0]
    )

    if step.picked_detector_label != "green block":
        raise AssertionError(
            "Unexpected demonstrated picked detector label: "
            f"{step.picked_detector_label!r}"
        )

    if step.destination_track_id != 0:
        raise AssertionError(
            "Unexpected demonstrated destination track ID: "
            f"{step.destination_track_id}"
        )

    if (
        str(
            step.destination_category
        ).strip().lower()
        != "bin"
    ):
        raise AssertionError(
            "Unexpected demonstrated destination category: "
            f"{step.destination_category!r}"
        )

    if (
        str(
            step.relation
        ).strip().lower()
        != "in"
    ):
        raise AssertionError(
            "Unexpected demonstrated relation: "
            f"{step.relation!r}"
        )

    demo_scene = (
        controller.demo_structured_scene
    )

    if demo_scene.directions != 8:
        raise AssertionError(
            "Unexpected demonstration structured-scene "
            f"direction mode: {demo_scene.directions}"
        )

    demo_object_ids = {
        obj.object_id
        for obj
        in demo_scene.objects
    }

    if (
        demo_object_ids
        != EXPECTED_DEMO_PLACE_IDS
    ):
        raise AssertionError(
            "Unexpected demonstration place-object IDs: "
            f"{sorted(demo_object_ids)}"
        )

    if len(
        demo_scene.relations
    ) != 12:
        raise AssertionError(
            "Expected 12 demonstration structural relations, "
            f"received {len(demo_scene.relations)}."
        )


def _validate_runtime_pipeline(
    controller: SeeDoController,
) -> None:
    if controller.perception_result is None:
        raise AssertionError(
            "SeeDoController did not generate a ScenePerceptionResult."
        )

    if controller.scene_state is None:
        raise AssertionError(
            "SeeDoController did not generate a SceneState."
        )

    if controller.runtime_structured_scene is None:
        raise AssertionError(
            "SeeDoController did not generate the runtime StructuredScene."
        )

    if controller.structural_matching_result is None:
        raise AssertionError(
            "SeeDoController did not generate a StructuralMatchingResult."
        )

    if controller.replicability_result is None:
        raise AssertionError(
            "SeeDoController did not generate a ReplicabilityResult."
        )

    if controller.primitive_plan is None:
        raise AssertionError(
            "SeeDoController did not generate a PrimitivePlan."
        )

    if controller.execution_status != "plan_ready":
        raise AssertionError(
            "Unexpected controller status after inference(t=0): "
            f"{controller.execution_status}"
        )

    scene_state = (
        controller.scene_state
    )

    runtime_ids = {
        obj.object_id
        for obj
        in scene_state.objects
    }

    if (
        runtime_ids
        != EXPECTED_RUNTIME_OBJECT_IDS
    ):
        raise AssertionError(
            "Unexpected generalized runtime SceneState IDs: "
            f"expected={sorted(EXPECTED_RUNTIME_OBJECT_IDS)}, "
            f"received={sorted(runtime_ids)}"
        )

    runtime_structured_scene = (
        controller.runtime_structured_scene
    )

    runtime_place_ids = {
        obj.object_id
        for obj
        in runtime_structured_scene.objects
    }

    if (
        runtime_place_ids
        != EXPECTED_RUNTIME_PLACE_IDS
    ):
        raise AssertionError(
            "Unexpected runtime structured-scene destination IDs: "
            f"{sorted(runtime_place_ids)}"
        )

    if len(
        runtime_structured_scene.relations
    ) != 12:
        raise AssertionError(
            "Expected 12 runtime structural relations, "
            f"received {len(runtime_structured_scene.relations)}."
        )

    matching_result = (
        controller.structural_matching_result
    )

    if not matching_result.is_valid:
        raise AssertionError(
            "Structural matching returned no valid mapping."
        )

    if not matching_result.is_unique:
        raise AssertionError(
            "Expected a unique structural mapping in the "
            "canonical runtime scene."
        )

    if len(
        matching_result.valid_mappings
    ) != 1:
        raise AssertionError(
            "Expected exactly one valid structural mapping, "
            f"received {len(matching_result.valid_mappings)}."
        )

    mapping = {
        match.demo_object_id:
            match.runtime_object_id
        for match
        in matching_result
        .valid_mappings[0]
        .matches
    }

    if mapping != EXPECTED_STRUCTURAL_MAPPING:
        raise AssertionError(
            "Unexpected controller structural mapping: "
            f"expected={EXPECTED_STRUCTURAL_MAPPING}, "
            f"received={mapping}"
        )

    replicability_result = (
        controller.replicability_result
    )

    if not replicability_result.replicable:
        raise AssertionError(
            "Canonical task should be replicable: "
            f"{replicability_result.failure_reasons}"
        )

    if replicability_result.failure_reasons:
        raise AssertionError(
            "Replicable controller result contains failure reasons: "
            f"{replicability_result.failure_reasons}"
        )

    if len(
        replicability_result.resolved_targets
    ) != 1:
        raise AssertionError(
            "Expected exactly one resolved runtime target pair, "
            f"received "
            f"{len(replicability_result.resolved_targets)}."
        )

    resolved = (
        replicability_result
        .resolved_targets[0]
    )

    if resolved.action_step_index != 0:
        raise AssertionError(
            "Unexpected resolved action-step index: "
            f"{resolved.action_step_index}"
        )

    if (
        resolved.runtime_pick_object_id
        != "green_block_0"
    ):
        raise AssertionError(
            "Unexpected resolved runtime pick target: "
            f"{resolved.runtime_pick_object_id!r}"
        )

    if (
        resolved.runtime_place_object_id
        != "storage_bin_0"
    ):
        raise AssertionError(
            "Unexpected resolved runtime place target: "
            f"{resolved.runtime_place_object_id!r}"
        )

    primitive_plan = (
        controller.primitive_plan
    )

    if len(
        primitive_plan.steps
    ) != len(
        EXPECTED_PRIMITIVES
    ):
        raise AssertionError(
            "Unexpected number of CAP primitives: "
            f"expected={len(EXPECTED_PRIMITIVES)}, "
            f"received={len(primitive_plan.steps)}"
        )

    for index, (
        primitive_step,
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
        if primitive_step.name != expected_name:
            raise AssertionError(
                f"Primitive {index}: expected {expected_name!r}, "
                f"received {primitive_step.name!r}."
            )

        if primitive_step.arguments != {
            "target": expected_target,
        }:
            raise AssertionError(
                f"Primitive {index}: expected target "
                f"{expected_target!r}, received "
                f"{primitive_step.arguments!r}."
            )

    if (
        "green_block_0"
        not in primitive_plan.source_code
    ):
        raise AssertionError(
            "CAP source code does not reference the resolved "
            "runtime pick ID."
        )

    if (
        "storage_bin_0"
        not in primitive_plan.source_code
    ):
        raise AssertionError(
            "CAP source code does not reference the resolved "
            "runtime place ID."
        )


def _validate_pipeline_artifacts(
    artifacts_dir: Path,
) -> None:
    required_files = (
        artifacts_dir
        / "demo_structured_scene"
        / "demo_structured_scene.json",
        artifacts_dir
        / "action_planning"
        / "action_plan.json",
        artifacts_dir
        / "scene_perceiver"
        / "raw_scene_state.json",
        artifacts_dir
        / "scene_interpreter"
        / "scene_state.json",
        artifacts_dir
        / "runtime_structured_scene"
        / "runtime_structured_scene.json",
        artifacts_dir
        / "structural_matching"
        / "structural_matching_result.json",
        artifacts_dir
        / "replicability_check"
        / "replicability_result.json",
        artifacts_dir
        / "lmp_generator"
        / "generated_program.py",
        artifacts_dir
        / "lmp_generator"
        / "primitive_plan.json",
    )

    for artifact_path in (
        required_files
    ):
        _require_file(
            artifact_path
        )


def _validate_motion_artifact(
    *,
    artifacts_dir: Path,
    controller: SeeDoController,
    returned_actions: list[np.ndarray],
) -> None:
    motion_plan_path = (
        artifacts_dir
        / "motion_layer"
        / "motion_plan.json"
    )

    payload = _load_json(
        motion_plan_path
    )

    if (
        payload.get(
            "total_actions"
        )
        != len(
            returned_actions
        )
    ):
        raise AssertionError(
            "Motion Layer artifact total_actions does not match "
            "the actions returned by SeeDoController. "
            f"artifact={payload.get('total_actions')}, "
            f"returned={len(returned_actions)}"
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
        controller.primitive_plan.steps
    ):
        raise AssertionError(
            "motion_plan.json primitive count does not match "
            "the controller PrimitivePlan."
        )

    for (
        primitive_step,
        artifact_primitive,
    ) in zip(
        controller.primitive_plan.steps,
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

    if (
        float(
            final_state[
                "gripper_position"
            ]
        )
        != 0.1
    ):
        raise AssertionError(
            "Expected final gripper position 0.1, received "
            f"{final_state['gripper_position']}."
        )


def run_seedo_controller_test(
    args: argparse.Namespace,
) -> int:
    """Run the complete generalized SeeDo pipeline from demo to low-level actions."""

    _validate_canonical_inputs(
        args
    )

    controller = SeeDoController(
        model_config=args.model_config,
    )

    if controller.perception_mode != "generalized":
        raise AssertionError(
            "This end-to-end integration test requires "
            f"perception_mode='generalized', received "
            f"{controller.perception_mode!r}."
        )

    persistent_artifacts = (
        args.artifacts_dir
        is not None
    )

    print(
        "=== RUNNING COMPLETE DEMONSTRATION PIPELINE ==="
    )

    controller.load_command(
        demo_path=args.video,
        task_id="test",
        artifacts_dir=args.artifacts_dir,
    )

    if controller.artifacts_dir is None:
        raise AssertionError(
            "SeeDoController did not initialize artifact storage."
        )

    artifacts_dir_before_reset = (
        controller.artifacts_dir
    )

    if not artifacts_dir_before_reset.is_dir():
        raise AssertionError(
            "Controller artifact directory does not exist: "
            f"{artifacts_dir_before_reset}"
        )

    print(
        "Artifact mode: "
        + (
            "persistent"
            if persistent_artifacts
            else "temporary"
        )
    )

    print(
        "Artifact directory: "
        f"{artifacts_dir_before_reset}"
    )

    _validate_demo_pipeline(
        controller
    )

    print(
        "Demonstration plan: "
        f"{controller.action_plan.natural_language_plan}"
    )

    print(
        "Demo structured objects: "
        f"{len(controller.demo_structured_scene.objects)}"
    )

    print()
    print(
        "=== RUNNING COMPLETE RUNTIME PIPELINE ==="
    )

    runtime_input = (
        load_scene_runtime_input(
            args
        )
    )

    result = (
        controller.inference(
            input_data=runtime_input,
            t=0,
        )
    )

    if result is not None:
        raise AssertionError(
            "inference(t=0) must return None."
        )

    _validate_runtime_pipeline(
        controller
    )

    _validate_pipeline_artifacts(
        artifacts_dir_before_reset
    )

    print(
        "Runtime scene objects: "
        f"{len(controller.scene_state.objects)}"
    )

    print(
        "Structural mappings: "
        f"{len(controller.structural_matching_result.valid_mappings)}"
    )

    resolved = (
        controller.replicability_result
        .resolved_targets[0]
    )

    print(
        "Resolved runtime targets: "
        f"pick={resolved.runtime_pick_object_id}, "
        f"place={resolved.runtime_place_object_id}"
    )

    print()
    print(
        "=== VALIDATING PRIMITIVE PLAN ==="
    )

    for index, primitive_step in enumerate(
        controller.primitive_plan.steps,
        start=1,
    ):
        print(
            f"  t={index}: "
            f"{primitive_step.name}"
            f"({primitive_step.arguments})"
        )

    print()
    print(
        "=== TRANSLATING COMPLETE PRIMITIVE PLAN ==="
    )

    robot_state = {
        EEF_POS_NAME: (
            INITIAL_POSITION.copy()
        ),
        EEF_QUAT_NAME: (
            INITIAL_ORIENTATION.copy()
        ),
    }

    motion_runtime_input = dict(
        runtime_input
    )

    motion_runtime_input[
        "robot_state"
    ] = robot_state

    returned_actions: list[
        np.ndarray
    ] = []

    for t in range(
        1,
        len(
            controller.primitive_plan.steps
        )
        + 1,
    ):
        primitive_step = (
            controller
            .primitive_plan
            .steps[
                t - 1
            ]
        )

        actions = (
            controller.inference(
                input_data=(
                    motion_runtime_input
                ),
                t=t,
            )
        )

        if actions is None:
            raise AssertionError(
                "SeeDoController returned None before all "
                f"primitive steps were translated at t={t}."
            )

        if not isinstance(
            actions,
            list,
        ):
            raise AssertionError(
                "SeeDoController inference() must return a list "
                f"of actions at t={t}; received "
                f"{type(actions).__name__}."
            )

        if not actions:
            raise AssertionError(
                "SeeDoController returned an empty action list "
                f"for primitive {primitive_step.name!r} at t={t}."
            )

        for action_index, action in enumerate(
            actions
        ):
            if not isinstance(
                action,
                np.ndarray,
            ):
                raise AssertionError(
                    "SeeDoController returned a non-array action "
                    f"for primitive {primitive_step.name!r}, "
                    f"action index {action_index}: "
                    f"{type(action).__name__}."
                )

            if action.shape != (
                8,
            ):
                raise AssertionError(
                    "SeeDo low-level action has invalid shape "
                    f"{action.shape} for primitive "
                    f"{primitive_step.name!r}; expected (8,)."
                )

            if not np.all(
                np.isfinite(
                    action
                )
            ):
                raise AssertionError(
                    "SeeDo low-level action contains non-finite "
                    f"values for primitive "
                    f"{primitive_step.name!r}: {action}"
                )

        returned_actions.extend(
            actions
        )

        print(
            f"  t={t}: "
            f"{primitive_step.name}"
            f"({primitive_step.arguments}) "
            f"-> {len(actions)} low-level action(s)"
        )

    if not returned_actions:
        raise AssertionError(
            "SeeDoController did not produce any low-level actions."
        )

    print(
        "\nTotal low-level actions generated: "
        f"{len(returned_actions)}"
    )

    _validate_motion_artifact(
        artifacts_dir=(
            artifacts_dir_before_reset
        ),
        controller=controller,
        returned_actions=(
            returned_actions
        ),
    )

    completion_t = (
        len(
            controller.primitive_plan.steps
        )
        + 1
    )

    completion_result = (
        controller.inference(
            input_data=(
                motion_runtime_input
            ),
            t=completion_t,
        )
    )

    if completion_result is not None:
        raise AssertionError(
            "SeeDoController must return None after the "
            "primitive plan has been consumed."
        )

    if (
        controller.execution_status
        != "completed"
    ):
        raise AssertionError(
            "Controller did not enter completed state. "
            f"Current status: "
            f"{controller.execution_status}"
        )

    print()
    print(
        "=== RESETTING CONTROLLER ==="
    )

    controller.reset()

    state_attributes = {
        "perception_result":
            controller.perception_result,
        "action_plan":
            controller.action_plan,
        "scene_state":
            controller.scene_state,
        "primitive_plan":
            controller.primitive_plan,
        "demo_structured_scene":
            controller.demo_structured_scene,
        "runtime_structured_scene":
            controller.runtime_structured_scene,
        "structural_matching_result":
            controller.structural_matching_result,
        "replicability_result":
            controller.replicability_result,
    }

    uncleared = {
        name: value
        for name, value
        in state_attributes.items()
        if value is not None
    }

    if uncleared:
        raise AssertionError(
            "reset() did not clear controller pipeline state: "
            f"{list(uncleared)}"
        )

    if (
        controller.execution_status
        != "idle"
    ):
        raise AssertionError(
            "reset() did not restore idle status."
        )

    if controller.execution_error is not None:
        raise AssertionError(
            "reset() did not clear execution_error."
        )

    if controller.artifacts_dir is not None:
        raise AssertionError(
            "reset() did not clear the controller "
            "artifact directory reference."
        )

    if persistent_artifacts:
        if not (
            artifacts_dir_before_reset
            .is_dir()
        ):
            raise AssertionError(
                "Persistent artifact directory was unexpectedly "
                "removed by reset(): "
                f"{artifacts_dir_before_reset}"
            )

        print(
            "Persistent artifacts preserved: "
            f"{artifacts_dir_before_reset}"
        )

    else:
        if (
            artifacts_dir_before_reset
            .exists()
        ):
            raise AssertionError(
                "Temporary artifact directory was not removed "
                "by reset(): "
                f"{artifacts_dir_before_reset}"
            )

        print(
            "Temporary artifacts cleaned successfully: "
            f"{artifacts_dir_before_reset}"
        )

    print()
    print(
        "END-TO-END SEEDO CONTROLLER TEST PASSED"
    )

    return 0
