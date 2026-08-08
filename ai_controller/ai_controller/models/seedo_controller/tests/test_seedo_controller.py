from __future__ import annotations

import argparse

from .common import load_scene_runtime_input
from ai_controller.models.seedo_controller.seedo_controller import SeeDoController

def run_seedo_controller_test(
    args: argparse.Namespace,
) -> int:
    if args.video is None:
        raise ValueError(
            "--video is required for the integration test."
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

    controller = SeeDoController(
        model_config=args.model_config,
    )

    persistent_artifacts = (
        args.artifacts_dir is not None
    )

    print("=== LOADING DEMONSTRATION ===")

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

    if persistent_artifacts:
        print(
            "Artifact mode: persistent"
        )
    else:
        print(
            "Artifact mode: temporary"
        )

    print(
        "Artifact directory: "
        f"{artifacts_dir_before_reset}"
    )

    if controller.action_plan is None:
        raise AssertionError(
            "SeeDoController did not generate an action plan."
        )

    if controller.execution_status != "plan_ready":
        raise AssertionError(
            "Unexpected controller status after load_command(): "
            f"{controller.execution_status}"
        )

    print(
        "Action plan: "
        f"{controller.action_plan.natural_language_plan}"
    )

    print("\n=== LOADING RUNTIME SCENE ===")

    runtime_input = load_scene_runtime_input(
        args
    )

    print("\n=== RUNNING INITIAL INFERENCE ===")

    result = controller.inference(
        input_data=runtime_input,
        t=0,
    )

    if result is not None:
        raise AssertionError(
            "inference(t=0) must return None."
        )

    if controller.scene_state is None:
        raise AssertionError(
            "SeeDoController did not generate a semantic scene."
        )

    if controller.primitive_plan is None:
        raise AssertionError(
            "SeeDoController did not generate a primitive plan."
        )

    if controller.execution_status != "plan_ready":
        raise AssertionError(
            "Unexpected controller status after inference(t=0): "
            f"{controller.execution_status}"
        )

    print(
        f"Scene objects: "
        f"{len(controller.scene_state.objects)}"
    )

    print(
        "Primitive count: "
        f"{len(controller.primitive_plan.steps)}"
    )

    print("\n=== CONSUMING PRIMITIVE PLAN ===")

    returned_steps = []

    for t in range(
        1,
        len(controller.primitive_plan.steps) + 1,
    ):
        step = controller.inference(
            input_data=runtime_input,
            t=t,
        )

        if step is None:
            raise AssertionError(
                "SeeDoController returned None before all "
                f"primitive steps were consumed at t={t}."
            )

        expected_step = (
            controller.primitive_plan.steps[
                t - 1
            ]
        )

        if step != expected_step:
            raise AssertionError(
                "Returned primitive does not match the "
                f"generated plan at t={t}."
            )

        returned_steps.append(
            step
        )

        print(
            f"  t={t}: "
            f"{step.name}({step.arguments})"
        )

    if (
        tuple(returned_steps)
        != controller.primitive_plan.steps
    ):
        raise AssertionError(
            "The primitive sequence returned by inference() "
            "does not match PrimitivePlan."
        )

    completion_t = (
        len(controller.primitive_plan.steps)
        + 1
    )

    completion_result = controller.inference(
        input_data=runtime_input,
        t=completion_t,
    )

    if completion_result is not None:
        raise AssertionError(
            "SeeDoController must return None after the "
            "primitive plan has been consumed."
        )

    if controller.execution_status != "completed":
        raise AssertionError(
            "Controller did not enter completed state. "
            f"Current status: {controller.execution_status}"
        )

    print("\n=== RESETTING CONTROLLER ===")

    controller.reset()

    if controller.action_plan is not None:
        raise AssertionError(
            "reset() did not clear action_plan."
        )

    if controller.scene_state is not None:
        raise AssertionError(
            "reset() did not clear scene_state."
        )

    if controller.primitive_plan is not None:
        raise AssertionError(
            "reset() did not clear primitive_plan."
        )

    if controller.execution_status != "idle":
        raise AssertionError(
            "reset() did not restore idle status."
        )

    if controller.artifacts_dir is not None:
        raise AssertionError(
            "reset() did not clear the controller "
            "artifact directory reference."
        )

    if persistent_artifacts:
        if not artifacts_dir_before_reset.is_dir():
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
        if artifacts_dir_before_reset.exists():
            raise AssertionError(
                "Temporary artifact directory was not removed "
                "by reset(): "
                f"{artifacts_dir_before_reset}"
            )

        print(
            "Temporary artifacts cleaned successfully: "
            f"{artifacts_dir_before_reset}"
        )

    print("\nTEST PASSED")

    return 0
