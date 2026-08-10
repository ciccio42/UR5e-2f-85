from __future__ import annotations

import argparse
import numpy as np

from ai_controller.utils.utils import (
    EEF_POS_NAME,
    EEF_QUAT_NAME,
)
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

    if not controller.primitive_plan.steps:
        raise AssertionError(
            "SeeDoController generated an empty PrimitivePlan."
        )

    print("\n=== VALIDATING PRIMITIVE PLAN ===")

    for index, primitive_step in enumerate(
        controller.primitive_plan.steps,
        start=1,
    ):
        print(
            f"  t={index}: "
            f"{primitive_step.name}"
            f"({primitive_step.arguments})"
        )

    print("\n=== INITIALIZING MOTION RUNTIME STATE ===")

    robot_state = {
        EEF_POS_NAME: np.array(
            [
                -0.15552094619366708,
                0.34869994018501943,
                0.1532803451753288,
            ],
            dtype=np.float64,
        ),
        EEF_QUAT_NAME: np.array(
            [
                0.9994452044624775,
                0.03161651380119412,
                0.0021438049655468088,
                0.010251021036213035,
            ],
            dtype=np.float64,
        ),
    }

    motion_runtime_input = dict(
        runtime_input
    )

    motion_runtime_input["robot_state"] = (
        robot_state
    )

    print("\n=== TRANSLATING PRIMITIVE PLAN ===")

    returned_actions = []

    for t in range(
        1,
        len(controller.primitive_plan.steps) + 1,
    ):
        primitive_step = (
            controller.primitive_plan.steps[
                t - 1
            ]
        )

        actions = controller.inference(
            input_data=motion_runtime_input,
            t=t,
        )

        if actions is None:
            raise AssertionError(
                "SeeDoController returned None before all "
                f"primitive steps were translated at t={t}."
            )

        if not isinstance(actions, list):
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
            if not isinstance(action, np.ndarray):
                raise AssertionError(
                    "SeeDoController returned a non-array action "
                    f"for primitive {primitive_step.name!r}, "
                    f"action index {action_index}: "
                    f"{type(action).__name__}."
                )

            if action.shape != (8,):
                raise AssertionError(
                    "SeeDo low-level action has invalid shape "
                    f"{action.shape} for primitive "
                    f"{primitive_step.name!r}; expected (8,)."
                )

            if not np.all(np.isfinite(action)):
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

    completion_t = (
        len(controller.primitive_plan.steps)
        + 1
    )

    completion_result = controller.inference(
        input_data=motion_runtime_input,
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
