from __future__ import annotations

import argparse

import rclpy

from ai_controller.ai_controller_node import AIControllerNode
from ai_controller.models.seedo_controller.tests.common import (
    load_scene_runtime_input,
)


def run_test(args: argparse.Namespace) -> int:
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "ai_controller_target:=seedo_controller",
            "-p",
            f"model_config_path:={args.model_config}",
            "-p",
            "move_robot:=False",
        ]
    )

    node = None

    try:
        print("=== INITIALIZING AI CONTROLLER NODE ===")

        node = AIControllerNode()

        if node.ai_controller_target != "seedo_controller":
            raise AssertionError(
                "AIControllerNode is not configured to use "
                "seedo_controller."
            )

        print("AIControllerNode initialized successfully")

        print("\n=== LOADING DEMONSTRATION ===")

        node.controller.reset()

        node.controller.load_command(
            demo_path=args.video,
            task_id=args.task_id,
            artifacts_dir=args.artifacts_dir,
        )

        if node.controller.action_plan is None:
            raise AssertionError(
                "SeeDoController did not generate an ActionPlan."
            )

        if node.controller.execution_status != "plan_ready":
            raise AssertionError(
                "Unexpected controller status after "
                "load_command(): "
                f"{node.controller.execution_status}"
            )

        print(
            "Action plan: "
            f"{node.controller.action_plan.natural_language_plan}"
        )

        print("\n=== LOADING OFFLINE RUNTIME SCENE ===")

        runtime_input = load_scene_runtime_input(args)

        runtime_input = node._build_seedo_runtime_input(
            rgb_image=runtime_input["rgb"],
            depth_image=runtime_input["depth"],
            camera_info=runtime_input["camera_info"],
            base_to_table_transform=runtime_input[
                "base_to_table_transform"
            ],
        )

        print("\n=== RUNNING t=0 THROUGH NODE CONTROLLER ===")

        result = node.controller.inference(
            input_data=runtime_input,
            t=0,
        )

        if result is not None:
            raise AssertionError(
                "SeeDo inference(t=0) must return None."
            )

        if node.controller.scene_state is None:
            raise AssertionError(
                "SeeDoController did not generate a SceneState."
            )

        if node.controller.primitive_plan is None:
            raise AssertionError(
                "SeeDoController did not generate a PrimitivePlan."
            )

        if node.controller.execution_status != "plan_ready":
            raise AssertionError(
                "Unexpected controller status after t=0: "
                f"{node.controller.execution_status}"
            )

        primitive_count = len(
            node.controller.primitive_plan.steps
        )

        if primitive_count == 0:
            raise AssertionError(
                "SeeDo generated an empty PrimitivePlan."
            )

        print(
            f"Scene objects: "
            f"{len(node.controller.scene_state.objects)}"
        )
        print(f"Primitive count: {primitive_count}")

        print("\n=== CONSUMING PRIMITIVE PLAN ===")

        trajectory_actions = []

        for t in range(1, primitive_count + 1):
            primitive_step = node.controller.inference(
                input_data={},
                t=t,
            )

            if primitive_step is None:
                raise AssertionError(
                    "SeeDo returned None before the "
                    f"PrimitivePlan was consumed at t={t}."
                )

            action = {
                "name": primitive_step.name,
                "arguments": dict(
                    primitive_step.arguments
                ),
                "source_code": primitive_step.source_code,
            }

            trajectory_actions.append(action)

            print(
                f"  t={t}: "
                f"{action['name']}"
                f"({action['arguments']})"
            )

        print("\n=== CHECKING COMPLETION ===")

        completion_result = node.controller.inference(
            input_data={},
            t=primitive_count + 1,
        )

        if completion_result is not None:
            raise AssertionError(
                "SeeDo must return None after the "
                "PrimitivePlan has been consumed."
            )

        if node.controller.execution_status != "completed":
            raise AssertionError(
                "SeeDoController did not enter completed state. "
                f"Current status: "
                f"{node.controller.execution_status}"
            )

        if len(trajectory_actions) != primitive_count:
            raise AssertionError(
                "The number of serialized actions does not "
                "match the PrimitivePlan."
            )

        print(
            f"Serialized actions: {len(trajectory_actions)}"
        )

        print("\nTEST PASSED")
        return 0

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--video",
        required=True,
    )

    parser.add_argument(
        "--scene-dir",
        required=True,
    )

    parser.add_argument(
        "--base-to-table-transform",
        required=True,
    )

    parser.add_argument(
        "--artifacts-dir",
        default=None,
    )

    parser.add_argument(
        "--task-id",
        default="test",
    )

    parser.add_argument(
        "--model-config",
        required=True,
    )

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    return run_test(args)


if __name__ == "__main__":
    raise SystemExit(main())