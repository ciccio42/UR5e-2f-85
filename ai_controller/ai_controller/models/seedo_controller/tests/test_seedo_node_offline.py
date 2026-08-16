from __future__ import annotations

import argparse
from unittest.mock import patch

import numpy as np
import rclpy

import ai_controller.ai_controller_node as ai_controller_node_module

from ai_controller.ai_controller_node import AIControllerNode
from ai_controller.models.seedo_controller.tests.common import (
    load_scene_runtime_input,
)
from ai_controller.utils.utils import (
    EEF_POS_NAME,
    EEF_QUAT_NAME,
)


INITIAL_EEF_POSITION = np.array(
    [
        -0.15552094619366708,
        0.34869994018501943,
        0.1532803451753288,
    ],
    dtype=np.float64,
)

INITIAL_EEF_ORIENTATION = np.array(
    [
        0.9994452044624775,
        0.03161651380119412,
        0.0021438049655468088,
        0.010251021036213035,
    ],
    dtype=np.float64,
)


class _ControlLoopFinished(Exception):
    """Used by the offline test to stop control_loop after one trajectory."""


class _OfflineTrajectory:
    """Minimal trajectory implementation used by the offline node test."""

    def __init__(self) -> None:
        self.entries: list[dict] = []

    def append(
        self,
        obs,
        action,
        done,
        reward,
    ) -> None:
        self.entries.append(
            {
                "obs": obs,
                "action": action,
                "done": done,
                "reward": reward,
            }
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

        if node.move_robot:
            raise AssertionError(
                "Offline SeeDo node test must run with move_robot=False."
            )

        node.demo_path = args.video

        print("AIControllerNode initialized successfully")

        print("\n=== LOADING OFFLINE RUNTIME SCENE ===")

        scene_runtime_input = load_scene_runtime_input(
            args
        )

        seedo_runtime_input = node._build_seedo_runtime_input(
            rgb_image=scene_runtime_input["rgb"],
            depth_image=scene_runtime_input["depth"],
            camera_info=scene_runtime_input["camera_info"],
            base_to_table_transform=scene_runtime_input[
                "base_to_table_transform"
            ],
        )

        if "rgb" not in seedo_runtime_input:
            raise AssertionError(
                "Offline SeeDo runtime input does not contain RGB data."
            )

        print("Offline runtime scene loaded successfully")

        print("\n=== PREPARING OFFLINE ROBOT STATE ===")

        robot_state = {
            EEF_POS_NAME: INITIAL_EEF_POSITION.copy(),
            EEF_QUAT_NAME: INITIAL_EEF_ORIENTATION.copy(),
        }

        print(
            "EEF position: "
            f"{robot_state[EEF_POS_NAME]}"
        )

        print(
            "EEF orientation: "
            f"{robot_state[EEF_QUAT_NAME]}"
        )

        #
        # Capture the real controller.inference() outputs while leaving
        # control_loop() responsible for invoking inference.
        #
        original_inference = node.controller.inference

        inference_results: dict[
            int,
            object,
        ] = {}

        def tracked_inference(
            input_data,
            t=0,
            save_path=None,
        ):
            result = original_inference(
                input_data=input_data,
                t=t,
                save_path=save_path,
            )

            inference_results[t] = result

            return result

        saved_trajectory = None

        def fake_save_rollout(
            traj,
            save_path,
            task_id,
            traj_number,
        ):
            nonlocal saved_trajectory

            saved_trajectory = traj

            print(
                "\n=== CONTROL LOOP REACHED SAVE_ROLLOUT ==="
            )

            raise _ControlLoopFinished()

        #
        # control_loop() asks for:
        #
        #   1. trajectory count
        #   2. Enter before starting
        #   3. task ID
        #
        # These values simulate one complete interactive iteration.
        #
        input_values = iter(
            [
                "0",
                "",
                args.task_id,
            ]
        )

        def fake_input(prompt=""):
            try:
                value = next(input_values)
            except StopIteration as exc:
                raise AssertionError(
                    "control_loop() requested more interactive "
                    "inputs than expected."
                ) from exc

            print(
                f"[offline input] {prompt}{value}"
            )

            return value

        #
        # get_synced_images() is still used by control_loop() for each
        # primitive timestep. Reuse the captured offline RGB image.
        #
        def fake_get_synced_images():
            return [
                seedo_runtime_input["rgb"]
            ]

        def fake_capture_robot_state():
            return {
                EEF_POS_NAME: (
                    robot_state[
                        EEF_POS_NAME
                    ].copy()
                ),
                EEF_QUAT_NAME: (
                    robot_state[
                        EEF_QUAT_NAME
                    ].copy()
                ),
            }

        def fake_wait_for_seedo_runtime_data(
            *args,
            **kwargs,
        ):
            return None

        def fake_get_seedo_base_to_table_transform():
            return scene_runtime_input[
                "base_to_table_transform"
            ]

        def fake_get_seedo_runtime_input(
            base_to_table_transform,
        ):
            return seedo_runtime_input

        print("\n=== RUNNING REAL CONTROL LOOP ===")

        with (
            patch(
                "builtins.input",
                side_effect=fake_input,
            ),
            patch.object(
                ai_controller_node_module,
                "_get_trajectory_cls",
                return_value=_OfflineTrajectory,
            ),
            patch.object(
                node,
                "get_synced_images",
                side_effect=fake_get_synced_images,
            ),
            patch.object(
                node,
                "_capture_robot_state",
                side_effect=fake_capture_robot_state,
            ),
            patch.object(
                node,
                "_wait_for_seedo_runtime_data",
                side_effect=fake_wait_for_seedo_runtime_data,
            ),
            patch.object(
                node,
                "_get_seedo_base_to_table_transform",
                side_effect=(
                    fake_get_seedo_base_to_table_transform
                ),
            ),
            patch.object(
                node,
                "_get_seedo_runtime_input",
                side_effect=fake_get_seedo_runtime_input,
            ),
            patch.object(
                node,
                "save_rollout",
                side_effect=fake_save_rollout,
            ),
            patch.object(
                node.controller,
                "inference",
                side_effect=tracked_inference,
            ),
            patch.object(
                node,
                "_get_seedo_artifacts_dir",
                return_value=args.artifacts_dir,
            ),
        ):
            try:
                node.control_loop()
            except _ControlLoopFinished:
                pass

        print("\n=== VALIDATING CONTROL LOOP ===")

        if node.controller.action_plan is None:
            raise AssertionError(
                "control_loop() did not generate an ActionPlan."
            )

        if node.controller.scene_state is None:
            raise AssertionError(
                "control_loop() did not generate a SceneState."
            )

        if node.controller.primitive_plan is None:
            raise AssertionError(
                "control_loop() did not generate a PrimitivePlan."
            )

        primitive_count = len(
            node.controller.primitive_plan.steps
        )

        if primitive_count == 0:
            raise AssertionError(
                "control_loop() generated an empty PrimitivePlan."
            )

        print(
            f"Primitive count: {primitive_count}"
        )

        #
        # t=0 must have been executed by control_loop().
        #
        if 0 not in inference_results:
            raise AssertionError(
                "control_loop() did not execute SeeDo inference(t=0)."
            )

        if inference_results[0] is not None:
            raise AssertionError(
                "SeeDo inference(t=0) must return None."
            )

        print("[PASS] inference(t=0)")

        print(
            "\n=== VALIDATING MOTION LAYER OUTPUTS ==="
        )

        total_low_level_actions = 0

        for t in range(
            1,
            primitive_count + 1,
        ):
            if t not in inference_results:
                raise AssertionError(
                    "control_loop() did not execute "
                    f"inference(t={t})."
                )

            actions = inference_results[t]

            if actions is None:
                raise AssertionError(
                    "SeeDo returned None before the "
                    f"PrimitivePlan was completed at t={t}."
                )

            if not isinstance(actions, list):
                raise AssertionError(
                    "SeeDo inference() did not return "
                    f"a list at t={t}: "
                    f"{type(actions).__name__}."
                )

            if not actions:
                raise AssertionError(
                    "SeeDo returned an empty low-level "
                    f"action list at t={t}."
                )

            primitive_step = (
                node.controller.primitive_plan.steps[
                    t - 1
                ]
            )

            for action_index, action in enumerate(
                actions
            ):
                if not isinstance(
                    action,
                    np.ndarray,
                ):
                    raise AssertionError(
                        "Invalid low-level action type "
                        f"at t={t}, index={action_index}: "
                        f"{type(action).__name__}."
                    )

                if action.shape != (8,):
                    raise AssertionError(
                        "Invalid low-level action shape "
                        f"at t={t}, index={action_index}: "
                        f"{action.shape}."
                    )

                if not np.all(
                    np.isfinite(action)
                ):
                    raise AssertionError(
                        "Non-finite low-level action "
                        f"at t={t}, index={action_index}: "
                        f"{action}"
                    )

            total_low_level_actions += len(
                actions
            )

            print(
                f"[PASS] t={t}: "
                f"{primitive_step.name}"
                f"({primitive_step.arguments}) "
                f"-> {len(actions)} action(s)"
            )

        print(
            "Total low-level actions processed by "
            f"control_loop(): {total_low_level_actions}"
        )

        #
        # control_loop() explicitly performs one additional inference
        # call after the final primitive to verify completion.
        #
        completion_t = (
            primitive_count
            + 1
        )

        if completion_t not in inference_results:
            raise AssertionError(
                "control_loop() did not perform the final "
                "SeeDo completion inference."
            )

        if inference_results[
            completion_t
        ] is not None:
            raise AssertionError(
                "SeeDo completion inference must return None."
            )

        if (
            node.controller.execution_status
            != "completed"
        ):
            raise AssertionError(
                "SeeDoController did not enter completed state. "
                f"Current status: "
                f"{node.controller.execution_status}"
            )

        print("[PASS] completion state")

        #
        # Verify that control_loop() reached trajectory serialization.
        #
        if saved_trajectory is None:
            raise AssertionError(
                "control_loop() never reached save_rollout()."
            )

        if not isinstance(
            saved_trajectory,
            _OfflineTrajectory,
        ):
            raise AssertionError(
                "Unexpected trajectory type captured by "
                "the offline test."
            )

        if len(
            saved_trajectory.entries
        ) != total_low_level_actions:
            raise AssertionError(
                "Expected one trajectory entry "
                "per SeeDo low-level action. "
                f"Expected {total_low_level_actions}, "
                f"found {len(saved_trajectory.entries)}."
            )

        final_entry = (
            saved_trajectory.entries[-1]
        )

        if not final_entry["done"]:
            raise AssertionError(
                "Final SeeDo trajectory entry was not "
                "marked as done."
            )

        if final_entry["reward"] != 1:
            raise AssertionError(
                "Final SeeDo trajectory entry does not "
                "contain reward=1."
            )

        print(
            "[PASS] trajectory reached save_rollout()"
        )

        #
        # The Motion Layer should also have produced its artifact
        # when persistent artifact storage is enabled.
        #
        if args.artifacts_dir is not None:
            motion_artifact_path = (
                node.controller.artifacts_dir
                / "motion_layer"
                / "motion_plan.json"
            )

            if not motion_artifact_path.is_file():
                raise AssertionError(
                    "Motion Layer artifact was not generated: "
                    f"{motion_artifact_path}"
                )

            print(
                "[PASS] Motion Layer artifact: "
                f"{motion_artifact_path}"
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
        default="1",
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