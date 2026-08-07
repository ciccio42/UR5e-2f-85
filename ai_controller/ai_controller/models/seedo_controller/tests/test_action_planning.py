from __future__ import annotations

import argparse
from pathlib import Path

from ai_controller.models.seedo_controller.action_planner import ActionPlanner
from ai_controller.models.seedo_controller.visual_prompter import VisualPrompter

def run_action_planning_test(
        args: argparse.Namespace,
    ) -> int:

        if args.video is None:
            raise ValueError(
                "--video is required for the action_planning test."
            )

        if args.expected_keyframes is None:
            raise ValueError(
                "--expected-keyframes is required for the action_planning test."
            )

        expected_keyframes = tuple(args.expected_keyframes)

        visual_prompter = VisualPrompter(
            grounding_config=args.grounding_config,
            grounding_checkpoint=args.grounding_checkpoint,
            bert_model=args.bert_model,
            sam_checkpoint=args.sam_checkpoint,
            sam2_checkpoint=args.sam2_checkpoint,
        )

        visual_result = visual_prompter.run(
            video_path=args.video,
            keyframes=expected_keyframes,
            artifacts_dir=(
                Path(args.artifacts_dir)
                / "visual_prompting"
            ),
        )

        planner = ActionPlanner()

        action_result = planner.run(
            annotated_video_path=visual_result.annotated_video_path,
            keyframes=expected_keyframes,
            track_id_map=visual_result.track_id_map,
            key_frame_coordinates=visual_result.key_frame_coordinates,
            artifacts_dir=(
                Path(args.artifacts_dir)
                / "action_planning"
            ),
        )

        print("Action planning completed")
        print(f"Status: {action_result.status}")
        print(
            "Natural-language plan: "
            f"{action_result.natural_language_plan}"
        )

        print("\nSteps:")
        for index, step in enumerate(
            action_result.steps,
            start=1,
        ):
            print(f"  Step {index}: {step}")

        print("\nAmbiguities:")
        for ambiguity in action_result.ambiguities:
            print(f"  - {ambiguity}")

        if action_result.status not in (
            "completed",
            "ambiguous",
        ):
            raise AssertionError(
                f"Unexpected status: {action_result.status}"
            )

        if (
            action_result.status == "completed"
            and len(action_result.steps) != 1
        ):
            raise AssertionError(
                "A completed action plan must contain exactly one step."
            )

        if (
            action_result.status == "ambiguous"
            and not action_result.ambiguities
        ):
            raise AssertionError(
                "An ambiguous action plan must explain the ambiguity."
            )

        if not action_result.natural_language_plan.strip():
            raise AssertionError(
                "Natural-language plan is empty."
            )

        print("\nTEST PASSED")

        return 0
