from __future__ import annotations

import argparse

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

        controller = SeeDoController(
            model_config=args.model_config,
        )

        controller.load_command(
            demo_path=args.video,
            task_id="test",
            artifacts_dir=args.artifacts_dir,
        )

        result = controller.inference(
            input_data={
                "video_path": args.video,
                "artifacts_dir": args.artifacts_dir,
            },
            t=0,
        )

        if result is not controller.last_result:
            raise AssertionError(
                "SeeDoController.last_result was not updated correctly."
            )

        if result.status not in (
            "completed",
            "ambiguous",
        ):
            raise AssertionError(
                f"Unexpected status: {result.status}"
            )

        if (
            result.status == "completed"
            and len(result.steps) != 1
        ):
            raise AssertionError(
                "A completed plan must contain exactly one action step."
            )

        if (
            result.status == "ambiguous"
            and not result.ambiguities
        ):
            raise AssertionError(
                "An ambiguous plan must explain its ambiguities."
            )

        if not result.natural_language_plan.strip():
            raise AssertionError(
                "Natural-language plan is empty."
            )

        print("SeeDo controller pipeline completed")
        print(f"Status: {result.status}")
        print(
            "Natural-language plan: "
            f"{result.natural_language_plan}"
        )

        print("\nSteps:")
        for index, step in enumerate(
            result.steps,
            start=1,
        ):
            print(f"  Step {index}: {step}")

        print("\nAmbiguities:")
        for ambiguity in result.ambiguities:
            print(f"  - {ambiguity}")

        controller.reset()

        if controller.last_result is not None:
            raise AssertionError(
                "SeeDoController.reset() did not clear last_result."
            )

        print("\nTEST PASSED")
        return 0
