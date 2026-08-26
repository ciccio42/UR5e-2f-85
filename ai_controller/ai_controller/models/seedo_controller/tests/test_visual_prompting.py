from __future__ import annotations

import argparse

from ai_controller.models.seedo_controller.visual_prompter import VisualPrompter

def run_visual_prompting_test(
        args: argparse.Namespace,
    ) -> int:

        if args.video is None:
            raise ValueError(
                "--video is required for the visual prompting test."
            )

        if args.expected_keyframes is None:
            raise ValueError(
                "--expected-keyframes is required for the visual_prompting test."
            )

        prompter = VisualPrompter(
            grounding_config=args.grounding_config,
            grounding_checkpoint=args.grounding_checkpoint,
            bert_model=args.bert_model,
            sam_checkpoint=args.sam_checkpoint,
            sam2_checkpoint=args.sam2_checkpoint,
            objects=args.objects,
        )

        expected_keyframes = tuple(args.expected_keyframes)

        result = prompter.run(
            video_path=args.video,
            keyframes=expected_keyframes,
            artifacts_dir=args.artifacts_dir,
        )

        if not result.annotated_video_path.is_file():
            raise AssertionError(
                "Annotated video was not created: "
                f"{result.annotated_video_path}"
            )

        if result.annotated_video_path.stat().st_size == 0:
            raise AssertionError(
                "Annotated video is empty: "
                f"{result.annotated_video_path}"
            )

        if not result.track_id_map:
            raise AssertionError(
                "No tracked objects were returned."
            )

        if not result.key_frame_coordinates:
            raise AssertionError(
                "No key-frame coordinates were returned."
            )

        if not result.bounding_box_summary.strip():
            raise AssertionError(
                "Bounding box summary is empty."
            )

        if not result.count_diagnostics:
            raise AssertionError(
                "Count diagnostics are empty."
            )

        expected_frame_keys = {
            f"key_frame{frame_index}"
            for frame_index in expected_keyframes
        }

        returned_frame_keys = set(
            result.key_frame_coordinates.keys()
        )

        if returned_frame_keys != expected_frame_keys:
            raise AssertionError(
                "Unexpected key-frame coordinate keys. "
                f"Expected {sorted(expected_frame_keys)}, "
                f"received {sorted(returned_frame_keys)}."
            )

        for frame_key, coordinates in (
            result.key_frame_coordinates.items()
        ):
            if not coordinates:
                raise AssertionError(
                    f"No object coordinates were returned for {frame_key}."
                )

        tracked_object_ids = set(result.track_id_map.keys())

        if any(
            not isinstance(track_id, int)
            for track_id in tracked_object_ids
        ):
            raise AssertionError(
                "All track IDs must be integers."
            )

        for track_id, info in result.track_id_map.items():
            if "detector_label" not in info:
                raise AssertionError(
                    f"Track {track_id} has no detector_label."
                )

            if "initial_center" not in info:
                raise AssertionError(
                    f"Track {track_id} has no initial_center."
                )

        print("Visual prompting completed")
        print(f"Annotated video: {result.annotated_video_path}")

        print("\nTracked objects:")
        for track_id, info in result.track_id_map.items():
            print(f"  Track {track_id}: {info}")

        print("\nKey-frame coordinates:")
        for frame_name, coordinates in (
            result.key_frame_coordinates.items()
        ):
            print(f"  {frame_name}: {coordinates}")

        print("\nBounding box summary:")
        print(result.bounding_box_summary)

        print("\nCount diagnostics:")
        print(result.count_diagnostics)

        print("\nTEST PASSED")
        return 0
