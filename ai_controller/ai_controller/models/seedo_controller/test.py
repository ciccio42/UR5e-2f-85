from __future__ import annotations

import argparse
from pathlib import Path

from ai_controller.models.seedo_controller.keyframe_selector import (
    KeyframeSelector,
)

from ai_controller.models.seedo_controller.visual_prompter import (
    VisualPrompter,
)

from ai_controller.models.seedo_controller.action_planner import (
    ActionPlanner,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Standalone test for the SeeDo keyframe selector."
    )
    parser.add_argument(
        "--video",
        type=Path,
        required=True,
        help="Path to the input demonstration video.",
    )
    parser.add_argument(
        "--artifacts-dir",
        type=Path,
        default=None,
        help="Optional directory for debug artifacts and previews.",
    )
    parser.add_argument(
        "--expected-keyframes",
        type=int,
        nargs="*",
        default=None,
        help="Optional expected keyframe indexes.",
    )

    parser.add_argument(
        "--stage",
        choices=["keyframe", "visual_prompting", "action_planning"],
        default="keyframe",
        help="Pipeline stage to test.",
    )

    parser.add_argument(
        "--grounding-config",
        type=Path,
        default=Path(
            "/opt/checkpoints/seedo/groundingdino/"
            "GroundingDINO_SwinB.cfg.py"
        ),
    )

    parser.add_argument(
        "--grounding-checkpoint",
        type=Path,
        default=Path(
            "/opt/checkpoints/seedo/groundingdino/"
            "groundingdino_swinb_cogcoor.pth"
        ),
    )

    parser.add_argument(
        "--bert-model",
        type=Path,
        default=Path(
            "/opt/checkpoints/seedo/bert-base-uncased"
        ),
    )

    parser.add_argument(
        "--sam-checkpoint",
        type=Path,
        default=Path(
            "/opt/checkpoints/seedo/sam/sam_vit_h_4b8939.pth"
        ),
    )

    parser.add_argument(
        "--sam2-checkpoint",
        type=Path,
        default=Path(
            "/opt/checkpoints/seedo/sam2/sam2_hiera_large.pt"
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.stage == "keyframe":
        return run_keyframe_test(args)

    if args.stage == "visual_prompting":
        return run_visual_prompting_test(args)

    if args.stage == "action_planning":
        return run_action_planning_test(args)

    raise ValueError(f"Unsupported stage: {args.stage}")
    

def run_keyframe_test(args: argparse.Namespace) -> int:
    selector = KeyframeSelector(
        gaussian_sigma=5.0,
        prominence=0.8,
        expected_keyframes=2,
        save_preview=True,
    )

    result = selector.run(
        video_path=args.video,
        artifacts_dir=args.artifacts_dir,
    )

    print("Keyframe selection completed")
    print(f"Video: {result.video_path}")
    print(f"Keyframes: {list(result.keyframes)}")
    print(f"Images returned: {len(result.keyframe_images)}")
    print(f"Artifacts directory: {result.artifacts_dir}")

    for frame_index, image in zip(
        result.keyframes,
        result.keyframe_images,
        strict=True,
    ):
        print(
            f"Frame {frame_index}: "
            f"shape={image.shape}, dtype={image.dtype}"
        )

    if args.expected_keyframes is not None:
        expected = tuple(args.expected_keyframes)

        if result.keyframes != expected:
            raise AssertionError(
                f"Expected keyframes {list(expected)}, "
                f"but received {list(result.keyframes)}"
            )

    preview_dir = result.artifacts_dir / "returned_keyframes"

    if not preview_dir.is_dir():
        raise AssertionError(
            f"Preview directory was not created: {preview_dir}"
        )

    for frame_index in result.keyframes:
        preview_path = (
            preview_dir
            / f"keyframe_{frame_index:06d}.png"
        )

        if not preview_path.is_file():
            raise AssertionError(
                f"Missing keyframe preview: {preview_path}"
            )

    print("TEST PASSED")
    return 0

def run_visual_prompting_test(
        args: argparse.Namespace,
    ) -> int:
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

def run_action_planning_test(
    args: argparse.Namespace,
) -> int:
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

if __name__ == "__main__":
    raise SystemExit(main())