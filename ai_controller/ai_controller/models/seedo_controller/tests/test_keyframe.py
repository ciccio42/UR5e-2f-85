from __future__ import annotations

import argparse
import json

from ai_controller.models.seedo_controller.keyframe_selector import (
    KeyframeSelector,
)


def run_keyframe_test(
    args: argparse.Namespace,
) -> int:

    if args.video is None:
        raise ValueError(
            "--video is required for the keyframe test."
        )

    selector = KeyframeSelector(
        mode="vlm",
        model="gpt-4o-2024-08-06",
        sample_stride=5,
        gaussian_sigma=5.0,
        prominence=0.8,
        expected_keyframes=2,
        save_preview=True,
    )

    result = selector.run(
        video_path=args.video,
        artifacts_dir=args.artifacts_dir,
    )

    print("VLM keyframe selection completed")
    print(f"Video: {result.video_path}")
    print(
        f"Keyframes: "
        f"{list(result.keyframes)}"
    )
    print(
        f"Images returned: "
        f"{len(result.keyframe_images)}"
    )
    print(
        f"Artifacts directory: "
        f"{result.artifacts_dir}"
    )

    # ---------------------------------------------------------
    # Validate returned keyframes.
    # ---------------------------------------------------------

    if len(result.keyframes) != 2:
        raise AssertionError(
            "Expected exactly two keyframes, "
            f"received {list(result.keyframes)}"
        )

    pick_frame, place_frame = (
        result.keyframes
    )

    if pick_frame >= place_frame:
        raise AssertionError(
            "Invalid temporal ordering: "
            f"pick={pick_frame}, "
            f"place={place_frame}"
        )

    # ---------------------------------------------------------
    # Validate returned RGB images.
    # ---------------------------------------------------------

    for (
        frame_index,
        image,
    ) in zip(
        result.keyframes,
        result.keyframe_images,
        strict=True,
    ):
        print(
            f"Frame {frame_index}: "
            f"shape={image.shape}, "
            f"dtype={image.dtype}"
        )

    # ---------------------------------------------------------
    # Optional exact expected keyframes.
    # ---------------------------------------------------------

    if args.expected_keyframes is not None:
        expected = tuple(
            args.expected_keyframes
        )

        if result.keyframes != expected:
            raise AssertionError(
                f"Expected keyframes {list(expected)}, "
                f"but received "
                f"{list(result.keyframes)}"
            )

    # ---------------------------------------------------------
    # Validate selected-keyframe previews.
    # ---------------------------------------------------------

    preview_dir = (
        result.artifacts_dir
        / "returned_keyframes"
    )

    if not preview_dir.is_dir():
        raise AssertionError(
            "Preview directory was not created: "
            f"{preview_dir}"
        )

    for frame_index in result.keyframes:
        preview_path = (
            preview_dir
            / f"keyframe_{frame_index:06d}.png"
        )

        if not preview_path.is_file():
            raise AssertionError(
                "Missing keyframe preview: "
                f"{preview_path}"
            )

    # ---------------------------------------------------------
    # Validate VLM candidate-frame artifacts.
    # ---------------------------------------------------------

    candidate_frames_dir = (
        result.artifacts_dir
        / "candidate_frames"
    )

    if not candidate_frames_dir.is_dir():
        raise AssertionError(
            "Candidate-frame directory was not created: "
            f"{candidate_frames_dir}"
        )

    candidate_frames = sorted(
        candidate_frames_dir.glob(
            "frame_*.jpg"
        )
    )

    if not candidate_frames:
        raise AssertionError(
            "No candidate frames were saved."
        )

    print(
        "Candidate frames saved: "
        f"{len(candidate_frames)}"
    )

    # ---------------------------------------------------------
    # Validate VLM result artifact.
    # ---------------------------------------------------------

    vlm_result_path = (
        result.artifacts_dir
        / "vlm_keyframe_selection.json"
    )

    if not vlm_result_path.is_file():
        raise AssertionError(
            "Missing VLM selection artifact: "
            f"{vlm_result_path}"
        )

    with vlm_result_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        vlm_result = json.load(
            stream
        )

    selection = vlm_result[
        "selection"
    ]

    if selection["status"] != "completed":
        raise AssertionError(
            "VLM selection did not complete: "
            f"{selection}"
        )

    if (
        int(selection["pick_frame"])
        != pick_frame
    ):
        raise AssertionError(
            "Returned pick frame does not match "
            "the VLM artifact."
        )

    if (
        int(selection["place_frame"])
        != place_frame
    ):
        raise AssertionError(
            "Returned place frame does not match "
            "the VLM artifact."
        )

    print(
        "Pick evidence: "
        f"{selection['pick_evidence']}"
    )

    print(
        "Place evidence: "
        f"{selection['place_evidence']}"
    )

    print("TEST PASSED")

    return 0