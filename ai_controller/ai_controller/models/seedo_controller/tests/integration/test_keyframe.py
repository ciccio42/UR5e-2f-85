from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from ai_controller.models.seedo_controller.keyframe_selector import (
    KeyframeSelector,
)


def _load_rgb_frame(
    video_path: Path,
    frame_index: int,
) -> np.ndarray:
    """Read one exact frame from the source video and return it in RGB."""

    capture = cv2.VideoCapture(str(video_path))

    try:
        if not capture.isOpened():
            raise AssertionError(
                "Could not reopen the input video for validation: "
                f"{video_path}"
            )

        capture.set(
            cv2.CAP_PROP_POS_FRAMES,
            frame_index,
        )

        ok, frame_bgr = capture.read()

        if not ok or frame_bgr is None:
            raise AssertionError(
                "Could not read selected frame "
                f"{frame_index} from {video_path}."
            )

        return cv2.cvtColor(
            frame_bgr,
            cv2.COLOR_BGR2RGB,
        )

    finally:
        capture.release()


def _parse_candidate_frame_index(
    path: Path,
) -> int:
    """Extract the integer index from frame_XXXXXX.jpg."""

    prefix = "frame_"

    if (
        not path.stem.startswith(prefix)
        or not path.stem[len(prefix):].isdigit()
    ):
        raise AssertionError(
            "Unexpected candidate-frame filename: "
            f"{path.name}"
        )

    return int(path.stem[len(prefix):])


def run_keyframe_test(
    args: argparse.Namespace,
) -> int:
    """Run the real KeyframeSelector VLM integration test."""

    if args.video is None:
        raise ValueError(
            "--video is required for the keyframe test."
        )

    video_path = Path(args.video).expanduser().resolve()

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
        video_path=video_path,
        artifacts_dir=args.artifacts_dir,
    )

    print("VLM keyframe selection completed")
    print(f"Video: {result.video_path}")
    print(f"Keyframes: {list(result.keyframes)}")
    print(f"Images returned: {len(result.keyframe_images)}")
    print(f"Artifacts directory: {result.artifacts_dir}")

    if result.video_path != video_path:
        raise AssertionError(
            "Returned video_path does not match the input video: "
            f"expected={video_path}, received={result.video_path}"
        )

    if not result.artifacts_dir.is_dir():
        raise AssertionError(
            "Artifacts directory does not exist: "
            f"{result.artifacts_dir}"
        )

    if len(result.keyframes) != 2:
        raise AssertionError(
            "Expected exactly two keyframes, "
            f"received {list(result.keyframes)}"
        )

    if len(set(result.keyframes)) != 2:
        raise AssertionError(
            "Duplicate keyframes were returned: "
            f"{list(result.keyframes)}"
        )

    pick_frame, place_frame = result.keyframes

    if pick_frame < 0 or place_frame < 0:
        raise AssertionError(
            "Keyframe indexes cannot be negative: "
            f"{list(result.keyframes)}"
        )

    if pick_frame >= place_frame:
        raise AssertionError(
            "Invalid temporal ordering: "
            f"pick={pick_frame}, place={place_frame}"
        )

    if len(result.keyframe_images) != len(result.keyframes):
        raise AssertionError(
            "The number of returned images does not match "
            "the number of selected keyframes."
        )

    for frame_index, image in zip(
        result.keyframes,
        result.keyframe_images,
        strict=True,
    ):
        if not isinstance(image, np.ndarray):
            raise AssertionError(
                f"Frame {frame_index} is not a NumPy array."
            )

        if image.ndim != 3 or image.shape[2] != 3:
            raise AssertionError(
                f"Frame {frame_index} has invalid shape "
                f"{image.shape}; expected HxWx3."
            )

        if image.dtype != np.uint8:
            raise AssertionError(
                f"Frame {frame_index} has invalid dtype "
                f"{image.dtype}; expected uint8."
            )

        if image.size == 0:
            raise AssertionError(
                f"Frame {frame_index} is empty."
            )

        expected_rgb = _load_rgb_frame(
            video_path,
            frame_index,
        )

        if not np.array_equal(image, expected_rgb):
            raise AssertionError(
                "Returned keyframe image does not match the "
                f"source video frame {frame_index}."
            )

        print(
            f"Frame {frame_index}: "
            f"shape={image.shape}, dtype={image.dtype}"
        )

    if args.expected_keyframes is not None:
        expected = tuple(
            int(frame)
            for frame in args.expected_keyframes
        )

        if len(expected) != 2:
            raise AssertionError(
                "--expected-keyframes must contain exactly "
                f"two indexes, received {list(expected)}."
            )

        if result.keyframes != expected:
            raise AssertionError(
                f"Expected keyframes {list(expected)}, "
                f"but received {list(result.keyframes)}"
            )

    preview_dir = result.artifacts_dir / "returned_keyframes"

    if not preview_dir.is_dir():
        raise AssertionError(
            "Preview directory was not created: "
            f"{preview_dir}"
        )

    for frame_index, image in zip(
        result.keyframes,
        result.keyframe_images,
        strict=True,
    ):
        preview_path = (
            preview_dir
            / f"keyframe_{frame_index:06d}.png"
        )

        if not preview_path.is_file():
            raise AssertionError(
                "Missing keyframe preview: "
                f"{preview_path}"
            )

        preview_bgr = cv2.imread(
            str(preview_path),
            cv2.IMREAD_COLOR,
        )

        if preview_bgr is None:
            raise AssertionError(
                "Could not read keyframe preview: "
                f"{preview_path}"
            )

        preview_rgb = cv2.cvtColor(
            preview_bgr,
            cv2.COLOR_BGR2RGB,
        )

        if not np.array_equal(preview_rgb, image):
            raise AssertionError(
                "Saved keyframe preview does not match "
                f"returned frame {frame_index}."
            )

    candidate_frames_dir = (
        result.artifacts_dir
        / "candidate_frames"
    )

    if not candidate_frames_dir.is_dir():
        raise AssertionError(
            "Candidate-frame directory was not created: "
            f"{candidate_frames_dir}"
        )

    candidate_paths = sorted(
        candidate_frames_dir.glob("frame_*.jpg")
    )

    if len(candidate_paths) < 2:
        raise AssertionError(
            "Expected at least two saved candidate frames, "
            f"received {len(candidate_paths)}."
        )

    saved_candidate_indices = tuple(
        _parse_candidate_frame_index(path)
        for path in candidate_paths
    )

    if tuple(sorted(saved_candidate_indices)) != saved_candidate_indices:
        raise AssertionError(
            "Saved candidate frames are not chronologically ordered."
        )

    if len(set(saved_candidate_indices)) != len(saved_candidate_indices):
        raise AssertionError(
            "Duplicate candidate-frame indexes were saved."
        )

    for frame_index, path in zip(
        saved_candidate_indices,
        candidate_paths,
        strict=True,
    ):
        if frame_index % selector.sample_stride != 0:
            raise AssertionError(
                "Candidate frame does not respect sample_stride: "
                f"frame={frame_index}, "
                f"sample_stride={selector.sample_stride}"
            )

        candidate_image = cv2.imread(
            str(path),
            cv2.IMREAD_COLOR,
        )

        if candidate_image is None:
            raise AssertionError(
                "Could not read saved candidate frame: "
                f"{path}"
            )

        if (
            candidate_image.ndim != 3
            or candidate_image.shape[2] != 3
        ):
            raise AssertionError(
                "Saved candidate frame has invalid shape: "
                f"{path} -> {candidate_image.shape}"
            )

    print(
        "Candidate frames saved: "
        f"{len(candidate_paths)}"
    )

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
        vlm_result = json.load(stream)

    if Path(vlm_result["video_path"]).resolve() != video_path:
        raise AssertionError(
            "VLM artifact contains an unexpected video_path."
        )

    if vlm_result["model"] != selector.model:
        raise AssertionError(
            "VLM artifact contains an unexpected model: "
            f"{vlm_result['model']!r}"
        )

    if int(vlm_result["sample_stride"]) != selector.sample_stride:
        raise AssertionError(
            "VLM artifact contains an unexpected sample_stride."
        )

    artifact_candidate_indices = tuple(
        int(frame)
        for frame in vlm_result["candidate_frames"]
    )

    if artifact_candidate_indices != saved_candidate_indices:
        raise AssertionError(
            "Candidate-frame indexes in the VLM artifact do not "
            "match the candidate-frame files on disk."
        )

    if pick_frame not in artifact_candidate_indices:
        raise AssertionError(
            "Selected pick frame is not present in the "
            "candidate-frame set."
        )

    if place_frame not in artifact_candidate_indices:
        raise AssertionError(
            "Selected place frame is not present in the "
            "candidate-frame set."
        )

    selection = vlm_result["selection"]

    if selection["status"] != "completed":
        raise AssertionError(
            "VLM selection did not complete: "
            f"{selection}"
        )

    if int(selection["pick_frame"]) != pick_frame:
        raise AssertionError(
            "Returned pick frame does not match "
            "the VLM artifact."
        )

    if int(selection["place_frame"]) != place_frame:
        raise AssertionError(
            "Returned place frame does not match "
            "the VLM artifact."
        )

    pick_evidence = str(selection["pick_evidence"]).strip()
    place_evidence = str(selection["place_evidence"]).strip()

    if not pick_evidence:
        raise AssertionError(
            "The VLM artifact contains empty pick evidence."
        )

    if not place_evidence:
        raise AssertionError(
            "The VLM artifact contains empty place evidence."
        )

    if not isinstance(selection["ambiguity"], str):
        raise AssertionError(
            "The VLM artifact ambiguity field is not a string."
        )

    print(f"Pick evidence: {pick_evidence}")
    print(f"Place evidence: {place_evidence}")
    print("TEST PASSED")

    return 0
