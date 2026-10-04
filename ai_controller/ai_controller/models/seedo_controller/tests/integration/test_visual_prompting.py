from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import cv2

from ai_controller.models.seedo_controller.visual_prompter import (
    VisualPrompter,
)


# ---------------------------------------------------------------------
# Standard integration-test ground truth.
#
# The selected pick-and-place demonstration contains:
#   - 4 manipulable colored objects
#   - 4 storage bins
# ---------------------------------------------------------------------

EXPECTED_TOTAL_OBJECTS = 8
EXPECTED_STORAGE_BINS = 4

_COORDINATE_PATTERN = re.compile(
    r"^Object\s+(\d+):\s+\((-?\d+),\s*(-?\d+)\)$"
)


def _video_metadata(
    video_path: Path,
) -> tuple[int, int, int]:
    """Return (frame_count, width, height) for a readable video."""

    capture = cv2.VideoCapture(
        str(video_path)
    )

    try:
        if not capture.isOpened():
            raise AssertionError(
                "Could not open video: "
                f"{video_path}"
            )

        frame_count = int(
            capture.get(
                cv2.CAP_PROP_FRAME_COUNT
            )
        )

        width = int(
            capture.get(
                cv2.CAP_PROP_FRAME_WIDTH
            )
        )

        height = int(
            capture.get(
                cv2.CAP_PROP_FRAME_HEIGHT
            )
        )

        if frame_count <= 0:
            raise AssertionError(
                "Video contains no readable frames: "
                f"{video_path}"
            )

        if width <= 0 or height <= 0:
            raise AssertionError(
                "Video has invalid dimensions: "
                f"{width}x{height}"
            )

        return (
            frame_count,
            width,
            height,
        )

    finally:
        capture.release()


def _parse_keyframe_coordinates(
    entries: list[str],
    *,
    frame_key: str,
    width: int,
    height: int,
) -> dict[int, tuple[int, int]]:
    """Parse VisualPrompter coordinate strings and validate image bounds."""

    parsed: dict[
        int,
        tuple[int, int],
    ] = {}

    for entry in entries:
        match = _COORDINATE_PATTERN.fullmatch(
            str(entry).strip()
        )

        if match is None:
            raise AssertionError(
                "Unexpected key-frame coordinate format in "
                f"{frame_key}: {entry!r}"
            )

        object_id = int(
            match.group(1)
        )

        x = int(
            match.group(2)
        )

        y = int(
            match.group(3)
        )

        if object_id in parsed:
            raise AssertionError(
                "Duplicate object ID in "
                f"{frame_key}: {object_id}"
            )

        if not (
            0 <= x < width
            and 0 <= y < height
        ):
            raise AssertionError(
                "Object center is outside the image in "
                f"{frame_key}: "
                f"object={object_id}, "
                f"center=({x}, {y}), "
                f"image={width}x{height}"
            )

        parsed[object_id] = (
            x,
            y,
        )

    return parsed


def _validate_object_discovery_artifact(
    path: Path,
) -> None:
    """Validate the generalized VLM discovery artifact."""

    if not path.is_file():
        raise AssertionError(
            "Missing generalized object-discovery artifact: "
            f"{path}"
        )

    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        data = json.load(
            stream
        )

    records = data.get(
        "objects"
    )

    if not isinstance(
        records,
        list,
    ) or not records:
        raise AssertionError(
            "object_discovery.json does not contain "
            "a non-empty 'objects' list."
        )

    total_reported_instances = 0

    for index, record in enumerate(
        records
    ):
        if not isinstance(
            record,
            dict,
        ):
            raise AssertionError(
                "Invalid object-discovery record at "
                f"index {index}: {record!r}"
            )

        category = str(
            record.get(
                "category",
                "",
            )
        ).strip()

        detector_label = str(
            record.get(
                "detector_label",
                "",
            )
        ).strip()

        attributes = record.get(
            "attributes"
        )

        count = record.get(
            "count"
        )

        if not category:
            raise AssertionError(
                "Object-discovery record has an empty category: "
                f"{record}"
            )

        if not detector_label:
            raise AssertionError(
                "Object-discovery record has an empty detector_label: "
                f"{record}"
            )

        if not isinstance(
            attributes,
            dict,
        ):
            raise AssertionError(
                "Object-discovery attributes must be a dictionary: "
                f"{record}"
            )

        if (
            type(count) is not int
            or count <= 0
        ):
            raise AssertionError(
                "Object-discovery count must be a positive integer: "
                f"{record}"
            )

        total_reported_instances += count

    if (
        total_reported_instances
        != EXPECTED_TOTAL_OBJECTS
    ):
        raise AssertionError(
            "Unexpected number of instances in object discovery: "
            f"expected={EXPECTED_TOTAL_OBJECTS}, "
            f"received={total_reported_instances}"
        )


def run_visual_prompting_test(
    args: argparse.Namespace,
) -> int:
    """Run the real generalized VisualPrompter integration test."""

    if args.video is None:
        raise ValueError(
            "--video is required for the visual_prompting test."
        )

    if args.expected_keyframes is None:
        raise ValueError(
            "--expected-keyframes is required for "
            "the visual_prompting test."
        )

    if args.objects is not None:
        raise ValueError(
            "The VisualPrompter integration test exercises "
            "real generalized VLM object discovery. "
            "Do not provide --objects."
        )

    video_path = (
        Path(args.video)
        .expanduser()
        .resolve()
    )

    expected_keyframes = tuple(
        int(frame)
        for frame
        in args.expected_keyframes
    )

    if len(expected_keyframes) != 2:
        raise ValueError(
            "--expected-keyframes must contain exactly "
            "two frame indexes."
        )

    if (
        tuple(
            sorted(
                expected_keyframes
            )
        )
        != expected_keyframes
    ):
        raise ValueError(
            "--expected-keyframes must be in chronological order."
        )

    (
        source_frame_count,
        source_width,
        source_height,
    ) = _video_metadata(
        video_path
    )

    for frame_index in expected_keyframes:
        if (
            frame_index < 0
            or frame_index >= source_frame_count
        ):
            raise ValueError(
                "Expected keyframe is outside the source video: "
                f"frame={frame_index}, "
                f"frame_count={source_frame_count}"
            )

    prompter = VisualPrompter(
        grounding_config=args.grounding_config,
        grounding_checkpoint=args.grounding_checkpoint,
        bert_model=args.bert_model,
        sam_checkpoint=args.sam_checkpoint,
        sam2_checkpoint=args.sam2_checkpoint,
        objects=None,
        perception_mode="generalized",
    )

    result = prompter.run(
        video_path=video_path,
        keyframes=expected_keyframes,
        artifacts_dir=args.artifacts_dir,
    )

    # ---------------------------------------------------------
    # Annotated video.
    # ---------------------------------------------------------

    annotated_video_path = (
        Path(
            result.annotated_video_path
        )
        .expanduser()
        .resolve()
    )

    if not annotated_video_path.is_file():
        raise AssertionError(
            "Annotated video was not created: "
            f"{annotated_video_path}"
        )

    if (
        annotated_video_path.stat().st_size
        == 0
    ):
        raise AssertionError(
            "Annotated video is empty: "
            f"{annotated_video_path}"
        )

    (
        annotated_frame_count,
        annotated_width,
        annotated_height,
    ) = _video_metadata(
        annotated_video_path
    )

    if (
        annotated_frame_count
        != source_frame_count
    ):
        raise AssertionError(
            "Annotated video frame count differs from "
            "the source video: "
            f"source={source_frame_count}, "
            f"annotated={annotated_frame_count}"
        )

    if (
        annotated_width
        != source_width
        or annotated_height
        != source_height
    ):
        raise AssertionError(
            "Annotated video dimensions differ from "
            "the source video: "
            f"source={source_width}x{source_height}, "
            f"annotated={annotated_width}x{annotated_height}"
        )

    # ---------------------------------------------------------
    # Track metadata.
    # ---------------------------------------------------------

    if not isinstance(
        result.track_id_map,
        dict,
    ) or not result.track_id_map:
        raise AssertionError(
            "No tracked objects were returned."
        )

    track_ids = tuple(
        sorted(
            result.track_id_map.keys()
        )
    )

    if any(
        not isinstance(
            track_id,
            int,
        )
        for track_id in track_ids
    ):
        raise AssertionError(
            "All track IDs must be integers."
        )

    expected_track_ids = tuple(
        range(
            len(track_ids)
        )
    )

    if track_ids != expected_track_ids:
        raise AssertionError(
            "Track IDs are expected to be contiguous "
            "and zero-based: "
            f"expected={expected_track_ids}, "
            f"received={track_ids}"
        )

    if (
        len(track_ids)
        != EXPECTED_TOTAL_OBJECTS
    ):
        raise AssertionError(
            "Unexpected number of tracked objects: "
            f"expected={EXPECTED_TOTAL_OBJECTS}, "
            f"received={len(track_ids)}"
        )

    bin_track_ids: list[int] = []

    for (
        track_id,
        info,
    ) in result.track_id_map.items():
        if not isinstance(
            info,
            dict,
        ):
            raise AssertionError(
                f"Track {track_id} metadata are not a dictionary."
            )

        detector_label = str(
            info.get(
                "detector_label",
                "",
            )
        ).strip()

        if not detector_label:
            raise AssertionError(
                f"Track {track_id} has an empty detector_label."
            )

        center = info.get(
            "initial_center"
        )

        if (
            not isinstance(
                center,
                (list, tuple),
            )
            or len(center) != 2
        ):
            raise AssertionError(
                f"Track {track_id} has an invalid initial_center: "
                f"{center!r}"
            )

        center_x = int(
            center[0]
        )

        center_y = int(
            center[1]
        )

        if not (
            0 <= center_x < source_width
            and 0 <= center_y < source_height
        ):
            raise AssertionError(
                f"Track {track_id} initial_center is outside "
                "the source image: "
                f"{center}"
            )

        category = str(
            info.get(
                "category",
                "",
            )
        ).strip().lower()

        if not category:
            raise AssertionError(
                f"Track {track_id} has no generalized category."
            )

        attributes = info.get(
            "attributes"
        )

        if not isinstance(
            attributes,
            dict,
        ):
            raise AssertionError(
                f"Track {track_id} attributes are not a dictionary."
            )

        if category == "bin":
            bin_track_ids.append(
                track_id
            )

            if detector_label != "storage bin":
                raise AssertionError(
                    "Generalized storage-bin detector label "
                    "was not normalized: "
                    f"track={track_id}, "
                    f"label={detector_label!r}"
                )

    if (
        len(bin_track_ids)
        != EXPECTED_STORAGE_BINS
    ):
        raise AssertionError(
            "Unexpected number of storage bins: "
            f"expected={EXPECTED_STORAGE_BINS}, "
            f"received={len(bin_track_ids)}"
        )

    # ---------------------------------------------------------
    # Key-frame SAM centers.
    # ---------------------------------------------------------

    if not isinstance(
        result.key_frame_coordinates,
        dict,
    ) or not result.key_frame_coordinates:
        raise AssertionError(
            "No key-frame coordinates were returned."
        )

    expected_frame_keys = {
        f"key_frame{frame_index}"
        for frame_index
        in expected_keyframes
    }

    returned_frame_keys = set(
        result.key_frame_coordinates.keys()
    )

    if (
        returned_frame_keys
        != expected_frame_keys
    ):
        raise AssertionError(
            "Unexpected key-frame coordinate keys. "
            f"Expected {sorted(expected_frame_keys)}, "
            f"received {sorted(returned_frame_keys)}."
        )

    for frame_key in sorted(
        expected_frame_keys
    ):
        coordinates = (
            result
            .key_frame_coordinates[
                frame_key
            ]
        )

        if not isinstance(
            coordinates,
            list,
        ) or not coordinates:
            raise AssertionError(
                "No object coordinates were returned for "
                f"{frame_key}."
            )

        parsed_coordinates = (
            _parse_keyframe_coordinates(
                coordinates,
                frame_key=frame_key,
                width=source_width,
                height=source_height,
            )
        )

        returned_ids = set(
            parsed_coordinates.keys()
        )

        if returned_ids != set(
            track_ids
        ):
            raise AssertionError(
                "Tracked object IDs in the keyframe do not "
                "match track_id_map: "
                f"frame={frame_key}, "
                f"expected={sorted(track_ids)}, "
                f"received={sorted(returned_ids)}"
            )

    # ---------------------------------------------------------
    # Bounding-box / coordinate summary.
    # ---------------------------------------------------------

    if not isinstance(
        result.bounding_box_summary,
        str,
    ) or not result.bounding_box_summary.strip():
        raise AssertionError(
            "Bounding box summary is empty."
        )

    for frame_key in expected_frame_keys:
        if (
            frame_key
            not in result.bounding_box_summary
        ):
            raise AssertionError(
                "Bounding box summary is missing "
                f"{frame_key}."
            )

    # ---------------------------------------------------------
    # Count diagnostics.
    # ---------------------------------------------------------

    diagnostics = (
        result.count_diagnostics
    )

    if not isinstance(
        diagnostics,
        dict,
    ) or not diagnostics:
        raise AssertionError(
            "Count diagnostics are empty."
        )

    if (
        diagnostics.get(
            "source"
        )
        != "openai"
    ):
        raise AssertionError(
            "Expected real OpenAI object discovery, "
            f"received source={diagnostics.get('source')!r}."
        )

    count_fields = (
        "reported_objects",
        "parsed_objects",
        "selected_boxes",
        "masks_after_filter",
        "tracked_objects_min",
        "tracked_objects_max",
    )

    for field in count_fields:
        value = diagnostics.get(
            field
        )

        if value != EXPECTED_TOTAL_OBJECTS:
            raise AssertionError(
                "Unexpected object count in diagnostics: "
                f"{field}={value}, "
                f"expected={EXPECTED_TOTAL_OBJECTS}"
            )

    if (
        diagnostics.get(
            "count_consistent"
        )
        is not True
    ):
        raise AssertionError(
            "Visual prompting object counts are inconsistent: "
            f"{json.dumps(diagnostics, sort_keys=True)}"
        )

    requested_by_label = (
        diagnostics.get(
            "requested_by_label"
        )
    )

    if not isinstance(
        requested_by_label,
        dict,
    ) or not requested_by_label:
        raise AssertionError(
            "requested_by_label diagnostics are missing."
        )

    if (
        sum(
            int(value)
            for value
            in requested_by_label.values()
        )
        != EXPECTED_TOTAL_OBJECTS
    ):
        raise AssertionError(
            "requested_by_label does not sum to "
            f"{EXPECTED_TOTAL_OBJECTS}."
        )

    grounding_by_label = (
        diagnostics.get(
            "grounding_by_label"
        )
    )

    if not isinstance(
        grounding_by_label,
        dict,
    ):
        raise AssertionError(
            "grounding_by_label diagnostics are missing."
        )

    for (
        label,
        requested_count,
    ) in requested_by_label.items():
        label_diagnostics = (
            grounding_by_label.get(
                label
            )
        )

        if not isinstance(
            label_diagnostics,
            dict,
        ):
            raise AssertionError(
                "Missing GroundingDINO diagnostics for "
                f"{label!r}."
            )

        if (
            int(
                label_diagnostics.get(
                    "requested",
                    -1,
                )
            )
            != int(
                requested_count
            )
        ):
            raise AssertionError(
                "GroundingDINO requested count mismatch "
                f"for {label!r}."
            )

        if (
            int(
                label_diagnostics.get(
                    "selected",
                    -1,
                )
            )
            != int(
                requested_count
            )
        ):
            raise AssertionError(
                "GroundingDINO did not select the requested "
                f"number of instances for {label!r}: "
                f"{label_diagnostics}"
            )

    # ---------------------------------------------------------
    # Important generalized artifacts.
    # ---------------------------------------------------------

    artifacts_dir = (
        annotated_video_path.parent
    )

    _validate_object_discovery_artifact(
        artifacts_dir
        / "object_discovery.json"
    )

    dino_input_path = (
        artifacts_dir
        / "groundingdino_input.png"
    )

    if not dino_input_path.is_file():
        raise AssertionError(
            "Missing GroundingDINO input artifact: "
            f"{dino_input_path}"
        )

    dino_input = cv2.imread(
        str(dino_input_path),
        cv2.IMREAD_COLOR,
    )

    if dino_input is None:
        raise AssertionError(
            "GroundingDINO input artifact is not readable: "
            f"{dino_input_path}"
        )

    if (
        dino_input.ndim != 3
        or dino_input.shape[2] != 3
    ):
        raise AssertionError(
            "GroundingDINO input artifact has invalid shape: "
            f"{dino_input.shape}"
        )

    # ---------------------------------------------------------
    # Save integration handoff artifact.
    # ---------------------------------------------------------

    handoff_path = (
        annotated_video_path.parent
        / "visual_prompting_result.json"
    )

    handoff_result = {
        "source_video_path": str(
            video_path
        ),
        "keyframes": list(
            expected_keyframes
        ),
        "annotated_video_path": str(
            annotated_video_path
        ),
        "track_id_map": {
            str(track_id): info
            for track_id, info
            in result.track_id_map.items()
        },
        "key_frame_coordinates": (
            result.key_frame_coordinates
        ),
        "bounding_box_summary": (
            result.bounding_box_summary
        ),
        "count_diagnostics": (
            result.count_diagnostics
        ),
    }

    with handoff_path.open(
        "w",
        encoding="utf-8",
    ) as stream:
        json.dump(
            handoff_result,
            stream,
            indent=2,
            ensure_ascii=False,
        )

    print(
        "Visual prompting handoff: "
        f"{handoff_path}"
    )

    # ---------------------------------------------------------
    # Report.
    # ---------------------------------------------------------

    print("Visual prompting completed")
    print(
        "Annotated video: "
        f"{annotated_video_path}"
    )
    print(
        "Source video frames: "
        f"{source_frame_count}"
    )
    print(
        "Tracked objects: "
        f"{len(track_ids)}"
    )
    print(
        "Storage bins: "
        f"{len(bin_track_ids)}"
    )

    print("\nTracked objects:")

    for (
        track_id,
        info,
    ) in result.track_id_map.items():
        print(
            f"  Track {track_id}: "
            f"{info}"
        )

    print(
        "\nKey-frame coordinates:"
    )

    for (
        frame_name,
        coordinates,
    ) in (
        result
        .key_frame_coordinates
        .items()
    ):
        print(
            f"  {frame_name}: "
            f"{coordinates}"
        )

    print(
        "\nCount diagnostics:"
    )

    print(
        json.dumps(
            diagnostics,
            indent=2,
            sort_keys=True,
        )
    )

    print(
        "\nTEST PASSED"
    )

    return 0
