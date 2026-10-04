from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

from results import VisualPromptingResult

from ai_controller.models.seedo_controller.demo_structured_scene_builder import (
    DemoStructuredSceneBuilder,
)


DEFAULT_VISUAL_PROMPTING_RESULT = Path(
    "/seedo_tests/visual_prompting/visual_prompting_result.json"
)

EXPECTED_OBJECTS = {
    "0": ("bin", (203.0, 298.0)),
    "1": ("bin", (305.0, 296.0)),
    "2": ("bin", (408.0, 293.0)),
    "3": ("bin", (511.0, 290.0)),
}


def _load_visual_prompting_result(
    artifact_path: Path,
) -> VisualPromptingResult:
    """Reconstruct the real VisualPromptingResult from the integration handoff."""

    normalized_path = (
        artifact_path
        .expanduser()
        .resolve()
    )

    if not normalized_path.is_file():
        raise FileNotFoundError(
            "Visual prompting handoff artifact does not exist: "
            f"{normalized_path}"
        )

    if normalized_path.stat().st_size == 0:
        raise ValueError(
            "Visual prompting handoff artifact is empty: "
            f"{normalized_path}"
        )

    with normalized_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        data = json.load(
            stream
        )

    annotated_video_path = (
        Path(
            data["annotated_video_path"]
        )
        .expanduser()
        .resolve()
    )

    if not annotated_video_path.is_file():
        raise FileNotFoundError(
            "Annotated video referenced by the handoff artifact "
            "does not exist: "
            f"{annotated_video_path}"
        )

    raw_track_id_map = data.get(
        "track_id_map"
    )

    if not isinstance(
        raw_track_id_map,
        dict,
    ) or not raw_track_id_map:
        raise ValueError(
            "The visual prompting handoff contains an invalid "
            "track_id_map."
        )

    track_id_map = {
        int(track_id): dict(track_info)
        for track_id, track_info
        in raw_track_id_map.items()
    }

    key_frame_coordinates = data.get(
        "key_frame_coordinates"
    )

    if not isinstance(
        key_frame_coordinates,
        dict,
    ) or not key_frame_coordinates:
        raise ValueError(
            "The visual prompting handoff contains invalid "
            "key_frame_coordinates."
        )

    bounding_box_summary = str(
        data.get(
            "bounding_box_summary",
            "",
        )
    )

    if not bounding_box_summary.strip():
        raise ValueError(
            "The visual prompting handoff contains an empty "
            "bounding_box_summary."
        )

    count_diagnostics = data.get(
        "count_diagnostics"
    )

    if not isinstance(
        count_diagnostics,
        dict,
    ) or not count_diagnostics:
        raise ValueError(
            "The visual prompting handoff contains invalid "
            "count_diagnostics."
        )

    return VisualPromptingResult(
        annotated_video_path=annotated_video_path,
        track_id_map=track_id_map,
        key_frame_coordinates={
            str(frame_key): list(entries)
            for frame_key, entries
            in key_frame_coordinates.items()
        },
        bounding_box_summary=bounding_box_summary,
        count_diagnostics=dict(
            count_diagnostics
        ),
    )


def run_demo_structured_scene_builder_test(
    args: argparse.Namespace,
) -> int:
    """Run DemoStructuredSceneBuilder on the real VisualPrompter output."""

    handoff_path = Path(
        getattr(
            args,
            "visual_prompting_result",
            None,
        )
        or DEFAULT_VISUAL_PROMPTING_RESULT
    )

    if args.artifacts_dir is None:
        artifacts_dir = Path(
            "/seedo_tests/demo_structured_scene_builder"
        )
    else:
        artifacts_dir = (
            Path(args.artifacts_dir)
            .expanduser()
            .resolve()
        )

    visual_prompting_result = (
        _load_visual_prompting_result(
            handoff_path
        )
    )

    builder = DemoStructuredSceneBuilder(
        directions=8,
    )

    structured_scene = builder.run(
        visual_prompting_result=(
            visual_prompting_result
        ),
        artifacts_dir=artifacts_dir,
    )

    # ---------------------------------------------------------
    # Validate top-level scene.
    # ---------------------------------------------------------

    if structured_scene.directions != 8:
        raise AssertionError(
            "Unexpected direction mode: "
            f"{structured_scene.directions}"
        )

    if len(structured_scene.objects) != 4:
        raise AssertionError(
            "Expected exactly four demonstration place objects, "
            f"received {len(structured_scene.objects)}."
        )

    if len(structured_scene.relations) != 12:
        raise AssertionError(
            "Expected 4 * 3 = 12 directed spatial relations, "
            f"received {len(structured_scene.relations)}."
        )

    # ---------------------------------------------------------
    # Validate filtering and SAM centroids.
    # ---------------------------------------------------------

    returned_objects = {
        obj.object_id: obj
        for obj
        in structured_scene.objects
    }

    if set(returned_objects) != set(
        EXPECTED_OBJECTS
    ):
        raise AssertionError(
            "Unexpected structured-scene object IDs: "
            f"expected={sorted(EXPECTED_OBJECTS)}, "
            f"received={sorted(returned_objects)}"
        )

    for (
        object_id,
        (
            expected_category,
            expected_center,
        ),
    ) in EXPECTED_OBJECTS.items():
        obj = returned_objects[
            object_id
        ]

        if obj.category != expected_category:
            raise AssertionError(
                "Unexpected category for demonstration object "
                f"{object_id}: "
                f"expected={expected_category!r}, "
                f"received={obj.category!r}"
            )

        if obj.center != expected_center:
            raise AssertionError(
                "Unexpected SAM centroid for demonstration object "
                f"{object_id}: "
                f"expected={expected_center}, "
                f"received={obj.center}"
            )

    # The four manipulable blocks must not participate in the
    # destination structured scene.
    excluded_track_ids = {
        "4",
        "5",
        "6",
        "7",
    }

    if (
        set(returned_objects)
        & excluded_track_ids
    ):
        raise AssertionError(
            "Non-place demonstration objects were included in "
            "the structured scene."
        )

    # ---------------------------------------------------------
    # Validate complete directed relation graph.
    # ---------------------------------------------------------

    expected_pairs = {
        (
            subject_id,
            reference_id,
        )
        for subject_id in EXPECTED_OBJECTS
        for reference_id in EXPECTED_OBJECTS
        if subject_id != reference_id
    }

    returned_pairs: set[
        tuple[str, str]
    ] = set()

    for relation in (
        structured_scene.relations
    ):
        pair = (
            relation.subject_object_id,
            relation.reference_object_id,
        )

        if (
            relation.subject_object_id
            == relation.reference_object_id
        ):
            raise AssertionError(
                "Structured scene contains a self-relation: "
                f"{relation}"
            )

        if pair in returned_pairs:
            raise AssertionError(
                "Structured scene contains a duplicate relation pair: "
                f"{pair}"
            )

        if not str(
            relation.relation
        ).strip():
            raise AssertionError(
                "Structured scene contains an empty qualitative relation: "
                f"{relation}"
            )

        returned_pairs.add(
            pair
        )

    if returned_pairs != expected_pairs:
        raise AssertionError(
            "Structured scene does not contain the complete "
            "directed relation graph. "
            f"missing={sorted(expected_pairs - returned_pairs)}, "
            f"unexpected={sorted(returned_pairs - expected_pairs)}"
        )

    # ---------------------------------------------------------
    # Validate generated artifact.
    # ---------------------------------------------------------

    structured_scene_path = (
        artifacts_dir
        / "demo_structured_scene.json"
    )

    if not structured_scene_path.is_file():
        raise AssertionError(
            "Demo structured-scene artifact was not created: "
            f"{structured_scene_path}"
        )

    if (
        structured_scene_path.stat().st_size
        == 0
    ):
        raise AssertionError(
            "Demo structured-scene artifact is empty: "
            f"{structured_scene_path}"
        )

    with structured_scene_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        artifact = json.load(
            stream
        )

    expected_artifact = json.loads(
        json.dumps(
            asdict(structured_scene),
            ensure_ascii=False,
        )
    )

    if artifact != expected_artifact:
        raise AssertionError(
            "demo_structured_scene.json does not match "
            "the returned StructuredScene."
        )

    # ---------------------------------------------------------
    # Report.
    # ---------------------------------------------------------

    print(
        "Demo structured scene completed"
    )
    print(
        "Visual prompting handoff: "
        f"{handoff_path.expanduser().resolve()}"
    )
    print(
        "Structured-scene artifact: "
        f"{structured_scene_path}"
    )
    print(
        "Direction mode: "
        f"{structured_scene.directions}"
    )
    print(
        "Place objects: "
        f"{len(structured_scene.objects)}"
    )
    print(
        "Directed relations: "
        f"{len(structured_scene.relations)}"
    )

    print(
        "\nStructured objects:"
    )

    for obj in (
        structured_scene.objects
    ):
        print(
            f"  {obj.object_id}: "
            f"category={obj.category}, "
            f"center={obj.center}"
        )

    print(
        "\nSpatial relations:"
    )

    for relation in (
        structured_scene.relations
    ):
        print(
            "  "
            f"{relation.subject_object_id} "
            f"{relation.relation} "
            f"{relation.reference_object_id}"
        )

    print(
        "\nTEST PASSED"
    )

    return 0
