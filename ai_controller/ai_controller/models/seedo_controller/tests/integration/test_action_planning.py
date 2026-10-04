from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

from ai_controller.models.seedo_controller.action_planner import (
    ActionPlanner,
)


DEFAULT_VISUAL_PROMPTING_RESULT = Path(
    "/seedo_tests/visual_prompting/visual_prompting_result.json"
)

EXPECTED_KEYFRAMES = (20, 35)
EXPECTED_PICKED_TRACK_ID = 5
EXPECTED_DESTINATION_TRACK_ID = 0
EXPECTED_PICKED_LABEL = "green block"
EXPECTED_PICKED_CATEGORY = "block"
EXPECTED_DESTINATION_CATEGORY = "bin"
EXPECTED_RELATION = "in"
EXPECTED_TASK_TYPE = "pick_and_place"
EXPECTED_DESTINATION_CENTER = (203, 303)

EXPECTED_ACTION = (
    "Pick the green block and place it into the "
    "storage bin centered at (x=203, y=303) "
    "in the demonstration place frame."
)


def _load_visual_prompting_handoff(
    artifact_path: Path,
) -> dict:
    """Load and validate the VisualPrompter integration handoff."""

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

    required_fields = {
        "source_video_path",
        "keyframes",
        "annotated_video_path",
        "track_id_map",
        "key_frame_coordinates",
        "bounding_box_summary",
        "count_diagnostics",
    }

    missing_fields = (
        required_fields
        - set(data)
    )

    if missing_fields:
        raise ValueError(
            "Visual prompting handoff is missing fields: "
            f"{sorted(missing_fields)}"
        )

    return data


def _normalize_track_id_map(
    value: dict,
) -> dict[int, dict[str, object]]:
    if not isinstance(
        value,
        dict,
    ) or not value:
        raise ValueError(
            "Visual prompting handoff contains an invalid track_id_map."
        )

    return {
        int(track_id): dict(track_info)
        for track_id, track_info
        in value.items()
    }


def _load_json(
    path: Path,
) -> dict:
    if not path.is_file():
        raise AssertionError(
            f"Missing action-planning artifact: {path}"
        )

    if path.stat().st_size == 0:
        raise AssertionError(
            f"Action-planning artifact is empty: {path}"
        )

    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        return json.load(
            stream
        )


def run_action_planning_test(
    args: argparse.Namespace,
) -> int:
    """Run the real generalized ActionPlanner integration test."""

    handoff_path = Path(
        getattr(
            args,
            "visual_prompting_result",
            None,
        )
        or DEFAULT_VISUAL_PROMPTING_RESULT
    )

    handoff = (
        _load_visual_prompting_handoff(
            handoff_path
        )
    )

    keyframes = tuple(
        int(frame)
        for frame
        in handoff["keyframes"]
    )

    if keyframes != EXPECTED_KEYFRAMES:
        raise AssertionError(
            "Unexpected keyframes in VisualPrompter handoff: "
            f"expected={EXPECTED_KEYFRAMES}, "
            f"received={keyframes}"
        )

    annotated_video_path = (
        Path(
            handoff[
                "annotated_video_path"
            ]
        )
        .expanduser()
        .resolve()
    )

    if not annotated_video_path.is_file():
        raise FileNotFoundError(
            "Annotated VisualPrompter video does not exist: "
            f"{annotated_video_path}"
        )

    if (
        annotated_video_path.stat().st_size
        == 0
    ):
        raise ValueError(
            "Annotated VisualPrompter video is empty: "
            f"{annotated_video_path}"
        )

    track_id_map = (
        _normalize_track_id_map(
            handoff["track_id_map"]
        )
    )

    key_frame_coordinates = (
        handoff[
            "key_frame_coordinates"
        ]
    )

    if not isinstance(
        key_frame_coordinates,
        dict,
    ) or not key_frame_coordinates:
        raise ValueError(
            "Visual prompting handoff contains invalid "
            "key_frame_coordinates."
        )

    expected_coordinate_keys = {
        "key_frame20",
        "key_frame35",
    }

    if (
        set(
            key_frame_coordinates
        )
        != expected_coordinate_keys
    ):
        raise AssertionError(
            "Unexpected key-frame coordinate keys in handoff: "
            f"{sorted(key_frame_coordinates)}"
        )

    artifacts_dir = (
        Path(
            args.artifacts_dir
            or "/seedo_tests/action_planning"
        )
        .expanduser()
        .resolve()
    )

    planner = ActionPlanner(
        model="gpt-4o-2024-08-06",
        demonstration_bin_order=(
            "left_to_right"
        ),
        perception_mode="generalized",
    )

    action_result = planner.run(
        annotated_video_path=(
            annotated_video_path
        ),
        keyframes=keyframes,
        track_id_map=track_id_map,
        key_frame_coordinates=(
            key_frame_coordinates
        ),
        artifacts_dir=artifacts_dir,
    )

    # ---------------------------------------------------------
    # Validate top-level result.
    # ---------------------------------------------------------

    if action_result.status != "completed":
        raise AssertionError(
            "Expected a completed action plan, "
            f"received status={action_result.status!r}, "
            f"ambiguities={action_result.ambiguities}"
        )

    if (
        str(
            action_result.task_type
        ).strip().lower()
        != EXPECTED_TASK_TYPE
    ):
        raise AssertionError(
            "Unexpected task classification: "
            f"expected={EXPECTED_TASK_TYPE!r}, "
            f"received={action_result.task_type!r}"
        )

    if action_result.ambiguities:
        raise AssertionError(
            "Completed action plan contains unexpected ambiguities: "
            f"{action_result.ambiguities}"
        )

    if len(action_result.steps) != 1:
        raise AssertionError(
            "Expected exactly one demonstrated action step, "
            f"received {len(action_result.steps)}."
        )

    step = action_result.steps[0]

    # ---------------------------------------------------------
    # Validate VLM-selected identities and semantics.
    # ---------------------------------------------------------

    if step.pick_keyframe != EXPECTED_KEYFRAMES[0]:
        raise AssertionError(
            "Unexpected pick keyframe: "
            f"{step.pick_keyframe}"
        )

    if step.place_keyframe != EXPECTED_KEYFRAMES[1]:
        raise AssertionError(
            "Unexpected place keyframe: "
            f"{step.place_keyframe}"
        )

    if (
        step.picked_track_id
        != EXPECTED_PICKED_TRACK_ID
    ):
        raise AssertionError(
            "Unexpected picked track: "
            f"expected={EXPECTED_PICKED_TRACK_ID}, "
            f"received={step.picked_track_id}"
        )

    if (
        step.destination_track_id
        != EXPECTED_DESTINATION_TRACK_ID
    ):
        raise AssertionError(
            "Unexpected destination track: "
            f"expected={EXPECTED_DESTINATION_TRACK_ID}, "
            f"received={step.destination_track_id}"
        )

    if (
        step.picked_detector_label
        != EXPECTED_PICKED_LABEL
    ):
        raise AssertionError(
            "Unexpected picked detector label: "
            f"expected={EXPECTED_PICKED_LABEL!r}, "
            f"received={step.picked_detector_label!r}"
        )

    if (
        step.picked_category
        != EXPECTED_PICKED_CATEGORY
    ):
        raise AssertionError(
            "Unexpected picked category: "
            f"expected={EXPECTED_PICKED_CATEGORY!r}, "
            f"received={step.picked_category!r}"
        )

    if step.picked_color != "":
        raise AssertionError(
            "Generalized action planning must not use picked_color: "
            f"received={step.picked_color!r}"
        )

    if (
        step.destination_category
        != EXPECTED_DESTINATION_CATEGORY
    ):
        raise AssertionError(
            "Unexpected destination category: "
            f"expected={EXPECTED_DESTINATION_CATEGORY!r}, "
            f"received={step.destination_category!r}"
        )

    if (
        step.destination_ordinal_from_left
        is not None
    ):
        raise AssertionError(
            "Generalized action planning must not produce "
            "destination_ordinal_from_left."
        )

    if (
        step.relation.strip().lower()
        != EXPECTED_RELATION
    ):
        raise AssertionError(
            "Unexpected placement relation: "
            f"expected={EXPECTED_RELATION!r}, "
            f"received={step.relation!r}"
        )

    # ---------------------------------------------------------
    # Validate semantic consistency with VisualPrompter metadata.
    # ---------------------------------------------------------

    picked_info = track_id_map[
        EXPECTED_PICKED_TRACK_ID
    ]

    destination_info = track_id_map[
        EXPECTED_DESTINATION_TRACK_ID
    ]

    if (
        picked_info[
            "detector_label"
        ]
        != step.picked_detector_label
    ):
        raise AssertionError(
            "Picked detector label does not match "
            "VisualPrompter metadata."
        )

    if (
        picked_info["category"]
        != step.picked_category
    ):
        raise AssertionError(
            "Picked category does not match "
            "VisualPrompter metadata."
        )

    if (
        destination_info["category"]
        != step.destination_category
    ):
        raise AssertionError(
            "Destination category does not match "
            "VisualPrompter metadata."
        )

    # ---------------------------------------------------------
    # Validate deterministic destination center and action text.
    # ---------------------------------------------------------

    place_coordinates = (
        key_frame_coordinates[
            "key_frame35"
        ]
    )

    expected_destination_entry = (
        "Object 0: "
        f"({EXPECTED_DESTINATION_CENTER[0]}, "
        f"{EXPECTED_DESTINATION_CENTER[1]})"
    )

    if (
        expected_destination_entry
        not in place_coordinates
    ):
        raise AssertionError(
            "Expected destination center is missing from "
            "VisualPrompter place-frame coordinates: "
            f"{expected_destination_entry!r}"
        )

    if step.action != EXPECTED_ACTION:
        raise AssertionError(
            "Unexpected deterministic action description:\n"
            f"expected: {EXPECTED_ACTION}\n"
            f"received: {step.action}"
        )

    if (
        action_result.natural_language_plan
        != EXPECTED_ACTION
    ):
        raise AssertionError(
            "Natural-language plan does not match the "
            "deterministically generated action."
        )

    # ---------------------------------------------------------
    # Validate input_manifest.json.
    # ---------------------------------------------------------

    manifest_path = (
        artifacts_dir
        / "input_manifest.json"
    )

    manifest = _load_json(
        manifest_path
    )

    if manifest.get("status") != "completed":
        raise AssertionError(
            "Action-planning manifest did not complete: "
            f"{manifest}"
        )

    if (
        manifest.get(
            "perception_mode"
        )
        != "generalized"
    ):
        raise AssertionError(
            "Action-planning manifest has an unexpected "
            "perception_mode."
        )

    if (
        manifest.get("model")
        != planner.model
    ):
        raise AssertionError(
            "Action-planning manifest has an unexpected model."
        )

    if (
        manifest.get(
            "input_video"
        )
        != str(
            annotated_video_path
        )
    ):
        raise AssertionError(
            "Action-planning manifest references an unexpected video."
        )

    if (
        manifest.get(
            "frame_roles"
        )
        != {
            "initial": 0,
            "pick": 20,
            "place": 35,
        }
    ):
        raise AssertionError(
            "Action-planning manifest contains unexpected frame roles: "
            f"{manifest.get('frame_roles')}"
        )

    if (
        manifest.get(
            "track_ids"
        )
        != list(
            range(8)
        )
    ):
        raise AssertionError(
            "Action-planning manifest contains unexpected track IDs."
        )

    if not str(
        manifest.get(
            "input_sha256",
            "",
        )
    ).strip():
        raise AssertionError(
            "Action-planning manifest is missing input_sha256."
        )

    if not manifest.get(
        "openai_response_id"
    ):
        raise AssertionError(
            "Action-planning manifest is missing openai_response_id."
        )

    # ---------------------------------------------------------
    # Validate generalized raw OpenAI response.
    # ---------------------------------------------------------

    raw_response_path = (
        artifacts_dir
        / "raw_response.json"
    )

    raw_response = _load_json(
        raw_response_path
    )

    if (
        raw_response.get(
            "status"
        )
        != "completed"
    ):
        raise AssertionError(
            "Raw OpenAI action-planning response is not completed."
        )

    if (
        raw_response.get(
            "task_type"
        )
        != EXPECTED_TASK_TYPE
    ):
        raise AssertionError(
            "Raw OpenAI response contains an unexpected task_type."
        )

    raw_steps = raw_response.get(
        "steps"
    )

    if (
        not isinstance(
            raw_steps,
            list,
        )
        or len(raw_steps) != 1
    ):
        raise AssertionError(
            "Raw OpenAI response must contain exactly one step."
        )

    raw_step = raw_steps[0]

    # These fields intentionally do NOT belong to the generalized
    # OpenAI schema. They are created deterministically afterward.
    if (
        "destination_ordinal_from_left"
        in raw_step
    ):
        raise AssertionError(
            "Generalized raw OpenAI response unexpectedly contains "
            "destination_ordinal_from_left."
        )

    if "action" in raw_step:
        raise AssertionError(
            "Generalized raw OpenAI response unexpectedly contains "
            "a natural-language action."
        )

    if (
        int(
            raw_step[
                "picked_track_id"
            ]
        )
        != EXPECTED_PICKED_TRACK_ID
    ):
        raise AssertionError(
            "Raw OpenAI response selected an unexpected picked track."
        )

    if (
        int(
            raw_step[
                "destination_track_id"
            ]
        )
        != EXPECTED_DESTINATION_TRACK_ID
    ):
        raise AssertionError(
            "Raw OpenAI response selected an unexpected destination track."
        )

    # ---------------------------------------------------------
    # Validate deterministic final artifacts.
    # ---------------------------------------------------------

    action_plan_json_path = (
        artifacts_dir
        / "action_plan.json"
    )

    action_plan_json = _load_json(
        action_plan_json_path
    )

    expected_action_plan_json = (
        json.loads(
            json.dumps(
                asdict(
                    action_result
                ),
                ensure_ascii=False,
            )
        )
    )

    if (
        action_plan_json
        != expected_action_plan_json
    ):
        raise AssertionError(
            "action_plan.json does not match the returned "
            "ActionPlanningResult."
        )

    action_plan_txt_path = (
        artifacts_dir
        / "action_plan.txt"
    )

    if not action_plan_txt_path.is_file():
        raise AssertionError(
            "Missing action_plan.txt artifact."
        )

    if (
        action_plan_txt_path
        .read_text(
            encoding="utf-8"
        )
        .strip()
        != EXPECTED_ACTION
    ):
        raise AssertionError(
            "action_plan.txt does not contain the expected "
            "deterministic action."
        )

    prompt_path = (
        artifacts_dir
        / "prompt.txt"
    )

    if not prompt_path.is_file():
        raise AssertionError(
            "Missing prompt.txt artifact."
        )

    prompt = prompt_path.read_text(
        encoding="utf-8"
    )

    required_prompt_fragments = (
        "TASK CLASSIFICATION:",
        "OBJECT METADATA:",
        "PICK OBJECT:",
        "DESTINATION:",
        "DESTINATION IDENTITY:",
        "RELATION:",
        "OUTPUT SEMANTICS:",
        "Tracking evidence:",
    )

    for fragment in (
        required_prompt_fragments
    ):
        if fragment not in prompt:
            raise AssertionError(
                "Action-planning prompt is missing required section: "
                f"{fragment}"
            )

    # ---------------------------------------------------------
    # Report.
    # ---------------------------------------------------------

    print(
        "Action planning completed"
    )
    print(
        "Visual prompting handoff: "
        f"{handoff_path.expanduser().resolve()}"
    )
    print(
        "Annotated video: "
        f"{annotated_video_path}"
    )
    print(
        "Keyframes: "
        f"{list(keyframes)}"
    )
    print(
        "Task type: "
        f"{action_result.task_type}"
    )
    print(
        "Picked track: "
        f"{step.picked_track_id} "
        f"({step.picked_detector_label})"
    )
    print(
        "Destination track: "
        f"{step.destination_track_id} "
        f"({step.destination_category})"
    )
    print(
        "Relation: "
        f"{step.relation}"
    )
    print(
        "Natural-language plan: "
        f"{action_result.natural_language_plan}"
    )
    print(
        "Artifacts directory: "
        f"{artifacts_dir}"
    )

    print(
        "\nTEST PASSED"
    )

    return 0
