from __future__ import annotations

import argparse
import json
from pathlib import Path

from ai_controller.models.seedo_controller.action_planner import (
    ActionPlanner,
)


DEFAULT_VISUAL_PROMPTING_RESULT = Path(
    "/seedo_tests/visual_prompting_nut/visual_prompting_result.json"
)

DEFAULT_ARTIFACTS_DIR = Path(
    "/seedo_tests/action_planning_nut"
)


def _load_json(
    path: Path,
) -> dict:
    if not path.is_file():
        raise FileNotFoundError(
            f"JSON file does not exist: {path}"
        )

    if path.stat().st_size == 0:
        raise ValueError(
            f"JSON file is empty: {path}"
        )

    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        return json.load(
            stream
        )


def run_action_planning_nut_test(
    visual_prompting_result: Path,
    artifacts_dir: Path,
) -> int:
    """
    Run the real generalized ActionPlanner pipeline on the nut/ring demo.

    This test intentionally contains no task-specific ground truth.

    It only:
      - loads the VisualPrompter handoff;
      - runs the real ActionPlanner;
      - lets the VLM infer the demonstrated action;
      - verifies that the expected pipeline artifacts were produced;
      - prints the resulting plan for inspection.
    """

    handoff_path = (
        Path(
            visual_prompting_result
        )
        .expanduser()
        .resolve()
    )

    artifacts_dir = (
        Path(
            artifacts_dir
        )
        .expanduser()
        .resolve()
    )

    handoff = _load_json(
        handoff_path
    )

    # ---------------------------------------------------------
    # Extract VisualPrompter handoff.
    # ---------------------------------------------------------

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

    keyframes = tuple(
        int(frame)
        for frame in handoff[
            "keyframes"
        ]
    )

    if len(keyframes) != 2:
        raise ValueError(
            "VisualPrompter handoff must contain exactly "
            f"two keyframes, received={keyframes}"
        )

    track_id_map = {
        int(track_id): dict(
            track_info
        )
        for track_id, track_info
        in handoff[
            "track_id_map"
        ].items()
    }

    key_frame_coordinates = (
        handoff[
            "key_frame_coordinates"
        ]
    )

    artifacts_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    # ---------------------------------------------------------
    # Run real ActionPlanner.
    # ---------------------------------------------------------

    print(
        "[Nut ActionPlanner Test] Starting pipeline"
    )

    print(
        "[Nut ActionPlanner Test] VisualPrompter handoff: "
        f"{handoff_path}"
    )

    print(
        "[Nut ActionPlanner Test] Annotated video: "
        f"{annotated_video_path}"
    )

    print(
        "[Nut ActionPlanner Test] Keyframes: "
        f"{keyframes}"
    )

    print(
        "[Nut ActionPlanner Test] Artifacts: "
        f"{artifacts_dir}"
    )

    planner = ActionPlanner(
        model="gpt-4o-2024-08-06",
        demonstration_bin_order="left_to_right",
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

    if action_result is None:
        raise RuntimeError(
            "ActionPlanner returned None."
        )

    # ---------------------------------------------------------
    # Verify only that pipeline artifacts exist.
    # No scene-specific assertions.
    # ---------------------------------------------------------

    expected_artifacts = (
        "input_manifest.json",
        "prompt.txt",
        "raw_response.json",
        "action_plan.json",
        "action_plan.txt",
    )

    for artifact_name in expected_artifacts:
        artifact_path = (
            artifacts_dir
            / artifact_name
        )

        if not artifact_path.is_file():
            raise AssertionError(
                "ActionPlanner did not produce expected artifact: "
                f"{artifact_path}"
            )

        if artifact_path.stat().st_size == 0:
            raise AssertionError(
                "ActionPlanner produced an empty artifact: "
                f"{artifact_path}"
            )

    # ---------------------------------------------------------
    # Load artifacts so malformed JSON also fails the test.
    # ---------------------------------------------------------

    manifest = _load_json(
        artifacts_dir
        / "input_manifest.json"
    )

    raw_response = _load_json(
        artifacts_dir
        / "raw_response.json"
    )

    action_plan_json = _load_json(
        artifacts_dir
        / "action_plan.json"
    )

    # ---------------------------------------------------------
    # Report.
    # ---------------------------------------------------------

    print(
        "\n[Nut ActionPlanner Test] Pipeline completed"
    )

    print(
        "Status: "
        f"{action_result.status}"
    )

    print(
        "Task type: "
        f"{action_result.task_type}"
    )

    print(
        "Ambiguities: "
        f"{action_result.ambiguities}"
    )

    print(
        "Natural-language plan: "
        f"{action_result.natural_language_plan}"
    )

    print(
        "\nAction steps:"
    )

    if not action_result.steps:
        print(
            "  No action steps returned."
        )

    for index, step in enumerate(
        action_result.steps
    ):
        print(
            f"\n  Step {index}:"
        )

        print(
            "    pick_keyframe: "
            f"{step.pick_keyframe}"
        )

        print(
            "    place_keyframe: "
            f"{step.place_keyframe}"
        )

        print(
            "    picked_track_id: "
            f"{step.picked_track_id}"
        )

        print(
            "    picked_detector_label: "
            f"{step.picked_detector_label}"
        )

        print(
            "    picked_category: "
            f"{step.picked_category}"
        )

        print(
            "    destination_track_id: "
            f"{step.destination_track_id}"
        )

        print(
            "    destination_category: "
            f"{step.destination_category}"
        )

        print(
            "    relation: "
            f"{step.relation}"
        )

        print(
            "    grasp_instruction: "
            f"{step.grasp_instruction}"
        )

        print(
            "    action: "
            f"{step.action}"
        )

    print(
        "\nRaw OpenAI response:"
    )

    print(
        json.dumps(
            raw_response,
            indent=2,
            ensure_ascii=False,
        )
    )

    print(
        "\nFinal action_plan.json:"
    )

    print(
        json.dumps(
            action_plan_json,
            indent=2,
            ensure_ascii=False,
        )
    )

    print(
        "\nManifest status: "
        f"{manifest.get('status')}"
    )

    print(
        "Artifacts directory: "
        f"{artifacts_dir}"
    )

    print(
        "\nTEST PASSED"
    )

    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run the generalized ActionPlanner pipeline "
            "on the nut/ring demonstration without "
            "task-specific assertions."
        )
    )

    parser.add_argument(
        "--visual-prompting-result",
        type=Path,
        default=DEFAULT_VISUAL_PROMPTING_RESULT,
        help=(
            "VisualPrompter handoff JSON."
        ),
    )

    parser.add_argument(
        "--artifacts-dir",
        type=Path,
        default=DEFAULT_ARTIFACTS_DIR,
        help=(
            "Directory in which ActionPlanner artifacts "
            "will be written."
        ),
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    return run_action_planning_nut_test(
        visual_prompting_result=(
            args.visual_prompting_result
        ),
        artifacts_dir=(
            args.artifacts_dir
        ),
    )


if __name__ == "__main__":
    raise SystemExit(
        main()
    )