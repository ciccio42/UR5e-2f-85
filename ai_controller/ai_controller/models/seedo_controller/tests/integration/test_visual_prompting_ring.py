from __future__ import annotations

import argparse
import json
from pathlib import Path

from ai_controller.models.seedo_controller.visual_prompter import (
    VisualPrompter,
)


DEFAULT_ARTIFACTS_DIR = Path(
    "/seedo_tests/visual_prompting_ring"
)


def run_visual_prompting_ring_test(
    video: Path,
    pick_frame: int,
    place_frame: int,
    artifacts_dir: Path,
    grounding_config: Path,
    grounding_checkpoint: Path,
    bert_model: Path,
    sam_checkpoint: Path,
    sam2_checkpoint: Path,
) -> int:
    """
    Run the real generalized VisualPrompter pipeline on the ring demo.

    This test intentionally does not validate scene-specific semantics,
    object counts, detector labels, categories, or expected tracks.

    Its purpose is only to execute the complete VisualPrompter pipeline
    and persist the artifacts required by the following pipeline stages.
    """

    video_path = (
        Path(video)
        .expanduser()
        .resolve()
    )

    artifacts_dir = (
        Path(artifacts_dir)
        .expanduser()
        .resolve()
    )

    if not video_path.is_file():
        raise FileNotFoundError(
            f"Input video does not exist: {video_path}"
        )

    if video_path.stat().st_size == 0:
        raise ValueError(
            f"Input video is empty: {video_path}"
        )

    keyframes = (
        int(pick_frame),
        int(place_frame),
    )

    if keyframes[0] >= keyframes[1]:
        raise ValueError(
            "The pick frame must precede the place frame: "
            f"{keyframes}"
        )

    artifacts_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    print(
        "[Ring VisualPrompter Test] Starting pipeline"
    )
    print(
        f"[Ring VisualPrompter Test] Video: {video_path}"
    )
    print(
        f"[Ring VisualPrompter Test] Keyframes: {keyframes}"
    )
    print(
        f"[Ring VisualPrompter Test] Artifacts: {artifacts_dir}"
    )

    prompter = VisualPrompter(
        grounding_config=grounding_config,
        grounding_checkpoint=grounding_checkpoint,
        bert_model=bert_model,
        sam_checkpoint=sam_checkpoint,
        sam2_checkpoint=sam2_checkpoint,
        objects=None,
        perception_mode="generalized",
    )

    result = prompter.run(
        video_path=video_path,
        keyframes=keyframes,
        artifacts_dir=artifacts_dir,
    )

    if result is None:
        raise RuntimeError(
            "VisualPrompter returned None."
        )

    # ---------------------------------------------------------
    # Save handoff for ActionPlanner.
    # ---------------------------------------------------------

    handoff_path = (
        artifacts_dir
        / "visual_prompting_result.json"
    )

    handoff_result = {
        "source_video_path": str(
            video_path
        ),
        "keyframes": list(
            keyframes
        ),
        "annotated_video_path": str(
            Path(
                result.annotated_video_path
            )
            .expanduser()
            .resolve()
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

    # ---------------------------------------------------------
    # Report only.
    # No task-specific assertions.
    # ---------------------------------------------------------

    print(
        "\n[Ring VisualPrompter Test] Pipeline completed"
    )

    print(
        "Annotated video: "
        f"{result.annotated_video_path}"
    )

    print(
        "Tracked objects:"
    )

    for track_id, info in (
        result.track_id_map.items()
    ):
        print(
            f"  Track {track_id}: {info}"
        )

    print(
        "\nKey-frame coordinates:"
    )

    for frame_name, coordinates in (
        result.key_frame_coordinates.items()
    ):
        print(
            f"  {frame_name}: {coordinates}"
        )

    print(
        "\nCount diagnostics:"
    )

    print(
        json.dumps(
            result.count_diagnostics,
            indent=2,
            sort_keys=True,
        )
    )

    print(
        "\nVisual prompting handoff:"
    )

    print(
        handoff_path
    )

    print(
        "\nTEST PASSED"
    )

    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run the generalized VisualPrompter pipeline "
            "on the ring demonstration without task-specific assertions."
        )
    )

    parser.add_argument(
        "--video",
        type=Path,
        required=True,
        help="Path to the ring demonstration video.",
    )

    parser.add_argument(
        "--pick-frame",
        type=int,
        required=True,
        help="Pick-event keyframe.",
    )

    parser.add_argument(
        "--place-frame",
        type=int,
        required=True,
        help="Place-event keyframe.",
    )

    parser.add_argument(
        "--artifacts-dir",
        type=Path,
        default=DEFAULT_ARTIFACTS_DIR,
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
            "/opt/checkpoints/seedo/sam/"
            "sam_vit_h_4b8939.pth"
        ),
    )

    parser.add_argument(
        "--sam2-checkpoint",
        type=Path,
        default=Path(
            "/opt/checkpoints/seedo/sam2/"
            "sam2_hiera_large.pt"
        ),
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    return run_visual_prompting_ring_test(
        video=args.video,
        pick_frame=args.pick_frame,
        place_frame=args.place_frame,
        artifacts_dir=args.artifacts_dir,
        grounding_config=args.grounding_config,
        grounding_checkpoint=args.grounding_checkpoint,
        bert_model=args.bert_model,
        sam_checkpoint=args.sam_checkpoint,
        sam2_checkpoint=args.sam2_checkpoint,
    )


if __name__ == "__main__":
    raise SystemExit(
        main()
    )