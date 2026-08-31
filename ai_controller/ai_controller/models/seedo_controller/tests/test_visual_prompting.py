from __future__ import annotations

import argparse
import json
from pathlib import Path

from ai_controller.models.seedo_controller.visual_prompter import (
    VisualPrompter,
)


def _ask_yes_no(question: str) -> bool:
    while True:
        answer = input(
            f"{question} [y/n]: "
        ).strip().lower()

        if answer in {"y", "yes"}:
            return True

        if answer in {"n", "no"}:
            return False

        print(
            "Please answer with 'y'/'yes' or 'n'/'no'."
        )


def _ask_int_range(
    question: str,
    min_value: int,
    max_value: int,
) -> int:
    while True:
        answer = input(
            f"{question} [{min_value}-{max_value}]: "
        ).strip()

        try:
            value = int(answer)
        except ValueError:
            print(
                f"Please enter an integer between "
                f"{min_value} and {max_value}."
            )
            continue

        if min_value <= value <= max_value:
            return value

        print(
            f"Please enter an integer between "
            f"{min_value} and {max_value}."
        )


def _compute_metrics(
    true_positives: int,
    false_positives: int,
    false_negatives: int,
) -> dict[str, float]:
    tp = true_positives
    fp = false_positives
    fn = false_negatives

    accuracy_denominator = (
        tp + fp + fn
    )

    precision_denominator = (
        tp + fp
    )

    recall_denominator = (
        tp + fn
    )

    f1_denominator = (
        (2 * tp) + fp + fn
    )

    accuracy = (
        tp / accuracy_denominator
        if accuracy_denominator > 0
        else 0.0
    )

    precision = (
        tp / precision_denominator
        if precision_denominator > 0
        else 0.0
    )

    recall = (
        tp / recall_denominator
        if recall_denominator > 0
        else 0.0
    )

    f1_score = (
        (2 * tp) / f1_denominator
        if f1_denominator > 0
        else 0.0
    )

    return {
        "accuracy_percent": round(
            accuracy * 100.0,
            2,
        ),
        "precision_percent": round(
            precision * 100.0,
            2,
        ),
        "recall_percent": round(
            recall * 100.0,
            2,
        ),
        "f1_score_percent": round(
            f1_score * 100.0,
            2,
        ),
    }


def run_visual_prompting_test(
    args: argparse.Namespace,
) -> int:

    if args.video is None:
        raise ValueError(
            "--video is required for the visual prompting test."
        )

    if args.expected_keyframes is None:
        raise ValueError(
            "--expected-keyframes is required for "
            "the visual_prompting test."
        )

    prompter = VisualPrompter(
        grounding_config=args.grounding_config,
        grounding_checkpoint=args.grounding_checkpoint,
        bert_model=args.bert_model,
        sam_checkpoint=args.sam_checkpoint,
        sam2_checkpoint=args.sam2_checkpoint,
        objects=args.objects,
    )

    expected_keyframes = tuple(
        args.expected_keyframes
    )

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

    if (
        result.annotated_video_path.stat().st_size
        == 0
    ):
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
                f"No object coordinates were returned "
                f"for {frame_key}."
            )

    tracked_object_ids = set(
        result.track_id_map.keys()
    )

    if any(
        not isinstance(track_id, int)
        for track_id in tracked_object_ids
    ):
        raise AssertionError(
            "All track IDs must be integers."
        )

    for track_id, info in (
        result.track_id_map.items()
    ):
        if "detector_label" not in info:
            raise AssertionError(
                f"Track {track_id} has no detector_label."
            )

        if "initial_center" not in info:
            raise AssertionError(
                f"Track {track_id} has no initial_center."
            )

    print("Visual prompting completed")
    print(
        f"Annotated video: "
        f"{result.annotated_video_path}"
    )

    print("\nTracked objects:")

    for track_id, info in (
        result.track_id_map.items()
    ):
        print(
            f"  Track {track_id}: {info}"
        )

    print("\nKey-frame coordinates:")

    for frame_name, coordinates in (
        result.key_frame_coordinates.items()
    ):
        print(
            f"  {frame_name}: {coordinates}"
        )

    print("\nBounding box summary:")
    print(
        result.bounding_box_summary
    )

    print("\nCount diagnostics:")
    print(
        result.count_diagnostics
    )

    # ---------------------------------------------------------
    # Manual evaluation
    # ---------------------------------------------------------

    print(
        "\n=== MANUAL VISUAL PROMPTING EVALUATION ==="
    )

    print(
        "\nReview the annotated video "
        "before answering:"
    )

    print(
        result.annotated_video_path
    )

    # ---------------------------------------------------------
    # Object class evaluation
    # ---------------------------------------------------------

    object_class_answers = {
        "red_cube": _ask_yes_no(
            "Was the red ground-truth cube correctly "
            "detected as a cube?"
        ),
        "green_cube": _ask_yes_no(
            "Was the green ground-truth cube correctly "
            "detected as a cube?"
        ),
        "yellow_cube": _ask_yes_no(
            "Was the yellow ground-truth cube correctly "
            "detected as a cube?"
        ),
        "blue_cube": _ask_yes_no(
            "Was the blue ground-truth cube correctly "
            "detected as a cube?"
        ),
    }

    storage_bins_detected = _ask_int_range(
        "How many of the 4 storage bins were "
        "detected correctly?",
        0,
        4,
    )

    # ---------------------------------------------------------
    # Object attribute evaluation
    # ---------------------------------------------------------

    object_attribute_answers = {
        "red_cube": _ask_yes_no(
            "Was the red color correctly associated "
            "with the cube?"
        ),
        "yellow_cube": _ask_yes_no(
            "Was the yellow color correctly associated "
            "with the cube?"
        ),
        "green_cube": _ask_yes_no(
            "Was the green color correctly associated "
            "with the cube?"
        ),
        "blue_cube": _ask_yes_no(
            "Was the blue color correctly associated "
            "with the cube?"
        ),
    }

    # ---------------------------------------------------------
    # False positives
    # ---------------------------------------------------------

    object_class_false_positives = (
        _ask_int_range(
            "How many incorrect object-class "
            "predictions were produced?",
            0,
            100,
        )
    )

    object_attribute_false_positives = (
        _ask_int_range(
            "How many incorrect color associations "
            "were produced?",
            0,
            100,
        )
    )

    # ---------------------------------------------------------
    # Object class TP / FP / FN
    # ---------------------------------------------------------

    object_class_true_positives = (
        sum(
            object_class_answers.values()
        )
        + storage_bins_detected
    )

    object_class_ground_truth = 8

    object_class_false_negatives = (
        object_class_ground_truth
        - object_class_true_positives
    )

    object_class_metrics = (
        _compute_metrics(
            true_positives=(
                object_class_true_positives
            ),
            false_positives=(
                object_class_false_positives
            ),
            false_negatives=(
                object_class_false_negatives
            ),
        )
    )

    # ---------------------------------------------------------
    # Object attribute TP / FP / FN
    # ---------------------------------------------------------

    object_attribute_true_positives = sum(
        object_attribute_answers.values()
    )

    object_attribute_ground_truth = 4

    object_attribute_false_negatives = (
        object_attribute_ground_truth
        - object_attribute_true_positives
    )

    object_attribute_metrics = (
        _compute_metrics(
            true_positives=(
                object_attribute_true_positives
            ),
            false_positives=(
                object_attribute_false_positives
            ),
            false_negatives=(
                object_attribute_false_negatives
            ),
        )
    )

    # ---------------------------------------------------------
    # Evaluation JSON
    # ---------------------------------------------------------

    evaluation_result = {
        "test": "visual_prompting",
        "video": str(
            args.video
        ),
        "expected_keyframes": list(
            expected_keyframes
        ),
        "answers": {
            "object_class_detection": {
                "cubes": (
                    object_class_answers
                ),
                "storage_bins": {
                    "correctly_detected": (
                        storage_bins_detected
                    ),
                    "expected": 4,
                },
            },
            "object_attribute_association": {
                "cubes": (
                    object_attribute_answers
                ),
            },
            "false_positives": {
                "object_class": (
                    object_class_false_positives
                ),
                "object_attribute": (
                    object_attribute_false_positives
                ),
            },
        },
        "metrics": {
            "object_class": {
                "true_positives": (
                    object_class_true_positives
                ),
                "false_positives": (
                    object_class_false_positives
                ),
                "false_negatives": (
                    object_class_false_negatives
                ),
                "ground_truth": (
                    object_class_ground_truth
                ),
                **object_class_metrics,
            },
            "object_attribute": {
                "true_positives": (
                    object_attribute_true_positives
                ),
                "false_positives": (
                    object_attribute_false_positives
                ),
                "false_negatives": (
                    object_attribute_false_negatives
                ),
                "ground_truth": (
                    object_attribute_ground_truth
                ),
                **object_attribute_metrics,
            },
        },
    }

    if args.artifacts_dir is not None:
        evaluation_dir = (
            Path(args.artifacts_dir)
            .expanduser()
            .resolve()
        )
    else:
        evaluation_dir = (
            result
            .annotated_video_path
            .parent
        )

    evaluation_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    evaluation_json_path = (
        evaluation_dir
        / "visual_prompting_evaluation.json"
    )

    with evaluation_json_path.open(
        "w",
        encoding="utf-8",
    ) as stream:
        json.dump(
            evaluation_result,
            stream,
            indent=4,
        )

    # ---------------------------------------------------------
    # Final report
    # ---------------------------------------------------------

    print(
        "\n=== OBJECT CLASS METRICS ==="
    )

    print(
        "TP: "
        f"{object_class_true_positives}"
    )

    print(
        "FP: "
        f"{object_class_false_positives}"
    )

    print(
        "FN: "
        f"{object_class_false_negatives}"
    )

    print(
        "Accuracy:  "
        f"{object_class_metrics['accuracy_percent']:.2f}%"
    )

    print(
        "Precision: "
        f"{object_class_metrics['precision_percent']:.2f}%"
    )

    print(
        "Recall:    "
        f"{object_class_metrics['recall_percent']:.2f}%"
    )

    print(
        "F1-score:  "
        f"{object_class_metrics['f1_score_percent']:.2f}%"
    )

    print(
        "\n=== OBJECT ATTRIBUTE METRICS ==="
    )

    print(
        "TP: "
        f"{object_attribute_true_positives}"
    )

    print(
        "FP: "
        f"{object_attribute_false_positives}"
    )

    print(
        "FN: "
        f"{object_attribute_false_negatives}"
    )

    print(
        "Accuracy:  "
        f"{object_attribute_metrics['accuracy_percent']:.2f}%"
    )

    print(
        "Precision: "
        f"{object_attribute_metrics['precision_percent']:.2f}%"
    )

    print(
        "Recall:    "
        f"{object_attribute_metrics['recall_percent']:.2f}%"
    )

    print(
        "F1-score:  "
        f"{object_attribute_metrics['f1_score_percent']:.2f}%"
    )

    print(
        "\nEvaluation JSON: "
        f"{evaluation_json_path}"
    )

    print(
        "\nTEST PASSED"
    )

    return 0