from __future__ import annotations

import argparse
import json
from pathlib import Path

from .common import build_scene_perception_result


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


def run_scene_perceiver_test(
    args: argparse.Namespace,
) -> int:

    perception_result = (
        build_scene_perception_result(
            args
        )
    )

    raw_scene = (
        perception_result.raw_scene
    )

    if not raw_scene.objects:
        raise AssertionError(
            "ScenePerceiver returned no objects."
        )

    raw_ids = [
        obj.object_id
        for obj in raw_scene.objects
    ]

    if len(raw_ids) != len(
        set(raw_ids)
    ):
        raise AssertionError(
            "ScenePerceiver produced duplicate object IDs."
        )

    for obj in raw_scene.objects:

        if not obj.object_id.strip():
            raise AssertionError(
                "Detected object has an empty object ID."
            )

        if not obj.label.strip():
            raise AssertionError(
                f"{obj.object_id} has an empty detector label."
            )

        if len(obj.pixel_coordinates) != 2:
            raise AssertionError(
                f"{obj.object_id} has invalid pixel coordinates."
            )

        if len(obj.position_camera) != 3:
            raise AssertionError(
                f"{obj.object_id} has invalid camera coordinates."
            )

        if len(obj.position_base) != 3:
            raise AssertionError(
                f"{obj.object_id} has invalid base coordinates."
            )

        if obj.mask is None:
            raise AssertionError(
                f"{obj.object_id} has no segmentation mask."
            )

    if args.artifacts_dir is not None:

        if (
            perception_result.overlay_image_path
            is None
        ):
            raise AssertionError(
                "ScenePerceiver did not return "
                "an overlay path."
            )

        if (
            not perception_result
            .overlay_image_path
            .is_file()
        ):
            raise AssertionError(
                "Raw scene overlay was not created: "
                f"{perception_result.overlay_image_path}"
            )

        if (
            perception_result.raw_scene_json_path
            is None
        ):
            raise AssertionError(
                "ScenePerceiver did not return "
                "a raw-scene JSON path."
            )

        if (
            not perception_result
            .raw_scene_json_path
            .is_file()
        ):
            raise AssertionError(
                "Raw scene JSON was not created: "
                f"{perception_result.raw_scene_json_path}"
            )

    # ---------------------------------------------------------
    # Raw scene report
    # ---------------------------------------------------------

    print(
        "\n=== RAW SCENE STATE ==="
    )

    print(
        f"Detected objects: "
        f"{len(raw_scene.objects)}"
    )

    for obj in raw_scene.objects:

        print()

        print(
            f"Object ID:        "
            f"{obj.object_id}"
        )

        print(
            f"Label:            "
            f"{obj.label}"
        )

        print(
            f"Pixel:            "
            f"{obj.pixel_coordinates}"
        )

        print(
            f"Confidence:       "
            f"{obj.confidence}"
        )

        print(
            f"Camera position:  "
            f"{obj.position_camera}"
        )

        print(
            f"Base position:    "
            f"{obj.position_base}"
        )

    if (
        perception_result.overlay_image_path
        is not None
    ):
        print(
            "\nOverlay: "
            f"{perception_result.overlay_image_path}"
        )

    if (
        perception_result.raw_scene_json_path
        is not None
    ):
        print(
            "Raw scene JSON: "
            f"{perception_result.raw_scene_json_path}"
        )

    # ---------------------------------------------------------
    # Manual evaluation
    # ---------------------------------------------------------

    print(
        "\n=== MANUAL SCENE PERCEPTION EVALUATION ==="
    )

    print(
        "\nReview the raw scene labels and overlay "
        "before answering."
    )

    distractors_present = _ask_yes_no(
        "Are there distractors in the scene?"
    )

    # ---------------------------------------------------------
    # Object class - ground-truth cubes
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
    # Distractors
    #
    # IMPORTANT:
    # On a.infante32 the rings are NEGATIVE examples.
    # They are expected to be ignored.
    # ---------------------------------------------------------

    distractor_ignored_answers = {}
    distractor_attribute_predictions = {}

    if distractors_present:

        distractor_ignored_answers = {
            "blue_ring": _ask_yes_no(
                "Was the blue ring correctly ignored?"
            ),
            "yellow_ring": _ask_yes_no(
                "Was the yellow ring correctly ignored?"
            ),
            "gray_ring": _ask_yes_no(
                "Was the gray ring correctly ignored?"
            ),
        }

        for distractor_name, ignored in (
            distractor_ignored_answers.items()
        ):

            if ignored:
                distractor_attribute_predictions[
                    distractor_name
                ] = False
                continue

            readable_name = (
                distractor_name.replace(
                    "_",
                    " ",
                )
            )

            distractor_attribute_predictions[
                distractor_name
            ] = _ask_yes_no(
                "Was any color attribute assigned to "
                f"the {readable_name}?"
            )

    # ---------------------------------------------------------
    # Object attributes - ground-truth cubes
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
    # False positives caused by distractors
    # ---------------------------------------------------------

    distractor_class_false_positives = sum(
        not ignored
        for ignored in distractor_ignored_answers.values()
    )

    distractor_attribute_false_positives = sum(
        distractor_attribute_predictions.values()
    )

    # ---------------------------------------------------------
    # Additional false positives
    #
    # Do NOT count distractor errors here because they are
    # already included automatically above.
    # ---------------------------------------------------------

    if distractors_present:

        additional_class_false_positives = (
            _ask_int_range(
                "How many additional incorrect "
                "object-class predictions were produced, "
                "excluding the distractors?",
                0,
                100,
            )
        )

        additional_attribute_false_positives = (
            _ask_int_range(
                "How many additional incorrect color "
                "associations were produced, excluding "
                "the distractors?",
                0,
                100,
            )
        )

    else:

        additional_class_false_positives = (
            _ask_int_range(
                "How many incorrect object-class "
                "predictions were produced?",
                0,
                100,
            )
        )

        additional_attribute_false_positives = (
            _ask_int_range(
                "How many incorrect color associations "
                "were produced?",
                0,
                100,
            )
        )

    object_class_false_positives = (
        distractor_class_false_positives
        + additional_class_false_positives
    )

    object_attribute_false_positives = (
        distractor_attribute_false_positives
        + additional_attribute_false_positives
    )

    # ---------------------------------------------------------
    # Object class TP / FP / FN
    #
    # Ground truth ALWAYS remains:
    # 4 cubes + 4 storage bins = 8
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
    #
    # Ground truth ALWAYS remains:
    # four cube colors = 4
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
        "test": "scene_perceiver",
        "scene_dir": (
            str(args.scene_dir)
            if args.scene_dir is not None
            else None
        ),
        "distractors_present": (
            distractors_present
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
            "distractors": {
                "expected_behavior": "ignored",
                "ignored": (
                    distractor_ignored_answers
                ),
                "color_attribute_assigned": (
                    distractor_attribute_predictions
                ),
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
                "from_distractors": {
                    "object_class": (
                        distractor_class_false_positives
                    ),
                    "object_attribute": (
                        distractor_attribute_false_positives
                    ),
                },
                "additional": {
                    "object_class": (
                        additional_class_false_positives
                    ),
                    "object_attribute": (
                        additional_attribute_false_positives
                    ),
                },
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

    elif (
        perception_result.raw_scene_json_path
        is not None
    ):

        evaluation_dir = (
            perception_result
            .raw_scene_json_path
            .parent
        )

    else:

        evaluation_dir = Path.cwd()

    evaluation_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    evaluation_json_path = (
        evaluation_dir
        / "scene_perceiver_evaluation.json"
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
        "\n=== EVALUATION RESULTS ==="
    )

    print(
        "Scene with distractors: "
        f"{'yes' if distractors_present else 'no'}"
    )

    if distractors_present:

        print(
            "Distractor class false positives: "
            f"{distractor_class_false_positives}"
        )

        print(
            "Distractor attribute false positives: "
            f"{distractor_attribute_false_positives}"
        )

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