from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

from results import (
    ActionPlanningResult,
    ActionStep,
    ReplicabilityResult,
    SceneObject,
    SceneState,
    StructuralMapping,
    StructuralMatchingResult,
    StructuralObjectMatch,
)

from ai_controller.models.seedo_controller.replicability_checker import (
    ReplicabilityChecker,
)


DEFAULT_ACTION_PLAN = Path(
    "/seedo_tests/action_planning/action_plan.json"
)

DEFAULT_STRUCTURAL_MATCHING_RESULT = Path(
    "/seedo_tests/structural_matcher/structural_matching_result.json"
)

DEFAULT_SCENE_STATE = Path(
    "/seedo_tests/scene_interpreter/scene_state.json"
)

EXPECTED_PICK_OBJECT_ID = "green_block_0"
EXPECTED_PLACE_OBJECT_ID = "storage_bin_0"


def _load_json(
    path: Path,
) -> dict:
    normalized_path = (
        Path(path)
        .expanduser()
        .resolve()
    )

    if not normalized_path.is_file():
        raise FileNotFoundError(
            f"Required integration artifact does not exist: "
            f"{normalized_path}"
        )

    if normalized_path.stat().st_size == 0:
        raise ValueError(
            f"Required integration artifact is empty: "
            f"{normalized_path}"
        )

    with normalized_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        value = json.load(stream)

    if not isinstance(value, dict):
        raise ValueError(
            f"Expected a JSON object in {normalized_path}."
        )

    return value


def _load_action_plan(
    path: Path,
) -> ActionPlanningResult:
    data = _load_json(
        path
    )

    steps_data = data.get(
        "steps"
    )

    if not isinstance(
        steps_data,
        list,
    ):
        raise ValueError(
            "action_plan.json does not contain a steps list."
        )

    steps: list[ActionStep] = []

    for index, item in enumerate(
        steps_data
    ):
        if not isinstance(
            item,
            dict,
        ):
            raise ValueError(
                "Invalid action step at index "
                f"{index}: {item!r}"
            )

        steps.append(
            ActionStep(
                pick_keyframe=int(
                    item["pick_keyframe"]
                ),
                place_keyframe=int(
                    item["place_keyframe"]
                ),
                picked_track_id=int(
                    item["picked_track_id"]
                ),
                picked_category=str(
                    item["picked_category"]
                ),
                picked_color=str(
                    item["picked_color"]
                ),
                destination_track_id=int(
                    item["destination_track_id"]
                ),
                destination_category=str(
                    item["destination_category"]
                ),
                destination_ordinal_from_left=(
                    None
                    if item.get(
                        "destination_ordinal_from_left"
                    ) is None
                    else int(
                        item[
                            "destination_ordinal_from_left"
                        ]
                    )
                ),
                relation=str(
                    item["relation"]
                ),
                action=str(
                    item["action"]
                ),
                picked_detector_label=str(
                    item.get(
                        "picked_detector_label",
                        "",
                    )
                ),
            )
        )

    ambiguities = data.get(
        "ambiguities",
        [],
    )

    if not isinstance(
        ambiguities,
        list,
    ):
        raise ValueError(
            "action_plan.json ambiguities must be a list."
        )

    return ActionPlanningResult(
        steps=tuple(
            steps
        ),
        status=str(
            data.get(
                "status",
                "",
            )
        ),
        ambiguities=tuple(
            str(item)
            for item
            in ambiguities
        ),
        natural_language_plan=str(
            data.get(
                "natural_language_plan",
                "",
            )
        ),
        task_type=data.get(
            "task_type"
        ),
    )


def _load_structural_matching_result(
    path: Path,
) -> StructuralMatchingResult:
    data = _load_json(
        path
    )

    mappings_data = data.get(
        "valid_mappings"
    )

    if not isinstance(
        mappings_data,
        list,
    ):
        raise ValueError(
            "structural_matching_result.json does not contain "
            "a valid_mappings list."
        )

    mappings: list[
        StructuralMapping
    ] = []

    for mapping_index, mapping_data in enumerate(
        mappings_data
    ):
        if not isinstance(
            mapping_data,
            dict,
        ):
            raise ValueError(
                "Invalid structural mapping at index "
                f"{mapping_index}: {mapping_data!r}"
            )

        matches_data = mapping_data.get(
            "matches"
        )

        if not isinstance(
            matches_data,
            list,
        ):
            raise ValueError(
                "Structural mapping does not contain a matches list."
            )

        matches: list[
            StructuralObjectMatch
        ] = []

        for match_index, match_data in enumerate(
            matches_data
        ):
            if not isinstance(
                match_data,
                dict,
            ):
                raise ValueError(
                    "Invalid structural match at index "
                    f"{match_index}: {match_data!r}"
                )

            matches.append(
                StructuralObjectMatch(
                    demo_object_id=str(
                        match_data[
                            "demo_object_id"
                        ]
                    ),
                    runtime_object_id=str(
                        match_data[
                            "runtime_object_id"
                        ]
                    ),
                )
            )

        mappings.append(
            StructuralMapping(
                matches=tuple(
                    matches
                )
            )
        )

    return StructuralMatchingResult(
        valid_mappings=tuple(
            mappings
        )
    )


def _load_scene_state(
    path: Path,
) -> SceneState:
    data = _load_json(
        path
    )

    objects_data = data.get(
        "objects"
    )

    if (
        not isinstance(
            objects_data,
            list,
        )
        or not objects_data
    ):
        raise ValueError(
            "scene_state.json must contain a non-empty objects list."
        )

    objects: list[
        SceneObject
    ] = []

    for index, item in enumerate(
        objects_data
    ):
        if not isinstance(
            item,
            dict,
        ):
            raise ValueError(
                "Invalid scene object at index "
                f"{index}: {item!r}"
            )

        pixel = item.get(
            "pixel_coordinates"
        )

        position_camera = item.get(
            "position_camera"
        )

        position_base = item.get(
            "position_base"
        )

        if (
            not isinstance(
                pixel,
                list,
            )
            or len(pixel) != 2
        ):
            raise ValueError(
                f"Invalid pixel_coordinates at index {index}: "
                f"{pixel!r}"
            )

        if (
            not isinstance(
                position_camera,
                list,
            )
            or len(position_camera) != 3
        ):
            raise ValueError(
                f"Invalid position_camera at index {index}: "
                f"{position_camera!r}"
            )

        if (
            not isinstance(
                position_base,
                list,
            )
            or len(position_base) != 3
        ):
            raise ValueError(
                f"Invalid position_base at index {index}: "
                f"{position_base!r}"
            )

        attributes = item.get(
            "attributes",
            {},
        )

        if not isinstance(
            attributes,
            dict,
        ):
            raise ValueError(
                "SceneObject attributes must be a dictionary."
            )

        objects.append(
            SceneObject(
                object_id=str(
                    item["object_id"]
                ),
                label=str(
                    item["label"]
                ),
                pixel_coordinates=(
                    int(
                        pixel[0]
                    ),
                    int(
                        pixel[1]
                    ),
                ),
                position_camera=tuple(
                    float(value)
                    for value
                    in position_camera
                ),
                position_base=tuple(
                    float(value)
                    for value
                    in position_base
                ),
                category=(
                    None
                    if item.get(
                        "category"
                    ) is None
                    else str(
                        item[
                            "category"
                        ]
                    )
                ),
                attributes=dict(
                    attributes
                ),
            )
        )

    return SceneState(
        objects=tuple(
            objects
        )
    )


def run_replicability_checker_test(
    args: argparse.Namespace,
) -> int:
    """Resolve runtime pick/place targets from persisted integration artifacts."""

    action_plan_path = (
        DEFAULT_ACTION_PLAN
    )

    structural_matching_path = (
        DEFAULT_STRUCTURAL_MATCHING_RESULT
    )

    scene_state_path = (
        DEFAULT_SCENE_STATE
    )

    action_plan = _load_action_plan(
        action_plan_path
    )

    structural_matching_result = (
        _load_structural_matching_result(
            structural_matching_path
        )
    )

    scene_state = _load_scene_state(
        scene_state_path
    )

    if args.artifacts_dir is None:
        artifacts_dir = Path(
            "/seedo_tests/replicability_checker"
        )
    else:
        artifacts_dir = (
            Path(args.artifacts_dir)
            .expanduser()
            .resolve()
        )

    # ---------------------------------------------------------
    # Validate the three upstream handoffs.
    # ---------------------------------------------------------

    if action_plan.status != "completed":
        raise AssertionError(
            "Replicability integration requires a completed "
            f"action plan, received {action_plan.status!r}."
        )

    if len(
        action_plan.steps
    ) != 1:
        raise AssertionError(
            "Expected exactly one demonstrated action step, "
            f"received {len(action_plan.steps)}."
        )

    action_step = (
        action_plan.steps[0]
    )

    if (
        str(
            action_plan.task_type
        ).strip().lower()
        != "pick_and_place"
    ):
        raise AssertionError(
            "Unexpected task type in ActionPlanner handoff: "
            f"{action_plan.task_type!r}"
        )

    if (
        action_step.picked_detector_label
        != "green block"
    ):
        raise AssertionError(
            "Unexpected demonstrated picked detector label: "
            f"{action_step.picked_detector_label!r}"
        )

    if (
        action_step.destination_track_id
        != 0
    ):
        raise AssertionError(
            "Unexpected demonstrated destination track ID: "
            f"{action_step.destination_track_id}"
        )

    if len(
        structural_matching_result.valid_mappings
    ) != 1:
        raise AssertionError(
            "Expected exactly one structural mapping, "
            f"received "
            f"{len(structural_matching_result.valid_mappings)}."
        )

    structural_mapping = (
        structural_matching_result
        .valid_mappings[0]
    )

    mapped_destinations = {
        match.demo_object_id:
            match.runtime_object_id
        for match
        in structural_mapping.matches
    }

    if (
        mapped_destinations.get(
            "0"
        )
        != EXPECTED_PLACE_OBJECT_ID
    ):
        raise AssertionError(
            "Unexpected structural destination mapping for demo "
            f"object '0': {mapped_destinations.get('0')!r}"
        )

    runtime_objects_by_id = {
        obj.object_id: obj
        for obj
        in scene_state.objects
    }

    if (
        EXPECTED_PICK_OBJECT_ID
        not in runtime_objects_by_id
    ):
        raise AssertionError(
            "Expected runtime picked object is absent from SceneState: "
            f"{EXPECTED_PICK_OBJECT_ID}"
        )

    if (
        EXPECTED_PLACE_OBJECT_ID
        not in runtime_objects_by_id
    ):
        raise AssertionError(
            "Expected runtime destination object is absent from SceneState: "
            f"{EXPECTED_PLACE_OBJECT_ID}"
        )

    pick_label_matches = [
        obj.object_id
        for obj
        in scene_state.objects
        if (
            obj.label
            .strip()
            .casefold()
            == "green block"
        )
    ]

    if pick_label_matches != [
        EXPECTED_PICK_OBJECT_ID
    ]:
        raise AssertionError(
            "The demonstrated picked detector label does not resolve "
            "uniquely in SceneState: "
            f"{pick_label_matches}"
        )

    destination_object = (
        runtime_objects_by_id[
            EXPECTED_PLACE_OBJECT_ID
        ]
    )

    if (
        str(
            destination_object.category
        ).strip().casefold()
        != "bin"
    ):
        raise AssertionError(
            "Expected runtime destination category 'bin', received "
            f"{destination_object.category!r}."
        )

    # ---------------------------------------------------------
    # Run the real ReplicabilityChecker.
    # ---------------------------------------------------------

    action_plan_before = (
        asdict(
            action_plan
        )
    )

    structural_matching_before = (
        asdict(
            structural_matching_result
        )
    )

    scene_state_before = (
        asdict(
            scene_state
        )
    )

    checker = (
        ReplicabilityChecker()
    )

    result: ReplicabilityResult = checker.run(
        action_plan=action_plan,
        structural_matching_result=(
            structural_matching_result
        ),
        scene_state=scene_state,
        artifacts_dir=artifacts_dir,
    )

    # ---------------------------------------------------------
    # Validate successful replicability resolution.
    # ---------------------------------------------------------

    if not result.replicable:
        raise AssertionError(
            "Expected the demonstrated task to be replicable, "
            f"failure_reasons={result.failure_reasons}"
        )

    if result.failure_reasons:
        raise AssertionError(
            "Replicable result contains unexpected failure reasons: "
            f"{result.failure_reasons}"
        )

    if len(
        result.resolved_targets
    ) != 1:
        raise AssertionError(
            "Expected exactly one resolved action target, "
            f"received {len(result.resolved_targets)}."
        )

    resolved = (
        result.resolved_targets[0]
    )

    if (
        resolved.action_step_index
        != 0
    ):
        raise AssertionError(
            "Unexpected resolved action-step index: "
            f"{resolved.action_step_index}"
        )

    if (
        resolved.runtime_pick_object_id
        != EXPECTED_PICK_OBJECT_ID
    ):
        raise AssertionError(
            "Unexpected runtime pick target: "
            f"expected={EXPECTED_PICK_OBJECT_ID!r}, "
            f"received={resolved.runtime_pick_object_id!r}"
        )

    if (
        resolved.runtime_place_object_id
        != EXPECTED_PLACE_OBJECT_ID
    ):
        raise AssertionError(
            "Unexpected runtime place target: "
            f"expected={EXPECTED_PLACE_OBJECT_ID!r}, "
            f"received={resolved.runtime_place_object_id!r}"
        )

    if (
        resolved.runtime_pick_object_id
        == resolved.runtime_place_object_id
    ):
        raise AssertionError(
            "Runtime pick and place targets resolved to the same object."
        )

    # ---------------------------------------------------------
    # Verify that the checker did not mutate its inputs.
    # ---------------------------------------------------------

    if (
        asdict(
            action_plan
        )
        != action_plan_before
    ):
        raise AssertionError(
            "ReplicabilityChecker mutated ActionPlanningResult."
        )

    if (
        asdict(
            structural_matching_result
        )
        != structural_matching_before
    ):
        raise AssertionError(
            "ReplicabilityChecker mutated StructuralMatchingResult."
        )

    if (
        asdict(
            scene_state
        )
        != scene_state_before
    ):
        raise AssertionError(
            "ReplicabilityChecker mutated SceneState."
        )

    # ---------------------------------------------------------
    # Validate persistent artifact.
    # ---------------------------------------------------------

    artifact_path = (
        artifacts_dir
        / "replicability_result.json"
    )

    if not artifact_path.is_file():
        raise AssertionError(
            "ReplicabilityChecker artifact was not created: "
            f"{artifact_path}"
        )

    if artifact_path.stat().st_size == 0:
        raise AssertionError(
            "ReplicabilityChecker artifact is empty: "
            f"{artifact_path}"
        )

    with artifact_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        artifact = json.load(
            stream
        )

    expected_artifact = json.loads(
        json.dumps(
            asdict(
                result
            ),
            ensure_ascii=False,
        )
    )

    if artifact != expected_artifact:
        raise AssertionError(
            "replicability_result.json does not match "
            "the returned ReplicabilityResult."
        )

    # ---------------------------------------------------------
    # Report.
    # ---------------------------------------------------------

    print(
        "Replicability check completed"
    )
    print(
        "Action plan: "
        f"{action_plan_path}"
    )
    print(
        "Structural matching result: "
        f"{structural_matching_path}"
    )
    print(
        "SceneState: "
        f"{scene_state_path}"
    )
    print(
        "Replicable: "
        f"{result.replicable}"
    )

    print(
        "\nResolved targets:"
    )

    for target in (
        result.resolved_targets
    ):
        print(
            f"  step={target.action_step_index}: "
            f"pick={target.runtime_pick_object_id} -> "
            f"place={target.runtime_place_object_id}"
        )

    print(
        "\nReplicability artifact: "
        f"{artifact_path}"
    )

    print(
        "\nTEST PASSED"
    )

    return 0
