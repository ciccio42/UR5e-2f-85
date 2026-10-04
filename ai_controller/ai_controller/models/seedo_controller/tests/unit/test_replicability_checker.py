from __future__ import annotations

import json

import pytest

from results import (
    ActionPlanningResult,
    ActionStep,
    SceneObject,
    SceneState,
    StructuralMapping,
    StructuralMatchingResult,
    StructuralObjectMatch,
)

from ai_controller.models.seedo_controller.replicability_checker import (
    ReplicabilityChecker,
)
from ai_controller.models.seedo_controller.task_types import (
    TaskType,
)


def _action_step(
    *,
    picked_detector_label: str = "green cube",
    destination_track_id: int = 10,
) -> ActionStep:
    return ActionStep(
        pick_keyframe=0,
        place_keyframe=1,
        picked_track_id=1,
        picked_category="cube",
        picked_color="green",
        destination_track_id=destination_track_id,
        destination_category="bin",
        destination_ordinal_from_left=None,
        relation="in",
        action="test action",
        picked_detector_label=picked_detector_label,
    )


def _action_plan(
    *,
    task_type=TaskType.PICK_AND_PLACE,
    steps=None,
) -> ActionPlanningResult:
    if steps is None:
        steps = (
            _action_step(),
        )

    return ActionPlanningResult(
        steps=tuple(steps),
        status="ok",
        ambiguities=(),
        natural_language_plan="test plan",
        task_type=task_type,
    )


def _scene_object(
    *,
    object_id: str,
    label: str,
    category: str | None,
) -> SceneObject:
    return SceneObject(
        object_id=object_id,
        label=label,
        pixel_coordinates=(0, 0),
        position_camera=(0.0, 0.0, 0.0),
        position_base=(0.0, 0.0, 0.0),
        category=category,
    )


def _scene_state(
    *objects: SceneObject,
) -> SceneState:
    return SceneState(
        objects=tuple(objects),
    )


def _mapping(
    *pairs: tuple[str, str],
) -> StructuralMapping:
    return StructuralMapping(
        matches=tuple(
            StructuralObjectMatch(
                demo_object_id=demo_id,
                runtime_object_id=runtime_id,
            )
            for demo_id, runtime_id in pairs
        )
    )


def _matching_result(
    *mappings: StructuralMapping,
) -> StructuralMatchingResult:
    return StructuralMatchingResult(
        valid_mappings=tuple(mappings),
    )


def test_run_resolves_replicable_pick_and_place():
    checker = ReplicabilityChecker()

    action_plan = _action_plan()

    matching_result = _matching_result(
        _mapping(
            ("10", "runtime_bin"),
        )
    )

    scene_state = _scene_state(
        _scene_object(
            object_id="runtime_cube",
            label="green cube",
            category="cube",
        ),
        _scene_object(
            object_id="runtime_bin",
            label="storage bin",
            category="bin",
        ),
    )

    result = checker.run(
        action_plan=action_plan,
        structural_matching_result=matching_result,
        scene_state=scene_state,
    )

    assert result.replicable is True
    assert result.failure_reasons == ()

    assert len(result.resolved_targets) == 1

    target = result.resolved_targets[0]

    assert target.action_step_index == 0
    assert target.runtime_pick_object_id == "runtime_cube"
    assert target.runtime_place_object_id == "runtime_bin"


@pytest.mark.parametrize(
    "task_type",
    [
        TaskType.PICK_AND_PLACE,
        "pick_and_place",
    ],
)
def test_run_accepts_enum_and_string_task_type(
    task_type,
):
    checker = ReplicabilityChecker()

    result = checker.run(
        action_plan=_action_plan(
            task_type=task_type,
        ),
        structural_matching_result=_matching_result(
            _mapping(
                ("10", "runtime_bin"),
            )
        ),
        scene_state=_scene_state(
            _scene_object(
                object_id="runtime_cube",
                label="green cube",
                category="cube",
            ),
            _scene_object(
                object_id="runtime_bin",
                label="bin",
                category="bin",
            ),
        ),
    )

    assert result.replicable is True


@pytest.mark.parametrize(
    "task_type",
    [
        None,
        "unknown",
        "unsupported",
    ],
)
def test_run_rejects_unsupported_task_type(
    task_type,
):
    checker = ReplicabilityChecker()

    result = checker.run(
        action_plan=_action_plan(
            task_type=task_type,
        ),
        structural_matching_result=_matching_result(
            _mapping(
                ("10", "runtime_bin"),
            )
        ),
        scene_state=_scene_state(),
    )

    assert result.replicable is False
    assert result.resolved_targets == ()
    assert result.failure_reasons == (
        "The action plan does not contain a supported task type.",
    )


def test_run_rejects_empty_action_plan():
    checker = ReplicabilityChecker()

    result = checker.run(
        action_plan=_action_plan(
            steps=(),
        ),
        structural_matching_result=_matching_result(
            _mapping(
                ("10", "runtime_bin"),
            )
        ),
        scene_state=_scene_state(),
    )

    assert result.replicable is False
    assert result.resolved_targets == ()
    assert result.failure_reasons == (
        "The action plan contains no action steps.",
    )


def test_run_rejects_missing_structural_mapping():
    checker = ReplicabilityChecker()

    result = checker.run(
        action_plan=_action_plan(),
        structural_matching_result=_matching_result(),
        scene_state=_scene_state(),
    )

    assert result.replicable is False
    assert result.resolved_targets == ()
    assert result.failure_reasons == (
        "No valid structural mapping exists between "
        "the demonstration and runtime scenes.",
    )


def test_run_rejects_missing_pick_label():
    checker = ReplicabilityChecker()

    action_plan = _action_plan(
        steps=(
            _action_step(
                picked_detector_label="   ",
            ),
        )
    )

    result = checker.run(
        action_plan=action_plan,
        structural_matching_result=_matching_result(
            _mapping(
                ("10", "runtime_bin"),
            )
        ),
        scene_state=_scene_state(
            _scene_object(
                object_id="runtime_bin",
                label="bin",
                category="bin",
            ),
        ),
    )

    assert result.replicable is False
    assert result.resolved_targets == ()

    assert result.failure_reasons == (
        "Action step 0: missing picked_detector_label.",
    )


def test_run_rejects_missing_pick_object():
    checker = ReplicabilityChecker()

    result = checker.run(
        action_plan=_action_plan(),
        structural_matching_result=_matching_result(
            _mapping(
                ("10", "runtime_bin"),
            )
        ),
        scene_state=_scene_state(
            _scene_object(
                object_id="runtime_bin",
                label="bin",
                category="bin",
            ),
        ),
    )

    assert result.replicable is False
    assert result.resolved_targets == ()

    assert len(result.failure_reasons) == 1
    assert "cannot uniquely resolve picked object" in result.failure_reasons[0]
    assert "matches=[]" in result.failure_reasons[0]


def test_run_rejects_ambiguous_pick_object():
    checker = ReplicabilityChecker()

    scene_state = _scene_state(
        _scene_object(
            object_id="cube_a",
            label="green cube",
            category="cube",
        ),
        _scene_object(
            object_id="cube_b",
            label=" GREEN CUBE ",
            category="cube",
        ),
        _scene_object(
            object_id="runtime_bin",
            label="bin",
            category="bin",
        ),
    )

    result = checker.run(
        action_plan=_action_plan(),
        structural_matching_result=_matching_result(
            _mapping(
                ("10", "runtime_bin"),
            )
        ),
        scene_state=scene_state,
    )

    assert result.replicable is False
    assert result.resolved_targets == ()

    assert len(result.failure_reasons) == 1
    assert "cannot uniquely resolve picked object" in result.failure_reasons[0]
    assert "cube_a" in result.failure_reasons[0]
    assert "cube_b" in result.failure_reasons[0]


def test_run_normalizes_pick_label():
    checker = ReplicabilityChecker()

    action_plan = _action_plan(
        steps=(
            _action_step(
                picked_detector_label="  GREEN CUBE  ",
            ),
        )
    )

    result = checker.run(
        action_plan=action_plan,
        structural_matching_result=_matching_result(
            _mapping(
                ("10", "runtime_bin"),
            )
        ),
        scene_state=_scene_state(
            _scene_object(
                object_id="runtime_cube",
                label="green cube",
                category="cube",
            ),
            _scene_object(
                object_id="runtime_bin",
                label="bin",
                category="bin",
            ),
        ),
    )

    assert result.replicable is True
    assert result.resolved_targets[0].runtime_pick_object_id == (
        "runtime_cube"
    )


def test_run_rejects_mapping_missing_demo_destination():
    checker = ReplicabilityChecker()

    result = checker.run(
        action_plan=_action_plan(),
        structural_matching_result=_matching_result(
            _mapping(
                ("99", "runtime_bin"),
            )
        ),
        scene_state=_scene_state(
            _scene_object(
                object_id="runtime_cube",
                label="green cube",
                category="cube",
            ),
            _scene_object(
                object_id="runtime_bin",
                label="bin",
                category="bin",
            ),
        ),
    )

    assert result.replicable is False
    assert result.resolved_targets == ()

    assert result.failure_reasons == (
        "Action step 0: demonstration destination '10' "
        "is not present in a structural mapping.",
    )


def test_run_accepts_multiple_mappings_with_same_destination():
    checker = ReplicabilityChecker()

    matching_result = _matching_result(
        _mapping(
            ("10", "runtime_bin"),
            ("11", "runtime_other_a"),
        ),
        _mapping(
            ("10", "runtime_bin"),
            ("11", "runtime_other_b"),
        ),
    )

    result = checker.run(
        action_plan=_action_plan(),
        structural_matching_result=matching_result,
        scene_state=_scene_state(
            _scene_object(
                object_id="runtime_cube",
                label="green cube",
                category="cube",
            ),
            _scene_object(
                object_id="runtime_bin",
                label="bin",
                category="bin",
            ),
        ),
    )

    assert result.replicable is True
    assert result.failure_reasons == ()

    assert result.resolved_targets[0].runtime_place_object_id == (
        "runtime_bin"
    )


def test_run_rejects_multiple_mappings_with_different_destinations():
    checker = ReplicabilityChecker()

    matching_result = _matching_result(
        _mapping(
            ("10", "runtime_bin_a"),
        ),
        _mapping(
            ("10", "runtime_bin_b"),
        ),
    )

    result = checker.run(
        action_plan=_action_plan(),
        structural_matching_result=matching_result,
        scene_state=_scene_state(
            _scene_object(
                object_id="runtime_cube",
                label="green cube",
                category="cube",
            ),
            _scene_object(
                object_id="runtime_bin_a",
                label="bin a",
                category="bin",
            ),
            _scene_object(
                object_id="runtime_bin_b",
                label="bin b",
                category="bin",
            ),
        ),
    )

    assert result.replicable is False
    assert result.resolved_targets == ()

    assert len(result.failure_reasons) == 1
    assert "does not uniquely determine" in result.failure_reasons[0]
    assert "runtime_bin_a" in result.failure_reasons[0]
    assert "runtime_bin_b" in result.failure_reasons[0]


def test_run_rejects_destination_missing_from_scene_state():
    checker = ReplicabilityChecker()

    result = checker.run(
        action_plan=_action_plan(),
        structural_matching_result=_matching_result(
            _mapping(
                ("10", "missing_bin"),
            )
        ),
        scene_state=_scene_state(
            _scene_object(
                object_id="runtime_cube",
                label="green cube",
                category="cube",
            ),
        ),
    )

    assert result.replicable is False
    assert result.resolved_targets == ()

    assert result.failure_reasons == (
        "Action step 0: mapped runtime destination "
        "'missing_bin' does not exist in SceneState.",
    )


def test_run_rejects_wrong_place_category_for_pick_and_place():
    checker = ReplicabilityChecker()

    result = checker.run(
        action_plan=_action_plan(
            task_type=TaskType.PICK_AND_PLACE,
        ),
        structural_matching_result=_matching_result(
            _mapping(
                ("10", "runtime_peg"),
            )
        ),
        scene_state=_scene_state(
            _scene_object(
                object_id="runtime_cube",
                label="green cube",
                category="cube",
            ),
            _scene_object(
                object_id="runtime_peg",
                label="peg",
                category="peg",
            ),
        ),
    )

    assert result.replicable is False
    assert result.resolved_targets == ()

    assert len(result.failure_reasons) == 1
    assert "category 'peg'" in result.failure_reasons[0]
    assert "'pick_and_place'" in result.failure_reasons[0]


def test_run_resolves_nut_assembly_to_peg():
    checker = ReplicabilityChecker()

    result = checker.run(
        action_plan=_action_plan(
            task_type=TaskType.NUT_ASSEMBLY,
        ),
        structural_matching_result=_matching_result(
            _mapping(
                ("10", "runtime_peg"),
            )
        ),
        scene_state=_scene_state(
            _scene_object(
                object_id="runtime_nut",
                label="green cube",
                category="nut",
            ),
            _scene_object(
                object_id="runtime_peg",
                label="peg",
                category="peg",
            ),
        ),
    )

    assert result.replicable is True
    assert result.failure_reasons == ()

    assert result.resolved_targets[0].runtime_pick_object_id == (
        "runtime_nut"
    )
    assert result.resolved_targets[0].runtime_place_object_id == (
        "runtime_peg"
    )


def test_run_rejects_pick_and_place_resolving_to_same_object():
    checker = ReplicabilityChecker()

    result = checker.run(
        action_plan=_action_plan(),
        structural_matching_result=_matching_result(
            _mapping(
                ("10", "same_object"),
            )
        ),
        scene_state=_scene_state(
            _scene_object(
                object_id="same_object",
                label="green cube",
                category="bin",
            ),
        ),
    )

    assert result.replicable is False
    assert result.resolved_targets == ()

    assert result.failure_reasons == (
        "Action step 0: picked object and place destination "
        "resolve to the same runtime object 'same_object'.",
    )


def test_run_resolves_multiple_action_steps():
    checker = ReplicabilityChecker()

    action_plan = _action_plan(
        steps=(
            _action_step(
                picked_detector_label="green cube",
                destination_track_id=10,
            ),
            _action_step(
                picked_detector_label="red cube",
                destination_track_id=20,
            ),
        )
    )

    matching_result = _matching_result(
        _mapping(
            ("10", "runtime_bin_a"),
            ("20", "runtime_bin_b"),
        )
    )

    scene_state = _scene_state(
        _scene_object(
            object_id="runtime_green",
            label="green cube",
            category="cube",
        ),
        _scene_object(
            object_id="runtime_red",
            label="red cube",
            category="cube",
        ),
        _scene_object(
            object_id="runtime_bin_a",
            label="bin a",
            category="bin",
        ),
        _scene_object(
            object_id="runtime_bin_b",
            label="bin b",
            category="bin",
        ),
    )

    result = checker.run(
        action_plan=action_plan,
        structural_matching_result=matching_result,
        scene_state=scene_state,
    )

    assert result.replicable is True
    assert result.failure_reasons == ()

    assert len(result.resolved_targets) == 2

    assert result.resolved_targets[0].action_step_index == 0
    assert result.resolved_targets[0].runtime_pick_object_id == (
        "runtime_green"
    )
    assert result.resolved_targets[0].runtime_place_object_id == (
        "runtime_bin_a"
    )

    assert result.resolved_targets[1].action_step_index == 1
    assert result.resolved_targets[1].runtime_pick_object_id == (
        "runtime_red"
    )
    assert result.resolved_targets[1].runtime_place_object_id == (
        "runtime_bin_b"
    )


def test_run_discards_partial_resolutions_when_any_step_fails():
    checker = ReplicabilityChecker()

    action_plan = _action_plan(
        steps=(
            _action_step(
                picked_detector_label="green cube",
                destination_track_id=10,
            ),
            _action_step(
                picked_detector_label="missing cube",
                destination_track_id=20,
            ),
        )
    )

    result = checker.run(
        action_plan=action_plan,
        structural_matching_result=_matching_result(
            _mapping(
                ("10", "runtime_bin_a"),
                ("20", "runtime_bin_b"),
            )
        ),
        scene_state=_scene_state(
            _scene_object(
                object_id="runtime_green",
                label="green cube",
                category="cube",
            ),
            _scene_object(
                object_id="runtime_bin_a",
                label="bin a",
                category="bin",
            ),
            _scene_object(
                object_id="runtime_bin_b",
                label="bin b",
                category="bin",
            ),
        ),
    )

    assert result.replicable is False
    assert result.resolved_targets == ()

    assert len(result.failure_reasons) == 1
    assert "Action step 1" in result.failure_reasons[0]


def test_run_writes_success_artifact(
    tmp_path,
):
    checker = ReplicabilityChecker()

    result = checker.run(
        action_plan=_action_plan(),
        structural_matching_result=_matching_result(
            _mapping(
                ("10", "runtime_bin"),
            )
        ),
        scene_state=_scene_state(
            _scene_object(
                object_id="runtime_cube",
                label="green cube",
                category="cube",
            ),
            _scene_object(
                object_id="runtime_bin",
                label="bin",
                category="bin",
            ),
        ),
        artifacts_dir=tmp_path,
    )

    artifact_path = (
        tmp_path
        / "replicability_result.json"
    )

    assert artifact_path.is_file()

    with artifact_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        data = json.load(stream)

    assert data == {
        "replicable": True,
        "resolved_targets": [
            {
                "action_step_index": 0,
                "runtime_pick_object_id": "runtime_cube",
                "runtime_place_object_id": "runtime_bin",
            }
        ],
        "failure_reasons": [],
    }

    assert result.replicable is True


def test_run_writes_failure_artifact(
    tmp_path,
):
    checker = ReplicabilityChecker()

    result = checker.run(
        action_plan=_action_plan(),
        structural_matching_result=_matching_result(),
        scene_state=_scene_state(),
        artifacts_dir=tmp_path,
    )

    artifact_path = (
        tmp_path
        / "replicability_result.json"
    )

    assert artifact_path.is_file()

    with artifact_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        data = json.load(stream)

    assert data == {
        "replicable": False,
        "resolved_targets": [],
        "failure_reasons": [
            "No valid structural mapping exists between "
            "the demonstration and runtime scenes."
        ],
    }

    assert result.replicable is False