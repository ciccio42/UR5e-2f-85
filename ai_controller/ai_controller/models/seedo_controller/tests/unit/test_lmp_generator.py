from __future__ import annotations

import json

import numpy as np
import pytest

from results import (
    ActionPlanningResult,
    ActionStep,
    ReplicabilityResult,
    ResolvedActionTargets,
    SceneObject,
    SceneState,
)

from ai_controller.models.seedo_controller import (
    lmp_generator as lmp_generator_module,
)
from ai_controller.models.seedo_controller.lmp_generator import (
    LMPGenerator,
    LMPSceneWrapper,
)
from ai_controller.models.seedo_controller.task_types import (
    TaskType,
)


@pytest.fixture(autouse=True)
def _fake_openai(monkeypatch):
    monkeypatch.setattr(
        lmp_generator_module,
        "OpenAI",
        lambda: object(),
    )


def _scene_object(
    object_id: str,
    *,
    label: str | None = None,
    position_base=(0.1, 0.2, 0.3),
    category: str | None = None,
) -> SceneObject:
    return SceneObject(
        object_id=object_id,
        label=label or object_id,
        pixel_coordinates=(10, 20),
        position_camera=(0.4, 0.5, 0.6),
        position_base=position_base,
        category=category,
    )


def _scene_state(
    *objects: SceneObject,
) -> SceneState:
    return SceneState(
        objects=tuple(objects),
    )


def _action_step(
    *,
    picked_detector_label: str = "green cube",
    destination_track_id: int = 10,
    relation: str = "in",
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
        relation=relation,
        action="demonstration action",
        picked_detector_label=picked_detector_label,
    )


def _action_plan(
    *,
    task_type=TaskType.PICK_AND_PLACE,
    status: str = "completed",
    natural_language_plan: str = "Original natural-language plan.",
    steps=None,
) -> ActionPlanningResult:
    if steps is None:
        steps = (
            _action_step(),
        )

    return ActionPlanningResult(
        steps=tuple(steps),
        status=status,
        ambiguities=(),
        natural_language_plan=natural_language_plan,
        task_type=task_type,
    )


def _resolved_target(
    *,
    step_index: int = 0,
    pick_id: str = "runtime_pick",
    place_id: str = "runtime_place",
) -> ResolvedActionTargets:
    return ResolvedActionTargets(
        action_step_index=step_index,
        runtime_pick_object_id=pick_id,
        runtime_place_object_id=place_id,
    )


def _replicability_result(
    *targets: ResolvedActionTargets,
    replicable: bool = True,
) -> ReplicabilityResult:
    return ReplicabilityResult(
        replicable=replicable,
        resolved_targets=tuple(targets),
        failure_reasons=(
            ()
            if replicable
            else ("not replicable",)
        ),
    )


def _install_fake_lmp(
    monkeypatch,
    generator,
    captured,
    *,
    record_primitive: bool = True,
    source_code: str = "  generated_code()  ",
):
    class FakeLMP:
        def __init__(self, wrapper):
            self.wrapper = wrapper
            self.exec_hist = source_code

        def __call__(
            self,
            instruction,
            *,
            context,
        ):
            captured["instruction"] = instruction
            captured["context"] = context

            if record_primitive:
                self.wrapper.reach(
                    "runtime_pick"
                )
                self.wrapper.pick(
                    "runtime_pick"
                )

    def fake_setup_lmp(
        *,
        wrapper,
        scene_state,
        task_type,
    ):
        captured["wrapper"] = wrapper
        captured["scene_state"] = scene_state
        captured["task_type"] = task_type

        return FakeLMP(
            wrapper
        )

    monkeypatch.setattr(
        generator,
        "_setup_lmp",
        fake_setup_lmp,
    )


# ---------------------------------------------------------------------
# LMPSceneWrapper
# ---------------------------------------------------------------------


@pytest.mark.parametrize(
    "bottom_left",
    [
        (0.0,),
        (0.0, 1.0, 2.0),
    ],
)
def test_wrapper_rejects_invalid_bottom_left_shape(
    bottom_left,
):
    with pytest.raises(
        ValueError,
        match="workspace_bottom_left",
    ):
        LMPSceneWrapper(
            scene_state=_scene_state(),
            workspace_bottom_left=bottom_left,
            workspace_top_right=(1.0, 1.0),
        )


@pytest.mark.parametrize(
    "top_right",
    [
        (1.0,),
        (1.0, 2.0, 3.0),
    ],
)
def test_wrapper_rejects_invalid_top_right_shape(
    top_right,
):
    with pytest.raises(
        ValueError,
        match="workspace_top_right",
    ):
        LMPSceneWrapper(
            scene_state=_scene_state(),
            workspace_bottom_left=(0.0, 0.0),
            workspace_top_right=top_right,
        )


@pytest.mark.parametrize(
    ("bottom_left", "top_right"),
    [
        ((0.0, 0.0), (0.0, 1.0)),
        ((0.0, 0.0), (1.0, 0.0)),
        ((1.0, 1.0), (0.0, 2.0)),
    ],
)
def test_wrapper_rejects_invalid_workspace_bounds(
    bottom_left,
    top_right,
):
    with pytest.raises(
        ValueError,
        match="workspace_top_right must be greater",
    ):
        LMPSceneWrapper(
            scene_state=_scene_state(),
            workspace_bottom_left=bottom_left,
            workspace_top_right=top_right,
        )


def test_wrapper_exposes_scene_objects():
    wrapper = LMPSceneWrapper(
        scene_state=_scene_state(
            _scene_object(
                "obj_a",
                position_base=(0.1, 0.2, 0.3),
            ),
            _scene_object(
                "obj_b",
                position_base=(0.4, 0.5, 0.6),
            ),
        ),
        workspace_bottom_left=(0.0, 0.0),
        workspace_top_right=(1.0, 1.0),
    )

    assert wrapper.get_obj_names() == [
        "obj_a",
        "obj_b",
    ]

    assert wrapper.is_obj_visible(
        "obj_a"
    ) is True

    assert wrapper.is_obj_visible(
        "missing"
    ) is False

    assert np.allclose(
        wrapper.get_obj_pos(
            "obj_a"
        ),
        np.array(
            [0.1, 0.2]
        ),
    )

    assert np.allclose(
        wrapper.get_obj_pos_3d(
            "obj_b"
        ),
        np.array(
            [0.4, 0.5, 0.6]
        ),
    )

    assert (
        wrapper.get_scene_object(
            "obj_a"
        ).object_id
        == "obj_a"
    )


@pytest.mark.parametrize(
    "method_name",
    [
        "get_obj_pos",
        "get_obj_pos_3d",
        "get_scene_object",
    ],
)
def test_wrapper_rejects_unknown_object(
    method_name,
):
    wrapper = LMPSceneWrapper(
        scene_state=_scene_state(),
        workspace_bottom_left=(0.0, 0.0),
        workspace_top_right=(1.0, 1.0),
    )

    method = getattr(
        wrapper,
        method_name,
    )

    with pytest.raises(
        KeyError,
        match="Unknown object",
    ):
        method(
            "missing"
        )


def test_wrapper_denormalizes_workspace_coordinates():
    wrapper = LMPSceneWrapper(
        scene_state=_scene_state(),
        workspace_bottom_left=(1.0, 2.0),
        workspace_top_right=(5.0, 10.0),
    )

    result = wrapper.denormalize_xy(
        (0.5, 0.25)
    )

    assert np.allclose(
        result,
        np.array(
            [3.0, 4.0]
        ),
    )


def test_wrapper_rejects_invalid_normalized_position():
    wrapper = LMPSceneWrapper(
        scene_state=_scene_state(),
        workspace_bottom_left=(0.0, 0.0),
        workspace_top_right=(1.0, 1.0),
    )

    with pytest.raises(
        ValueError,
        match="exactly two coordinates",
    ):
        wrapper.denormalize_xy(
            (0.5,)
        )


def test_wrapper_returns_expected_corner_positions():
    wrapper = LMPSceneWrapper(
        scene_state=_scene_state(),
        workspace_bottom_left=(0.0, 0.0),
        workspace_top_right=(10.0, 20.0),
    )

    assert np.allclose(
        wrapper.get_corner_positions(),
        np.array(
            [
                [0.0, 20.0],
                [10.0, 20.0],
                [0.0, 0.0],
                [10.0, 0.0],
            ]
        ),
    )


def test_wrapper_returns_expected_side_positions():
    wrapper = LMPSceneWrapper(
        scene_state=_scene_state(),
        workspace_bottom_left=(0.0, 0.0),
        workspace_top_right=(10.0, 20.0),
    )

    assert np.allclose(
        wrapper.get_side_positions(),
        np.array(
            [
                [5.0, 20.0],
                [10.0, 10.0],
                [5.0, 0.0],
                [0.0, 10.0],
            ]
        ),
    )


@pytest.mark.parametrize(
    ("position", "expected"),
    [
        ((0.0, 10.0), "top left corner"),
        ((10.0, 10.0), "top right corner"),
        ((0.0, 0.0), "bottom left corner"),
        ((10.0, 0.0), "bottom right corner"),
    ],
)
def test_wrapper_get_corner_name(
    position,
    expected,
):
    wrapper = LMPSceneWrapper(
        scene_state=_scene_state(),
        workspace_bottom_left=(0.0, 0.0),
        workspace_top_right=(10.0, 10.0),
    )

    assert wrapper.get_corner_name(
        position
    ) == expected


@pytest.mark.parametrize(
    ("position", "expected"),
    [
        ((5.0, 10.0), "top side"),
        ((10.0, 5.0), "right side"),
        ((5.0, 0.0), "bottom side"),
        ((0.0, 5.0), "left side"),
    ],
)
def test_wrapper_get_side_name(
    position,
    expected,
):
    wrapper = LMPSceneWrapper(
        scene_state=_scene_state(),
        workspace_bottom_left=(0.0, 0.0),
        workspace_top_right=(10.0, 10.0),
    )

    assert wrapper.get_side_name(
        position
    ) == expected


def test_wrapper_get_obj_positions_np():
    wrapper = LMPSceneWrapper(
        scene_state=_scene_state(
            _scene_object(
                "a",
                position_base=(0.1, 0.2, 0.3),
            ),
            _scene_object(
                "b",
                position_base=(0.4, 0.5, 0.6),
            ),
        ),
        workspace_bottom_left=(0.0, 0.0),
        workspace_top_right=(1.0, 1.0),
    )

    result = wrapper.get_obj_positions_np(
        [
            "a",
            "b",
        ]
    )

    assert np.allclose(
        result,
        np.array(
            [
                [0.1, 0.2],
                [0.4, 0.5],
            ]
        ),
    )


def test_wrapper_get_obj_positions_np_returns_empty_array():
    wrapper = LMPSceneWrapper(
        scene_state=_scene_state(),
        workspace_bottom_left=(0.0, 0.0),
        workspace_top_right=(1.0, 1.0),
    )

    result = wrapper.get_obj_positions_np(
        []
    )

    assert result.shape == (
        0,
        2,
    )


def test_wrapper_get_obj_positions_np_rejects_invalid_type():
    wrapper = LMPSceneWrapper(
        scene_state=_scene_state(),
        workspace_bottom_left=(0.0, 0.0),
        workspace_top_right=(1.0, 1.0),
    )

    with pytest.raises(
        TypeError,
        match="list or tuple",
    ):
        wrapper.get_obj_positions_np(
            "a"
        )


def test_wrapper_records_robot_primitives():
    wrapper = LMPSceneWrapper(
        scene_state=_scene_state(),
        workspace_bottom_left=(0.0, 0.0),
        workspace_top_right=(1.0, 1.0),
    )

    wrapper.reach("a")
    wrapper.approaching("a")
    wrapper.pick("a")
    wrapper.lift_up("a")
    wrapper.moving("b")
    wrapper.placing("b")
    wrapper.aligning("b")
    wrapper.inserting("b")

    assert [
        step.name
        for step in wrapper.primitive_steps
    ] == [
        "reach",
        "approaching",
        "pick",
        "lift_up",
        "moving",
        "placing",
        "aligning",
        "inserting",
    ]

    assert [
        step.arguments
        for step in wrapper.primitive_steps
    ] == [
        {"target": "a"},
        {"target": "a"},
        {"target": "a"},
        {"target": "a"},
        {"target": "b"},
        {"target": "b"},
        {"target": "b"},
        {"target": "b"},
    ]


# ---------------------------------------------------------------------
# LMPGenerator
# ---------------------------------------------------------------------


@pytest.mark.parametrize(
    ("mode", "expected"),
    [
        ("generalized", "generalized"),
        ("GENERALIZED", "generalized"),
        ("  generalized  ", "generalized"),
        ("prior_guided", "prior_guided"),
        ("PRIOR_GUIDED", "prior_guided"),
    ],
)
def test_generator_normalizes_perception_mode(
    mode,
    expected,
):
    generator = LMPGenerator(
        perception_mode=mode,
    )

    assert (
        generator.perception_mode
        == expected
    )


@pytest.mark.parametrize(
    "mode",
    [
        "",
        "unknown",
        "prior-guided",
    ],
)
def test_generator_rejects_invalid_perception_mode(
    mode,
):
    with pytest.raises(
        ValueError,
        match="Invalid perception_mode",
    ):
        LMPGenerator(
            perception_mode=mode,
        )


def test_run_rejects_incomplete_action_plan(
    monkeypatch,
):
    generator = LMPGenerator()

    with pytest.raises(
        ValueError,
        match="completed SeeDo action plan",
    ):
        generator.run(
            action_plan=_action_plan(
                status="failed",
            ),
            scene_state=_scene_state(),
            workspace_bottom_left=(0.0, 0.0),
            workspace_top_right=(1.0, 1.0),
        )


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
    generator = LMPGenerator()

    with pytest.raises(
        ValueError,
        match="unsupported task type",
    ):
        generator.run(
            action_plan=_action_plan(
                task_type=task_type,
            ),
            scene_state=_scene_state(),
            workspace_bottom_left=(0.0, 0.0),
            workspace_top_right=(1.0, 1.0),
        )


@pytest.mark.parametrize(
    "natural_language_plan",
    [
        "",
        "   ",
    ],
)
def test_run_rejects_empty_natural_language_plan(
    natural_language_plan,
):
    generator = LMPGenerator()

    with pytest.raises(
        ValueError,
        match="natural-language plan is empty",
    ):
        generator.run(
            action_plan=_action_plan(
                natural_language_plan=(
                    natural_language_plan
                ),
            ),
            scene_state=_scene_state(),
            workspace_bottom_left=(0.0, 0.0),
            workspace_top_right=(1.0, 1.0),
        )


def test_generalized_requires_replicability_result(
    monkeypatch,
):
    generator = LMPGenerator(
        perception_mode="generalized",
    )

    captured = {}

    _install_fake_lmp(
        monkeypatch,
        generator,
        captured,
    )

    with pytest.raises(
        ValueError,
        match="requires a ReplicabilityResult",
    ):
        generator.run(
            action_plan=_action_plan(),
            scene_state=_scene_state(),
            workspace_bottom_left=(0.0, 0.0),
            workspace_top_right=(1.0, 1.0),
            replicability_result=None,
        )


def test_generalized_rejects_non_replicable_task(
    monkeypatch,
):
    generator = LMPGenerator(
        perception_mode="generalized",
    )

    captured = {}

    _install_fake_lmp(
        monkeypatch,
        generator,
        captured,
    )

    with pytest.raises(
        ValueError,
        match="non-replicable task",
    ):
        generator.run(
            action_plan=_action_plan(),
            scene_state=_scene_state(),
            workspace_bottom_left=(0.0, 0.0),
            workspace_top_right=(1.0, 1.0),
            replicability_result=(
                _replicability_result(
                    replicable=False,
                )
            ),
        )


def test_generalized_rejects_wrong_number_of_resolved_targets(
    monkeypatch,
):
    generator = LMPGenerator(
        perception_mode="generalized",
    )

    captured = {}

    _install_fake_lmp(
        monkeypatch,
        generator,
        captured,
    )

    with pytest.raises(
        ValueError,
        match="exactly one resolved target pair",
    ):
        generator.run(
            action_plan=_action_plan(),
            scene_state=_scene_state(),
            workspace_bottom_left=(0.0, 0.0),
            workspace_top_right=(1.0, 1.0),
            replicability_result=(
                _replicability_result()
            ),
        )


def test_generalized_rejects_missing_step_index(
    monkeypatch,
):
    generator = LMPGenerator(
        perception_mode="generalized",
    )

    captured = {}

    _install_fake_lmp(
        monkeypatch,
        generator,
        captured,
    )

    action_plan = _action_plan(
        steps=(
            _action_step(),
            _action_step(
                destination_track_id=20,
            ),
        )
    )

    replicability_result = (
        _replicability_result(
            _resolved_target(
                step_index=0,
            ),
            _resolved_target(
                step_index=2,
                pick_id="runtime_pick_2",
                place_id="runtime_place_2",
            ),
        )
    )

    with pytest.raises(
        ValueError,
        match="Missing resolved runtime targets for action step 1",
    ):
        generator.run(
            action_plan=action_plan,
            scene_state=_scene_state(),
            workspace_bottom_left=(0.0, 0.0),
            workspace_top_right=(1.0, 1.0),
            replicability_result=(
                replicability_result
            ),
        )


def test_generalized_builds_pick_and_place_instruction(
    monkeypatch,
):
    generator = LMPGenerator(
        perception_mode="generalized",
    )

    captured = {}

    _install_fake_lmp(
        monkeypatch,
        generator,
        captured,
    )

    scene_state = _scene_state(
        _scene_object(
            "runtime_pick",
        ),
        _scene_object(
            "runtime_place",
        ),
    )

    primitive_plan = generator.run(
        action_plan=_action_plan(
            task_type=(
                TaskType.PICK_AND_PLACE
            ),
            steps=(
                _action_step(
                    relation="in",
                ),
            ),
        ),
        scene_state=scene_state,
        workspace_bottom_left=(0.0, 0.0),
        workspace_top_right=(1.0, 1.0),
        replicability_result=(
            _replicability_result(
                _resolved_target()
            )
        ),
    )

    assert captured["instruction"] == (
        "Pick 'runtime_pick' and place it "
        "in 'runtime_place'."
    )

    assert captured["context"] == (
        "objects = ['runtime_pick', "
        "'runtime_place']"
    )

    assert captured["task_type"] == (
        TaskType.PICK_AND_PLACE
    )

    assert [
        step.name
        for step in primitive_plan.steps
    ] == [
        "reach",
        "pick",
    ]

    assert primitive_plan.source_code == (
        "generated_code()"
    )


def test_generalized_builds_nut_assembly_instruction(
    monkeypatch,
):
    generator = LMPGenerator(
        perception_mode="generalized",
    )

    captured = {}

    _install_fake_lmp(
        monkeypatch,
        generator,
        captured,
    )

    generator.run(
        action_plan=_action_plan(
            task_type=(
                TaskType.NUT_ASSEMBLY
            ),
        ),
        scene_state=_scene_state(
            _scene_object(
                "runtime_pick"
            ),
            _scene_object(
                "runtime_place"
            ),
        ),
        workspace_bottom_left=(0.0, 0.0),
        workspace_top_right=(1.0, 1.0),
        replicability_result=(
            _replicability_result(
                _resolved_target()
            )
        ),
    )

    assert captured["instruction"] == (
        "Pick 'runtime_pick' and assemble it "
        "onto 'runtime_place'."
    )

    assert captured["task_type"] == (
        TaskType.NUT_ASSEMBLY
    )


def test_generalized_joins_multiple_actions(
    monkeypatch,
):
    generator = LMPGenerator(
        perception_mode="generalized",
    )

    captured = {}

    _install_fake_lmp(
        monkeypatch,
        generator,
        captured,
    )

    action_plan = _action_plan(
        steps=(
            _action_step(
                relation="in",
            ),
            _action_step(
                destination_track_id=20,
                relation="on",
            ),
        )
    )

    replicability_result = (
        _replicability_result(
            _resolved_target(
                step_index=0,
                pick_id="pick_a",
                place_id="place_a",
            ),
            _resolved_target(
                step_index=1,
                pick_id="pick_b",
                place_id="place_b",
            ),
        )
    )

    generator.run(
        action_plan=action_plan,
        scene_state=_scene_state(
            _scene_object(
                "pick_a"
            ),
            _scene_object(
                "place_a"
            ),
            _scene_object(
                "pick_b"
            ),
            _scene_object(
                "place_b"
            ),
        ),
        workspace_bottom_left=(0.0, 0.0),
        workspace_top_right=(1.0, 1.0),
        replicability_result=(
            replicability_result
        ),
    )

    assert captured["instruction"] == (
        "Pick 'pick_a' and place it in 'place_a'."
        " and then "
        "Pick 'pick_b' and place it on 'place_b'."
    )


def test_prior_guided_uses_original_natural_language_plan(
    monkeypatch,
):
    generator = LMPGenerator(
        perception_mode="prior_guided",
    )

    captured = {}

    _install_fake_lmp(
        monkeypatch,
        generator,
        captured,
    )

    generator.run(
        action_plan=_action_plan(
            natural_language_plan=(
                "Pick the red cube and place "
                "it in the first storage bin."
            ),
        ),
        scene_state=_scene_state(
            _scene_object(
                "red cube"
            ),
            _scene_object(
                "first storage bin"
            ),
        ),
        workspace_bottom_left=(0.0, 0.0),
        workspace_top_right=(1.0, 1.0),
        replicability_result=None,
    )

    assert captured["instruction"] == (
        "Pick the red cube and place "
        "it in the first storage bin."
    )


def test_run_rejects_empty_primitive_plan(
    monkeypatch,
):
    generator = LMPGenerator(
        perception_mode="prior_guided",
    )

    captured = {}

    _install_fake_lmp(
        monkeypatch,
        generator,
        captured,
        record_primitive=False,
    )

    with pytest.raises(
        RuntimeError,
        match="did not generate any robot primitive",
    ):
        generator.run(
            action_plan=_action_plan(),
            scene_state=_scene_state(),
            workspace_bottom_left=(0.0, 0.0),
            workspace_top_right=(1.0, 1.0),
        )


def test_run_writes_artifacts(
    tmp_path,
    monkeypatch,
):
    generator = LMPGenerator(
        perception_mode="generalized",
    )

    captured = {}

    _install_fake_lmp(
        monkeypatch,
        generator,
        captured,
        source_code=(
            "\n"
            "reach('runtime_pick')\n"
            "pick('runtime_pick')\n"
        ),
    )

    artifacts_dir = (
        tmp_path
        / "nested"
        / "lmp"
    )

    primitive_plan = generator.run(
        action_plan=_action_plan(),
        scene_state=_scene_state(
            _scene_object(
                "runtime_pick"
            ),
            _scene_object(
                "runtime_place"
            ),
        ),
        workspace_bottom_left=(0.0, 0.0),
        workspace_top_right=(1.0, 1.0),
        replicability_result=(
            _replicability_result(
                _resolved_target()
            )
        ),
        artifacts_dir=artifacts_dir,
    )

    generated_program_path = (
        artifacts_dir
        / "generated_program.py"
    )

    primitive_plan_path = (
        artifacts_dir
        / "primitive_plan.json"
    )

    assert generated_program_path.is_file()
    assert primitive_plan_path.is_file()

    assert generated_program_path.read_text(
        encoding="utf-8"
    ) == (
        "reach('runtime_pick')\n"
        "pick('runtime_pick')"
    )

    with primitive_plan_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        data = json.load(
            stream
        )

    assert data == {
        "steps": [
            {
                "name": "reach",
                "arguments": {
                    "target": "runtime_pick",
                },
                "source_code": None,
            },
            {
                "name": "pick",
                "arguments": {
                    "target": "runtime_pick",
                },
                "source_code": None,
            },
        ],
        "source_code": (
            "reach('runtime_pick')\n"
            "pick('runtime_pick')"
        ),
    }

    assert primitive_plan.source_code == (
        "reach('runtime_pick')\n"
        "pick('runtime_pick')"
    )