from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from results import (
    ActionPlanningResult,
    ActionStep,
    PrimitivePlan,
    PrimitiveStep,
    ReplicabilityResult,
    SceneState,
    StructuredScene,
    StructuredSceneObject,
)

from ai_controller.models.seedo_controller import (
    seedo_controller as seedo_controller_module,
)
from ai_controller.models.seedo_controller.seedo_controller import (
    SeeDoController,
)
from ai_controller.models.seedo_controller.task_types import TaskType


def _action_step(
    *,
    ordinal=None,
    action="Pick the green cube and place it in the bin.",
) -> ActionStep:
    return ActionStep(
        pick_keyframe=1,
        place_keyframe=2,
        picked_track_id=1,
        picked_category="cube",
        picked_color="green",
        destination_track_id=10,
        destination_category="bin",
        destination_ordinal_from_left=ordinal,
        relation="in",
        action=action,
        picked_detector_label="green cube",
    )


def _action_plan(
    *,
    status="completed",
    steps=None,
    ambiguities=(),
    natural_language_plan=None,
    task_type=TaskType.PICK_AND_PLACE,
) -> ActionPlanningResult:
    if steps is None:
        steps = (_action_step(),)

    if natural_language_plan is None:
        if steps:
            natural_language_plan = " and then ".join(
                step.action
                for step in steps
            )
        else:
            natural_language_plan = "No action plan generated: ambiguous."

    return ActionPlanningResult(
        steps=tuple(steps),
        status=status,
        ambiguities=tuple(ambiguities),
        natural_language_plan=natural_language_plan,
        task_type=task_type,
    )


def _structured_scene(
    *,
    directions=8,
) -> StructuredScene:
    return StructuredScene(
        objects=(
            StructuredSceneObject(
                object_id="10",
                category="bin",
                center=(100.0, 200.0),
            ),
        ),
        relations=(),
        directions=directions,
    )


def _primitive_plan() -> PrimitivePlan:
    return PrimitivePlan(
        steps=(
            PrimitiveStep(
                name="reach",
                arguments={"target": "runtime_pick"},
            ),
            PrimitiveStep(
                name="pick",
                arguments={"target": "runtime_pick"},
            ),
        ),
        source_code="reach('runtime_pick')\npick('runtime_pick')",
    )


def _bare_controller(
    mode="generalized",
) -> SeeDoController:
    controller = SeeDoController.__new__(
        SeeDoController
    )

    controller.device = None
    controller.demo_path = None
    controller.task_id = None

    controller.artifacts_dir = None
    controller._temporary_artifacts_dir = None

    controller.action_plan = None
    controller.demo_structured_scene = None

    controller.perception_result = None
    controller.scene_state = None
    controller.primitive_plan = None

    controller.runtime_structured_scene = None
    controller.structural_matching_result = None
    controller.replicability_result = None

    controller.execution_status = "idle"
    controller.execution_error = None

    controller.perception_mode = mode
    controller.structured_scene_directions = 8

    controller.workspace_bottom_left = (0.0, 0.0)
    controller.workspace_top_right = (1.0, 1.0)

    controller.keyframe_selector = SimpleNamespace()
    controller.visual_prompter = SimpleNamespace()
    controller.demo_structured_scene_builder = SimpleNamespace()
    controller.action_planner = SimpleNamespace()
    controller.scene_perceiver = SimpleNamespace()
    controller.scene_interpreter = SimpleNamespace()
    controller.runtime_structured_scene_builder = SimpleNamespace()
    controller.structural_matcher = SimpleNamespace()
    controller.replicability_checker = SimpleNamespace()
    controller.lmp_generator = SimpleNamespace()
    controller.motion_layer = SimpleNamespace()

    return controller


def _write_nonempty_file(
    path: Path,
    content: bytes = b"x",
) -> Path:
    path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )
    path.write_bytes(content)
    return path


def _write_action_plan_json(
    path: Path,
    *,
    ordinal=None,
    include_ordinal=True,
) -> Path:
    step = {
        "pick_keyframe": 1,
        "place_keyframe": 2,
        "picked_track_id": 1,
        "picked_category": "cube",
        "picked_color": "green",
        "destination_track_id": 10,
        "destination_category": "bin",
        "relation": "in",
        "action": "Pick the green cube and place it in the bin.",
        "picked_detector_label": "green cube",
    }

    if include_ordinal:
        step["destination_ordinal_from_left"] = ordinal

    path.write_text(
        json.dumps(
            {
                "steps": [step],
                "status": "completed",
                "ambiguities": [],
                "task_type": "pick_and_place",
            }
        ),
        encoding="utf-8",
    )

    return path


def _write_structured_scene_json(
    path: Path,
    *,
    directions=8,
    objects=None,
) -> Path:
    if objects is None:
        objects = [
            {
                "object_id": "10",
                "category": "bin",
                "center": [100, 200],
            }
        ]

    path.write_text(
        json.dumps(
            {
                "directions": directions,
                "objects": objects,
                "relations": [],
            }
        ),
        encoding="utf-8",
    )

    return path


def _runtime_input():
    return {
        "rgb": object(),
        "depth": object(),
        "camera_info": object(),
        "base_to_table_transform": object(),
    }


class _Stage:
    def __init__(
        self,
        result=None,
        error=None,
    ):
        self.result = result
        self.error = error
        self.calls = []

    def run(
        self,
        **kwargs,
    ):
        self.calls.append(kwargs)

        if self.error is not None:
            raise self.error

        return self.result


# ---------------------------------------------------------------------
# load_model
# ---------------------------------------------------------------------


def _patch_model_components(
    monkeypatch,
):
    calls = {}

    def factory(
        name,
    ):
        def build(
            *args,
            **kwargs,
        ):
            calls.setdefault(
                name,
                [],
            ).append(
                {
                    "args": args,
                    "kwargs": kwargs,
                }
            )

            return SimpleNamespace(
                component=name,
                args=args,
                kwargs=kwargs,
            )

        return build

    names = (
        "KeyframeSelector",
        "VisualPrompter",
        "DemoStructuredSceneBuilder",
        "ActionPlanner",
        "ScenePerceiver",
        "SceneInterpreter",
        "RuntimeStructuredSceneBuilder",
        "StructuralMatcher",
        "ReplicabilityChecker",
        "LMPGenerator",
        "SeeDoMotionLayer",
    )

    for name in names:
        monkeypatch.setattr(
            seedo_controller_module,
            name,
            factory(name),
        )

    return calls


def test_load_model_rejects_missing_config(
    tmp_path,
):
    controller = _bare_controller()

    with pytest.raises(
        FileNotFoundError,
        match="model configuration does not exist",
    ):
        controller.load_model(
            tmp_path / "missing.yaml"
        )


@pytest.mark.parametrize(
    "mode",
    [
        "",
        "unknown",
        "prior-guided",
    ],
)
def test_load_model_rejects_invalid_perception_mode(
    tmp_path,
    mode,
):
    config_path = (
        tmp_path
        / "config.yaml"
    )

    config_path.write_text(
        f"perception_mode: {mode!r}\n",
        encoding="utf-8",
    )

    controller = _bare_controller()

    with pytest.raises(
        ValueError,
        match="Invalid perception_mode",
    ):
        controller.load_model(
            config_path
        )


@pytest.mark.parametrize(
    "directions",
    [
        0,
        6,
        16,
    ],
)
def test_load_model_rejects_invalid_structured_scene_directions(
    tmp_path,
    directions,
):
    config_path = (
        tmp_path
        / "config.yaml"
    )

    config_path.write_text(
        (
            "perception_mode: generalized\n"
            "structured_scene:\n"
            f"  directions: {directions}\n"
        ),
        encoding="utf-8",
    )

    controller = _bare_controller()

    with pytest.raises(
        ValueError,
        match="Invalid structured_scene.directions",
    ):
        controller.load_model(
            config_path
        )


def test_load_model_rejects_unknown_camera_pose_noise_level(
    tmp_path,
    monkeypatch,
):
    _patch_model_components(
        monkeypatch
    )

    config_path = (
        tmp_path
        / "config.yaml"
    )

    config_path.write_text(
        """
perception_mode: generalized
scene_perceiver:
  camera_pose_noise:
    level: unknown
    levels:
      baseline:
        translation_std_mm: 0.0
        rotation_std_deg: 0.0
""".strip(),
        encoding="utf-8",
    )

    controller = _bare_controller()

    with pytest.raises(
        ValueError,
        match="Unknown camera pose noise level",
    ):
        controller.load_model(
            config_path
        )


def test_load_model_builds_pipeline_and_propagates_mode(
    tmp_path,
    monkeypatch,
):
    calls = _patch_model_components(
        monkeypatch
    )

    config_path = (
        tmp_path
        / "config.yaml"
    )

    config_path.write_text(
        """
perception_mode: GENERALIZED
structured_scene:
  directions: 4
scene_interpreter:
  model: interpreter-model
scene_perceiver:
  camera_name: zed_front
  workspace_bottom_left: [-0.5, -0.4]
  workspace_top_right: [0.5, 0.6]
  camera_pose_noise:
    level: test
    levels:
      test:
        translation_std_mm: 3.0
        rotation_std_deg: 2.0
lmp_generator:
  model: lmp-model
action_planner:
  model: action-model
  demonstration_bin_order: right_to_left
motion_layer:
  min_step: 0.03
""".strip(),
        encoding="utf-8",
    )

    controller = _bare_controller()

    pipeline = controller.load_model(
        config_path
    )

    assert controller.perception_mode == (
        "generalized"
    )

    assert (
        controller.structured_scene_directions
        == 4
    )

    assert controller.workspace_bottom_left == (
        -0.5,
        -0.4,
    )

    assert controller.workspace_top_right == (
        0.5,
        0.6,
    )

    assert (
        calls["SceneInterpreter"][0][
            "kwargs"
        ]["perception_mode"]
        == "generalized"
    )

    assert (
        calls["ScenePerceiver"][0][
            "kwargs"
        ]["perception_mode"]
        == "generalized"
    )

    assert (
        calls["ScenePerceiver"][0][
            "kwargs"
        ]["translation_noise_std_mm"]
        == pytest.approx(3.0)
    )

    assert (
        calls["ScenePerceiver"][0][
            "kwargs"
        ]["rotation_noise_std_deg"]
        == pytest.approx(2.0)
    )

    assert (
        calls["VisualPrompter"][0][
            "kwargs"
        ]["perception_mode"]
        == "generalized"
    )

    assert (
        calls["ActionPlanner"][0][
            "kwargs"
        ]["perception_mode"]
        == "generalized"
    )

    assert (
        calls["LMPGenerator"][0][
            "kwargs"
        ]["perception_mode"]
        == "generalized"
    )

    assert (
        calls[
            "DemoStructuredSceneBuilder"
        ][0]["kwargs"]["directions"]
        == 4
    )

    assert (
        calls[
            "RuntimeStructuredSceneBuilder"
        ][0]["kwargs"]["directions"]
        == 4
    )

    assert set(
        pipeline.keys()
    ) == {
        "keyframe_selector",
        "visual_prompter",
        "demo_structured_scene_builder",
        "action_planner",
        "scene_perceiver",
        "scene_interpreter",
        "runtime_structured_scene_builder",
        "structural_matcher",
        "replicability_checker",
        "lmp_generator",
        "motion_layer",
    }


# ---------------------------------------------------------------------
# Basic state, pre/post processing and artifacts
# ---------------------------------------------------------------------


def test_move_model_to_device():
    controller = _bare_controller()

    device = object()

    controller.move_model_to_device(
        device
    )

    assert controller.device is device


def test_pre_process_rejects_non_dictionary():
    controller = _bare_controller()

    with pytest.raises(
        TypeError,
        match="expects a dictionary",
    ):
        controller.pre_process(
            []
        )


def test_pre_process_keeps_supported_fields_only():
    controller = _bare_controller()

    rgb = object()
    depth = object()

    result = controller.pre_process(
        {
            "rgb": rgb,
            "depth": depth,
            "camera_info": "camera-info",
            "camera_name": 123,
            "base_to_table_transform": "transform",
            "robot_state": {"state": 1},
            "ignored": "value",
        }
    )

    assert result == {
        "rgb": rgb,
        "depth": depth,
        "camera_info": "camera-info",
        "camera_name": "123",
        "base_to_table_transform": "transform",
        "robot_state": {"state": 1},
    }


def test_post_process_returns_completed_plan():
    controller = _bare_controller()

    plan = _action_plan()

    assert controller.post_process(
        plan
    ) is plan


def test_post_process_rejects_wrong_type():
    controller = _bare_controller()

    with pytest.raises(
        TypeError,
        match="expected an ActionPlanningResult",
    ):
        controller.post_process(
            {}
        )


@pytest.mark.parametrize(
    "status",
    [
        "",
        "failed",
        "planning",
    ],
)
def test_post_process_rejects_unknown_status(
    status,
):
    controller = _bare_controller()

    with pytest.raises(
        ValueError,
        match="Unexpected action-planning status",
    ):
        controller.post_process(
            _action_plan(
                status=status,
            )
        )


def test_post_process_rejects_completed_plan_without_steps():
    controller = _bare_controller()

    with pytest.raises(
        ValueError,
        match="must contain at least one step",
    ):
        controller.post_process(
            _action_plan(
                steps=(),
                status="completed",
                natural_language_plan=(
                    "No action."
                ),
            )
        )


def test_post_process_rejects_ambiguous_plan_without_reason():
    controller = _bare_controller()

    with pytest.raises(
        ValueError,
        match="must explain its ambiguities",
    ):
        controller.post_process(
            _action_plan(
                steps=(),
                status="ambiguous",
                ambiguities=(),
                natural_language_plan=(
                    "Ambiguous."
                ),
            )
        )


def test_post_process_accepts_explained_ambiguous_plan():
    controller = _bare_controller()

    plan = _action_plan(
        steps=(),
        status="ambiguous",
        ambiguities=(
            "destination ambiguous",
        ),
        natural_language_plan=(
            "No action plan generated: "
            "destination ambiguous"
        ),
    )

    assert controller.post_process(
        plan
    ) is plan


@pytest.mark.parametrize(
    "text",
    [
        "",
        "   ",
    ],
)
def test_post_process_rejects_empty_natural_language_plan(
    text,
):
    controller = _bare_controller()

    with pytest.raises(
        ValueError,
        match="natural-language action plan is empty",
    ):
        controller.post_process(
            _action_plan(
                natural_language_plan=text,
            )
        )


def test_prepare_artifacts_dir_creates_persistent_directory(
    tmp_path,
):
    controller = _bare_controller()

    artifacts = (
        tmp_path
        / "nested"
        / "artifacts"
    )

    result = controller._prepare_artifacts_dir(
        artifacts
    )

    assert result == artifacts.resolve()
    assert result.is_dir()
    assert controller.artifacts_dir == result
    assert (
        controller._temporary_artifacts_dir
        is None
    )


def test_prepare_artifacts_dir_creates_and_cleans_temporary_directory():
    controller = _bare_controller()

    result = controller._prepare_artifacts_dir(
        None
    )

    assert result.is_dir()
    assert (
        controller._temporary_artifacts_dir
        is not None
    )

    controller._cleanup_temporary_artifacts()

    assert not result.exists()
    assert controller.artifacts_dir is None
    assert (
        controller._temporary_artifacts_dir
        is None
    )


def test_reset_clears_pipeline_state_and_temporary_artifacts():
    controller = _bare_controller()

    temp_dir = controller._prepare_artifacts_dir(
        None
    )

    controller.perception_result = object()
    controller.action_plan = _action_plan()
    controller.scene_state = SceneState(
        objects=()
    )
    controller.primitive_plan = _primitive_plan()
    controller.demo_structured_scene = _structured_scene()
    controller.runtime_structured_scene = _structured_scene()
    controller.structural_matching_result = object()
    controller.replicability_result = object()
    controller.execution_status = "failed"
    controller.execution_error = "error"
    controller.demo_path = Path("/tmp/demo.mp4")
    controller.task_id = "task"

    controller.reset()

    assert controller.perception_result is None
    assert controller.action_plan is None
    assert controller.scene_state is None
    assert controller.primitive_plan is None
    assert controller.demo_structured_scene is None
    assert controller.runtime_structured_scene is None
    assert controller.structural_matching_result is None
    assert controller.replicability_result is None
    assert controller.execution_status == "idle"
    assert controller.execution_error is None
    assert controller.demo_path is None
    assert controller.task_id is None
    assert controller.artifacts_dir is None
    assert not temp_dir.exists()


# ---------------------------------------------------------------------
# Precomputed loaders
# ---------------------------------------------------------------------


def test_load_precomputed_action_plan_rejects_missing_file(
    tmp_path,
):
    controller = _bare_controller()

    with pytest.raises(
        FileNotFoundError,
        match="action plan does not exist",
    ):
        controller._load_precomputed_action_plan(
            tmp_path / "missing.json"
        )


def test_load_precomputed_action_plan_rejects_empty_file(
    tmp_path,
):
    controller = _bare_controller()

    path = (
        tmp_path
        / "empty.json"
    )
    path.touch()

    with pytest.raises(
        ValueError,
        match="action plan is empty",
    ):
        controller._load_precomputed_action_plan(
            path
        )


def test_load_precomputed_generalized_plan_discards_ordinal(
    tmp_path,
):
    controller = _bare_controller(
        mode="generalized"
    )

    path = _write_action_plan_json(
        tmp_path / "plan.json",
        ordinal=3,
    )

    plan = (
        controller
        ._load_precomputed_action_plan(
            path
        )
    )

    assert plan.status == "completed"
    assert len(plan.steps) == 1

    step = plan.steps[0]

    assert (
        step.destination_ordinal_from_left
        is None
    )
    assert step.picked_detector_label == (
        "green cube"
    )
    assert plan.task_type == (
        "pick_and_place"
    )
    assert plan.natural_language_plan == (
        step.action
    )


def test_load_precomputed_prior_guided_plan_requires_ordinal(
    tmp_path,
):
    controller = _bare_controller(
        mode="prior_guided"
    )

    path = _write_action_plan_json(
        tmp_path / "plan.json",
        include_ordinal=False,
    )

    with pytest.raises(
        ValueError,
        match="missing destination_ordinal_from_left",
    ):
        controller._load_precomputed_action_plan(
            path
        )


def test_load_precomputed_prior_guided_plan_preserves_ordinal(
    tmp_path,
):
    controller = _bare_controller(
        mode="prior_guided"
    )

    path = _write_action_plan_json(
        tmp_path / "plan.json",
        ordinal="2",
    )

    plan = (
        controller
        ._load_precomputed_action_plan(
            path
        )
    )

    assert (
        plan.steps[0]
        .destination_ordinal_from_left
        == 2
    )


def test_load_precomputed_structured_scene_rejects_missing_file(
    tmp_path,
):
    controller = _bare_controller()

    with pytest.raises(
        FileNotFoundError,
        match="structured scene does not exist",
    ):
        controller._load_precomputed_demo_structured_scene(
            tmp_path / "missing.json"
        )


def test_load_precomputed_structured_scene_rejects_empty_file(
    tmp_path,
):
    controller = _bare_controller()

    path = (
        tmp_path
        / "empty.json"
    )
    path.touch()

    with pytest.raises(
        ValueError,
        match="structured scene is empty",
    ):
        controller._load_precomputed_demo_structured_scene(
            path
        )


@pytest.mark.parametrize(
    "directions",
    [
        0,
        6,
        16,
    ],
)
def test_load_precomputed_structured_scene_rejects_invalid_direction_mode(
    tmp_path,
    directions,
):
    controller = _bare_controller()

    path = _write_structured_scene_json(
        tmp_path / "scene.json",
        directions=directions,
    )

    with pytest.raises(
        ValueError,
        match="Invalid direction mode",
    ):
        controller._load_precomputed_demo_structured_scene(
            path
        )


def test_load_precomputed_structured_scene_rejects_configured_mode_mismatch(
    tmp_path,
):
    controller = _bare_controller()
    controller.structured_scene_directions = 4

    path = _write_structured_scene_json(
        tmp_path / "scene.json",
        directions=8,
    )

    with pytest.raises(
        ValueError,
        match="uses a different direction mode",
    ):
        controller._load_precomputed_demo_structured_scene(
            path
        )


def test_load_precomputed_structured_scene_rejects_empty_objects(
    tmp_path,
):
    controller = _bare_controller()

    path = _write_structured_scene_json(
        tmp_path / "scene.json",
        objects=[],
    )

    with pytest.raises(
        ValueError,
        match="contains no objects",
    ):
        controller._load_precomputed_demo_structured_scene(
            path
        )


def test_load_precomputed_structured_scene_loads_objects_and_relations(
    tmp_path,
):
    controller = _bare_controller()

    path = (
        tmp_path
        / "scene.json"
    )

    path.write_text(
        json.dumps(
            {
                "directions": 8,
                "objects": [
                    {
                        "object_id": 10,
                        "category": "bin",
                        "center": [100, 200.5],
                    },
                    {
                        "object_id": 20,
                        "category": "bin",
                        "center": [300, 200.5],
                    },
                ],
                "relations": [
                    {
                        "subject_object_id": 10,
                        "reference_object_id": 20,
                        "relation": "left",
                    },
                    {
                        "subject_object_id": 20,
                        "reference_object_id": 10,
                        "relation": "right",
                    },
                ],
            }
        ),
        encoding="utf-8",
    )

    scene = (
        controller
        ._load_precomputed_demo_structured_scene(
            path
        )
    )

    assert scene.directions == 8

    assert tuple(
        obj.object_id
        for obj in scene.objects
    ) == (
        "10",
        "20",
    )

    assert scene.objects[0].center == (
        100.0,
        200.5,
    )

    assert (
        scene.relations[0].relation
        == "left"
    )


# ---------------------------------------------------------------------
# load_command
# ---------------------------------------------------------------------


def test_load_command_rejects_missing_demo_video(
    tmp_path,
):
    controller = _bare_controller()

    with pytest.raises(
        FileNotFoundError,
        match="demonstration video does not exist",
    ):
        controller.load_command(
            demo_path=str(
                tmp_path
                / "missing.mp4"
            ),
            task_id="task",
        )


def test_load_command_rejects_empty_demo_video(
    tmp_path,
):
    controller = _bare_controller()

    video = (
        tmp_path
        / "demo.mp4"
    )
    video.touch()

    with pytest.raises(
        ValueError,
        match="demonstration video is empty",
    ):
        controller.load_command(
            demo_path=str(video),
            task_id="task",
        )


def _install_demo_pipeline(
    controller,
    *,
    action_result=None,
):
    if action_result is None:
        action_result = _action_plan()

    keyframe_result = (
        SimpleNamespace(
            keyframes=(3, 9),
        )
    )

    visual_result = (
        SimpleNamespace(
            annotated_video_path=Path(
                "/tmp/annotated.mp4"
            ),
            track_id_map={
                1: {
                    "category": "cube",
                }
            },
            key_frame_coordinates={
                3: {},
                9: {},
            },
        )
    )

    demo_scene = _structured_scene()

    controller.keyframe_selector = (
        _Stage(
            result=keyframe_result
        )
    )

    controller.visual_prompter = (
        _Stage(
            result=visual_result
        )
    )

    controller.demo_structured_scene_builder = (
        _Stage(
            result=demo_scene
        )
    )

    controller.action_planner = (
        _Stage(
            result=action_result
        )
    )

    return (
        keyframe_result,
        visual_result,
        demo_scene,
    )


def test_load_command_generalized_runs_demo_structured_scene_pipeline(
    tmp_path,
):
    controller = _bare_controller(
        mode="generalized"
    )

    (
        keyframe_result,
        visual_result,
        demo_scene,
    ) = _install_demo_pipeline(
        controller
    )

    video = _write_nonempty_file(
        tmp_path / "demo.mp4"
    )

    artifacts = (
        tmp_path
        / "artifacts"
    )

    controller.load_command(
        demo_path=str(video),
        task_id="task_00",
        artifacts_dir=artifacts,
    )

    assert controller.demo_path == (
        video.resolve()
    )
    assert controller.task_id == "task_00"
    assert controller.execution_status == (
        "plan_ready"
    )
    assert controller.execution_error is None
    assert controller.action_plan == (
        _action_plan()
    )
    assert (
        controller.demo_structured_scene
        is demo_scene
    )

    assert len(
        controller.keyframe_selector.calls
    ) == 1

    assert (
        controller.visual_prompter.calls[0][
            "keyframes"
        ]
        == keyframe_result.keyframes
    )

    assert (
        controller.demo_structured_scene_builder
        .calls[0]["visual_prompting_result"]
        is visual_result
    )

    assert (
        controller.action_planner.calls[0][
            "track_id_map"
        ]
        is visual_result.track_id_map
    )


def test_load_command_prior_guided_skips_demo_structured_scene(
    tmp_path,
):
    controller = _bare_controller(
        mode="prior_guided"
    )

    _install_demo_pipeline(
        controller
    )

    controller.demo_structured_scene_builder = (
        _Stage(
            error=AssertionError(
                "must not run"
            )
        )
    )

    video = _write_nonempty_file(
        tmp_path / "demo.mp4"
    )

    controller.load_command(
        demo_path=str(video),
        task_id="task",
        artifacts_dir=(
            tmp_path
            / "artifacts"
        ),
    )

    assert controller.execution_status == (
        "plan_ready"
    )
    assert controller.demo_structured_scene is None
    assert (
        controller.demo_structured_scene_builder.calls
        == []
    )


def test_load_command_failure_sets_failed_state(
    tmp_path,
):
    controller = _bare_controller()

    controller.keyframe_selector = (
        _Stage(
            error=RuntimeError(
                "keyframe failure"
            )
        )
    )

    video = _write_nonempty_file(
        tmp_path / "demo.mp4"
    )

    with pytest.raises(
        RuntimeError,
        match="keyframe failure",
    ):
        controller.load_command(
            demo_path=str(video),
            task_id="task",
            artifacts_dir=(
                tmp_path
                / "artifacts"
            ),
        )

    assert controller.execution_status == (
        "failed"
    )
    assert controller.execution_error == (
        "keyframe failure"
    )


def test_load_command_generalized_precomputed_requires_demo_structured_scene(
    tmp_path,
    monkeypatch,
):
    controller = _bare_controller(
        mode="generalized"
    )

    video = _write_nonempty_file(
        tmp_path / "demo.mp4"
    )

    monkeypatch.setattr(
        controller,
        "_load_precomputed_action_plan",
        lambda path: _action_plan(),
    )

    with pytest.raises(
        ValueError,
        match="requires precomputed_demo_structured_scene_path",
    ):
        controller.load_command(
            demo_path=str(video),
            task_id="task",
            artifacts_dir=(
                tmp_path
                / "artifacts"
            ),
            precomputed_action_plan_path="plan.json",
        )

    assert controller.execution_status == (
        "failed"
    )


def test_load_command_generalized_precomputed_loads_both_artifacts(
    tmp_path,
    monkeypatch,
):
    controller = _bare_controller(
        mode="generalized"
    )

    video = _write_nonempty_file(
        tmp_path / "demo.mp4"
    )

    plan = _action_plan()
    scene = _structured_scene()

    captured = {}

    def load_plan(
        path,
    ):
        captured["plan_path"] = path
        return plan

    def load_scene(
        path,
    ):
        captured["scene_path"] = path
        return scene

    monkeypatch.setattr(
        controller,
        "_load_precomputed_action_plan",
        load_plan,
    )

    monkeypatch.setattr(
        controller,
        "_load_precomputed_demo_structured_scene",
        load_scene,
    )

    controller.load_command(
        demo_path=str(video),
        task_id="task",
        artifacts_dir=(
            tmp_path
            / "artifacts"
        ),
        precomputed_action_plan_path="plan.json",
        precomputed_demo_structured_scene_path="scene.json",
    )

    assert controller.action_plan is plan
    assert (
        controller.demo_structured_scene
        is scene
    )
    assert captured == {
        "plan_path": "plan.json",
        "scene_path": "scene.json",
    }
    assert controller.execution_status == (
        "plan_ready"
    )


def test_load_command_prior_guided_precomputed_does_not_require_structured_scene(
    tmp_path,
    monkeypatch,
):
    controller = _bare_controller(
        mode="prior_guided"
    )

    video = _write_nonempty_file(
        tmp_path / "demo.mp4"
    )

    plan = _action_plan()

    monkeypatch.setattr(
        controller,
        "_load_precomputed_action_plan",
        lambda path: plan,
    )

    def fail_if_called(
        path,
    ):
        raise AssertionError(
            "structured scene loader must not run"
        )

    monkeypatch.setattr(
        controller,
        "_load_precomputed_demo_structured_scene",
        fail_if_called,
    )

    controller.load_command(
        demo_path=str(video),
        task_id="task",
        artifacts_dir=(
            tmp_path
            / "artifacts"
        ),
        precomputed_action_plan_path="plan.json",
    )

    assert controller.action_plan is plan
    assert controller.demo_structured_scene is None
    assert controller.execution_status == (
        "plan_ready"
    )


# ---------------------------------------------------------------------
# inference(t=0)
# ---------------------------------------------------------------------


def _prepare_runtime_pipeline(
    controller,
    *,
    replicable=True,
):
    perception_result = object()
    scene_state = SceneState(
        objects=()
    )
    runtime_scene = _structured_scene()
    matching_result = object()

    replicability_result = (
        ReplicabilityResult(
            replicable=replicable,
            resolved_targets=(),
            failure_reasons=(
                ()
                if replicable
                else (
                    "destination is ambiguous",
                )
            ),
        )
    )

    primitive_plan = _primitive_plan()

    controller.scene_perceiver = (
        _Stage(
            result=perception_result
        )
    )

    controller.scene_interpreter = (
        _Stage(
            result=scene_state
        )
    )

    controller.runtime_structured_scene_builder = (
        _Stage(
            result=runtime_scene
        )
    )

    controller.structural_matcher = (
        _Stage(
            result=matching_result
        )
    )

    controller.replicability_checker = (
        _Stage(
            result=replicability_result
        )
    )

    controller.lmp_generator = (
        _Stage(
            result=primitive_plan
        )
    )

    return (
        perception_result,
        scene_state,
        runtime_scene,
        matching_result,
        replicability_result,
        primitive_plan,
    )


def test_inference_requires_load_command():
    controller = _bare_controller()

    with pytest.raises(
        RuntimeError,
        match=r"load_command\(\) must be called",
    ):
        controller.inference(
            {},
            t=0,
        )


def test_inference_requires_artifact_storage():
    controller = _bare_controller()
    controller.action_plan = _action_plan()

    with pytest.raises(
        RuntimeError,
        match="Artifact storage has not been initialized",
    ):
        controller.inference(
            {},
            t=0,
        )


def test_inference_rejects_negative_timestep(
    tmp_path,
):
    controller = _bare_controller()
    controller.action_plan = _action_plan()
    controller.artifacts_dir = tmp_path

    with pytest.raises(
        ValueError,
        match="cannot be negative",
    ):
        controller.inference(
            {},
            t=-1,
        )


def test_inference_t0_requires_all_perception_inputs(
    tmp_path,
):
    controller = _bare_controller()
    controller.action_plan = _action_plan()
    controller.artifacts_dir = tmp_path

    with pytest.raises(
        ValueError,
        match="Missing runtime perception inputs",
    ) as exc_info:
        controller.inference(
            {
                "rgb": object(),
            },
            t=0,
        )

    message = str(
        exc_info.value
    )

    assert "depth" in message
    assert "camera_info" in message
    assert (
        "base_to_table_transform"
        in message
    )


def test_inference_t0_generalized_runs_structural_pipeline(
    tmp_path,
):
    controller = _bare_controller(
        mode="generalized"
    )

    controller.action_plan = _action_plan()
    controller.demo_structured_scene = (
        _structured_scene()
    )
    controller.artifacts_dir = tmp_path

    (
        perception_result,
        scene_state,
        runtime_scene,
        matching_result,
        replicability_result,
        primitive_plan,
    ) = _prepare_runtime_pipeline(
        controller
    )

    result = controller.inference(
        _runtime_input(),
        t=0,
    )

    assert result is None
    assert controller.execution_status == (
        "plan_ready"
    )
    assert controller.execution_error is None

    assert (
        controller.perception_result
        is perception_result
    )
    assert controller.scene_state is scene_state
    assert (
        controller.runtime_structured_scene
        is runtime_scene
    )
    assert (
        controller.structural_matching_result
        is matching_result
    )
    assert (
        controller.replicability_result
        is replicability_result
    )
    assert controller.primitive_plan is (
        primitive_plan
    )

    assert len(
        controller.runtime_structured_scene_builder.calls
    ) == 1
    assert len(
        controller.structural_matcher.calls
    ) == 1
    assert len(
        controller.replicability_checker.calls
    ) == 1

    lmp_call = (
        controller.lmp_generator.calls[0]
    )

    assert (
        lmp_call["replicability_result"]
        is replicability_result
    )


def test_inference_t0_prior_guided_skips_structural_pipeline(
    tmp_path,
):
    controller = _bare_controller(
        mode="prior_guided"
    )

    controller.action_plan = _action_plan()
    controller.artifacts_dir = tmp_path

    (
        _,
        scene_state,
        _,
        _,
        _,
        primitive_plan,
    ) = _prepare_runtime_pipeline(
        controller
    )

    controller.runtime_structured_scene_builder = (
        _Stage(
            error=AssertionError(
                "must not run"
            )
        )
    )
    controller.structural_matcher = (
        _Stage(
            error=AssertionError(
                "must not run"
            )
        )
    )
    controller.replicability_checker = (
        _Stage(
            error=AssertionError(
                "must not run"
            )
        )
    )

    result = controller.inference(
        _runtime_input(),
        t=0,
    )

    assert result is None
    assert controller.execution_status == (
        "plan_ready"
    )
    assert controller.scene_state is scene_state
    assert controller.primitive_plan is (
        primitive_plan
    )
    assert controller.demo_structured_scene is None
    assert controller.runtime_structured_scene is None
    assert controller.structural_matching_result is None
    assert controller.replicability_result is None

    assert (
        controller.runtime_structured_scene_builder.calls
        == []
    )
    assert (
        controller.structural_matcher.calls
        == []
    )
    assert (
        controller.replicability_checker.calls
        == []
    )

    assert (
        controller.lmp_generator.calls[0][
            "replicability_result"
        ]
        is None
    )


def test_inference_t0_generalized_requires_demo_structured_scene(
    tmp_path,
):
    controller = _bare_controller(
        mode="generalized"
    )

    controller.action_plan = _action_plan()
    controller.artifacts_dir = tmp_path

    _prepare_runtime_pipeline(
        controller
    )

    controller.demo_structured_scene = None

    with pytest.raises(
        RuntimeError,
        match="Demo structured scene is not available",
    ):
        controller.inference(
            _runtime_input(),
            t=0,
        )

    assert controller.execution_status == (
        "failed"
    )
    assert (
        "Demo structured scene is not available"
        in controller.execution_error
    )


def test_inference_t0_rejects_non_replicable_runtime_scene(
    tmp_path,
):
    controller = _bare_controller(
        mode="generalized"
    )

    controller.action_plan = _action_plan()
    controller.demo_structured_scene = (
        _structured_scene()
    )
    controller.artifacts_dir = tmp_path

    _prepare_runtime_pipeline(
        controller,
        replicable=False,
    )

    with pytest.raises(
        RuntimeError,
        match="not replicable",
    ):
        controller.inference(
            _runtime_input(),
            t=0,
        )

    assert controller.execution_status == (
        "failed"
    )
    assert (
        "destination is ambiguous"
        in controller.execution_error
    )
    assert (
        controller.lmp_generator.calls
        == []
    )


def test_inference_t0_stage_failure_sets_failed_state(
    tmp_path,
):
    controller = _bare_controller()

    controller.action_plan = _action_plan()
    controller.demo_structured_scene = (
        _structured_scene()
    )
    controller.artifacts_dir = tmp_path

    controller.scene_perceiver = (
        _Stage(
            error=RuntimeError(
                "perception failure"
            )
        )
    )

    with pytest.raises(
        RuntimeError,
        match="perception failure",
    ):
        controller.inference(
            _runtime_input(),
            t=0,
        )

    assert controller.execution_status == (
        "failed"
    )
    assert controller.execution_error == (
        "perception failure"
    )


# ---------------------------------------------------------------------
# inference(t>=1): Motion Layer
# ---------------------------------------------------------------------


def _prepare_motion_execution(
    controller,
    tmp_path,
):
    controller.action_plan = _action_plan()
    controller.artifacts_dir = tmp_path
    controller.scene_state = SceneState(
        objects=()
    )
    controller.primitive_plan = (
        _primitive_plan()
    )


def test_inference_after_t0_requires_primitive_plan(
    tmp_path,
):
    controller = _bare_controller()

    controller.action_plan = _action_plan()
    controller.artifacts_dir = tmp_path
    controller.scene_state = SceneState(
        objects=()
    )

    with pytest.raises(
        RuntimeError,
        match="No primitive plan has been generated yet",
    ):
        controller.inference(
            {},
            t=1,
        )


def test_inference_after_t0_requires_scene_state(
    tmp_path,
):
    controller = _bare_controller()

    controller.action_plan = _action_plan()
    controller.artifacts_dir = tmp_path
    controller.primitive_plan = (
        _primitive_plan()
    )

    with pytest.raises(
        RuntimeError,
        match="No SceneState is available",
    ):
        controller.inference(
            {},
            t=1,
        )


def test_inference_t1_requires_robot_state(
    tmp_path,
):
    controller = _bare_controller()

    _prepare_motion_execution(
        controller,
        tmp_path,
    )

    with pytest.raises(
        ValueError,
        match="requires robot_state",
    ):
        controller.inference(
            {},
            t=1,
        )


@pytest.mark.parametrize(
    "missing_key",
    [
        seedo_controller_module.EEF_POS_NAME,
        seedo_controller_module.EEF_QUAT_NAME,
    ],
)
def test_inference_t1_requires_complete_robot_state(
    tmp_path,
    missing_key,
):
    controller = _bare_controller()

    _prepare_motion_execution(
        controller,
        tmp_path,
    )

    robot_state = {
        seedo_controller_module.EEF_POS_NAME: (
            0.1,
            0.2,
            0.3,
        ),
        seedo_controller_module.EEF_QUAT_NAME: (
            0.0,
            0.0,
            0.0,
            1.0,
        ),
    }

    del robot_state[
        missing_key
    ]

    with pytest.raises(
        ValueError,
        match="does not contain",
    ) as exc_info:
        controller.inference(
            {
                "robot_state": robot_state,
            },
            t=1,
        )

    assert missing_key in str(
        exc_info.value
    )


def test_inference_t1_resets_motion_layer_and_translates_first_primitive(
    tmp_path,
):
    controller = _bare_controller()

    _prepare_motion_execution(
        controller,
        tmp_path,
    )

    calls = {
        "reset": [],
        "translate": [],
    }

    expected_actions = [
        "action-1",
        "action-2",
    ]

    class MotionLayer:
        def reset(
            self,
            **kwargs,
        ):
            calls["reset"].append(
                kwargs
            )

        def translate(
            self,
            **kwargs,
        ):
            calls["translate"].append(
                kwargs
            )
            return expected_actions

    controller.motion_layer = (
        MotionLayer()
    )

    robot_state = {
        seedo_controller_module.EEF_POS_NAME: (
            0.1,
            0.2,
            0.3,
        ),
        seedo_controller_module.EEF_QUAT_NAME: (
            0.0,
            0.0,
            0.0,
            1.0,
        ),
    }

    result = controller.inference(
        {
            "robot_state": robot_state,
        },
        t=1,
    )

    assert result is expected_actions
    assert controller.execution_status == (
        "executing"
    )

    assert len(
        calls["reset"]
    ) == 1

    assert calls["reset"][0][
        "current_position"
    ] == robot_state[
        seedo_controller_module.EEF_POS_NAME
    ]

    assert calls["reset"][0][
        "current_orientation"
    ] == robot_state[
        seedo_controller_module.EEF_QUAT_NAME
    ]

    assert calls["translate"][0][
        "primitive_step"
    ] is controller.primitive_plan.steps[0]

    assert calls["translate"][0][
        "scene_state"
    ] is controller.scene_state


def test_inference_t2_does_not_reset_motion_layer_again(
    tmp_path,
):
    controller = _bare_controller()

    _prepare_motion_execution(
        controller,
        tmp_path,
    )

    calls = {
        "reset": 0,
        "translate": [],
    }

    class MotionLayer:
        def reset(
            self,
            **kwargs,
        ):
            calls["reset"] += 1

        def translate(
            self,
            **kwargs,
        ):
            calls["translate"].append(
                kwargs
            )
            return ["action"]

    controller.motion_layer = (
        MotionLayer()
    )

    result = controller.inference(
        {},
        t=2,
    )

    assert result == [
        "action"
    ]
    assert calls["reset"] == 0

    assert calls["translate"][0][
        "primitive_step"
    ] is controller.primitive_plan.steps[1]


def test_inference_marks_completed_after_last_primitive(
    tmp_path,
):
    controller = _bare_controller()

    _prepare_motion_execution(
        controller,
        tmp_path,
    )

    result = controller.inference(
        {},
        t=3,
    )

    assert result is None
    assert controller.execution_status == (
        "completed"
    )


def test_inference_motion_failure_sets_failed_state(
    tmp_path,
):
    controller = _bare_controller()

    _prepare_motion_execution(
        controller,
        tmp_path,
    )

    class MotionLayer:
        def reset(
            self,
            **kwargs,
        ):
            pass

        def translate(
            self,
            **kwargs,
        ):
            raise RuntimeError(
                "motion failure"
            )

    controller.motion_layer = (
        MotionLayer()
    )

    robot_state = {
        seedo_controller_module.EEF_POS_NAME: (
            0.1,
            0.2,
            0.3,
        ),
        seedo_controller_module.EEF_QUAT_NAME: (
            0.0,
            0.0,
            0.0,
            1.0,
        ),
    }

    with pytest.raises(
        RuntimeError,
        match="motion failure",
    ):
        controller.inference(
            {
                "robot_state": robot_state,
            },
            t=1,
        )

    assert controller.execution_status == (
        "failed"
    )
    assert controller.execution_error == (
        "motion failure"
    )
