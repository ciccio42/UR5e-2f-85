"""Regression tests for SeeDo generalized destination resolution.

Place in:
    ai_controller/ai_controller/models/seedo_controller/tests/unit/

Covers: load_model strategy selection, ActionPlanner forwarding,
SceneInterpreter naming, historical 1D LMP resolution, structural 2D
LMP resolution, Full Demo / Precomputed / Runtime t=0 pipeline routing,
and precomputed artifact compatibility. No API requests, camera, ROS
node, GraspMolmo or robot movement are performed.

The precomputed compatibility tests target the LOCAL controller change
which rejects mixing ordinal_1d and structural_2d action plans.
"""

from __future__ import annotations

import warnings

# Suppress only known warnings from third-party dependencies during collection.
# Leave other warning messages visible, including new SeeDo/test warnings.
_EXTERNAL_WARNING_FILTERS = (
    (
        r"Type google\._upb\._message\."
        r"(MessageMapContainer|ScalarMapContainer) uses PyType_Spec.*",
        DeprecationWarning,
    ),
    (
        r"Unable to import Axes3D\..*",
        UserWarning,
    ),
    (
        r"`torch\.jit\.interface` is deprecated\..*",
        DeprecationWarning,
    ),
    (
        r"Importing from timm\.models\.layers is deprecated.*",
        FutureWarning,
    ),
    (
        r"invalid escape sequence '\\s'",
        SyntaxWarning,
    ),
)

for _message, _category in _EXTERNAL_WARNING_FILTERS:
    warnings.filterwarnings(
        "ignore", message=_message, category=_category,
    )

import json
from dataclasses import replace
from contextlib import nullcontext
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

# Also suppress the same known dependency warnings during test execution.
pytestmark = [
    pytest.mark.filterwarnings(
        f"ignore:{message}:{category.__name__}"
    )
    for message, category in _EXTERNAL_WARNING_FILTERS
]

from results import (
    ActionPlanningResult,
    ActionStep,
    PrimitivePlan,
    PrimitiveStep,
    RawSceneObject,
    RawSceneState,
    ReplicabilityResult,
    ResolvedActionTargets,
    SceneObject,
    ScenePerceptionResult,
    SceneState,
)
from ai_controller.models.seedo_controller import (
    action_planner as action_planner_module,
    lmp_generator as lmp_module,
    seedo_controller as controller_module,
)
from ai_controller.models.seedo_controller.action_planner import ActionPlanner
from ai_controller.models.seedo_controller.lmp_generator import (
    LMPGenerator,
    LMPSceneWrapper,
)
from ai_controller.models.seedo_controller.scene_interpreter import SceneInterpreter
from ai_controller.models.seedo_controller.seedo_controller import SeeDoController
from ai_controller.models.seedo_controller.task_types import TaskType
from vlm import (
    GENERALIZED_PLAN_SCHEMA,
    PLAN_SCHEMA,
    build_prompt_ordinal_1d,
    validate_plan,
    validate_plan_ordinal_1d,
)


# ---------------------------------------------------------------------------
# Shared minimal data: no device/camera/LLM interaction.
# ---------------------------------------------------------------------------


def _step(*, ordinal=None, grasp_instruction="") -> ActionStep:
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
        action="Pick the green cube and place it in the storage bin.",
        picked_detector_label="green cube",
        grasp_instruction=grasp_instruction,
    )


def _plan(*, ordinal=None, grasp_instruction="") -> ActionPlanningResult:
    step = _step(ordinal=ordinal, grasp_instruction=grasp_instruction)
    return ActionPlanningResult(
        steps=(step,),
        status="completed",
        ambiguities=(),
        natural_language_plan=step.action,
        task_type=TaskType.PICK_AND_PLACE,
    )


def _plan_file(path: Path, *, ordinal=None, grasp_instruction=None) -> Path:
    step = {
        "pick_keyframe": 1,
        "place_keyframe": 2,
        "picked_track_id": 1,
        "picked_category": "cube",
        "picked_color": "green",
        "destination_track_id": 10,
        "destination_category": "bin",
        "relation": "in",
        "action": "Pick the green cube and place it in the storage bin.",
        "picked_detector_label": "green cube",
        "destination_ordinal_from_left": ordinal,
    }
    if grasp_instruction is not None:
        step["grasp_instruction"] = grasp_instruction
    path.write_text(
        json.dumps({
            "steps": [step],
            "status": "completed",
            "ambiguities": [],
            "task_type": "pick_and_place",
        }),
        encoding="utf-8",
    )
    return path


def _scene_object(object_id, label, category, x):
    return SceneObject(
        object_id=object_id,
        label=label,
        pixel_coordinates=(x, 25),
        position_camera=(0.1, 0.2, 0.3),
        position_base=(0.4, 0.5, 0.6),
        category=category,
        attributes={},
    )


def _scene():
    # Order intentionally different from spatial order.
    return SceneState(objects=(
        _scene_object("bin_right", "storage bin", "bin", 300),
        _scene_object("cube", "green cube", "cube", 130),
        _scene_object("bin_left", "storage bin", "bin", 100),
    ))


def _raw_object(object_id, label, category, x):
    return RawSceneObject(
        object_id=object_id,
        label=label,
        category=category,
        attributes={},
        pixel_coordinates=(x, 25),
        position_camera=(0.1, 0.2, 0.3),
        position_base=(0.4, 0.5, 0.6),
        confidence=0.9,
        mask=None,
    )


def _raw_scene():
    return RawSceneState(objects=(
        _raw_object("raw_right", "storage bin", "bin", 300),
        _raw_object("raw_pick", "green cube", "cube", 130),
        _raw_object("raw_left", "storage bin", "bin", 100),
    ))


def _controller(mode="generalized", resolution="structural_2d"):
    """Bypass ROS/controller initialization; install state only."""
    c = SeeDoController.__new__(SeeDoController)
    c.perception_mode = mode
    c.generalized_destination_resolution = resolution
    c.use_structural_matching = (
        mode == "generalized" and resolution == "structural_2d"
    )
    c.grasp_planning_enabled = c.use_structural_matching
    c.grasp_planner = None
    c.token_usage = SimpleNamespace(reset=MagicMock())
    c._temporary_artifacts_dir = None
    c.artifacts_dir = None
    c.action_plan = None
    c.demo_structured_scene = None
    c.runtime_structured_scene = None
    c.structural_matching_result = None
    c.replicability_result = None
    c.scene_state = None
    c.primitive_plan = None
    c.execution_status = "idle"
    c.execution_error = None
    c.workspace_bottom_left = (0.0, 0.0)
    c.workspace_top_right = (1.0, 1.0)
    c._reset_nut_visual_debug_state = MagicMock()
    return c


@pytest.fixture
def no_pipeline_side_effects(monkeypatch):
    monkeypatch.setattr(
        controller_module.TOKEN_USAGE, "stage", lambda *args, **kwargs: nullcontext()
    )
    monkeypatch.setattr(
        controller_module.TIMING, "measure", lambda *args, **kwargs: nullcontext()
    )
    monkeypatch.setattr(controller_module, "emit_pipeline_event", MagicMock())


# ---------------------------------------------------------------------------
# Configuration: strategy normalization, propagation and grasp gating.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "mode,resolution,prior_task,expected_structural",
    [
        ("generalized", "ordinal_1d", None, False),
        ("generalized", "structural_2d", None, True),
        ("prior_guided", "ordinal_1d", "pick_and_place", False),
        ("prior_guided", "structural_2d", "nut_assembly", False),
    ],
)
def test_load_model_routes_all_components_and_grasp(
    tmp_path, monkeypatch, mode, resolution, prior_task, expected_structural
):
    captures = {}
    constructors = (
        "KeyframeSelector", "VisualPrompter", "DemoStructuredSceneBuilder",
        "ActionPlanner", "ScenePerceiver", "SceneInterpreter",
        "RuntimeStructuredSceneBuilder", "StructuralMatcher",
        "ReplicabilityChecker", "LMPGenerator", "SeeDoMotionLayer",
    )

    for name in constructors:
        ctor = MagicMock(name=name)
        ctor.return_value = SimpleNamespace(name=name)
        monkeypatch.setattr(controller_module, name, ctor)
        captures[name] = ctor

    grasp_factory = MagicMock(return_value=SimpleNamespace(name="grasp"))
    monkeypatch.setattr(controller_module, "build_grasp_planner", grasp_factory)

    config = tmp_path / "config.yaml"
    config.write_text(
        f"perception_mode: {mode}\n"
        f"prior_guided_task: {prior_task or 'null'}\n"
        f"generalized_destination_resolution: {resolution}\n"
        "grasp_planner:\n  enabled: true\n"
        "scene_perceiver:\n"
        "  camera_pose_noise:\n"
        "    level: baseline\n"
        "    levels:\n"
        "      baseline:\n"
        "        translation_std_mm: 0.0\n"
        "        rotation_std_deg: 0.0\n",
        encoding="utf-8",
    )
    c = _controller(mode, resolution)
    c.load_model(config)

    assert c.use_structural_matching is expected_structural
    assert c.grasp_planning_enabled is expected_structural
    assert grasp_factory.called is expected_structural

    for name in ("ActionPlanner", "SceneInterpreter", "LMPGenerator"):
        assert captures[name].call_args.kwargs[
            "generalized_destination_resolution"
        ] == resolution
    assert captures["ActionPlanner"].call_args.kwargs["perception_mode"] == mode


@pytest.mark.parametrize("resolution", ["bogus", "", "ordinal", "2d"])
def test_load_model_rejects_invalid_generalized_resolution(tmp_path, resolution):
    config = tmp_path / "config.yaml"
    config.write_text(
        f"perception_mode: generalized\n"
        f"generalized_destination_resolution: {resolution!r}\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="Invalid generalized_destination_resolution"):
        _controller().load_model(config)


@pytest.mark.parametrize("resolution", ["ordinal_1d", "structural_2d"])
def test_action_planner_forwards_resolution(tmp_path, monkeypatch, resolution):
    video = tmp_path / "demo.mp4"
    video.write_bytes(b"demo")
    expected = _plan(ordinal=1)
    mocked = MagicMock(return_value=expected)
    monkeypatch.setattr(action_planner_module, "generate_action_plan", mocked)
    planner = ActionPlanner(
        perception_mode="generalized",
        generalized_destination_resolution=resolution.upper(),
    )
    actual = planner.run(
        annotated_video_path=video,
        keyframes=(1, 2),
        track_id_map={1: {"category": "cube"}},
        key_frame_coordinates={"key_frame1": [], "key_frame2": []},
        artifacts_dir=tmp_path / "out",
    )
    assert actual is expected
    assert mocked.call_args.kwargs["generalized_destination_resolution"] == resolution
    assert mocked.call_args.kwargs["perception_mode"] == "generalized"


def test_historical_and_solution_vlm_schemas_remain_separate():
    old_fields = PLAN_SCHEMA["schema"]["properties"]["steps"]["items"]["properties"]
    new_fields = GENERALIZED_PLAN_SCHEMA["schema"]["properties"]["steps"]["items"]["properties"]
    assert "destination_ordinal_from_left" in old_fields
    assert "grasp_instruction" not in old_fields
    assert "destination_ordinal_from_left" not in new_fields
    assert "grasp_instruction" in new_fields


def test_historical_prompt_mentions_ordinal_and_not_structural_matching():
    prompt = build_prompt_ordinal_1d(
        pick_frame=1,
        place_frame=2,
        track_map={
            "1": {"detector_label": "green cube", "category": "cube", "attributes": {}},
            "10": {"detector_label": "storage bin", "category": "bin", "attributes": {}},
            "20": {"detector_label": "storage bin", "category": "bin", "attributes": {}},
        },
        coordinates={
            "1": {"1": [130, 25], "10": [100, 25], "20": [300, 25]},
            "2": {"1": [130, 25], "10": [100, 25], "20": [300, 25]},
        },
    )
    assert "DESTINATION ORDINAL:" in prompt
    assert "from the left" in prompt
    assert "destination_ordinal_from_left" in prompt
    assert "structural matcher" not in prompt.casefold()


# ---------------------------------------------------------------------------
# SceneInterpreter: old ordinal IDs vs solution neutral IDs.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "resolution,expected_ids",
    [
        ("ordinal_1d", (
            "second storage bin from the left", "green cube",
            "first storage bin from the left",
        )),
        ("structural_2d", ("raw_right", "raw_pick", "raw_left")),
    ],
)
def test_scene_interpreter_generalized_id_strategy(
    tmp_path, resolution, expected_ids
):
    overlay = tmp_path / "overlay.jpg"
    overlay.write_bytes(b"mock overlay: only file-existence is checked")
    interpreter = SceneInterpreter(
        perception_mode="generalized",
        generalized_destination_resolution=resolution,
    )
    scene = interpreter.run(
        perception_result=ScenePerceptionResult(
            raw_scene=_raw_scene(), overlay_image_path=overlay
        ),
        artifacts_dir=tmp_path / "scene",
    )
    assert tuple(obj.object_id for obj in scene.objects) == expected_ids
    assert tuple(obj.category for obj in scene.objects) == ("bin", "cube", "bin")
    result_file = tmp_path / "scene" / "scene_interpretation.json"
    assert result_file.is_file()
    names = json.loads(result_file.read_text())["objects"]
    assert tuple(entry["semantic_name"] for entry in names) == expected_ids


@pytest.mark.parametrize("prior_task", ["pick_and_place", "nut_assembly"])
def test_prior_guided_scene_interpreter_still_uses_semantic_assigner(
    tmp_path, monkeypatch, prior_task
):
    overlay = tmp_path / "overlay.jpg"
    overlay.write_bytes(b"mock")
    interpreter = SceneInterpreter(
        perception_mode="prior_guided", prior_guided_task=prior_task,
        generalized_destination_resolution="ordinal_1d",
    )
    assigned = MagicMock(return_value={
        "raw_right": "second storage bin",
        "raw_pick": "green cube",
        "raw_left": "first storage bin",
    })
    monkeypatch.setattr(interpreter, "_assign_semantic_names", assigned)
    result = interpreter.run(
        ScenePerceptionResult(raw_scene=_raw_scene(), overlay_image_path=overlay)
    )
    assigned.assert_called_once()
    assert tuple(obj.object_id for obj in result.objects) == (
        "second storage bin", "green cube", "first storage bin"
    )


# ---------------------------------------------------------------------------
# LMP: 1D resolver, 2D replicability and untouched prior-guided branch.
# ---------------------------------------------------------------------------

def _wrapper(scene=None):
    return LMPSceneWrapper(
        scene_state=scene or _scene(),
        workspace_bottom_left=(0.0, 0.0),
        workspace_top_right=(1.0, 1.0),
    )


@pytest.mark.parametrize(
    "ordinal,expected_destination",
    [(1, "bin_left"), (2, "bin_right")],
)
def test_ordinal_resolver_uses_image_x_not_detection_order(
    ordinal, expected_destination
):
    assert _wrapper().resolve_action_step(
        _step(ordinal=ordinal)
    ) == ("cube", expected_destination)


@pytest.mark.parametrize(
    "ordinal", [0, 3]
)
def test_ordinal_resolver_rejects_out_of_range(ordinal):
    with pytest.raises(ValueError, match="Invalid destination ordinal"):
        _wrapper().resolve_action_step(_step(ordinal=ordinal))


def test_ordinal_resolver_rejects_ambiguous_x():
    scene = SceneState(objects=(
        _scene_object("cube", "green cube", "cube", 10),
        _scene_object("a", "storage bin", "bin", 200),
        _scene_object("b", "storage bin", "bin", 200),
    ))
    with pytest.raises(ValueError, match="Ambiguous destination ordering"):
        _wrapper(scene).resolve_action_step(_step(ordinal=1))


@pytest.mark.parametrize("resolution", ["ordinal_1d", "structural_2d"])
def test_lmp_generator_generalized_instruction_routing(
    monkeypatch, resolution
):
    monkeypatch.setattr(lmp_module, "OpenAI", lambda: object())
    generator = LMPGenerator(
        perception_mode="generalized",
        generalized_destination_resolution=resolution,
    )
    captured = {}

    class FakeLMP:
        exec_hist = "reach('cube')"

        def __call__(self, instruction, *, context):
            captured["instruction"] = instruction
            captured["context"] = context
            captured["wrapper"].reach("cube")

    def fake_setup(*, wrapper, scene_state, task_type):
        captured["wrapper"] = wrapper
        return FakeLMP()

    monkeypatch.setattr(generator, "_setup_lmp", fake_setup)
    matching = ReplicabilityResult(
        replicable=True,
        resolved_targets=(ResolvedActionTargets(
            action_step_index=0,
            runtime_pick_object_id="cube",
            runtime_place_object_id="bin_right",
        ),),
        failure_reasons=(),
    )
    result = generator.run(
        action_plan=_plan(ordinal=1 if resolution == "ordinal_1d" else None),
        scene_state=_scene(),
        workspace_bottom_left=(0.0, 0.0),
        workspace_top_right=(1.0, 1.0),
        replicability_result=(None if resolution == "ordinal_1d" else matching),
    )
    assert len(result.steps) == 1
    assert "Pick 'cube'" in captured["instruction"]
    # 1D -> first bin from left; 2D -> target supplied by matching.
    expected_target = "bin_left" if resolution == "ordinal_1d" else "bin_right"
    assert repr(expected_target) in captured["instruction"]


def test_lmp_solution_rejects_missing_replicability(monkeypatch):
    monkeypatch.setattr(lmp_module, "OpenAI", lambda: object())
    generator = LMPGenerator(
        perception_mode="generalized",
        generalized_destination_resolution="structural_2d",
    )
    # LMP setup is mocked so the failure specifically tests routing.
    monkeypatch.setattr(generator, "_setup_lmp", lambda **kwargs: object())
    with pytest.raises(ValueError, match="requires a ReplicabilityResult"):
        generator.run(
            action_plan=_plan(), scene_state=_scene(),
            workspace_bottom_left=(0.0, 0.0),
            workspace_top_right=(1.0, 1.0),
        )


def test_lmp_prior_guided_uses_natural_language_unchanged(monkeypatch):
    monkeypatch.setattr(lmp_module, "OpenAI", lambda: object())
    generator = LMPGenerator(
        perception_mode="prior_guided",
        generalized_destination_resolution="ordinal_1d",
    )
    calls = []

    def setup(*, wrapper: LMPSceneWrapper, scene_state, task_type):
        # Bind wrapper to the returned LMP instance without external calls.
        class BoundFakeLMP:
            exec_hist = "reach('cube')"

            def __call__(self, instruction, *, context):
                calls.append(instruction)
                wrapper.reach("cube")

        return BoundFakeLMP()

    monkeypatch.setattr(generator, "_setup_lmp", setup)
    generator.run(
        action_plan=_plan(ordinal=1), scene_state=_scene(),
        workspace_bottom_left=(0.0, 0.0),
        workspace_top_right=(1.0, 1.0),
    )
    assert calls == [_plan(ordinal=1).natural_language_plan]


# ---------------------------------------------------------------------------
# Precomputed: accept the matching strategy and refuse incompatible plans.
# These tests intentionally fail against the old remote code; they require
# the local compatibility check shown in the user's last diff.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "mode,resolution,ordinal,grasp,expected_ordinal",
    [
        ("generalized", "ordinal_1d", 2, None, 2),
        ("generalized", "structural_2d", None, "Pick up the green cube.", None),
        ("prior_guided", "ordinal_1d", 2, None, 2),
    ],
)
def test_precomputed_accepts_matching_format(
    tmp_path, mode, resolution, ordinal, grasp, expected_ordinal
):
    c = _controller(mode, resolution)
    path = _plan_file(tmp_path / "plan.json", ordinal=ordinal, grasp_instruction=grasp)
    plan = c._load_precomputed_action_plan(path)
    assert plan.status == "completed"
    assert plan.steps[0].destination_ordinal_from_left == expected_ordinal
    if resolution == "structural_2d" and mode == "generalized":
        assert plan.steps[0].grasp_instruction == grasp


@pytest.mark.parametrize(
    "mode,resolution,ordinal,grasp,part",
    [
        ("generalized", "structural_2d", 1, None, "structural_2d"),
        ("generalized", "structural_2d", None, None, "grasp_instruction"),
        ("generalized", "structural_2d", None, "   ", "grasp_instruction"),
        ("generalized", "ordinal_1d", None, None, "ordinal_1d"),
        ("generalized", "ordinal_1d", 1, "Pick it.", "ordinal_1d"),
        ("generalized", "ordinal_1d", 0, None, "positive integer"),
        ("prior_guided", "ordinal_1d", None, None, "missing destination_ordinal"),
    ],
)
def test_precomputed_rejects_wrong_format(
    tmp_path, mode, resolution, ordinal, grasp, part
):
    c = _controller(mode, resolution)
    path = _plan_file(tmp_path / "plan.json", ordinal=ordinal, grasp_instruction=grasp)
    with pytest.raises(ValueError, match=part):
        c._load_precomputed_action_plan(path)


# ---------------------------------------------------------------------------
# Full demonstration, Precomputed and runtime t=0: routed stages by strategy.
# ---------------------------------------------------------------------------

def _stage(result=None):
    return SimpleNamespace(run=MagicMock(return_value=result))


def _video(tmp_path):
    video = tmp_path / "demo.mp4"
    video.write_bytes(b"test")
    return video


def _install_demo_stages(c, video):
    c.keyframe_selector = _stage(SimpleNamespace(keyframes=(1, 2)))
    c.scene_perceiver = SimpleNamespace(
        grounding_model=object(), sam=object(),
        adopt_visual_prompting_models=MagicMock(),
    )
    c.visual_prompter = _stage(SimpleNamespace(
        annotated_video_path=video,
        track_id_map={1: {"category": "cube"}},
        key_frame_coordinates={"key_frame1": [], "key_frame2": []},
    ))
    demo_scene = object()
    c.demo_structured_scene_builder = _stage(demo_scene)
    is_solution = c.use_structural_matching
    c.action_planner = _stage(_plan(
        ordinal=None if is_solution else 1,
        grasp_instruction="Pick up the green cube." if is_solution else "",
    ))
    return demo_scene


@pytest.mark.parametrize(
    "mode,resolution,structural",
    [
        ("generalized", "ordinal_1d", False),
        ("generalized", "structural_2d", True),
        ("prior_guided", "ordinal_1d", False),
    ],
)
def test_full_demo_runs_only_expected_stages(
    tmp_path, no_pipeline_side_effects, mode, resolution, structural
):
    c = _controller(mode, resolution)
    video = _video(tmp_path)
    expected_demo_scene = _install_demo_stages(c, video)
    c.load_command(
        demo_path=str(video),
        task_id="test",
        artifacts_dir=tmp_path / "artifacts",
    )
    assert c.execution_status == "plan_ready"
    c.keyframe_selector.run.assert_called_once()
    c.visual_prompter.run.assert_called_once()
    c.action_planner.run.assert_called_once()
    assert c.demo_structured_scene_builder.run.called is structural
    assert c.demo_structured_scene is (expected_demo_scene if structural else None)


@pytest.mark.parametrize(
    "mode,resolution,need_scene",
    [
        ("generalized", "ordinal_1d", False),
        ("generalized", "structural_2d", True),
        ("prior_guided", "ordinal_1d", False),
    ],
)
def test_load_command_precomputed_demonstration_requirements(
    tmp_path, no_pipeline_side_effects, mode, resolution, need_scene
):
    c = _controller(mode, resolution)
    video = _video(tmp_path)
    action_path = _plan_file(
        tmp_path / "plan.json",
        ordinal=(None if need_scene else 1),
        grasp_instruction=("Pick up the green cube." if need_scene else None),
    )
    demo_scene = object()
    c._load_precomputed_demo_structured_scene = MagicMock(return_value=demo_scene)

    if need_scene:
        with pytest.raises(ValueError, match="requires precomputed_demo_structured_scene_path"):
            c.load_command(
                demo_path=str(video), task_id="test",
                artifacts_dir=tmp_path / "failed_artifacts",
                precomputed_action_plan_path=str(action_path),
            )
        assert c.execution_status == "failed"

    kwargs = {}
    if need_scene:
        kwargs["precomputed_demo_structured_scene_path"] = "dummy_scene.json"
    c.load_command(
        demo_path=str(video), task_id="test",
        artifacts_dir=tmp_path / "artifacts",
        precomputed_action_plan_path=str(action_path),
        **kwargs,
    )
    assert c.execution_status == "plan_ready"
    assert c.action_plan is not None
    assert c._load_precomputed_demo_structured_scene.called is need_scene
    assert c.demo_structured_scene is (demo_scene if need_scene else None)


@pytest.mark.parametrize(
    "mode,resolution,structural",
    [
        ("generalized", "ordinal_1d", False),
        ("generalized", "structural_2d", True),
        ("prior_guided", "ordinal_1d", False),
    ],
)
def test_runtime_t0_routes_structural_stages_and_lmp(
    tmp_path, no_pipeline_side_effects, mode, resolution, structural
):
    c = _controller(mode, resolution)
    c.action_plan = _plan(ordinal=1 if not structural else None)
    c.artifacts_dir = tmp_path
    c.demo_structured_scene = object() if structural else None
    perception_result = object()
    interpreted_scene = _scene()
    runtime_scene = object()
    structural_matching_result = object()
    repl = ReplicabilityResult(
        replicable=True,
        resolved_targets=(ResolvedActionTargets(
            action_step_index=0,
            runtime_pick_object_id="cube",
            runtime_place_object_id="bin_left",
        ),),
        failure_reasons=(),
    )
    primitive_plan = PrimitivePlan(
        steps=(PrimitiveStep(name="reach", arguments={"target": "cube"}),),
        source_code="reach('cube')",
    )
    c.scene_perceiver = _stage(perception_result)
    c.scene_interpreter = _stage(interpreted_scene)
    c.runtime_structured_scene_builder = _stage(runtime_scene)
    c.structural_matcher = _stage(structural_matching_result)
    c.replicability_checker = _stage(repl)
    c.lmp_generator = _stage(primitive_plan)

    runtime_input = {
        "rgb": object(), "depth": object(), "camera_info": object(),
        "base_to_table_transform": object(),
    }
    assert c.inference(runtime_input, t=0) is None
    assert c.execution_status == "plan_ready"
    c.scene_perceiver.run.assert_called_once()
    c.scene_interpreter.run.assert_called_once()
    assert c.runtime_structured_scene_builder.run.called is structural
    assert c.structural_matcher.run.called is structural
    assert c.replicability_checker.run.called is structural
    c.lmp_generator.run.assert_called_once()
    assert c.lmp_generator.run.call_args.kwargs["replicability_result"] is (
        repl if structural else None
    )


def test_runtime_solution_rejects_nonreplicable_scene(
    tmp_path, no_pipeline_side_effects
):
    c = _controller("generalized", "structural_2d")
    c.action_plan = _plan()
    c.artifacts_dir = tmp_path
    c.demo_structured_scene = object()
    c.scene_perceiver = _stage(object())
    c.scene_interpreter = _stage(_scene())
    c.runtime_structured_scene_builder = _stage(object())
    c.structural_matcher = _stage(object())
    c.replicability_checker = _stage(ReplicabilityResult(
        replicable=False, resolved_targets=(),
        failure_reasons=("destination ambiguous",),
    ))
    c.lmp_generator = _stage(PrimitivePlan(steps=(), source_code=""))
    with pytest.raises(RuntimeError, match="not replicable"):
        c.inference({
            "rgb": object(), "depth": object(), "camera_info": object(),
            "base_to_table_transform": object(),
        }, t=0)
    assert c.execution_status == "failed"
    c.lmp_generator.run.assert_not_called()


# ---------------------------------------------------------------------------
# VLM validation: exact historical 1D ordinal checks and current 2D checks.
# ---------------------------------------------------------------------------

def _vlm_tracking_evidence():
    track_map = {
        "1": {"detector_label": "green cube", "category": "cube", "attributes": {}},
        "10": {"detector_label": "storage bin", "category": "bin", "attributes": {}},
        "20": {"detector_label": "storage bin", "category": "bin", "attributes": {}},
    }
    positions = {
        "1": {"1": [130, 25], "10": [100, 25], "20": [300, 25]},
        "2": {"1": [130, 25], "10": [100, 25], "20": [300, 25]},
    }
    return track_map, positions


def _vlm_step(*, ordinal=1, grasp_instruction=None):
    step = {
        "pick_keyframe": 1,
        "place_keyframe": 2,
        "picked_track_id": 1,
        "picked_detector_label": "green cube",
        "picked_category": "cube",
        "picked_color": "green",
        "destination_track_id": 10,
        "destination_category": "bin",
        "relation": "in",
        "action": "Pick the green cube and place it in the storage bin.",
    }
    if ordinal is not None:
        step["destination_ordinal_from_left"] = ordinal
    if grasp_instruction is not None:
        step["grasp_instruction"] = grasp_instruction
    return step


def _vlm_plan(step, *, task_type="pick_and_place"):
    return {
        "steps": [step],
        "status": "completed",
        "task_type": task_type,
        "ambiguities": [],
    }


@pytest.mark.parametrize(
    "order,expected_ordinal",
    [("left_to_right", 1), ("right_to_left", 2)],
)
def test_historical_vlm_validator_enforces_demo_image_order(
    order, expected_ordinal
):
    tracks, positions = _vlm_tracking_evidence()
    validate_plan_ordinal_1d(
        plan=_vlm_plan(_vlm_step(ordinal=expected_ordinal)),
        pick_frame=1,
        place_frame=2,
        track_map=tracks,
        coordinates=positions,
        demonstration_bin_order=order,
        perception_mode="generalized",
    )
    with pytest.raises(ValueError, match="Destination ordinal mismatch"):
        validate_plan_ordinal_1d(
            plan=_vlm_plan(_vlm_step(ordinal=3 - expected_ordinal)),
            pick_frame=1,
            place_frame=2,
            track_map=tracks,
            coordinates=positions,
            demonstration_bin_order=order,
            perception_mode="generalized",
        )


def test_solution_vlm_validator_requires_grasp_instruction():
    tracks, positions = _vlm_tracking_evidence()
    kwargs = dict(
        pick_frame=1, place_frame=2,
        track_map=tracks, coordinates=positions,
        demonstration_bin_order="left_to_right",
        perception_mode="generalized",
    )
    validate_plan(
        plan=_vlm_plan(_vlm_step(ordinal=None, grasp_instruction="Pick up the green cube.")),
        **kwargs,
    )
    with pytest.raises(ValueError, match="empty grasp_instruction"):
        validate_plan(plan=_vlm_plan(_vlm_step(ordinal=None)), **kwargs)


def test_prior_guided_vlm_validation_is_unaffected():
    tracks, positions = _vlm_tracking_evidence()
    validate_plan(
        plan=_vlm_plan(_vlm_step(ordinal=1)),
        pick_frame=1, place_frame=2,
        track_map=tracks, coordinates=positions,
        demonstration_bin_order="left_to_right",
        perception_mode="prior_guided",
        prior_guided_task="pick_and_place",
    )


def test_prior_guided_nut_assembly_vlm_validation_is_unaffected():
    tracks = {
        "1": {"detector_label": "gray ring", "category": "ring", "attributes": {}},
        "10": {"detector_label": "wooden peg", "category": "peg", "attributes": {}},
    }
    positions = {"1": {"1": [10, 20], "10": [100, 20]},
                 "2": {"1": [10, 20], "10": [100, 20]}}
    step = {
        **_vlm_step(ordinal=1),
        "picked_detector_label": "gray ring",
        "picked_category": "ring",
        "picked_color": "gray",
        "destination_category": "peg",
        "relation": "on",
    }
    validate_plan(
        plan=_vlm_plan(step, task_type="nut_assembly"),
        pick_frame=1, place_frame=2,
        track_map=tracks, coordinates=positions,
        demonstration_bin_order="left_to_right",
        perception_mode="prior_guided",
        prior_guided_task="nut_assembly",
    )


# ---------------------------------------------------------------------------
# Nut Assembly: same 1D/2D LMP routing, but with the assembly instruction.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("resolution", ["ordinal_1d", "structural_2d"])
def test_lmp_nut_assembly_uses_correct_destination(
    monkeypatch, resolution
):
    monkeypatch.setattr(lmp_module, "OpenAI", lambda: object())
    generator = LMPGenerator(
        perception_mode="generalized",
        generalized_destination_resolution=resolution,
    )
    scene = SceneState(objects=(
        _scene_object("ring", "gray ring", "ring", 120),
        _scene_object("peg_right", "wooden peg", "peg", 300),
        _scene_object("peg_left", "wooden peg", "peg", 100),
    ))
    step = replace(
        _step(ordinal=1 if resolution == "ordinal_1d" else None),
        picked_category="ring",
        picked_color="gray",
        picked_detector_label="gray ring",
        destination_category="peg",
        relation="on",
        action="Pick the gray ring and assemble it onto the wooden peg.",
    )
    plan = ActionPlanningResult(
        steps=(step,),
        status="completed",
        ambiguities=(),
        natural_language_plan=step.action,
        task_type=TaskType.NUT_ASSEMBLY,
    )
    captured = []

    def setup(*, wrapper, scene_state, task_type):
        class FakeLMP:
            exec_hist = "reach('ring')"

            def __call__(self, instruction, *, context):
                captured.append(instruction)
                wrapper.reach("ring")

        return FakeLMP()

    monkeypatch.setattr(generator, "_setup_lmp", setup)
    repl = ReplicabilityResult(
        replicable=True,
        resolved_targets=(ResolvedActionTargets(
            action_step_index=0,
            runtime_pick_object_id="ring",
            runtime_place_object_id="peg_right",
        ),),
        failure_reasons=(),
    )
    generator.run(
        action_plan=plan,
        scene_state=scene,
        workspace_bottom_left=(0.0, 0.0),
        workspace_top_right=(1.0, 1.0),
        replicability_result=None if resolution == "ordinal_1d" else repl,
    )
    assert len(captured) == 1
    assert "assemble it onto" in captured[0]
    assert repr("peg_left" if resolution == "ordinal_1d" else "peg_right") in captured[0]


# ---------------------------------------------------------------------------
# Cross-strategy Precomputed should fail in load_command before t=0.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "resolution,ordinal,grasp,expected_error",
    [
        ("ordinal_1d", None, "Pick up the green cube.", "ordinal_1d"),
        ("structural_2d", 1, None, "structural_2d"),
    ],
)
def test_load_command_refuses_incompatible_precomputed_before_runtime(
    tmp_path, no_pipeline_side_effects,
    resolution, ordinal, grasp, expected_error,
):
    c = _controller("generalized", resolution)
    video = _video(tmp_path)
    path = _plan_file(tmp_path / "bad_plan.json", ordinal=ordinal,
                      grasp_instruction=grasp)
    c.scene_perceiver = _stage(object())
    c.lmp_generator = _stage(PrimitivePlan(steps=(), source_code=""))
    c._load_precomputed_demo_structured_scene = MagicMock()

    with pytest.raises(ValueError, match=expected_error):
        c.load_command(
            demo_path=str(video), task_id="test",
            artifacts_dir=tmp_path / "artifacts",
            precomputed_action_plan_path=str(path),
            precomputed_demo_structured_scene_path="dummy_scene.json",
        )
    assert c.execution_status == "failed"
    c.scene_perceiver.run.assert_not_called()
    c.lmp_generator.run.assert_not_called()
    c._load_precomputed_demo_structured_scene.assert_not_called()


# ---------------------------------------------------------------------------
# Demo Only / launchers: static safety guards, without importing ROS nodes.
# Unlike the mock-based tests above, these inspect configuration gates only.
# ---------------------------------------------------------------------------

def test_demo_only_uses_structural_artifact_gate_not_all_generalized_modes():
    import ast

    # .../models/seedo_controller/tests/unit/test_*.py -> ai_controller package.
    node_file = Path(__file__).resolve().parents[4] / "ai_controller_node.py"
    parsed = ast.parse(node_file.read_text(encoding="utf-8"))
    matching_if_nodes = [
        node for node in ast.walk(parsed)
        if isinstance(node, ast.If)
        and isinstance(node.test, ast.Attribute)
        and node.test.attr == "use_structural_matching"
    ]
    assert matching_if_nodes, (
        "Demo Only must gate the structured scene artifact on "
        "self.controller.use_structural_matching"
    )
    assert any(
        isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == "append"
        and isinstance(call.func.value, ast.Name)
        and call.func.value.id == "required_files"
        for stmt in matching_if_nodes for call in ast.walk(stmt)
    ), "Demo Only did not conditionally add required structured scene artifact"


@pytest.mark.parametrize("filename", [
    "run_seedo_real_test.sh", "run_seedo_isaac_test.sh",
])
def test_launchers_gate_precomputed_demo_scene_by_strategy(filename):
    source = (
        Path(__file__).resolve().parents[2]
        / "launch" / filename
    ).read_text(encoding="utf-8")
    assert 'USE_STRUCTURAL_MATCHING=false' in source
    assert 'if [[ "${USE_STRUCTURAL_MATCHING}" == true ]]; then' in source
    assert 'GENERALIZED_DESTINATION_RESOLUTION' in source
