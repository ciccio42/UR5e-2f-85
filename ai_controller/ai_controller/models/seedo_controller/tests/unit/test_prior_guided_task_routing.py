from __future__ import annotations

import copy
import inspect
import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import yaml

import track_objects

from results import (
    RawSceneObject,
    RawSceneState,
)

from vlm import (
    build_prompt,
    validate_plan,
)

from ai_controller.models.seedo_controller import (
    scene_interpreter as scene_interpreter_module,
)
from ai_controller.models.seedo_controller import (
    scene_perceiver as scene_perceiver_module,
)
from ai_controller.models.seedo_controller import (
    seedo_controller as seedo_controller_module,
)
from ai_controller.models.seedo_controller.action_planner import (
    ActionPlanner,
)
from ai_controller.models.seedo_controller.scene_interpreter import (
    SceneInterpreter,
)
from ai_controller.models.seedo_controller.scene_interpreter_prompts import (
    PRIOR_GUIDED_NUT_ASSEMBLY_SCENE_INTERPRETER_SYSTEM_PROMPT,
    PRIOR_GUIDED_PICK_AND_PLACE_SCENE_INTERPRETER_SYSTEM_PROMPT,
)
from ai_controller.models.seedo_controller.seedo_controller import (
    SeeDoController,
)


# =====================================================================
# Helpers
# =====================================================================


def _repo_root() -> Path:
    """
    Find the UR5e-2f-85 repository root without assuming the absolute path.
    """

    current = Path(__file__).resolve()

    for parent in current.parents:
        expected = (
            parent
            / "ai_controller"
            / "ai_controller"
            / "models"
            / "seedo_controller"
        )

        if expected.is_dir():
            return parent

    raise RuntimeError(
        "Could not locate the UR5e-2f-85 repository root."
    )


def _load_yaml(relative_path: str) -> dict:
    path = _repo_root() / relative_path

    if not path.is_file():
        raise FileNotFoundError(
            f"Expected configuration file does not exist: {path}"
        )

    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        return yaml.safe_load(stream) or {}


def _write_yaml(
    path: Path,
    payload: dict,
) -> Path:
    path.write_text(
        yaml.safe_dump(
            payload,
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    return path


def _minimal_controller_config(
    *,
    perception_mode: str,
    prior_guided_task=None,
    detector_labels=None,
) -> dict:
    if detector_labels is None:
        detector_labels = [
            "cube",
            "storage bin",
        ]

    return {
        "perception_mode": perception_mode,
        "prior_guided_task": prior_guided_task,
        "grounding_dino": {
            "crop": {
                "top_px": 80,
                "bottom_px": 0,
                "left_px": 0,
                "right_px": 0,
            },
        },
        "keyframe_selector": {
            "mode": "hand_velocity",
            "sample_stride": 5,
            "model": "test-model",
            "gaussian_sigma": 5.0,
            "prominence": 0.8,
            "expected_keyframes": 2,
            "save_preview": False,
        },
        "visual_prompter": {},
        "action_planner": {
            "model": "test-model",
            "demonstration_bin_order": "left_to_right",
        },
        "structured_scene": {
            "directions": 4,
        },
        "scene_perceiver": {
            "camera_name": "zed_front",
            "detector_labels": list(
                detector_labels
            ),
            "workspace_bottom_left": [
                0.0,
                0.0,
            ],
            "workspace_top_right": [
                1.0,
                1.0,
            ],
            "camera_pose_noise": {
                "level": "baseline",
                "levels": {
                    "baseline": {
                        "translation_std_mm": 0.0,
                        "rotation_std_deg": 0.0,
                    },
                },
            },
        },
        "scene_interpreter": {
            "model": "test-model",
        },
        "lmp_generator": {
            "model": "test-model",
        },
        "grasp_planner": {
            "enabled": False,
        },
        "post_grasp_refinement": {
            "enabled": False,
        },
        "motion_layer": {},
    }


def _patch_controller_components(
    monkeypatch,
):
    """
    Replace all heavy model/pipeline constructors used by load_model().

    This lets the test inspect constructor kwargs without loading
    GroundingDINO, SAM, OpenAI, MoveIt, or robot-side components.
    """

    calls: dict[str, list[dict]] = {}

    def factory(name):
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

    component_names = (
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

    for name in component_names:
        monkeypatch.setattr(
            seedo_controller_module,
            name,
            factory(name),
        )

    return calls


def _controller_from_config(
    tmp_path,
    monkeypatch,
    config: dict,
):
    calls = _patch_controller_components(
        monkeypatch
    )

    config_path = _write_yaml(
        tmp_path / "config.yaml",
        config,
    )

    controller = SeeDoController.__new__(
        SeeDoController
    )

    controller.load_model(
        config_path
    )

    return controller, calls


class _FakeVideoCapture:
    """
    Minimal cv2.VideoCapture replacement used by get_object_list().

    It exposes one synthetic RGB frame and then stops.
    """

    def __init__(
        self,
        *_args,
        **_kwargs,
    ):
        self._read_count = 0

    def isOpened(self):
        return self._read_count == 0

    def read(self):
        if self._read_count != 0:
            return False, None

        self._read_count += 1

        return (
            True,
            np.zeros(
                (
                    32,
                    32,
                    3,
                ),
                dtype=np.uint8,
            ),
        )

    def release(self):
        return None


def _flatten_prompt_text(value) -> list[str]:
    """
    Extract textual prompt content while ignoring embedded image payloads.
    """

    if isinstance(
        value,
        str,
    ):
        return [
            value,
        ]

    if isinstance(
        value,
        (list, tuple),
    ):
        result = []

        for item in value:
            result.extend(
                _flatten_prompt_text(
                    item
                )
            )

        return result

    if isinstance(
        value,
        dict,
    ):
        result = []

        for key in (
            "text",
            "content",
        ):
            if key in value:
                result.extend(
                    _flatten_prompt_text(
                        value[key]
                    )
                )

        return result

    return []


def _capture_visual_prompt(
    monkeypatch,
    *,
    prior_guided_task: str,
) -> tuple[str, bool]:
    captured = {}

    monkeypatch.setattr(
        track_objects.cv2,
        "VideoCapture",
        _FakeVideoCapture,
    )

    def fake_call_openai_api(
        messages,
        client,
        json_mode=False,
    ):
        captured[
            "messages"
        ] = messages

        captured[
            "json_mode"
        ] = json_mode

        return (
            "Number: 4\n"
            "Objects: gray ring, wooden peg, "
            "wooden peg, wooden peg"
        )

    monkeypatch.setattr(
        track_objects,
        "call_openai_api",
        fake_call_openai_api,
    )

    track_objects.get_object_list(
        "unused.mp4",
        client=object(),
        perception_mode="prior_guided",
        prior_guided_task=prior_guided_task,
    )

    prompt_text = "\n".join(
        _flatten_prompt_text(
            captured["messages"]
        )
    )

    return (
        prompt_text,
        bool(
            captured[
                "json_mode"
            ]
        ),
    )


def _pick_place_tracking_evidence():
    return {
        "track_map": {
            "1": {
                "detector_label": "green cube",
            },
            "2": {
                "detector_label": "storage bin",
            },
            "3": {
                "detector_label": "storage bin",
            },
        },
        "coordinates": {
            "10": {
                "1": [
                    100,
                    100,
                ],
                "2": [
                    250,
                    100,
                ],
                "3": [
                    400,
                    100,
                ],
            },
            "20": {
                "1": [
                    245,
                    100,
                ],
                "2": [
                    250,
                    100,
                ],
                "3": [
                    400,
                    100,
                ],
            },
        },
    }


def _nut_tracking_evidence():
    return {
        "track_map": {
            "1": {
                "detector_label": "yellow ring",
            },
            "2": {
                "detector_label": "wooden peg",
            },
            "3": {
                "detector_label": "wooden peg",
            },
            "4": {
                "detector_label": "wooden peg",
            },
        },
        "coordinates": {
            "10": {
                "1": [
                    100,
                    100,
                ],
                "2": [
                    200,
                    100,
                ],
                "3": [
                    350,
                    100,
                ],
                "4": [
                    500,
                    100,
                ],
            },
            "20": {
                "1": [
                    345,
                    100,
                ],
                "2": [
                    200,
                    100,
                ],
                "3": [
                    350,
                    100,
                ],
                "4": [
                    500,
                    100,
                ],
            },
        },
    }


def _valid_nut_plan():
    return {
        "steps": [
            {
                "pick_keyframe": 10,
                "place_keyframe": 20,
                "picked_track_id": 1,
                "picked_detector_label": "yellow ring",
                "picked_category": "ring",
                "picked_color": "yellow",
                "destination_track_id": 3,
                "destination_category": "peg",
                "destination_ordinal_from_left": 2,
                "relation": "on",
                "action": (
                    "Pick the yellow ring and assemble it onto "
                    "the second peg from the left."
                ),
            },
        ],
        "status": "completed",
        "task_type": "nut_assembly",
        "ambiguities": [],
    }


def _raw_object(
    object_id: str,
    *,
    label: str,
    pixel_coordinates,
) -> RawSceneObject:
    return RawSceneObject(
        object_id=object_id,
        label=label,
        pixel_coordinates=(
            int(
                pixel_coordinates[
                    0
                ]
            ),
            int(
                pixel_coordinates[
                    1
                ]
            ),
        ),
        position_camera=(
            0.1,
            0.2,
            0.3,
        ),
        position_base=(
            0.4,
            0.5,
            0.6,
        ),
        mask=None,
        confidence=0.9,
        category=None,
        attributes={},
    )


def _write_overlay(
    tmp_path,
) -> Path:
    overlay_path = (
        tmp_path
        / "scene_interpreter_overlay.jpg"
    )

    image = np.zeros(
        (
            64,
            64,
            3,
        ),
        dtype=np.uint8,
    )

    assert cv2.imwrite(
        str(
            overlay_path
        ),
        image,
    )

    return overlay_path


def _install_fake_scene_interpreter_openai(
    monkeypatch,
    *,
    response_payload: dict,
    captured: dict,
):
    class FakeCompletions:
        def create(
            self,
            **kwargs,
        ):
            captured.update(
                kwargs
            )

            return SimpleNamespace(
                choices=[
                    SimpleNamespace(
                        message=SimpleNamespace(
                            content=json.dumps(
                                response_payload
                            ),
                        )
                    ),
                ]
            )

    class FakeChat:
        def __init__(
            self,
        ):
            self.completions = (
                FakeCompletions()
            )

    class FakeOpenAI:
        def __init__(
            self,
        ):
            self.chat = FakeChat()

    monkeypatch.setattr(
        scene_interpreter_module,
        "OpenAI",
        FakeOpenAI,
    )


# =====================================================================
# Configuration audit
# =====================================================================


def test_prior_guided_yaml_configs_select_different_task_domains():
    pick_place_config = _load_yaml(
        "ai_controller/ai_controller/models/"
        "seedo_controller/config/experiments/"
        "baseline_prior_guided_pick_place.yaml"
    )

    nut_config = _load_yaml(
        "ai_controller/ai_controller/models/"
        "seedo_controller/config/experiments/"
        "baseline_prior_guided_nut_assembly.yaml"
    )

    assert (
        pick_place_config[
            "perception_mode"
        ]
        == "prior_guided"
    )

    assert (
        pick_place_config[
            "prior_guided_task"
        ]
        == "pick_and_place"
    )

    assert (
        nut_config[
            "perception_mode"
        ]
        == "prior_guided"
    )

    assert (
        nut_config[
            "prior_guided_task"
        ]
        == "nut_assembly"
    )

    assert (
        nut_config[
            "scene_perceiver"
        ][
            "detector_labels"
        ]
        == [
            "ring",
            "wooden peg",
        ]
    )


@pytest.mark.parametrize(
    "relative_path",
    [
        (
            "ai_controller/ai_controller/models/"
            "seedo_controller/config/seedo_controller.yaml"
        ),
        (
            "ai_controller/ai_controller/models/"
            "seedo_controller/config/experiments/"
            "baseline_generalized_multitask.yaml"
        ),
    ],
)
def test_generalized_yaml_configs_do_not_select_prior_guided_task(
    relative_path,
):
    config = _load_yaml(
        relative_path
    )

    assert (
        config[
            "perception_mode"
        ]
        == "generalized"
    )

    assert (
        config.get(
            "prior_guided_task"
        )
        is None
    )


# =====================================================================
# SeeDoController routing audit
# =====================================================================


@pytest.mark.parametrize(
    (
        "task",
        "detector_labels",
    ),
    [
        (
            "pick_and_place",
            [
                "cube",
                "storage bin",
            ],
        ),
        (
            "nut_assembly",
            [
                "ring",
                "wooden peg",
            ],
        ),
    ],
)
def test_controller_propagates_prior_guided_task_to_all_four_stages(
    tmp_path,
    monkeypatch,
    task,
    detector_labels,
):
    controller, calls = _controller_from_config(
        tmp_path,
        monkeypatch,
        _minimal_controller_config(
            perception_mode="prior_guided",
            prior_guided_task=task,
            detector_labels=detector_labels,
        ),
    )

    assert (
        controller.prior_guided_task
        == task
    )

    for component in (
        "VisualPrompter",
        "ScenePerceiver",
        "ActionPlanner",
        "SceneInterpreter",
    ):
        assert (
            calls[
                component
            ][
                0
            ][
                "kwargs"
            ][
                "prior_guided_task"
            ]
            == task
        )


def test_controller_rejects_missing_prior_guided_task(
    tmp_path,
):
    config = _minimal_controller_config(
        perception_mode="prior_guided",
        prior_guided_task=None,
    )

    config_path = _write_yaml(
        tmp_path / "missing_task.yaml",
        config,
    )

    controller = SeeDoController.__new__(
        SeeDoController
    )

    with pytest.raises(
        ValueError,
        match="prior_guided_task",
    ):
        controller.load_model(
            config_path
        )


def test_controller_rejects_invalid_prior_guided_task(
    tmp_path,
):
    config = _minimal_controller_config(
        perception_mode="prior_guided",
        prior_guided_task="unknown_task",
    )

    config_path = _write_yaml(
        tmp_path / "invalid_task.yaml",
        config,
    )

    controller = SeeDoController.__new__(
        SeeDoController
    )

    with pytest.raises(
        ValueError,
        match="Invalid prior_guided_task",
    ):
        controller.load_model(
            config_path
        )


def test_generalized_controller_ignores_prior_guided_task(
    tmp_path,
    monkeypatch,
):
    controller, calls = _controller_from_config(
        tmp_path,
        monkeypatch,
        _minimal_controller_config(
            perception_mode="generalized",
            prior_guided_task="nut_assembly",
        ),
    )

    assert (
        controller.prior_guided_task
        is None
    )

    for component in (
        "VisualPrompter",
        "ScenePerceiver",
        "ActionPlanner",
        "SceneInterpreter",
    ):
        assert (
            calls[
                component
            ][
                0
            ][
                "kwargs"
            ][
                "prior_guided_task"
            ]
            is None
        )


# =====================================================================
# VisualPrompter / track_objects prompt routing audit
# =====================================================================


def test_visual_prompting_pick_place_prior_prompt_is_preserved(
    monkeypatch,
):
    prompt_text, json_mode = (
        _capture_visual_prompt(
            monkeypatch,
            prior_guided_task=(
                "pick_and_place"
            ),
        )
    )

    assert (
        "colored cubes and storage bins"
        in prompt_text
    )

    assert (
        "'<color> cube'"
        in prompt_text
    )

    assert (
        "red cube"
        in prompt_text
    )

    assert (
        "green cube"
        in prompt_text
    )

    assert (
        "blue cube"
        in prompt_text
    )

    assert (
        "yellow cube"
        in prompt_text
    )

    assert (
        "storage bin"
        in prompt_text
    )

    assert (
        "wooden peg"
        not in prompt_text
    )

    assert (
        "gray ring"
        not in prompt_text
    )

    assert json_mode is False


def test_visual_prompting_nut_prompt_uses_ring_and_wooden_peg(
    monkeypatch,
):
    prompt_text, json_mode = (
        _capture_visual_prompt(
            monkeypatch,
            prior_guided_task=(
                "nut_assembly"
            ),
        )
    )

    assert (
        "colored rings and wooden pegs"
        in prompt_text
    )

    assert (
        "'<color> ring'"
        in prompt_text
    )

    for label in (
        "gray ring",
        "yellow ring",
        "blue ring",
        "wooden peg",
    ):
        assert (
            label
            in prompt_text
        )

    assert (
        "storage bin"
        not in prompt_text
    )

    assert json_mode is False


# =====================================================================
# ScenePerceiver prompt-routing structural audit
# =====================================================================


def test_scene_perceiver_contains_separate_prior_guided_nut_prompt():
    """
    ScenePerceiver builds its prompt inline inside run(), so this audit
    deliberately checks the implementation source rather than executing
    the complete RGB-D / GroundingDINO / SAM runtime pipeline.
    """

    source = inspect.getsource(
        scene_perceiver_module.ScenePerceiver.run
    )

    compact_source = (
        source
        .replace(
            "'",
            '"',
        )
        .replace(
            " ",
            "",
        )
        .replace(
            "\n",
            "",
        )
    )

    assert (
        "prior_guided_messages"
        in source
    )

    assert (
        "prior_guided_nut_assembly_messages"
        in source
    )

    assert (
        'self.prior_guided_task=="pick_and_place"'
        in compact_source
    )

    assert (
        'self.prior_guided_task=="nut_assembly"'
        in compact_source
    )

    for label in (
        "gray ring",
        "yellow ring",
        "blue ring",
        "wooden peg",
    ):
        assert (
            label
            in source
        )


# =====================================================================
# ActionPlanner prompt and validation audit
# =====================================================================


@pytest.mark.parametrize(
    (
        "task",
        "expected",
    ),
    [
        (
            "pick_and_place",
            "pick_and_place",
        ),
        (
            "PICK_AND_PLACE",
            "pick_and_place",
        ),
        (
            " nut_assembly ",
            "nut_assembly",
        ),
    ],
)
def test_action_planner_normalizes_prior_guided_task(
    task,
    expected,
):
    planner = ActionPlanner(
        perception_mode="prior_guided",
        prior_guided_task=task,
    )

    assert (
        planner.prior_guided_task
        == expected
    )


def test_action_planner_rejects_invalid_prior_guided_task():
    with pytest.raises(
        ValueError,
        match="Invalid prior_guided_task",
    ):
        ActionPlanner(
            perception_mode="prior_guided",
            prior_guided_task="unknown",
        )


def test_pick_place_action_prompt_is_preserved():
    evidence = (
        _pick_place_tracking_evidence()
    )

    prompt = build_prompt(
        pick_frame=10,
        place_frame=20,
        track_map=evidence[
            "track_map"
        ],
        coordinates=evidence[
            "coordinates"
        ],
        demonstration_bin_order="left_to_right",
        perception_mode="prior_guided",
        prior_guided_task="pick_and_place",
    )

    assert (
        "Infer one pick-and-place action"
        in prompt
    )

    assert (
        "cubes and storage bins"
        in prompt
    )

    assert (
        "'<colour> cube'"
        in prompt
    )

    assert (
        "storage-bin track ID"
        in prompt
    )

    assert (
        "relation for placing a cube inside a storage bin is 'in'"
        in prompt
    )

    assert (
        "nut-assembly action"
        not in prompt
    )

    assert (
        "wooden pegs"
        not in prompt
    )


def test_nut_action_prompt_is_dual_to_pick_place_prompt():
    evidence = _nut_tracking_evidence()

    prompt = build_prompt(
        pick_frame=10,
        place_frame=20,
        track_map=evidence[
            "track_map"
        ],
        coordinates=evidence[
            "coordinates"
        ],
        demonstration_bin_order="left_to_right",
        perception_mode="prior_guided",
        prior_guided_task="nut_assembly",
    )

    assert (
        "Infer one nut-assembly action"
        in prompt
    )

    assert (
        "colored rings and wooden pegs"
        in prompt
    )

    assert (
        "'<colour> ring'"
        in prompt
    )

    for label in (
        "gray ring",
        "yellow ring",
        "blue ring",
        "wooden peg",
    ):
        assert (
            label
            in prompt
        )

    assert (
        "picked_category to 'ring'"
        in prompt
    )

    assert (
        "destination_category to 'peg'"
        in prompt
    )

    assert (
        "relation for assembling a ring onto a wooden peg is 'on'"
        in prompt
    )

    assert (
        "Set task_type to 'nut_assembly'"
        in prompt
    )

    assert (
        "assemble it onto the <ordinal> peg from the left"
        in prompt
    )


def test_validate_plan_accepts_valid_prior_guided_nut_plan():
    evidence = _nut_tracking_evidence()

    validate_plan(
        plan=_valid_nut_plan(),
        pick_frame=10,
        place_frame=20,
        track_map=evidence[
            "track_map"
        ],
        coordinates=evidence[
            "coordinates"
        ],
        demonstration_bin_order="left_to_right",
        perception_mode="prior_guided",
        prior_guided_task="nut_assembly",
    )


def test_validate_plan_rejects_non_peg_nut_destination():
    evidence = _nut_tracking_evidence()

    evidence[
        "track_map"
    ][
        "5"
    ] = {
        "detector_label": "storage bin",
    }

    evidence[
        "coordinates"
    ][
        "20"
    ][
        "5"
    ] = [
        600,
        100,
    ]

    plan = copy.deepcopy(
        _valid_nut_plan()
    )

    plan[
        "steps"
    ][
        0
    ][
        "destination_track_id"
    ] = 5

    with pytest.raises(
        ValueError,
        match="wooden peg|destination",
    ):
        validate_plan(
            plan=plan,
            pick_frame=10,
            place_frame=20,
            track_map=evidence[
                "track_map"
            ],
            coordinates=evidence[
                "coordinates"
            ],
            demonstration_bin_order="left_to_right",
            perception_mode="prior_guided",
            prior_guided_task="nut_assembly",
        )


def test_validate_plan_rejects_non_ring_nut_pick():
    evidence = _nut_tracking_evidence()

    evidence[
        "track_map"
    ][
        "1"
    ] = {
        "detector_label": "green cube",
    }

    plan = copy.deepcopy(
        _valid_nut_plan()
    )

    plan[
        "steps"
    ][
        0
    ][
        "picked_detector_label"
    ] = "green cube"

    plan[
        "steps"
    ][
        0
    ][
        "picked_category"
    ] = "cube"

    plan[
        "steps"
    ][
        0
    ][
        "picked_color"
    ] = "green"

    with pytest.raises(
        ValueError,
        match="ring|Nut-Assembly",
    ):
        validate_plan(
            plan=plan,
            pick_frame=10,
            place_frame=20,
            track_map=evidence[
                "track_map"
            ],
            coordinates=evidence[
                "coordinates"
            ],
            demonstration_bin_order="left_to_right",
            perception_mode="prior_guided",
            prior_guided_task="nut_assembly",
        )


def test_validate_plan_rejects_wrong_task_type_for_prior_guided_nut():
    evidence = _nut_tracking_evidence()

    plan = copy.deepcopy(
        _valid_nut_plan()
    )

    plan[
        "task_type"
    ] = "pick_and_place"

    with pytest.raises(
        ValueError,
        match="task_type|Nut-Assembly",
    ):
        validate_plan(
            plan=plan,
            pick_frame=10,
            place_frame=20,
            track_map=evidence[
                "track_map"
            ],
            coordinates=evidence[
                "coordinates"
            ],
            demonstration_bin_order="left_to_right",
            perception_mode="prior_guided",
            prior_guided_task="nut_assembly",
        )


# =====================================================================
# SceneInterpreter prompt and semantic-name audit
# =====================================================================


def test_pick_place_scene_interpreter_prompt_is_preserved():
    prompt = (
        PRIOR_GUIDED_PICK_AND_PLACE_SCENE_INTERPRETER_SYSTEM_PROMPT
    )

    assert (
        "CUBES"
        in prompt
    )

    assert (
        "STORAGE BINS"
        in prompt
    )

    assert (
        "red cube"
        in prompt
    )

    assert (
        "green cube"
        in prompt
    )

    assert (
        "first bin from the left"
        in prompt
    )

    assert (
        "WOODEN PEGS"
        not in prompt
    )

    assert (
        "gray ring"
        not in prompt
    )


def test_nut_scene_interpreter_prompt_is_dual_to_pick_place():
    prompt = (
        PRIOR_GUIDED_NUT_ASSEMBLY_SCENE_INTERPRETER_SYSTEM_PROMPT
    )

    assert (
        "RINGS"
        in prompt
    )

    assert (
        "WOODEN PEGS"
        in prompt
    )

    for label in (
        "gray ring",
        "yellow ring",
        "blue ring",
    ):
        assert (
            label
            in prompt
        )

    assert (
        "first peg from the left"
        in prompt
    )

    assert (
        "second peg from the left"
        in prompt
    )

    assert (
        "third peg from the left"
        in prompt
    )


def test_scene_interpreter_routes_nut_prompt_and_accepts_ordinal_pegs(
    tmp_path,
    monkeypatch,
):
    monkeypatch.setenv(
        "OPENAI_API_KEY",
        "test-key",
    )

    raw_scene = RawSceneState(
        objects=(
            _raw_object(
                "yellow_ring_0",
                label="yellow ring",
                pixel_coordinates=(
                    100,
                    200,
                ),
            ),
            _raw_object(
                "wooden_peg_0",
                label="wooden peg",
                pixel_coordinates=(
                    200,
                    200,
                ),
            ),
            _raw_object(
                "wooden_peg_1",
                label="wooden peg",
                pixel_coordinates=(
                    350,
                    200,
                ),
            ),
            _raw_object(
                "wooden_peg_2",
                label="wooden peg",
                pixel_coordinates=(
                    500,
                    200,
                ),
            ),
        )
    )

    response_payload = {
        "objects": [
            {
                "raw_object_id": "yellow_ring_0",
                "semantic_name": "yellow ring",
            },
            {
                "raw_object_id": "wooden_peg_0",
                "semantic_name": "first peg from the left",
            },
            {
                "raw_object_id": "wooden_peg_1",
                "semantic_name": "second peg from the left",
            },
            {
                "raw_object_id": "wooden_peg_2",
                "semantic_name": "third peg from the left",
            },
        ],
    }

    captured = {}

    _install_fake_scene_interpreter_openai(
        monkeypatch,
        response_payload=response_payload,
        captured=captured,
    )

    interpreter = SceneInterpreter(
        model="test-model",
        perception_mode="prior_guided",
        prior_guided_task="nut_assembly",
    )

    semantic_names = (
        interpreter._assign_semantic_names(
            raw_scene=raw_scene,
            overlay_image_path=_write_overlay(
                tmp_path
            ),
        )
    )

    assert semantic_names == {
        "yellow_ring_0": "yellow ring",
        "wooden_peg_0": "first peg from the left",
        "wooden_peg_1": "second peg from the left",
        "wooden_peg_2": "third peg from the left",
    }

    assert (
        captured[
            "messages"
        ][
            0
        ][
            "content"
        ]
        == PRIOR_GUIDED_NUT_ASSEMBLY_SCENE_INTERPRETER_SYSTEM_PROMPT
    )


def test_scene_interpreter_rejects_non_ordinal_peg_name(
    tmp_path,
    monkeypatch,
):
    monkeypatch.setenv(
        "OPENAI_API_KEY",
        "test-key",
    )

    raw_scene = RawSceneState(
        objects=(
            _raw_object(
                "gray_ring_0",
                label="gray ring",
                pixel_coordinates=(
                    100,
                    200,
                ),
            ),
            _raw_object(
                "wooden_peg_0",
                label="wooden peg",
                pixel_coordinates=(
                    200,
                    200,
                ),
            ),
        )
    )

    response_payload = {
        "objects": [
            {
                "raw_object_id": "gray_ring_0",
                "semantic_name": "gray ring",
            },
            {
                "raw_object_id": "wooden_peg_0",
                "semantic_name": "wooden peg",
            },
        ],
    }

    captured = {}

    _install_fake_scene_interpreter_openai(
        monkeypatch,
        response_payload=response_payload,
        captured=captured,
    )

    interpreter = SceneInterpreter(
        model="test-model",
        perception_mode="prior_guided",
        prior_guided_task="nut_assembly",
    )

    with pytest.raises(
        RuntimeError,
        match="wooden-peg semantic name|peg from the left",
    ):
        interpreter._assign_semantic_names(
            raw_scene=raw_scene,
            overlay_image_path=_write_overlay(
                tmp_path
            ),
        )
