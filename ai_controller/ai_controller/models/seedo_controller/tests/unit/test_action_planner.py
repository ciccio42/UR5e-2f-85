from __future__ import annotations

from pathlib import Path

import pytest

from ai_controller.models.seedo_controller import action_planner
from ai_controller.models.seedo_controller.action_planner import (
    ActionPlanner,
)
from results import ActionStep
from vlm import (
    GENERALIZED_PLAN_SCHEMA,
    build_prompt,
    validate_plan,
)


@pytest.mark.parametrize(
    ("perception_mode", "expected"),
    [
        ("generalized", "generalized"),
        ("GENERALIZED", "generalized"),
        ("  generalized  ", "generalized"),
        ("prior_guided", "prior_guided"),
        ("PRIOR_GUIDED", "prior_guided"),
        ("  prior_guided  ", "prior_guided"),
    ],
)
def test_constructor_normalizes_perception_mode(
    perception_mode,
    expected,
):
    planner = ActionPlanner(
        perception_mode=perception_mode,
    )

    assert planner.perception_mode == expected


@pytest.mark.parametrize(
    "perception_mode",
    [
        "",
        "unknown",
        "prior-guided",
        "general",
    ],
)
def test_constructor_rejects_invalid_perception_mode(
    perception_mode,
):
    with pytest.raises(
        ValueError,
        match="Invalid perception_mode",
    ):
        ActionPlanner(
            perception_mode=perception_mode,
        )


@pytest.mark.parametrize(
    ("order", "expected"),
    [
        ("left_to_right", "left_to_right"),
        ("LEFT_TO_RIGHT", "left_to_right"),
        ("  left_to_right  ", "left_to_right"),
        ("right_to_left", "right_to_left"),
        ("RIGHT_TO_LEFT", "right_to_left"),
        ("  right_to_left  ", "right_to_left"),
    ],
)
def test_constructor_normalizes_demonstration_bin_order(
    order,
    expected,
):
    planner = ActionPlanner(
        demonstration_bin_order=order,
    )

    assert planner.demonstration_bin_order == expected


@pytest.mark.parametrize(
    "order",
    [
        "",
        "left",
        "right",
        "top_to_bottom",
        "unknown",
    ],
)
def test_constructor_rejects_invalid_demonstration_bin_order(
    order,
):
    with pytest.raises(
        ValueError,
        match="Invalid demonstration_bin_order",
    ):
        ActionPlanner(
            demonstration_bin_order=order,
        )


def test_constructor_stores_model():
    planner = ActionPlanner(
        model="test-model",
    )

    assert planner.model == "test-model"


def test_run_rejects_missing_video(
    tmp_path,
):
    planner = ActionPlanner()

    missing_video = (
        tmp_path
        / "missing.mp4"
    )

    with pytest.raises(
        FileNotFoundError,
        match="Annotated video does not exist",
    ):
        planner.run(
            annotated_video_path=missing_video,
            keyframes=(1, 2),
            track_id_map={
                1: {
                    "category": "cube",
                },
            },
            key_frame_coordinates={
                "1": ["test"],
            },
            artifacts_dir=tmp_path / "artifacts",
        )


def test_run_rejects_empty_video(
    tmp_path,
):
    planner = ActionPlanner()

    video_path = (
        tmp_path
        / "empty.mp4"
    )

    video_path.touch()

    with pytest.raises(
        ValueError,
        match="Annotated video is empty",
    ):
        planner.run(
            annotated_video_path=video_path,
            keyframes=(1, 2),
            track_id_map={
                1: {
                    "category": "cube",
                },
            },
            key_frame_coordinates={
                "1": ["test"],
            },
            artifacts_dir=tmp_path / "artifacts",
        )


@pytest.mark.parametrize(
    "keyframes",
    [
        (),
        (1,),
        (1, 2, 3),
    ],
)
def test_run_requires_exactly_two_keyframes(
    tmp_path,
    keyframes,
):
    planner = ActionPlanner()

    video_path = (
        tmp_path
        / "video.mp4"
    )

    video_path.write_bytes(b"video")

    with pytest.raises(
        ValueError,
        match="requires exactly two keyframes",
    ):
        planner.run(
            annotated_video_path=video_path,
            keyframes=keyframes,
            track_id_map={
                1: {
                    "category": "cube",
                },
            },
            key_frame_coordinates={
                "1": ["test"],
            },
            artifacts_dir=tmp_path / "artifacts",
        )


@pytest.mark.parametrize(
    "keyframes",
    [
        (10, 10),
        (20, 10),
    ],
)
def test_run_requires_pick_before_place(
    tmp_path,
    keyframes,
):
    planner = ActionPlanner()

    video_path = (
        tmp_path
        / "video.mp4"
    )

    video_path.write_bytes(b"video")

    with pytest.raises(
        ValueError,
        match="pick keyframe must precede",
    ):
        planner.run(
            annotated_video_path=video_path,
            keyframes=keyframes,
            track_id_map={
                1: {
                    "category": "cube",
                },
            },
            key_frame_coordinates={
                "1": ["test"],
            },
            artifacts_dir=tmp_path / "artifacts",
        )


def test_run_converts_keyframes_to_int(
    tmp_path,
    monkeypatch,
):
    planner = ActionPlanner()

    video_path = (
        tmp_path
        / "video.mp4"
    )

    video_path.write_bytes(b"video")

    captured = {}

    def fake_generate_action_plan(**kwargs):
        captured.update(kwargs)
        return object()

    monkeypatch.setattr(
        action_planner,
        "generate_action_plan",
        fake_generate_action_plan,
    )

    planner.run(
        annotated_video_path=video_path,
        keyframes=("10", "20"),
        track_id_map={
            1: {
                "category": "cube",
            },
        },
        key_frame_coordinates={
            "10": ["test"],
        },
        artifacts_dir=tmp_path / "artifacts",
    )

    assert captured["keyframes"] == (
        10,
        20,
    )


def test_run_rejects_empty_track_id_map(
    tmp_path,
):
    planner = ActionPlanner()

    video_path = (
        tmp_path
        / "video.mp4"
    )

    video_path.write_bytes(b"video")

    with pytest.raises(
        ValueError,
        match="track_id_map cannot be empty",
    ):
        planner.run(
            annotated_video_path=video_path,
            keyframes=(1, 2),
            track_id_map={},
            key_frame_coordinates={
                "1": ["test"],
            },
            artifacts_dir=tmp_path / "artifacts",
        )


def test_run_rejects_empty_key_frame_coordinates(
    tmp_path,
):
    planner = ActionPlanner()

    video_path = (
        tmp_path
        / "video.mp4"
    )

    video_path.write_bytes(b"video")

    with pytest.raises(
        ValueError,
        match="key_frame_coordinates cannot be empty",
    ):
        planner.run(
            annotated_video_path=video_path,
            keyframes=(1, 2),
            track_id_map={
                1: {
                    "category": "cube",
                },
            },
            key_frame_coordinates={},
            artifacts_dir=tmp_path / "artifacts",
        )


def test_run_creates_artifacts_directory(
    tmp_path,
    monkeypatch,
):
    planner = ActionPlanner()

    video_path = (
        tmp_path
        / "video.mp4"
    )

    video_path.write_bytes(b"video")

    artifacts_dir = (
        tmp_path
        / "nested"
        / "action_planning"
    )

    def fake_generate_action_plan(**kwargs):
        return object()

    monkeypatch.setattr(
        action_planner,
        "generate_action_plan",
        fake_generate_action_plan,
    )

    planner.run(
        annotated_video_path=video_path,
        keyframes=(1, 2),
        track_id_map={
            1: {
                "category": "cube",
            },
        },
        key_frame_coordinates={
            "1": ["test"],
        },
        artifacts_dir=artifacts_dir,
    )

    assert artifacts_dir.is_dir()


def test_run_forwards_normalized_arguments_to_vlm(
    tmp_path,
    monkeypatch,
):
    planner = ActionPlanner(
        model="test-model",
        demonstration_bin_order="RIGHT_TO_LEFT",
        perception_mode="GENERALIZED",
    )

    video_path = (
        tmp_path
        / "video.mp4"
    )

    video_path.write_bytes(b"video")

    artifacts_dir = (
        tmp_path
        / "artifacts"
    )

    track_id_map = {
        5: {
            "category": "cube",
        },
        10: {
            "category": "bin",
        },
    }

    key_frame_coordinates = {
        "5": [
            "x=10, y=20",
        ],
        "10": [
            "x=30, y=40",
        ],
    }

    captured = {}
    expected_result = object()

    def fake_generate_action_plan(**kwargs):
        captured.update(kwargs)
        return expected_result

    monkeypatch.setattr(
        action_planner,
        "generate_action_plan",
        fake_generate_action_plan,
    )

    result = planner.run(
        annotated_video_path=video_path,
        keyframes=(10, 20),
        track_id_map=track_id_map,
        key_frame_coordinates=key_frame_coordinates,
        artifacts_dir=artifacts_dir,
    )

    assert result is expected_result

    assert captured == {
        "annotated_video_path": video_path.resolve(),
        "keyframes": (10, 20),
        "track_id_map": track_id_map,
        "key_frame_coordinates": key_frame_coordinates,
        "artifacts_dir": artifacts_dir.resolve(),
        "model": "test-model",
        "demonstration_bin_order": "right_to_left",
        "perception_mode": "generalized",
    }


def test_run_resolves_input_paths(
    tmp_path,
    monkeypatch,
):
    planner = ActionPlanner()

    video_path = (
        tmp_path
        / "video.mp4"
    )

    video_path.write_bytes(b"video")

    artifacts_dir = (
        tmp_path
        / "artifacts"
    )

    captured = {}

    def fake_generate_action_plan(**kwargs):
        captured.update(kwargs)
        return object()

    monkeypatch.setattr(
        action_planner,
        "generate_action_plan",
        fake_generate_action_plan,
    )

    planner.run(
        annotated_video_path=video_path,
        keyframes=(1, 2),
        track_id_map={
            1: {
                "category": "cube",
            },
        },
        key_frame_coordinates={
            "1": ["test"],
        },
        artifacts_dir=artifacts_dir,
    )

    assert isinstance(
        captured["annotated_video_path"],
        Path,
    )

    assert isinstance(
        captured["artifacts_dir"],
        Path,
    )

    assert captured["annotated_video_path"].is_absolute()
    assert captured["artifacts_dir"].is_absolute()


def test_run_raises_when_generate_action_plan_returns_none(
    tmp_path,
    monkeypatch,
):
    planner = ActionPlanner()

    video_path = (
        tmp_path
        / "video.mp4"
    )

    video_path.write_bytes(b"video")

    def fake_generate_action_plan(**kwargs):
        return None

    monkeypatch.setattr(
        action_planner,
        "generate_action_plan",
        fake_generate_action_plan,
    )

    with pytest.raises(
        RuntimeError,
        match=r"generate_action_plan\(\) returned None",
    ):
        planner.run(
            annotated_video_path=video_path,
            keyframes=(1, 2),
            track_id_map={
                1: {
                    "category": "cube",
                },
            },
            key_frame_coordinates={
                "1": ["test"],
            },
            artifacts_dir=tmp_path / "artifacts",
        )


def test_run_propagates_generate_action_plan_exception(
    tmp_path,
    monkeypatch,
):
    planner = ActionPlanner()

    video_path = (
        tmp_path
        / "video.mp4"
    )

    video_path.write_bytes(b"video")

    def fake_generate_action_plan(**kwargs):
        raise RuntimeError(
            "VLM failure"
        )

    monkeypatch.setattr(
        action_planner,
        "generate_action_plan",
        fake_generate_action_plan,
    )

    with pytest.raises(
        RuntimeError,
        match="VLM failure",
    ):
        planner.run(
            annotated_video_path=video_path,
            keyframes=(1, 2),
            track_id_map={
                1: {
                    "category": "cube",
                },
            },
            key_frame_coordinates={
                "1": ["test"],
            },
            artifacts_dir=tmp_path / "artifacts",
        )


# ------------------------------------------------------------------
# grasp_instruction tests
# ------------------------------------------------------------------


def _build_generalized_test_track_map():
    return {
        "1": {
            "detector_label": "gray ring",
            "category": "ring",
            "attributes": {},
        },
        "2": {
            "detector_label": "wooden peg",
            "category": "peg",
            "attributes": {},
        },
    }


def _build_generalized_test_coordinates():
    return {
        "10": {
            "1": [100, 120],
            "2": [300, 120],
        },
        "20": {
            "1": [295, 125],
            "2": [300, 120],
        },
    }


def _build_generalized_test_plan(
    grasp_instruction: str,
):
    return {
        "steps": [
            {
                "pick_keyframe": 10,
                "place_keyframe": 20,
                "picked_track_id": 1,
                "picked_category": "ring",
                "picked_color": "",
                "picked_detector_label": "gray ring",
                "grasp_instruction": grasp_instruction,
                "destination_track_id": 2,
                "destination_category": "peg",
                "relation": "on",
            }
        ],
        "status": "completed",
        "task_type": "nut_assembly",
        "ambiguities": [],
    }


def test_action_step_grasp_instruction_defaults_to_empty_string():
    step = ActionStep(
        pick_keyframe=10,
        place_keyframe=20,
        picked_track_id=1,
        picked_category="ring",
        picked_color="",
        destination_track_id=2,
        destination_category="peg",
        destination_ordinal_from_left=None,
        relation="on",
        action=(
            "Pick the gray ring and assemble it "
            "onto the wooden peg."
        ),
        picked_detector_label="gray ring",
    )

    assert step.grasp_instruction == ""


def test_action_step_stores_grasp_instruction():
    instruction = (
        "Pick up the gray ring by grasping its handle, "
        "not the circular ring body."
    )

    step = ActionStep(
        pick_keyframe=10,
        place_keyframe=20,
        picked_track_id=1,
        picked_category="ring",
        picked_color="",
        destination_track_id=2,
        destination_category="peg",
        destination_ordinal_from_left=None,
        relation="on",
        action=(
            "Pick the gray ring and assemble it "
            "onto the wooden peg."
        ),
        picked_detector_label="gray ring",
        grasp_instruction=instruction,
    )

    assert step.grasp_instruction == instruction


def test_generalized_schema_requires_grasp_instruction():
    step_schema = (
        GENERALIZED_PLAN_SCHEMA[
            "schema"
        ][
            "properties"
        ][
            "steps"
        ][
            "items"
        ]
    )

    assert (
        "grasp_instruction"
        in step_schema["properties"]
    )

    assert (
        "grasp_instruction"
        in step_schema["required"]
    )


def test_generalized_prompt_requests_grasp_instruction():
    prompt = build_prompt(
        pick_frame=10,
        place_frame=20,
        track_map=(
            _build_generalized_test_track_map()
        ),
        coordinates=(
            _build_generalized_test_coordinates()
        ),
        perception_mode="generalized",
    )

    assert "GRASP INSTRUCTION:" in prompt
    assert "grasp_instruction" in prompt

    assert (
        "Pick up the gray ring by grasping its handle, "
        "not the circular ring body."
        in prompt
    )

    assert (
        "Do not specify pixel coordinates"
        in prompt
    )


def test_validate_generalized_plan_accepts_valid_grasp_instruction():
    validate_plan(
        plan=_build_generalized_test_plan(
            (
                "Pick up the gray ring by grasping "
                "its handle, not the circular ring body."
            )
        ),
        pick_frame=10,
        place_frame=20,
        track_map=(
            _build_generalized_test_track_map()
        ),
        coordinates=(
            _build_generalized_test_coordinates()
        ),
        demonstration_bin_order="left_to_right",
        perception_mode="generalized",
    )


def test_validate_generalized_plan_rejects_empty_grasp_instruction():
    with pytest.raises(
        ValueError,
        match="empty grasp_instruction",
    ):
        validate_plan(
            plan=_build_generalized_test_plan(
                "   "
            ),
            pick_frame=10,
            place_frame=20,
            track_map=(
                _build_generalized_test_track_map()
            ),
            coordinates=(
                _build_generalized_test_coordinates()
            ),
            demonstration_bin_order="left_to_right",
            perception_mode="generalized",
        )


def test_validate_generalized_plan_requires_detector_label_in_grasp_instruction():
    with pytest.raises(
        ValueError,
        match=(
            "must explicitly name the exact "
            "picked_detector_label"
        ),
    ):
        validate_plan(
            plan=_build_generalized_test_plan(
                "Pick up the object by grasping its handle."
            ),
            pick_frame=10,
            place_frame=20,
            track_map=(
                _build_generalized_test_track_map()
            ),
            coordinates=(
                _build_generalized_test_coordinates()
            ),
            demonstration_bin_order="left_to_right",
            perception_mode="generalized",
        )