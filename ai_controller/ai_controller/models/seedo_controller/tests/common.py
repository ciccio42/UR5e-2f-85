from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml

from results import ActionPlanningResult, ScenePerceptionResult, SceneState
from ai_controller.models.seedo_controller.action_planner import ActionPlanner
from ai_controller.models.seedo_controller.scene_interpreter import SceneInterpreter
from ai_controller.models.seedo_controller.scene_perceiver import ScenePerceiver
from ai_controller.models.seedo_controller.visual_prompter import VisualPrompter

def build_action_planning_result(
    args: argparse.Namespace,
    artifacts_dir: Path,
) -> ActionPlanningResult:
    """Run the real visual-prompting and action-planning stages."""

    if args.video is None:
        raise ValueError(
            "--video is required to build the action plan."
        )

    if args.expected_keyframes is None:
        raise ValueError(
            "--expected-keyframes is required to build the action plan."
        )

    expected_keyframes = tuple(
        int(frame)
        for frame in args.expected_keyframes
    )

    if len(expected_keyframes) != 2:
        raise ValueError(
            "Exactly two expected keyframes are required."
        )

    if expected_keyframes[0] >= expected_keyframes[1]:
        raise ValueError(
            "The first expected keyframe must precede "
            "the second one."
        )

    artifacts_dir = (
        Path(artifacts_dir)
        .expanduser()
        .resolve()
    )

    visual_artifacts_dir = (
        artifacts_dir
        / "visual_prompting"
    )

    action_artifacts_dir = (
        artifacts_dir
        / "action_planning"
    )

    visual_prompter = VisualPrompter(
        grounding_config=args.grounding_config,
        grounding_checkpoint=args.grounding_checkpoint,
        bert_model=args.bert_model,
        sam_checkpoint=args.sam_checkpoint,
        sam2_checkpoint=args.sam2_checkpoint,
    )

    visual_result = visual_prompter.run(
        video_path=args.video,
        keyframes=expected_keyframes,
        artifacts_dir=visual_artifacts_dir,
    )

    if not visual_result.annotated_video_path.is_file():
        raise AssertionError(
            "VisualPrompter did not produce an annotated video: "
            f"{visual_result.annotated_video_path}"
        )

    if not visual_result.track_id_map:
        raise AssertionError(
            "VisualPrompter returned an empty track_id_map."
        )

    if not visual_result.key_frame_coordinates:
        raise AssertionError(
            "VisualPrompter returned empty key-frame coordinates."
        )

    planner = ActionPlanner()

    action_result = planner.run(
        annotated_video_path=(
            visual_result.annotated_video_path
        ),
        keyframes=expected_keyframes,
        track_id_map=visual_result.track_id_map,
        key_frame_coordinates=(
            visual_result.key_frame_coordinates
        ),
        artifacts_dir=action_artifacts_dir,
    )

    if action_result.status != "completed":
        raise AssertionError(
            "LMP integration requires a completed action plan, "
            f"but ActionPlanner returned "
            f"'{action_result.status}'. "
            f"Ambiguities: {action_result.ambiguities}"
        )

    if not action_result.steps:
        raise AssertionError(
            "Completed ActionPlanningResult contains no steps."
        )

    if not action_result.natural_language_plan.strip():
        raise AssertionError(
            "ActionPlanner returned an empty natural-language plan."
        )

    return action_result

def load_scene_runtime_input(
    args: argparse.Namespace,
) -> dict[str, object]:
    """Load raw runtime inputs from an offline scene capture."""

    if args.scene_dir is None:
        raise ValueError(
            "--scene-dir is required for runtime scene tests."
        )

    if args.base_to_table_transform is None:
        raise ValueError(
            "--base-to-table-transform is required for "
            "runtime scene tests."
        )

    scene_dir = (
        Path(args.scene_dir)
        .expanduser()
        .resolve()
    )

    transform_path = (
        Path(args.base_to_table_transform)
        .expanduser()
        .resolve()
    )

    rgb_path = scene_dir / "rgb.png"
    depth_path = scene_dir / "depth.npy"
    camera_info_path = (
        scene_dir
        / "camera_info.yaml"
    )

    required_paths = (
        rgb_path,
        depth_path,
        camera_info_path,
        transform_path,
    )

    for path in required_paths:
        if not path.is_file():
            raise FileNotFoundError(
                "Required runtime input does not exist: "
                f"{path}"
            )

    rgb_bgr = cv2.imread(
        str(rgb_path)
    )

    if rgb_bgr is None:
        raise RuntimeError(
            f"Could not read RGB image: {rgb_path}"
        )

    rgb = cv2.cvtColor(
        rgb_bgr,
        cv2.COLOR_BGR2RGB,
    )

    depth = np.load(
        depth_path
    )

    with camera_info_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        camera_info = yaml.safe_load(
            stream
        )

    with transform_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        base_to_table_transform = yaml.safe_load(
            stream
        )

    return {
        "rgb": rgb,
        "depth": depth,
        "camera_info": camera_info,
        "base_to_table_transform": (
            base_to_table_transform
        ),
    }

def build_scene_perception_result(
    args: argparse.Namespace,
) -> ScenePerceptionResult:
    """Build the runtime ScenePerceptionResult from an offline scene capture."""

    if not args.model_config:
        raise ValueError(
            "--model-config is required for runtime scene tests."
        )

    model_config_path = (
        Path(args.model_config)
        .expanduser()
        .resolve()
    )

    if not model_config_path.is_file():
        raise FileNotFoundError(
            "Required test input does not exist: "
            f"{model_config_path}"
        )

    runtime_input = load_scene_runtime_input(
        args
    )

    with model_config_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        config = yaml.safe_load(
            stream
        )

    visual_config = config[
        "visual_prompter"
    ]

    perception_config = config[
        "scene_perceiver"
    ]

    perceiver = ScenePerceiver(
        camera_calibration_path=perception_config[
            "camera_calibration_path"
        ],
        camera_name=perception_config.get(
            "camera_name",
            "zed_front",
        ),
        grounding_config=visual_config[
            "grounding_config"
        ],
        grounding_checkpoint=visual_config[
            "grounding_checkpoint"
        ],
        bert_model=visual_config[
            "bert_model"
        ],
        sam_checkpoint=visual_config[
            "sam_checkpoint"
        ],
        detector_labels=perception_config[
            "detector_labels"
        ],
    )

    artifacts_dir = None

    if args.artifacts_dir is not None:
        artifacts_dir = (
            Path(args.artifacts_dir)
            .expanduser()
            .resolve()
        )

    return perceiver.run(
        rgb_image=runtime_input["rgb"],
        depth_image=runtime_input["depth"],
        camera_info=runtime_input["camera_info"],
        base_to_table_transform=runtime_input[
            "base_to_table_transform"
        ],
        artifacts_dir=artifacts_dir,
    )

def build_scene_state(
    args: argparse.Namespace,
    artifacts_dir: Path,
) -> SceneState:
    """Run the real runtime perception and semantic interpretation stages."""

    if not args.model_config:
        raise ValueError(
            "--model-config is required to build SceneState."
        )

    artifacts_dir = (
        Path(artifacts_dir)
        .expanduser()
        .resolve()
    )

    perception_artifacts_dir = (
        artifacts_dir
        / "scene_perception"
    )

    interpretation_artifacts_dir = (
        artifacts_dir
        / "scene_interpretation"
    )

    original_artifacts_dir = (
        args.artifacts_dir
    )

    try:
        args.artifacts_dir = (
            perception_artifacts_dir
        )

        perception_result = (
            build_scene_perception_result(
                args
            )
        )

    finally:
        args.artifacts_dir = (
            original_artifacts_dir
        )

    if not perception_result.raw_scene.objects:
        raise AssertionError(
            "ScenePerceiver returned no objects."
        )

    if (
        perception_result.overlay_image_path
        is None
    ):
        raise AssertionError(
            "ScenePerceiver did not produce "
            "the semantic-interpreter overlay."
        )

    if (
        not perception_result
        .overlay_image_path
        .is_file()
    ):
        raise AssertionError(
            "ScenePerceiver overlay does not exist: "
            f"{perception_result.overlay_image_path}"
        )

    model_config_path = (
        Path(args.model_config)
        .expanduser()
        .resolve()
    )

    if not model_config_path.is_file():
        raise FileNotFoundError(
            "SeeDo controller configuration does not exist: "
            f"{model_config_path}"
        )

    with model_config_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        config = yaml.safe_load(
            stream
        )

    interpreter_config = config.get(
        "scene_interpreter",
        {},
    )

    interpreter = SceneInterpreter(
        model=interpreter_config.get(
            "model",
            "gpt-4o-2024-08-06",
        ),
    )

    scene_state = interpreter.run(
        perception_result=perception_result,
        artifacts_dir=(
            interpretation_artifacts_dir
        ),
    )

    if not scene_state.objects:
        raise AssertionError(
            "SceneInterpreter returned no semantic objects."
        )

    object_ids = [
        obj.object_id
        for obj in scene_state.objects
    ]

    if len(object_ids) != len(
        set(object_ids)
    ):
        raise AssertionError(
            "SceneInterpreter produced duplicate semantic IDs."
        )

    return scene_state
