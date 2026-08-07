from __future__ import annotations

import argparse
import cv2
import numpy as np
import yaml

from pathlib import Path
from results import ScenePerceptionResult

from ai_controller.models.seedo_controller.keyframe_selector import (
    KeyframeSelector,
)

from ai_controller.models.seedo_controller.visual_prompter import (
    VisualPrompter,
)

from ai_controller.models.seedo_controller.action_planner import (
    ActionPlanner,
)

from ai_controller.models.seedo_controller.seedo_controller import (
    SeeDoController,
)

from ai_controller.models.seedo_controller.scene_perceiver import (
    ScenePerceiver,
)
from ai_controller.models.seedo_controller.scene_interpreter import (
    SceneInterpreter,
)
from ai_controller.models.seedo_controller.lmp_generator import (
    LMPGenerator,
)

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Standalone test for the SeeDo keyframe selector."
    )
    parser.add_argument(
        "--video",
        type=Path,
        default=None,
        help="Path to the input demonstration video.",
    )
    parser.add_argument(
        "--artifacts-dir",
        type=Path,
        default=Path("artifacts"),
        help="Optional directory for debug artifacts and previews.",
    )
    parser.add_argument(
        "--expected-keyframes",
        type=int,
        nargs="*",
        default=None,
        help="Optional expected keyframe indexes.",
    )

    parser.add_argument(
        "--stage",
        choices=[
            "keyframe",
            "visual_prompting",
            "action_planning",
            "scene_perceiver",
            "scene_interpreter",
            "lmp_generator",
            "seedo_controller",
        ],
        default="keyframe",
        help="Pipeline stage to test.",
    )

    parser.add_argument(
        "--grounding-config",
        type=Path,
        default=Path(
            "/opt/checkpoints/seedo/groundingdino/"
            "GroundingDINO_SwinB.cfg.py"
        ),
    )

    parser.add_argument(
        "--grounding-checkpoint",
        type=Path,
        default=Path(
            "/opt/checkpoints/seedo/groundingdino/"
            "groundingdino_swinb_cogcoor.pth"
        ),
    )

    parser.add_argument(
        "--bert-model",
        type=Path,
        default=Path(
            "/opt/checkpoints/seedo/bert-base-uncased"
        ),
    )

    parser.add_argument(
        "--sam-checkpoint",
        type=Path,
        default=Path(
            "/opt/checkpoints/seedo/sam/sam_vit_h_4b8939.pth"
        ),
    )

    parser.add_argument(
        "--sam2-checkpoint",
        type=Path,
        default=Path(
            "/opt/checkpoints/seedo/sam2/sam2_hiera_large.pt"
        ),
    )
    
    parser.add_argument(
        "--model-config",
        type=str,
        help="Path to the SeeDo controller YAML configuration.",
    )

    parser.add_argument(
        "--scene-dir",
        type=Path,
        default=None,
        help=(
            "Directory containing the offline runtime scene "
            "(rgb.png, depth.npy, camera_info.yaml)."
        ),
    )

    parser.add_argument(
        "--base-to-table-transform",
        type=Path,
        default=None,
        help=(
            "YAML file containing the base_link/table_0 "
            "transform used for offline runtime tests."
        ),
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.stage == "keyframe":
        return run_keyframe_test(args)

    if args.stage == "visual_prompting":
        return run_visual_prompting_test(args)

    if args.stage == "action_planning":
        return run_action_planning_test(args)

    if args.stage == "scene_perceiver":
        return run_scene_perceiver_test(args)

    if args.stage == "scene_interpreter":
        return run_scene_interpreter_test(args)

    if args.stage == "lmp_generator":
        return run_lmp_generator_test(args)

    if args.stage == "seedo_controller":
        return run_seedo_controller_test(args)

    raise ValueError(f"Unsupported stage: {args.stage}")
    

def run_keyframe_test(args: argparse.Namespace) -> int:

    if args.video is None:
        raise ValueError(
            "--video is required for the keyframe test."
        )

    selector = KeyframeSelector(
        gaussian_sigma=5.0,
        prominence=0.8,
        expected_keyframes=2,
        save_preview=True,
    )

    result = selector.run(
        video_path=args.video,
        artifacts_dir=args.artifacts_dir,
    )

    print("Keyframe selection completed")
    print(f"Video: {result.video_path}")
    print(f"Keyframes: {list(result.keyframes)}")
    print(f"Images returned: {len(result.keyframe_images)}")
    print(f"Artifacts directory: {result.artifacts_dir}")

    for frame_index, image in zip(
        result.keyframes,
        result.keyframe_images,
        strict=True,
    ):
        print(
            f"Frame {frame_index}: "
            f"shape={image.shape}, dtype={image.dtype}"
        )

    if args.expected_keyframes is not None:
        expected = tuple(args.expected_keyframes)

        if result.keyframes != expected:
            raise AssertionError(
                f"Expected keyframes {list(expected)}, "
                f"but received {list(result.keyframes)}"
            )

    preview_dir = result.artifacts_dir / "returned_keyframes"

    if not preview_dir.is_dir():
        raise AssertionError(
            f"Preview directory was not created: {preview_dir}"
        )

    for frame_index in result.keyframes:
        preview_path = (
            preview_dir
            / f"keyframe_{frame_index:06d}.png"
        )

        if not preview_path.is_file():
            raise AssertionError(
                f"Missing keyframe preview: {preview_path}"
            )

    print("TEST PASSED")
    return 0

def run_visual_prompting_test(
        args: argparse.Namespace,
    ) -> int:

        if args.video is None:
            raise ValueError(
                "--video is required for the visual prompting test."
            )

        if args.expected_keyframes is None:
            raise ValueError(
                "--expected-keyframes is required for the visual_prompting test."
            )

        prompter = VisualPrompter(
            grounding_config=args.grounding_config,
            grounding_checkpoint=args.grounding_checkpoint,
            bert_model=args.bert_model,
            sam_checkpoint=args.sam_checkpoint,
            sam2_checkpoint=args.sam2_checkpoint,
        )

        expected_keyframes = tuple(args.expected_keyframes)

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

        if result.annotated_video_path.stat().st_size == 0:
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
                    f"No object coordinates were returned for {frame_key}."
                )

        tracked_object_ids = set(result.track_id_map.keys())

        if any(
            not isinstance(track_id, int)
            for track_id in tracked_object_ids
        ):
            raise AssertionError(
                "All track IDs must be integers."
            )

        for track_id, info in result.track_id_map.items():
            if "detector_label" not in info:
                raise AssertionError(
                    f"Track {track_id} has no detector_label."
                )

            if "initial_center" not in info:
                raise AssertionError(
                    f"Track {track_id} has no initial_center."
                )

        print("Visual prompting completed")
        print(f"Annotated video: {result.annotated_video_path}")

        print("\nTracked objects:")
        for track_id, info in result.track_id_map.items():
            print(f"  Track {track_id}: {info}")

        print("\nKey-frame coordinates:")
        for frame_name, coordinates in (
            result.key_frame_coordinates.items()
        ):
            print(f"  {frame_name}: {coordinates}")

        print("\nBounding box summary:")
        print(result.bounding_box_summary)

        print("\nCount diagnostics:")
        print(result.count_diagnostics)

        print("\nTEST PASSED")
        return 0

def run_action_planning_test(
        args: argparse.Namespace,
    ) -> int:

        if args.video is None:
            raise ValueError(
                "--video is required for the action_planning test."
            )

        if args.expected_keyframes is None:
            raise ValueError(
                "--expected-keyframes is required for the action_planning test."
            )

        expected_keyframes = tuple(args.expected_keyframes)

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
            artifacts_dir=(
                Path(args.artifacts_dir)
                / "visual_prompting"
            ),
        )

        planner = ActionPlanner()

        action_result = planner.run(
            annotated_video_path=visual_result.annotated_video_path,
            keyframes=expected_keyframes,
            track_id_map=visual_result.track_id_map,
            key_frame_coordinates=visual_result.key_frame_coordinates,
            artifacts_dir=(
                Path(args.artifacts_dir)
                / "action_planning"
            ),
        )

        print("Action planning completed")
        print(f"Status: {action_result.status}")
        print(
            "Natural-language plan: "
            f"{action_result.natural_language_plan}"
        )

        print("\nSteps:")
        for index, step in enumerate(
            action_result.steps,
            start=1,
        ):
            print(f"  Step {index}: {step}")

        print("\nAmbiguities:")
        for ambiguity in action_result.ambiguities:
            print(f"  - {ambiguity}")

        if action_result.status not in (
            "completed",
            "ambiguous",
        ):
            raise AssertionError(
                f"Unexpected status: {action_result.status}"
            )

        if (
            action_result.status == "completed"
            and len(action_result.steps) != 1
        ):
            raise AssertionError(
                "A completed action plan must contain exactly one step."
            )

        if (
            action_result.status == "ambiguous"
            and not action_result.ambiguities
        ):
            raise AssertionError(
                "An ambiguous action plan must explain the ambiguity."
            )

        if not action_result.natural_language_plan.strip():
            raise AssertionError(
                "Natural-language plan is empty."
            )

        print("\nTEST PASSED")

        return 0

def build_scene_perception_result(
    args: argparse.Namespace,
) -> ScenePerceptionResult:
    """Build the runtime ScenePerceptionResult from an offline scene capture."""

    if args.scene_dir is None:
        raise ValueError(
            "--scene-dir is required for runtime scene tests."
        )

    if not args.model_config:
        raise ValueError(
            "--model-config is required for runtime scene tests."
        )

    if args.base_to_table_transform is None:
        raise ValueError(
            "--base-to-table-transform is required for runtime scene tests."
        )

    scene_dir = (
        Path(args.scene_dir)
        .expanduser()
        .resolve()
    )

    model_config_path = (
        Path(args.model_config)
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
    camera_info_path = scene_dir / "camera_info.yaml"

    required_paths = (
        rgb_path,
        depth_path,
        camera_info_path,
        model_config_path,
        transform_path,
    )

    for path in required_paths:
        if not path.is_file():
            raise FileNotFoundError(
                f"Required test input does not exist: {path}"
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
        rgb_image=rgb,
        depth_image=depth,
        camera_info=camera_info,
        base_to_table_transform=(
            base_to_table_transform
        ),
        artifacts_dir=artifacts_dir,
    )

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
                "ScenePerceiver did not return an overlay path."
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

    print("\n=== RAW SCENE STATE ===")
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

    print("\nTEST PASSED")

    return 0

def run_scene_interpreter_test(
    args: argparse.Namespace,
) -> int:
    if not args.model_config:
        raise ValueError(
            "--model-config is required for "
            "the scene_interpreter test."
        )

    if args.artifacts_dir is None:
        raise ValueError(
            "--artifacts-dir is required for "
            "the scene_interpreter test."
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

    artifacts_dir = (
        Path(args.artifacts_dir)
        .expanduser()
        .resolve()
    )

    artifacts_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    # First run the real perception stage.
    #
    # Its raw_scene_overlay.png and raw_scene_state.json
    # are written to the same test artifact directory.
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
            "ScenePerceiver returned no objects "
            "before scene interpretation."
        )

    if (
        perception_result.overlay_image_path
        is None
    ):
        raise AssertionError(
            "ScenePerceiver did not produce "
            "a raw scene overlay."
        )

    if (
        not perception_result
        .overlay_image_path
        .is_file()
    ):
        raise AssertionError(
            "Raw scene overlay does not exist: "
            f"{perception_result.overlay_image_path}"
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
        artifacts_dir=artifacts_dir,
    )

    # ------------------------------------------------------------
    # Structural checks
    # ------------------------------------------------------------

    if len(scene_state.objects) != len(
        raw_scene.objects
    ):
        raise AssertionError(
            "SceneInterpreter changed the number of objects. "
            f"Raw={len(raw_scene.objects)}, "
            f"semantic={len(scene_state.objects)}."
        )

    semantic_ids = [
        obj.object_id
        for obj in scene_state.objects
    ]

    if len(semantic_ids) != len(
        set(semantic_ids)
    ):
        raise AssertionError(
            "SceneInterpreter produced duplicate "
            "semantic object IDs."
        )

    for raw_obj, scene_obj in zip(
        raw_scene.objects,
        scene_state.objects,
        strict=True,
    ):
        if raw_obj.label != scene_obj.label:
            raise AssertionError(
                f"{raw_obj.object_id}: detector label changed "
                f"from '{raw_obj.label}' "
                f"to '{scene_obj.label}'."
            )

        if (
            raw_obj.pixel_coordinates
            != scene_obj.pixel_coordinates
        ):
            raise AssertionError(
                f"{raw_obj.object_id}: "
                "pixel coordinates changed during "
                "semantic interpretation."
            )

        if (
            raw_obj.position_camera
            != scene_obj.position_camera
        ):
            raise AssertionError(
                f"{raw_obj.object_id}: "
                "camera-space position changed during "
                "semantic interpretation."
            )

        if (
            raw_obj.position_base
            != scene_obj.position_base
        ):
            raise AssertionError(
                f"{raw_obj.object_id}: "
                "base-link position changed during "
                "semantic interpretation."
            )

        if not scene_obj.object_id.strip():
            raise AssertionError(
                f"{raw_obj.object_id}: "
                "semantic object ID is empty."
            )

    # ------------------------------------------------------------
    # Artifact checks
    # ------------------------------------------------------------

    expected_artifacts = (
        artifacts_dir
        / "raw_scene_overlay.png",

        artifacts_dir
        / "raw_scene_state.json",

        artifacts_dir
        / "scene_interpretation.json",

        artifacts_dir
        / "scene_state.json",
    )

    for artifact_path in expected_artifacts:
        if not artifact_path.is_file():
            raise AssertionError(
                f"Missing test artifact: {artifact_path}"
            )

        if artifact_path.stat().st_size == 0:
            raise AssertionError(
                f"Empty test artifact: {artifact_path}"
            )

    # ------------------------------------------------------------
    # Human-readable output
    # ------------------------------------------------------------

    print(
        "\n=== SEMANTIC SCENE STATE ==="
    )

    print(
        f"Objects: {len(scene_state.objects)}"
    )

    for obj in scene_state.objects:
        print()
        print(
            f"Semantic ID:      "
            f"{obj.object_id}"
        )
        print(
            f"Detector label:   "
            f"{obj.label}"
        )
        print(
            f"Pixel:            "
            f"{obj.pixel_coordinates}"
        )
        print(
            f"Camera position:  "
            f"{obj.position_camera}"
        )
        print(
            f"Base position:    "
            f"{obj.position_base}"
        )

    print(
        "\nArtifacts:"
    )

    for artifact_path in expected_artifacts:
        print(
            f"  {artifact_path}"
        )

    print("\nTEST PASSED")

    return 0

def run_seedo_controller_test(
        args: argparse.Namespace,
    ) -> int:

        if args.video is None:
            raise ValueError(
                "--video is required for the integration test."
            )
        
        if not args.model_config:
            raise ValueError(
                "--model-config is required for the seedo_controller test."
            )

        controller = SeeDoController(
            model_config=args.model_config,
        )

        controller.load_command(
            demo_path=args.video,
            task_id="test",
            artifacts_dir=args.artifacts_dir,
        )

        result = controller.inference(
            input_data={
                "video_path": args.video,
                "artifacts_dir": args.artifacts_dir,
            },
            t=0,
        )

        if result is not controller.last_result:
            raise AssertionError(
                "SeeDoController.last_result was not updated correctly."
            )

        if result.status not in (
            "completed",
            "ambiguous",
        ):
            raise AssertionError(
                f"Unexpected status: {result.status}"
            )

        if (
            result.status == "completed"
            and len(result.steps) != 1
        ):
            raise AssertionError(
                "A completed plan must contain exactly one action step."
            )

        if (
            result.status == "ambiguous"
            and not result.ambiguities
        ):
            raise AssertionError(
                "An ambiguous plan must explain its ambiguities."
            )

        if not result.natural_language_plan.strip():
            raise AssertionError(
                "Natural-language plan is empty."
            )

        print("SeeDo controller pipeline completed")
        print(f"Status: {result.status}")
        print(
            "Natural-language plan: "
            f"{result.natural_language_plan}"
        )

        print("\nSteps:")
        for index, step in enumerate(
            result.steps,
            start=1,
        ):
            print(f"  Step {index}: {step}")

        print("\nAmbiguities:")
        for ambiguity in result.ambiguities:
            print(f"  - {ambiguity}")

        controller.reset()

        if controller.last_result is not None:
            raise AssertionError(
                "SeeDoController.reset() did not clear last_result."
            )

        print("\nTEST PASSED")
        return 0

if __name__ == "__main__":
    raise SystemExit(main())