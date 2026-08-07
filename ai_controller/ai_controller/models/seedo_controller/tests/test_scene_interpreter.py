from __future__ import annotations

import argparse
from pathlib import Path

import yaml

from ai_controller.models.seedo_controller.scene_interpreter import SceneInterpreter
from .common import build_scene_perception_result

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
