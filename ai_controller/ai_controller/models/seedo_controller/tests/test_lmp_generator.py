from __future__ import annotations

import argparse
from pathlib import Path

import torch
import yaml

from ai_controller.models.seedo_controller.lmp_generator import LMPGenerator
from .common import build_action_planning_result, build_scene_state

def run_lmp_generator_test(
    args: argparse.Namespace,
) -> int:
    """Integration test from demo interpretation to CAP primitive generation."""

    if args.artifacts_dir is None:
        raise ValueError(
            "--artifacts-dir is required for "
            "the lmp_generator test."
        )

    if not args.model_config:
        raise ValueError(
            "--model-config is required for "
            "the lmp_generator test."
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

    upstream_artifacts_dir = (
        artifacts_dir
        / "_upstream"
    )

    upstream_artifacts_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    # ------------------------------------------------------------
    # 1. Demo-side planning
    # ------------------------------------------------------------

    print(
        "\n=== BUILDING ACTION PLAN ==="
    )

    action_plan = (
        build_action_planning_result(
            args=args,
            artifacts_dir=(
                upstream_artifacts_dir
            ),
        )
    )

    print(
        "CUDA autocast active after VisualPrompter:",
        torch.is_autocast_enabled("cuda"),
    )

    print(
        "Action-plan status: "
        f"{action_plan.status}"
    )

    print(
        "Natural-language plan: "
        f"{action_plan.natural_language_plan}"
    )

    # ------------------------------------------------------------
    # 2. Runtime scene understanding
    # ------------------------------------------------------------

    print(
        "\n=== BUILDING SEMANTIC SCENE ==="
    )

    scene_state = build_scene_state(
        args=args,
        artifacts_dir=(
            upstream_artifacts_dir
        ),
    )

    print(
        f"Scene objects: "
        f"{len(scene_state.objects)}"
    )

    for obj in scene_state.objects:
        print(
            f"  - {obj.object_id} "
            f"[{obj.label}] "
            f"@ {obj.position_base}"
        )

    # ------------------------------------------------------------
    # 3. Configuration
    # ------------------------------------------------------------

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

    lmp_config = config.get(
        "lmp_generator",
        {},
    )

    perception_config = config.get(
        "scene_perceiver",
        {},
    )

    if "workspace_bottom_left" not in perception_config:
        raise KeyError(
            "Missing scene_perceiver.workspace_bottom_left "
            "in model configuration."
        )

    if "workspace_top_right" not in perception_config:
        raise KeyError(
            "Missing scene_perceiver.workspace_top_right "
            "in model configuration."
        )

    workspace_bottom_left = tuple(
        float(value)
        for value in perception_config[
            "workspace_bottom_left"
        ]
    )

    workspace_top_right = tuple(
        float(value)
        for value in perception_config[
            "workspace_top_right"
        ]
    )

    # ------------------------------------------------------------
    # 4. Real CAP / LMP generation
    # ------------------------------------------------------------

    print(
        "\n=== RUNNING LMP GENERATOR ==="
    )

    generator = LMPGenerator(
        model=lmp_config.get(
            "model",
            "gpt-4o-2024-08-06",
        ),
    )

    primitive_plan = generator.run(
        action_plan=action_plan,
        scene_state=scene_state,
        workspace_bottom_left=(
            workspace_bottom_left
        ),
        workspace_top_right=(
            workspace_top_right
        ),
        artifacts_dir=artifacts_dir,
    )

    # ------------------------------------------------------------
    # 5. Structural validation
    # ------------------------------------------------------------

    if not primitive_plan.steps:
        raise AssertionError(
            "LMPGenerator returned an empty PrimitivePlan."
        )

    if not primitive_plan.source_code.strip():
        raise AssertionError(
            "LMPGenerator returned empty CAP source code."
        )

    allowed_primitives = {
        "reach",
        "approaching",
        "pick",
        "lift_up",
        "moving",
        "placing",
    }

    scene_object_ids = {
        obj.object_id
        for obj in scene_state.objects
    }

    for index, step in enumerate(
        primitive_plan.steps,
        start=1,
    ):
        if step.name not in allowed_primitives:
            raise AssertionError(
                "CAP generated an unsupported primitive "
                f"at step {index}: '{step.name}'."
            )

        if "target" not in step.arguments:
            raise AssertionError(
                "Primitive step does not contain a target: "
                f"{step}"
            )

        target = step.arguments[
            "target"
        ]

        if not isinstance(target, str):
            raise AssertionError(
                "Primitive target must be a string: "
                f"{step}"
            )

        if target not in scene_object_ids:
            raise AssertionError(
                "CAP generated a primitive targeting an object "
                "that does not exist in SceneState: "
                f"'{target}'."
            )

    # ------------------------------------------------------------
    # 6. Pick/place sequence sanity checks
    # ------------------------------------------------------------

    primitive_names = [
        step.name
        for step in primitive_plan.steps
    ]

    if "pick" not in primitive_names:
        raise AssertionError(
            "PrimitivePlan does not contain a pick primitive."
        )

    if "placing" not in primitive_names:
        raise AssertionError(
            "PrimitivePlan does not contain a placing primitive."
        )

    first_pick_index = (
        primitive_names.index(
            "pick"
        )
    )

    first_placing_index = (
        primitive_names.index(
            "placing"
        )
    )

    if first_pick_index >= first_placing_index:
        raise AssertionError(
            "The first placing primitive occurs before "
            "the first pick primitive."
        )

    # ------------------------------------------------------------
    # 7. Artifact validation
    # ------------------------------------------------------------

    generated_program_path = (
        artifacts_dir
        / "generated_program.py"
    )

    primitive_plan_path = (
        artifacts_dir
        / "primitive_plan.json"
    )

    required_artifacts = (
        generated_program_path,
        primitive_plan_path,
    )

    for artifact_path in required_artifacts:
        if not artifact_path.is_file():
            raise AssertionError(
                "Missing LMPGenerator artifact: "
                f"{artifact_path}"
            )

        if artifact_path.stat().st_size == 0:
            raise AssertionError(
                "Empty LMPGenerator artifact: "
                f"{artifact_path}"
            )

    generated_program = (
        generated_program_path
        .read_text(
            encoding="utf-8",
        )
        .strip()
    )

    if (
        generated_program
        != primitive_plan.source_code.strip()
    ):
        raise AssertionError(
            "generated_program.py does not match "
            "PrimitivePlan.source_code."
        )

    # ------------------------------------------------------------
    # 8. Human-readable result
    # ------------------------------------------------------------

    print(
        "\n=== PRIMITIVE PLAN ==="
    )

    print(
        f"Primitive count: "
        f"{len(primitive_plan.steps)}"
    )

    for index, step in enumerate(
        primitive_plan.steps,
        start=1,
    ):
        print(
            f"  {index}. "
            f"{step.name}"
            f"({step.arguments})"
        )

    print(
        "\n=== GENERATED CAP PROGRAM ==="
    )

    print(
        primitive_plan.source_code
    )

    print(
        "\nArtifacts:"
    )

    for artifact_path in required_artifacts:
        print(
            f"  {artifact_path}"
        )

    print(
        "\nTEST PASSED"
    )

    return 0
