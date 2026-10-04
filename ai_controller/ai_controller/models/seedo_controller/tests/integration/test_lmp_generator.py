from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

import yaml

from results import (
    ActionPlanningResult,
    ActionStep,
    ReplicabilityResult,
    ResolvedActionTargets,
    SceneObject,
    SceneState,
)

from ai_controller.models.seedo_controller.lmp_generator import (
    LMPGenerator,
)


DEFAULT_ACTION_PLAN = Path(
    "/seedo_tests/action_planning/action_plan.json"
)

DEFAULT_SCENE_STATE = Path(
    "/seedo_tests/scene_interpreter/scene_state.json"
)

DEFAULT_REPLICABILITY_RESULT = Path(
    "/seedo_tests/replicability_checker/replicability_result.json"
)

EXPECTED_PICK_ID = "green_block_0"
EXPECTED_PLACE_ID = "storage_bin_0"

EXPECTED_PRIMITIVE_NAMES = (
    "reach",
    "approaching",
    "pick",
    "lift_up",
    "moving",
    "placing",
)


def _load_json(
    path: Path,
) -> dict:
    normalized_path = (
        Path(path)
        .expanduser()
        .resolve()
    )

    if not normalized_path.is_file():
        raise FileNotFoundError(
            "Required integration artifact does not exist: "
            f"{normalized_path}"
        )

    if normalized_path.stat().st_size == 0:
        raise ValueError(
            "Required integration artifact is empty: "
            f"{normalized_path}"
        )

    with normalized_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        value = json.load(
            stream
        )

    if not isinstance(
        value,
        dict,
    ):
        raise ValueError(
            f"Expected a JSON object in {normalized_path}."
        )

    return value


def _load_action_plan(
    path: Path,
) -> ActionPlanningResult:
    data = _load_json(
        path
    )

    steps_data = data.get(
        "steps"
    )

    if not isinstance(
        steps_data,
        list,
    ):
        raise ValueError(
            "action_plan.json does not contain a steps list."
        )

    steps = []

    for index, item in enumerate(
        steps_data
    ):
        if not isinstance(
            item,
            dict,
        ):
            raise ValueError(
                f"Invalid action step at index {index}: "
                f"{item!r}"
            )

        steps.append(
            ActionStep(
                pick_keyframe=int(
                    item["pick_keyframe"]
                ),
                place_keyframe=int(
                    item["place_keyframe"]
                ),
                picked_track_id=int(
                    item["picked_track_id"]
                ),
                picked_category=str(
                    item["picked_category"]
                ),
                picked_color=str(
                    item["picked_color"]
                ),
                destination_track_id=int(
                    item["destination_track_id"]
                ),
                destination_category=str(
                    item["destination_category"]
                ),
                destination_ordinal_from_left=(
                    None
                    if item.get(
                        "destination_ordinal_from_left"
                    ) is None
                    else int(
                        item[
                            "destination_ordinal_from_left"
                        ]
                    )
                ),
                relation=str(
                    item["relation"]
                ),
                action=str(
                    item["action"]
                ),
                picked_detector_label=str(
                    item.get(
                        "picked_detector_label",
                        "",
                    )
                ),
            )
        )

    ambiguities = data.get(
        "ambiguities",
        [],
    )

    if not isinstance(
        ambiguities,
        list,
    ):
        raise ValueError(
            "Action-plan ambiguities must be a list."
        )

    return ActionPlanningResult(
        steps=tuple(
            steps
        ),
        status=str(
            data.get(
                "status",
                "",
            )
        ),
        ambiguities=tuple(
            str(item)
            for item
            in ambiguities
        ),
        natural_language_plan=str(
            data.get(
                "natural_language_plan",
                "",
            )
        ),
        task_type=data.get(
            "task_type"
        ),
    )


def _load_scene_state(
    path: Path,
) -> SceneState:
    data = _load_json(
        path
    )

    objects_data = data.get(
        "objects"
    )

    if (
        not isinstance(
            objects_data,
            list,
        )
        or not objects_data
    ):
        raise ValueError(
            "scene_state.json must contain a non-empty objects list."
        )

    objects = []

    for index, item in enumerate(
        objects_data
    ):
        if not isinstance(
            item,
            dict,
        ):
            raise ValueError(
                f"Invalid scene object at index {index}: "
                f"{item!r}"
            )

        pixel = item.get(
            "pixel_coordinates"
        )

        position_camera = item.get(
            "position_camera"
        )

        position_base = item.get(
            "position_base"
        )

        attributes = item.get(
            "attributes",
            {},
        )

        if (
            not isinstance(
                pixel,
                list,
            )
            or len(pixel) != 2
        ):
            raise ValueError(
                f"Invalid pixel_coordinates at index {index}: "
                f"{pixel!r}"
            )

        if (
            not isinstance(
                position_camera,
                list,
            )
            or len(position_camera) != 3
        ):
            raise ValueError(
                f"Invalid position_camera at index {index}: "
                f"{position_camera!r}"
            )

        if (
            not isinstance(
                position_base,
                list,
            )
            or len(position_base) != 3
        ):
            raise ValueError(
                f"Invalid position_base at index {index}: "
                f"{position_base!r}"
            )

        if not isinstance(
            attributes,
            dict,
        ):
            raise ValueError(
                "SceneObject attributes must be a dictionary."
            )

        objects.append(
            SceneObject(
                object_id=str(
                    item["object_id"]
                ),
                label=str(
                    item["label"]
                ),
                pixel_coordinates=(
                    int(
                        pixel[0]
                    ),
                    int(
                        pixel[1]
                    ),
                ),
                position_camera=tuple(
                    float(value)
                    for value
                    in position_camera
                ),
                position_base=tuple(
                    float(value)
                    for value
                    in position_base
                ),
                category=(
                    None
                    if item.get(
                        "category"
                    ) is None
                    else str(
                        item[
                            "category"
                        ]
                    )
                ),
                attributes=dict(
                    attributes
                ),
            )
        )

    return SceneState(
        objects=tuple(
            objects
        )
    )


def _load_replicability_result(
    path: Path,
) -> ReplicabilityResult:
    data = _load_json(
        path
    )

    resolved_data = data.get(
        "resolved_targets"
    )

    if not isinstance(
        resolved_data,
        list,
    ):
        raise ValueError(
            "replicability_result.json does not contain "
            "a resolved_targets list."
        )

    resolved_targets = []

    for index, item in enumerate(
        resolved_data
    ):
        if not isinstance(
            item,
            dict,
        ):
            raise ValueError(
                "Invalid resolved target at index "
                f"{index}: {item!r}"
            )

        resolved_targets.append(
            ResolvedActionTargets(
                action_step_index=int(
                    item[
                        "action_step_index"
                    ]
                ),
                runtime_pick_object_id=str(
                    item[
                        "runtime_pick_object_id"
                    ]
                ),
                runtime_place_object_id=str(
                    item[
                        "runtime_place_object_id"
                    ]
                ),
            )
        )

    failure_reasons = data.get(
        "failure_reasons",
        [],
    )

    if not isinstance(
        failure_reasons,
        list,
    ):
        raise ValueError(
            "Replicability failure_reasons must be a list."
        )

    return ReplicabilityResult(
        replicable=bool(
            data.get(
                "replicable",
                False,
            )
        ),
        resolved_targets=tuple(
            resolved_targets
        ),
        failure_reasons=tuple(
            str(reason)
            for reason
            in failure_reasons
        ),
    )


def run_lmp_generator_test(
    args: argparse.Namespace,
) -> int:
    """Run real generalized CAP/LMP generation from validated handoff artifacts."""

    if args.artifacts_dir is None:
        raise ValueError(
            "--artifacts-dir is required for the "
            "lmp_generator integration test."
        )

    if not args.model_config:
        raise ValueError(
            "--model-config is required for the "
            "lmp_generator integration test."
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

    action_plan_path = (
        DEFAULT_ACTION_PLAN
    )

    scene_state_path = (
        DEFAULT_SCENE_STATE
    )

    replicability_path = (
        DEFAULT_REPLICABILITY_RESULT
    )

    action_plan = _load_action_plan(
        action_plan_path
    )

    scene_state = _load_scene_state(
        scene_state_path
    )

    replicability_result = (
        _load_replicability_result(
            replicability_path
        )
    )

    # ---------------------------------------------------------
    # Validate upstream handoffs.
    # ---------------------------------------------------------

    if action_plan.status != "completed":
        raise AssertionError(
            "LMP integration requires a completed action plan, "
            f"received {action_plan.status!r}."
        )

    if (
        str(
            action_plan.task_type
        ).strip().lower()
        != "pick_and_place"
    ):
        raise AssertionError(
            "Unexpected task type in action-plan handoff: "
            f"{action_plan.task_type!r}"
        )

    if len(
        action_plan.steps
    ) != 1:
        raise AssertionError(
            "Expected exactly one action step, "
            f"received {len(action_plan.steps)}."
        )

    action_step = (
        action_plan.steps[0]
    )

    if (
        action_step.relation
        .strip()
        .lower()
        != "in"
    ):
        raise AssertionError(
            "Unexpected demonstrated relation: "
            f"{action_step.relation!r}"
        )

    if not replicability_result.replicable:
        raise AssertionError(
            "LMP integration requires a replicable task, "
            f"failure_reasons={replicability_result.failure_reasons}"
        )

    if replicability_result.failure_reasons:
        raise AssertionError(
            "Replicable handoff contains failure reasons: "
            f"{replicability_result.failure_reasons}"
        )

    if len(
        replicability_result.resolved_targets
    ) != 1:
        raise AssertionError(
            "Expected exactly one resolved runtime target pair, "
            f"received {len(replicability_result.resolved_targets)}."
        )

    resolved = (
        replicability_result
        .resolved_targets[0]
    )

    if (
        resolved.action_step_index
        != 0
    ):
        raise AssertionError(
            "Unexpected resolved action-step index: "
            f"{resolved.action_step_index}"
        )

    if (
        resolved.runtime_pick_object_id
        != EXPECTED_PICK_ID
    ):
        raise AssertionError(
            "Unexpected resolved pick target: "
            f"{resolved.runtime_pick_object_id!r}"
        )

    if (
        resolved.runtime_place_object_id
        != EXPECTED_PLACE_ID
    ):
        raise AssertionError(
            "Unexpected resolved place target: "
            f"{resolved.runtime_place_object_id!r}"
        )

    scene_object_ids = {
        obj.object_id
        for obj
        in scene_state.objects
    }

    if (
        EXPECTED_PICK_ID
        not in scene_object_ids
    ):
        raise AssertionError(
            "Resolved pick target is absent from SceneState."
        )

    if (
        EXPECTED_PLACE_ID
        not in scene_object_ids
    ):
        raise AssertionError(
            "Resolved place target is absent from SceneState."
        )

    # ---------------------------------------------------------
    # Load LMP/workspace configuration.
    # ---------------------------------------------------------

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

    if not isinstance(
        config,
        dict,
    ):
        raise ValueError(
            "SeeDo controller configuration must contain a YAML mapping."
        )

    perception_mode = str(
        config.get(
            "perception_mode",
            "generalized",
        )
    ).strip().lower()

    if perception_mode != "generalized":
        raise AssertionError(
            "This integration test requires generalized mode, "
            f"received {perception_mode!r}."
        )

    lmp_config = config.get(
        "lmp_generator",
        {},
    )

    perception_config = config.get(
        "scene_perceiver",
        {},
    )

    if not isinstance(
        lmp_config,
        dict,
    ):
        raise ValueError(
            "lmp_generator configuration must be a mapping."
        )

    if not isinstance(
        perception_config,
        dict,
    ):
        raise ValueError(
            "scene_perceiver configuration must be a mapping."
        )

    if (
        "workspace_bottom_left"
        not in perception_config
    ):
        raise KeyError(
            "Missing scene_perceiver.workspace_bottom_left "
            "in model configuration."
        )

    if (
        "workspace_top_right"
        not in perception_config
    ):
        raise KeyError(
            "Missing scene_perceiver.workspace_top_right "
            "in model configuration."
        )

    workspace_bottom_left = tuple(
        float(value)
        for value
        in perception_config[
            "workspace_bottom_left"
        ]
    )

    workspace_top_right = tuple(
        float(value)
        for value
        in perception_config[
            "workspace_top_right"
        ]
    )

    if len(
        workspace_bottom_left
    ) != 2:
        raise AssertionError(
            "workspace_bottom_left must contain two coordinates."
        )

    if len(
        workspace_top_right
    ) != 2:
        raise AssertionError(
            "workspace_top_right must contain two coordinates."
        )

    # ---------------------------------------------------------
    # Run real generalized CAP / LMP generation.
    # ---------------------------------------------------------

    generator = LMPGenerator(
        model=lmp_config.get(
            "model",
            "gpt-4o-2024-08-06",
        ),
        perception_mode=(
            perception_mode
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
        replicability_result=(
            replicability_result
        ),
        artifacts_dir=artifacts_dir,
    )

    # ---------------------------------------------------------
    # Validate CAP output.
    # ---------------------------------------------------------

    if not primitive_plan.steps:
        raise AssertionError(
            "LMPGenerator returned an empty PrimitivePlan."
        )

    if not primitive_plan.source_code.strip():
        raise AssertionError(
            "LMPGenerator returned empty CAP source code."
        )

    primitive_names = tuple(
        step.name
        for step
        in primitive_plan.steps
    )

    if primitive_names != EXPECTED_PRIMITIVE_NAMES:
        raise AssertionError(
            "Unexpected generalized pick-and-place primitive sequence: "
            f"expected={EXPECTED_PRIMITIVE_NAMES}, "
            f"received={primitive_names}"
        )

    expected_targets = (
        EXPECTED_PICK_ID,
        EXPECTED_PICK_ID,
        EXPECTED_PICK_ID,
        EXPECTED_PICK_ID,
        EXPECTED_PLACE_ID,
        EXPECTED_PLACE_ID,
    )

    returned_targets = []

    for index, (
        primitive_step,
        expected_name,
        expected_target,
    ) in enumerate(
        zip(
            primitive_plan.steps,
            EXPECTED_PRIMITIVE_NAMES,
            expected_targets,
            strict=True,
        ),
        start=1,
    ):
        if (
            primitive_step.name
            != expected_name
        ):
            raise AssertionError(
                "Unexpected primitive at step "
                f"{index}: expected={expected_name!r}, "
                f"received={primitive_step.name!r}"
            )

        if (
            set(
                primitive_step.arguments
            )
            != {"target"}
        ):
            raise AssertionError(
                "Primitive arguments must contain only the target "
                f"at step {index}: {primitive_step.arguments}"
            )

        target = (
            primitive_step.arguments[
                "target"
            ]
        )

        if not isinstance(
            target,
            str,
        ):
            raise AssertionError(
                "Primitive target must be a string at step "
                f"{index}: {target!r}"
            )

        if target != expected_target:
            raise AssertionError(
                "CAP targeted the wrong runtime object at step "
                f"{index}: expected={expected_target!r}, "
                f"received={target!r}"
            )

        if target not in scene_object_ids:
            raise AssertionError(
                "CAP generated a primitive targeting an object "
                f"absent from SceneState: {target!r}"
            )

        returned_targets.append(
            target
        )

    if set(
        returned_targets
    ) != {
        EXPECTED_PICK_ID,
        EXPECTED_PLACE_ID,
    }:
        raise AssertionError(
            "CAP used runtime objects outside the resolved "
            "ReplicabilityResult targets."
        )

    # The generalized LMP must act on runtime IDs rather than
    # demo-frame coordinates or the demonstration natural-language plan.
    source_code = (
        primitive_plan
        .source_code
        .strip()
    )

    if (
        EXPECTED_PICK_ID
        not in source_code
    ):
        raise AssertionError(
            "Generated CAP program does not reference the resolved "
            "runtime pick ID."
        )

    if (
        EXPECTED_PLACE_ID
        not in source_code
    ):
        raise AssertionError(
            "Generated CAP program does not reference the resolved "
            "runtime place ID."
        )

    if (
        "(x=203, y=303)"
        in source_code
    ):
        raise AssertionError(
            "Generated CAP program leaked demonstration-frame "
            "destination coordinates."
        )

    if (
        action_plan.natural_language_plan
        in source_code
    ):
        raise AssertionError(
            "Generated CAP program reused the demonstration "
            "natural-language plan instead of resolved runtime IDs."
        )

    # ---------------------------------------------------------
    # Validate persistent artifacts.
    # ---------------------------------------------------------

    generated_program_path = (
        artifacts_dir
        / "generated_program.py"
    )

    primitive_plan_path = (
        artifacts_dir
        / "primitive_plan.json"
    )

    for artifact_path in (
        generated_program_path,
        primitive_plan_path,
    ):
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
        != source_code
    ):
        raise AssertionError(
            "generated_program.py does not match "
            "PrimitivePlan.source_code."
        )

    with primitive_plan_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        primitive_plan_json = (
            json.load(
                stream
            )
        )

    expected_primitive_plan_json = (
        json.loads(
            json.dumps(
                asdict(
                    primitive_plan
                ),
                ensure_ascii=False,
            )
        )
    )

    if (
        primitive_plan_json
        != expected_primitive_plan_json
    ):
        raise AssertionError(
            "primitive_plan.json does not match "
            "the returned PrimitivePlan."
        )

    # ---------------------------------------------------------
    # Report.
    # ---------------------------------------------------------

    print(
        "LMP generation completed"
    )
    print(
        "Action plan: "
        f"{action_plan_path}"
    )
    print(
        "SceneState: "
        f"{scene_state_path}"
    )
    print(
        "Replicability result: "
        f"{replicability_path}"
    )
    print(
        "Perception mode: "
        f"{perception_mode}"
    )
    print(
        "Resolved instruction targets: "
        f"pick={EXPECTED_PICK_ID}, "
        f"place={EXPECTED_PLACE_ID}"
    )
    print(
        "Primitive count: "
        f"{len(primitive_plan.steps)}"
    )

    print(
        "\nPrimitive plan:"
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
        "\nGenerated CAP program:"
    )
    print(
        primitive_plan.source_code
    )

    print(
        "\nArtifacts:"
    )
    print(
        f"  {generated_program_path}"
    )
    print(
        f"  {primitive_plan_path}"
    )

    print(
        "\nTEST PASSED"
    )

    return 0
