from __future__ import annotations

import argparse
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
import yaml

from ai_controller.utils.utils import (
    EEF_POS_NAME,
    EEF_QUAT_NAME,
)
from ai_controller.models.seedo_controller.seedo_controller import (
    SeeDoController,
)

from ..common import (
    load_scene_runtime_input,
)

from .test_seedo_controller import (
    INITIAL_POSITION,
    INITIAL_ORIENTATION,
    _validate_canonical_inputs,
    _require_file,
    _validate_motion_artifact,
)


EXPECTED_RUNTIME_OBJECT_IDS = {
    "red cube",
    "green cube",
    "blue cube",
    "yellow cube",
    "first bin from the left",
    "second bin from the left",
    "third bin from the left",
    "fourth bin from the left",
}


EXPECTED_PRIMITIVES = (
    ("reach", "green cube"),
    ("approaching", "green cube"),
    ("pick", "green cube"),
    ("lift_up", "green cube"),
    ("moving", "first bin from the left"),
    ("placing", "first bin from the left"),
)


def _normalize_task_type(
    value,
) -> str:
    raw_value = getattr(
        value,
        "value",
        value,
    )

    return str(
        raw_value
    ).strip().lower()


def _create_prior_guided_config(
    source_config: str | Path,
    output_dir: Path,
) -> Path:
    source_path = (
        Path(source_config)
        .expanduser()
        .resolve()
    )

    if not source_path.is_file():
        raise FileNotFoundError(
            "SeeDo model configuration does not exist: "
            f"{source_path}"
        )

    with source_path.open(
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
            "SeeDo model configuration must contain "
            "a YAML mapping."
        )

    config["perception_mode"] = (
        "prior_guided"
    )

    output_path = (
        output_dir
        / "seedo_controller_prior_guided.yaml"
    )

    with output_path.open(
        "w",
        encoding="utf-8",
    ) as stream:
        yaml.safe_dump(
            config,
            stream,
            sort_keys=False,
        )

    return output_path


def _validate_demo_pipeline(
    controller: SeeDoController,
) -> None:
    if (
        controller.perception_mode
        != "prior_guided"
    ):
        raise AssertionError(
            "The controller is not running in "
            "prior_guided mode."
        )

    if controller.action_plan is None:
        raise AssertionError(
            "SeeDoController did not generate "
            "an ActionPlanningResult."
        )

    if (
        controller.execution_status
        != "plan_ready"
    ):
        raise AssertionError(
            "Unexpected controller status after "
            "load_command(): "
            f"{controller.execution_status}"
        )

    # ---------------------------------------------------------
    # The structural generalized branch must remain disabled.
    # ---------------------------------------------------------

    if (
        controller.demo_structured_scene
        is not None
    ):
        raise AssertionError(
            "prior_guided unexpectedly generated a "
            "demonstration StructuredScene."
        )

    if (
        controller.runtime_structured_scene
        is not None
    ):
        raise AssertionError(
            "Runtime StructuredScene must not exist "
            "before runtime perception."
        )

    if (
        controller.structural_matching_result
        is not None
    ):
        raise AssertionError(
            "prior_guided unexpectedly contains a "
            "StructuralMatchingResult."
        )

    if (
        controller.replicability_result
        is not None
    ):
        raise AssertionError(
            "prior_guided unexpectedly contains a "
            "ReplicabilityResult."
        )

    action_plan = (
        controller.action_plan
    )

    if (
        action_plan.status
        != "completed"
    ):
        raise AssertionError(
            "Expected a completed prior-guided "
            "demonstration plan, received "
            f"{action_plan.status!r}. "
            f"Ambiguities={action_plan.ambiguities}"
        )

    if (
        _normalize_task_type(
            action_plan.task_type
        )
        != "pick_and_place"
    ):
        raise AssertionError(
            "Unexpected prior-guided task type: "
            f"{action_plan.task_type!r}"
        )

    if (
        len(
            action_plan.steps
        )
        != 1
    ):
        raise AssertionError(
            "Expected exactly one prior-guided "
            "action step, received "
            f"{len(action_plan.steps)}."
        )

    step = (
        action_plan.steps[0]
    )

    # ---------------------------------------------------------
    # Legacy prior-guided semantics.
    # ---------------------------------------------------------

    if (
        step.picked_detector_label
        .strip()
        .lower()
        != "green cube"
    ):
        raise AssertionError(
            "Unexpected picked detector label: "
            f"{step.picked_detector_label!r}"
        )

    if (
        step.picked_category
        .strip()
        .lower()
        != "cube"
    ):
        raise AssertionError(
            "Unexpected picked category: "
            f"{step.picked_category!r}"
        )

    if (
        step.picked_color
        .strip()
        .lower()
        != "green"
    ):
        raise AssertionError(
            "Unexpected picked color: "
            f"{step.picked_color!r}"
        )

    if (
        "bin"
        not in step.destination_category
        .strip()
        .lower()
    ):
        raise AssertionError(
            "Unexpected destination category: "
            f"{step.destination_category!r}"
        )

    if (
        step.destination_ordinal_from_left
        != 1
    ):
        raise AssertionError(
            "The canonical prior-guided demonstration "
            "must target the first bin from the left. "
            "Received ordinal: "
            f"{step.destination_ordinal_from_left!r}"
        )

    if (
        step.relation
        .strip()
        .lower()
        != "in"
    ):
        raise AssertionError(
            "Unexpected placement relation: "
            f"{step.relation!r}"
        )

    if (
        action_plan.natural_language_plan
        != step.action
    ):
        raise AssertionError(
            "Prior-guided natural_language_plan must "
            "preserve the original ActionPlanner action."
        )

    normalized_plan = (
        action_plan
        .natural_language_plan
        .strip()
        .lower()
    )

    required_fragments = (
        "green cube",
        "first",
        "from the left",
    )

    for fragment in required_fragments:
        if fragment not in normalized_plan:
            raise AssertionError(
                "Prior-guided natural-language plan "
                "does not preserve legacy semantics. "
                f"Missing {fragment!r}: "
                f"{action_plan.natural_language_plan!r}"
            )


def _validate_runtime_pipeline(
    controller: SeeDoController,
) -> None:
    if controller.perception_result is None:
        raise AssertionError(
            "SeeDoController did not generate a "
            "ScenePerceptionResult."
        )

    if controller.scene_state is None:
        raise AssertionError(
            "SeeDoController did not generate "
            "a SceneState."
        )

    if controller.primitive_plan is None:
        raise AssertionError(
            "SeeDoController did not generate "
            "a PrimitivePlan."
        )

    if (
        controller.execution_status
        != "plan_ready"
    ):
        raise AssertionError(
            "Unexpected controller status after "
            "inference(t=0): "
            f"{controller.execution_status}"
        )

    # ---------------------------------------------------------
    # Generalized structural stages must remain skipped.
    # ---------------------------------------------------------

    if (
        controller.runtime_structured_scene
        is not None
    ):
        raise AssertionError(
            "prior_guided unexpectedly generated "
            "a runtime StructuredScene."
        )

    if (
        controller.structural_matching_result
        is not None
    ):
        raise AssertionError(
            "prior_guided unexpectedly executed "
            "StructuralMatcher."
        )

    if (
        controller.replicability_result
        is not None
    ):
        raise AssertionError(
            "prior_guided unexpectedly executed "
            "ReplicabilityChecker."
        )

    # ---------------------------------------------------------
    # Verify legacy runtime semantic naming.
    # ---------------------------------------------------------

    runtime_object_ids = {
        obj.object_id
        for obj
        in controller.scene_state.objects
    }

    if (
        runtime_object_ids
        != EXPECTED_RUNTIME_OBJECT_IDS
    ):
        raise AssertionError(
            "Unexpected prior-guided runtime "
            "semantic object IDs. "
            f"Expected={sorted(EXPECTED_RUNTIME_OBJECT_IDS)}, "
            f"received={sorted(runtime_object_ids)}"
        )

    # ---------------------------------------------------------
    # Verify legacy CAP output.
    # ---------------------------------------------------------

    primitive_plan = (
        controller.primitive_plan
    )

    if (
        len(
            primitive_plan.steps
        )
        != len(
            EXPECTED_PRIMITIVES
        )
    ):
        raise AssertionError(
            "Unexpected number of prior-guided CAP "
            "primitives: "
            f"expected={len(EXPECTED_PRIMITIVES)}, "
            f"received={len(primitive_plan.steps)}"
        )

    for index, (
        primitive_step,
        (
            expected_name,
            expected_target,
        ),
    ) in enumerate(
        zip(
            primitive_plan.steps,
            EXPECTED_PRIMITIVES,
            strict=True,
        ),
        start=1,
    ):
        if (
            primitive_step.name
            != expected_name
        ):
            raise AssertionError(
                f"Primitive {index}: expected "
                f"{expected_name!r}, received "
                f"{primitive_step.name!r}."
            )

        expected_arguments = {
            "target": expected_target,
        }

        if (
            primitive_step.arguments
            != expected_arguments
        ):
            raise AssertionError(
                f"Primitive {index}: expected "
                f"{expected_arguments}, received "
                f"{primitive_step.arguments}."
            )

        if (
            expected_target
            not in runtime_object_ids
        ):
            raise AssertionError(
                "CAP generated a primitive target "
                "that does not exist in the "
                "prior-guided SceneState: "
                f"{expected_target!r}"
            )


def _validate_pipeline_artifacts(
    artifacts_dir: Path,
) -> None:
    required_files = (
        artifacts_dir
        / "action_planning"
        / "action_plan.json",
        artifacts_dir
        / "scene_perceiver"
        / "raw_scene_state.json",
        artifacts_dir
        / "scene_interpreter"
        / "scene_state.json",
        artifacts_dir
        / "lmp_generator"
        / "generated_program.py",
        artifacts_dir
        / "lmp_generator"
        / "primitive_plan.json",
    )

    for path in required_files:
        _require_file(
            path
        )


def run_seedo_controller_prior_guided_test(
    args: argparse.Namespace,
) -> int:
    """Run the complete legacy prior-guided SeeDo pipeline."""

    _validate_canonical_inputs(
        args
    )

    if not args.model_config:
        raise ValueError(
            "--model-config is required for the "
            "prior-guided SeeDoController test."
        )

    with TemporaryDirectory(
        prefix="seedo_prior_guided_config_"
    ) as temporary_config_dir:

        prior_guided_config = (
            _create_prior_guided_config(
                source_config=(
                    args.model_config
                ),
                output_dir=Path(
                    temporary_config_dir
                ),
            )
        )

        controller = SeeDoController(
            model_config=str(
                prior_guided_config
            ),
        )

        if (
            controller.perception_mode
            != "prior_guided"
        ):
            raise AssertionError(
                "Temporary test configuration did "
                "not activate prior_guided mode."
            )

        print(
            "=== RUNNING PRIOR-GUIDED "
            "DEMONSTRATION PIPELINE ==="
        )

        controller.load_command(
            demo_path=args.video,
            task_id="test",
            artifacts_dir=(
                args.artifacts_dir
            ),
        )

        if (
            controller.artifacts_dir
            is None
        ):
            raise AssertionError(
                "SeeDoController did not initialize "
                "artifact storage."
            )

        artifacts_dir = (
            controller.artifacts_dir
        )

        _validate_demo_pipeline(
            controller
        )

        print(
            "Prior-guided demonstration plan: "
            f"{controller.action_plan.natural_language_plan}"
        )

        print()
        print(
            "=== RUNNING PRIOR-GUIDED "
            "RUNTIME PIPELINE ==="
        )

        runtime_input = (
            load_scene_runtime_input(
                args
            )
        )

        result = (
            controller.inference(
                input_data=runtime_input,
                t=0,
            )
        )

        if result is not None:
            raise AssertionError(
                "inference(t=0) must return None."
            )

        _validate_runtime_pipeline(
            controller
        )

        _validate_pipeline_artifacts(
            artifacts_dir
        )

        print(
            "Prior-guided runtime objects:"
        )

        for obj in (
            controller.scene_state.objects
        ):
            print(
                f"  {obj.object_id}"
            )

        print()
        print(
            "Prior-guided CAP primitives:"
        )

        for step in (
            controller.primitive_plan.steps
        ):
            print(
                f"  {step.name}"
                f"({step.arguments})"
            )

        # -----------------------------------------------------
        # Motion Layer.
        # -----------------------------------------------------

        print()
        print(
            "=== RUNNING PRIOR-GUIDED "
            "MOTION LAYER ==="
        )

        robot_state = {
            EEF_POS_NAME: (
                INITIAL_POSITION.copy()
            ),
            EEF_QUAT_NAME: (
                INITIAL_ORIENTATION.copy()
            ),
        }

        motion_runtime_input = dict(
            runtime_input
        )

        motion_runtime_input[
            "robot_state"
        ] = robot_state

        returned_actions: list[
            np.ndarray
        ] = []

        for t in range(
            1,
            len(
                controller
                .primitive_plan
                .steps
            )
            + 1,
        ):
            primitive_step = (
                controller
                .primitive_plan
                .steps[
                    t - 1
                ]
            )

            actions = (
                controller.inference(
                    input_data=(
                        motion_runtime_input
                    ),
                    t=t,
                )
            )

            if actions is None:
                raise AssertionError(
                    "SeeDoController returned None "
                    "before all prior-guided "
                    "primitive steps were translated. "
                    f"t={t}"
                )

            if not isinstance(
                actions,
                list,
            ):
                raise AssertionError(
                    "Motion Layer must return a list "
                    f"at t={t}, received "
                    f"{type(actions).__name__}."
                )

            if not actions:
                raise AssertionError(
                    "Motion Layer returned no actions "
                    f"for primitive "
                    f"{primitive_step.name!r}."
                )

            for action in actions:
                if not isinstance(
                    action,
                    np.ndarray,
                ):
                    raise AssertionError(
                        "Motion Layer returned a "
                        "non-array action."
                    )

                if action.shape != (
                    8,
                ):
                    raise AssertionError(
                        "Unexpected low-level action "
                        f"shape: {action.shape}."
                    )

                if not np.all(
                    np.isfinite(
                        action
                    )
                ):
                    raise AssertionError(
                        "Motion Layer returned "
                        "non-finite values."
                    )

            returned_actions.extend(
                actions
            )

            print(
                f"  t={t}: "
                f"{primitive_step.name}"
                f"({primitive_step.arguments}) "
                f"-> {len(actions)} "
                "low-level action(s)"
            )

        _validate_motion_artifact(
            artifacts_dir=artifacts_dir,
            controller=controller,
            returned_actions=(
                returned_actions
            ),
        )

        completion_t = (
            len(
                controller
                .primitive_plan
                .steps
            )
            + 1
        )

        completion_result = (
            controller.inference(
                input_data=(
                    motion_runtime_input
                ),
                t=completion_t,
            )
        )

        if (
            completion_result
            is not None
        ):
            raise AssertionError(
                "Controller must return None "
                "after all primitives have "
                "been consumed."
            )

        if (
            controller.execution_status
            != "completed"
        ):
            raise AssertionError(
                "Prior-guided controller did not "
                "enter completed state: "
                f"{controller.execution_status}"
            )

        print()
        print(
            "PRIOR-GUIDED END-TO-END "
            "SEEDO CONTROLLER TEST PASSED"
        )

    return 0