from __future__ import annotations

from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any
import json
import numpy as np
import os

from ai_controller.utils.ai_controller import AIController

from ai_controller.models.seedo_controller.keyframe_selector import (
    KeyframeSelector,
)
from ai_controller.models.seedo_controller.visual_prompter import (
    VisualPrompter,
)
from ai_controller.models.seedo_controller.demo_structured_scene_builder import (
    DemoStructuredSceneBuilder,
)
from ai_controller.models.seedo_controller.action_planner import (
    ActionPlanner,
)
from ai_controller.models.seedo_controller.scene_perceiver import (
    ScenePerceiver,
)
from ai_controller.models.seedo_controller.scene_interpreter import (
    SceneInterpreter,
)
from ai_controller.models.seedo_controller.runtime_structured_scene_builder import (
    RuntimeStructuredSceneBuilder,
)
from ai_controller.models.seedo_controller.structural_matcher import (
    StructuralMatcher,
)
from ai_controller.models.seedo_controller.replicability_checker import (
    ReplicabilityChecker,
)
from ai_controller.models.seedo_controller.lmp_generator import (
    LMPGenerator,
)
from ai_controller.models.seedo_controller.motion_layer import (
    SeeDoMotionLayer,
)
from ai_controller.models.seedo_controller.nut_visual_debug import (
    export_nut_visual_debug_artifacts,
)
from ai_controller.utils.grasping.grasp_planner_base import (
    GraspPlanResult,
)
from ai_controller.utils.grasping.grasp_planner_factory import (
    build_grasp_planner,
)
from results import (
    ActionPlanningResult,
    ActionStep,
    SceneState,
    PrimitivePlan,
    StructuredScene,
    StructuredSceneObject,
    StructuredSceneRelation,
    StructuralMatchingResult,
    ReplicabilityResult,
)
from ai_controller.utils.utils import (
    EEF_POS_NAME,
    EEF_QUAT_NAME,
)
from ai_controller.models.seedo_controller.timing_utils import TIMING


class SeeDoController(AIController):
    """Orchestrate the complete SeeDo inference pipeline."""

    def __init__(
        self,
        model_config: str,
    ) -> None:
        self.device: Any = None

        # Demonstration / task configuration.
        self.demo_path: Path | None = None
        self.task_id: str | None = None

        # Artifact management.
        self.artifacts_dir: Path | None = None
        self._temporary_artifacts_dir: TemporaryDirectory[str] | None = None

        # Result of the SeeDo demonstration-understanding pipeline.
        self.action_plan: ActionPlanningResult | None = None

        # Structured scene representation of the demonstration.
        self.demo_structured_scene: StructuredScene | None = None

        # Runtime state.
        self.perception_result = None
        self.scene_state: SceneState | None = None
        self.primitive_plan: PrimitivePlan | None = None

        self.grasp_plan: GraspPlanResult | None = None
        self.grasp_plan_target: str | None = None

        self.nut_place_refinements: dict[
            str,
            dict[str, Any],
        ] = {}

        self.nut_pick_refinements: dict[
            str,
            dict[str, Any],
        ] = {}

        # Nut-Assembly visual diagnostics.
        #
        # The initial RGB-D scene is cached once at t=0.
        # Motion actions are then accumulated until one INSERTING
        # primitive completes.
        self._nut_visual_debug_scene_context: (
            dict[str, Any]
            | None
        ) = None

        self._nut_visual_debug_action_groups: dict[
            str,
            list[np.ndarray],
        ] = {
            "reach": [],
            "approaching": [],
            "pick": [],
            "lift_up": [],
            "moving": [],
            "aligning": [],
            "inserting": [],
        }

        self._nut_visual_debug_export_index = 0

        # Structured scene representation of the runtime scene.
        self.runtime_structured_scene: StructuredScene | None = None

        # Result of the structural matching between the demo and runtime scenes.
        self.structural_matching_result: StructuralMatchingResult | None = None

        # Result of the replicability check between the demo and runtime scenes.
        self.replicability_result: ReplicabilityResult | None = None

        # Execution state.
        self.execution_status: str = "idle"
        self.execution_error: str | None = None

        super().__init__(model_config=model_config)

    def _load_precomputed_action_plan(
        self,
        action_plan_path: str | Path,
    ) -> ActionPlanningResult:
        """Load a previously generated SeeDo ActionPlanningResult."""

        normalized_path = (
            Path(action_plan_path)
            .expanduser()
            .resolve()
        )

        if not normalized_path.is_file():
            raise FileNotFoundError(
                "Precomputed SeeDo action plan does not exist: "
                f"{normalized_path}"
            )

        if normalized_path.stat().st_size == 0:
            raise ValueError(
                "Precomputed SeeDo action plan is empty: "
                f"{normalized_path}"
            )

        with normalized_path.open(
            "r",
            encoding="utf-8",
        ) as stream:
            plan = json.load(stream)

        raw_steps = plan.get("steps", [])
        raw_ambiguities = plan.get("ambiguities", [])

        action_steps_list: list[ActionStep] = []

        for step in raw_steps:
            raw_destination_ordinal = step.get(
                "destination_ordinal_from_left"
            )

            if self.perception_mode == "generalized":
                destination_ordinal_from_left = None

            else:
                if raw_destination_ordinal is None:
                    raise ValueError(
                        "Precomputed prior-guided action plan is "
                        "missing destination_ordinal_from_left."
                    )

                destination_ordinal_from_left = int(
                    raw_destination_ordinal
                )

            action_steps_list.append(
                ActionStep(
                    pick_keyframe=int(
                        step["pick_keyframe"]
                    ),
                    place_keyframe=int(
                        step["place_keyframe"]
                    ),
                    picked_track_id=int(
                        step["picked_track_id"]
                    ),
                    picked_category=str(
                        step["picked_category"]
                    ),
                    picked_color=str(
                        step["picked_color"]
                    ),
                    destination_track_id=int(
                        step["destination_track_id"]
                    ),
                    destination_category=str(
                        step["destination_category"]
                    ),
                    destination_ordinal_from_left=(
                        destination_ordinal_from_left
                    ),
                    relation=str(
                        step["relation"]
                    ),
                    action=str(
                        step["action"]
                    ),
                    picked_detector_label=str(
                        step.get(
                            "picked_detector_label",
                            "",
                        )
                    ),
                    grasp_instruction=str(
                        step.get(
                            "grasp_instruction",
                            "",
                        )
                    ).strip(),
                )
            )

        action_steps = tuple(
            action_steps_list
        )

        ambiguities = tuple(
            str(item)
            for item in raw_ambiguities
        )

        if action_steps:
            natural_language_plan = " and then ".join(
                step.action
                for step in action_steps
            )
        else:
            natural_language_plan = (
                "No action plan generated: "
                + "; ".join(ambiguities)
            )

        result = ActionPlanningResult(
            steps=action_steps,
            status=str(plan["status"]),
            ambiguities=ambiguities,
            natural_language_plan=natural_language_plan,
            task_type=str(
                plan["task_type"]
            ),
        )

        return self.post_process(result)

    def _load_precomputed_demo_structured_scene(
        self,
        structured_scene_path: str | Path,
    ) -> StructuredScene:
        """Load a previously generated demonstration StructuredScene."""

        normalized_path = (
            Path(structured_scene_path)
            .expanduser()
            .resolve()
        )

        if not normalized_path.is_file():
            raise FileNotFoundError(
                "Precomputed demonstration structured scene "
                "does not exist: "
                f"{normalized_path}"
            )

        if normalized_path.stat().st_size == 0:
            raise ValueError(
                "Precomputed demonstration structured scene "
                "is empty: "
                f"{normalized_path}"
            )

        with normalized_path.open(
            "r",
            encoding="utf-8",
        ) as stream:
            data = json.load(stream)

        directions = int(
            data["directions"]
        )

        if directions not in (4, 8):
            raise ValueError(
                "Invalid direction mode in precomputed "
                "demonstration structured scene: "
                f"{directions}."
            )

        if directions != self.structured_scene_directions:
            raise ValueError(
                "Precomputed demonstration structured scene "
                "uses a different direction mode: "
                f"precomputed={directions}, "
                f"configured={self.structured_scene_directions}."
            )

        objects = tuple(
            StructuredSceneObject(
                object_id=str(obj["object_id"]),
                category=str(obj["category"]),
                center=(
                    float(obj["center"][0]),
                    float(obj["center"][1]),
                ),
            )
            for obj in data.get("objects", [])
        )

        relations = tuple(
            StructuredSceneRelation(
                subject_object_id=str(
                    relation["subject_object_id"]
                ),
                reference_object_id=str(
                    relation["reference_object_id"]
                ),
                relation=str(
                    relation["relation"]
                ),
            )
            for relation in data.get(
                "relations",
                [],
            )
        )

        if not objects:
            raise ValueError(
                "Precomputed demonstration structured scene "
                "contains no objects."
            )

        return StructuredScene(
            objects=objects,
            relations=relations,
            directions=directions,
        )

    def load_model(
        self,
        model_config: str,
    ) -> dict[str, Any]:
        import yaml

        config_path = Path(model_config).expanduser().resolve()

        if not config_path.is_file():
            raise FileNotFoundError(
                f"SeeDo model configuration does not exist: {config_path}"
            )

        with config_path.open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}

        # ---------------------------------------------------------
        # Perception mode
        # ---------------------------------------------------------

        self.perception_mode = str(
            config.get(
                "perception_mode",
                "generalized",
            )
        ).strip().lower()

        allowed_perception_modes = {
            "prior_guided",
            "generalized",
        }

        if self.perception_mode not in allowed_perception_modes:
            raise ValueError(
                "Invalid perception_mode: "
                f"{self.perception_mode!r}. "
                "Expected 'prior_guided' or 'generalized'."
            )

        keyframe_config = config.get(
            "keyframe_selector",
            {},
        )
        visual_config = config.get(
            "visual_prompter",
            {},
        )
        grounding_dino_config = config.get(
            "grounding_dino",
            {},
        )

        grounding_dino_crop_config = (
            grounding_dino_config.get(
                "crop",
                {},
            )
        )

        self.grounding_dino_crop = {
            "top_px": grounding_dino_crop_config.get(
                "top_px",
                80,
            ),
            "bottom_px": grounding_dino_crop_config.get(
                "bottom_px",
                0,
            ),
            "left_px": grounding_dino_crop_config.get(
                "left_px",
                0,
            ),
            "right_px": grounding_dino_crop_config.get(
                "right_px",
                0,
            ),
        }
        action_config = config.get(
            "action_planner",
            {},
        )
        structured_scene_config = config.get(
            "structured_scene",
            {},
        )

        structured_scene_directions = int(
            structured_scene_config.get(
                "directions",
                8,
            )
        )

        if structured_scene_directions not in (4, 8):
            raise ValueError(
                "Invalid structured_scene.directions: "
                f"{structured_scene_directions}. "
                "Expected 4 or 8."
            )

        self.structured_scene_directions = (
            structured_scene_directions
        )
        perception_config = config.get(
            "scene_perceiver",
            {},
        )

        lmp_config = config.get(
            "lmp_generator",
            {},
        )

        scene_interpreter_config = config.get(
            "scene_interpreter",
            {},
        )

        grasp_config = config.get(
            "grasp_planner",
            {},
        )

        self.grasp_planning_enabled = bool(
            grasp_config.get(
                "enabled",
                True,
            )
        )

        motion_config = config.get(
            "motion_layer",
            {},
        )

        self.scene_interpreter = SceneInterpreter(
            model=scene_interpreter_config.get(
                "model",
                "gpt-4o-2024-08-06",
            ),
            perception_mode=self.perception_mode,
        )

        self.runtime_structured_scene_builder = (
            RuntimeStructuredSceneBuilder(
                directions=self.structured_scene_directions,
            )
        )

        self.structural_matcher = StructuralMatcher()

        self.replicability_checker = ReplicabilityChecker()

        camera_pose_noise_config = perception_config.get(
            "camera_pose_noise",
            {},
        )

        camera_pose_noise_level = str(
            camera_pose_noise_config.get(
                "level",
                "baseline",
            )
        ).strip().lower()

        camera_pose_noise_levels = camera_pose_noise_config.get(
            "levels",
            {},
        )

        if camera_pose_noise_level not in camera_pose_noise_levels:
            raise ValueError(
                "Unknown camera pose noise level: "
                f"'{camera_pose_noise_level}'. "
                "Expected one of: "
                f"{list(camera_pose_noise_levels.keys())}"
            )

        selected_camera_pose_noise = camera_pose_noise_levels[
            camera_pose_noise_level
        ]

        self.scene_perceiver = ScenePerceiver(
            camera_calibration_path=perception_config.get(
                "camera_calibration_path"
            ),
            camera_name=perception_config.get(
                "camera_name",
                "zed_front",
            ),
            grounding_config=visual_config.get(
                "grounding_config"
            ),
            grounding_checkpoint=visual_config.get(
                "grounding_checkpoint"
            ),
            bert_model=visual_config.get(
                "bert_model"
            ),
            sam_checkpoint=visual_config.get(
                "sam_checkpoint"
            ),
            detector_labels=perception_config.get(
                "detector_labels"
            ),
            camera_pose_noise_level=camera_pose_noise_level,
            translation_noise_std_mm=float(
                selected_camera_pose_noise.get(
                    "translation_std_mm",
                    0.0,
                )
            ),
            rotation_noise_std_deg=float(
                selected_camera_pose_noise.get(
                    "rotation_std_deg",
                    0.0,
                )
            ),
            perception_mode=self.perception_mode,
            grounding_dino_crop=self.grounding_dino_crop,
        )

        self.lmp_generator = LMPGenerator(
            model=lmp_config.get(
                "model",
                "gpt-4o-2024-08-06",
            ),
            perception_mode=self.perception_mode,
        )

        self.workspace_bottom_left = tuple(
            float(value)
            for value in perception_config.get(
                "workspace_bottom_left",
                [0.0, 0.0],
            )
        )

        self.workspace_top_right = tuple(
            float(value)
            for value in perception_config.get(
                "workspace_top_right",
                [1.0, 1.0],
            )
        )

        self.keyframe_selector = KeyframeSelector(
            mode=str(
                keyframe_config.get(
                    "mode",
                    "hand_velocity",
                )
            ),
            sample_stride=int(
                keyframe_config.get(
                    "sample_stride",
                    5,
                )
            ),
            model=str(
                keyframe_config.get(
                    "model",
                    "gpt-4o-2024-08-06",
                )
            ),
            gaussian_sigma=float(
                keyframe_config.get(
                    "gaussian_sigma",
                    5.0,
                )
            ),
            prominence=float(
                keyframe_config.get(
                    "prominence",
                    0.8,
                )
            ),
            expected_keyframes=int(
                keyframe_config.get(
                    "expected_keyframes",
                    2,
                )
            ),
            save_preview=bool(
                keyframe_config.get(
                    "save_preview",
                    False,
                )
            ),
        )

        self.visual_prompter = VisualPrompter(
            grounding_config=visual_config.get(
                "grounding_config"
            ),
            grounding_checkpoint=visual_config.get(
                "grounding_checkpoint"
            ),
            bert_model=visual_config.get(
                "bert_model"
            ),
            sam_checkpoint=visual_config.get(
                "sam_checkpoint"
            ),
            sam2_checkpoint=visual_config.get(
                "sam2_checkpoint"
            ),
            objects=visual_config.get(
                "objects"
            ),
            perception_mode=self.perception_mode,
            grounding_dino_crop=self.grounding_dino_crop,
        )

        self.demo_structured_scene_builder = (
            DemoStructuredSceneBuilder(
                directions=self.structured_scene_directions,
            )
        )

        self.action_planner = ActionPlanner(
            model=action_config.get(
                "model",
                "gpt-4o-2024-08-06",
            ),
            demonstration_bin_order=action_config.get(
                "demonstration_bin_order",
                "left_to_right",
            ),
            perception_mode=self.perception_mode,
        )

        if self.grasp_planning_enabled:
            self.grasp_planner = build_grasp_planner(
                grasp_config
            )
        else:
            self.grasp_planner = None

        self.motion_layer = SeeDoMotionLayer(
            grasp_orientation=motion_config.get(
                "grasp_orientation",
                [
                    0.9994452044624775,
                    0.03161651380119412,
                    0.0021438049655468088,
                    0.010251021036213035,
                ],
            ),
            min_step=float(
                motion_config.get(
                    "min_step",
                    0.02,
                )
            ),
            reach_hover_height=float(
                motion_config.get(
                    "reach_hover_height",
                    0.15,
                )
            ),
            approach_z_offset=float(
                motion_config.get(
                    "approach_z_offset",
                    0.0,
                )
            ),
            release_height_offset=float(
                motion_config.get(
                    "release_height_offset",
                    0.10,
                )
            ),
            lift_height=float(
                motion_config.get(
                    "lift_height",
                    0.15,
                )
            ),
            gripper_open_position=float(
                motion_config.get(
                    "gripper_open_position",
                    0.1,
                )
            ),
            gripper_closed_position=float(
                motion_config.get(
                    "gripper_closed_position",
                    0.8,
                )
            ),
            object_y_offset=float(
                motion_config.get(
                    "object_y_offset",
                    -0.06,
                )
            ),
            grasp_planning_enabled=(
                self.grasp_planning_enabled
            ),
            pregrasp_distance=float(
                motion_config.get(
                    "pregrasp_distance",
                    0.08,
                )
            ),
            grasp_depth_offset=float(
                motion_config.get(
                    "grasp_depth_offset",
                    0.005,
                )
            ),
            retreat_distance=float(
                motion_config.get(
                    "retreat_distance",
                    0.04,
                )
            ),
            nut_assembly_config=motion_config.get(
                "nut_assembly",
                {},
            ),
        )

        self.pipeline_config = config

        return {
            "keyframe_selector": self.keyframe_selector,
            "visual_prompter": self.visual_prompter,
            "demo_structured_scene_builder":  self.demo_structured_scene_builder,
            "action_planner": self.action_planner,
            "scene_perceiver": self.scene_perceiver,
            "scene_interpreter": self.scene_interpreter,
            "runtime_structured_scene_builder": self.runtime_structured_scene_builder,
            "structural_matcher": self.structural_matcher,
            "replicability_checker": self.replicability_checker,
            "lmp_generator": self.lmp_generator,
            "motion_layer": self.motion_layer,
            "grasp_planner": self.grasp_planner,
        }

    def move_model_to_device(
        self,
        device: Any,
    ) -> None:
        self.device = device

    def _cleanup_temporary_artifacts(self) -> None:
        """Remove artifacts created for a temporary runtime session."""

        if self._temporary_artifacts_dir is not None:
            self._temporary_artifacts_dir.cleanup()
            self._temporary_artifacts_dir = None

        self.artifacts_dir = None


    def _prepare_artifacts_dir(
        self,
        artifacts_dir: str | Path | None,
    ) -> Path:
        """Prepare persistent or temporary storage for pipeline artifacts."""

        self._cleanup_temporary_artifacts()

        if artifacts_dir is not None:
            normalized_artifacts_dir = (
                Path(artifacts_dir)
                .expanduser()
                .resolve()
            )

            normalized_artifacts_dir.mkdir(
                parents=True,
                exist_ok=True,
            )

            self.artifacts_dir = normalized_artifacts_dir
            return normalized_artifacts_dir

        self._temporary_artifacts_dir = TemporaryDirectory(
            prefix="seedo_"
        )

        self.artifacts_dir = Path(
            self._temporary_artifacts_dir.name
        ).resolve()

        return self.artifacts_dir

    def reset(self) -> None:
        self.perception_result = None
        self.action_plan = None
        self.scene_state = None
        self.primitive_plan = None
        self.grasp_plan = None
        self.grasp_plan_target = None
        self.nut_place_refinements = {}
        self._reset_nut_visual_debug_state()
        self.nut_pick_refinements = {}
        self.demo_structured_scene = None
        self.runtime_structured_scene = None
        self.structural_matching_result = None
        self.replicability_result = None

        self.execution_status = "idle"
        self.execution_error = None

        self.demo_path = None
        self.task_id = None

        self._cleanup_temporary_artifacts()

    def pre_process(
        self,
        input_data: Any,
    ) -> dict[str, Any]:
        if not isinstance(input_data, dict):
            raise TypeError(
                "SeeDoController expects a dictionary as runtime input."
            )

        processed_input: dict[str, Any] = {}

        if "rgb" in input_data:
            processed_input["rgb"] = input_data["rgb"]

        if "depth" in input_data:
            processed_input["depth"] = input_data["depth"]

        if "camera_info" in input_data:
            processed_input["camera_info"] = input_data["camera_info"]

        if "camera_name" in input_data:
            processed_input["camera_name"] = str(
                input_data["camera_name"]
            )

        if "base_to_table_transform" in input_data:
            processed_input["base_to_table_transform"] = (
                input_data["base_to_table_transform"]
            )

        if "robot_state" in input_data:
            processed_input["robot_state"] = (
                input_data["robot_state"]
            )

        if "grasp_input" in input_data:
            processed_input["grasp_input"] = (
                input_data["grasp_input"]
            )

        return processed_input

    def _resolve_grasp_action_step(
        self,
        primitive_step,
    ) -> tuple[int, ActionStep, str]:
        """
        Resolve the ActionStep associated with the pick target of
        the current approaching primitive.
        """

        if self.action_plan is None:
            raise RuntimeError(
                "Action plan is not available."
            )

        target = str(
            primitive_step.arguments.get(
                "target",
                "",
            )
        ).strip()

        if not target:
            raise RuntimeError(
                "The approaching primitive does not contain "
                "a target argument."
            )

        normalized_target = (
            target.casefold()
        )

        # In generalized mode the ReplicabilityChecker already
        # provides the authoritative mapping:
        #
        # ActionStep -> runtime pick object.
        if self.perception_mode == "generalized":

            if self.replicability_result is None:
                raise RuntimeError(
                    "ReplicabilityResult is not available "
                    "while resolving the grasp target."
                )

            matches = [
                resolved_target
                for resolved_target
                in self.replicability_result.resolved_targets
                if (
                    str(
                        resolved_target.runtime_pick_object_id
                    )
                    .strip()
                    .casefold()
                    == normalized_target
                )
            ]

            if len(matches) != 1:
                raise RuntimeError(
                    "Could not uniquely associate the approaching "
                    f"target {target!r} with an ActionStep. "
                    f"Matches: {len(matches)}."
                )

            action_step_index = int(
                matches[
                    0
                ].action_step_index
            )

            if not (
                0
                <= action_step_index
                < len(self.action_plan.steps)
            ):
                raise RuntimeError(
                    "Resolved ActionStep index is outside "
                    "the action plan: "
                    f"{action_step_index}."
                )

            return (
                action_step_index,
                self.action_plan.steps[
                    action_step_index
                ],
                target,
            )

        # Current prior-guided executions normally contain a
        # single pick/place action.
        if len(self.action_plan.steps) == 1:
            return (
                0,
                self.action_plan.steps[0],
                target,
            )

        raise RuntimeError(
            "Cannot uniquely associate the approaching primitive "
            f"target {target!r} with one ActionStep."
        )

    def _resolve_nut_pick_targets(
        self,
    ) -> tuple[str, ...]:
        """
        Return the runtime objects actually used by Nut-Assembly
        APPROACHING primitives.

        The refinement therefore happens only after CAP has already
        decided which object is being picked.
        """

        if self.action_plan is None:
            raise RuntimeError(
                "Action plan is not available."
            )

        if (
            str(self.action_plan.task_type)
            .strip()
            .lower()
            != "nut_assembly"
        ):
            return ()

        if self.primitive_plan is None:
            raise RuntimeError(
                "Primitive plan is not available."
            )

        targets: list[str] = []
        normalized_targets: set[str] = set()

        for primitive_step in self.primitive_plan.steps:

            if (
                str(primitive_step.name)
                .strip()
                .lower()
                != "approaching"
            ):
                continue

            target = primitive_step.arguments.get(
                "target"
            )

            if not isinstance(target, str):
                raise RuntimeError(
                    "Nut-Assembly APPROACHING primitive does "
                    "not contain a valid target."
                )

            target = target.strip()

            if not target:
                raise RuntimeError(
                    "Nut-Assembly APPROACHING target is empty."
                )

            normalized = target.casefold()

            if normalized not in normalized_targets:

                targets.append(
                    target
                )

                normalized_targets.add(
                    normalized
                )

        if not targets:
            raise RuntimeError(
                "Nut-Assembly primitive plan contains no "
                "pick target."
            )

        return tuple(targets)

    def _resolve_nut_place_targets(
        self,
    ) -> tuple[str, ...]:
        """
        Resolve the actual runtime destination objects used by the
        Nut-Assembly place primitives.

        CAP is authoritative here: refinement is performed only after
        moving/aligning/inserting have been generated, therefore
        ScenePerceiver never has to guess which visible object is a peg.

        The expected place sequence for every assembly is:

            moving(target)
            aligning(target)
            inserting(target)

        and all three primitives must refer to the same target.
        """

        if self.action_plan is None:
            raise RuntimeError(
                "Action plan is not available while resolving "
                "Nut-Assembly place targets."
            )

        if (
            str(self.action_plan.task_type)
            .strip()
            .lower()
            != "nut_assembly"
        ):
            return ()

        if self.primitive_plan is None:
            raise RuntimeError(
                "Primitive plan is not available while resolving "
                "Nut-Assembly place targets."
            )

        place_primitive_names = {
            "moving",
            "aligning",
            "inserting",
        }

        expected_phase = "moving"
        active_target: str | None = None

        resolved_targets: list[str] = []
        normalized_targets: set[str] = set()

        for primitive_step in self.primitive_plan.steps:

            primitive_name = (
                str(primitive_step.name)
                .strip()
                .lower()
            )

            if primitive_name not in place_primitive_names:
                continue

            if primitive_name != expected_phase:
                raise RuntimeError(
                    "Invalid Nut-Assembly place primitive sequence: "
                    f"expected {expected_phase!r}, "
                    f"received {primitive_name!r}."
                )

            target = primitive_step.arguments.get(
                "target"
            )

            if not isinstance(target, str):
                raise RuntimeError(
                    "Nut-Assembly place primitive "
                    f"{primitive_name!r} does not contain a "
                    "string 'target' argument."
                )

            target = target.strip()

            if not target:
                raise RuntimeError(
                    "Nut-Assembly place primitive "
                    f"{primitive_name!r} contains an empty target."
                )

            normalized_target = (
                target.casefold()
            )

            if primitive_name == "moving":

                active_target = target

                if (
                    normalized_target
                    not in normalized_targets
                ):
                    resolved_targets.append(
                        target
                    )

                    normalized_targets.add(
                        normalized_target
                    )

                expected_phase = "aligning"

            elif primitive_name == "aligning":

                if active_target is None:
                    raise RuntimeError(
                        "Nut-Assembly ALIGNING does not have "
                        "an active MOVING destination."
                    )

                if (
                    normalized_target
                    != active_target.casefold()
                ):
                    raise RuntimeError(
                        "Nut-Assembly destination changed between "
                        "MOVING and ALIGNING: "
                        f"{active_target!r} -> {target!r}."
                    )

                expected_phase = "inserting"

            elif primitive_name == "inserting":

                if active_target is None:
                    raise RuntimeError(
                        "Nut-Assembly INSERTING does not have "
                        "an active destination."
                    )

                if (
                    normalized_target
                    != active_target.casefold()
                ):
                    raise RuntimeError(
                        "Nut-Assembly destination changed between "
                        "ALIGNING and INSERTING: "
                        f"{active_target!r} -> {target!r}."
                    )

                active_target = None
                expected_phase = "moving"

        if active_target is not None:
            raise RuntimeError(
                "Incomplete Nut-Assembly place sequence: "
                "MOVING/ALIGNING was not followed by INSERTING."
            )

        if not resolved_targets:
            raise RuntimeError(
                "Nut-Assembly primitive plan contains no "
                "MOVING -> ALIGNING -> INSERTING destination."
            )

        return tuple(
            resolved_targets
        )

    def _resolve_raw_object_for_runtime_target(
        self,
        target: str,
    ):
        """
        Recover the RawSceneObject, including its SAM mask, associated
        with one already-resolved runtime SceneState target.

        Generalized mode normally preserves the raw object_id directly.
        Pixel coordinates provide a deterministic identity-preserving
        fallback for modes where SceneInterpreter changes the semantic
        object name.
        """

        if self.scene_state is None:
            raise RuntimeError(
                "SceneState is not available while resolving "
                "the Nut-Assembly destination mask."
            )

        if self.perception_result is None:
            raise RuntimeError(
                "Scene perception result is not available while "
                "resolving the Nut-Assembly destination mask."
            )

        normalized_target = (
            str(target)
            .strip()
            .casefold()
        )

        scene_matches = [
            obj
            for obj in self.scene_state.objects
            if (
                obj.object_id
                .strip()
                .casefold()
                == normalized_target
            )
        ]

        if len(scene_matches) != 1:
            raise RuntimeError(
                "Could not uniquely resolve Nut-Assembly "
                f"destination {target!r} in SceneState. "
                f"Matches: {len(scene_matches)}."
            )

        scene_object = scene_matches[0]

        raw_objects = (
            self.perception_result
            .raw_scene
            .objects
        )

        # ---------------------------------------------------------
        # Fast path used by generalized perception:
        #
        # SceneInterpreter intentionally preserves raw object IDs.
        # ---------------------------------------------------------

        raw_id_matches = [
            raw_object
            for raw_object in raw_objects
            if (
                raw_object.object_id
                .strip()
                .casefold()
                == normalized_target
            )
        ]

        if len(raw_id_matches) == 1:
            return raw_id_matches[0]

        if len(raw_id_matches) > 1:
            raise RuntimeError(
                "Multiple raw perception objects share "
                f"destination ID {target!r}."
            )

        # ---------------------------------------------------------
        # Identity-preserving fallback.
        #
        # SceneInterpreter copies pixel_coordinates directly from
        # RawSceneObject into SceneObject, so they can be used to
        # recover the exact original SAM mask.
        # ---------------------------------------------------------

        scene_pixel = tuple(
            int(value)
            for value in scene_object.pixel_coordinates
        )

        pixel_matches = [
            raw_object
            for raw_object in raw_objects
            if tuple(
                int(value)
                for value
                in raw_object.pixel_coordinates
            )
            == scene_pixel
        ]

        if len(pixel_matches) != 1:
            raise RuntimeError(
                "Could not uniquely recover the raw SAM mask "
                f"for Nut-Assembly destination {target!r}. "
                f"Pixel={scene_pixel}, "
                f"matches={len(pixel_matches)}."
            )

        return pixel_matches[0]

    def _resolve_nut_pick_refinement(
        self,
        target: str,
    ) -> dict[str, Any]:
        """
        Return the frozen hole refinement associated with the
        runtime object actually being grasped.
        """

        normalized_target = (
            str(target)
            .strip()
            .casefold()
        )

        matches = [
            refinement
            for object_id, refinement
            in self.nut_pick_refinements.items()
            if (
                str(object_id)
                .strip()
                .casefold()
                == normalized_target
            )
        ]

        if len(matches) != 1:
            raise RuntimeError(
                "Could not uniquely resolve Nut-Assembly "
                f"pick refinement for {target!r}. "
                f"Matches: {len(matches)}."
            )

        return matches[0]

    def _reset_nut_visual_debug_state(
        self,
    ) -> None:
        """
        Reset all runtime state used only for Nut-Assembly
        visual diagnostics.
        """

        self._nut_visual_debug_scene_context = None

        self._nut_visual_debug_action_groups = {
            "reach": [],
            "approaching": [],
            "pick": [],
            "lift_up": [],
            "moving": [],
            "aligning": [],
            "inserting": [],
        }

        self._nut_visual_debug_export_index = 0

    def _clear_nut_visual_debug_actions(
        self,
    ) -> None:
        """
        Clear only the primitive actions accumulated for the current
        Nut-Assembly cycle.

        The initial scene remains cached so that a PrimitivePlan
        containing multiple assembly cycles can still be visualized.
        """

        self._nut_visual_debug_action_groups = {
            "reach": [],
            "approaching": [],
            "pick": [],
            "lift_up": [],
            "moving": [],
            "aligning": [],
            "inserting": [],
        }

    def _cache_nut_visual_debug_scene_context(
        self,
        *,
        rgb_image: np.ndarray,
        depth_image: np.ndarray,
        camera_info: dict[str, Any],
    ) -> None:
        """
        Freeze the exact initial RGB-D and geometric calibration used
        by ScenePerceiver.run().

        No calibration is reconstructed or re-sampled here.
        """

        if self.action_plan is None:
            return

        if (
            str(
                self.action_plan.task_type
            )
            .strip()
            .lower()
            != "nut_assembly"
        ):
            return

        runtime_camera_calibration = (
            self.scene_perceiver
            ._last_runtime_camera_calibration
        )

        table_to_base_rotation = (
            self.scene_perceiver
            ._last_base_to_table_rotation
        )

        table_to_base_translation = (
            self.scene_perceiver
            ._last_base_to_table_translation
        )

        if runtime_camera_calibration is None:
            raise RuntimeError(
                "Nut visual-debug cache requires the "
                "runtime camera calibration used by "
                "ScenePerceiver.run()."
            )

        if table_to_base_rotation is None:
            raise RuntimeError(
                "Nut visual-debug cache requires the "
                "runtime table-to-base rotation."
            )

        if table_to_base_translation is None:
            raise RuntimeError(
                "Nut visual-debug cache requires the "
                "runtime table-to-base translation."
            )

        camera_matrix = (
            self.scene_perceiver
            .camera_matrix_from_info(
                camera_info
            )
        )

        T_base_table = np.eye(
            4,
            dtype=np.float64,
        )

        T_base_table[
            :3,
            :3,
        ] = np.asarray(
            table_to_base_rotation,
            dtype=np.float64,
        )

        T_base_table[
            :3,
            3,
        ] = np.asarray(
            table_to_base_translation,
            dtype=np.float64,
        )

        self._nut_visual_debug_scene_context = {
            "front_rgb": np.asarray(
                rgb_image,
                dtype=np.uint8,
            ).copy(),

            "front_depth": np.asarray(
                depth_image,
                dtype=np.float64,
            ).copy(),

            "front_K": np.asarray(
                camera_matrix,
                dtype=np.float64,
            ).copy(),

            "front_camera_calibration": {
                "position": np.asarray(
                    runtime_camera_calibration[
                        "position"
                    ],
                    dtype=np.float64,
                ).copy(),

                "orientation_matrix": np.asarray(
                    runtime_camera_calibration[
                        "orientation_matrix"
                    ],
                    dtype=np.float64,
                ).copy(),
            },

            "T_base_table": (
                T_base_table
            ),
        }

    def _record_nut_visual_debug_actions(
        self,
        primitive_name: str,
        actions,
    ) -> None:
        """
        Save a copy of the actions already produced by MotionLayer.

        This does not alter or regenerate the trajectory.
        """

        if self.action_plan is None:
            return

        if (
            str(
                self.action_plan.task_type
            )
            .strip()
            .lower()
            != "nut_assembly"
        ):
            return

        normalized_name = (
            str(
                primitive_name
            )
            .strip()
            .lower()
        )

        if (
            normalized_name
            not in self._nut_visual_debug_action_groups
        ):
            return

        if actions is None:
            return

        for action in actions:

            self._nut_visual_debug_action_groups[
                normalized_name
            ].append(
                np.asarray(
                    action,
                    dtype=np.float64,
                ).copy()
            )

    def _resolve_nut_place_refinement(
        self,
        target: str,
    ) -> dict[str, Any]:
        """
        Return the frozen peg refinement associated with the
        runtime destination currently being assembled.
        """

        normalized_target = (
            str(
                target
            )
            .strip()
            .casefold()
        )

        matches = [
            refinement
            for object_id, refinement
            in self.nut_place_refinements.items()
            if (
                str(
                    object_id
                )
                .strip()
                .casefold()
                == normalized_target
            )
        ]

        if len(matches) != 1:
            raise RuntimeError(
                "Could not uniquely resolve Nut-Assembly "
                f"place refinement for {target!r}. "
                f"Matches: {len(matches)}."
            )

        return matches[
            0
        ]

    def _export_current_nut_visual_debug(
        self,
        *,
        place_target: str,
    ) -> None:
        """
        Export diagnostics for the Nut-Assembly cycle that has just
        completed INSERTING.

        No model inference is performed by this method.
        """

        if self.action_plan is None:
            return

        if (
            str(
                self.action_plan.task_type
            )
            .strip()
            .lower()
            != "nut_assembly"
        ):
            return

        if self.artifacts_dir is None:
            return

        if (
            self._nut_visual_debug_scene_context
            is None
        ):
            raise RuntimeError(
                "Nut visual-debug initial scene context "
                "is not available."
            )

        if self.grasp_plan is None:
            raise RuntimeError(
                "Nut visual-debug export requires a "
                "completed grasp plan."
            )

        if self.grasp_plan_target is None:
            raise RuntimeError(
                "Nut visual-debug export requires the "
                "active grasp target."
            )

        pick_target = str(
            self.grasp_plan_target
        )

        place_target = str(
            place_target
        ).strip()

        if not place_target:
            raise RuntimeError(
                "Nut visual-debug export received an "
                "empty place target."
            )

        pick_refinement = (
            self._resolve_nut_pick_refinement(
                pick_target
            )
        )

        place_refinement = (
            self._resolve_nut_place_refinement(
                place_target
            )
        )

        raw_pick_object = (
            self._resolve_raw_object_for_runtime_target(
                pick_target
            )
        )

        if raw_pick_object.mask is None:
            raise RuntimeError(
                "Nut visual-debug export requires the "
                "raw SAM mask of the picked object."
            )

        assembly_object_mask = np.asarray(
            raw_pick_object.mask,
            dtype=bool,
        )

        hole_top = np.asarray(
            pick_refinement[
                "hole_center_base"
            ],
            dtype=np.float64,
        )

        hole_bottom = np.asarray(
            pick_refinement[
                "hole_bottom_center_base"
            ],
            dtype=np.float64,
        )

        hole_axis = np.asarray(
            pick_refinement[
                "hole_axis_base"
            ],
            dtype=np.float64,
        )

        hole_axis /= np.linalg.norm(
            hole_axis
        )

        peg_support = np.asarray(
            place_refinement[
                "peg_support_point_base"
            ],
            dtype=np.float64,
        )

        peg_tip = np.asarray(
            place_refinement[
                "peg_tip_point_base"
            ],
            dtype=np.float64,
        )

        peg_axis = np.asarray(
            place_refinement[
                "axis_direction_base"
            ],
            dtype=np.float64,
        )

        peg_axis /= np.linalg.norm(
            peg_axis
        )

        motion = self.motion_layer

        if motion._nut_hole_top_local is None:
            raise RuntimeError(
                "Nut visual-debug export requires "
                "grasp-relative hole geometry."
            )

        if motion._nut_hole_bottom_local is None:
            raise RuntimeError(
                "Nut visual-debug export requires "
                "grasp-relative hole geometry."
            )

        if motion._nut_hole_axis_local is None:
            raise RuntimeError(
                "Nut visual-debug export requires "
                "grasp-relative hole geometry."
            )

        hole_top_local = np.asarray(
            motion._nut_hole_top_local,
            dtype=np.float64,
        )

        hole_bottom_local = np.asarray(
            motion._nut_hole_bottom_local,
            dtype=np.float64,
        )

        hole_axis_local = np.asarray(
            motion._nut_hole_axis_local,
            dtype=np.float64,
        )

        hole_axis_local /= np.linalg.norm(
            hole_axis_local
        )

        nut_config = (
            motion.nut_assembly_config
        )

        peg_clearance_m = float(
            nut_config[
                "peg_clearance_m"
            ]
        )

        initial_entry_m = float(
            nut_config[
                "initial_entry_m"
            ]
        )

        release_bottom_height_m = float(
            nut_config[
                "release_bottom_height_m"
            ]
        )

        auto_release = bool(
            nut_config.get(
                "auto_release",
                False,
            )
        )

        align_target_hole_bottom = (
            peg_tip
            + peg_clearance_m
            * peg_axis
        )

        initial_entry_hole_bottom = (
            peg_tip
            - initial_entry_m
            * peg_axis
        )

        final_release_hole_bottom = (
            peg_support
            + release_bottom_height_m
            * peg_axis
        )

        (
            final_hole_top,
            final_hole_bottom,
            final_hole_axis,
        ) = (
            motion
            ._nut_current_hole_geometry()
        )

        final_hole_top = np.asarray(
            final_hole_top,
            dtype=np.float64,
        )

        final_hole_bottom = np.asarray(
            final_hole_bottom,
            dtype=np.float64,
        )

        final_hole_axis = np.asarray(
            final_hole_axis,
            dtype=np.float64,
        )

        final_hole_axis /= np.linalg.norm(
            final_hole_axis
        )

        final_tcp_position = np.asarray(
            motion.current_pose.position,
            dtype=np.float64,
        )

        final_tcp_orientation = np.asarray(
            motion.current_pose.orientation,
            dtype=np.float64,
        )

        lift_actions = (
            self
            ._nut_visual_debug_action_groups
            .get(
                "lift_up",
                [],
            )
        )

        if lift_actions:

            lift_tcp = np.asarray(
                lift_actions[
                    -1
                ][
                    :3
                ],
                dtype=np.float64,
            )

        else:

            lift_tcp = (
                final_tcp_position.copy()
            )

        scene_context = (
            self._nut_visual_debug_scene_context
        )

        action_index = (
            self._nut_visual_debug_export_index
        )

        output_dir = (
            self.artifacts_dir
            / "nut_visual_debug"
            / (
                f"action_"
                f"{action_index:02d}"
            )
        )

        paths = (
            export_nut_visual_debug_artifacts(
                output_dir=(
                    output_dir
                ),

                front_rgb=(
                    scene_context[
                        "front_rgb"
                    ]
                ),

                front_depth=(
                    scene_context[
                        "front_depth"
                    ]
                ),

                front_K=(
                    scene_context[
                        "front_K"
                    ]
                ),

                front_camera_calibration=(
                    scene_context[
                        "front_camera_calibration"
                    ]
                ),

                T_base_table=(
                    scene_context[
                        "T_base_table"
                    ]
                ),

                assembly_object_mask=(
                    assembly_object_mask
                ),

                pick_target=(
                    pick_target
                ),

                place_target=(
                    place_target
                ),

                real_grasp_tcp=np.asarray(
                    self.grasp_plan
                    .tcp_position_base,
                    dtype=np.float64,
                ),

                real_grasp_quaternion=np.asarray(
                    self.grasp_plan
                    .tcp_orientation_base,
                    dtype=np.float64,
                ),

                final_tcp_position=(
                    final_tcp_position
                ),

                final_tcp_orientation=(
                    final_tcp_orientation
                ),

                hole_top_initial=(
                    hole_top
                ),

                hole_bottom_initial=(
                    hole_bottom
                ),

                hole_axis_initial=(
                    hole_axis
                ),

                peg_support=(
                    peg_support
                ),

                peg_tip=(
                    peg_tip
                ),

                peg_axis=(
                    peg_axis
                ),

                align_target_hole_bottom=(
                    align_target_hole_bottom
                ),

                initial_entry_hole_bottom=(
                    initial_entry_hole_bottom
                ),

                final_release_hole_bottom=(
                    final_release_hole_bottom
                ),

                final_hole_top=(
                    final_hole_top
                ),

                final_hole_bottom=(
                    final_hole_bottom
                ),

                final_hole_axis=(
                    final_hole_axis
                ),

                lift_tcp=(
                    lift_tcp
                ),

                object_sweep_radius_m=float(
                    motion
                    ._nut_object_sweep_radius_m
                ),

                object_thickness_m=float(
                    pick_refinement[
                        "nut_thickness_estimate_m"
                    ]
                ),

                peg_height_m=float(
                    place_refinement[
                        "peg_height_estimate_m"
                    ]
                ),

                action_groups=(
                    self
                    ._nut_visual_debug_action_groups
                ),

                hole_top_local=(
                    hole_top_local
                ),

                hole_bottom_local=(
                    hole_bottom_local
                ),

                hole_axis_local=(
                    hole_axis_local
                ),

                auto_release=(
                    auto_release
                ),
            )
        )

        print(
            "[NUT DEBUG] Visual artifacts exported:",
            flush=True,
        )

        for name, path in paths.items():
            print(
                f"  - {name}: {path}",
                flush=True,
            )

        self._nut_visual_debug_export_index += 1

        self._clear_nut_visual_debug_actions()

    def _refine_nut_pick_targets(
        self,
        *,
        depth_image,
        camera_info,
    ) -> None:
        """
        Refine the internal hole geometry of every object actually
        picked by a Nut-Assembly plan.
        """

        if self.action_plan is None:
            raise RuntimeError(
                "Action plan is not available during "
                "Nut-Assembly pick refinement."
            )

        if (
            str(self.action_plan.task_type)
            .strip()
            .lower()
            != "nut_assembly"
        ):
            self.nut_pick_refinements = {}
            return

        targets = (
            self._resolve_nut_pick_targets()
        )

        camera_matrix = (
            self.scene_perceiver
            .camera_matrix_from_info(
                camera_info
            )
        )

        refinements: dict[
            str,
            dict[str, Any],
        ] = {}

        for target in targets:

            raw_object = (
                self._resolve_raw_object_for_runtime_target(
                    target
                )
            )

            if raw_object.mask is None:
                raise RuntimeError(
                    "Nut-Assembly pick target "
                    f"{target!r} does not have a SAM mask."
                )

            print(
                "[NUT] Refining picked-object hole:",
                target,
                flush=True,
            )

            refinement = (
                self.scene_perceiver
                .refine_nut_hole(
                    mask=raw_object.mask,
                    depth_image=depth_image,
                    camera_matrix=camera_matrix,
                )
            )

            if not refinement["accepted"]:
                raise RuntimeError(
                    "Nut-hole refinement rejected "
                    f"{target!r}."
                )

            refinements[target] = (
                refinement
            )

            print(
                "[NUT] Pick geometry accepted | "
                f"target={target} | "
                f"center="
                f"{refinement['hole_center_base']} | "
                f"axis="
                f"{refinement['hole_axis_base']} | "
                f"thickness="
                f"{refinement['nut_thickness_estimate_m'] * 1000.0:.2f} mm",
                flush=True,
            )

        self.nut_pick_refinements = (
            refinements
        )

        if self.artifacts_dir is not None:

            output_path = (
                self.artifacts_dir
                / "scene_perceiver"
                / "nut_pick_refinements.json"
            )

            with output_path.open(
                "w",
                encoding="utf-8",
            ) as stream:

                json.dump(
                    refinements,
                    stream,
                    indent=2,
                    ensure_ascii=False,
                )

    def _refine_nut_place_targets(
        self,
        *,
        depth_image,
        camera_info,
    ) -> None:
        """
        Refine the peg axis for every destination actually used by
        Nut-Assembly place primitives.

        Refinement failure is fatal for Nut-Assembly: the controller
        must never silently fall back to the biased SceneState centroid.
        """

        if self.action_plan is None:
            raise RuntimeError(
                "Action plan is not available during "
                "Nut-Assembly place refinement."
            )

        if (
            str(self.action_plan.task_type)
            .strip()
            .lower()
            != "nut_assembly"
        ):
            self.nut_place_refinements = {}
            return

        targets = (
            self._resolve_nut_place_targets()
        )

        camera_matrix = (
            self.scene_perceiver
            .camera_matrix_from_info(
                camera_info
            )
        )

        refinements: dict[
            str,
            dict[str, Any],
        ] = {}

        for target in targets:

            raw_object = (
                self._resolve_raw_object_for_runtime_target(
                    target
                )
            )

            if raw_object.mask is None:
                raise RuntimeError(
                    "Nut-Assembly destination "
                    f"{target!r} does not have a SAM mask."
                )

            print(
                "[NUT] Refining assembly destination:",
                target,
                flush=True,
            )

            refinement = (
                self.scene_perceiver
                .refine_peg_axis(
                    mask=raw_object.mask,
                    depth_image=depth_image,
                    camera_matrix=camera_matrix,
                )
            )

            if not refinement["accepted"]:
                raise RuntimeError(
                    "Nut-Assembly peg-axis refinement rejected "
                    f"destination {target!r}: "
                    f"reasons="
                    f"{refinement['rejection_reasons']}, "
                    f"rms="
                    f"{refinement['rms_m'] * 1000.0:.2f} mm, "
                    f"inlier_fraction="
                    f"{refinement['inlier_fraction']:.3f}, "
                    f"arc="
                    f"{refinement['arc_coverage_deg']:.1f} deg."
                )

            refinements[target] = refinement

            print(
                "[NUT] Destination accepted | "
                f"target={target} | "
                f"axis_xy_base="
                f"{refinement['axis_xy_base']} | "
                f"rms="
                f"{refinement['rms_m'] * 1000.0:.2f} mm | "
                f"inliers="
                f"{refinement['inlier_fraction']:.3f} | "
                f"arc="
                f"{refinement['arc_coverage_deg']:.1f} deg",
                flush=True,
            )

        self.nut_place_refinements = (
            refinements
        )

        # ---------------------------------------------------------
        # Diagnostics artifact.
        # ---------------------------------------------------------

        if self.artifacts_dir is not None:

            refinement_dir = (
                self.artifacts_dir
                / "scene_perceiver"
            )

            refinement_dir.mkdir(
                parents=True,
                exist_ok=True,
            )

            output_path = (
                refinement_dir
                / "nut_place_refinements.json"
            )

            with output_path.open(
                "w",
                encoding="utf-8",
            ) as stream:

                json.dump(
                    refinements,
                    stream,
                    indent=2,
                    ensure_ascii=False,
                )

    def inference(
        self,
        input_data: Any,
        t: int = 0,
        save_path: str | None = None,
    ) -> Any:
        print(f"[INFO] SeeDoController.inference(t={t}) called.")
        if self.action_plan is None:
            raise RuntimeError(
                "load_command() must be called before inference()."
            )

        if self.artifacts_dir is None:
            raise RuntimeError(
                "Artifact storage has not been initialized."
            )

        if t < 0:
            raise ValueError(
                f"Invalid timestep: {t}. "
                "The timestep cannot be negative."
            )

        processed_input = self.pre_process(
            input_data
        )

        if t == 0:
            self.execution_status = "perceiving"

            required_inputs = (
                "rgb",
                "depth",
                "camera_info",
                "base_to_table_transform",
            )

            missing_inputs = [
                key
                for key in required_inputs
                if key not in processed_input
            ]

            if missing_inputs:
                raise ValueError(
                    "Missing runtime perception inputs: "
                    + ", ".join(missing_inputs)
                )

            try:
                self.execution_status = "perceiving"

                with TIMING.measure(
                    "scene_perception"
                ):
                    self.perception_result = self.scene_perceiver.run(
                        rgb_image=processed_input["rgb"],
                        depth_image=processed_input["depth"],
                        camera_info=processed_input["camera_info"],
                        base_to_table_transform=processed_input[
                            "base_to_table_transform"
                        ],
                        artifacts_dir=(
                            self.artifacts_dir
                            / "scene_perceiver"
                        ),
                    )

                if (
                    str(
                        self.action_plan.task_type
                    )
                    .strip()
                    .lower()
                    == "nut_assembly"
                ):
                    self._reset_nut_visual_debug_state()

                    self._cache_nut_visual_debug_scene_context(
                        rgb_image=processed_input[
                            "rgb"
                        ],

                        depth_image=processed_input[
                            "depth"
                        ],

                        camera_info=processed_input[
                            "camera_info"
                        ],
                    )

                self.execution_status = "interpreting_scene"
                
                with TIMING.measure(
                    "scene_interpretation"
                ):
                    self.scene_state = self.scene_interpreter.run(
                        perception_result=self.perception_result,
                        artifacts_dir=(
                            self.artifacts_dir
                            / "scene_interpreter"
                        ),
                    )

                if self.perception_mode == "generalized":
                    with TIMING.measure(
                        "runtime_structured_scene"
                    ):
                        self.runtime_structured_scene = (
                            self.runtime_structured_scene_builder.run(
                                scene_state=self.scene_state,
                                artifacts_dir=(
                                    self.artifacts_dir
                                    / "runtime_structured_scene"
                                ),
                            )
                        )

                    if self.demo_structured_scene is None:
                        raise RuntimeError(
                            "Demo structured scene is not available. "
                            "The demonstration must be processed before "
                            "structural matching."
                        )

                    with TIMING.measure(
                        "structural_matching"
                    ):
                        self.structural_matching_result = (
                            self.structural_matcher.run(
                                demo_scene=self.demo_structured_scene,
                                runtime_scene=self.runtime_structured_scene,
                                artifacts_dir=(
                                    self.artifacts_dir
                                    / "structural_matching"
                                ),
                            )
                        )

                    self.execution_status = "checking_replicability"

                    with TIMING.measure(
                        "replicability_check"
                    ):
                        self.replicability_result = (
                            self.replicability_checker.run(
                                action_plan=self.action_plan,
                                structural_matching_result=(
                                    self.structural_matching_result
                                ),
                                scene_state=self.scene_state,
                                artifacts_dir=(
                                    self.artifacts_dir
                                    / "replicability_check"
                                ),
                            )
                        )

                    if not self.replicability_result.replicable:
                        raise RuntimeError(
                            "The demonstrated task is not replicable in the "
                            "runtime scene: "
                            + "; ".join(
                                self.replicability_result.failure_reasons
                            )
                        )

                self.execution_status = "generating_lmp"

                with TIMING.measure(
                    "lmp_generation"
                ):
                    self.primitive_plan = self.lmp_generator.run(
                        action_plan=self.action_plan,
                        scene_state=self.scene_state,
                        workspace_bottom_left=self.workspace_bottom_left,
                        workspace_top_right=self.workspace_top_right,
                        replicability_result=self.replicability_result,
                        artifacts_dir=(
                            self.artifacts_dir
                            / "lmp_generator"
                        ),
                    )

                if (
                    str(self.action_plan.task_type)
                    .strip()
                    .lower()
                    == "nut_assembly"
                ):
                    with TIMING.measure(
                        "nut_place_refinement"
                    ):
                        self._refine_nut_place_targets(
                            depth_image=processed_input[
                                "depth"
                            ],
                            camera_info=processed_input[
                                "camera_info"
                            ],
                        )

                    with TIMING.measure(
                        "nut_pick_refinement"
                    ):
                        self._refine_nut_pick_targets(
                            depth_image=processed_input[
                                "depth"
                            ],
                            camera_info=processed_input[
                                "camera_info"
                            ],
                        )

            except Exception as exc:
                self.execution_status = "failed"
                self.execution_error = str(exc)
                raise

            self.execution_status = "plan_ready"

            return None

        if self.primitive_plan is None:
            raise RuntimeError(
                "No primitive plan has been generated yet. "
                "The perception/CAP step at t=0 must complete first."
            )

        if self.scene_state is None:
            raise RuntimeError(
                "No SceneState is available for motion translation."
            )

        primitive_index = t - 1

        if primitive_index >= len(self.primitive_plan.steps):
            self.execution_status = "completed"
            return None

        #
        # Initialize the Motion Layer exactly once, before translating
        # the first primitive.
        #
        if primitive_index == 0:
            robot_state = processed_input.get(
                "robot_state"
            )

            if robot_state is None:
                raise ValueError(
                    "SeeDo inference(t=1) requires robot_state "
                    "to initialize the Motion Layer."
                )

            current_position = robot_state.get(
                EEF_POS_NAME
            )

            current_orientation = robot_state.get(
                EEF_QUAT_NAME
            )

            if current_position is None:
                raise ValueError(
                    "SeeDo robot_state does not contain "
                    f"{EEF_POS_NAME!r}."
                )

            if current_orientation is None:
                raise ValueError(
                    "SeeDo robot_state does not contain "
                    f"{EEF_QUAT_NAME!r}."
                )

            if self.action_plan is None:
                raise RuntimeError(
                    "Action plan unavailable before Motion Layer reset"
                )

            self.motion_layer.reset(
                current_position=current_position,
                current_orientation=current_orientation,
                artifacts_dir=(
                    self.artifacts_dir
                    / "motion_layer"
                ),
                task_type=self.action_plan.task_type,
            )

            if (
                str(self.action_plan.task_type)
                .strip()
                .lower()
                == "nut_assembly"
            ):
                if not self.nut_place_refinements:
                    raise RuntimeError(
                        "Nut-Assembly Motion Layer initialization "
                        "requires refined place geometry."
                    )

                self.motion_layer.set_nut_place_refinements(
                    self.nut_place_refinements
                )

        primitive_step = self.primitive_plan.steps[
            primitive_index
        ]

        try:

            # ---------------------------------------------------------
            # Task-oriented grasp planning
            #
            # Run exactly after REACH has been physically completed
            # and immediately before APPROACHING is translated.
            # ---------------------------------------------------------

            if (
                self.grasp_planning_enabled and
                primitive_step.name
                .strip()
                .lower()
                == "approaching"
            ):

                if self.grasp_planner is None:
                    raise RuntimeError(
                        "Grasp planning is enabled but "
                        "GraspPlanner is not initialized."
                    )
                
                grasp_input = processed_input.get(
                    "grasp_input"
                )

                if grasp_input is None:
                    raise ValueError(
                        "SeeDo approaching requires a fresh "
                        "eye-in-hand grasp_input."
                    )

                robot_state = processed_input.get(
                    "robot_state"
                )

                if robot_state is None:
                    raise ValueError(
                        "SeeDo grasp planning requires robot_state."
                    )

                current_position = robot_state.get(
                    EEF_POS_NAME
                )

                current_orientation = robot_state.get(
                    EEF_QUAT_NAME
                )

                if current_position is None:
                    raise ValueError(
                        "SeeDo grasp planning robot_state does not "
                        f"contain {EEF_POS_NAME!r}."
                    )

                if current_orientation is None:
                    raise ValueError(
                        "SeeDo grasp planning robot_state does not "
                        f"contain {EEF_QUAT_NAME!r}."
                    )

                (
                    action_step_index,
                    action_step,
                    grasp_target,
                ) = self._resolve_grasp_action_step(
                    primitive_step
                )

                grasp_instruction = str(
                    action_step.grasp_instruction
                ).strip()

                if not grasp_instruction:
                    raise RuntimeError(
                        "The ActionStep associated with the "
                        "approaching primitive contains an empty "
                        "grasp_instruction."
                    )

                required_grasp_inputs = (
                    "rgb_image",
                    "depth_image",
                    "camera_matrix",
                    "T_base_camera",
                    "T_base_table",
                )

                missing_grasp_inputs = [
                    key
                    for key in required_grasp_inputs
                    if key not in grasp_input
                ]

                if missing_grasp_inputs:
                    raise ValueError(
                        "Missing GraspPlanner runtime inputs: "
                        + ", ".join(
                            missing_grasp_inputs
                        )
                    )

                self.execution_status = (
                    "planning_grasp"
                )

                print()
                print(
                    "[GRASP] Planning grasp for:",
                    grasp_target,
                )

                print(
                    "[GRASP] Instruction:",
                    grasp_instruction,
                )

                with TIMING.measure(
                    "grasp_planning"
                ):
                    self.grasp_plan = (
                        self.grasp_planner.plan(
                            rgb_image=(
                                grasp_input[
                                    "rgb_image"
                                ]
                            ),
                            depth_image=(
                                grasp_input[
                                    "depth_image"
                                ]
                            ),
                            camera_matrix=(
                                grasp_input[
                                    "camera_matrix"
                                ]
                            ),
                            T_base_camera=(
                                grasp_input[
                                    "T_base_camera"
                                ]
                            ),
                            T_base_table=(
                                grasp_input[
                                    "T_base_table"
                                ]
                            ),
                            current_tcp_position=(
                                current_position
                            ),
                            current_tcp_orientation=(
                                current_orientation
                            ),
                            grasp_instruction=(
                                grasp_instruction
                            ),
                            artifacts_dir=(
                                self.artifacts_dir
                                / "grasp_planner"
                                / (
                                    f"action_"
                                    f"{action_step_index:02d}"
                                )
                            ),
                        )
                    )

                # ---------------------------------------------------------
                # TEMPORARY ISAAC-ONLY POST-PLANNING GRASP Z OFFSET
                #
                # The grasp planner and all of its validation checks operate
                # on the original pose. The optional offset is applied only
                # afterwards, immediately before sending the grasp pose to
                # the Motion Layer.
                # ---------------------------------------------------------

                executed_grasp_position = np.asarray(
                    self.grasp_plan.tcp_position_base,
                    dtype=np.float64,
                ).copy()

                isaac_grasp_z_offset_m = float(
                    os.environ.get(
                        "SEEDO_ISAAC_GRASP_Z_OFFSET_M",
                        "0.0",
                    )
                )

                if abs(isaac_grasp_z_offset_m) > 1e-12:

                    executed_grasp_position[2] += (
                        isaac_grasp_z_offset_m
                    )

                    print(
                        "[ISAAC TEMP] Post-planning grasp Z offset:",
                        f"{isaac_grasp_z_offset_m * 1000.0:+.1f} mm",
                        flush=True,
                    )

                    print(
                        "[ISAAC TEMP] Planner TCP position:",
                        self.grasp_plan.tcp_position_base,
                        flush=True,
                    )

                    print(
                        "[ISAAC TEMP] Executed TCP position:",
                        executed_grasp_position,
                        flush=True,
                    )


                self.motion_layer.set_grasp_pose(
                    position=(
                        executed_grasp_position
                    ),
                    orientation=(
                        self.grasp_plan.tcp_orientation_base
                    ),
                    approach_direction=(
                        self.grasp_plan.approach_direction_base
                    ),
                    backend_name=(
                        self.grasp_planner.backend_name
                    ),
                )

                if (
                    str(self.action_plan.task_type)
                    .strip()
                    .lower()
                    == "nut_assembly"
                ):
                    nut_pick_refinement = (
                        self._resolve_nut_pick_refinement(
                            grasp_target
                        )
                    )

                    self.motion_layer.set_nut_grasp_geometry(
                        target_id=grasp_target,

                        hole_center_base=(
                            nut_pick_refinement[
                                "hole_center_base"
                            ]
                        ),

                        hole_bottom_center_base=(
                            nut_pick_refinement[
                                "hole_bottom_center_base"
                            ]
                        ),

                        hole_axis_base=(
                            nut_pick_refinement[
                                "hole_axis_base"
                            ]
                        ),

                        object_radius_from_hole_m=(
                            nut_pick_refinement[
                                "object_radius_from_hole_m"
                            ]
                        ),
                    )

                self.grasp_plan_target = (
                    grasp_target
                )

                print(
                    "[GRASP] Backend:",
                    self.grasp_planner.backend_name,
                )

                print(
                    "[GRASP] Confidence:",
                    self.grasp_plan.confidence,
                )

                print(
                    "[GRASP] Semantic point:",
                    self.grasp_plan.semantic_point_px,
                )

                print(
                    "[GRASP] TCP position base:",
                    self.grasp_plan.tcp_position_base,
                )

                print(
                    "[GRASP] TCP orientation base:",
                    self.grasp_plan.tcp_orientation_base,
                )

                print(
                    "[GRASP] Approach direction base:",
                    self.grasp_plan.approach_direction_base,
                )

                print()

            # ---------------------------------------------------------
            # Primitive -> robot actions
            # ---------------------------------------------------------

            self.execution_status = "executing"

            with TIMING.measure(
                "motion_layer"
            ):
                actions = self.motion_layer.translate(
                    primitive_step=primitive_step,
                    scene_state=self.scene_state,
                )

            if (
                str(
                    self.action_plan.task_type
                )
                .strip()
                .lower()
                == "nut_assembly"
            ):
                primitive_name = (
                    str(
                        primitive_step.name
                    )
                    .strip()
                    .lower()
                )

                self._record_nut_visual_debug_actions(
                    primitive_name=(
                        primitive_name
                    ),
                    actions=(
                        actions
                    ),
                )

                if primitive_name == "inserting":

                    place_target = (
                        primitive_step.arguments.get(
                            "target"
                        )
                    )

                    if not isinstance(
                        place_target,
                        str,
                    ):
                        raise RuntimeError(
                            "Nut-Assembly INSERTING primitive "
                            "does not contain a valid target."
                        )

                    place_target = (
                        place_target.strip()
                    )

                    if not place_target:
                        raise RuntimeError(
                            "Nut-Assembly INSERTING target "
                            "is empty."
                        )

                    #
                    # Visual diagnostics are deliberately
                    # non-critical.
                    #
                    # A plotting/filesystem problem must not turn
                    # a successfully generated robot trajectory
                    # into a failed task execution.
                    #
                    try:
                        self._export_current_nut_visual_debug(
                            place_target=(
                                place_target
                            )
                        )

                    except Exception as debug_exc:
                        print(
                            "[NUT DEBUG] WARNING: "
                            "could not export visual artifacts: "
                            f"{debug_exc}",
                            flush=True,
                        )

        except Exception as exc:
            self.execution_status = "failed"
            self.execution_error = str(exc)
            raise

        return actions

    def post_process(
        self,
        output_data: ActionPlanningResult,
    ) -> ActionPlanningResult:
        if not isinstance(output_data, ActionPlanningResult):
            raise TypeError(
                "SeeDoController expected an ActionPlanningResult, "
                f"received {type(output_data).__name__}."
            )

        if output_data.status not in (
            "completed",
            "ambiguous",
        ):
            raise ValueError(
                f"Unexpected action-planning status: {output_data.status}"
            )

        if (
            output_data.status == "completed"
            and not output_data.steps
        ):
            raise ValueError(
                "A completed action plan must contain at least one step."
            )

        if (
            output_data.status == "ambiguous"
            and not output_data.ambiguities
        ):
            raise ValueError(
                "An ambiguous action plan must explain its ambiguities."
            )

        if not output_data.natural_language_plan.strip():
            raise ValueError(
                "The natural-language action plan is empty."
            )

        return output_data

    def load_command(
        self,
        demo_path: str,
        task_id: str,
        **kwargs: Any,
    ) -> None:
        normalized_demo_path = (
            Path(demo_path)
            .expanduser()
            .resolve()
        )

        if not normalized_demo_path.is_file():
            raise FileNotFoundError(
                "SeeDo demonstration video does not exist: "
                f"{normalized_demo_path}"
            )

        if normalized_demo_path.stat().st_size == 0:
            raise ValueError(
                "SeeDo demonstration video is empty: "
                f"{normalized_demo_path}"
            )

        self.demo_path = normalized_demo_path
        self.task_id = str(task_id)

        requested_artifacts_dir = kwargs.get(
            "artifacts_dir"
        )

        artifacts_dir = self._prepare_artifacts_dir(
            requested_artifacts_dir
        )

        precomputed_action_plan_path = kwargs.get(
            "precomputed_action_plan_path"
        )

        precomputed_demo_structured_scene_path = kwargs.get(
            "precomputed_demo_structured_scene_path"
        )

        if precomputed_action_plan_path:
            self.execution_status = "planning"
            self.execution_error = None

            # Reset the state associated with any previously loaded
            # demonstration/runtime execution.
            self.action_plan = None
            self.scene_state = None
            self.primitive_plan = None
            self.grasp_plan = None
            self.grasp_plan_target = None
            self.nut_place_refinements = {}
            self._reset_nut_visual_debug_state()
            self.nut_pick_refinements = {}
            self.demo_structured_scene = None
            self.runtime_structured_scene = None
            self.structural_matching_result = None
            self.replicability_result = None

            try:
                self.action_plan = (
                    self._load_precomputed_action_plan(
                        precomputed_action_plan_path
                    )
                )

                if self.perception_mode == "generalized":

                    if not precomputed_demo_structured_scene_path:
                        raise ValueError(
                            "Generalized precomputed execution requires "
                            "precomputed_demo_structured_scene_path."
                        )

                    self.demo_structured_scene = (
                        self._load_precomputed_demo_structured_scene(
                            precomputed_demo_structured_scene_path
                        )
                    )

            except Exception as exc:
                self.execution_status = "failed"
                self.execution_error = str(exc)
                raise

            self.execution_status = "plan_ready"
            return

        # Reset runtime/execution state for the new demonstration.
        self.action_plan = None
        self.scene_state = None
        self.primitive_plan = None
        self.grasp_plan = None
        self.grasp_plan_target = None
        self.nut_place_refinements = {}
        self._reset_nut_visual_debug_state()
        self.nut_pick_refinements = {}
        self.demo_structured_scene = None
        self.runtime_structured_scene = None
        self.structural_matching_result = None
        self.replicability_result = None

        self.execution_status = "planning"
        self.execution_error = None

        try:
            with TIMING.measure(
                "keyframe_selection"
            ):
                keyframe_result = self.keyframe_selector.run(
                    video_path=self.demo_path,
                    artifacts_dir=(
                        artifacts_dir
                        / "keyframe_selection"
                    ),
                )

            with TIMING.measure(
                "visual_prompting_wall"
            ):
                visual_result = self.visual_prompter.run(
                    video_path=self.demo_path,
                    keyframes=keyframe_result.keyframes,
                    artifacts_dir=(
                        artifacts_dir
                        / "visual_prompting"
                    ),
                )

            if self.perception_mode == "generalized":
                with TIMING.measure(
                    "demo_structured_scene"
                ):
                    self.demo_structured_scene = (
                        self.demo_structured_scene_builder.run(
                            visual_prompting_result=visual_result,
                            artifacts_dir=(
                                artifacts_dir
                                / "demo_structured_scene"
                            ),
                        )
                    )

            with TIMING.measure(
                "action_planning"
            ):
                action_result = self.action_planner.run(
                    annotated_video_path=(
                        visual_result.annotated_video_path
                    ),
                    keyframes=keyframe_result.keyframes,
                    track_id_map=visual_result.track_id_map,
                    key_frame_coordinates=(
                        visual_result.key_frame_coordinates
                    ),
                    artifacts_dir=(
                        artifacts_dir
                        / "action_planning"
                    ),
                )

            self.action_plan = self.post_process(
                action_result
            )

        except Exception as exc:
            self.execution_status = "failed"
            self.execution_error = str(exc)
            raise

        self.execution_status = "plan_ready"