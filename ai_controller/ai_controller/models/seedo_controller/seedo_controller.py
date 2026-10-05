from __future__ import annotations

from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any
import json

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

        return processed_input

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

            self.motion_layer.reset(
                current_position=current_position,
                current_orientation=current_orientation,
                artifacts_dir=(
                    self.artifacts_dir
                    / "motion_layer"
                ),
            )

        primitive_step = self.primitive_plan.steps[
            primitive_index
        ]

        self.execution_status = "executing"

        try:

            with TIMING.measure(
                "motion_layer"
            ):
                actions = self.motion_layer.translate(
                    primitive_step=primitive_step,
                    scene_state=self.scene_state,
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