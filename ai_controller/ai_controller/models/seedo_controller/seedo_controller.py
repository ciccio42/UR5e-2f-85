from __future__ import annotations

from pathlib import Path
from typing import Any

from ai_controller.utils.ai_controller import AIController

from ai_controller.models.seedo_controller.action_planner import (
    ActionPlanner,
)
from ai_controller.models.seedo_controller.keyframe_selector import (
    KeyframeSelector,
)
from ai_controller.models.seedo_controller.visual_prompter import (
    VisualPrompter,
)
from ai_controller.models.seedo_controller.scene_perceiver import (
    ScenePerceiver,
)
from ai_controller.models.seedo_controller.lmp_generator import (
    LMPGenerator,
)
from ai_controller.models.seedo_controller.scene_interpreter import (
    SceneInterpreter,
)
from results import (
    ActionPlanningResult,
    SceneState,
    PrimitivePlan,
)


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
        self.artifacts_dir: Path | None = None

        # Result of the SeeDo demonstration-understanding pipeline.
        self.action_plan: ActionPlanningResult | None = None

        # Runtime state. These will be populated later by perception and CAP.
        self.scene_state: SceneState | None = None
        self.primitive_plan: PrimitivePlan | None = None

        # Execution state.
        self.execution_status: str = "idle"
        self.execution_error: str | None = None

        super().__init__(model_config=model_config)

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

        keyframe_config = config.get(
            "keyframe_selector",
            {},
        )
        visual_config = config.get(
            "visual_prompter",
            {},
        )
        action_config = config.get(
            "action_planner",
            {},
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

        self.scene_interpreter = SceneInterpreter(
            model=scene_interpreter_config.get(
                "model",
                "gpt-4o-2024-08-06",
            ),
        )

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
        )

        self.lmp_generator = LMPGenerator(
            model=lmp_config.get(
                "model",
                "gpt-4o-2024-08-06",
            ),
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
        )

        self.action_planner = ActionPlanner(
            model=action_config.get(
                "model",
                "gpt-4o-2024-08-06",
            ),
        )

        self.pipeline_config = config

        return {
            "keyframe_selector": self.keyframe_selector,
            "visual_prompter": self.visual_prompter,
            "action_planner": self.action_planner,
            "scene_perceiver": self.scene_perceiver,
            "scene_interpreter": self.scene_interpreter,
            "lmp_generator": self.lmp_generator,
        }

    def move_model_to_device(
        self,
        device: Any,
    ) -> None:
        self.device = device

    def reset(self) -> None:
        self.action_plan = None
        self.scene_state = None
        self.primitive_plan = None

        self.execution_status = "idle"
        self.execution_error = None

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

        return processed_input

    def inference(
        self,
        input_data: Any,
        t: int = 0,
        save_path: str | None = None,
    ) -> Any:
        if self.action_plan is None:
            raise RuntimeError(
                "load_command() must be called before inference()."
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

                raw_scene = self.scene_perceiver.run(
                    rgb_image=processed_input["rgb"],
                    depth_image=processed_input["depth"],
                    camera_info=processed_input["camera_info"],
                    base_to_table_transform=processed_input[
                        "base_to_table_transform"
                    ],
                )

                self.execution_status = "interpreting_scene"

                self.scene_state = self.scene_interpreter.run(
                    raw_scene=raw_scene,
                    rgb_image=processed_input["rgb"],
                    artifacts_dir=(
                        artifacts_dir
                        / "scene_interpreter"
                    ),
                )

                self.execution_status = "generating_lmp"

                self.primitive_plan = self.lmp_generator.run(
                    action_plan=self.action_plan,
                    scene_state=self.scene_state,
                    workspace_bottom_left=self.workspace_bottom_left,
                    workspace_top_right=self.workspace_top_right,
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

        primitive_index = t - 1

        if primitive_index >= len(self.primitive_plan.steps):
            self.execution_status = "completed"
            return None

        self.execution_status = "executing"

        return self.primitive_plan.steps[
            primitive_index
        ]

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

        artifacts_dir = kwargs.get(
            "artifacts_dir",
            self.artifacts_dir,
        )

        if artifacts_dir is None:
            raise ValueError(
                "artifacts_dir has not been specified."
            )

        self.artifacts_dir = (
            Path(artifacts_dir)
            .expanduser()
            .resolve()
        )

        self.artifacts_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        # Reset only runtime/execution state for the new demonstration.
        self.action_plan = None
        self.scene_state = None
        self.primitive_plan = None
        self.execution_status = "planning"
        self.execution_error = None

        keyframe_result = self.keyframe_selector.run(
            video_path=self.demo_path,
            artifacts_dir=(
                self.artifacts_dir
                / "keyframe_selection"
            ),
        )

        visual_result = self.visual_prompter.run(
            video_path=self.demo_path,
            keyframes=keyframe_result.keyframes,
            artifacts_dir=(
                self.artifacts_dir
                / "visual_prompting"
            ),
        )

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
                self.artifacts_dir
                / "action_planning"
            ),
        )

        self.action_plan = self.post_process(
            action_result
        )

        self.execution_status = "plan_ready"