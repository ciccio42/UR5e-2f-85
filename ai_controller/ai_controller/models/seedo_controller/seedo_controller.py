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
from results import ActionPlanningResult


class SeeDoController(AIController):
    """Orchestrate the complete SeeDo inference pipeline."""

    def __init__(
        self,
        model_config: str,
    ) -> None:
        self.device: Any = None
        self.last_result: ActionPlanningResult | None = None
        self.demo_path: Path | None = None
        self.task_id: str | None = None
        self.artifacts_dir: Path | None = None

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
        }

    def move_model_to_device(
        self,
        device: Any,
    ) -> None:
        self.device = device

    def reset(self) -> None:
        self.last_result = None

    def pre_process(
        self,
        input_data: Any,
    ) -> dict[str, Any]:
        if not isinstance(input_data, dict):
            raise TypeError(
                "SeeDoController expects a dictionary as input."
            )

        if "video_path" not in input_data:
            raise ValueError(
                "Missing 'video_path' in input_data."
            )

        video_path = (
            Path(input_data["video_path"])
            .expanduser()
            .resolve()
        )

        if not video_path.is_file():
            raise FileNotFoundError(
                f"Video does not exist: {video_path}"
            )

        artifacts_dir = input_data.get(
            "artifacts_dir",
            self.artifacts_dir,
        )

        if artifacts_dir is None:
            raise ValueError(
                "artifacts_dir has not been specified."
            )

        artifacts_dir = (
            Path(artifacts_dir)
            .expanduser()
            .resolve()
        )

        artifacts_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        return {
            "video_path": video_path,
            "artifacts_dir": artifacts_dir,
        }

    def inference(
        self,
        input_data: Any,
        t: int = 0,
        save_path: str | None = None,
    ) -> ActionPlanningResult:
        processed_input = self.pre_process(
            input_data
        )

        video_path = processed_input["video_path"]
        artifacts_dir = processed_input[
            "artifacts_dir"
        ]

        keyframe_result = self.keyframe_selector.run(
            video_path=video_path,
            artifacts_dir=artifacts_dir / "keyframe_selection",
        )

        visual_result = self.visual_prompter.run(
            video_path=video_path,
            keyframes=keyframe_result.keyframes,
            artifacts_dir=artifacts_dir / "visual_prompting",
        )

        action_result = self.action_planner.run(
            annotated_video_path=visual_result.annotated_video_path,
            keyframes=keyframe_result.keyframes,
            track_id_map=visual_result.track_id_map,
            key_frame_coordinates=visual_result.key_frame_coordinates,
            artifacts_dir=artifacts_dir / "action_planning",
        )

        self.last_result = self.post_process(
            action_result
        )

        return self.last_result

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
            and len(output_data.steps) != 1
        ):
            raise ValueError(
                "A completed action plan must contain exactly one step."
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

        artifacts_dir = kwargs.get("artifacts_dir")

        if artifacts_dir is not None:
            self.artifacts_dir = (
                Path(artifacts_dir)
                .expanduser()
                .resolve()
            )

            self.artifacts_dir.mkdir(
                parents=True,
                exist_ok=True,
            )