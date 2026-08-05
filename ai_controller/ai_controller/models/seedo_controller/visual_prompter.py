from __future__ import annotations

from pathlib import Path
from tempfile import mkdtemp

from track_objects import run_visual_prompting
from results import VisualPromptingResult


class VisualPrompter:
    """Task-agnostic wrapper around the SeeDo visual-prompting stage."""

    def __init__(
        self,
        grounding_config: str | Path = (
            "/opt/checkpoints/seedo/groundingdino/"
            "GroundingDINO_SwinB.cfg.py"
        ),
        grounding_checkpoint: str | Path = (
            "/opt/checkpoints/seedo/groundingdino/"
            "groundingdino_swinb_cogcoor.pth"
        ),
        bert_model: str | Path = (
            "/opt/checkpoints/seedo/bert-base-uncased"
        ),
        sam_checkpoint: str | Path = (
            "/opt/checkpoints/seedo/sam/sam_vit_h_4b8939.pth"
        ),
        sam2_checkpoint: str | Path = (
            "/opt/checkpoints/seedo/sam2/sam2_hiera_large.pt"
        ),
        objects: str | None = None,
    ) -> None:
        self.grounding_config = Path(grounding_config)
        self.grounding_checkpoint = Path(grounding_checkpoint)
        self.bert_model = Path(bert_model)
        self.sam_checkpoint = Path(sam_checkpoint)
        self.sam2_checkpoint = Path(sam2_checkpoint)
        self.objects = objects

    def run(
        self,
        video_path: str | Path,
        keyframes: tuple[int, ...],
        artifacts_dir: str | Path | None = None,
    ) -> VisualPromptingResult:
        """Run visual prompting on one video using selected keyframes."""

        normalized_video_path = self._validate_video_path(video_path)
        normalized_keyframes = self._validate_keyframes(keyframes)
        normalized_artifacts_dir = self._prepare_artifacts_dir(
            artifacts_dir
        )

        self._validate_model_paths()

        output_video_path = (
            normalized_artifacts_dir
            / f"{normalized_video_path.stem}-tracked.mp4"
        )

        print("=== VisualPrompter configuration ===")
        print(f"Video: {normalized_video_path}")
        print(f"Keyframes: {list(normalized_keyframes)}")
        print(f"Artifacts: {normalized_artifacts_dir}")
        print(f"Output video: {output_video_path}")
        print(f"GroundingDINO config: {self.grounding_config}")
        print(f"GroundingDINO checkpoint: {self.grounding_checkpoint}")
        print(f"BERT model: {self.bert_model}")
        print(f"SAM checkpoint: {self.sam_checkpoint}")
        print(f"SAM2 checkpoint: {self.sam2_checkpoint}")
        print("====================================")

        core_result = run_visual_prompting(
            input_video_path=str(normalized_video_path),
            output_video_path=str(output_video_path),
            artifacts_dir=str(normalized_artifacts_dir),
            key_frames=str(list(normalized_keyframes)),
            objects=self.objects,
            grounding_config=str(self.grounding_config),
            grounding_checkpoint=str(self.grounding_checkpoint),
            bert_model=str(self.bert_model),
            sam_checkpoint=str(self.sam_checkpoint),
            sam2_checkpoint=str(self.sam2_checkpoint),
        )

        if core_result is None:
            raise RuntimeError(
                "track_objects.main() returned None."
            )

        annotated_video_path = (
            Path(core_result.annotated_video_path)
            .expanduser()
            .resolve()
        )

        if not annotated_video_path.is_file():
            raise RuntimeError(
                "The annotated video was not created: "
                f"{annotated_video_path}"
            )

        if annotated_video_path.stat().st_size == 0:
            raise RuntimeError(
                f"The annotated video is empty: {annotated_video_path}"
            )

        return core_result

    @staticmethod
    def _validate_video_path(video_path: str | Path) -> Path:
        normalized_video_path = Path(video_path).expanduser().resolve()

        if not normalized_video_path.is_file():
            raise FileNotFoundError(
                f"Input video does not exist: {normalized_video_path}"
            )

        if normalized_video_path.stat().st_size == 0:
            raise ValueError(
                f"Input video is empty: {normalized_video_path}"
            )

        return normalized_video_path

    @staticmethod
    def _validate_keyframes(
        keyframes: tuple[int, ...],
    ) -> tuple[int, ...]:
        normalized_keyframes = tuple(
            int(frame) for frame in keyframes
        )

        if not normalized_keyframes:
            raise ValueError(
                "At least one keyframe must be provided."
            )

        if any(frame < 0 for frame in normalized_keyframes):
            raise ValueError(
                "Keyframe indexes cannot be negative: "
                f"{list(normalized_keyframes)}"
            )

        if tuple(sorted(normalized_keyframes)) != normalized_keyframes:
            raise ValueError(
                "Keyframes are not in chronological order: "
                f"{list(normalized_keyframes)}"
            )

        if len(set(normalized_keyframes)) != len(
            normalized_keyframes
        ):
            raise ValueError(
                "Duplicate keyframes were provided: "
                f"{list(normalized_keyframes)}"
            )

        return normalized_keyframes

    @staticmethod
    def _prepare_artifacts_dir(
        artifacts_dir: str | Path | None,
    ) -> Path:
        if artifacts_dir is None:
            return Path(
                mkdtemp(prefix="seedo_visual_prompting_")
            ).resolve()

        normalized_artifacts_dir = (
            Path(artifacts_dir)
            .expanduser()
            .resolve()
        )
        normalized_artifacts_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        return normalized_artifacts_dir

    def _validate_model_paths(self) -> None:
        required_paths = {
            "GroundingDINO config": self.grounding_config,
            "GroundingDINO checkpoint": self.grounding_checkpoint,
            "BERT model": self.bert_model,
            "SAM checkpoint": self.sam_checkpoint,
            "SAM2 checkpoint": self.sam2_checkpoint,
        }

        for name, path in required_paths.items():
            if not path.exists():
                raise FileNotFoundError(
                    f"{name} does not exist: {path}"
                )