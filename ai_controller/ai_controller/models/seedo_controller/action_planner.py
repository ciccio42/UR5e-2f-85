from pathlib import Path

from results import ActionPlanningResult
from vlm import generate_action_plan


class ActionPlanner:
    """Wrapper around the SeeDo action-planning stage."""

    def __init__(
        self,
        model: str = "gpt-4o-2024-08-06",
    ) -> None:
        self.model = model

    def run(
        self,
        annotated_video_path: str | Path,
        keyframes: tuple[int, int],
        track_id_map: dict[int, dict[str, object]],
        key_frame_coordinates: dict[str, list[str]],
        artifacts_dir: str | Path,
    ) -> ActionPlanningResult:
        normalized_video_path = (
            Path(annotated_video_path)
            .expanduser()
            .resolve()
        )

        normalized_artifacts_dir = (
            Path(artifacts_dir)
            .expanduser()
            .resolve()
        )

        if not normalized_video_path.is_file():
            raise FileNotFoundError(
                "Annotated video does not exist: "
                f"{normalized_video_path}"
            )

        if normalized_video_path.stat().st_size == 0:
            raise ValueError(
                "Annotated video is empty: "
                f"{normalized_video_path}"
            )

        normalized_keyframes = tuple(
            int(frame)
            for frame in keyframes
        )

        if len(normalized_keyframes) != 2:
            raise ValueError(
                "Action planning requires exactly two keyframes."
            )

        if normalized_keyframes[0] >= normalized_keyframes[1]:
            raise ValueError(
                "The pick keyframe must precede the place keyframe: "
                f"{normalized_keyframes}"
            )

        if not track_id_map:
            raise ValueError(
                "track_id_map cannot be empty."
            )

        if not key_frame_coordinates:
            raise ValueError(
                "key_frame_coordinates cannot be empty."
            )

        normalized_artifacts_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        result = generate_action_plan(
            annotated_video_path=normalized_video_path,
            keyframes=normalized_keyframes,
            track_id_map=track_id_map,
            key_frame_coordinates=key_frame_coordinates,
            artifacts_dir=normalized_artifacts_dir,
            model=self.model,
        )

        if result is None:
            raise RuntimeError(
                "generate_action_plan() returned None."
            )

        return result