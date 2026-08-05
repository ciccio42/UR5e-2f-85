from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from tempfile import mkdtemp

import cv2
import numpy as np

from get_frame_by_hands import FrameExtractor


@dataclass(frozen=True)
class KeyframeSelectionResult:
    """Structured output produced by the keyframe-selection stage.

    Notes:
        keyframe_images are NumPy arrays in RGB channel order.
    """

    video_path: Path
    keyframes: tuple[int, ...]
    keyframe_images: tuple[np.ndarray, ...]
    artifacts_dir: Path


class KeyframeSelector:
    """Task-agnostic wrapper around SeeDo keyframe selection.

    The selector receives a video and returns the selected frame indexes
    together with their RGB images.
    """

    def __init__(
        self,
        gaussian_sigma: float = 5.0,
        prominence: float = 0.8,
        expected_keyframes: int = 2,
        save_preview: bool = False,
    ) -> None:
        if gaussian_sigma <= 0:
            raise ValueError("gaussian_sigma must be greater than zero")

        if prominence < 0:
            raise ValueError("prominence cannot be negative")

        if expected_keyframes <= 0:
            raise ValueError("expected_keyframes must be greater than zero")

        self.gaussian_sigma = gaussian_sigma
        self.prominence = prominence
        self.expected_keyframes = expected_keyframes
        self.save_preview = save_preview

    def run(
        self,
        video_path: str | Path,
        artifacts_dir: str | Path | None = None,
    ) -> KeyframeSelectionResult:
        """Run keyframe selection on one video."""

        normalized_video_path = self._validate_video_path(video_path)
        normalized_artifacts_dir = self._prepare_artifacts_dir(
            artifacts_dir
        )

        csv_path = normalized_artifacts_dir / "selected_valleys.csv"

        extractor = FrameExtractor(
            video_path=str(normalized_video_path),
            output_dir=str(normalized_artifacts_dir),
            gaussian_sigma=self.gaussian_sigma,
            prominence=self.prominence,
            csv_file=str(csv_path),
        )

        extractor_result = extractor.extract_frames()

        if extractor_result is None:
            raise RuntimeError(
                "FrameExtractor.extract_frames() returned None."
            )

        keyframes = extractor_result.keyframes
        keyframe_images = extractor_result.keyframe_images

        self._validate_keyframes(keyframes)
        self._validate_keyframe_images(
            keyframes=keyframes,
            keyframe_images=keyframe_images,
        )

        if self.save_preview:
            self._save_keyframe_previews(
                keyframes=keyframes,
                keyframe_images=keyframe_images,
                artifacts_dir=normalized_artifacts_dir,
            )

        return KeyframeSelectionResult(
            video_path=normalized_video_path,
            keyframes=keyframes,
            keyframe_images=keyframe_images,
            artifacts_dir=normalized_artifacts_dir,
        )

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

        capture = cv2.VideoCapture(str(normalized_video_path))

        try:
            if not capture.isOpened():
                raise ValueError(
                    f"OpenCV cannot open the input video: "
                    f"{normalized_video_path}"
                )

            frame_count = int(
                capture.get(cv2.CAP_PROP_FRAME_COUNT)
            )

            if frame_count <= 0:
                raise ValueError(
                    f"Input video contains no readable frames: "
                    f"{normalized_video_path}"
                )
        finally:
            capture.release()

        return normalized_video_path

    @staticmethod
    def _prepare_artifacts_dir(
        artifacts_dir: str | Path | None,
    ) -> Path:
        if artifacts_dir is None:
            return Path(
                mkdtemp(prefix="seedo_keyframes_")
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

    def _validate_keyframes(
        self,
        keyframes: tuple[int, ...],
    ) -> None:
        if len(keyframes) != self.expected_keyframes:
            raise ValueError(
                f"Expected {self.expected_keyframes} keyframes, "
                f"but FrameExtractor returned {list(keyframes)}"
            )

        if any(frame < 0 for frame in keyframes):
            raise ValueError(
                f"Keyframe indexes cannot be negative: "
                f"{list(keyframes)}"
            )

        if tuple(sorted(keyframes)) != keyframes:
            raise ValueError(
                f"Keyframes are not in chronological order: "
                f"{list(keyframes)}"
            )

        if len(set(keyframes)) != len(keyframes):
            raise ValueError(
                f"Duplicate keyframes were returned: "
                f"{list(keyframes)}"
            )

    @staticmethod
    def _validate_keyframe_images(
        keyframes: tuple[int, ...],
        keyframe_images: tuple[np.ndarray, ...],
    ) -> None:
        if len(keyframe_images) != len(keyframes):
            raise ValueError(
                "The number of keyframe images does not match the "
                "number of keyframe indexes."
            )

        for frame_index, image in zip(
            keyframes,
            keyframe_images,
            strict=True,
        ):
            if not isinstance(image, np.ndarray):
                raise TypeError(
                    f"Keyframe {frame_index} is not a NumPy array."
                )

            if image.ndim != 3 or image.shape[2] != 3:
                raise ValueError(
                    f"Keyframe {frame_index} has invalid shape "
                    f"{image.shape}; expected HxWx3."
                )

            if image.dtype != np.uint8:
                raise ValueError(
                    f"Keyframe {frame_index} has invalid dtype "
                    f"{image.dtype}; expected uint8."
                )

            if image.size == 0:
                raise ValueError(
                    f"Keyframe {frame_index} is empty."
                )

    @staticmethod
    def _save_keyframe_previews(
        keyframes: tuple[int, ...],
        keyframe_images: tuple[np.ndarray, ...],
        artifacts_dir: Path,
    ) -> None:
        preview_dir = artifacts_dir / "returned_keyframes"

        preview_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        for frame_index, frame_rgb in zip(
            keyframes,
            keyframe_images,
            strict=True,
        ):
            frame_bgr = cv2.cvtColor(
                frame_rgb,
                cv2.COLOR_RGB2BGR,
            )

            output_path = (
                preview_dir
                / f"keyframe_{frame_index:06d}.png"
            )

            if not cv2.imwrite(
                str(output_path),
                frame_bgr,
            ):
                raise RuntimeError(
                    f"Cannot save keyframe preview: {output_path}"
                )