from __future__ import annotations

import base64
import json
import os

from dataclasses import dataclass
from pathlib import Path
from tempfile import mkdtemp
from openai import OpenAI

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
        mode: str = "hand_velocity",
        model: str = "gpt-4o-2024-08-06",
        sample_stride: int = 5,
        gaussian_sigma: float = 5.0,
        prominence: float = 0.8,
        expected_keyframes: int = 2,
        save_preview: bool = False,
    ) -> None:

        self.mode = str(
            mode
        ).strip().lower()

        allowed_modes = {
            "hand_velocity",
            "vlm",
        }

        if self.mode not in allowed_modes:
            raise ValueError(
                "Invalid keyframe selection mode: "
                f"{self.mode!r}. "
                f"Expected one of: {sorted(allowed_modes)}"
            )

        self.model = str(
            model
        ).strip()

        if not self.model:
            raise ValueError(
                "model cannot be empty"
            )

        if sample_stride <= 0:
            raise ValueError(
                "sample_stride must be greater than zero"
            )

        self.sample_stride = int(
            sample_stride
        )

        if gaussian_sigma <= 0:
            raise ValueError(
                "gaussian_sigma must be greater than zero"
            )

        if prominence < 0:
            raise ValueError(
                "prominence cannot be negative"
            )

        if expected_keyframes <= 0:
            raise ValueError(
                "expected_keyframes must be greater than zero"
            )

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

        normalized_video_path = self._validate_video_path(
            video_path
        )

        normalized_artifacts_dir = self._prepare_artifacts_dir(
            artifacts_dir
        )

        if self.mode == "hand_velocity":
            (
                keyframes,
                keyframe_images,
            ) = self._run_hand_velocity(
                video_path=normalized_video_path,
                artifacts_dir=normalized_artifacts_dir,
            )

        elif self.mode == "vlm":
            (
                keyframes,
                keyframe_images,
            ) = self._run_vlm(
                video_path=normalized_video_path,
                artifacts_dir=normalized_artifacts_dir,
            )

        else:
            raise RuntimeError(
                "Unexpected keyframe selection mode: "
                f"{self.mode!r}"
            )

        self._validate_keyframes(
            keyframes
        )

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

    def _run_hand_velocity(
        self,
        video_path: Path,
        artifacts_dir: Path,
    ) -> tuple[
        tuple[int, ...],
        tuple[np.ndarray, ...],
    ]:
        """Run the original hand-velocity keyframe selection."""

        csv_path = (
            artifacts_dir
            / "selected_valleys.csv"
        )

        extractor = FrameExtractor(
            video_path=str(video_path),
            output_dir=str(artifacts_dir),
            gaussian_sigma=self.gaussian_sigma,
            prominence=self.prominence,
            csv_file=str(csv_path),
        )

        extractor_result = (
            extractor.extract_frames()
        )

        if extractor_result is None:
            raise RuntimeError(
                "FrameExtractor.extract_frames() returned None."
            )

        return (
            tuple(
                int(frame)
                for frame
                in extractor_result.keyframes
            ),
            tuple(
                extractor_result.keyframe_images
            ),
        )

    def _run_vlm(
        self,
        video_path: Path,
        artifacts_dir: Path,
    ) -> tuple[
        tuple[int, ...],
        tuple[np.ndarray, ...],
    ]:
        """Select pick/place keyframes with one VLM call."""

        print(
            "[KeyframeSelector/VLM] "
            f"Using model {self.model} "
            f"to select keyframes from {video_path}",
            flush=True,
        )

        if not os.environ.get("OPENAI_API_KEY"):
            raise ValueError(
                "OPENAI_API_KEY is not configured."
            )

        sampled_frames = self._sample_video_frames(
            video_path
        )

        candidate_indices = tuple(
            sampled_frames.keys()
        )

        if len(candidate_indices) < 2:
            raise RuntimeError(
                "At least two candidate frames are required "
                "for VLM keyframe selection."
            )

        # ---------------------------------------------------------
        # Save candidate frames as artifacts.
        # ---------------------------------------------------------

        candidate_frames_dir = (
            artifacts_dir
            / "candidate_frames"
        )

        candidate_frames_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        for (
            frame_index,
            frame_bgr,
        ) in sampled_frames.items():

            output_path = (
                candidate_frames_dir
                / f"frame_{frame_index:06d}.jpg"
            )

            if not cv2.imwrite(
                str(output_path),
                frame_bgr,
            ):
                raise RuntimeError(
                    "Could not save candidate frame: "
                    f"{output_path}"
                )

        # ---------------------------------------------------------
        # Build multimodal VLM request.
        # ---------------------------------------------------------

        prompt = (
            "Identify the two keyframes required by the downstream "
            "robot imitation pipeline.\n\n"

            "You are given chronologically ordered candidate frames "
            "sampled from a human manipulation demonstration.\n\n"

            "PICK FRAME:\n"
            "Select the candidate frame that most clearly shows which "
            "physical object is being grasped or deliberately manipulated "
            "by the human hand. Prefer a frame where the grasp/contact is "
            "already established and the manipulated object's identity is "
            "visually clear. Do not select a frame where the hand is only "
            "approaching the object.\n\n"

            "PLACE FRAME:\n"
            "Select a later candidate frame that most clearly shows the "
            "manipulated object being placed at, into, onto, or with "
            "respect to its destination or reference object. Prefer a "
            "frame near the placement/release event where the relationship "
            "between the manipulated object and its destination is clearly "
            "visible. The destination is not necessarily a container; it "
            "may also be another object such as a peg.\n\n"

            "RULES:\n"
            "- pick_frame must occur before place_frame.\n"
            "- Select only frame indexes explicitly provided below.\n"
            "- Use only the visual evidence in the supplied frames.\n"
            "- Do not invent intermediate frame indexes.\n"
            "- If the interaction cannot be determined reliably, return "
            "status='ambiguous'.\n\n"

            "Candidate frame indexes: "
            f"{list(candidate_indices)}"
        )

        content = [
            {
                "type": "text",
                "text": prompt,
            }
        ]

        for (
            frame_index,
            frame_bgr,
        ) in sampled_frames.items():

            success, encoded_frame = cv2.imencode(
                ".jpg",
                frame_bgr,
                [
                    cv2.IMWRITE_JPEG_QUALITY,
                    85,
                ],
            )

            if not success:
                raise RuntimeError(
                    "Could not encode candidate frame "
                    f"{frame_index}."
                )

            frame_base64 = base64.b64encode(
                encoded_frame.tobytes()
            ).decode(
                "ascii"
            )

            content.append(
                {
                    "type": "text",
                    "text": (
                        "Candidate frame "
                        f"{frame_index}"
                    ),
                }
            )

            content.append(
                {
                    "type": "image_url",
                    "image_url": {
                        "url": (
                            "data:image/jpeg;base64,"
                            + frame_base64
                        ),
                        "detail": "low",
                    },
                }
            )

        messages = [
            {
                "role": "system",
                "content": (
                    "You are the temporal event-localization stage "
                    "of a robot imitation system. "
                    "Use only the supplied visual evidence. "
                    "Your task is to identify the most informative "
                    "pick and place frames for the downstream "
                    "perception and action-planning stages."
                ),
            },
            {
                "role": "user",
                "content": content,
            },
        ]

        # ---------------------------------------------------------
        # Single VLM call.
        # ---------------------------------------------------------

        response = OpenAI().chat.completions.create(
            model=self.model,
            messages=messages,
            temperature=0,
            max_tokens=500,
            response_format={
                "type": "json_schema",
                "json_schema": {
                    "name": "keyframe_selection",
                    "strict": True,
                    "schema": {
                        "type": "object",
                        "properties": {
                            "status": {
                                "type": "string",
                                "enum": [
                                    "completed",
                                    "ambiguous",
                                ],
                            },
                            "pick_frame": {
                                "type": "integer",
                            },
                            "place_frame": {
                                "type": "integer",
                            },
                            "pick_evidence": {
                                "type": "string",
                            },
                            "place_evidence": {
                                "type": "string",
                            },
                            "ambiguity": {
                                "type": "string",
                            },
                        },
                        "required": [
                            "status",
                            "pick_frame",
                            "place_frame",
                            "pick_evidence",
                            "place_evidence",
                            "ambiguity",
                        ],
                        "additionalProperties": False,
                    },
                },
            },
        )

        raw_response = (
            response
            .choices[0]
            .message
            .content
        )

        if not raw_response:
            raise RuntimeError(
                "The VLM returned an empty keyframe-selection response."
            )

        selection = json.loads(
            raw_response
        )

        # ---------------------------------------------------------
        # Save VLM result.
        # ---------------------------------------------------------

        result_path = (
            artifacts_dir
            / "vlm_keyframe_selection.json"
        )

        with result_path.open(
            "w",
            encoding="utf-8",
        ) as stream:
            json.dump(
                {   
                    "video_path": str(video_path),
                    "model": self.model,
                    "sample_stride": self.sample_stride,
                    "candidate_frames": list(
                        candidate_indices
                    ),
                    "selection": selection,
                },
                stream,
                indent=2,
                ensure_ascii=False,
            )

        # ---------------------------------------------------------
        # Validate VLM result.
        # ---------------------------------------------------------

        if (
            selection["status"]
            != "completed"
        ):
            raise RuntimeError(
                "VLM keyframe selection is ambiguous: "
                f"{selection['ambiguity']}"
            )

        pick_frame = int(
            selection["pick_frame"]
        )

        place_frame = int(
            selection["place_frame"]
        )

        if (
            pick_frame
            not in sampled_frames
        ):
            raise ValueError(
                "The VLM selected a pick frame that was not "
                "provided as a candidate: "
                f"{pick_frame}"
            )

        if (
            place_frame
            not in sampled_frames
        ):
            raise ValueError(
                "The VLM selected a place frame that was not "
                "provided as a candidate: "
                f"{place_frame}"
            )

        if pick_frame >= place_frame:
            raise ValueError(
                "Invalid VLM keyframe ordering: "
                f"pick={pick_frame}, "
                f"place={place_frame}"
            )

        keyframes = (
            pick_frame,
            place_frame,
        )

        keyframe_images = (
            cv2.cvtColor(
                sampled_frames[pick_frame],
                cv2.COLOR_BGR2RGB,
            ),
            cv2.cvtColor(
                sampled_frames[place_frame],
                cv2.COLOR_BGR2RGB,
            ),
        )

        print(
            "[KeyframeSelector/VLM] "
            f"pick_frame={pick_frame}, "
            f"place_frame={place_frame}",
            flush=True,
        )

        return (
            keyframes,
            keyframe_images,
        )

    def _sample_video_frames(
        self,
        video_path: Path,
    ) -> dict[int, np.ndarray]:
        """Sample frames from the video using the configured stride."""

        capture = cv2.VideoCapture(
            str(video_path)
        )

        if not capture.isOpened():
            raise ValueError(
                "OpenCV cannot open the input video: "
                f"{video_path}"
            )

        sampled_frames: dict[
            int,
            np.ndarray,
        ] = {}

        frame_index = 0

        try:
            while True:
                ret, frame_bgr = capture.read()

                if not ret:
                    break

                if (
                    frame_index
                    % self.sample_stride
                    == 0
                ):
                    sampled_frames[
                        frame_index
                    ] = frame_bgr

                frame_index += 1

        finally:
            capture.release()

        if not sampled_frames:
            raise RuntimeError(
                "No frames were sampled from the video."
            )

        return sampled_frames

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