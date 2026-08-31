from __future__ import annotations

import torch
import numpy as np
import cv2
import json
import base64
import os
import re

from collections import Counter
from openai import OpenAI

from pathlib import Path
from typing import Any
from GroundingDINO.groundingdino.util import box_ops
from ai_controller.script_controller.vision import (
    load_camera_calibration,
    robust_depth_at,
    deproject_pixel,
    camera_point_to_aruco,
    aruco_point_to_table0,
)
from results import (
    DetectedObject,
    RawSceneObject,
    RawSceneState,
    ScenePerceptionResult,
)
from GroundingDINO.groundingdino.util.inference import (
    load_image_from_array,
    predict,
)
from segment_anything import (
    SamPredictor,
    build_sam,
)
from track_objects import (
    filter_oversized_storage_bin_detections,
    load_groundingdino_model,
)

from ai_controller.models.seedo_controller.timing_utils import TIMING


class ScenePerceiver:
    """Runtime RGB-D perception used by SeeDo.

    The baseline performs perception only once, at timestep 0.

    This first implementation provides the geometric conversion from an
    RGB-image pixel to a 3D point expressed in base_link. Object detection
    and segmentation will be added on top of this component later.
    """

    def __init__(
        self,
        camera_calibration_path: str | Path,
        camera_name: str = "zed_front",
        grounding_config: str | Path | None = None,
        grounding_checkpoint: str | Path | None = None,
        bert_model: str | Path | None = None,
        sam_checkpoint: str | Path | None = None,
        detector_labels: list[str] | tuple[str, ...] | None = None,
    ) -> None:
        self.camera_calibration_path = (
            Path(camera_calibration_path)
            .expanduser()
            .resolve()
        )

        if not self.camera_calibration_path.is_file():
            raise FileNotFoundError(
                "Camera calibration file does not exist: "
                f"{self.camera_calibration_path}"
            )

        self.camera_name = str(camera_name)

        self.camera_calibration = load_camera_calibration(
            str(self.camera_calibration_path)
        )

        if self.camera_name not in self.camera_calibration:
            raise KeyError(
                f"Camera '{self.camera_name}' is not present in "
                f"{self.camera_calibration_path}."
            )

        self.grounding_config = (
            Path(grounding_config).expanduser().resolve()
            if grounding_config is not None
            else None
        )

        self.grounding_checkpoint = (
            Path(grounding_checkpoint).expanduser().resolve()
            if grounding_checkpoint is not None
            else None
        )

        self.bert_model = (
            Path(bert_model).expanduser().resolve()
            if bert_model is not None
            else None
        )

        self.sam_checkpoint = (
            Path(sam_checkpoint).expanduser().resolve()
            if sam_checkpoint is not None
            else None
        )

        required_model_paths = {
            "GroundingDINO config": self.grounding_config,
            "GroundingDINO checkpoint": self.grounding_checkpoint,
            "BERT model": self.bert_model,
            "SAM checkpoint": self.sam_checkpoint,
        }

        for name, path in required_model_paths.items():
            if path is None:
                raise ValueError(
                    f"{name} has not been configured."
                )

            if not path.exists():
                raise FileNotFoundError(
                    f"{name} does not exist: {path}"
                )
        
        if detector_labels is None:
            raise ValueError(
                "At least one detector label must be configured."
            )

        self.detector_labels = tuple(
            str(label).strip()
            for label in detector_labels
            if str(label).strip()
        )

        if not self.detector_labels:
            raise ValueError(
                "At least one non-empty detector label must be configured."
            )

        self.device = torch.device(
            "cuda:0"
            if torch.cuda.is_available()
            else "cpu"
        )

        if self.device.type == "cuda":
            torch.cuda.set_device(self.device)

        # Models are loaded lazily when runtime scene perception is first used.
        # This avoids keeping a second GroundingDINO + SAM instance on the GPU
        # while the demonstration video is being processed.
        self.grounding_model = None
        self.sam = None
        self.sam_predictor = None

    def pixel_to_base_link(
        self,
        pixel: tuple[int, int],
        depth_image: np.ndarray,
        camera_matrix: np.ndarray,
        base_to_table_rotation: np.ndarray,
        base_to_table_translation: np.ndarray,
    ) -> tuple[float, float, float]:
        """Convert an RGB pixel into a 3D point in base_link."""

        u, v = int(pixel[0]), int(pixel[1])

        if depth_image.ndim != 2:
            raise ValueError(
                "Depth image must be a 2D array."
            )

        if camera_matrix.shape != (3, 3):
            raise ValueError(
                "Camera matrix must have shape (3, 3)."
            )

        base_to_table_rotation = np.asarray(
            base_to_table_rotation,
            dtype=np.float64,
        )

        base_to_table_translation = np.asarray(
            base_to_table_translation,
            dtype=np.float64,
        )

        if base_to_table_rotation.shape != (3, 3):
            raise ValueError(
                "base_to_table_rotation must have shape (3, 3)."
            )

        if base_to_table_translation.shape != (3,):
            raise ValueError(
                "base_to_table_translation must have shape (3,)."
            )

        depth = robust_depth_at(
            depth_image,
            u,
            v,
        )

        point_camera = deproject_pixel(
            u,
            v,
            depth,
            camera_matrix,
        )

        camera_calibration = self.camera_calibration[
            self.camera_name
        ]

        point_aruco = camera_point_to_aruco(
            point_camera,
            camera_calibration,
        )

        point_table = aruco_point_to_table0(
            point_aruco
        )

        point_base = (
            base_to_table_rotation
            @ np.asarray(point_table, dtype=np.float64)
            + base_to_table_translation
        )

        return (
            float(point_base[0]),
            float(point_base[1]),
            float(point_base[2]),
        )
    
    def _ensure_models_loaded(self) -> None:
        """Load GroundingDINO and SAM only when scene perception is needed."""

        if (
            self.grounding_model is not None
            and self.sam is not None
            and self.sam_predictor is not None
        ):
            return

        print(
            "[ScenePerceiver] Loading GroundingDINO and SAM...",
            flush=True,
        )

        self.grounding_model = load_groundingdino_model(
            str(self.grounding_config),
            str(self.grounding_checkpoint),
            device=str(self.device),
            text_encoder_path=str(self.bert_model),
        )

        self.sam = build_sam(
            checkpoint=str(self.sam_checkpoint)
        )

        self.sam.to(
            device=self.device
        )

        self.sam_predictor = SamPredictor(
            self.sam
        )

        print(
            "[ScenePerceiver] GroundingDINO and SAM loaded.",
            flush=True,
        )

    def camera_matrix_from_info(
        self,
        camera_info: dict,
    ) -> np.ndarray:
        """Build the 3x3 intrinsic camera matrix from CameraInfo data."""

        if not isinstance(camera_info, dict):
            raise TypeError(
                "camera_info must be a dictionary."
            )

        if "k" not in camera_info:
            raise ValueError(
                "camera_info does not contain the intrinsic matrix 'k'."
            )

        k = np.asarray(
            camera_info["k"],
            dtype=np.float64,
        )

        if k.size != 9:
            raise ValueError(
                "camera_info['k'] must contain exactly 9 values."
            )

        return k.reshape(3, 3)

    def pixel_to_base_link_from_info(
        self,
        pixel: tuple[int, int],
        depth_image: np.ndarray,
        camera_info: dict,
        base_to_table_rotation: np.ndarray,
        base_to_table_translation: np.ndarray,
    ) -> tuple[float, float, float]:
        camera_matrix = self.camera_matrix_from_info(
            camera_info
        )

        return self.pixel_to_base_link(
            pixel=pixel,
            depth_image=depth_image,
            camera_matrix=camera_matrix,
            base_to_table_rotation=base_to_table_rotation,
            base_to_table_translation=base_to_table_translation,
        )

    def _discover_detector_labels(
        self,
        rgb_image: np.ndarray,
    ) -> list[str]:
        """Discover semantic detector labels from the runtime RGB scene."""

        if not isinstance(rgb_image, np.ndarray):
            raise TypeError(
                "rgb_image must be a numpy array."
            )

        if not os.environ.get("OPENAI_API_KEY"):
            raise ValueError(
                "OPENAI_API_KEY is not configured."
            )

        # rgb_image is RGB, while OpenCV encoding expects BGR.
        image_bgr = cv2.cvtColor(
            rgb_image,
            cv2.COLOR_RGB2BGR,
        )

        ok, encoded_image = cv2.imencode(
            ".jpg",
            image_bgr,
            [
                cv2.IMWRITE_JPEG_QUALITY,
                95,
            ],
        )

        if not ok:
            raise RuntimeError(
                "Could not encode runtime RGB image."
            )

        image_base64 = base64.b64encode(
            encoded_image.tobytes()
        ).decode("ascii")

        messages = [
            {
                "role": "system",
                "content": (
                    "You are a visual object detector whose output will be used "
                    "directly as text queries for GroundingDINO. "
                    "The scene contains colored cubes and storage bins. "
                    "For every cube, include its visible color in the detector "
                    "label using the exact form '<color> cube'. "
                    "For every storage bin, always use the exact detector label "
                    "'storage bin' without adding color, position, material, "
                    "or other attributes."
                ),
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": (
                            "Inspect the physical objects visible on the table "
                            "and classify them using these rules:\n"
                            "1. For every visible cube or graspable colored block, "
                            "identify its visible color and return '<color> cube'.\n"
                            "The only valid cube colors in this benchmark are:"
                            "red, green, blue, and yellow."
                            "For every cube, the detector label MUST therefore be exactly one of:"
                            "'red cube', 'green cube', 'blue cube', or 'yellow cube'."
                            "Do not use any other cube color.\n"
                            "2. Examples are 'red cube', 'green cube', "
                            "'blue cube', and 'yellow cube'.\n"
                            "3. For every bin, box, tray, container, or receptacle, "
                            "return exactly 'storage bin'.\n"
                            "4. Count every physical instance separately.\n"
                            "5. Repeat 'storage bin' once for every visible bin.\n"
                            "6. Do not include the robot, gripper, table, "
                            "or background objects.\n"
                            "7. Never use spatial descriptions such as "
                            "'first bin from the left'.\n"
                            "8. Do not add objects that are not visible.\n\n"
                            "Return exactly two lines and no additional explanation:\n"
                            "Number: <total number of instances>\n"
                            "Objects: <comma-separated detector labels, "
                            "repeated once per instance>"
                        ),
                    },
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": (
                                "data:image/jpeg;base64,"
                                + image_base64
                            ),
                            "detail": "high",
                        },
                    },
                ],
            },
        ]

        response = OpenAI().chat.completions.create(
            model="gpt-4o-2024-08-06",
            messages=messages,
            temperature=0,
            max_tokens=300,
        )

        raw_output = (
            response
            .choices[0]
            .message
            .content
        )

        if not raw_output:
            raise RuntimeError(
                "Runtime object discovery returned no output."
            )

        number_match = re.search(
            r"Number:\s*(\d+)",
            raw_output,
        )

        objects_match = re.search(
            r"Objects:\s*(.+)",
            raw_output,
        )

        if objects_match is None:
            raise RuntimeError(
                "Could not parse object list from VLM output: "
                f"{raw_output!r}"
            )

        detector_labels = [
            label.strip().lower()
            for label in objects_match.group(1).split(",")
            if label.strip()
        ]

        if not detector_labels:
            raise RuntimeError(
                "VLM object discovery returned an empty object list."
            )

        reported_count = (
            int(number_match.group(1))
            if number_match is not None
            else None
        )

        print(
            "[ScenePerceiver] Discovered detector labels: "
            f"{detector_labels}"
        )

        if (
            reported_count is not None
            and reported_count != len(detector_labels)
        ):
            print(
                "[ScenePerceiver] WARNING: "
                f"VLM reported {reported_count} objects but "
                f"returned {len(detector_labels)} labels."
            )

        return detector_labels

    def _detect_objects(
        self,
        rgb_image: np.ndarray,
    ) -> list[DetectedObject]:
        """Detect and segment configured object categories in one RGB frame."""
        
        self._ensure_models_loaded()

        image_source, image = load_image_from_array(
            rgb_image
        )

        detector_labels = self._discover_detector_labels(
            rgb_image
        )

        object_counts = Counter(
            detector_labels
        )

        semantic_object_box_threshold = 0.20
        default_box_threshold = 0.30
        text_threshold = 0.25

        detected_boxes: list[torch.Tensor] = []
        detected_labels: list[str] = []
        detected_confidences: list[float] = []

        semantic_candidates: list[dict[str, Any]] = []

        for detector_label, requested_count in object_counts.items():

            timing_label = detector_label.replace(" ", "_")

            with TIMING.measure(
                f"scene.grounding_dino.{timing_label}",
                cuda=True,
            ):

                is_storage_bin = (
                    detector_label.strip().lower()
                    == "storage bin"
                )

                current_box_threshold = (
                    default_box_threshold
                    if is_storage_bin
                    else semantic_object_box_threshold
                )

                boxes, logits, phrases = predict(
                    model=self.grounding_model,
                    image=image,
                    caption=detector_label,
                    box_threshold=current_box_threshold,
                    text_threshold=text_threshold,
                    device=self.device,
                )

                print(
                    f"\n[DINO] {detector_label!r}: "
                    f"{len(boxes)} candidates"
                )

                for i, (box, logit) in enumerate(
                    zip(boxes, logits)
                ):
                    print(
                        f"  [{i}] "
                        f"box={box.detach().cpu().tolist()} "
                        f"score="
                        f"{float(logit.detach().cpu().item()):.4f}"
                    )

            # Small-object sanity filter for all non-bin
            # semantic detections.
            if not is_storage_bin:

                box_areas = (
                    boxes[:, 2]
                    * boxes[:, 3]
                )

                keep_mask = (
                    box_areas < 0.1
                )

                boxes = boxes[
                    keep_mask
                ]

                logits = logits[
                    keep_mask
                ]

                keep_values = (
                    keep_mask
                    .detach()
                    .cpu()
                    .tolist()
                )

                phrases = [
                    phrase
                    for phrase, keep in zip(
                        phrases,
                        keep_values,
                    )
                    if keep
                ]

            boxes, logits, phrases, diagnostics = (
                filter_oversized_storage_bin_detections(
                    boxes=boxes,
                    logits=logits,
                    phrases=phrases,
                    detector_label=detector_label,
                )
            )

            if diagnostics["removed_count"]:
                print(
                    "[ScenePerceiver] Oversized detections removed: "
                    f"{diagnostics}"
                )

            # ---------------------------------------------------------
            # Semantic objects
            # ---------------------------------------------------------

            if not is_storage_bin:

                for box, logit in zip(
                    boxes,
                    logits,
                ):
                    semantic_candidates.append(
                        {
                            "label": detector_label,
                            "box": box,
                            "confidence": float(
                                logit
                                .detach()
                                .cpu()
                                .item()
                            ),
                        }
                    )

                continue

            # ---------------------------------------------------------
            # Storage bins
            # ---------------------------------------------------------

            selected_count = min(
                requested_count,
                int(boxes.shape[0]),
            )

            boxes = boxes[
                :selected_count
            ]

            logits = logits[
                :selected_count
            ]

            phrases = phrases[
                :selected_count
            ]

            for box, logit in zip(
                boxes,
                logits,
            ):
                detected_boxes.append(
                    box.unsqueeze(0)
                )

                detected_labels.append(
                    detector_label
                )

                detected_confidences.append(
                    float(
                        logit
                        .detach()
                        .cpu()
                        .item()
                    )
                )

        # -------------------------------------------------------------
        # Resolve competing semantic detections globally
        # -------------------------------------------------------------

        semantic_candidates.sort(
            key=lambda candidate: candidate[
                "confidence"
            ],
            reverse=True,
        )

        accepted_semantic: list[
            dict[str, Any]
        ] = []

        accepted_counts: dict[str, int] = {
            label: 0
            for label in object_counts
            if (
                label.strip().lower()
                != "storage bin"
            )
        }

        semantic_iou_threshold = 0.7

        for candidate in semantic_candidates:

            label = candidate[
                "label"
            ]

            requested_count = (
                object_counts[
                    label
                ]
            )

            if (
                accepted_counts[label]
                >= requested_count
            ):
                continue

            candidate_box_xyxy = (
                box_ops.box_cxcywh_to_xyxy(
                    candidate[
                        "box"
                    ].unsqueeze(0)
                )
            )

            conflicting_detection = None
            conflicting_iou = 0.0

            for accepted in accepted_semantic:

                accepted_box_xyxy = (
                    box_ops.box_cxcywh_to_xyxy(
                        accepted[
                            "box"
                        ].unsqueeze(0)
                    )
                )

                iou_matrix, _ = (
                    box_ops.box_iou(
                        candidate_box_xyxy,
                        accepted_box_xyxy,
                    )
                )

                iou = float(
                    iou_matrix[
                        0,
                        0,
                    ]
                    .detach()
                    .cpu()
                    .item()
                )

                if (
                    iou
                    >= semantic_iou_threshold
                ):
                    conflicting_detection = (
                        accepted
                    )

                    conflicting_iou = iou
                    break

            if (
                conflicting_detection
                is not None
            ):
                print(
                    "[ScenePerceiver] Semantic IoU conflict: "
                    f"{label!r} "
                    f"score={candidate['confidence']:.4f} "
                    "rejected because it overlaps "
                    f"{conflicting_detection['label']!r} "
                    "with "
                    f"score="
                    f"{conflicting_detection['confidence']:.4f}, "
                    f"IoU={conflicting_iou:.4f}"
                )

                continue

            accepted_semantic.append(
                candidate
            )

            accepted_counts[
                label
            ] += 1

        # -------------------------------------------------------------
        # Fallback for unresolved semantic labels
        # -------------------------------------------------------------
        #
        # Example in this cube-specific branch:
        #
        # blue cube
        #   -> dark blue cube
        #   -> light blue cube
        #   -> cube
        #
        # The original label "blue cube" is always preserved.
        # -------------------------------------------------------------

        for label, requested_count in object_counts.items():

            if (
                label.strip().lower()
                == "storage bin"
            ):
                continue

            resolved_count = accepted_counts[
                label
            ]

            missing_count = (
                requested_count
                - resolved_count
            )

            if missing_count <= 0:
                continue

            print(
                "[ScenePerceiver] Unresolved semantic label: "
                f"{label!r}, "
                f"requested={requested_count}, "
                f"resolved={resolved_count}"
            )

            label_parts = (
                label
                .strip()
                .lower()
                .split()
            )

            if len(label_parts) < 2:
                raise RuntimeError(
                    "Semantic detector label does not follow "
                    "the expected '<color> <object type>' format: "
                    f"{label!r}"
                )

            color = label_parts[0]
            object_type = label_parts[-1]

            fallback_queries = [
                f"dark {color} {object_type}",
                f"light {color} {object_type}",
                object_type,
            ]

            for fallback_query in fallback_queries:

                if missing_count <= 0:
                    break

                print(
                    "[ScenePerceiver] Trying fallback: "
                    f"{label!r} -> {fallback_query!r}"
                )

                timing_query = (
                    fallback_query
                    .replace(" ", "_")
                )

                with TIMING.measure(
                    f"scene.grounding_dino.fallback_{timing_query}",
                    cuda=True,
                ):
                    (
                        fallback_boxes,
                        fallback_logits,
                        fallback_phrases,
                    ) = predict(
                        model=self.grounding_model,
                        image=image,
                        caption=fallback_query,
                        box_threshold=(
                            semantic_object_box_threshold
                        ),
                        text_threshold=text_threshold,
                        device=self.device,
                    )

                print(
                    f"[DINO FALLBACK] {label!r} "
                    f"using {fallback_query!r}: "
                    f"{len(fallback_boxes)} candidates"
                )

                for i, (box, logit) in enumerate(
                    zip(
                        fallback_boxes,
                        fallback_logits,
                    )
                ):
                    print(
                        f"  [{i}] "
                        f"box={box.detach().cpu().tolist()} "
                        f"score="
                        f"{float(logit.detach().cpu().item()):.4f}"
                    )

                # Same sanity filter as the first semantic pass.
                if len(fallback_boxes) > 0:

                    box_areas = (
                        fallback_boxes[:, 2]
                        * fallback_boxes[:, 3]
                    )

                    keep_mask = (
                        box_areas < 0.1
                    )

                    fallback_boxes = (
                        fallback_boxes[
                            keep_mask
                        ]
                    )

                    fallback_logits = (
                        fallback_logits[
                            keep_mask
                        ]
                    )

                    keep_values = (
                        keep_mask
                        .detach()
                        .cpu()
                        .tolist()
                    )

                    fallback_phrases = [
                        phrase
                        for phrase, keep in zip(
                            fallback_phrases,
                            keep_values,
                        )
                        if keep
                    ]

                fallback_candidates = [
                    {
                        # Preserve ORIGINAL VLM label.
                        "label": label,
                        "box": box,
                        "confidence": float(
                            logit
                            .detach()
                            .cpu()
                            .item()
                        ),
                    }
                    for box, logit in zip(
                        fallback_boxes,
                        fallback_logits,
                    )
                ]

                fallback_candidates.sort(
                    key=lambda candidate: candidate[
                        "confidence"
                    ],
                    reverse=True,
                )

                for candidate in fallback_candidates:

                    if missing_count <= 0:
                        break

                    candidate_box_xyxy = (
                        box_ops.box_cxcywh_to_xyxy(
                            candidate[
                                "box"
                            ].unsqueeze(0)
                        )
                    )

                    conflict = False

                    # ---------------------------------------------
                    # Protect semantic objects already assigned.
                    # ---------------------------------------------

                    for accepted in accepted_semantic:

                        accepted_box_xyxy = (
                            box_ops.box_cxcywh_to_xyxy(
                                accepted[
                                    "box"
                                ].unsqueeze(0)
                            )
                        )

                        iou_matrix, _ = (
                            box_ops.box_iou(
                                candidate_box_xyxy,
                                accepted_box_xyxy,
                            )
                        )

                        iou = float(
                            iou_matrix[
                                0,
                                0,
                            ]
                            .detach()
                            .cpu()
                            .item()
                        )

                        if (
                            iou
                            >= semantic_iou_threshold
                        ):
                            print(
                                "[ScenePerceiver] "
                                "Fallback candidate rejected: "
                                f"{label!r} "
                                f"query={fallback_query!r} "
                                f"score="
                                f"{candidate['confidence']:.4f} "
                                "overlaps semantic object "
                                f"{accepted['label']!r}, "
                                f"IoU={iou:.4f}"
                            )

                            conflict = True
                            break

                    if conflict:
                        continue

                    # ---------------------------------------------
                    # Protect storage-bin regions.
                    # ---------------------------------------------

                    for (
                        existing_box,
                        existing_label,
                    ) in zip(
                        detected_boxes,
                        detected_labels,
                    ):

                        if (
                            existing_label
                            .strip()
                            .lower()
                            != "storage bin"
                        ):
                            continue

                        existing_box_xyxy = (
                            box_ops.box_cxcywh_to_xyxy(
                                existing_box
                            )
                        )

                        iou_matrix, _ = (
                            box_ops.box_iou(
                                candidate_box_xyxy,
                                existing_box_xyxy,
                            )
                        )

                        iou = float(
                            iou_matrix[
                                0,
                                0,
                            ]
                            .detach()
                            .cpu()
                            .item()
                        )

                        if (
                            iou
                            >= semantic_iou_threshold
                        ):
                            print(
                                "[ScenePerceiver] "
                                "Fallback candidate rejected: "
                                f"{label!r} "
                                f"query={fallback_query!r} "
                                f"score="
                                f"{candidate['confidence']:.4f} "
                                "overlaps storage bin, "
                                f"IoU={iou:.4f}"
                            )

                            conflict = True
                            break

                    if conflict:
                        continue

                    print(
                        "[ScenePerceiver] "
                        "Fallback candidate accepted: "
                        f"{label!r} "
                        f"localized using "
                        f"{fallback_query!r}, "
                        f"score="
                        f"{candidate['confidence']:.4f}"
                    )

                    accepted_semantic.append(
                        candidate
                    )

                    accepted_counts[
                        label
                    ] += 1

                    missing_count -= 1

        # -------------------------------------------------------------
        # Add resolved semantic detections
        # -------------------------------------------------------------

        for candidate in accepted_semantic:

            detected_boxes.append(
                candidate[
                    "box"
                ].unsqueeze(0)
            )

            detected_labels.append(
                candidate[
                    "label"
                ]
            )

            detected_confidences.append(
                candidate[
                    "confidence"
                ]
            )

        if not detected_boxes:
            return []

        boxes = torch.cat(
            detected_boxes,
            dim=0,
        )

        height, width, _ = image_source.shape

        boxes_xyxy = (
            box_ops.box_cxcywh_to_xyxy(boxes)
            * torch.tensor(
                [
                    width,
                    height,
                    width,
                    height,
                ],
                dtype=boxes.dtype,
                device=boxes.device,
            )
        )
        
        with TIMING.measure(
            "scene.sam",
            cuda=True,
        ):

            self.sam_predictor.set_image(
                image_source
            )

            transformed_boxes = (
                self.sam_predictor.transform
                .apply_boxes_torch(
                    boxes_xyxy,
                    image_source.shape[:2],
                )
                .to(self.device)
            )

            masks, _, _ = self.sam_predictor.predict_torch(
                point_coords=None,
                point_labels=None,
                boxes=transformed_boxes,
                multimask_output=False,
            )

        masks_np = (
            masks
            .detach()
            .cpu()
            .numpy()
        )

        detections: list[DetectedObject] = []

        image_area = height * width

        label_counters: dict[str, int] = {
            label: 0
            for label in object_counts
        }

        for detector_label, confidence, mask in zip(
            detected_labels,
            detected_confidences,
            masks_np,
        ):
            object_mask = mask[0] > 0

            mask_area = int(
                np.count_nonzero(
                    object_mask
                )
            )

            # Same sanity check used by SeeDo visual prompting.
            if mask_area > image_area * 0.3:
                continue

            mask_indices = np.argwhere(
                object_mask
            )

            if len(mask_indices) == 0:
                continue

            avg_y, avg_x = np.mean(
                mask_indices,
                axis=0,
            )

            pixel = (
                int(avg_x),
                int(avg_y),
            )

            object_index = label_counters[
                detector_label
            ]

            label_counters[
                detector_label
            ] += 1

            normalized_label = (
                detector_label
                .strip()
                .lower()
                .replace(" ", "_")
            )

            object_id = (
                f"{normalized_label}_{object_index}"
            )

            detections.append(
                DetectedObject(
                    object_id=object_id,
                    label=detector_label,
                    pixel_coordinates=pixel,
                    mask=object_mask,
                    confidence=confidence,
                )
            )

        return detections

    def _parse_base_to_table_transform(
        self,
        transform: dict[str, Any],
    ) -> tuple[np.ndarray, np.ndarray]:
        if not isinstance(transform, dict):
            raise TypeError(
                "base_to_table_transform must be a dictionary."
            )

        if "rotation" not in transform:
            raise ValueError(
                "base_to_table_transform is missing 'rotation'."
            )

        if "translation" not in transform:
            raise ValueError(
                "base_to_table_transform is missing 'translation'."
            )

        rotation = np.asarray(
            transform["rotation"],
            dtype=np.float64,
        )

        translation = np.asarray(
            transform["translation"],
            dtype=np.float64,
        )

        if rotation.shape != (3, 3):
            raise ValueError(
                "Transform rotation must have shape (3, 3)."
            )

        if translation.shape != (3,):
            raise ValueError(
                "Transform translation must have shape (3,)."
            )

        return rotation, translation

    def _build_raw_scene_overlay(
        self,
        rgb_image: np.ndarray,
        raw_scene: RawSceneState,
    ) -> np.ndarray:
        if not isinstance(rgb_image, np.ndarray):
            raise TypeError(
                "rgb_image must be a numpy array."
            )

        prompted = rgb_image.copy()

        for obj in raw_scene.objects:
            mask = obj.mask

            if mask is None:
                continue

            mask_bool = np.asarray(mask).astype(bool)

            if mask_bool.ndim != 2:
                raise ValueError(
                    f"Invalid mask shape for {obj.object_id}: "
                    f"{mask_bool.shape}"
                )

            if mask_bool.shape != prompted.shape[:2]:
                raise ValueError(
                    f"Mask shape {mask_bool.shape} does not match "
                    f"RGB shape {prompted.shape[:2]} "
                    f"for {obj.object_id}."
                )

            contours, _ = cv2.findContours(
                mask_bool.astype(np.uint8),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )

            cv2.drawContours(
                prompted,
                contours,
                -1,
                (255, 255, 255),
                2,
            )

            u, v = obj.pixel_coordinates

            cv2.circle(
                prompted,
                (int(u), int(v)),
                4,
                (255, 255, 255),
                -1,
            )

            label = obj.object_id

            cv2.putText(
                prompted,
                label,
                (
                    int(u) + 6,
                    int(v) - 6,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        return prompted

    def run(
        self,
        rgb_image: np.ndarray,
        depth_image: np.ndarray,
        camera_info: dict[str, Any],
        base_to_table_transform: dict[str, Any],
        artifacts_dir: Path | None = None,
    ) -> ScenePerceptionResult:
        """Perceive the initial robot scene and return a frozen RawSceneState."""

        if not isinstance(rgb_image, np.ndarray):
            raise TypeError(
                "rgb_image must be a numpy array."
            )

        if rgb_image.ndim != 3:
            raise ValueError(
                "rgb_image must have shape (H, W, C)."
            )

        if not isinstance(depth_image, np.ndarray):
            raise TypeError(
                "depth_image must be a numpy array."
            )

        if depth_image.ndim != 2:
            raise ValueError(
                "depth_image must have shape (H, W)."
            )

        if rgb_image.shape[:2] != depth_image.shape:
            raise ValueError(
                "RGB and depth images must have the same spatial resolution."
            )

        camera_matrix = self.camera_matrix_from_info(
            camera_info
        )

        rotation, translation = (
            self._parse_base_to_table_transform(
                base_to_table_transform
            )
        )

        detections = self._detect_objects(
            rgb_image
        )

        if not detections:
            raise RuntimeError(
                "No objects were detected in the runtime scene."
            )

        raw_objects: list[RawSceneObject] = []

        with TIMING.measure(
            "scene.geometry"
        ):

            for detection in detections:

                object_id = detection.object_id
                label = detection.label
                pixel = detection.pixel_coordinates

                position_base = self.pixel_to_base_link(
                    pixel=pixel,
                    depth_image=depth_image,
                    camera_matrix=camera_matrix,
                    base_to_table_rotation=rotation,
                    base_to_table_translation=translation,
                )

                depth = robust_depth_at(
                    depth_image,
                    pixel[0],
                    pixel[1],
                )

                if depth is None:
                    raise RuntimeError(
                        f"No valid depth for object '{object_id}' "
                        f"at pixel {pixel}."
                    )

                position_camera_np = deproject_pixel(
                    pixel[0],
                    pixel[1],
                    depth,
                    camera_matrix,
                )

                position_camera = tuple(
                    float(value)
                    for value in position_camera_np
                )

                raw_objects.append(
                RawSceneObject(
                    object_id=object_id,
                    label=label,
                    pixel_coordinates=pixel,
                    position_camera=position_camera,
                    position_base=position_base,
                    mask=detection.mask,
                    confidence=detection.confidence,
                )
            )

        raw_scene = RawSceneState(
            objects=tuple(raw_objects)
        )

        overlay_image_path = None
        raw_scene_json_path = None

        if artifacts_dir is not None:

            with TIMING.measure(
                "io.scene_perception_artifacts"
            ):

                artifacts_dir = (
                    Path(artifacts_dir)
                    .expanduser()
                    .resolve()
                )

                artifacts_dir.mkdir(
                    parents=True,
                    exist_ok=True,
                )

                overlay = self._build_raw_scene_overlay(
                    rgb_image=rgb_image,
                    raw_scene=raw_scene,
                )

                overlay_image_path = (
                    artifacts_dir
                    / "raw_scene_overlay.png"
                )

                saved = cv2.imwrite(
                    str(overlay_image_path),
                    cv2.cvtColor(
                        overlay,
                        cv2.COLOR_RGB2BGR,
                    ),
                )

                if not saved:
                    raise RuntimeError(
                        f"Could not save raw scene overlay: "
                        f"{overlay_image_path}"
                    )

                raw_scene_json_path = (
                    artifacts_dir
                    / "raw_scene_state.json"
                )

                with raw_scene_json_path.open(
                    "w",
                    encoding="utf-8",
                ) as stream:
                    json.dump(
                        {
                            "objects": [
                                {
                                    "object_id": obj.object_id,
                                    "label": obj.label,
                                    "pixel_coordinates": list(
                                        obj.pixel_coordinates
                                    ),
                                    "confidence": obj.confidence,
                                    "position_camera": list(
                                        obj.position_camera
                                    ),
                                    "position_base": list(
                                        obj.position_base
                                    ),
                                }
                                for obj in raw_scene.objects
                            ]
                        },
                        stream,
                        indent=2,
                        ensure_ascii=False,
                    )

        return ScenePerceptionResult(
            raw_scene=raw_scene,
            overlay_image_path=overlay_image_path,
            raw_scene_json_path=raw_scene_json_path,
        )