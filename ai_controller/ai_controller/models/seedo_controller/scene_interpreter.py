from __future__ import annotations

import numpy as np
import base64
import json
import os
import cv2

from typing import Any
from openai import OpenAI
from pathlib import Path
from collections import Counter
from results import (
    RawSceneState,
    ScenePerceptionResult,
    SceneObject,
    SceneState,
)
from ai_controller.models.seedo_controller.scene_interpreter_prompts import (
    GENERALIZED_SCENE_INTERPRETER_SYSTEM_PROMPT,
    PRIOR_GUIDED_SCENE_INTERPRETER_SYSTEM_PROMPT,
)

SCENE_INTERPRETATION_SCHEMA: dict[str, Any] = {
    "name": "runtime_scene_interpretation",
    "strict": True,
    "schema": {
        "type": "object",
        "properties": {
            "objects": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "raw_object_id": {
                            "type": "string",
                        },
                        "semantic_name": {
                            "type": "string",
                        },
                    },
                    "required": [
                        "raw_object_id",
                        "semantic_name",
                    ],
                    "additionalProperties": False,
                },
            },
        },
        "required": [
            "objects",
        ],
        "additionalProperties": False,
    },
}

class SceneInterpreter:
    """Assign task-level semantics to a geometrically perceived scene."""

    def __init__(
        self,
        model: str = "gpt-4o-2024-08-06",
        perception_mode: str = "generalized",
    ) -> None:
        self.model = model

        self.perception_mode = str(
            perception_mode
        ).strip().lower()

        if self.perception_mode not in {
            "generalized",
            "prior_guided",
        }:
            raise ValueError(
                "Invalid perception_mode: "
                f"{self.perception_mode!r}"
            )

    def run(
        self,
        perception_result: ScenePerceptionResult,
        artifacts_dir: Path | None = None,
    ) -> SceneState:

        raw_scene = perception_result.raw_scene

        if artifacts_dir is not None:
            artifacts_dir = (
                Path(artifacts_dir)
                .expanduser()
                .resolve()
            )

            artifacts_dir.mkdir(
                parents=True,
                exist_ok=True,
            )

        if not raw_scene.objects:
            raise ValueError(
                "Raw scene contains no objects."
            )

        if perception_result.overlay_image_path is None:
            raise ValueError(
                "ScenePerceptionResult does not contain "
                "a raw scene overlay."
            )

        if not perception_result.overlay_image_path.is_file():
            raise FileNotFoundError(
                "Raw scene overlay does not exist: "
                f"{perception_result.overlay_image_path}"
            )

        if self.perception_mode == "generalized":

            semantic_names = (
                self._assign_generalized_semantic_names(
                    raw_scene
                )
            )

            # Preserve the existing artifact format.
            if artifacts_dir is not None:

                interpretation_path = (
                    artifacts_dir
                    / "scene_interpretation.json"
                )

                interpretation_result = {
                    "objects": [
                        {
                            "raw_object_id": obj.object_id,
                            "semantic_name": semantic_names[
                                obj.object_id
                            ],
                        }
                        for obj in raw_scene.objects
                    ]
                }

                with interpretation_path.open(
                    "w",
                    encoding="utf-8",
                ) as stream:

                    json.dump(
                        interpretation_result,
                        stream,
                        indent=2,
                        ensure_ascii=False,
                    )

        else:

            semantic_names = self._assign_semantic_names(
                raw_scene=raw_scene,
                overlay_image_path=(
                    perception_result.overlay_image_path
                ),
                artifacts_dir=artifacts_dir,
            )

        if set(semantic_names.keys()) != {
            obj.object_id
            for obj in raw_scene.objects
        }:
            raise RuntimeError(
                "SceneInterpreter did not assign exactly one semantic "
                "name to every perceived object."
            )

        scene_objects: list[SceneObject] = []

        for raw_object in raw_scene.objects:
            semantic_name = semantic_names[
                raw_object.object_id
            ]

            scene_objects.append(
                SceneObject(
                    object_id=semantic_name,
                    label=raw_object.label,

                    pixel_coordinates=(
                        raw_object.pixel_coordinates
                    ),

                    position_camera=(
                        raw_object.position_camera
                    ),

                    position_base=(
                        raw_object.position_base
                    ),

                    category=(
                        raw_object.category
                        if self.perception_mode == "generalized"
                        else None
                    ),

                    attributes=(
                        dict(raw_object.attributes)
                        if self.perception_mode == "generalized"
                        else {}
                    ),
                )
            )

        scene_state = SceneState(
            objects=tuple(scene_objects)
        )

        if artifacts_dir is not None:
            state_path = (
                artifacts_dir
                / "scene_state.json"
            )

            with state_path.open(
                "w",
                encoding="utf-8",
            ) as stream:
                json.dump(
                    {
                        "objects": [
                            {
                                "object_id": obj.object_id,
                                "label": obj.label,
                                **(
                                    {
                                        "category": obj.category,
                                        "attributes": obj.attributes,
                                    }
                                    if self.perception_mode == "generalized"
                                    else {}
                                ),
                                "pixel_coordinates": list(
                                    obj.pixel_coordinates
                                ),
                                "position_camera": list(
                                    obj.position_camera
                                ),
                                "position_base": list(
                                    obj.position_base
                                ),
                            }
                            for obj in scene_state.objects
                        ]
                    },
                    stream,
                    indent=2,
                    ensure_ascii=False,
                )

        return scene_state

    @staticmethod
    def _ordinal(index: int) -> str:
        """Convert a positive integer to an English ordinal."""

        ordinal_words = {
            1: "first",
            2: "second",
            3: "third",
            4: "fourth",
            5: "fifth",
            6: "sixth",
            7: "seventh",
            8: "eighth",
            9: "ninth",
            10: "tenth",
        }

        if index in ordinal_words:
            return ordinal_words[index]

        if 10 <= index % 100 <= 20:
            suffix = "th"
        else:
            suffix = {
                1: "st",
                2: "nd",
                3: "rd",
            }.get(index % 10, "th")

        return f"{index}{suffix}"

    def _assign_generalized_semantic_names(
        self,
        raw_scene: RawSceneState,
    ) -> dict[str, str]:
        """
        Generate unique semantic identifiers using the
        authoritative object-discovery metadata.

        No additional VLM classification is performed.
        """

        label_counts = Counter(
            obj.label
            for obj in raw_scene.objects
        )

        objects_by_label = {}

        for obj in raw_scene.objects:

            if not obj.category:
                raise RuntimeError(
                    "Missing category for generalized object: "
                    f"{obj.object_id!r}"
                )

            if not isinstance(obj.attributes, dict):
                raise RuntimeError(
                    "Invalid attributes for generalized object: "
                    f"{obj.object_id!r}"
                )

            objects_by_label.setdefault(
                obj.label,
                [],
            ).append(obj)

        semantic_names = {}

        for label, objects in objects_by_label.items():

            # A unique detector label is already an
            # unambiguous semantic identifier.
            if label_counts[label] == 1:

                obj = objects[0]

                semantic_names[obj.object_id] = label

                continue

            # Repeated instances require distinct identifiers.
            # Ordinals are based on image coordinates,
            # never on detection order or raw object IDs.
            ordered_objects = sorted(
                objects,
                key=lambda obj: (
                    obj.pixel_coordinates[0],
                    obj.pixel_coordinates[1],
                    obj.object_id,
                ),
            )

            for index, obj in enumerate(
                ordered_objects,
                start=1,
            ):

                semantic_names[obj.object_id] = (
                    f"{self._ordinal(index)} "
                    f"{label} from the left"
                )

        if len(semantic_names) != len(raw_scene.objects):
            raise RuntimeError(
                "Generalized scene interpretation lost objects."
            )

        if len(set(semantic_names.values())) != len(
            semantic_names
        ):
            raise RuntimeError(
                "Generalized scene interpretation produced "
                "duplicate semantic names."
            )

        return semantic_names

    def _assign_semantic_names(
        self,
        raw_scene: RawSceneState,
        overlay_image_path: Path,
        artifacts_dir: Path | None = None,
    ) -> dict[str, str]:
        if not os.environ.get("OPENAI_API_KEY"):
            raise ValueError(
                "OPENAI_API_KEY is not configured."
            )

        prompted_bgr = cv2.imread(
            str(overlay_image_path)
        )

        if prompted_bgr is None:
            raise RuntimeError(
                "Could not read raw scene overlay: "
                f"{overlay_image_path}"
            )

        prompted_image = cv2.cvtColor(
            prompted_bgr,
            cv2.COLOR_BGR2RGB,
        )

        ok, encoded_image = cv2.imencode(
            ".jpg",
            cv2.cvtColor(
                prompted_image,
                cv2.COLOR_RGB2BGR,
            ),
            [
                cv2.IMWRITE_JPEG_QUALITY,
                95,
            ],
        )

        if not ok:
            raise RuntimeError(
                "Could not encode the prompted runtime image."
            )

        image_base64 = base64.b64encode(
            encoded_image.tobytes()
        ).decode("ascii")

        raw_objects = [
            {
                "raw_object_id": obj.object_id,
                "detector_label": obj.label,
                "pixel_coordinates": [
                    int(obj.pixel_coordinates[0]),
                    int(obj.pixel_coordinates[1]),
                ],
                "confidence": obj.confidence,
            }
            for obj in raw_scene.objects
        ]

        scene_context = {
            "perceived_objects": raw_objects,
        }

        # ---------------------------------------------------------
        # Select scene-interpreter prompt
        # ---------------------------------------------------------

        if self.perception_mode == "generalized":
            system_prompt = (
                GENERALIZED_SCENE_INTERPRETER_SYSTEM_PROMPT
            )
        else:
            system_prompt = (
                PRIOR_GUIDED_SCENE_INTERPRETER_SYSTEM_PROMPT
            )

        print(
            "[SceneInterpreter] Perception mode: "
            f"{self.perception_mode}"
        )

        messages = [
            {
                "role": "system",
                "content": system_prompt,
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": (
                            "Assign a semantic name to each perceived object.\n\n"
                            "The runtime scene image already contains the raw object "
                            "annotations generated by the perception system.\n\n"
                            "Perceived objects:\n"
                            + json.dumps(
                                scene_context,
                                indent=2,
                                sort_keys=True,
                            )
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
            model=self.model,
            messages=messages,
            temperature=0,
            max_tokens=800,
            response_format={
                "type": "json_schema",
                "json_schema": (
                    SCENE_INTERPRETATION_SCHEMA
                ),
            },
        )

        raw_output = (
            response
            .choices[0]
            .message
            .content
        )

        if not raw_output:
            raise RuntimeError(
                "The scene-interpreter VLM returned no output."
            )

        result = json.loads(
            raw_output
        )

        if artifacts_dir is not None:
            interpretation_path = (
                artifacts_dir
                / "scene_interpretation.json"
            )

            with interpretation_path.open(
                "w",
                encoding="utf-8",
            ) as stream:
                json.dump(
                    result,
                    stream,
                    indent=2,
                    ensure_ascii=False,
                )

        semantic_names = {
            item["raw_object_id"]: item["semantic_name"]
            for item in result["objects"]
        }

        expected_ids = {
            obj.object_id
            for obj in raw_scene.objects
        }

        returned_ids = set(
            semantic_names.keys()
        )

        if returned_ids != expected_ids:
            missing = expected_ids - returned_ids
            unexpected = returned_ids - expected_ids

            raise RuntimeError(
                "Scene interpretation returned an invalid object mapping. "
                f"Missing={sorted(missing)}, "
                f"unexpected={sorted(unexpected)}."
            )

        # Detector labels are authoritative for non-bin objects.
        for raw_object in raw_scene.objects:

            detector_label = (
                raw_object.label
                .strip()
                .lower()
            )

            if detector_label != "storage bin":

                returned_name = (
                    semantic_names[
                        raw_object.object_id
                    ]
                    .strip()
                    .lower()
                )

                if returned_name != detector_label:
                    raise RuntimeError(
                        "Scene interpretation changed an authoritative "
                        "semantic detector label: "
                        f"raw_object_id={raw_object.object_id!r}, "
                        f"detector_label={detector_label!r}, "
                        f"semantic_name={returned_name!r}"
                    )

        expected_ids = {
            obj.object_id
            for obj in raw_scene.objects
        }

        returned_ids = set(
            semantic_names.keys()
        )

        if returned_ids != expected_ids:
            missing = expected_ids - returned_ids
            unexpected = returned_ids - expected_ids

            raise RuntimeError(
                "Scene interpretation returned an invalid object mapping. "
                f"Missing={sorted(missing)}, "
                f"unexpected={sorted(unexpected)}."
            )

        if len(set(semantic_names.values())) != len(
            semantic_names
        ):
            raise RuntimeError(
                "Scene interpretation produced duplicate semantic names."
            )

        return semantic_names

