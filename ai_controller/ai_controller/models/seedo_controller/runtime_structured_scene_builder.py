from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path

from results import (
    SceneState,
    StructuredScene,
    StructuredSceneObject,
)

from ai_controller.models.seedo_controller.task_types import (
    all_place_categories,
)

from ai_controller.models.seedo_controller.utils import (
    DirectionMode,
    build_spatial_relations,
)


class RuntimeStructuredSceneBuilder:
    """Build the structural representation of the runtime scene."""

    def __init__(
        self,
        directions: DirectionMode = 8,
    ) -> None:
        if directions not in (4, 8):
            raise ValueError(
                f"Unsupported number of directions: {directions}. "
                "Expected 4 or 8."
            )

        self.directions = directions
        self.place_categories = all_place_categories()

    def run(
        self,
        scene_state: SceneState,
        artifacts_dir: Path | None = None,
    ) -> StructuredScene:
        """Build the structured runtime scene from SAM centroids."""

        if not scene_state.objects:
            raise ValueError(
                "SceneState contains no objects."
            )

        objects: list[StructuredSceneObject] = []

        for scene_object in scene_state.objects:
            category = str(
                scene_object.category or ""
            ).strip().lower()

            if not category:
                raise ValueError(
                    "Missing category for runtime object "
                    f"{scene_object.object_id!r}."
                )

            if category not in self.place_categories:
                continue

            center_x, center_y = scene_object.pixel_coordinates

            objects.append(
                StructuredSceneObject(
                    object_id=scene_object.object_id,
                    category=category,
                    center=(
                        float(center_x),
                        float(center_y),
                    ),
                )
            )

        if not objects:
            raise ValueError(
                "No possible place destinations were found "
                "in the runtime scene."
            )

        structured_objects = tuple(objects)

        relations = build_spatial_relations(
            objects=structured_objects,
            directions=self.directions,
        )

        structured_scene = StructuredScene(
            objects=structured_objects,
            relations=relations,
            directions=self.directions,
        )

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

            structured_scene_path = (
                artifacts_dir
                / "runtime_structured_scene.json"
            )

            with structured_scene_path.open(
                "w",
                encoding="utf-8",
            ) as stream:
                json.dump(
                    asdict(structured_scene),
                    stream,
                    indent=2,
                    ensure_ascii=False,
                )

        return structured_scene