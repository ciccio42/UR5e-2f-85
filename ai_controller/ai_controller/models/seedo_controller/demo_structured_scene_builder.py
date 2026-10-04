from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path

from results import (
    StructuredScene,
    StructuredSceneObject,
    VisualPromptingResult,
)

from ai_controller.models.seedo_controller.task_types import (
    all_place_categories,
)

from ai_controller.models.seedo_controller.utils import (
    DirectionMode,
    build_spatial_relations,
)


class DemoStructuredSceneBuilder:
    """Build the structural representation of the demonstration scene."""

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
        visual_prompting_result: VisualPromptingResult,
        artifacts_dir: Path | None = None,
    ) -> StructuredScene:
        """Build the structured demonstration scene from SAM centroids."""

        if not visual_prompting_result.track_id_map:
            raise ValueError(
                "VisualPromptingResult contains an empty track_id_map."
            )

        objects: list[StructuredSceneObject] = []

        for track_id, track_info in (
            visual_prompting_result.track_id_map.items()
        ):
            category = str(
                track_info.get("category", "")
            ).strip().lower()

            if not category:
                raise ValueError(
                    "Missing category for demonstration track "
                    f"{track_id}."
                )

            if category not in self.place_categories:
                continue

            center = track_info.get("initial_center")

            if center is None:
                raise ValueError(
                    "Missing SAM centroid for demonstration track "
                    f"{track_id}."
                )

            if (
                not isinstance(center, (list, tuple))
                or len(center) != 2
            ):
                raise ValueError(
                    "Invalid SAM centroid for demonstration track "
                    f"{track_id}: {center!r}"
                )

            center_x, center_y = (
                float(value)
                for value in center
            )

            objects.append(
                StructuredSceneObject(
                    object_id=str(track_id),
                    category=category,
                    center=(
                        center_x,
                        center_y,
                    ),
                )
            )

        if not objects:
            raise ValueError(
                "No possible place destinations were found "
                "in the demonstration scene."
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
                / "demo_structured_scene.json"
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