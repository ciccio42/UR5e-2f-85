from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path

from itertools import permutations

from results import (
    StructuredScene,
    StructuralMapping,
    StructuralMatchingResult,
    StructuralObjectMatch,
)


class StructuralMatcher:
    """Find structure-preserving bijections between two structured scenes."""

    def run(
        self,
        demo_scene: StructuredScene,
        runtime_scene: StructuredScene,
        artifacts_dir: Path | None = None,
    ) -> StructuralMatchingResult:
        """Find all bijections preserving the qualitative spatial structure."""

        if demo_scene.directions != runtime_scene.directions:
            raise ValueError(
                "Demo and runtime structured scenes use different "
                "direction modes: "
                f"{demo_scene.directions} vs "
                f"{runtime_scene.directions}."
            )

        demo_objects = demo_scene.objects
        runtime_objects = runtime_scene.objects

        # A bijection can exist only if the two sets have the same size.
        if len(demo_objects) != len(runtime_objects):
            result = StructuralMatchingResult(
                valid_mappings=()
            )

            self._save_result(
                result=result,
                artifacts_dir=artifacts_dir,
            )

            return result

        demo_relations = self._build_relation_lookup(
            demo_scene
        )

        runtime_relations = self._build_relation_lookup(
            runtime_scene
        )

        valid_mappings: list[StructuralMapping] = []

        for runtime_permutation in permutations(
            runtime_objects
        ):
            mapping = {
                demo_object.object_id:
                    runtime_object.object_id
                for demo_object, runtime_object in zip(
                    demo_objects,
                    runtime_permutation,
                )
            }

            if not self._preserves_structure(
                demo_scene=demo_scene,
                demo_relations=demo_relations,
                runtime_relations=runtime_relations,
                mapping=mapping,
            ):
                continue

            valid_mappings.append(
                StructuralMapping(
                    matches=tuple(
                        StructuralObjectMatch(
                            demo_object_id=demo_object.object_id,
                            runtime_object_id=mapping[
                                demo_object.object_id
                            ],
                        )
                        for demo_object in demo_objects
                    )
                )
            )

        result = StructuralMatchingResult(
            valid_mappings=tuple(valid_mappings)
        )

        self._save_result(
            result=result,
            artifacts_dir=artifacts_dir,
        )

        return result

    @staticmethod
    def _build_relation_lookup(
        scene: StructuredScene,
    ) -> dict[tuple[str, str], str]:
        """Index scene relations by directed object pair."""

        object_ids = {
            obj.object_id
            for obj in scene.objects
        }

        expected_relation_count = (
            len(scene.objects)
            * (len(scene.objects) - 1)
        )

        if len(scene.relations) != expected_relation_count:
            raise ValueError(
                "Structured scene does not contain the expected "
                "number of directed relations: "
                f"expected {expected_relation_count}, "
                f"received {len(scene.relations)}."
            )

        relation_lookup: dict[
            tuple[str, str],
            str,
        ] = {}

        for relation in scene.relations:
            subject_id = relation.subject_object_id
            reference_id = relation.reference_object_id

            if subject_id not in object_ids:
                raise ValueError(
                    "Structured relation references unknown "
                    f"subject object: {subject_id!r}."
                )

            if reference_id not in object_ids:
                raise ValueError(
                    "Structured relation references unknown "
                    f"reference object: {reference_id!r}."
                )

            if subject_id == reference_id:
                raise ValueError(
                    "Structured relations cannot reference the "
                    "same object as both subject and reference."
                )

            key = (
                subject_id,
                reference_id,
            )

            if key in relation_lookup:
                raise ValueError(
                    "Duplicate structured relation for pair: "
                    f"{key!r}."
                )

            relation_lookup[key] = relation.relation

        return relation_lookup

    @staticmethod
    def _preserves_structure(
        demo_scene: StructuredScene,
        demo_relations: dict[tuple[str, str], str],
        runtime_relations: dict[tuple[str, str], str],
        mapping: dict[str, str],
    ) -> bool:
        """Return whether one candidate bijection preserves all relations."""

        for subject in demo_scene.objects:
            for reference in demo_scene.objects:
                if subject.object_id == reference.object_id:
                    continue

                demo_key = (
                    subject.object_id,
                    reference.object_id,
                )

                runtime_key = (
                    mapping[subject.object_id],
                    mapping[reference.object_id],
                )

                if (
                    demo_relations[demo_key]
                    != runtime_relations[runtime_key]
                ):
                    return False

        return True

    @staticmethod
    def _save_result(
        result: StructuralMatchingResult,
        artifacts_dir: Path | None,
    ) -> None:
        """Persist the structural-matching result for inspection."""

        if artifacts_dir is None:
            return

        artifacts_dir = (
            Path(artifacts_dir)
            .expanduser()
            .resolve()
        )

        artifacts_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        result_path = (
            artifacts_dir
            / "structural_matching_result.json"
        )

        with result_path.open(
            "w",
            encoding="utf-8",
        ) as stream:
            json.dump(
                asdict(result),
                stream,
                indent=2,
                ensure_ascii=False,
            )