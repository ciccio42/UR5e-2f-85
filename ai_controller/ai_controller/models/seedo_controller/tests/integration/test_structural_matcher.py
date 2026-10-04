from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

from results import (
    StructuredScene,
    StructuredSceneObject,
    StructuredSceneRelation,
)

from ai_controller.models.seedo_controller.structural_matcher import (
    StructuralMatcher,
)


DEFAULT_DEMO_STRUCTURED_SCENE = Path(
    "/seedo_tests/demo_structured_scene_builder/demo_structured_scene.json"
)

DEFAULT_RUNTIME_STRUCTURED_SCENE = Path(
    "/seedo_tests/runtime_structured_scene_builder/runtime_structured_scene.json"
)

EXPECTED_MAPPING = {
    "0": "storage_bin_0",
    "1": "storage_bin_1",
    "2": "storage_bin_2",
    "3": "storage_bin_3",
}


def _load_structured_scene(
    path: Path,
) -> StructuredScene:
    """Reconstruct a StructuredScene from a persisted integration artifact."""

    normalized_path = (
        Path(path)
        .expanduser()
        .resolve()
    )

    if not normalized_path.is_file():
        raise FileNotFoundError(
            "Structured-scene artifact does not exist: "
            f"{normalized_path}"
        )

    if normalized_path.stat().st_size == 0:
        raise ValueError(
            "Structured-scene artifact is empty: "
            f"{normalized_path}"
        )

    with normalized_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        data = json.load(
            stream
        )

    objects_data = data.get(
        "objects"
    )

    relations_data = data.get(
        "relations"
    )

    directions = data.get(
        "directions"
    )

    if not isinstance(
        objects_data,
        list,
    ) or not objects_data:
        raise ValueError(
            f"{normalized_path} does not contain a non-empty objects list."
        )

    if not isinstance(
        relations_data,
        list,
    ):
        raise ValueError(
            f"{normalized_path} does not contain a relations list."
        )

    if directions not in (
        4,
        8,
    ):
        raise ValueError(
            f"{normalized_path} contains an invalid direction mode: "
            f"{directions!r}"
        )

    objects: list[
        StructuredSceneObject
    ] = []

    for index, item in enumerate(
        objects_data
    ):
        if not isinstance(
            item,
            dict,
        ):
            raise ValueError(
                "Invalid structured object at index "
                f"{index}: {item!r}"
            )

        object_id = str(
            item.get(
                "object_id",
                "",
            )
        ).strip()

        category = str(
            item.get(
                "category",
                "",
            )
        ).strip()

        center = item.get(
            "center"
        )

        if not object_id:
            raise ValueError(
                f"Structured object at index {index} has no object_id."
            )

        if not category:
            raise ValueError(
                f"Structured object {object_id!r} has no category."
            )

        if (
            not isinstance(
                center,
                (list, tuple),
            )
            or len(center) != 2
        ):
            raise ValueError(
                f"Structured object {object_id!r} has invalid center: "
                f"{center!r}"
            )

        objects.append(
            StructuredSceneObject(
                object_id=object_id,
                category=category,
                center=(
                    float(center[0]),
                    float(center[1]),
                ),
            )
        )

    relations: list[
        StructuredSceneRelation
    ] = []

    for index, item in enumerate(
        relations_data
    ):
        if not isinstance(
            item,
            dict,
        ):
            raise ValueError(
                "Invalid structured relation at index "
                f"{index}: {item!r}"
            )

        subject_object_id = str(
            item.get(
                "subject_object_id",
                "",
            )
        ).strip()

        reference_object_id = str(
            item.get(
                "reference_object_id",
                "",
            )
        ).strip()

        relation = str(
            item.get(
                "relation",
                "",
            )
        ).strip()

        if not subject_object_id:
            raise ValueError(
                f"Structured relation at index {index} has no subject ID."
            )

        if not reference_object_id:
            raise ValueError(
                f"Structured relation at index {index} has no reference ID."
            )

        if not relation:
            raise ValueError(
                f"Structured relation at index {index} has no relation label."
            )

        relations.append(
            StructuredSceneRelation(
                subject_object_id=(
                    subject_object_id
                ),
                reference_object_id=(
                    reference_object_id
                ),
                relation=relation,
            )
        )

    return StructuredScene(
        objects=tuple(
            objects
        ),
        relations=tuple(
            relations
        ),
        directions=int(
            directions
        ),
    )


def run_structural_matcher_test(
    args: argparse.Namespace,
) -> int:
    """Match the real demonstration and runtime structured-scene artifacts."""

    demo_scene_path = (
        DEFAULT_DEMO_STRUCTURED_SCENE
    )

    runtime_scene_path = (
        DEFAULT_RUNTIME_STRUCTURED_SCENE
    )

    demo_scene = _load_structured_scene(
        demo_scene_path
    )

    runtime_scene = _load_structured_scene(
        runtime_scene_path
    )

    if args.artifacts_dir is None:
        artifacts_dir = Path(
            "/seedo_tests/structural_matcher"
        )
    else:
        artifacts_dir = (
            Path(args.artifacts_dir)
            .expanduser()
            .resolve()
        )

    # ---------------------------------------------------------
    # Validate the two handoff scenes before matching.
    # ---------------------------------------------------------

    if demo_scene.directions != 8:
        raise AssertionError(
            "Unexpected demonstration direction mode: "
            f"{demo_scene.directions}"
        )

    if runtime_scene.directions != 8:
        raise AssertionError(
            "Unexpected runtime direction mode: "
            f"{runtime_scene.directions}"
        )

    if len(
        demo_scene.objects
    ) != 4:
        raise AssertionError(
            "Expected four demonstration place objects, "
            f"received {len(demo_scene.objects)}."
        )

    if len(
        runtime_scene.objects
    ) != 4:
        raise AssertionError(
            "Expected four runtime place objects, "
            f"received {len(runtime_scene.objects)}."
        )

    if len(
        demo_scene.relations
    ) != 12:
        raise AssertionError(
            "Expected twelve demonstration relations, "
            f"received {len(demo_scene.relations)}."
        )

    if len(
        runtime_scene.relations
    ) != 12:
        raise AssertionError(
            "Expected twelve runtime relations, "
            f"received {len(runtime_scene.relations)}."
        )

    demo_before = asdict(
        demo_scene
    )

    runtime_before = asdict(
        runtime_scene
    )

    # ---------------------------------------------------------
    # Run real structural matching.
    # ---------------------------------------------------------

    matcher = StructuralMatcher()

    result = matcher.run(
        demo_scene=demo_scene,
        runtime_scene=runtime_scene,
        artifacts_dir=artifacts_dir,
    )

    # ---------------------------------------------------------
    # Validate the result.
    # ---------------------------------------------------------

    if not result.is_valid:
        raise AssertionError(
            "StructuralMatcher found no valid mapping between "
            "the demonstration and runtime scenes."
        )

    if not result.is_unique:
        raise AssertionError(
            "Expected a unique structure-preserving mapping, "
            f"received {len(result.valid_mappings)} valid mappings."
        )

    if len(
        result.valid_mappings
    ) != 1:
        raise AssertionError(
            "Expected exactly one valid structural mapping, "
            f"received {len(result.valid_mappings)}."
        )

    mapping = (
        result.valid_mappings[0]
    )

    if len(
        mapping.matches
    ) != 4:
        raise AssertionError(
            "Expected four object matches in the unique mapping, "
            f"received {len(mapping.matches)}."
        )

    returned_mapping = {
        match.demo_object_id:
            match.runtime_object_id
        for match
        in mapping.matches
    }

    if returned_mapping != EXPECTED_MAPPING:
        raise AssertionError(
            "Unexpected structural mapping: "
            f"expected={EXPECTED_MAPPING}, "
            f"received={returned_mapping}"
        )

    if len(
        returned_mapping
    ) != len(
        set(
            returned_mapping.values()
        )
    ):
        raise AssertionError(
            "Structural mapping is not bijective."
        )

    # ---------------------------------------------------------
    # Explicitly verify that every demo relation is preserved
    # by the unique mapping.
    # ---------------------------------------------------------

    demo_relation_lookup = {
        (
            relation.subject_object_id,
            relation.reference_object_id,
        ): relation.relation
        for relation
        in demo_scene.relations
    }

    runtime_relation_lookup = {
        (
            relation.subject_object_id,
            relation.reference_object_id,
        ): relation.relation
        for relation
        in runtime_scene.relations
    }

    for (
        (
            demo_subject,
            demo_reference,
        ),
        demo_relation,
    ) in demo_relation_lookup.items():
        runtime_subject = (
            returned_mapping[
                demo_subject
            ]
        )

        runtime_reference = (
            returned_mapping[
                demo_reference
            ]
        )

        runtime_relation = (
            runtime_relation_lookup[
                (
                    runtime_subject,
                    runtime_reference,
                )
            ]
        )

        if (
            runtime_relation
            != demo_relation
        ):
            raise AssertionError(
                "Unique mapping does not preserve relation: "
                f"demo=({demo_subject}, {demo_relation}, "
                f"{demo_reference}), "
                f"runtime=({runtime_subject}, "
                f"{runtime_relation}, {runtime_reference})"
            )

    # ---------------------------------------------------------
    # The matcher must not mutate either input scene.
    # ---------------------------------------------------------

    if asdict(
        demo_scene
    ) != demo_before:
        raise AssertionError(
            "StructuralMatcher mutated the demonstration scene."
        )

    if asdict(
        runtime_scene
    ) != runtime_before:
        raise AssertionError(
            "StructuralMatcher mutated the runtime scene."
        )

    # ---------------------------------------------------------
    # Validate persistent artifact.
    # ---------------------------------------------------------

    artifact_path = (
        artifacts_dir
        / "structural_matching_result.json"
    )

    if not artifact_path.is_file():
        raise AssertionError(
            "StructuralMatcher artifact was not created: "
            f"{artifact_path}"
        )

    if artifact_path.stat().st_size == 0:
        raise AssertionError(
            "StructuralMatcher artifact is empty: "
            f"{artifact_path}"
        )

    with artifact_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        artifact = json.load(
            stream
        )

    expected_artifact = json.loads(
        json.dumps(
            asdict(
                result
            ),
            ensure_ascii=False,
        )
    )

    if artifact != expected_artifact:
        raise AssertionError(
            "structural_matching_result.json does not match "
            "the returned StructuralMatchingResult."
        )

    # ---------------------------------------------------------
    # Report.
    # ---------------------------------------------------------

    print(
        "Structural matching completed"
    )
    print(
        "Demo structured scene: "
        f"{demo_scene_path}"
    )
    print(
        "Runtime structured scene: "
        f"{runtime_scene_path}"
    )
    print(
        "Valid mappings: "
        f"{len(result.valid_mappings)}"
    )
    print(
        "Unique mapping: "
        f"{result.is_unique}"
    )

    print(
        "\nObject mapping:"
    )

    for match in (
        mapping.matches
    ):
        print(
            "  "
            f"{match.demo_object_id} "
            "-> "
            f"{match.runtime_object_id}"
        )

    print(
        "\nStructural matching artifact: "
        f"{artifact_path}"
    )

    print(
        "\nTEST PASSED"
    )

    return 0
