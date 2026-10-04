from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path

from results import (
    ActionPlanningResult,
    ReplicabilityResult,
    ResolvedActionTargets,
    SceneState,
    StructuralMatchingResult,
)

from ai_controller.models.seedo_controller.task_types import (
    TaskType,
    place_categories_for_task,
)


class ReplicabilityChecker:
    """Check whether a demonstrated task can be reproduced at runtime."""

    def run(
        self,
        action_plan: ActionPlanningResult,
        structural_matching_result: StructuralMatchingResult,
        scene_state: SceneState,
        artifacts_dir: Path | None = None,
    ) -> ReplicabilityResult:
        """Resolve runtime targets and verify task replicability."""

        failure_reasons: list[str] = []

        try:
            task_type = TaskType(
                action_plan.task_type
            )
        except (TypeError, ValueError):
            return self._save_and_return(
                ReplicabilityResult(
                    replicable=False,
                    resolved_targets=(),
                    failure_reasons=(
                        "The action plan does not contain a supported task type.",
                    ),
                ),
                artifacts_dir,
            )

        if not action_plan.steps:
            return self._save_and_return(
                ReplicabilityResult(
                    replicable=False,
                    resolved_targets=(),
                    failure_reasons=(
                        "The action plan contains no action steps.",
                    ),
                ),
                artifacts_dir,
            )

        if not structural_matching_result.valid_mappings:
            return self._save_and_return(
                ReplicabilityResult(
                    replicable=False,
                    resolved_targets=(),
                    failure_reasons=(
                        "No valid structural mapping exists between "
                        "the demonstration and runtime scenes.",
                    ),
                ),
                artifacts_dir,
            )

        allowed_place_categories = place_categories_for_task(
            task_type
        )

        runtime_objects_by_id = {
            obj.object_id: obj
            for obj in scene_state.objects
        }

        resolved_targets: list[ResolvedActionTargets] = []

        for step_index, step in enumerate(action_plan.steps):
            # ------------------------------------------------------
            # Resolve PICK target.
            # ------------------------------------------------------

            picked_label = self._normalize(
                step.picked_detector_label
            )

            if not picked_label:
                failure_reasons.append(
                    f"Action step {step_index}: "
                    "missing picked_detector_label."
                )
                continue

            pick_candidates = [
                obj
                for obj in scene_state.objects
                if self._normalize(obj.label) == picked_label
            ]

            if len(pick_candidates) != 1:
                failure_reasons.append(
                    f"Action step {step_index}: cannot uniquely "
                    "resolve picked object with detector label "
                    f"{picked_label!r}; matches="
                    f"{[obj.object_id for obj in pick_candidates]}."
                )
                continue

            picked_object = pick_candidates[0]

            # ------------------------------------------------------
            # Resolve PLACE target through structural matching.
            # ------------------------------------------------------

            demo_destination_id = str(
                step.destination_track_id
            )

            runtime_destination_ids: set[str] = set()

            for mapping in (
                structural_matching_result.valid_mappings
            ):
                mapped_destination = self._mapped_object_id(
                    mapping=mapping,
                    demo_object_id=demo_destination_id,
                )

                if mapped_destination is None:
                    failure_reasons.append(
                        f"Action step {step_index}: demonstration "
                        f"destination {demo_destination_id!r} is not "
                        "present in a structural mapping."
                    )
                    break

                runtime_destination_ids.add(
                    mapped_destination
                )

            else:
                # This block is reached only if every valid mapping
                # contains the demonstrated destination.

                if len(runtime_destination_ids) != 1:
                    failure_reasons.append(
                        f"Action step {step_index}: structural "
                        "matching does not uniquely determine the "
                        "runtime place destination; candidates="
                        f"{sorted(runtime_destination_ids)}."
                    )
                    continue

                runtime_destination_id = next(
                    iter(runtime_destination_ids)
                )

                destination_object = runtime_objects_by_id.get(
                    runtime_destination_id
                )

                if destination_object is None:
                    failure_reasons.append(
                        f"Action step {step_index}: mapped runtime "
                        f"destination {runtime_destination_id!r} "
                        "does not exist in SceneState."
                    )
                    continue

                destination_category = self._normalize(
                    destination_object.category
                )

                if (
                    destination_category
                    not in allowed_place_categories
                ):
                    failure_reasons.append(
                        f"Action step {step_index}: mapped runtime "
                        f"destination {runtime_destination_id!r} has "
                        f"category {destination_category!r}, which "
                        f"is not valid for task "
                        f"{task_type.value!r}."
                    )
                    continue

                if (
                    picked_object.object_id
                    == destination_object.object_id
                ):
                    failure_reasons.append(
                        f"Action step {step_index}: picked object "
                        "and place destination resolve to the same "
                        f"runtime object "
                        f"{picked_object.object_id!r}."
                    )
                    continue

                resolved_targets.append(
                    ResolvedActionTargets(
                        action_step_index=step_index,
                        runtime_pick_object_id=(
                            picked_object.object_id
                        ),
                        runtime_place_object_id=(
                            destination_object.object_id
                        ),
                    )
                )

        if failure_reasons:
            return self._save_and_return(
                ReplicabilityResult(
                    replicable=False,
                    resolved_targets=(),
                    failure_reasons=tuple(
                        failure_reasons
                    ),
                ),
                artifacts_dir,
            )

        if len(resolved_targets) != len(action_plan.steps):
            return self._save_and_return(
                ReplicabilityResult(
                    replicable=False,
                    resolved_targets=(),
                    failure_reasons=(
                        "Not all action steps could be resolved "
                        "to runtime targets.",
                    ),
                ),
                artifacts_dir,
            )

        return self._save_and_return(
            ReplicabilityResult(
                replicable=True,
                resolved_targets=tuple(
                    resolved_targets
                ),
                failure_reasons=(),
            ),
            artifacts_dir,
        )

    @staticmethod
    def _normalize(
        value: object,
    ) -> str:
        """Normalize semantic labels and categories."""
        return str(
            value if value is not None else ""
        ).strip().casefold()

    @staticmethod
    def _mapped_object_id(
        mapping,
        demo_object_id: str,
    ) -> str | None:
        """Return the runtime object mapped from one demo object."""

        for match in mapping.matches:
            if match.demo_object_id == demo_object_id:
                return match.runtime_object_id

        return None

    @staticmethod
    def _save_and_return(
        result: ReplicabilityResult,
        artifacts_dir: Path | None,
    ) -> ReplicabilityResult:
        """Persist the replicability result and return it unchanged."""

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

            result_path = (
                artifacts_dir
                / "replicability_result.json"
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

        return result