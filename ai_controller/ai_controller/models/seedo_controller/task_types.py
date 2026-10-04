from enum import Enum


class TaskType(str, Enum):
    PICK_AND_PLACE = "pick_and_place"
    NUT_ASSEMBLY = "nut_assembly"


UNKNOWN_TASK_TYPE = "unknown"

PLACE_CATEGORIES_BY_TASK: dict[TaskType, frozenset[str]] = {
    TaskType.PICK_AND_PLACE: frozenset({
        "bin",
    }),
    TaskType.NUT_ASSEMBLY: frozenset({
        "peg",
    }),
}


def supported_task_values() -> tuple[str, ...]:
    return tuple(task.value for task in TaskType)


def all_task_values() -> tuple[str, ...]:
    return supported_task_values() + (UNKNOWN_TASK_TYPE,)

def place_categories_for_task(
    task_type: TaskType,
) -> frozenset[str]:
    """Return the allowed place-destination categories for one task."""

    if task_type not in PLACE_CATEGORIES_BY_TASK:
        raise ValueError(
            f"No place categories configured for task: {task_type!r}"
        )

    return PLACE_CATEGORIES_BY_TASK[task_type]


def all_place_categories() -> frozenset[str]:
    """Return every category that can act as a place destination."""

    return frozenset(
        category
        for categories in PLACE_CATEGORIES_BY_TASK.values()
        for category in categories
    )