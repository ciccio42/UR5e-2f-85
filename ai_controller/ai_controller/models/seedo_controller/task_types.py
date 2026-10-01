from enum import Enum


class TaskType(str, Enum):
    PICK_AND_PLACE = "pick_and_place"
    NUT_ASSEMBLY = "nut_assembly"


UNKNOWN_TASK_TYPE = "unknown"


def supported_task_values() -> tuple[str, ...]:
    return tuple(task.value for task in TaskType)


def all_task_values() -> tuple[str, ...]:
    return supported_task_values() + (UNKNOWN_TASK_TYPE,)