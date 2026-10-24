from ai_controller.utils.gui.backend.controllers import (
    get_controller,
    list_controllers,
)
from ai_controller.utils.gui.backend.environment import (
    detect_execution_mode,
)


TASKS = [
    {
        "id": "pick_place",
        "label": "Pick & Place",
        "demo_task": {
            "minimum": 0,
            "maximum": 15,
            "default": 0,
        },
        "isaac_scene": {
            "supported": True,
            "task": {
                "minimum": 0,
                "maximum": 15,
                "default": 0,
            },
            "trajectory": {
                "minimum": 0,
                "maximum": 39,
                "default": 0,
            },
        },
    },
    {
        "id": "nut_assembly",
        "label": "Nut Assembly",
        "demo_task": {
            "minimum": 0,
            "maximum": 8,
            "default": 0,
        },
        "isaac_scene": {
            "supported": True,
            "task": {
                "minimum": 0,
                "maximum": 8,
                "default": 0,
            },
            "trajectory": {
                "minimum": 0,
                "maximum": 0,
                "default": 0,
            },
        },
    },
]


EXECUTION_TYPES = [
    {
        "id": "full",
        "label": "Full Pipeline",
        "requires_precomputed_plan": False,
        "requires_precomputed_demo_scene": False,
    },
    {
        "id": "precomputed",
        "label": "Precomputed",
        "requires_precomputed_plan": True,
        "requires_precomputed_demo_scene": True,
    },
]


def get_experiment_options() -> dict:
    execution_mode, detection_source = detect_execution_mode()

    default_controller = get_controller("seedo")

    if default_controller is None:
        raise RuntimeError(
            "Default controller 'seedo' is not registered."
        )

    return {
        "environment": {
            "execution_mode": execution_mode,
            "detection_source": detection_source,
            "editable": False,
        },

        "controllers": list_controllers(),

        "tasks": TASKS,

        "execution_types": EXECUTION_TYPES,

        "demo": {
            "custom_path_supported": True,
            "custom_path_required": False,
        },

        "isaac": {
            "available": execution_mode == "isaac",
            "scene_configuration_supported": True,
            "default_configure_scene": True,
        },

        "precomputed": {
            "action_plan_supported": True,
            "demo_scene_supported": True,
            "note": (
                "The Solution uses generalized perception, therefore "
                "precomputed execution requires both the action plan "
                "and the demonstration structured scene."
            ),
        },

        "defaults": {
            "controller": "seedo",
            "task": "pick_place",
            "demo_task": 0,
            "demo_path": None,
            "execution_type": "full",
            "config": default_controller.default_config,

            "configure_isaac_scene": (
                execution_mode == "isaac"
            ),
            "scene_task": 0,
            "scene_trajectory": 0,

            "precomputed_plan": None,
            "precomputed_demo_scene": None,
        },

        "config_policy": {
            "editable": False,
            "description": (
                "The controller default Solution configuration "
                "is selected automatically."
            ),
        },
    }