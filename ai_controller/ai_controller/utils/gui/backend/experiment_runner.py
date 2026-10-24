from __future__ import annotations

import shlex
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel

from ai_controller.utils.gui.backend.controllers import (
    get_controller,
)
from ai_controller.utils.gui.backend.environment import (
    detect_execution_mode,
)


REPO_ROOT = Path(
    "/home/ros2_ws/src/UR5e-2f-85"
)

SEEDO_LAUNCH_ROOT = (
    REPO_ROOT
    / "ai_controller"
    / "ai_controller"
    / "models"
    / "seedo_controller"
    / "launch"
)

ISAAC_LAUNCHER = (
    SEEDO_LAUNCH_ROOT
    / "run_seedo_isaac_test.sh"
)

REAL_LAUNCHER = (
    SEEDO_LAUNCH_ROOT
    / "run_seedo_real_test.sh"
)


class ExperimentRequest(BaseModel):
    controller: str = "seedo"

    task: Literal[
        "pick_place",
        "nut_assembly",
    ]

    demo_task: int

    demo_path: str | None = None

    execution_type: Literal[
        "full",
        "precomputed",
    ] = "full"

    configure_isaac_scene: bool = True

    scene_task: int | None = None
    scene_trajectory: int | None = None

    precomputed_plan: str | None = None
    precomputed_demo_scene: str | None = None


def _validate_integer_range(
    value: int,
    minimum: int,
    maximum: int,
    label: str,
) -> None:
    if value < minimum or value > maximum:
        raise ValueError(
            f"{label} must be between "
            f"{minimum} and {maximum}."
        )


def _validate_task_values(
    request: ExperimentRequest,
    execution_mode: str,
) -> None:
    if request.task == "pick_place":
        _validate_integer_range(
            request.demo_task,
            0,
            15,
            "Pick-and-Place demonstration task",
        )

        if (
            execution_mode == "isaac"
            and request.configure_isaac_scene
        ):
            if request.scene_task is None:
                raise ValueError(
                    "scene_task is required when "
                    "Isaac scene configuration is enabled."
                )

            if request.scene_trajectory is None:
                raise ValueError(
                    "scene_trajectory is required when "
                    "Isaac scene configuration is enabled."
                )

            _validate_integer_range(
                request.scene_task,
                0,
                15,
                "Pick-and-Place scene task",
            )

            _validate_integer_range(
                request.scene_trajectory,
                0,
                39,
                "Pick-and-Place scene trajectory",
            )

    elif request.task == "nut_assembly":
        _validate_integer_range(
            request.demo_task,
            0,
            8,
            "Nut-Assembly demonstration task",
        )

        if (
            execution_mode == "isaac"
            and request.configure_isaac_scene
        ):
            if request.scene_task is None:
                raise ValueError(
                    "scene_task is required when "
                    "Isaac scene configuration is enabled."
                )

            if request.scene_trajectory is None:
                raise ValueError(
                    "scene_trajectory is required when "
                    "Isaac scene configuration is enabled."
                )

            _validate_integer_range(
                request.scene_task,
                0,
                8,
                "Nut-Assembly scene task",
            )

            _validate_integer_range(
                request.scene_trajectory,
                0,
                0,
                "Nut-Assembly scene trajectory",
            )


def _read_perception_mode(
    config_path: Path,
) -> str:
    with config_path.open(
        "r",
        encoding="utf-8",
    ) as file:
        config = yaml.safe_load(file)

    perception_mode = str(
        config.get(
            "perception_mode",
            "",
        )
    ).strip().lower()

    if perception_mode not in {
        "generalized",
        "prior_guided",
    }:
        raise ValueError(
            "Unsupported or missing perception_mode "
            f"in config: {config_path}"
        )

    return perception_mode


def _validate_file(
    path_string: str,
    label: str,
) -> Path:
    path = Path(
        path_string
    ).expanduser()

    if not path.is_file():
        raise ValueError(
            f"{label} does not exist: {path}"
        )

    return path

def _resolve_demo_path(
    request: ExperimentRequest,
) -> Path:

    if request.demo_path is not None:
        return _validate_file(
            request.demo_path,
            "Demonstration video",
        )

    demo_task_id = f"{request.demo_task:02d}"

    if request.task == "pick_place":
        demo_path = Path(
            "/test_dataset"
            "/pick_place"
            "/human_rgb_pick_place"
            f"/task_{demo_task_id}"
            "/traj000"
            "/converted"
            "/traj000-h264-30fps.mp4"
        )

    elif request.task == "nut_assembly":
        demo_path = Path(
            f"/dataset_nut/task_{demo_task_id}"
            "/traj_000_camera_front.mp4"
        )

    else:
        raise ValueError(
            f"Unsupported task: {request.task}"
        )

    return _validate_file(
        str(demo_path),
        "Demonstration video",
    )

def build_experiment_command(
    request: ExperimentRequest,
) -> dict:
    execution_mode, detection_source = (
        detect_execution_mode()
    )

    controller = get_controller(
        request.controller
    )

    if controller is None:
        raise ValueError(
            f"Unknown controller: {request.controller}"
        )

    if execution_mode not in (
        controller.supported_execution_modes
    ):
        raise ValueError(
            f"Controller '{request.controller}' "
            f"does not support '{execution_mode}'."
        )

    if request.task not in (
        controller.supported_tasks
    ):
        raise ValueError(
            f"Controller '{request.controller}' "
            f"does not support task '{request.task}'."
        )

    _validate_task_values(
        request=request,
        execution_mode=execution_mode,
    )

    config_path = Path(
        controller.default_config
    )

    if not config_path.is_file():
        raise ValueError(
            "Controller configuration does not exist: "
            f"{config_path}"
        )

    perception_mode = _read_perception_mode(
        config_path
    )

    demo_path = _resolve_demo_path(
        request
    )

    if request.execution_type == "full":
        if (
            request.precomputed_plan is not None
            or request.precomputed_demo_scene is not None
        ):
            raise ValueError(
                "Precomputed artifacts cannot be supplied "
                "for a full pipeline execution."
            )

    elif request.execution_type == "precomputed":
        if request.precomputed_plan is None:
            raise ValueError(
                "Precomputed execution requires "
                "an action plan."
            )

        _validate_file(
            request.precomputed_plan,
            "Precomputed action plan",
        )

        if (
            perception_mode == "generalized"
            and request.precomputed_demo_scene is None
        ):
            raise ValueError(
                "Generalized precomputed execution "
                "requires a demonstration structured scene."
            )

        if request.precomputed_demo_scene is not None:
            _validate_file(
                request.precomputed_demo_scene,
                "Precomputed demonstration structured scene",
            )

    if execution_mode == "isaac":
        launcher = ISAAC_LAUNCHER

    elif execution_mode == "real":
        launcher = REAL_LAUNCHER

    else:
        raise ValueError(
            f"Unsupported execution mode: {execution_mode}"
        )

    if not launcher.is_file():
        raise ValueError(
            f"Launcher does not exist: {launcher}"
        )

    command = [
        "bash",
        str(launcher),

        "--env-type",
        request.task,

        "--demo-task",
        str(request.demo_task),

        "--config",
        str(config_path),
    ]

    command.extend(
        [
            "--demo-path",
            str(demo_path),
        ]
    )

    if execution_mode == "isaac":
        if request.configure_isaac_scene:
            command.append(
                "--configure-scene"
            )

            command.extend(
                [
                    "--scene-task",
                    str(request.scene_task),

                    "--scene-trajectory",
                    str(request.scene_trajectory),
                ]
            )

        else:
            command.append(
                "--skip-scene-config"
            )

    if request.execution_type == "precomputed":
        command.extend(
            [
                "--precomputed-plan",
                request.precomputed_plan,
            ]
        )

        if request.precomputed_demo_scene is not None:
            command.extend(
                [
                    "--precomputed-demo-scene",
                    request.precomputed_demo_scene,
                ]
            )

    return {
        "execution_mode": execution_mode,
        "execution_mode_source": detection_source,
        "controller": request.controller,
        "task": request.task,
        "execution_type": request.execution_type,
        "config": str(config_path),
        "perception_mode": perception_mode,
        "demo_path": str(demo_path),
        "launcher": str(launcher),
        "command": command,
        "shell_command": shlex.join(
            command
        ),
    }