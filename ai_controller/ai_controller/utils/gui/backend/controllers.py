import os
from dataclasses import asdict, dataclass
from pathlib import Path


REPO_ROOT = Path(
    os.environ.get(
        "UR_APPLICATION_ROOT",
        "/home/ros2_ws/src/UR5e-2f-85",
    )
)


@dataclass(frozen=True)
class ControllerDefinition:
    id: str
    label: str
    ros_target: str
    default_config: str
    supported_execution_modes: tuple[str, ...]
    supported_tasks: tuple[str, ...]
    supports_full: bool = True
    supports_precomputed: bool = True

    def to_dict(self) -> dict:
        data = asdict(self)

        # JSON-friendly representation.
        data["supported_execution_modes"] = list(
            self.supported_execution_modes
        )
        data["supported_tasks"] = list(self.supported_tasks)

        return data


SEEDO_CONFIG = (
    REPO_ROOT
    / "ai_controller"
    / "ai_controller"
    / "models"
    / "seedo_controller"
    / "config"
    / "seedo_controller.yaml"
)


CONTROLLERS: dict[str, ControllerDefinition] = {
    "seedo": ControllerDefinition(
        id="seedo",
        label="SeeDo",
        ros_target="seedo_controller",
        default_config=str(SEEDO_CONFIG),
        supported_execution_modes=(
            "isaac",
            "real",
        ),
        supported_tasks=(
            "pick_place",
            "nut_assembly",
        ),
        supports_full=True,
        supports_precomputed=True,
    ),
}


def list_controllers() -> list[dict]:
    return [
        controller.to_dict()
        for controller in CONTROLLERS.values()
    ]


def get_controller(controller_id: str) -> ControllerDefinition | None:
    return CONTROLLERS.get(controller_id)