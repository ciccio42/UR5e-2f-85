from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from ai_controller.script_controller.geometry_utils import (
    GripperPose,
    build_linear_waypoints,
)

from results import (
    PrimitiveStep,
    SceneObject,
    SceneState,
)


class SeeDoMotionLayer:
    """Translate SeeDo symbolic primitives into executable robot actions.

    The generated actions follow the format expected by AIControllerNode:

        [x, y, z, qx, qy, qz, qw, gripper_position]

    This class performs motion translation only. It does not execute ROS
    services, action clients, or physical robot motion.
    """

    def __init__(
        self,
        grasp_orientation,
        min_step: float,
        reach_hover_height: float,
        approach_z_offset: float,
        release_height_offset: float,
        lift_height: float,
        gripper_open_position: float,
        gripper_closed_position: float,
        object_y_offset: float = -0.06,
    ) -> None:
        self.grasp_orientation = np.asarray(
            grasp_orientation,
            dtype=np.float64,
        )

        if self.grasp_orientation.shape != (4,):
            raise ValueError(
                "grasp_orientation must contain exactly 4 values "
                "[qx, qy, qz, qw]."
            )

        self.min_step = float(min_step)
        self.reach_hover_height = float(reach_hover_height)
        self.approach_z_offset = float(approach_z_offset)
        self.release_height_offset = float(
            release_height_offset
        )
        self.lift_height = float(lift_height)

        self.gripper_open_position = float(
            gripper_open_position
        )
        self.gripper_closed_position = float(
            gripper_closed_position
        )

        self.object_y_offset = float(object_y_offset)

        self.current_pose: GripperPose | None = None
        self.current_gripper_position = (
            self.gripper_open_position
        )

        self.artifacts_dir: Path | None = None
        self.motion_history: list[dict[str, Any]] = []

    def reset(
        self,
        current_position,
        current_orientation,
        artifacts_dir: str | Path | None = None,
    ) -> None:
        """Initialize the motion state from the current TCP pose."""

        position = np.asarray(
            current_position,
            dtype=np.float64,
        )
        orientation = np.asarray(
            current_orientation,
            dtype=np.float64,
        )

        if position.shape != (3,):
            raise ValueError(
                "current_position must contain exactly 3 values "
                "[x, y, z]."
            )

        if orientation.shape != (4,):
            raise ValueError(
                "current_orientation must contain exactly 4 values "
                "[qx, qy, qz, qw]."
            )

        self.current_pose = GripperPose(
            position,
            orientation,
        )

        self.current_gripper_position = (
            self.gripper_open_position
        )

        self.motion_history = []

        if artifacts_dir is None:
            self.artifacts_dir = None
        else:
            self.artifacts_dir = (
                Path(artifacts_dir)
                .expanduser()
                .resolve()
            )

            self.artifacts_dir.mkdir(
                parents=True,
                exist_ok=True,
            )

            self._write_motion_artifact()

    def translate(
        self,
        primitive_step: PrimitiveStep,
        scene_state: SceneState,
    ) -> list[np.ndarray]:
        """Translate one symbolic primitive into executable actions."""

        if self.current_pose is None:
            raise RuntimeError(
                "SeeDoMotionLayer has not been initialized. "
                "Call reset() with the current TCP pose before "
                "translating primitives."
            )

        if not isinstance(primitive_step, PrimitiveStep):
            raise TypeError(
                "primitive_step must be a PrimitiveStep, "
                f"received {type(primitive_step).__name__}."
            )

        if not isinstance(scene_state, SceneState):
            raise TypeError(
                "scene_state must be a SceneState, "
                f"received {type(scene_state).__name__}."
            )

        primitive_name = primitive_step.name.strip()

        if primitive_name == "reach":
            actions = self._reach(
                primitive_step,
                scene_state,
            )

        elif primitive_name == "approaching":
            actions = self._approaching(
                primitive_step,
                scene_state,
            )

        elif primitive_name == "pick":
            actions = self._pick(
                primitive_step,
                scene_state,
            )

        elif primitive_name == "lift_up":
            actions = self._lift_up(
                primitive_step,
                scene_state,
            )

        elif primitive_name == "moving":
            actions = self._moving(
                primitive_step,
                scene_state,
            )

        elif primitive_name == "placing":
            actions = self._placing(
                primitive_step,
                scene_state,
            )

        else:
            raise ValueError(
                f"Unsupported SeeDo primitive: {primitive_name!r}"
            )

        self._record_translation(
            primitive_step=primitive_step,
            actions=actions,
        )

        return actions

    def _reach(
        self,
        primitive_step: PrimitiveStep,
        scene_state: SceneState,
    ) -> list[np.ndarray]:
        target = self._resolve_target(
            primitive_step,
            scene_state,
        )

        target_position = np.asarray(
            target.position_base,
            dtype=np.float64,
        ).copy()

        target_position[1] += self.object_y_offset
        target_position[2] += self.reach_hover_height

        return self._move_linear(
            target_position=target_position,
            target_orientation=self.grasp_orientation,
        )

    def _approaching(
        self,
        primitive_step: PrimitiveStep,
        scene_state: SceneState,
    ) -> list[np.ndarray]:
        target = self._resolve_target(
            primitive_step,
            scene_state,
        )

        target_position = np.asarray(
            target.position_base,
            dtype=np.float64,
        ).copy()

        target_position[1] += self.object_y_offset
        target_position[2] += self.approach_z_offset

        return self._move_linear(
            target_position=target_position,
            target_orientation=self.grasp_orientation,
        )

    def _pick(
        self,
        primitive_step: PrimitiveStep,
        scene_state: SceneState,
    ) -> list[np.ndarray]:
        self._resolve_target(
            primitive_step,
            scene_state,
        )

        self.current_gripper_position = (
            self.gripper_closed_position
        )

        return [
            self._build_action(
                position=self.current_pose.position,
                orientation=self.current_pose.orientation,
                gripper_position=(
                    self.current_gripper_position
                ),
            )
        ]

    def _lift_up(
        self,
        primitive_step: PrimitiveStep,
        scene_state: SceneState,
    ) -> list[np.ndarray]:
        self._resolve_target(
            primitive_step,
            scene_state,
        )

        target_position = np.array(
            [
                self.current_pose.position[0],
                self.current_pose.position[1],
                (
                    self.current_pose.position[2]
                    + self.lift_height
                ),
            ],
            dtype=np.float64,
        )

        return self._move_linear(
            target_position=target_position,
            target_orientation=self.grasp_orientation,
        )

    def _moving(
        self,
        primitive_step: PrimitiveStep,
        scene_state: SceneState,
    ) -> list[np.ndarray]:
        target = self._resolve_target(
            primitive_step,
            scene_state,
        )

        target_position = np.array(
            [
                target.position_base[0],
                target.position_base[1],
                self.current_pose.position[2],
            ],
            dtype=np.float64,
        )

        return self._move_linear(
            target_position=target_position,
            target_orientation=self.grasp_orientation,
        )

    def _placing(
        self,
        primitive_step: PrimitiveStep,
        scene_state: SceneState,
    ) -> list[np.ndarray]:
        target = self._resolve_target(
            primitive_step,
            scene_state,
        )

        target_position = np.array(
            [
                target.position_base[0],
                target.position_base[1],
                (
                    target.position_base[2]
                    + self.release_height_offset
                ),
            ],
            dtype=np.float64,
        )

        actions = self._move_linear(
            target_position=target_position,
            target_orientation=self.grasp_orientation,
        )

        self.current_gripper_position = (
            self.gripper_open_position
        )

        actions.append(
            self._build_action(
                position=self.current_pose.position,
                orientation=self.current_pose.orientation,
                gripper_position=(
                    self.current_gripper_position
                ),
            )
        )

        return actions

    def _move_linear(
        self,
        target_position,
        target_orientation,
    ) -> list[np.ndarray]:
        if self.current_pose is None:
            raise RuntimeError(
                "Cannot generate motion without a current pose."
            )

        waypoints = build_linear_waypoints(
            self.current_pose.position,
            self.current_pose.orientation,
            target_position,
            target_orientation,
            self.min_step,
        )

        actions = [
            self._build_action(
                position=waypoint.position,
                orientation=waypoint.orientation,
                gripper_position=(
                    self.current_gripper_position
                ),
            )
            for waypoint in waypoints
        ]

        if waypoints:
            self.current_pose = waypoints[-1]

        return actions

    @staticmethod
    def _build_action(
        position,
        orientation,
        gripper_position: float,
    ) -> np.ndarray:
        position = np.asarray(
            position,
            dtype=np.float64,
        )
        orientation = np.asarray(
            orientation,
            dtype=np.float64,
        )

        return np.concatenate(
            [
                position,
                orientation,
                np.array(
                    [gripper_position],
                    dtype=np.float64,
                ),
            ]
        )

    def _record_translation(
        self,
        primitive_step: PrimitiveStep,
        actions: list[np.ndarray],
    ) -> None:
        """Record one primitive-to-actions translation."""

        self.motion_history.append(
            {
                "name": primitive_step.name,
                "arguments": dict(
                    primitive_step.arguments
                ),
                "actions": [
                    {
                        "position": action[:3].tolist(),
                        "orientation": action[3:7].tolist(),
                        "gripper_position": float(
                            action[7]
                        ),
                    }
                    for action in actions
                ],
            }
        )

        self._write_motion_artifact()


    def _write_motion_artifact(self) -> None:
        """Write the accumulated motion plan when artifact storage is enabled."""

        if self.artifacts_dir is None:
            return

        total_actions = sum(
            len(primitive["actions"])
            for primitive in self.motion_history
        )

        final_state = None

        if self.current_pose is not None:
            final_state = {
                "position": (
                    np.asarray(
                        self.current_pose.position,
                        dtype=np.float64,
                    ).tolist()
                ),
                "orientation": (
                    np.asarray(
                        self.current_pose.orientation,
                        dtype=np.float64,
                    ).tolist()
                ),
                "gripper_position": float(
                    self.current_gripper_position
                ),
            }

        payload = {
            "primitives": self.motion_history,
            "total_actions": total_actions,
            "final_planned_state": final_state,
        }

        output_path = (
            self.artifacts_dir
            / "motion_plan.json"
        )

        with output_path.open(
            "w",
            encoding="utf-8",
        ) as stream:
            json.dump(
                payload,
                stream,
                indent=2,
            )

    def _resolve_target(
        self,
        primitive_step: PrimitiveStep,
        scene_state: SceneState,
    ) -> SceneObject:
        target_name = self._extract_target_name(
            primitive_step.arguments
        )

        normalized_target = target_name.strip().lower()

        matches = [
            obj
            for obj in scene_state.objects
            if obj.object_id.strip().lower() == normalized_target
        ]

        if not matches:
            available_objects = [
                obj.object_id
                for obj in scene_state.objects
            ]

            raise ValueError(
                "SeeDo Motion Layer could not resolve target "
                f"{target_name!r} in SceneState. "
                f"Available objects: {available_objects}"
            )

        if len(matches) > 1:
            raise ValueError(
                "SeeDo Motion Layer found multiple SceneState "
                f"objects matching target {target_name!r}."
            )

        return matches[0]

    @staticmethod
    def _extract_target_name(
        arguments: dict[str, Any],
    ) -> str:
        if "target" in arguments:
            target = arguments["target"]
        elif "object" in arguments:
            target = arguments["object"]
        elif "destination" in arguments:
            target = arguments["destination"]
        else:
            raise ValueError(
                "PrimitiveStep does not contain a recognized target "
                "argument. Expected one of: "
                "'target', 'object', 'destination'."
            )

        if not isinstance(target, str):
            raise TypeError(
                "Primitive target must be a string, "
                f"received {type(target).__name__}."
            )

        target = target.strip()

        if not target:
            raise ValueError(
                "Primitive target cannot be empty."
            )

        return target