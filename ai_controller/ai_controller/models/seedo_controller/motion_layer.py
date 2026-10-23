from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from ai_controller.script_controller.geometry_utils import (
    GripperPose,
    build_linear_waypoints,
)
from results import PrimitiveStep, SceneObject, SceneState


class SeeDoMotionLayer:
    """Translate SeeDo symbolic primitives to robot action vectors.

    Action format: [x, y, z, qx, qy, qz, qw, gripper_position].

    PICK_AND_PLACE follows the pre-existing primitives. NUT_ASSEMBLY uses
    task-specific branches in lift_up/moving and dedicated aligning/inserting.
    This class only generates actions; it never executes robot commands.
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
        assembly_alignment_z_offset: float = 0.135,
        assembly_insertion_z_offset: float = -0.11,
        grasp_planning_enabled: bool = True,
        pregrasp_distance: float = 0.08,
        grasp_depth_offset: float = 0.005,
        retreat_distance: float = 0.04,
        nut_assembly_config: dict[str, Any] | None = None,
    ) -> None:
        self.grasp_orientation = np.asarray(grasp_orientation, dtype=np.float64)
        if self.grasp_orientation.shape != (4,):
            raise ValueError("grasp_orientation must contain [qx, qy, qz, qw].")

        self.min_step = float(min_step)
        self.reach_hover_height = float(reach_hover_height)
        self.approach_z_offset = float(approach_z_offset)
        self.release_height_offset = float(release_height_offset)
        self.lift_height = float(lift_height)
        self.gripper_open_position = float(gripper_open_position)
        self.gripper_closed_position = float(gripper_closed_position)
        self.object_y_offset = float(object_y_offset)
        self.assembly_alignment_z_offset = float(assembly_alignment_z_offset)
        self.assembly_insertion_z_offset = float(assembly_insertion_z_offset)
        self.grasp_planning_enabled = bool(grasp_planning_enabled)
        self.pregrasp_distance = float(pregrasp_distance)
        self.grasp_depth_offset = float(grasp_depth_offset)
        self.retreat_distance = float(retreat_distance)

        # Nut assembly settings are inspected ONLY for the nut_assembly task.
        self.nut_assembly_config = dict(nut_assembly_config or {})
        self.task_type = "pick_and_place"
        self._nut_cfg: dict[str, Any] | None = None

        # Picked-object geometry expressed in the planned grasp TCP frame.
        #
        # Once the gripper closes, these quantities remain rigidly
        # attached to the TCP.
        self._nut_pick_target: str | None = None

        self._nut_hole_top_local: np.ndarray | None = None
        self._nut_hole_bottom_local: np.ndarray | None = None
        self._nut_hole_axis_local: np.ndarray | None = None

        self._nut_object_radius_from_hole_m: float | None = None
        self._nut_object_sweep_radius_m: float | None = None

        # Task-specific geometric refinements produced after CAP has
        # resolved the actual assembly destination.
        #
        # Keys are normalized runtime object IDs.
        self._nut_place_refinements: dict[
            str,
            dict[str, Any],
        ] = {}

        # Destination locked by MOVING and reused by ALIGNING/INSERTING.
        self._nut_peg_target: str | None = None

        self._nut_phase = "idle"

        self.current_pose: GripperPose | None = None
        self.current_gripper_position = self.gripper_open_position
        self.active_grasp_position: np.ndarray | None = None
        self.active_grasp_orientation: np.ndarray | None = None
        self.active_approach_direction: np.ndarray | None = None
        self.active_grasp_backend: str | None = None
        self.artifacts_dir: Path | None = None
        self.motion_history: list[dict[str, Any]] = []

    @staticmethod
    def _checked_nut_config(
        raw: dict[str, Any],
    ) -> dict[str, Any]:
        """
        Validate Nut-Assembly motion-policy parameters.

        Scene geometry is intentionally NOT configured here.
        Peg geometry, nut geometry and grasp-relative geometry
        are estimated at runtime from perception.
        """

        out: dict[str, Any] = {
            "peg_clearance_m": float(
                raw.get(
                    "peg_clearance_m",
                    0.02,
                )
            ),

            "transport_clearance_margin_m": float(
                raw.get(
                    "transport_clearance_margin_m",
                    0.050,
                )
            ),

            "initial_entry_m": float(
                raw.get(
                    "initial_entry_m",
                    0.005,
                )
            ),

            "insertion_step_m": float(
                raw.get(
                    "insertion_step_m",
                    0.005,
                )
            ),

            "initial_entry_step_m": float(
                raw.get(
                    "initial_entry_step_m",
                    0.003,
                )
            ),

            "insertion_slide_step_m": float(
                raw.get(
                    "insertion_slide_step_m",
                    0.015,
                )
            ),

            "release_bottom_height_m": float(
                raw.get(
                    "release_bottom_height_m",
                    0.01,
                )
            ),

            "rotation_step_deg": float(
                raw.get(
                    "rotation_step_deg",
                    5.0,
                )
            ),

            "auto_release": bool(
                raw.get(
                    "auto_release",
                    True,
                )
            ),
        }

        finite_fields = [
            value
            for key, value in out.items()
            if key != "auto_release"
        ]

        if not np.isfinite(
            finite_fields
        ).all():
            raise ValueError(
                "Nut-Assembly motion-policy "
                "configuration must be finite."
            )

        clearance = out[
            "peg_clearance_m"
        ]

        clearance_margin = out[
            "transport_clearance_margin_m"
        ]

        entry = out[
            "initial_entry_m"
        ]

        step = out[
            "insertion_step_m"
        ]

        initial_entry_step = out[
            "initial_entry_step_m"
        ]

        insertion_slide_step = out[
            "insertion_slide_step_m"
        ]

        release_height = out[
            "release_bottom_height_m"
        ]

        rotation_step = out[
            "rotation_step_deg"
        ]

        if not (
            0.005
            <= clearance
            <= 0.08
        ):
            raise ValueError(
                "peg_clearance_m must be "
                "between 5 and 80 mm."
            )

        if not (
            0.0
            <= clearance_margin
            <= 0.1
        ):
            raise ValueError(
                "transport_clearance_margin_m must be "
                "between 0 and 100 mm."
            )

        if not (
            0.001
            <= entry
            <= 0.02
        ):
            raise ValueError(
                "initial_entry_m must be "
                "between 1 and 20 mm."
            )

        if entry >= clearance:
            raise ValueError(
                "initial_entry_m must be smaller "
                "than peg_clearance_m."
            )

        if not (
            0.001
            <= step
            <= 0.02
        ):
            raise ValueError(
                "insertion_step_m must be "
                "between 1 and 20 mm."
            )

        if not (
            0.001
            <= initial_entry_step
            <= 0.02
        ):
            raise ValueError(
                "initial_entry_step_m must be "
                "between 1 and 20 mm."
            )

        if not (
            0.005
            <= insertion_slide_step
            <= 0.05
        ):
            raise ValueError(
                "insertion_slide_step_m must be "
                "between 5 and 50 mm."
            )

        if not (
            0.0
            <= release_height
            <= 0.10
        ):
            raise ValueError(
                "release_bottom_height_m must be "
                "between 0 and 100 mm."
            )

        if not (
            0.0
            < rotation_step
            <= 15.0
        ):
            raise ValueError(
                "rotation_step_deg must be "
                "in (0, 15] degrees."
            )

        return out

    def reset(
        self,
        current_position,
        current_orientation,
        artifacts_dir: str | Path | None = None,
        task_type: str = "pick_and_place",
    ) -> None:
        """Initialize the Motion Layer from the measured robot TCP pose."""
        position = np.asarray(current_position, dtype=np.float64)
        orientation = np.asarray(current_orientation, dtype=np.float64)
        if position.shape != (3,):
            raise ValueError(
                "current_position must contain exactly 3 values [x, y, z]."
            )
        if orientation.shape != (4,):
            raise ValueError(
                "current_orientation must contain exactly 4 values [qx, qy, qz, qw]."
            )

        task = str(task_type).strip().lower()
        if task not in ("pick_and_place", "nut_assembly"):
            raise ValueError(f"Unsupported Motion Layer task_type: {task!r}")
        cfg = (
            self._checked_nut_config(self.nut_assembly_config)
            if task == "nut_assembly" else None
        )

        self.current_pose = GripperPose(position, orientation)
        self.current_gripper_position = self.gripper_open_position
        self.active_grasp_position = None
        self.active_grasp_orientation = None
        self.active_approach_direction = None
        self.active_grasp_backend = None
        self.motion_history = []
        self.task_type = task
        self._nut_cfg = cfg

        self._nut_place_refinements = {}
        self._nut_peg_target = None
        self._nut_phase = "idle"

        self._nut_pick_target = None

        self._nut_hole_top_local = None
        self._nut_hole_bottom_local = None
        self._nut_hole_axis_local = None

        self._nut_object_radius_from_hole_m = None
        self._nut_object_sweep_radius_m = None

        if artifacts_dir is None:
            self.artifacts_dir = None
        else:
            self.artifacts_dir = Path(artifacts_dir).expanduser().resolve()
            self.artifacts_dir.mkdir(parents=True, exist_ok=True)
            self._write_motion_artifact()

    def set_grasp_pose(
        self,
        position,
        orientation,
        approach_direction,
        backend_name: str = "m2t2",
    ) -> None:
        """Set the active TCP grasp pose (all data in base_link)."""
        position = np.asarray(position, dtype=np.float64)
        orientation = np.asarray(orientation, dtype=np.float64)
        approach_direction = np.asarray(approach_direction, dtype=np.float64)
        if position.shape != (3,):
            raise ValueError(
                "Grasp position must contain exactly 3 values [x, y, z]."
            )
        if orientation.shape != (4,):
            raise ValueError(
                "Grasp orientation must contain exactly 4 values [qx, qy, qz, qw]."
            )
        if not np.isfinite(position).all():
            raise ValueError("Grasp position contains non-finite values.")
        if not np.isfinite(orientation).all():
            raise ValueError("Grasp orientation contains non-finite values.")
        if approach_direction.shape != (3,):
            raise ValueError(
                "Approach direction must contain exactly 3 values [x, y, z]."
            )
        if not np.isfinite(approach_direction).all():
            raise ValueError("Approach direction contains non-finite values.")
        approach_norm = np.linalg.norm(approach_direction)
        if approach_norm < 1e-8:
            raise ValueError("Approach direction has near-zero norm.")
        quaternion_norm = np.linalg.norm(orientation)
        if quaternion_norm < 1e-8:
            raise ValueError("Grasp orientation has near-zero norm.")

        self.active_grasp_position = position.copy()
        self.active_grasp_orientation = orientation / quaternion_norm
        self.active_approach_direction = approach_direction / approach_norm
        backend_name = str(backend_name).strip().lower()
        if backend_name not in ("m2t2", "graspgenx"):
            raise ValueError(
                f"Unsupported grasp backend for Motion Layer: {backend_name!r}."
            )
        self.active_grasp_backend = backend_name

    def set_nut_grasp_geometry(
        self,
        *,
        target_id: str,
        hole_center_base,
        hole_bottom_center_base,
        hole_axis_base,
        object_radius_from_hole_m: float,
        reference_tcp_position=None,
        reference_tcp_orientation=None,
        synchronize_current_pose: bool = False,
    ) -> None:
        """
        Freeze the perceived Nut-Assembly hole geometry in a TCP frame.

        When reference_tcp_position/reference_tcp_orientation are omitted,
        the geometry is expressed in the planned grasp TCP frame. This is
        the normal GraspPlanner path.

        When an explicit reference TCP is provided, the geometry is instead
        expressed in that measured TCP frame. This is used by the legacy
        grasp path after APPROACHING has physically completed.

        If synchronize_current_pose is True, the Motion Layer internal pose
        is also synchronized with the supplied reference TCP.
        """

        if self.task_type != "nut_assembly":
            raise RuntimeError(
                "Nut grasp geometry can only be configured "
                "for a nut_assembly task."
            )

        has_reference_position = (
            reference_tcp_position is not None
        )

        has_reference_orientation = (
            reference_tcp_orientation is not None
        )

        if (
            has_reference_position
            != has_reference_orientation
        ):
            raise ValueError(
                "reference_tcp_position and "
                "reference_tcp_orientation must be provided together."
            )

        use_reference_tcp = (
            has_reference_position
            and has_reference_orientation
        )

        if use_reference_tcp:

            grasp_position = np.asarray(
                reference_tcp_position,
                dtype=np.float64,
            )

            grasp_orientation = np.asarray(
                reference_tcp_orientation,
                dtype=np.float64,
            )

            if (
                grasp_position.shape != (3,)
                or not np.isfinite(
                    grasp_position
                ).all()
            ):
                raise ValueError(
                    "Invalid reference TCP position."
                )

            if (
                grasp_orientation.shape != (4,)
                or not np.isfinite(
                    grasp_orientation
                ).all()
            ):
                raise ValueError(
                    "Invalid reference TCP orientation."
                )

            quaternion_norm = float(
                np.linalg.norm(
                    grasp_orientation
                )
            )

            if quaternion_norm <= 1e-8:
                raise ValueError(
                    "Reference TCP quaternion has near-zero norm."
                )

            grasp_orientation = (
                grasp_orientation
                / quaternion_norm
            )

        else:

            if (
                self.active_grasp_position is None
                or self.active_grasp_orientation is None
            ):
                raise RuntimeError(
                    "Active grasp pose must be configured before "
                    "Nut-Assembly grasp geometry when no explicit "
                    "reference TCP is provided."
                )

            grasp_position = np.asarray(
                self.active_grasp_position,
                dtype=np.float64,
            ).copy()

            grasp_orientation = np.asarray(
                self.active_grasp_orientation,
                dtype=np.float64,
            ).copy()

        if (
            synchronize_current_pose
            and not use_reference_tcp
        ):
            raise ValueError(
                "synchronize_current_pose requires an explicit "
                "reference TCP."
            )

        target_id = str(
            target_id
        ).strip()

        if not target_id:
            raise ValueError(
                "Nut pick target cannot be empty."
            )

        hole_top = np.asarray(
            hole_center_base,
            dtype=np.float64,
        )

        hole_bottom = np.asarray(
            hole_bottom_center_base,
            dtype=np.float64,
        )

        hole_axis = np.asarray(
            hole_axis_base,
            dtype=np.float64,
        )

        if (
            hole_top.shape != (3,)
            or not np.isfinite(hole_top).all()
        ):
            raise ValueError(
                "Invalid hole_center_base."
            )

        if (
            hole_bottom.shape != (3,)
            or not np.isfinite(hole_bottom).all()
        ):
            raise ValueError(
                "Invalid hole_bottom_center_base."
            )

        if (
            hole_axis.shape != (3,)
            or not np.isfinite(hole_axis).all()
        ):
            raise ValueError(
                "Invalid hole_axis_base."
            )

        axis_norm = float(
            np.linalg.norm(
                hole_axis
            )
        )

        if axis_norm <= 1e-8:
            raise ValueError(
                "Hole axis has near-zero norm."
            )

        hole_axis /= axis_norm

        object_radius = float(
            object_radius_from_hole_m
        )

        if (
            not np.isfinite(object_radius)
            or object_radius <= 0.0
        ):
            raise ValueError(
                "Invalid object_radius_from_hole_m."
            )

        grasp_rotation = Rotation.from_quat(
            grasp_orientation
        )

        inverse_grasp_rotation = (
            grasp_rotation.inv()
        )

        hole_top_local = (
            inverse_grasp_rotation.apply(
                hole_top
                - grasp_position
            )
        )

        hole_bottom_local = (
            inverse_grasp_rotation.apply(
                hole_bottom
                - grasp_position
            )
        )

        hole_axis_local = (
            inverse_grasp_rotation.apply(
                hole_axis
            )
        )

        hole_axis_local /= np.linalg.norm(
            hole_axis_local
        )

        # Conservative radius of the swept object around the TCP.
        #
        # For every object point P:
        #
        # ||P - TCP||
        # <= ||hole_center - TCP||
        #    + ||P - hole_center||
        #
        # It therefore remains valid regardless of TCP orientation.
        object_sweep_radius = (
            max(
                float(
                    np.linalg.norm(
                        hole_top_local
                    )
                ),
                float(
                    np.linalg.norm(
                        hole_bottom_local
                    )
                ),
            )
            + object_radius_from_hole_m
        )

        self._nut_pick_target = (
            target_id.casefold()
        )

        self._nut_hole_top_local = (
            hole_top_local
        )

        self._nut_hole_bottom_local = (
            hole_bottom_local
        )

        self._nut_hole_axis_local = (
            hole_axis_local
        )

        self._nut_object_radius_from_hole_m = (
            object_radius
        )

        self._nut_object_sweep_radius_m = (
            object_sweep_radius
        )

        if synchronize_current_pose:

            self.current_pose = GripperPose(
                grasp_position.copy(),
                grasp_orientation.copy(),
            )

            print(
                "[NUT] Motion Layer synchronized with "
                "measured legacy grasp TCP | "
                f"position={grasp_position} | "
                f"orientation={grasp_orientation}",
                flush=True,
            )

        print(
            "[NUT] Grasp-relative hole geometry | "
            f"target={target_id!r} | "
            f"hole_bottom_local="
            f"{hole_bottom_local} | "
            f"hole_axis_local="
            f"{hole_axis_local} | "
            f"sweep_radius="
            f"{object_sweep_radius * 1000.0:.1f} mm",
            flush=True,
        )

    def update_nut_grasp_geometry_from_measured_tcp(
        self,
        *,
        target_id: str,
        hole_center_base,
        hole_bottom_center_base,
        hole_axis_base,
        current_tcp_position,
        current_tcp_orientation,
    ) -> None:
        """
        Replace the pre-grasp hole geometry with a post-grasp observation.

        The observed hole geometry is expressed in base_link and is converted
        into the ACTUAL measured TCP frame after the gripper has closed.

        This method is used only for Nut-Assembly.
        """

        if self.task_type != "nut_assembly":
            raise RuntimeError(
                "Post-grasp hole refinement is only valid "
                "for a nut_assembly task."
            )

        target_id = str(target_id).strip()

        if not target_id:
            raise ValueError(
                "Post-grasp nut target cannot be empty."
            )

        if self._nut_pick_target is None:
            raise RuntimeError(
                "Initial Nut-Assembly grasp geometry "
                "has not been configured."
            )

        if (
            target_id.casefold()
            != self._nut_pick_target
        ):
            raise RuntimeError(
                "Post-grasp refinement target does not match "
                "the currently grasped Nut-Assembly object: "
                f"{target_id!r} != {self._nut_pick_target!r}."
            )

        hole_top = np.asarray(
            hole_center_base,
            dtype=np.float64,
        )

        hole_bottom = np.asarray(
            hole_bottom_center_base,
            dtype=np.float64,
        )

        hole_axis = np.asarray(
            hole_axis_base,
            dtype=np.float64,
        )

        tcp_position = np.asarray(
            current_tcp_position,
            dtype=np.float64,
        )

        tcp_orientation = np.asarray(
            current_tcp_orientation,
            dtype=np.float64,
        )

        if (
            hole_top.shape != (3,)
            or not np.isfinite(hole_top).all()
        ):
            raise ValueError(
                "Invalid post-grasp hole_center_base."
            )

        if (
            hole_bottom.shape != (3,)
            or not np.isfinite(hole_bottom).all()
        ):
            raise ValueError(
                "Invalid post-grasp hole_bottom_center_base."
            )

        if (
            hole_axis.shape != (3,)
            or not np.isfinite(hole_axis).all()
        ):
            raise ValueError(
                "Invalid post-grasp hole_axis_base."
            )

        if (
            tcp_position.shape != (3,)
            or not np.isfinite(tcp_position).all()
        ):
            raise ValueError(
                "Invalid measured TCP position."
            )

        if (
            tcp_orientation.shape != (4,)
            or not np.isfinite(tcp_orientation).all()
        ):
            raise ValueError(
                "Invalid measured TCP orientation."
            )

        axis_norm = float(
            np.linalg.norm(hole_axis)
        )

        if axis_norm <= 1e-8:
            raise ValueError(
                "Post-grasp hole axis has near-zero norm."
            )

        hole_axis = (
            hole_axis
            / axis_norm
        )

        quaternion_norm = float(
            np.linalg.norm(tcp_orientation)
        )

        if quaternion_norm <= 1e-8:
            raise ValueError(
                "Measured TCP quaternion has near-zero norm."
            )

        tcp_orientation = (
            tcp_orientation
            / quaternion_norm
        )

        tcp_rotation = Rotation.from_quat(
            tcp_orientation
        )

        inverse_tcp_rotation = (
            tcp_rotation.inv()
        )

        new_hole_top_local = (
            inverse_tcp_rotation.apply(
                hole_top
                - tcp_position
            )
        )

        new_hole_bottom_local = (
            inverse_tcp_rotation.apply(
                hole_bottom
                - tcp_position
            )
        )

        new_hole_axis_local = (
            inverse_tcp_rotation.apply(
                hole_axis
            )
        )

        new_hole_axis_local /= np.linalg.norm(
            new_hole_axis_local
        )

        old_hole_bottom_local = (
            None
            if self._nut_hole_bottom_local is None
            else np.asarray(
                self._nut_hole_bottom_local,
                dtype=np.float64,
            ).copy()
        )

        old_hole_axis_local = (
            None
            if self._nut_hole_axis_local is None
            else np.asarray(
                self._nut_hole_axis_local,
                dtype=np.float64,
            ).copy()
        )

        self._nut_hole_top_local = (
            new_hole_top_local
        )

        self._nut_hole_bottom_local = (
            new_hole_bottom_local
        )

        self._nut_hole_axis_local = (
            new_hole_axis_local
        )

        # From this point onward the Motion Layer must start from
        # the REAL measured TCP pose, not from the previously
        # planned grasp pose.
        self.current_pose = GripperPose(
            tcp_position.copy(),
            tcp_orientation.copy(),
        )

        bottom_shift_mm = None

        if old_hole_bottom_local is not None:
            bottom_shift_mm = (
                float(
                    np.linalg.norm(
                        new_hole_bottom_local
                        - old_hole_bottom_local
                    )
                )
                * 1000.0
            )

        axis_change_deg = None

        if old_hole_axis_local is not None:

            old_axis = (
                old_hole_axis_local
                / np.linalg.norm(
                    old_hole_axis_local
                )
            )

            axis_dot = float(
                np.clip(
                    np.dot(
                        old_axis,
                        new_hole_axis_local,
                    ),
                    -1.0,
                    1.0,
                )
            )

            axis_change_deg = float(
                np.degrees(
                    np.arccos(
                        axis_dot
                    )
                )
            )

        print(
            "[NUT] POST-GRASP geometry updated | "
            f"target={target_id!r} | "
            f"hole_bottom_local="
            f"{new_hole_bottom_local} | "
            f"hole_axis_local="
            f"{new_hole_axis_local} | "
            f"bottom_shift="
            f"{bottom_shift_mm:.2f} mm"
            if bottom_shift_mm is not None
            else
            "[NUT] POST-GRASP geometry updated | "
            f"target={target_id!r} | "
            f"hole_bottom_local="
            f"{new_hole_bottom_local} | "
            f"hole_axis_local="
            f"{new_hole_axis_local}",
            flush=True,
        )

        if axis_change_deg is not None:
            print(
                "[NUT] POST-GRASP axis correction | "
                f"{axis_change_deg:.3f} deg",
                flush=True,
            )

    def _nut_refined_peg_geometry(
        self,
        primitive_step: PrimitiveStep,
        scene_state: SceneState,
    ) -> tuple[str, dict[str, Any]]:
        """
        Return the complete frozen runtime geometry of one
        already-resolved Nut-Assembly destination.
        """

        target = self._resolve_target(
            primitive_step,
            scene_state,
        )

        target_id = str(
            target.object_id
        ).strip()

        normalized_target = (
            target_id.casefold()
        )

        geometry = (
            self._nut_place_refinements.get(
                normalized_target
            )
        )

        if geometry is None:

            available_targets = [
                item["target_id"]
                for item
                in self._nut_place_refinements.values()
            ]

            raise RuntimeError(
                "No refined peg geometry is available for "
                f"Nut-Assembly destination {target_id!r}. "
                f"Available targets: {available_targets}."
            )

        return (
            target_id,
            geometry,
        )

    def set_nut_place_refinements(
        self,
        refinements: dict[str, dict[str, Any]],
    ) -> None:
        """
        Provide runtime peg geometry for Nut-Assembly destinations.

        SceneState remains responsible for semantic identity.
        This refinement is authoritative for assembly geometry.
        """

        if self.task_type != "nut_assembly":
            raise RuntimeError(
                "Nut place refinements can only be configured "
                "for a nut_assembly task."
            )

        if not isinstance(refinements, dict):
            raise TypeError(
                "Nut place refinements must be a dictionary."
            )

        if not refinements:
            raise ValueError(
                "Nut-Assembly requires at least one refined "
                "assembly destination."
            )

        validated: dict[
            str,
            dict[str, Any],
        ] = {}

        for target_id, refinement in refinements.items():

            if not isinstance(target_id, str):
                raise TypeError(
                    "Nut refinement target IDs must be strings."
                )

            target_id = target_id.strip()

            if not target_id:
                raise ValueError(
                    "Nut refinement target ID cannot be empty."
                )

            if not isinstance(refinement, dict):
                raise TypeError(
                    f"Nut refinement for {target_id!r} "
                    "must be a dictionary."
                )

            if refinement.get("accepted") is not True:
                raise ValueError(
                    "Motion Layer received a rejected peg-axis "
                    f"refinement for {target_id!r}."
                )

            axis_xy = np.asarray(
                refinement.get(
                    "axis_xy_base"
                ),
                dtype=np.float64,
            )

            axis_direction = np.asarray(
                refinement.get(
                    "axis_direction_base"
                ),
                dtype=np.float64,
            )

            tip_point = np.asarray(
                refinement.get(
                    "peg_tip_point_base"
                ),
                dtype=np.float64,
            )

            support_point = np.asarray(
                refinement.get(
                    "peg_support_point_base"
                ),
                dtype=np.float64,
            )

            if (
                axis_xy.shape != (2,)
                or not np.isfinite(
                    axis_xy
                ).all()
            ):
                raise ValueError(
                    "Invalid axis_xy_base for "
                    f"{target_id!r}."
                )

            if (
                axis_direction.shape != (3,)
                or not np.isfinite(
                    axis_direction
                ).all()
            ):
                raise ValueError(
                    "Invalid axis_direction_base for "
                    f"{target_id!r}."
                )

            axis_norm = float(
                np.linalg.norm(
                    axis_direction
                )
            )

            if axis_norm <= 1e-8:
                raise ValueError(
                    "Peg axis has near-zero norm for "
                    f"{target_id!r}."
                )

            axis_direction = (
                axis_direction
                / axis_norm
            )

            if (
                tip_point.shape != (3,)
                or not np.isfinite(
                    tip_point
                ).all()
            ):
                raise ValueError(
                    "Invalid peg_tip_point_base for "
                    f"{target_id!r}."
                )

            if (
                support_point.shape != (3,)
                or not np.isfinite(
                    support_point
                ).all()
            ):
                raise ValueError(
                    "Invalid peg_support_point_base for "
                    f"{target_id!r}."
                )

            measured_height = float(
                np.dot(
                    tip_point - support_point,
                    axis_direction,
                )
            )

            if measured_height <= 0.01:
                raise ValueError(
                    "Invalid peg tip/support ordering for "
                    f"{target_id!r}."
                )

            normalized_target = (
                target_id.casefold()
            )

            if normalized_target in validated:
                raise ValueError(
                    "Duplicate Nut-Assembly refinement target "
                    f"after normalization: {target_id!r}."
                )

            validated[normalized_target] = {
                "target_id": target_id,

                "axis_xy_base": (
                    axis_xy.copy()
                ),

                "axis_direction_base": (
                    axis_direction.copy()
                ),

                "peg_tip_point_base": (
                    tip_point.copy()
                ),

                "peg_support_point_base": (
                    support_point.copy()
                ),

                "peg_height_estimate_m": (
                    measured_height
                ),
            }

        self._nut_place_refinements = (
            validated
        )

        loaded_targets = [
            item["target_id"]
            for item in validated.values()
        ]

        print(
            "[NUT] Loaded refined place geometry: "
            f"{loaded_targets}",
            flush=True,
        )

    def translate(
        self,
        primitive_step: PrimitiveStep,
        scene_state: SceneState,
    ) -> list[np.ndarray]:
        """Translate one SeeDo primitive to an action sequence."""
        if self.current_pose is None:
            raise RuntimeError(
                "SeeDoMotionLayer has not been initialized. Call reset() first."
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
            actions = self._reach(primitive_step, scene_state)
        elif primitive_name == "approaching":
            actions = self._approaching(primitive_step, scene_state)
        elif primitive_name == "pick":
            actions = self._pick(primitive_step, scene_state)
        elif primitive_name == "lift_up":
            actions = self._lift_up(primitive_step, scene_state)
        elif primitive_name == "moving":
            actions = self._moving(primitive_step, scene_state)
        elif primitive_name == "placing":
            actions = self._placing(primitive_step, scene_state)
        elif primitive_name == "aligning":
            actions = self._aligning(primitive_step, scene_state)
        elif primitive_name == "inserting":
            actions = self._inserting(primitive_step, scene_state)
        else:
            raise ValueError(f"Unsupported SeeDo primitive: {primitive_name!r}.")

        self._record_translation(primitive_step=primitive_step, actions=actions)
        return actions

    def _reach(
        self,
        primitive_step: PrimitiveStep,
        scene_state: SceneState,
    ) -> list[np.ndarray]:
        target = self._resolve_target(primitive_step, scene_state)
        target_position = np.asarray(target.position_base, dtype=np.float64).copy()
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
        target = self._resolve_target(primitive_step, scene_state)

        if self.grasp_planning_enabled:
            if (
                self.active_grasp_position is None
                or self.active_grasp_orientation is None
                or self.active_approach_direction is None
            ):
                raise RuntimeError(
                    "Grasp planning is enabled but the active grasp "
                    "pose or approach direction is not available for the "
                    "approaching primitive."
                )
            if self.active_grasp_backend == "graspgenx":
                # GraspGen-X returns the final tcp_link pose; no M2T2 offsets.
                return self._move_linear(
                    target_position=self.active_grasp_position,
                    target_orientation=self.active_grasp_orientation,
                )
            final_grasp_position = (
                self.active_grasp_position
                + self.grasp_depth_offset * self.active_approach_direction
            )
            pregrasp_position = (
                final_grasp_position
                - self.pregrasp_distance * self.active_approach_direction
            )
            pregrasp_actions = self._move_linear(
                target_position=pregrasp_position,
                target_orientation=self.active_grasp_orientation,
            )
            final_approach_actions = self._move_linear(
                target_position=final_grasp_position,
                target_orientation=self.active_grasp_orientation,
            )
            return pregrasp_actions + final_approach_actions

        target_position = np.asarray(target.position_base, dtype=np.float64).copy()
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
        self._resolve_target(primitive_step, scene_state)
        self.current_gripper_position = self.gripper_closed_position
        actions = [
            self._build_action(
                position=self.current_pose.position,
                orientation=self.current_pose.orientation,
                gripper_position=self.current_gripper_position,
            )
        ]

        # Original M2T2-only post-pick retreat; GraspGen-X unchanged.
        if self.grasp_planning_enabled and self.active_grasp_backend != "graspgenx":
            if self.active_approach_direction is None:
                raise RuntimeError(
                    "Grasp planning is enabled but no active "
                    "approach direction is available for retreat."
                )
            retreat_position = (
                self.current_pose.position
                - self.retreat_distance * self.active_approach_direction
            )
            retreat_actions = self._move_linear(
                target_position=retreat_position,
                target_orientation=self.current_pose.orientation,
            )
            actions.extend(retreat_actions)
        return actions

    # ------------------------------------------------------------------
    # Nut-Assembly geometry helpers (used only for the assembly task).
    # ------------------------------------------------------------------
    def _nut_config(self) -> dict[str, Any]:
        if self._nut_cfg is None:
            raise RuntimeError("Nut-Assembly configuration not initialized")
        return self._nut_cfg


    def _nut_rotate_at_fixed_tcp(
        self,
        target_orientation,
    ) -> list[np.ndarray]:
        """
        Rotate the TCP at fixed position using bounded angular increments.
        """

        cfg = self._nut_config()

        start_q = np.asarray(
            self.current_pose.orientation,
            dtype=np.float64,
        ).copy()

        end_q = np.asarray(
            target_orientation,
            dtype=np.float64,
        ).copy()

        start_q /= np.linalg.norm(
            start_q
        )

        end_q /= np.linalg.norm(
            end_q
        )

        delta = (
            Rotation.from_quat(start_q).inv()
            * Rotation.from_quat(end_q)
        )

        steps = max(
            1,
            int(
                np.ceil(
                    np.degrees(
                        delta.magnitude()
                    )
                    / cfg["rotation_step_deg"]
                )
            ),
        )

        slerp = Slerp(
            [0.0, 1.0],
            Rotation.from_quat(
                [
                    start_q,
                    end_q,
                ]
            ),
        )

        position = np.asarray(
            self.current_pose.position,
            dtype=np.float64,
        ).copy()

        actions = []

        for part in np.linspace(
            1.0 / steps,
            1.0,
            steps,
        ):

            quaternion = (
                slerp(
                    [float(part)]
                )
                .as_quat()[0]
            )

            actions.append(
                self._build_action(
                    position=position,
                    orientation=quaternion,
                    gripper_position=(
                        self.current_gripper_position
                    ),
                )
            )

        self.current_pose = GripperPose(
            position,
            end_q,
        )

        return actions

    def _nut_move_linear(self, goal, orientation, max_step_m: float | None = None):
        """Generate at most max_step_m Cartesian increments, ONLY for nut.

        The shared build_linear_waypoints() uses floor(distance / min_step),
        so setting min_step=5mm there would NOT ensure a 5mm upper bound.
        Instead, split into ceil(distance / max_step_m) submoves. The
        original waypoint generator, and therefore Pick-and-Place, is unchanged.
        """
        goal = np.asarray(goal, dtype=np.float64)
        orientation = np.asarray(orientation, dtype=np.float64)
        if max_step_m is None:
            return self._move_linear(
                target_position=goal,
                target_orientation=orientation,
            )
        step = float(max_step_m)
        if not np.isfinite(step) or step <= 0:
            raise ValueError("Invalid Nut-Assembly waypoint step")
        start = np.asarray(self.current_pose.position, dtype=np.float64).copy()
        delta = goal - start
        steps = max(1, int(np.ceil(np.linalg.norm(delta) / step)))
        actions = []
        for i in range(1, steps + 1):
            next_pos = start + (i / steps) * delta
            actions.extend(
                self._move_linear(
                    target_position=next_pos,
                    target_orientation=orientation,
                )
            )
        return actions


    def _lift_up(
        self,
        primitive_step: PrimitiveStep,
        scene_state: SceneState,
    ) -> list[np.ndarray]:
        if self.task_type == "nut_assembly":

            self._resolve_target(
                primitive_step,
                scene_state,
            )

            cfg = self._nut_config()

            if not np.isclose(
                self.current_gripper_position,
                self.gripper_closed_position,
                atol=1e-6,
            ):
                raise RuntimeError(
                    "Cannot lift nut before closing the gripper."
                )

            if self._nut_hole_bottom_local is None:
                raise RuntimeError(
                    "Nut grasp geometry is unavailable before LIFT_UP."
                )

            if not self._nut_place_refinements:
                raise RuntimeError(
                    "No refined assembly destinations are available."
                )

            highest_peg_tip_z = max(
                float(
                    geometry[
                        "peg_tip_point_base"
                    ][2]
                )
                for geometry
                in self._nut_place_refinements.values()
            )

            current_rotation = Rotation.from_quat(
                self.current_pose.orientation
            )

            hole_bottom_offset = current_rotation.apply(
                self._nut_hole_bottom_local
            )

            required_clearance_m = (
                cfg["peg_clearance_m"]
                + cfg[
                    "transport_clearance_margin_m"
                ]
            )

            safe_tcp_z = (
                highest_peg_tip_z
                + required_clearance_m
                - float(
                    hole_bottom_offset[2]
                )
            )

            goal = np.asarray(
                self.current_pose.position,
                dtype=np.float64,
            ).copy()

            # Keep the normal lift policy as a minimum,
            # but automatically increase it when scene
            # geometry requires more clearance.
            goal[2] = max(
                float(
                    self.current_pose.position[2]
                    + self.lift_height
                ),
                float(safe_tcp_z),
            )

            actions = self._move_linear(
                target_position=goal,
                target_orientation=(
                    self.current_pose.orientation
                ),
            )

            self._nut_phase = "lifted"

            print(
                "[NUT] LIFT_UP | "
                f"TCP.z={goal[2]:.4f} | "
                f"highest_peg_tip_z="
                f"{highest_peg_tip_z:.4f} | "
                f"hole_bottom_offset_z="
                f"{hole_bottom_offset[2]:.4f} | "
                f"peg_clearance="
                f"{cfg['peg_clearance_m'] * 1000.0:.1f} mm",
                f"safe_tcp_z={safe_tcp_z:.4f}",
                f"required_clearance="
                f"{required_clearance_m * 1000.0:.1f} mm",
                flush=True,
            )

            return actions

        # Original Pick-and-Place behavior, unchanged.
        self._resolve_target(primitive_step, scene_state)
        target_position = np.array(
            [
                self.current_pose.position[0],
                self.current_pose.position[1],
                self.current_pose.position[2] + self.lift_height,
            ],
            dtype=np.float64,
        )
        return self._move_linear(
            target_position=target_position,
            target_orientation=self.current_pose.orientation,
        )

    def _moving(
        self,
        primitive_step: PrimitiveStep,
        scene_state: SceneState,
    ) -> list[np.ndarray]:
        if self.task_type == "nut_assembly":

            if self._nut_phase != "lifted":
                raise RuntimeError(
                    "Nut MOVING requires a preceding LIFT_UP."
                )

            cfg = self._nut_config()

            (
                peg_target,
                peg_geometry,
            ) = self._nut_refined_peg_geometry(
                primitive_step,
                scene_state,
            )

            peg_axis = np.asarray(
                peg_geometry[
                    "axis_direction_base"
                ],
                dtype=np.float64,
            )

            peg_axis /= np.linalg.norm(
                peg_axis
            )

            peg_tip = np.asarray(
                peg_geometry[
                    "peg_tip_point_base"
                ],
                dtype=np.float64,
            )

            if abs(float(peg_axis[2])) < 0.20:
                raise RuntimeError(
                    "Refined peg axis is too horizontal for "
                    "the current tabletop assembly strategy."
                )

            # First rotate at the safe lifted TCP position.
            target_orientation = (
                self._nut_alignment_orientation(
                    peg_axis
                )
            )

            actions = (
                self._nut_rotate_at_fixed_tcp(
                    target_orientation
                )
            )

            # Hole-bottom offset after the final rotation.
            rotation = Rotation.from_quat(
                self.current_pose.orientation
            )

            hole_bottom_offset = (
                rotation.apply(
                    self._nut_hole_bottom_local
                )
            )

            # Put the hole center on the peg axis while
            # preserving the current safe TCP Z.
            #
            # tcp = peg_tip + lambda * axis - hole_offset
            #
            # Solve lambda from tcp_z = current_tcp_z.
            lambda_m = float(
                (
                    self.current_pose.position[2]
                    + hole_bottom_offset[2]
                    - peg_tip[2]
                )
                / peg_axis[2]
            )

            required_clearance_m = (
                cfg["peg_clearance_m"]
                + cfg[
                    "transport_clearance_margin_m"
                ]
            )

            if lambda_m < required_clearance_m:

                old_lambda_m = float(
                    lambda_m
                )

                required_tcp_z = (
                    peg_tip[2]
                    + required_clearance_m
                    * peg_axis[2]
                    - hole_bottom_offset[2]
                )

                corrective_goal = np.asarray(
                    self.current_pose.position,
                    dtype=np.float64,
                ).copy()

                corrective_goal[2] = max(
                    float(
                        self.current_pose.position[2]
                    ),
                    float(
                        required_tcp_z
                    ),
                )

                print(
                    "[NUT] MOVING clearance below target | "
                    f"current="
                    f"{old_lambda_m * 1000.0:.1f} mm | "
                    f"required="
                    f"{required_clearance_m * 1000.0:.1f} mm | "
                    "performing corrective vertical lift | "
                    f"TCP.z "
                    f"{self.current_pose.position[2]:.4f} "
                    f"-> {corrective_goal[2]:.4f}",
                    flush=True,
                )

                actions.extend(
                    self._move_linear(
                        target_position=(
                            corrective_goal
                        ),
                        target_orientation=(
                            self.current_pose.orientation
                        ),
                    )
                )

                # Recompute the effective clearance after
                # the corrective vertical motion.
                lambda_m = float(
                    (
                        self.current_pose.position[2]
                        + hole_bottom_offset[2]
                        - peg_tip[2]
                    )
                    / peg_axis[2]
                )

                if (
                    lambda_m
                    < required_clearance_m
                    - 1e-6
                ):
                    raise RuntimeError(
                        "Nut MOVING corrective lift failed "
                        "to establish the required clearance: "
                        f"current={lambda_m:.4f} m, "
                        f"required="
                        f"{required_clearance_m:.4f} m."
                    )

                print(
                    "[NUT] MOVING corrective lift complete | "
                    f"axis_clearance="
                    f"{lambda_m * 1000.0:.1f} mm",
                    flush=True,
                )

            desired_hole_bottom = (
                peg_tip
                + lambda_m
                * peg_axis
            )

            target_position = (
                desired_hole_bottom
                - hole_bottom_offset
            )

            actions.extend(
                self._move_linear(
                    target_position=target_position,
                    target_orientation=(
                        self.current_pose.orientation
                    ),
                )
            )

            self._nut_peg_target = (
                peg_target.casefold()
            )

            self._nut_phase = "moved"

            print(
                "[NUT] MOVING | "
                f"target={peg_target!r} | "
                f"transport_clearance="
                f"{lambda_m:.4f} m | "
                f"TCP={target_position}",
                flush=True,
            )

            return actions

        # Original Pick-and-Place behavior, unchanged.
        target = self._resolve_target(primitive_step, scene_state)
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
            target_orientation=self.current_pose.orientation,
        )

    def _placing(
        self,
        primitive_step: PrimitiveStep,
        scene_state: SceneState,
    ) -> list[np.ndarray]:
        # Pick-and-Place ONLY: this body is unchanged.
        target = self._resolve_target(primitive_step, scene_state)
        target_position = np.array(
            [
                target.position_base[0],
                target.position_base[1],
                target.position_base[2] + self.release_height_offset,
            ],
            dtype=np.float64,
        )
        actions = self._move_linear(
            target_position=target_position,
            target_orientation=self.current_pose.orientation,
        )
        self.current_gripper_position = self.gripper_open_position
        actions.append(
            self._build_action(
                position=self.current_pose.position,
                orientation=self.current_pose.orientation,
                gripper_position=self.current_gripper_position,
            )
        )
        return actions

    def _aligning(
        self,
        primitive_step: PrimitiveStep,
        scene_state: SceneState,
    ) -> list[np.ndarray]:
        """
        Nut-Assembly only: descend until the lower hole opening
        is peg_clearance_m above the detected peg tip.
        """

        if self.task_type != "nut_assembly":
            raise RuntimeError(
                "ALIGNING is only valid for Nut-Assembly."
            )

        if self._nut_phase != "moved":
            raise RuntimeError(
                "Nut ALIGNING requires a preceding MOVING."
            )

        cfg = self._nut_config()

        (
            peg_target,
            peg_geometry,
        ) = self._nut_refined_peg_geometry(
            primitive_step,
            scene_state,
        )

        if (
            self._nut_peg_target is None
            or peg_target.casefold()
            != self._nut_peg_target
        ):
            raise RuntimeError(
                "Nut destination changed between "
                "MOVING and ALIGNING."
            )

        peg_axis = np.asarray(
            peg_geometry[
                "axis_direction_base"
            ],
            dtype=np.float64,
        )

        peg_axis /= np.linalg.norm(
            peg_axis
        )

        peg_tip = np.asarray(
            peg_geometry[
                "peg_tip_point_base"
            ],
            dtype=np.float64,
        )

        (
            _,
            _,
            current_hole_axis,
        ) = self._nut_current_hole_geometry()

        axis_alignment = float(
            np.dot(
                current_hole_axis,
                peg_axis,
            )
        )

        if axis_alignment < 0.999:
            raise RuntimeError(
                "Hole and peg axes are not sufficiently "
                "aligned before descent: "
                f"dot={axis_alignment:.6f}."
            )

        desired_hole_bottom = (
            peg_tip
            + cfg["peg_clearance_m"]
            * peg_axis
        )

        rotation = Rotation.from_quat(
            self.current_pose.orientation
        )

        hole_bottom_offset = (
            rotation.apply(
                self._nut_hole_bottom_local
            )
        )

        target_position = (
            desired_hole_bottom
            - hole_bottom_offset
        )

        actions = self._nut_move_linear(
            target_position,
            self.current_pose.orientation,
            max_step_m=(
                cfg["insertion_step_m"]
            ),
        )

        self._nut_phase = "aligned"

        print(
            "[NUT] ALIGNING | "
            f"target={peg_target!r} | "
            f"hole_bottom={desired_hole_bottom} | "
            f"clearance="
            f"{cfg['peg_clearance_m'] * 1000.0:.1f} mm",
            flush=True,
        )

        return actions

    def _inserting(
        self,
        primitive_step: PrimitiveStep,
        scene_state: SceneState,
    ) -> list[np.ndarray]:
        """
        Nut-Assembly only.

        Insert along the refined peg axis using the lower hole opening
        as the geometric reference.
        """

        if self.task_type != "nut_assembly":
            raise RuntimeError(
                "INSERTING is only valid for Nut-Assembly."
            )

        if self._nut_phase != "aligned":
            raise RuntimeError(
                "Nut INSERTING requires a preceding ALIGNING."
            )

        cfg = self._nut_config()

        (
            peg_target,
            peg_geometry,
        ) = self._nut_refined_peg_geometry(
            primitive_step,
            scene_state,
        )

        if (
            self._nut_peg_target is None
            or peg_target.casefold()
            != self._nut_peg_target
        ):
            raise RuntimeError(
                "Nut destination changed during INSERTING."
            )

        if not np.isclose(
            self.current_gripper_position,
            self.gripper_closed_position,
            atol=1e-6,
        ):
            raise RuntimeError(
                "Nut gripper is open before insertion."
            )

        peg_axis = np.asarray(
            peg_geometry[
                "axis_direction_base"
            ],
            dtype=np.float64,
        )

        peg_axis /= np.linalg.norm(
            peg_axis
        )

        peg_tip = np.asarray(
            peg_geometry[
                "peg_tip_point_base"
            ],
            dtype=np.float64,
        )

        peg_support = np.asarray(
            peg_geometry[
                "peg_support_point_base"
            ],
            dtype=np.float64,
        )

        (
            _,
            _,
            current_hole_axis,
        ) = self._nut_current_hole_geometry()

        axis_alignment = float(
            np.dot(
                current_hole_axis,
                peg_axis,
            )
        )

        if axis_alignment < 0.999:
            raise RuntimeError(
                "Hole and peg axes are not aligned "
                "before insertion."
            )

        # Stage 1:
        # let the peg enter the lower hole opening slightly.
        first_hole_bottom = (
            peg_tip
            - cfg["initial_entry_m"]
            * peg_axis
        )

        # Stage 2:
        # move the lower face of the ring close to the
        # locally observed support surface.
        final_hole_bottom = (
            peg_support
            + cfg["release_bottom_height_m"]
            * peg_axis
        )

        first_axis_coordinate = float(
            np.dot(
                first_hole_bottom
                - peg_support,
                peg_axis,
            )
        )

        final_axis_coordinate = float(
            np.dot(
                final_hole_bottom
                - peg_support,
                peg_axis,
            )
        )

        if (
            final_axis_coordinate
            >= first_axis_coordinate
        ):
            raise RuntimeError(
                "Final insertion point must lie below "
                "the initial-entry point along the peg axis."
            )

        rotation = Rotation.from_quat(
            self.current_pose.orientation
        )

        hole_bottom_offset = (
            rotation.apply(
                self._nut_hole_bottom_local
            )
        )

        first_tcp_goal = (
            first_hole_bottom
            - hole_bottom_offset
        )

        final_tcp_goal = (
            final_hole_bottom
            - hole_bottom_offset
        )

        # ---------------------------------------------------------
        # Stage 1: precise engagement of the peg tip.
        #
        # Keep the Cartesian increments small while the peg is
        # entering the lower hole opening.
        # ---------------------------------------------------------

        actions = self._nut_move_linear(
            first_tcp_goal,
            self.current_pose.orientation,
            max_step_m=(
                cfg["initial_entry_step_m"]
            ),
        )

        # ---------------------------------------------------------
        # Stage 2: slide along the already-engaged peg.
        #
        # Once the peg is inside the hole, larger increments reduce
        # repeated independent GoToPose replanning and therefore
        # produce a cleaner downward motion.
        # ---------------------------------------------------------

        actions.extend(
            self._nut_move_linear(
                final_tcp_goal,
                self.current_pose.orientation,
                max_step_m=(
                    cfg["insertion_slide_step_m"]
                ),
            )
        )

        if cfg["auto_release"]:

            self.current_gripper_position = (
                self.gripper_open_position
            )

            actions.append(
                self._build_action(
                    position=(
                        self.current_pose.position
                    ),
                    orientation=(
                        self.current_pose.orientation
                    ),
                    gripper_position=(
                        self.current_gripper_position
                    ),
                )
            )

        self._nut_phase = "inserted"

        print(
            "[NUT] INSERTING | "
            f"target={peg_target!r} | "
            f"initial_entry="
            f"{cfg['initial_entry_m'] * 1000.0:.1f} mm | "
            f"final_bottom_height="
            f"{cfg['release_bottom_height_m'] * 1000.0:.1f} mm | "
            f"release="
            f"{'yes' if cfg['auto_release'] else 'no'}",
            f"entry_step=",
            f"{cfg['initial_entry_step_m'] * 1000.0:.1f} mm | ",
            f"slide_step=",
            f"{cfg['insertion_slide_step_m'] * 1000.0:.1f} mm | ",
            flush=True,
        )

        return actions

    def _move_linear(
        self,
        target_position,
        target_orientation,
    ) -> list[np.ndarray]:
        if self.current_pose is None:
            raise RuntimeError("Cannot generate motion without a current pose.")
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
                gripper_position=self.current_gripper_position,
            )
            for waypoint in waypoints
        ]
        if waypoints:
            self.current_pose = waypoints[-1]
        return actions

    @staticmethod
    def _build_action(position, orientation, gripper_position: float) -> np.ndarray:
        position = np.asarray(position, dtype=np.float64)
        orientation = np.asarray(orientation, dtype=np.float64)
        return np.concatenate(
            [position, orientation, np.array([gripper_position], dtype=np.float64)]
        )

    def _record_translation(
        self,
        primitive_step: PrimitiveStep,
        actions: list[np.ndarray],
    ) -> None:
        self.motion_history.append(
            {
                "name": primitive_step.name,
                "arguments": dict(primitive_step.arguments),
                "actions": [
                    {
                        "position": action[:3].tolist(),
                        "orientation": action[3:7].tolist(),
                        "gripper_position": float(action[7]),
                    }
                    for action in actions
                ],
            }
        )
        self._write_motion_artifact()

    def _write_motion_artifact(self) -> None:
        if self.artifacts_dir is None:
            return
        total_actions = sum(
            len(primitive["actions"]) for primitive in self.motion_history
        )
        final_state = None
        if self.current_pose is not None:
            final_state = {
                "position": np.asarray(
                    self.current_pose.position, dtype=np.float64
                ).tolist(),
                "orientation": np.asarray(
                    self.current_pose.orientation, dtype=np.float64
                ).tolist(),
                "gripper_position": float(self.current_gripper_position),
            }
        payload = {
            "primitives": self.motion_history,
            "total_actions": total_actions,
            "final_planned_state": final_state,
        }
        output_path = self.artifacts_dir / "motion_plan.json"
        with output_path.open("w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2)

    def _nut_alignment_orientation(
        self,
        peg_axis_base,
    ) -> np.ndarray:
        """
        Rotate the TCP by the minimum amount required to align
        the grasped hole axis with the destination peg axis.
        """

        if self.current_pose is None:
            raise RuntimeError(
                "Current TCP pose is unavailable."
            )

        (
            _,
            _,
            current_hole_axis,
        ) = self._nut_current_hole_geometry()

        peg_axis = np.asarray(
            peg_axis_base,
            dtype=np.float64,
        )

        peg_axis /= np.linalg.norm(
            peg_axis
        )

        delta_rotation = (
            self._rotation_between_vectors(
                current_hole_axis,
                peg_axis,
            )
        )

        current_rotation = Rotation.from_quat(
            self.current_pose.orientation
        )

        target_rotation = (
            delta_rotation
            * current_rotation
        )

        return target_rotation.as_quat()

    @staticmethod
    def _rotation_between_vectors(
        source,
        target,
    ) -> Rotation:
        """
        Return the minimum rotation mapping source onto target.
        """

        source = np.asarray(
            source,
            dtype=np.float64,
        )

        target = np.asarray(
            target,
            dtype=np.float64,
        )

        source_norm = float(
            np.linalg.norm(source)
        )

        target_norm = float(
            np.linalg.norm(target)
        )

        if (
            source_norm <= 1e-8
            or target_norm <= 1e-8
        ):
            raise ValueError(
                "Cannot align zero-length vectors."
            )

        source /= source_norm
        target /= target_norm

        dot = float(
            np.clip(
                np.dot(source, target),
                -1.0,
                1.0,
            )
        )

        if dot >= 1.0 - 1e-10:
            return Rotation.identity()

        if dot <= -1.0 + 1e-10:

            reference = np.array(
                [1.0, 0.0, 0.0],
                dtype=np.float64,
            )

            if abs(
                np.dot(
                    source,
                    reference,
                )
            ) > 0.9:
                reference = np.array(
                    [0.0, 1.0, 0.0],
                    dtype=np.float64,
                )

            axis = np.cross(
                source,
                reference,
            )

            axis /= np.linalg.norm(
                axis
            )

            return Rotation.from_rotvec(
                np.pi * axis
            )

        cross = np.cross(
            source,
            target,
        )

        sin_angle = float(
            np.linalg.norm(
                cross
            )
        )

        axis = (
            cross
            / sin_angle
        )

        angle = float(
            np.arctan2(
                sin_angle,
                dot,
            )
        )

        return Rotation.from_rotvec(
            angle * axis
        )

    def _nut_current_hole_geometry(
        self,
    ) -> tuple[
        np.ndarray,
        np.ndarray,
        np.ndarray,
    ]:
        """
        Return current hole top center, bottom center and axis in base_link.
        """

        if self.current_pose is None:
            raise RuntimeError(
                "Current TCP pose is unavailable."
            )

        if (
            self._nut_hole_top_local is None
            or self._nut_hole_bottom_local is None
            or self._nut_hole_axis_local is None
        ):
            raise RuntimeError(
                "Nut grasp-relative hole geometry "
                "has not been initialized."
            )

        rotation = Rotation.from_quat(
            self.current_pose.orientation
        )

        position = np.asarray(
            self.current_pose.position,
            dtype=np.float64,
        )

        hole_top = (
            position
            + rotation.apply(
                self._nut_hole_top_local
            )
        )

        hole_bottom = (
            position
            + rotation.apply(
                self._nut_hole_bottom_local
            )
        )

        hole_axis = rotation.apply(
            self._nut_hole_axis_local
        )

        hole_axis /= np.linalg.norm(
            hole_axis
        )

        return (
            hole_top,
            hole_bottom,
            hole_axis,
        )

    def _resolve_nut_pick_refinement(
        self,
        target: str,
    ) -> dict[str, Any]:
        """
        Return the frozen hole refinement associated with the
        runtime object actually being grasped.
        """

        normalized_target = (
            str(target)
            .strip()
            .casefold()
        )

        matches = [
            refinement
            for object_id, refinement
            in self.nut_pick_refinements.items()
            if (
                str(object_id)
                .strip()
                .casefold()
                == normalized_target
            )
        ]

        if len(matches) != 1:
            raise RuntimeError(
                "Could not uniquely resolve Nut-Assembly "
                f"pick refinement for {target!r}. "
                f"Matches: {len(matches)}."
            )

        return matches[0]

    def _resolve_target(
        self,
        primitive_step: PrimitiveStep,
        scene_state: SceneState,
    ) -> SceneObject:
        target_name = self._extract_target_name(primitive_step.arguments)
        normalized_target = target_name.strip().lower()
        matches = [
            obj
            for obj in scene_state.objects
            if obj.object_id.strip().lower() == normalized_target
        ]
        if not matches:
            available_objects = [obj.object_id for obj in scene_state.objects]
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
    def _extract_target_name(arguments: dict[str, Any]) -> str:
        if "target" in arguments:
            target = arguments["target"]
        elif "object" in arguments:
            target = arguments["object"]
        elif "destination" in arguments:
            target = arguments["destination"]
        else:
            raise ValueError(
                "PrimitiveStep does not contain a recognized target argument. "
                "Expected one of: 'target', 'object', 'destination'."
            )
        if not isinstance(target, str):
            raise TypeError(
                "Primitive target must be a string, "
                f"received {type(target).__name__}."
            )
        target = target.strip()
        if not target:
            raise ValueError("Primitive target cannot be empty.")
        return target
