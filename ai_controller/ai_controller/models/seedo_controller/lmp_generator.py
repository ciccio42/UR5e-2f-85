from __future__ import annotations

import numpy as np
import copy
import shapely
from pathlib import Path
import json

from dataclasses import asdict
from typing import Any
from openai import OpenAI
from shapely.geometry import *
from shapely.affinity import *

from results import (
    ActionStep,
    ActionPlanningResult,
    PrimitivePlan,
    PrimitiveStep,
    SceneState,
)
from VLM_CaP.src.LMP import (
    LMP,
    LMPFGen,
)
from VLM_CaP.src.configs import (
    cfg_tabletop,
)
from ai_controller.models.seedo_controller.lmp_prompts import (
    prompt_parse_obj_name_ros,
    prompt_tabletop_ui_ros,
)

class LMPSceneWrapper:
    """Environment adapter exposed to SeeDo's Code-as-Policies LMPs.

    This wrapper replaces the simulation-specific LMP_wrapper used by
    SeeDo. It exposes information from the frozen SceneState and records
    robot primitive calls instead of executing them immediately.
    """

    def __init__(
        self,
        scene_state: SceneState,
        workspace_bottom_left: tuple[float, float],
        workspace_top_right: tuple[float, float],
    ) -> None:
        self.scene_state = scene_state
        self.primitive_steps: list[PrimitiveStep] = []

        self._min_xy = np.asarray(
            workspace_bottom_left,
            dtype=np.float64,
        )

        self._max_xy = np.asarray(
            workspace_top_right,
            dtype=np.float64,
        )

        if self._min_xy.shape != (2,):
            raise ValueError(
                "workspace_bottom_left must contain exactly two coordinates."
            )

        if self._max_xy.shape != (2,):
            raise ValueError(
                "workspace_top_right must contain exactly two coordinates."
            )

        if np.any(self._max_xy <= self._min_xy):
            raise ValueError(
                "workspace_top_right must be greater than "
                "workspace_bottom_left on both axes."
            )

        self._range_xy = (
            self._max_xy
            - self._min_xy
        )

        self._objects_by_id = {
            obj.object_id: obj
            for obj in scene_state.objects
        }

        self._objects_by_label: dict[str, list[Any]] = {}

        for obj in scene_state.objects:
            self._objects_by_label.setdefault(
                obj.label,
                [],
            ).append(obj)

    def get_obj_names(self) -> list[str]:
        """Return all object identifiers visible in the initial scene."""

        return list(self._objects_by_id.keys())

    def is_obj_visible(
        self,
        obj_name: str,
    ) -> bool:
        """Return whether an object exists in the frozen SceneState."""

        return obj_name in self._objects_by_id

    def get_obj_pos(
        self,
        obj_name: str,
    ) -> np.ndarray:
        """Return the object's initial XY position in base_link.

        SeeDo's original Code-as-Policies implementation performs
        spatial reasoning on the tabletop XY plane.
        """

        if obj_name not in self._objects_by_id:
            raise KeyError(
                f"Unknown object: {obj_name}"
            )

        obj = self._objects_by_id[obj_name]

        return np.asarray(
            obj.position_base[:2],
            dtype=np.float64,
        )
    
    def get_obj_pos_3d(
        self,
        obj_name: str,
    ) -> np.ndarray:
        """Return the object's full XYZ position in base_link."""

        if obj_name not in self._objects_by_id:
            raise KeyError(
                f"Unknown object: {obj_name}"
            )

        obj = self._objects_by_id[obj_name]

        return np.asarray(
            obj.position_base,
            dtype=np.float64,
        )

    def denormalize_xy(
        self,
        pos_normalized: Any,
    ) -> np.ndarray:
        pos_normalized = np.asarray(
            pos_normalized,
            dtype=np.float64,
        )

        if pos_normalized.shape != (2,):
            raise ValueError(
                "Normalized position must contain exactly two coordinates."
            )

        return (
            pos_normalized * self._range_xy
            + self._min_xy
        )

    def get_corner_positions(
        self,
    ) -> np.ndarray:
        normalized_corners = np.asarray(
            [
                [0.0, 1.0],
                [1.0, 1.0],
                [0.0, 0.0],
                [1.0, 0.0],
            ],
            dtype=np.float64,
        )

        return np.asarray(
            [
                self.denormalize_xy(position)
                for position in normalized_corners
            ]
        )

    def get_side_positions(
        self,
    ) -> np.ndarray:
        normalized_sides = np.asarray(
            [
                [0.5, 1.0],
                [1.0, 0.5],
                [0.5, 0.0],
                [0.0, 0.5],
            ],
            dtype=np.float64,
        )

        return np.asarray(
            [
                self.denormalize_xy(position)
                for position in normalized_sides
            ]
        )

    def get_corner_name(
        self,
        pos: Any,
    ) -> str:
        pos = np.asarray(
            pos,
            dtype=np.float64,
        )

        corner_positions = (
            self.get_corner_positions()
        )

        corner_idx = int(
            np.argmin(
                np.linalg.norm(
                    corner_positions - pos,
                    axis=1,
                )
            )
        )

        return [
            "top left corner",
            "top right corner",
            "bottom left corner",
            "bottom right corner",
        ][corner_idx]

    def get_side_name(
        self,
        pos: Any,
    ) -> str:
        pos = np.asarray(
            pos,
            dtype=np.float64,
        )

        side_positions = (
            self.get_side_positions()
        )

        side_idx = int(
            np.argmin(
                np.linalg.norm(
                    side_positions - pos,
                    axis=1,
                )
            )
        )

        return [
            "top side",
            "right side",
            "bottom side",
            "left side",
        ][side_idx]

    def get_obj_positions_np(
        self,
        obj_names: list[str],
    ) -> np.ndarray:
        """Return XY positions for a list of scene objects.

        This mirrors the helper used by SeeDo's original CAP prompts for
        spatial reasoning such as first/second from the left, closest, etc.
        """

        if not isinstance(obj_names, (list, tuple)):
            raise TypeError(
                "obj_names must be a list or tuple of object names."
            )

        if not obj_names:
            return np.empty(
                (0, 2),
                dtype=np.float64,
            )

        return np.asarray(
            [
                self.get_obj_pos(obj_name)
                for obj_name in obj_names
            ],
            dtype=np.float64,
        )

    def get_scene_object(
        self,
        obj_name: str,
    ):
        """Return the complete SceneObject associated with an object name."""

        if obj_name not in self._objects_by_id:
            raise KeyError(
                f"Unknown object: {obj_name}"
            )

        return self._objects_by_id[obj_name]

    def resolve_action_step(
        self,
        step: ActionStep,
    ) -> tuple[str, str]:
        """
        Resolve an ActionStep against the runtime SceneState.

        Returns:
            (picked_object_id, destination_object_id)
        """

        def normalize(value) -> str:
            return str(
                value if value is not None else ""
            ).strip().casefold()

        picked_label = normalize(
            step.picked_detector_label
        )

        destination_category = normalize(
            step.destination_category
        )

        if not picked_label:
            raise ValueError(
                "ActionStep is missing picked_detector_label. "
                "Regenerate the legacy action plan."
            )

        if not destination_category:
            raise ValueError(
                "ActionStep is missing destination_category."
            )

        # --------------------------------------------------
        # Resolve the object to pick using its detector label.
        # --------------------------------------------------

        pick_candidates = [
            obj
            for obj in self.scene_state.objects
            if normalize(obj.label) == picked_label
        ]

        if len(pick_candidates) != 1:
            raise ValueError(
                "Cannot uniquely resolve picked object: "
                f"label={picked_label!r}, "
                f"matches={[obj.object_id for obj in pick_candidates]}"
            )

        picked_object = pick_candidates[0]

        # --------------------------------------------------
        # Resolve the destination
        # --------------------------------------------------

        destination_candidates = [
            obj
            for obj in self.scene_state.objects
            if normalize(obj.category) == destination_category
        ]

        ordinal = step.destination_ordinal_from_left

        if not 1 <= ordinal <= len(destination_candidates):
            raise ValueError(
                "Invalid destination ordinal: "
                f"category={destination_category!r}, "
                f"ordinal={ordinal}, "
                f"available={len(destination_candidates)}"
            )

        # Canonical front-view ordering: image X, not base_link X.
        ordered_destinations = sorted(
            destination_candidates,
            key=lambda obj: obj.pixel_coordinates[0],
        )

        x_coordinates = [
            obj.pixel_coordinates[0]
            for obj in ordered_destinations
        ]

        if len(x_coordinates) != len(set(x_coordinates)):
            raise ValueError(
                "Ambiguous destination ordering: "
                "multiple objects share the same image X coordinate."
            )

        destination_object = ordered_destinations[
            ordinal - 1
        ]

        if picked_object.object_id == destination_object.object_id:
            raise ValueError(
                "Picked object and destination resolve "
                "to the same SceneObject."
            )

        return (
            picked_object.object_id,
            destination_object.object_id,
        )

    def _record_primitive(
        self,
        name: str,
        **arguments: Any,
    ) -> None:
        self.primitive_steps.append(
            PrimitiveStep(
                name=name,
                arguments=arguments,
            )
        )

    def reach(
        self,
        target: str,
    ) -> None:
        self._record_primitive(
            "reach",
            target=target,
        )

    def approaching(
        self,
        target: str,
    ) -> None:
        self._record_primitive(
            "approaching",
            target=target,
        )

    def pick(
        self,
        target: str,
    ) -> None:
        self._record_primitive(
            "pick",
            target=target,
        )

    def lift_up(
        self,
        target: str,
    ) -> None:
        self._record_primitive(
            "lift_up",
            target=target,
        )

    def moving(
        self,
        target: str,
    ) -> None:
        self._record_primitive(
            "moving",
            target=target,
        )

    def placing(
        self,
        target: str,
    ) -> None:
        self._record_primitive(
            "placing",
            target=target,
        )

class LMPGenerator:
    """Adapter between SeeDo's CAP implementation and the ROS2 controller.

    It reuses SeeDo's original LMP/LMPFGen machinery, replacing the
    simulation environment with a frozen SceneState and recording
    primitive calls into a PrimitivePlan.
    """

    def __init__(
        self,
        model: str = "gpt-3.5-turbo",
        perception_mode: str = "generalized",
    ) -> None:
        self.model = model
        self.client = OpenAI()

        self.perception_mode = str(
            perception_mode
        ).strip().lower()

        if self.perception_mode not in {
            "generalized",
            "prior_guided",
        }:
            raise ValueError(
                "Invalid perception_mode: "
                f"{self.perception_mode!r}"
            )

    def run(
        self,
        action_plan: ActionPlanningResult,
        scene_state: SceneState,
        workspace_bottom_left: tuple[float, float],
        workspace_top_right: tuple[float, float],
        artifacts_dir: Path | None = None,
    ) -> PrimitivePlan:
        if action_plan.status != "completed":
            raise ValueError(
                "CAP can only run on a completed SeeDo action plan."
            )

        if not action_plan.natural_language_plan.strip():
            raise ValueError(
                "The SeeDo natural-language plan is empty."
            )

        wrapper = LMPSceneWrapper(
            scene_state=scene_state,
            workspace_bottom_left=workspace_bottom_left,
            workspace_top_right=workspace_top_right,
        )

        lmp = self._setup_lmp(
            wrapper=wrapper,
            scene_state=scene_state,
        )

        context = (
            f"objects = {wrapper.get_obj_names()}"
        )

        if self.perception_mode == "generalized":

            resolved_actions = []

            for step in action_plan.steps:

                picked_id, destination_id = (
                    wrapper.resolve_action_step(step)
                )

                resolved_actions.append(
                    f"Pick {picked_id!r} and place it "
                    f"{step.relation} {destination_id!r}."
                )

            lmp_instruction = " and then ".join(
                resolved_actions
            )

        else:

            # Preserve the original prior-guided behavior.
            lmp_instruction = (
                action_plan.natural_language_plan
            )

        lmp(
            lmp_instruction,
            context=context,
        )

        if not wrapper.primitive_steps:
            raise RuntimeError(
                "CAP did not generate any robot primitive."
            )

        primitive_plan = PrimitivePlan(
            steps=tuple(wrapper.primitive_steps),
            source_code=lmp.exec_hist.strip(),
        )

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

            generated_program_path = (
                artifacts_dir
                / "generated_program.py"
            )

            generated_program_path.write_text(
                primitive_plan.source_code,
                encoding="utf-8",
            )

            primitive_plan_path = (
                artifacts_dir
                / "primitive_plan.json"
            )

            with primitive_plan_path.open(
                "w",
                encoding="utf-8",
            ) as stream:
                json.dump(
                    asdict(primitive_plan),
                    stream,
                    indent=2,
                    ensure_ascii=False,
                )

        return primitive_plan
    
    def _setup_lmp(
        self,
        wrapper: LMPSceneWrapper,
        scene_state: SceneState,
    ) -> LMP:
        config = copy.deepcopy(
            cfg_tabletop
        )

        config["lmps"]["tabletop_ui"]["prompt_text"] = (
            prompt_tabletop_ui_ros
        )

        config["lmps"]["parse_obj_name"]["prompt_text"] = (
            prompt_parse_obj_name_ros
        )

        # Use the model selected for the ROS2 SeeDo integration.
        for lmp_config in config["lmps"].values():
            if "engine" in lmp_config:
                lmp_config["engine"] = self.model

        # ------------------------------------------------------------------
        # Fixed variables
        # ------------------------------------------------------------------

        fixed_vars = {
            "np": np,
        }

        fixed_vars.update(
            {
                name: eval(name)
                for name in (
                    shapely.geometry.__all__
                    + shapely.affinity.__all__
                )
            }
        )

        # ------------------------------------------------------------------
        # Environment APIs exposed to CAP
        # ------------------------------------------------------------------

        variable_vars = {
            "get_obj_pos": wrapper.get_obj_pos,
            "get_obj_positions_np": wrapper.get_obj_positions_np,
            "get_obj_names": wrapper.get_obj_names,
            "is_obj_visible": wrapper.is_obj_visible,
            "denormalize_xy": wrapper.denormalize_xy,
            "get_corner_positions": wrapper.get_corner_positions,
            "get_side_positions": wrapper.get_side_positions,
            "get_corner_name": wrapper.get_corner_name,
            "get_side_name": wrapper.get_side_name,

            # Robot primitive APIs.
            "reach": wrapper.reach,
            "approaching": wrapper.approaching,
            "pick": wrapper.pick,
            "lift_up": wrapper.lift_up,
            "moving": wrapper.moving,
            "placing": wrapper.placing,
        }

        variable_vars["say"] = (
            lambda msg: print(
                f"robot says: {msg}"
            )
        )

        # ------------------------------------------------------------------
        # Function-generating LMP
        # ------------------------------------------------------------------

        lmp_fgen = LMPFGen(
            self.client,
            config["lmps"]["fgen"],
            fixed_vars,
            variable_vars,
        )

        # ------------------------------------------------------------------
        # Original low-level SeeDo / CAP LMPs
        # ------------------------------------------------------------------

        variable_vars.update(
            {
                name: LMP(
                    self.client,
                    name,
                    config["lmps"][name],
                    lmp_fgen,
                    fixed_vars,
                    variable_vars,
                )
                for name in (
                    "parse_obj_name",
                    "parse_position",
                    "parse_question",
                    "transform_shape_pts",
                )
            }
        )

        # ------------------------------------------------------------------
        # High-level Code-as-Policies LMP
        # ------------------------------------------------------------------

        lmp_tabletop_ui = LMP(
            self.client,
            "tabletop_ui",
            config["lmps"]["tabletop_ui"],
            lmp_fgen,
            fixed_vars,
            variable_vars,
        )

        return lmp_tabletop_ui