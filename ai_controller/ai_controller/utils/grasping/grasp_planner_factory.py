from __future__ import annotations

from typing import Any, Mapping

from ai_controller.utils.grasping.grasp_planner import (
    GraspPlanner,
)
from ai_controller.utils.grasping.grasp_planner_base import (
    GraspPlannerBackend,
)
from ai_controller.utils.grasping.graspgenx.planner import (
    GraspGenXPlanner,
)


def build_grasp_planner(
    grasp_config: Mapping[str, Any],
) -> GraspPlannerBackend:
    """
    Build the grasp-planning backend selected in the SeeDo config.
    """

    backend = str(
        grasp_config.get(
            "backend",
            "m2t2",
        )
    ).strip().lower()

    if backend == "m2t2":
        return GraspPlanner(
            m2t2_config_path=(
                grasp_config.get(
                    "m2t2_config_path"
                )
            ),
            graspmolmo_config_path=(
                grasp_config.get(
                    "graspmolmo_config_path"
                )
            ),
            num_runs=int(
                grasp_config.get(
                    "num_runs",
                    20,
                )
            ),
            seed=int(
                grasp_config.get(
                    "seed",
                    42,
                )
            ),
            confidence_threshold=float(
                grasp_config.get(
                    "confidence_threshold",
                    0.4,
                )
            ),
            semantic_radius_px=float(
                grasp_config.get(
                    "semantic_radius_px",
                    50.0,
                )
            ),
            max_approach_tilt_deg=float(
                grasp_config.get(
                    "max_approach_tilt_deg",
                    45.0,
                )
            ),
            max_wrist_rotation_deg=float(
                grasp_config.get(
                    "max_wrist_rotation_deg",
                    45.0,
                )
            ),
            m2t2_gripper_depth_m=float(
                grasp_config.get(
                    "m2t2_gripper_depth_m",
                    0.1034,
                )
            ),
            depth_min=float(
                grasp_config.get(
                    "depth_min",
                    0.15,
                )
            ),
            depth_max=float(
                grasp_config.get(
                    "depth_max",
                    0.60,
                )
            ),
            bottom_ignore_px=int(
                grasp_config.get(
                    "bottom_ignore_px",
                    30,
                )
            ),
            surface_snap=float(
                grasp_config.get(
                    "surface_snap",
                    0.0,
                )
            ),
            finger_collision_check=bool(
                grasp_config.get(
                    "finger_collision_check",
                    True,
                )
            ),
            gripper_opening_m=float(
                grasp_config.get(
                    "gripper_opening_m",
                    0.085,
                )
            ),
            finger_thickness_m=float(
                grasp_config.get(
                    "finger_thickness_m",
                    0.031214,
                )
            ),
            finger_width_m=float(
                grasp_config.get(
                    "finger_width_m",
                    0.027000,
                )
            ),
            finger_z_min_m=float(
                grasp_config.get(
                    "finger_z_min_m",
                    0.0483,
                )
            ),
            finger_z_max_m=float(
                grasp_config.get(
                    "finger_z_max_m",
                    0.1053,
                )
            ),
            collision_margin_m=float(
                grasp_config.get(
                    "collision_margin_m",
                    0.002,
                )
            ),
            collision_min_points=int(
                grasp_config.get(
                    "collision_min_points",
                    3,
                )
            ),
        )

    if backend == "graspgenx":
        return GraspGenXPlanner(
            server_host=str(
                grasp_config.get(
                    "graspgenx_server_host",
                    "127.0.0.1",
                )
            ),
            server_port=int(
                grasp_config.get(
                    "graspgenx_server_port",
                    8782,
                )
            ),
            gripper_name=str(
                grasp_config.get(
                    "graspgenx_gripper_name",
                    "robotiq_2f_85",
                )
            ),
            num_grasps=int(
                grasp_config.get(
                    "graspgenx_num_grasps",
                    200,
                )
            ),
            grasp_threshold=float(
                grasp_config.get(
                    "graspgenx_grasp_threshold",
                    -1.0,
                )
            ),
            topk_num_grasps=int(
                grasp_config.get(
                    "graspgenx_topk_num_grasps",
                    100,
                )
            ),
            graspmolmo_config_path=(
                grasp_config.get(
                    "graspmolmo_config_path"
                )
            ),
            seed=int(
                grasp_config.get(
                    "seed",
                    42,
                )
            ),
        )

    raise ValueError(
        "Unsupported grasp-planning backend: "
        f"{backend!r}. "
        "Supported backends are 'm2t2' and 'graspgenx'."
    )