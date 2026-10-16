
"""Offline GraspGen-X V2 geometry worker.

Input/output paths are supplied by the existing SeeDo V2 backend.
No ROS calls or physical robot commands.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .runtime_core import GraspGenXV2Geometry, V2Config


def run(
    input_path: Path,
    output_path: Path,
    urdf: Path,
    mesh_share: Path,
    open_q: float,
    table_margin_mm: float,
    max_expensive_checks: int,
) -> None:
    with np.load(
        input_path,
        allow_pickle=False,
    ) as src:
        data = {
            key: src[key]
            for key in src.files
        }

    cfg = V2Config(
        open_q_rad=float(open_q),
        table_margin_m=float(table_margin_mm) / 1000.0,
        max_expensive_checks=int(max_expensive_checks),
    )

    selector = GraspGenXV2Geometry(
        urdf,
        mesh_share,
        cfg,
    )

    choice = selector.select(
        grasps_base=data["grasps_base"],
        confidences=data["confidences"],
        branch_tags=data["branch_tags"],
        semantic_point_px=data["semantic_point_px"],
        T_base_camera=data["T_base_camera"],
        camera_matrix=data["camera_matrix"],
        T_base_table=data["T_base_table"],
        target_points_base=data["target_points_base"],
        background_points_base=data["background_points_base"],
        current_tcp_orientation=data.get(
            "current_tcp_orientation"
        ),
    )

    result = {
        "index": int(choice.index),
        "T_base_gripper": choice.T_base_gripper.tolist(),
        "confidence": float(choice.confidence),
        "branch": choice.branch,
        "symmetry": choice.symmetry,

        "table_clearance_m": float(
            choice.table_clearance_m
        ),
        "approach_table_clearance_m": float(
            choice.approach_table_clearance_m
        ),

        "semantic_distance_px": float(
            choice.semantic_distance_px
        ),
        "semantic_3d_gap_m": float(
            choice.semantic_3d_gap_m
        ),

        "rotation_deg": (
            None
            if choice.rotation_deg is None
            else float(choice.rotation_deg)
        ),
        "ranking_score": float(
            choice.ranking_score
        ),

        "tested_expensive": int(
            choice.tested_expensive
        ),
        "evaluated_branches": choice.evaluated_branches,
        "rejection_counts": choice.rejection_counts,

        "closure_first_contact_q_rad": (
            choice.closure_first_contact_q_rad
        ),
        "closure_first_contact_links": list(
            choice.closure_first_contact_links
        ),
        "closure_min_table_clearance_m": float(
            choice.closure_min_table_clearance_m
        ),
        "closure_contact_observed": bool(
            choice.closure_contact_observed
        ),

        "closure_contact_visible_points": int(
            choice.contact_visible_points
        ),
        "closure_contact_inward_normal_dot": (
            choice.contact_inward_normal_dot
        ),

        "max_approach_penetration_m": float(
            choice.max_approach_penetration_m
        ),
        "max_precontact_penetration_m": float(
            choice.max_precontact_penetration_m
        ),

        "contact_assessment": choice.contact_assessment,
        "geometry_limitations": list(
            choice.geometry_limitations
        ),

        "limits": {
            "table_margin_m": cfg.table_margin_m,
            "approach_step_m": cfg.approach_step_m,
            "closure_samples": cfg.closure_samples,
            "max_precontact_penetration_m": (
                cfg.max_precontact_penetration_m
            ),
            "contact_distance_m": cfg.contact_distance_m,
            "contact_min_visible_points": (
                cfg.contact_min_visible_points
            ),
            "contact_min_inward_normal_dot": (
                cfg.contact_min_inward_normal_dot
            ),
            "semantic_radius_px": cfg.semantic_radius_px,
            "semantic_gap_max_m": cfg.semantic_gap_max_m,
        },

        # Compatibility with the existing backend.
        "closure_postcontact_obstacle_risk": False,
        "closure_postcontact_table_risk": False,
        "closure_postcontact_first_risk": None,

        "status": "GEOMETRY_PREFILTERED_NOT_ROBOT_SAFE",
    }

    output_path.write_text(
        json.dumps(result, indent=2),
        encoding="utf-8",
    )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--input",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--mesh-share",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--open-q",
        type=float,
        default=0.1,
    )
    parser.add_argument(
        "--table-margin-mm",
        type=float,
        default=5.0,
    )
    parser.add_argument(
        "--max-expensive-checks",
        type=int,
        default=80,
    )

    args = parser.parse_args()

    run(
        args.input,
        args.output,
        args.urdf,
        args.mesh_share,
        args.open_q,
        args.table_margin_mm,
        args.max_expensive_checks,
    )


if __name__ == "__main__":
    main()
