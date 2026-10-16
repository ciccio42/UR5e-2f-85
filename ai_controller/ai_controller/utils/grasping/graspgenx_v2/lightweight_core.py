"""Lightweight GraspGen-X V2 selector, with local semantic centering.

This module is shared by offline and real SeeDo inference through the same
runtime_worker. A small 3D region around the GraspMolmo point helps prefer
grips centered on the intended object part. Candidate orientations are kept
within a small angle of the original preferred grasp; poses are not translated
arbitrarily. The module does NOT verify object/fingertip, scene or arm
collisions and cannot certify safe physical execution.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from .runtime_core import (
    NoValidGrasp,
    V2Choice,
    V2Config,
    _distance_to_box,
    _project,
    _table_clearances,
    _transform,
    fit_table_plane,
)


# Exact V1 Robotiq conservative outer envelope. This is a bounding box, NOT
# a finger contact model. Source: graspgenx/planner.py, ROBOTIQ_BBOX_MIN/MAX.
_V1_BBOX_MIN = np.array([-0.07421507382649573, -0.037499999999747013, -0.004])
_V1_BBOX_MAX = np.array([0.07421507382649821, 0.037499999999747013, 0.14907399999999998])
_BBOX_VERTICES = np.array([
    [x, y, z]
    for x in (_V1_BBOX_MIN[0], _V1_BBOX_MAX[0])
    for y in (_V1_BBOX_MIN[1], _V1_BBOX_MAX[1])
    for z in (_V1_BBOX_MIN[2], _V1_BBOX_MAX[2])
], dtype=float)

# V1 sweep/contact region, NOT the entire gripper volume.
_SWEEP_CENTER = np.array([0.0, 0.0, 0.130], dtype=float)
_SWEEP_HALF = np.array([0.085, 0.032, 0.036], dtype=float) / 2.0

# Centering refinement: only when the old best candidate is significantly
# off-center. Keep the chosen wrist/gripper orientation within 5 degrees.
_SEMANTIC_PATCH_RADIUS_M = 0.018
_CENTER_TRIGGER_FRACTION = 0.55
_CENTER_ORIENTATION_TOLERANCE_DEG = 5.0
_CENTER_SCORE_TIEBREAK = 0.12
_MIN_CENTER_IMPROVEMENT = 0.06


class GraspGenXV2LightweightGeometry:
    """Evaluate every V1-like grasp variant, including both jaw symmetries."""

    def __init__(self, driver_urdf=None, robotiq_mesh_share=None, cfg=None):
        # Accepted only for signature compatibility with the existing worker.
        # Neither the URDF nor Trimesh is instantiated in lightweight mode.
        del driver_urdf, robotiq_mesh_share
        self.cfg = cfg or V2Config()
        if self.cfg.table_margin_m < 0 or self.cfg.semantic_radius_px <= 0:
            raise ValueError("Invalid table margin or semantic radius")

    def select(
        self, *, grasps_base, confidences, branch_tags, semantic_point_px,
        T_base_camera, camera_matrix, T_base_table, target_points_base,
        background_points_base, current_tcp_orientation=None,
    ) -> V2Choice:
        c = self.cfg
        grasps = np.asarray(grasps_base, dtype=float)
        conf = np.asarray(confidences, dtype=float).reshape(-1)
        tags = np.asarray(branch_tags).astype(str).reshape(-1)
        target = np.asarray(target_points_base, dtype=float).reshape(-1, 3)
        background = np.asarray(background_points_base, dtype=float).reshape(-1, 3)
        pixel = np.asarray(semantic_point_px, dtype=float).reshape(2)
        camera = _transform(T_base_camera, "T_base_camera")
        K = np.asarray(camera_matrix, dtype=float)
        table_T = _transform(T_base_table, "T_base_table")

        if (grasps.ndim != 3 or grasps.shape[1:] != (4, 4)
                or len(conf) != len(grasps) or len(tags) != len(grasps)):
            raise ValueError("Candidate shapes or counts are inconsistent")
        if len(target) < 5 or len(background) < 2000:
            raise NoValidGrasp("Insufficient observed target/table points")
        if K.shape != (3, 3) or K[0, 0] <= 0 or K[1, 1] <= 0:
            raise ValueError("Invalid camera intrinsics")
        if (not all(np.isfinite(a).all() for a in (
            grasps, conf, target, background, pixel, K,
        ))):
            raise ValueError("Non-finite geometry or confidence input")
        if not set(tags).issubset({"obb", "diff"}):
            raise ValueError("Unknown candidate branch tag")

        table_point, table_normal, table_fit = fit_table_plane(
            background, table_T, c
        )
        print(
            "[V2 LIGHT] tabletop: "
            f"offset={table_fit['offset_from_nominal_m']*1000:.2f}mm "
            f"rms={table_fit['rms_m']*1000:.2f}mm "
            f"inliers={table_fit['inlier_fraction']*100:.1f}%",
            flush=True,
        )

        # Protect against a GraspMolmo point unrelated to the observed target.
        target_uv = _project(target, camera, K)
        visible = np.isfinite(target_uv).all(axis=1)
        if not visible.any():
            raise NoValidGrasp("Target not visible from eye-in-hand camera")
        pixel_dists = np.linalg.norm(target_uv[visible] - pixel, axis=1)
        if float(pixel_dists.min()) > 3.0:
            raise NoValidGrasp(
                "GraspMolmo point is not on the observed target mask "
                f"(nearest={pixel_dists.min():.2f}px)"
            )
        semantic_point = target[np.flatnonzero(visible)[np.argmin(pixel_dists)]]

        # GraspMolmo may point to a VISIBLE EDGE of the handle rather than
        # its middle. Build a local target region and use its robust median.
        # This excludes distant points on the ring body or another object.
        semantic_neighborhood = (
            np.linalg.norm(target - semantic_point, axis=1)
            <= _SEMANTIC_PATCH_RADIUS_M
        )
        if int(semantic_neighborhood.sum()) >= 10:
            semantic_anchor = np.median(
                target[semantic_neighborhood], axis=0
            )
        else:
            semantic_anchor = semantic_point.copy()
        print(
            "[V2 LIGHT CENTER] local region: "
            f"points={int(semantic_neighborhood.sum())} "
            f"radius={_SEMANTIC_PATCH_RADIUS_M*1000:.1f}mm",
            flush=True,
        )

        rotation0 = None
        if current_tcp_orientation is not None:
            q = np.asarray(current_tcp_orientation, dtype=float).reshape(4)
            if not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
                raise ValueError("Invalid current TCP orientation")
            rotation0 = Rotation.from_quat(q)

        rejected = {
            key: 0 for key in (
                "table", "semantic", "sweep", "approach_table",
                "approach_target", "closure_table", "closure_structural_target",
                "closure_finger_penetration", "closure_no_inner_pad_contact",
            )
        }
        accepted = []
        evaluated_branches = {str(tag): 0 for tag in sorted(set(tags))}
        sweep_min = _SWEEP_CENTER - _SWEEP_HALF
        sweep_max = _SWEEP_CENTER + _SWEEP_HALF

        # Exact V1 bounding-box clearance, but against the *observed* table.
        # Both jaw symmetries are evaluated; no branch is discarded merely
        # because at least one OBB candidate passed.
        for symmetry, flip in (
            ("original", np.eye(3)),
            ("pi_z", np.diag([-1.0, -1.0, 1.0])),
        ):
            poses = grasps.copy()
            poses[:, :3, :3] = poses[:, :3, :3] @ flip
            final_clearance = _table_clearances(
                poses, _BBOX_VERTICES, table_point, table_normal
            )
            approach_poses = poses.copy()
            approach_poses[:, :3, 3] -= (
                c.approach_distance_m * poses[:, :3, 2]
            )
            start_clearance = _table_clearances(
                approach_poses, _BBOX_VERTICES, table_point, table_normal
            )
            # For a straight translation at fixed orientation, the minimum
            # plane clearance occurs at one of the two endpoints.
            clearances = np.minimum(final_clearance, start_clearance)

            for i in range(len(poses)):
                if clearances[i] < c.table_margin_m:
                    rejected["table"] += 1
                    continue

                R, t = poses[i, :3, :3], poses[i, :3, 3]
                local_target = (target - t) @ R
                inside = np.all(
                    (local_target >= sweep_min)
                    & (local_target <= sweep_max), axis=1,
                )
                if not inside.any():
                    rejected["sweep"] += 1
                    continue

                # V1 semantics: choose an OBSERVED point inside the open
                # sweep volume nearest its center, then reproject to RGB.
                points = target[inside]
                grasp_center = t + R @ _SWEEP_CENTER
                p = points[np.argmin(np.sum((points - grasp_center)**2, axis=1))]
                projected = _project(p[None, :], camera, K)[0]
                if not np.isfinite(projected).all():
                    rejected["semantic"] += 1
                    continue
                pixel_gap = float(np.linalg.norm(projected - pixel))
                if pixel_gap > c.semantic_radius_px:
                    rejected["semantic"] += 1
                    continue

                rotation_deg = None
                if rotation0 is not None:
                    # V1's gripper rotation equals TCP rotation (fixed
                    # gripper->TCP rotational transform is identity).
                    relative = rotation0.inv() * Rotation.from_matrix(R)
                    rotation_deg = float(np.degrees(relative.magnitude()))
                rot_score = (
                    1.0 if rotation_deg is None
                    else 1.0 - min(rotation_deg / 180.0, 1.0)
                )
                sem_score = 1.0 - min(pixel_gap / c.semantic_radius_px, 1.0)
                if rotation_deg is None:
                    denom = c.weight_confidence + c.weight_semantic
                    score = (
                        c.weight_confidence * float(np.clip(conf[i], 0, 1))
                        + c.weight_semantic * sem_score
                    ) / denom
                else:
                    denom = c.weight_rotation + c.weight_confidence + c.weight_semantic
                    score = (
                        c.weight_rotation * rot_score
                        + c.weight_confidence * float(np.clip(conf[i], 0, 1))
                        + c.weight_semantic * sem_score
                    ) / denom

                gap_3d = _distance_to_box(
                    (semantic_point - t) @ R, sweep_min, sweep_max
                )
                # Metric only: no change to the candidate pose, depth or
                # orientation. X is jaw closing direction, Y is finger width.
                anchor_local = (semantic_anchor - t) @ R
                center_cost = max(
                    abs(float(anchor_local[0])) / _SWEEP_HALF[0],
                    abs(float(anchor_local[1])) / _SWEEP_HALF[1],
                )
                accepted.append((
                    score, i, symmetry, poses[i].copy(),
                    float(final_clearance[i]), float(clearances[i]),
                    pixel_gap, float(gap_3d), rotation_deg,
                    int(inside.sum()), float(center_cost),
                    anchor_local[:2].copy(),
                ))
                evaluated_branches[str(tags[i])] += 1

        total = 2 * len(grasps)
        if len(accepted) + sum(rejected.values()) != total:
            raise AssertionError("Candidate accounting mismatch")
        print(
            f"[V2 LIGHT] evaluated={total}/{total} "
            f"valid={len(accepted)} rejected={rejected} "
            f"branches={evaluated_branches}; NOT ROBOT-SAFE",
            flush=True,
        )
        if not accepted:
            raise NoValidGrasp(
                f"No V1-style table/semantic-valid grasp: {rejected}"
            )
        # Keep the original score and selection unchanged for grasps whose
        # intended contact region is already fairly centered. This protects
        # the working cube/upright-ring behavior from unnecessary changes.
        baseline = max(
            accepted,
            key=lambda o: (o[0], conf[o[1]], -o[1], o[2] == "original"),
        )
        best = baseline
        if baseline[10] > _CENTER_TRIGGER_FRACTION:
            baseline_rot = Rotation.from_matrix(baseline[3][:3, :3])
            possible = [
                option for option in accepted
                if np.degrees((
                    baseline_rot.inv()
                    * Rotation.from_matrix(option[3][:3, :3])
                ).magnitude()) <= _CENTER_ORIENTATION_TOLERANCE_DEG
                and conf[option[1]] >= max(0.10, conf[baseline[1]] * 0.5)
            ]
            if possible:
                proposed = min(
                    possible,
                    key=lambda o: (
                        o[10] + _CENTER_SCORE_TIEBREAK * (1.0 - o[0]),
                        -o[0], o[1], o[2] != "original",
                    ),
                )
                if proposed[10] + _MIN_CENTER_IMPROVEMENT < baseline[10]:
                    best = proposed
            print(
                "[V2 LIGHT CENTER] original idx="
                f"{baseline[1]} centerXYmm="
                f"{np.round(baseline[11]*1000, 2).tolist()} "
                f"norm={baseline[10]:.3f}; "
                f"selected idx={best[1]} centerXYmm="
                f"{np.round(best[11]*1000, 2).tolist()} "
                f"norm={best[10]:.3f}; "
                f"same_orientation_candidates={len(possible)}",
                flush=True,
            )
            if best[10] > 0.75:
                print(
                    "[V2 LIGHT CENTER] WARNING: selected region remains "
                    "far from jaw center; review offline before robot use",
                    flush=True,
                )

        score, index, symmetry, pose, final_h, approach_h, px, gap, rotation, npts, center_cost, center_xy = best
        print(
            f"[V2 LIGHT] selected branch={tags[index]} idx={index} "
            f"symmetry={symmetry} score={score:.3f} "
            f"sweep_points={npts} rotation={rotation}; "
            "APPROACH/TARGET/CLOSURE UNCHECKED",
            flush=True,
        )
        return V2Choice(
            index=int(index), T_base_gripper=pose,
            confidence=float(conf[index]), branch=str(tags[index]),
            symmetry=str(symmetry), table_clearance_m=final_h,
            approach_table_clearance_m=approach_h,
            semantic_distance_px=px, semantic_3d_gap_m=gap,
            tested_expensive=total, rejection_counts=rejected,
            closure_first_contact_q_rad=None,
            closure_first_contact_links=(),
            closure_min_table_clearance_m=final_h,
            closure_contact_observed=False,
            closure_postcontact_obstacle_risk=False,
            closure_postcontact_table_risk=False,
            closure_postcontact_first_risk=None,
            rotation_deg=rotation, ranking_score=float(score),
            evaluated_branches=evaluated_branches,
            contact_visible_points=0,
            contact_inward_normal_dot=None,
            max_approach_penetration_m=0.0,
            max_precontact_penetration_m=0.0,
            contact_assessment="LIGHTWEIGHT_SWEEP_OCCUPANCY_NOT_PHYSICAL_CONTACT",
            geometry_limitations=(
                "NO fingertip or target-mesh collision checks",
                "NO closure/force/contact verification",
                "NO scene-obstacle or robot-arm collision checks",
                "Bounding box is approximate and conservative",
                "Requires real MoveIt plan + updated planning scene",
            ),
        )
