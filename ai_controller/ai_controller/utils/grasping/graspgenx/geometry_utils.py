
"""Shared geometric utilities for GraspGen-X grasp selection."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np



class NoValidGrasp(RuntimeError):
    """No candidate satisfies the observed geometric constraints."""


@dataclass(frozen=True)
class GraspGenXGeometryConfig:
    open_q_rad: float = 0.10
    closed_q_rad: float = 0.80
    closure_samples: int = 24

    table_margin_m: float = 0.005
    min_table_inlier_fraction: float = 0.50
    max_table_rms_m: float = 0.003
    max_table_tilt_deg: float = 10.0
    table_ransac_seed: int = 42
    table_ransac_iterations: int = 250
    table_ransac_threshold_m: float = 0.003
    table_max_offset_m: float = 0.16

    semantic_radius_px: float = 50.0
    semantic_gap_max_m: float = 0.005

    approach_distance_m: float = 0.080
    approach_step_m: float = 0.005

    # Geometry uncertainty tolerance, not physical clearance permission.
    max_precontact_penetration_m: float = 0.001
    contact_distance_m: float = 0.0015
    contact_min_visible_points: int = 2
    contact_min_inward_normal_dot: float = 0.50

    # Legacy setting retained for compatibility: exhaustive search always checks
    # every candidate that survives the inexpensive geometric/semantic gates.
    max_expensive_checks: int = 0
    min_branch_checks: int = 8

    weight_rotation: float = 0.70
    weight_confidence: float = 0.25
    weight_semantic: float = 0.05


@dataclass(frozen=True)
class GraspGenXGeometryChoice:
    index: int
    T_base_gripper: np.ndarray
    confidence: float
    branch: str
    symmetry: str
    table_clearance_m: float
    approach_table_clearance_m: float
    semantic_distance_px: float
    semantic_3d_gap_m: float
    tested_expensive: int
    rejection_counts: dict[str, int]

    closure_first_contact_q_rad: float | None
    closure_first_contact_links: tuple[str, ...]
    closure_min_table_clearance_m: float
    closure_contact_observed: bool
    closure_postcontact_obstacle_risk: bool
    closure_postcontact_table_risk: bool
    closure_postcontact_first_risk: dict[str, Any] | None

    rotation_deg: float | None
    ranking_score: float
    evaluated_branches: dict[str, int]

    contact_visible_points: int
    contact_inward_normal_dot: float | None
    max_approach_penetration_m: float
    max_precontact_penetration_m: float
    contact_assessment: str
    geometry_limitations: tuple[str, ...]


def _transform(T: np.ndarray, name: str) -> np.ndarray:
    T = np.asarray(T, dtype=np.float64)

    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError(f"{name}: finite 4x4 matrix required")

    if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-6):
        raise ValueError(f"{name}: invalid homogeneous row")

    R = T[:3, :3]

    if (
        not np.allclose(R.T @ R, np.eye(3), atol=1e-3)
        or np.linalg.det(R) < 0.999
    ):
        raise ValueError(f"{name}: invalid orientation")

    return T


def _project(
    points: np.ndarray,
    T_bc: np.ndarray,
    K: np.ndarray,
) -> np.ndarray:
    p = (points - T_bc[:3, 3]) @ T_bc[:3, :3]

    result = np.full(
        (len(p), 2),
        np.nan,
        dtype=np.float64,
    )

    good = (
        np.isfinite(p).all(axis=1)
        & (p[:, 2] > 0.01)
    )

    result[good, 0] = (
        K[0, 0] * p[good, 0] / p[good, 2] + K[0, 2]
    )

    result[good, 1] = (
        K[1, 1] * p[good, 1] / p[good, 2] + K[1, 2]
    )

    return result


def _distance_to_box(
    p: np.ndarray,
    low: np.ndarray,
    high: np.ndarray,
) -> float:
    delta = np.maximum(
        np.maximum(low - p, p - high),
        0,
    )

    return float(np.linalg.norm(delta))


def _table_clearances(
    poses,
    vertices,
    table_point,
    table_normal,
    batch=96,
):
    result = np.empty(len(poses), dtype=np.float64)

    for i in range(0, len(poses), batch):
        chunk = poses[i:i + batch]

        local_normal = np.einsum(
            "bji,j->bi",
            chunk[:, :3, :3],
            table_normal,
        )

        result[i:i + batch] = (
            (local_normal @ vertices.T).min(axis=1)
            + (chunk[:, :3, 3] - table_point) @ table_normal
        )

    return result


def fit_table_plane(
    background_base,
    T_base_table,
    cfg: GraspGenXGeometryConfig,
):
    """RANSAC table fitting with a near-horizontal URDF prior."""

    cloud = np.asarray(background_base, dtype=np.float64)

    T = _transform(T_base_table, "T_base_table")
    anchor = T[:3, 3]
    prior = T[:3, 2].copy()

    if prior[2] < 0:
        prior = -prior

    prior /= np.linalg.norm(prior)

    heights = (cloud - anchor) @ prior

    eligible = (
        np.isfinite(cloud).all(axis=1)
        & (heights > -0.06)
        & (heights < cfg.table_max_offset_m)
    )

    cloud = cloud[eligible]

    if len(cloud) < 2000:
        raise NoValidGrasp(
            "Table: insufficient background points"
        )

    def design(points):
        return np.column_stack((
            points[:, 0] - anchor[0],
            points[:, 1] - anchor[1],
            np.ones(len(points)),
        ))

    rng = np.random.default_rng(cfg.table_ransac_seed)

    sample = cloud[
        rng.choice(
            len(cloud),
            min(len(cloud), 12000),
            replace=False,
        )
    ]

    A = design(sample)
    best = None
    count_best = -1

    for _ in range(cfg.table_ransac_iterations):
        ids = rng.choice(
            len(sample),
            3,
            replace=False,
        )

        tri = A[ids]

        if abs(np.linalg.det(tri)) < 1e-8:
            continue

        c = np.linalg.solve(
            tri,
            sample[ids, 2],
        )

        n = np.array([-c[0], -c[1], 1.0])
        n /= np.linalg.norm(n)

        tilt = np.degrees(
            np.arccos(
                np.clip(n @ prior, -1.0, 1.0)
            )
        )

        if tilt > cfg.max_table_tilt_deg:
            continue

        hits = int(np.count_nonzero(
            np.abs(sample[:, 2] - A @ c)
            < cfg.table_ransac_threshold_m
        ))

        if hits > count_best:
            best = c
            count_best = hits

    if best is None:
        raise NoValidGrasp(
            "Table: plane fitting failed"
        )

    A_full = design(cloud)
    c = best

    for _ in range(5):
        residual = cloud[:, 2] - A_full @ c

        inside = (
            np.abs(residual)
            <= cfg.table_ransac_threshold_m
        )

        if int(inside.sum()) < 2000:
            raise NoValidGrasp(
                "Table: too few inliers"
            )

        c = np.linalg.lstsq(
            A_full[inside],
            cloud[inside, 2],
            rcond=None,
        )[0]

    n = np.array([-c[0], -c[1], 1.0])
    n /= np.linalg.norm(n)

    residual = (
        cloud[:, 2] - A_full @ c
    ) * n[2]

    inside = (
        np.abs(residual)
        <= cfg.table_ransac_threshold_m
    )

    fraction = float(inside.mean())

    rms = (
        float(np.sqrt(np.mean(residual[inside] ** 2)))
        if inside.any()
        else float("inf")
    )

    if (
        fraction < cfg.min_table_inlier_fraction
        or rms > cfg.max_table_rms_m
    ):
        raise NoValidGrasp(
            f"Table uncertain: inliers={fraction:.3f}, "
            f"rms={1000 * rms:.2f}mm"
        )

    if min(np.ptp(cloud[inside, :2], axis=0)) < 0.12:
        raise NoValidGrasp(
            "Table: observed area too small"
        )

    point = np.array([
        anchor[0],
        anchor[1],
        c[2],
    ])

    return point, n, {
        "inlier_fraction": fraction,
        "rms_m": rms,
        "offset_from_nominal_m": float(
            (point - anchor) @ prior
        ),
    }