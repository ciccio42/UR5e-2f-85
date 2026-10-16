
"""GraspGen-X V2: conservative target/table geometric prefilter.

The candidate generator, semantic model, robot IK, physical grasp forces and
real MoveIt trajectory are OUTSIDE this module. An accepted pose is NOT a
robot-safe grasp. The depth-derived target is a partial observed surface.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from .frames import fixed_chain_transform
from .gripper_model import RobotiqURDFModel


class NoValidGrasp(RuntimeError):
    """No candidate satisfies the observed geometric constraints."""


@dataclass(frozen=True)
class V2Config:
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
class V2Choice:
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


def fit_table_plane(background_base, T_base_table, cfg: V2Config):
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


def _signed_near_mesh(mesh, points_local, bound_padding):
    """
    Compute signed distances between observed points and a Robotiq
    collision mesh.

    Trimesh convention:
        signed > 0 : inside the solid
        signed = 0 : on the surface
        signed < 0 : outside the solid

    STL meshes may contain duplicated vertices. Merge them before
    checking whether the geometry represents a valid closed volume.
    """

    solid = mesh.copy()
    solid.merge_vertices()

    if not solid.is_volume:
        raise RuntimeError(
            "Robotiq collision mesh is not a valid closed solid "
            f"even after vertex merging. "
            f"watertight={solid.is_watertight}, "
            f"winding_consistent={solid.is_winding_consistent}"
        )

    points_local = np.asarray(
        points_local,
        dtype=np.float64,
    )

    low, high = solid.bounds

    ids = np.flatnonzero(
        np.all(
            (points_local >= low - bound_padding)
            & (points_local <= high + bound_padding),
            axis=1,
        )
    )

    if len(ids) == 0:
        return ids, np.empty(0, dtype=np.float64)

    signed = trimesh.proximity.signed_distance(
        solid,
        points_local[ids],
    )

    if not np.isfinite(signed).all():
        raise RuntimeError(
            "Invalid signed distances from Robotiq mesh"
        )

    return ids, signed


def _observed_penetration(
    local_target,
    parts,
    cutoff,
):
    maximum = 0.0
    offending = None

    for name, mesh in parts.items():
        _, distances = _signed_near_mesh(
            mesh,
            local_target,
            cutoff,
        )

        if len(distances):
            depth = float(
                max(0.0, np.max(distances))
            )

            if depth > maximum:
                maximum = depth
                offending = name

    return maximum, offending


class GraspGenXV2Geometry:
    SWEEP_CENTER = np.array([0.0, 0.0, 0.130])
    SWEEP_HALF = (
        np.array([0.085, 0.032, 0.036]) / 2.0
    )
    PI_Z = np.diag([-1.0, -1.0, 1.0])

    LIMITATIONS = (
        "Single depth frame covers only visible object surfaces",
        "Contact is kinematic, not force/force-closure verification",
        "Straight approach is not the actual MoveIt trajectory",
        "Small/inter-sample collisions can be missed",
        "URDF model and camera-table extrinsics require independent calibration",
    )

    def __init__(
        self,
        driver_urdf,
        robotiq_mesh_share,
        cfg=None,
    ):
        self.urdf = Path(driver_urdf)
        self.mesh_share = Path(robotiq_mesh_share)
        self.cfg = cfg or V2Config()

        if (
            not self.urdf.is_file()
            or not self.mesh_share.is_dir()
        ):
            raise FileNotFoundError(
                "URDF or Robotiq mesh package not found"
            )

        c = self.cfg

        if not (
            0 <= c.open_q_rad < c.closed_q_rad <= 0.8
            and c.closure_samples >= 2
        ):
            raise ValueError(
                "Invalid gripper closure joint range"
            )

        if not (
            0 < c.approach_step_m <= c.approach_distance_m
            and c.semantic_radius_px > 0
            and c.semantic_gap_max_m > 0
            and 0 <= c.max_precontact_penetration_m < 0.004
            and c.contact_distance_m > 0
            and c.contact_min_visible_points >= 1
        ):
            raise ValueError(
                "Invalid geometric validation parameters"
            )

        if (
            c.max_expensive_checks < 0
            or c.min_branch_checks < 0
        ):
            raise ValueError(
                "Invalid candidate search budget"
            )

        if min(
            c.weight_rotation,
            c.weight_confidence,
            c.weight_semantic,
        ) < 0:
            raise ValueError(
                "Negative ranking weight"
            )

        if sum((
            c.weight_rotation,
            c.weight_confidence,
            c.weight_semantic,
        )) <= 0:
            raise ValueError(
                "Ranking weights sum to zero"
            )

        if (
            c.weight_confidence + c.weight_semantic
        ) <= 0:
            raise ValueError(
                "Need confidence/semantic weights without orientation"
            )

        self.model = RobotiqURDFModel(
            self.urdf,
            self.mesh_share,
        )

        self.open_parts = self.model.collision_parts(
            c.open_q_rad
        )

        self.open_vertices = np.vstack([
            mesh.vertices
            for mesh in self.open_parts.values()
        ])

        T_tg = fixed_chain_transform(
            self.urdf,
            "tool0",
            "robotiq_85_base_link",
        )

        T_tt = fixed_chain_transform(
            self.urdf,
            "tool0",
            "tcp_link",
        )

        self.T_gripper_tcp = (
            np.linalg.inv(T_tg) @ T_tt
        )

        self._closure_cache = None

    def _get_closure_cache(self):
        if self._closure_cache is None:
            result = []

            for q in np.linspace(
                self.cfg.open_q_rad,
                self.cfg.closed_q_rad,
                self.cfg.closure_samples + 1,
            ):
                parts = self.model.collision_parts(
                    float(q)
                )

                vertices = np.vstack([
                    mesh.vertices
                    for mesh in parts.values()
                ])

                result.append((
                    float(q),
                    parts,
                    vertices,
                ))

            self._closure_cache = result

        return self._closure_cache

    def _contact_on_inward_pad(
        self,
        local_target,
        mesh,
        link,
        ids,
        signed,
    ):
        """Check proximity to inward-facing fingertip surfaces."""

        c = self.cfg

        near_pad = (
            (signed >= -c.contact_distance_m)
            & (signed <= c.max_precontact_penetration_m)
        )

        ids = ids[near_pad]

        if not len(ids):
            return 0, 0.0

        points = local_target[ids]

        low = self.SWEEP_CENTER - self.SWEEP_HALF
        high = self.SWEEP_CENTER + self.SWEEP_HALF

        in_sweep = np.all(
            (points >= low) & (points <= high),
            axis=1,
        )

        if not in_sweep.any():
            return 0, 0.0

        # Determine the jaw side from the articulated geometry.
        side = np.sign(
            float(np.mean(mesh.vertices[:, 0]))
        )

        if abs(side) < 0.5:
            return 0, 0.0

        inward = np.array([-side, 0.0, 0.0])

        _, _, triangle_ids = (
            trimesh.proximity.closest_point(
                mesh,
                points,
            )
        )

        normals = np.asarray(
            mesh.face_normals
        )[triangle_ids]

        inward_dot = normals @ inward

        valid = (
            in_sweep
            & (
                inward_dot
                >= c.contact_min_inward_normal_dot
            )
        )

        if not valid.any():
            return 0, 0.0

        return (
            int(valid.sum()),
            float(np.max(inward_dot[valid])),
        )

    def _validate_closure(
        self,
        pose,
        target_local,
        table_point,
        table_normal,
    ):
        """Check target until first pad contact; table for full closure."""

        c = self.cfg
        R, t = pose[:3, :3], pose[:3, 3]

        n_local = R.T @ table_normal
        h_origin = float(
            (t - table_point) @ table_normal
        )

        first_q = None
        first_links = ()
        first_n = 0
        best_dot = None
        min_table = float("inf")
        max_penetration = 0.0
        checked = 0

        def info(**details):
            return {
                "first_contact_q_rad": first_q,
                "first_contact_links": first_links,
                "contact_visible_points": first_n,
                "contact_inward_normal_dot": best_dot,
                "min_table_clearance_m": min_table,
                "max_precontact_penetration_m": max_penetration,
                "closure_samples_checked": checked,
                "observed_contact": first_q is not None,
                **details,
            }

        for q, parts, vertices in self._get_closure_cache():
            checked += 1

            clearance = float(
                np.min(vertices @ n_local) + h_origin
            )

            min_table = min(min_table, clearance)

            if clearance < c.table_margin_m:
                return (
                    False,
                    "closure_table",
                    info(
                        failure_q_rad=q,
                        failure_clearance_m=clearance,
                    ),
                )

            if first_q is not None:
                # Object displacement after contact is unknown.
                # Continue checking the table, not the target.
                continue

            contacts = []
            support = 0
            dot_max = 0.0

            for link, mesh in parts.items():
                ids, signed = _signed_near_mesh(
                    mesh,
                    target_local,
                    max(
                        c.max_precontact_penetration_m,
                        c.contact_distance_m,
                    ),
                )

                if not len(ids):
                    continue

                deep = float(
                    max(0.0, np.max(signed))
                )

                max_penetration = max(
                    max_penetration,
                    deep,
                )

                tip = "finger_tip_link" in link

                if (
                    not tip
                    and deep > c.max_precontact_penetration_m
                ):
                    return (
                        False,
                        "closure_structural_target",
                        info(
                            failure_q_rad=q,
                            failure_link=link,
                            penetration_m=deep,
                        ),
                    )

                if not tip:
                    continue

                count, dot = (
                    self._contact_on_inward_pad(
                        target_local,
                        mesh,
                        link,
                        ids,
                        signed,
                    )
                )

                # Before first contact, even a finger pad must
                # not penetrate deeply into the observed target.
                if deep > c.max_precontact_penetration_m:
                    return (
                        False,
                        "closure_finger_penetration",
                        info(
                            failure_q_rad=q,
                            failure_link=link,
                            penetration_m=deep,
                        ),
                    )

                if count >= c.contact_min_visible_points:
                    contacts.append(link)
                    support += count
                    dot_max = max(dot_max, dot)

            if contacts:
                first_q = q
                first_links = tuple(sorted(set(contacts)))
                first_n = support
                best_dot = dot_max

        if first_q is None:
            return (
                False,
                "closure_no_inner_pad_contact",
                info(),
            )

        return True, "observed_first_pad_contact", info()

    def select(
        self,
        *,
        grasps_base,
        confidences,
        branch_tags,
        semantic_point_px,
        T_base_camera,
        camera_matrix,
        T_base_table,
        target_points_base,
        background_points_base,
        current_tcp_orientation=None,
    ):
        c = self.cfg

        grasps = np.asarray(
            grasps_base,
            dtype=np.float64,
        )

        conf = np.asarray(
            confidences,
            dtype=np.float64,
        ).reshape(-1)

        tags = np.asarray(
            branch_tags
        ).astype(str).reshape(-1)

        target = np.asarray(
            target_points_base,
            dtype=np.float64,
        ).reshape(-1, 3)

        background = np.asarray(
            background_points_base,
            dtype=np.float64,
        ).reshape(-1, 3)

        pixel = np.asarray(
            semantic_point_px,
            dtype=np.float64,
        ).reshape(2)

        K = np.asarray(
            camera_matrix,
            dtype=np.float64,
        )

        T_cam = _transform(
            T_base_camera,
            "T_base_camera",
        )

        if (
            grasps.ndim != 3
            or grasps.shape[1:] != (4, 4)
            or len(grasps) != len(conf)
            or len(grasps) != len(tags)
        ):
            raise ValueError(
                "Invalid GraspGen-X candidate arrays"
            )

        if not (
            np.isfinite(grasps).all()
            and np.isfinite(conf).all()
            and np.isfinite(target).all()
            and np.isfinite(background).all()
            and np.isfinite(pixel).all()
        ):
            raise ValueError(
                "Nonfinite scene input"
            )

        if (
            K.shape != (3, 3)
            or not np.isfinite(K).all()
            or K[0, 0] <= 0
            or K[1, 1] <= 0
        ):
            raise ValueError(
                "Invalid camera intrinsics"
            )

        if len(target) < 5 or len(background) < 2000:
            raise NoValidGrasp(
                "Insufficient target/table point cloud"
            )

        if not all(
            x in {"obb", "diff"}
            for x in set(tags)
        ):
            raise ValueError(
                "Unknown GraspGen-X branch tag"
            )

        current_R = None

        if current_tcp_orientation is not None:
            q = np.asarray(
                current_tcp_orientation,
                dtype=np.float64,
            ).reshape(4)

            if (
                not np.isfinite(q).all()
                or np.linalg.norm(q) < 1e-9
            ):
                raise ValueError(
                    "Invalid current TCP quaternion"
                )

            current_R = Rotation.from_quat(q)

        table_point, table_normal, table_fit = (
            fit_table_plane(
                background,
                T_base_table,
                c,
            )
        )

        print(
            "[V2] tabletop: "
            f"offset={table_fit['offset_from_nominal_m']*1000:.2f}mm "
            f"rms={table_fit['rms_m']*1000:.2f}mm "
            f"inliers={table_fit['inlier_fraction']*100:.1f}%",
            flush=True,
        )

        uv_target = _project(target, T_cam, K)
        visible = np.isfinite(uv_target).all(axis=1)

        if not visible.any():
            raise NoValidGrasp(
                "Target not visible from provided camera"
            )

        distances = np.sum(
            (uv_target[visible] - pixel) ** 2,
            axis=1,
        )

        nearest = np.flatnonzero(visible)[
            int(np.argmin(distances))
        ]

        if float(np.sqrt(np.min(distances))) > 3.0:
            raise NoValidGrasp(
                "GraspMolmo point does not coincide "
                "with observed target mask"
            )

        semantic_point = target[nearest]

        sweep_min = self.SWEEP_CENTER - self.SWEEP_HALF
        sweep_max = self.SWEEP_CENTER + self.SWEEP_HALF

        rejected = {
            reason: 0
            for reason in (
                "table",
                "semantic",
                "sweep",
                "approach_table",
                "approach_target",
                "closure_table",
                "closure_structural_target",
                "closure_finger_penetration",
                "closure_no_inner_pad_contact",
            )
        }

        options = []

        for symmetry, flip in (
            ("original", np.eye(3)),
            ("pi_z", self.PI_Z),
        ):
            poses = grasps.copy()

            poses[:, :3, :3] = (
                poses[:, :3, :3] @ flip
            )

            clear = _table_clearances(
                poses,
                self.open_vertices,
                table_point,
                table_normal,
            )

            safe_ids = np.flatnonzero(
                clear >= c.table_margin_m
            )

            rejected["table"] += (
                len(poses) - len(safe_ids)
            )

            for i in safe_ids:
                pose = poses[i]
                R, t = pose[:3, :3], pose[:3, 3]

                sem_local = (
                    semantic_point - t
                ) @ R

                gap = _distance_to_box(
                    sem_local,
                    sweep_min,
                    sweep_max,
                )

                if gap > c.semantic_gap_max_m:
                    rejected["semantic"] += 1
                    continue

                local_target = (target - t) @ R

                in_sweep = np.all(
                    (local_target >= sweep_min)
                    & (local_target <= sweep_max),
                    axis=1,
                )

                if not in_sweep.any():
                    rejected["sweep"] += 1
                    continue

                grasp_ref = (
                    t + R @ self.SWEEP_CENTER
                )

                candidates = target[in_sweep]

                semantic_contact = candidates[
                    int(np.argmin(
                        np.sum(
                            (candidates - grasp_ref) ** 2,
                            axis=1,
                        )
                    ))
                ]

                uv = _project(
                    semantic_contact[None, :],
                    T_cam,
                    K,
                )[0]

                pixel_gap = float(
                    np.linalg.norm(uv - pixel)
                )

                if (
                    not np.isfinite(pixel_gap)
                    or pixel_gap > c.semantic_radius_px
                ):
                    rejected["semantic"] += 1
                    continue

                tcp_rotation = (
                    R @ self.T_gripper_tcp[:3, :3]
                )

                rotation_deg = None

                if current_R is not None:
                    relative = (
                        current_R.inv()
                        * Rotation.from_matrix(tcp_rotation)
                    )

                    rotation_deg = float(
                        np.degrees(relative.magnitude())
                    )

                confidence_score = float(
                    np.clip(conf[i], 0, 1)
                )

                sem_score = (
                    1.0
                    - min(
                        gap / c.semantic_gap_max_m,
                        1.0,
                    )
                )

                if rotation_deg is None:
                    denom = (
                        c.weight_confidence
                        + c.weight_semantic
                    )

                    score = (
                        c.weight_confidence * confidence_score
                        + c.weight_semantic * sem_score
                    ) / denom

                else:
                    rotation_score = (
                        1.0
                        - min(
                            rotation_deg / 180.0,
                            1.0,
                        )
                    )

                    denom = (
                        c.weight_rotation
                        + c.weight_confidence
                        + c.weight_semantic
                    )

                    score = (
                        c.weight_rotation * rotation_score
                        + c.weight_confidence * confidence_score
                        + c.weight_semantic * sem_score
                    ) / denom

                options.append((
                    score,
                    int(i),
                    symmetry,
                    pose.copy(),
                    float(clear[i]),
                    pixel_gap,
                    gap,
                    rotation_deg,
                ))

        if not options:
            raise NoValidGrasp(
                "No semantically/table-valid candidates: "
                f"{rejected}"
            )

        options.sort(
            key=lambda o: (
                -o[0],
                -conf[o[1]],
                o[1],
                o[2],
            )
        )

        branches = sorted(set(
            str(tags[o[1]])
            for o in options
        ))

        # Full evaluation: do not truncate to max_expensive_checks.
        # The old 80-candidate limit could hide valid grasps.
        budget = len(options)

        reserved = (
            min(
                c.min_branch_checks,
                budget // len(branches),
            )
            if len(branches) > 1
            else 0
        )

        shortlist = []
        seen = set()

        if reserved > 0:
            for branch in branches:
                allocated = 0

                for option in options:
                    key = (option[1], option[2])

                    if (
                        str(tags[option[1]]) == branch
                        and key not in seen
                    ):
                        shortlist.append(option)
                        seen.add(key)
                        allocated += 1

                    if allocated >= reserved:
                        break

        for option in options:
            key = (option[1], option[2])

            if len(shortlist) >= budget:
                break

            if key not in seen:
                shortlist.append(option)
                seen.add(key)

        shortlist.sort(
            key=lambda o: (
                -o[0],
                -conf[o[1]],
                o[1],
                o[2],
            )
        )

        branch_budget = {
            branch: sum(
                str(tags[o[1]]) == branch
                for o in shortlist
            )
            for branch in branches
        }

        print(
            f"[V2] candidates={len(options)} "
            f"shortlist={len(shortlist)} "
            f"branches={branch_budget} "
            f"rotation={'active' if current_R else 'unavailable'}",
            flush=True,
        )

        steps = max(
            1,
            int(np.ceil(
                c.approach_distance_m
                / c.approach_step_m
            )),
        )

        checked = {
            branch: 0
            for branch in branches
        }

        accepted = []
        failure_examples = []

        for option in shortlist:
            (
                score,
                index,
                symmetry,
                pose,
                final_table,
                semantic_px,
                semantic_3d,
                rotation_deg,
            ) = option

            checked[str(tags[index])] += 1

            R, t = pose[:3, :3], pose[:3, 3]
            target_local_final = (target - t) @ R

            approach_table_min = float("inf")
            approach_max_penetration = 0.0
            reason = None

            for step in range(steps + 1):
                d = (
                    c.approach_distance_m
                    * (1.0 - step / steps)
                )

                T = pose.copy()
                T[:3, 3] = t - d * R[:, 2]

                clearance = float(
                    _table_clearances(
                        T[None, :, :],
                        self.open_vertices,
                        table_point,
                        table_normal,
                    )[0]
                )

                approach_table_min = min(
                    approach_table_min,
                    clearance,
                )

                if clearance < c.table_margin_m:
                    reason = "approach_table"
                    break

                local_target = (
                    target - T[:3, 3]
                ) @ R

                penetration, offending_link = (
                    _observed_penetration(
                        local_target,
                        self.open_parts,
                        c.max_precontact_penetration_m,
                    )
                )

                approach_max_penetration = max(
                    approach_max_penetration,
                    penetration,
                )

                if (
                    penetration
                    > c.max_precontact_penetration_m
                ):
                    reason = "approach_target"

                    # Diagnostics only: never relax the penetration threshold.
                    if len(failure_examples) < 8:
                        print(
                            f"[V2 APPROACH DEBUG] "
                            f"idx={index} "
                            f"symmetry={symmetry} "
                            f"step={step}/{steps} "
                            f"distance_to_goal={d * 1000:.1f}mm "
                            f"penetration={penetration * 1000:.2f}mm "
                            f"link={offending_link} "
                            f"table_clearance={clearance * 1000:.2f}mm",
                            flush=True,
                        )

                    break

            if reason is None:
                closure_ok, reason, info = (
                    self._validate_closure(
                        pose,
                        target_local_final,
                        table_point,
                        table_normal,
                    )
                )

                if closure_ok:
                    accepted.append((
                        option,
                        approach_table_min,
                        approach_max_penetration,
                        info,
                    ))

                    # Do not stop after the first valid grasp: rank all valid poses.
                    continue

            rejected[reason] += 1

            if len(failure_examples) < 8:
                failure_examples.append({
                    "index": index,
                    "branch": str(tags[index]),
                    "symmetry": symmetry,
                    "reason": reason,
                })

        if not accepted:
            raise NoValidGrasp(
                "No observed-geometry-valid grasp "
                f"({sum(checked.values())}/{len(shortlist)} tested): "
                f"{rejected}; examples={failure_examples}"
            )

        (
            option,
            approach_table,
            approach_max_pen,
            closure,
        ) = max(
            accepted,
            key=lambda a: (
                a[0][0],
                conf[a[0][1]],
                -a[0][1],
            ),
        )

        (
            score,
            index,
            symmetry,
            pose,
            final_table,
            semantic_px,
            semantic_3d,
            rotation_deg,
        ) = option

        print(
            f"[V2] selected branch={tags[index]} idx={index} "
            f"symmetry={symmetry} "
            f"rotation={rotation_deg if rotation_deg is not None else 'n/a'} "
            f"score={score:.3f} "
            f"visible_inner_contact={closure['contact_visible_points']} "
            f"approach_penetration={1000*approach_max_pen:.2f}mm "
            f"closure_penetration="
            f"{1000*closure['max_precontact_penetration_m']:.2f}mm "
            f"evaluated={checked}",
            flush=True,
        )

        assessment = (
            "TWO_VISIBLE_PAD_CONTACTS_FORCE_UNVERIFIED"
            if len(closure["first_contact_links"]) >= 2
            else "ONE_VISIBLE_PAD_CONTACT_OTHER_SIDE_UNVERIFIED"
        )

        return V2Choice(
            index=index,
            T_base_gripper=pose,
            confidence=float(conf[index]),
            branch=str(tags[index]),
            symmetry=symmetry,
            table_clearance_m=final_table,
            approach_table_clearance_m=approach_table,
            semantic_distance_px=semantic_px,
            semantic_3d_gap_m=semantic_3d,
            tested_expensive=sum(checked.values()),
            rejection_counts=rejected,

            closure_first_contact_q_rad=closure[
                "first_contact_q_rad"
            ],
            closure_first_contact_links=closure[
                "first_contact_links"
            ],
            closure_min_table_clearance_m=closure[
                "min_table_clearance_m"
            ],
            closure_contact_observed=closure[
                "observed_contact"
            ],
            closure_postcontact_obstacle_risk=False,
            closure_postcontact_table_risk=False,
            closure_postcontact_first_risk=None,

            rotation_deg=rotation_deg,
            ranking_score=float(score),
            evaluated_branches=checked,

            contact_visible_points=closure[
                "contact_visible_points"
            ],
            contact_inward_normal_dot=closure[
                "contact_inward_normal_dot"
            ],
            max_approach_penetration_m=approach_max_pen,
            max_precontact_penetration_m=closure[
                "max_precontact_penetration_m"
            ],
            contact_assessment=assessment,
            geometry_limitations=self.LIMITATIONS,
        )
