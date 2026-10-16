
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class SceneGeometry:
    target_points_base: np.ndarray
    background_points_base: np.ndarray
    K: np.ndarray
    T_base_camera: np.ndarray
    T_base_table: np.ndarray
    depth_valid_mask: np.ndarray
    target_mask: np.ndarray


def backproject(depth, mask, K):
    valid = (
        mask
        & np.isfinite(depth)
        & (depth >= 0.15)
        & (depth <= 0.60)
    )

    v, u = np.nonzero(valid)

    z = depth[v, u].astype(np.float64)

    x = (u - K[0, 2]) * z / K[0, 0]
    y = (v - K[1, 2]) * z / K[1, 1]

    return np.column_stack((x, y, z)), v, u


def recover_intrinsics(depth, target_mask, recorded_points):
    valid = (
        target_mask
        & np.isfinite(depth)
        & (depth >= 0.15)
        & (depth <= 0.60)
    )

    v, u = np.nonzero(valid)

    if len(recorded_points) != len(u):
        raise ValueError(
            "Target points and valid mask pixels do not match: "
            f"{len(recorded_points)} != {len(u)}"
        )

    z = depth[v, u]

    depth_error = np.max(
        np.abs(recorded_points[:, 2] - z)
    )

    if depth_error > 0.0002:
        raise ValueError(
            "Saved target cloud is inconsistent with depth: "
            f"{depth_error * 1000:.3f} mm"
        )

    x_normalized = (
        recorded_points[:, 0] / recorded_points[:, 2]
    )

    y_normalized = (
        recorded_points[:, 1] / recorded_points[:, 2]
    )

    fx, cx = np.linalg.lstsq(
        np.column_stack(
            (x_normalized, np.ones(len(u)))
        ),
        u,
        rcond=None,
    )[0]

    fy, cy = np.linalg.lstsq(
        np.column_stack(
            (y_normalized, np.ones(len(v)))
        ),
        v,
        rcond=None,
    )[0]

    K = np.array([
        [fx, 0.0, cx],
        [0.0, fy, cy],
        [0.0, 0.0, 1.0],
    ])

    if (
        not np.isfinite(K).all()
        or fx <= 0.0
        or fy <= 0.0
    ):
        raise ValueError("Invalid recovered intrinsics")

    return K


def transform_points(points, T):
    return (
        points @ T[:3, :3].T + T[:3, 3]
    )


def load_case_scene(case_dir):
    case_dir = Path(case_dir)

    artifact_dir = (
        case_dir / "grasp_planner/action_00"
    )

    with np.load(
        artifact_dir / "graspgenx_diagnostics.npz",
        allow_pickle=False,
    ) as data:
        target_mask = data["target_mask"].astype(bool)

        recorded_target_camera = np.asarray(
            data["target_point_cloud_camera"],
            dtype=np.float64,
        )

        recorded_target_base = np.asarray(
            data["target_point_cloud_base"],
            dtype=np.float64,
        )

        T_base_camera = np.asarray(
            data["T_base_camera"], dtype=np.float64
        )

        T_base_table = np.asarray(
            data["T_base_table"], dtype=np.float64
        )

    depth = np.load(
        artifact_dir / "input_depth.npy"
    ).astype(np.float64)

    if depth.shape != target_mask.shape:
        raise ValueError(
            "Depth and target mask have different shapes"
        )

    K = recover_intrinsics(
        depth,
        target_mask,
        recorded_target_camera,
    )

    valid_depth = (
        np.isfinite(depth)
        & (depth >= 0.15)
        & (depth <= 0.60)
    )

    target_camera, _, _ = backproject(
        depth, target_mask, K
    )

    background_camera, _, _ = backproject(
        depth, ~target_mask, K
    )

    target_base = transform_points(
        target_camera,
        T_base_camera,
    )

    background_base = transform_points(
        background_camera,
        T_base_camera,
    )

    camera_error = np.linalg.norm(
        target_camera - recorded_target_camera,
        axis=1,
    )

    base_error = np.linalg.norm(
        target_base - recorded_target_base,
        axis=1,
    )

    print(
        "Target reconstruction camera RMS [mm]:",
        round(np.sqrt(np.mean(camera_error**2)) * 1000, 6),
    )

    print(
        "Target reconstruction base RMS [mm]:",
        round(np.sqrt(np.mean(base_error**2)) * 1000, 6),
    )

    if (
        np.max(camera_error) > 0.001
        or np.max(base_error) > 0.001
    ):
        raise ValueError(
            "Reconstructed cloud does not match saved artifacts"
        )

    return SceneGeometry(
        target_points_base=target_base,
        background_points_base=background_base,
        K=K,
        T_base_camera=T_base_camera,
        T_base_table=T_base_table,
        depth_valid_mask=valid_depth,
        target_mask=target_mask,
    )
