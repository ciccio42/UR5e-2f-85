#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
import yaml

import cv2
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

# =====================================================================
# Paths / project imports
# =====================================================================

REPO_ROOT = Path("/home/ros2_ws/src/UR5e-2f-85")

SEEDO_DIR = (
    REPO_ROOT
    / "ai_controller"
    / "ai_controller"
    / "models"
    / "seedo_controller"
)

if str(SEEDO_DIR) not in sys.path:
    sys.path.insert(0, str(SEEDO_DIR))

from inspect_pkl import load_pickle, get_trajectory_step  # noqa: E402

from ai_controller.utils.grasping.grasp_planner_factory import (  # noqa: E402
    build_grasp_planner,
)


OUTPUT_ROOT = (
    REPO_ROOT
    / ".runtime"
    / "graspgenx_pipeline_test"
)

DEFAULT_K = np.array(
    [
        [363.8071594238281, 0.0, 335.86553955078125],
        [0.0, 363.8071594238281, 183.81210327148438],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)

# Validated transform used by the existing offline frame-7 checks.
# T_tcp_camera maps points expressed in the eye-in-hand optical frame
# into tcp_link composition as:
#     T_base_camera = T_base_tcp @ T_tcp_camera
T_TCP_CAMERA = np.array(
    [
        [0.999262529, -0.0383979045, 0.0, -0.0209004103],
        [0.0383979045, 0.999262529, 0.0, -0.0681027558],
        [0.0, 0.0, 1.0, -0.1455],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)

def load_transform_yaml(
    path: Path,
) -> np.ndarray:
    path = (
        Path(path)
        .expanduser()
        .resolve()
    )

    if not path.is_file():
        raise FileNotFoundError(
            f"Transform YAML does not exist: {path}"
        )

    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        data = yaml.safe_load(
            stream
        )

    T = np.eye(
        4,
        dtype=np.float64,
    )

    T[:3, :3] = np.asarray(
        data["rotation"],
        dtype=np.float64,
    )

    T[:3, 3] = np.asarray(
        data["translation"],
        dtype=np.float64,
    )

    return T

# =====================================================================
# Rollout helpers
# =====================================================================

def get_gripper_rgbd(
    obs: dict,
) -> tuple[np.ndarray, np.ndarray, str, str]:
    candidates = [
        ("eye_in_hand_image", "eye_in_hand_depth"),
        ("camera_gripper_image", "camera_gripper_depth"),
    ]

    for image_key, depth_key in candidates:
        if image_key in obs and depth_key in obs:
            return (
                np.asarray(obs[image_key]),
                np.asarray(obs[depth_key], dtype=np.float32),
                image_key,
                depth_key,
            )

    raise RuntimeError(
        "No supported gripper RGB-D pair found. Expected either "
        "'eye_in_hand_image' + 'eye_in_hand_depth' or "
        "'camera_gripper_image' + 'camera_gripper_depth'."
    )


def pose_to_transform(
    position: np.ndarray,
    quaternion_xyzw: np.ndarray,
) -> np.ndarray:
    position = np.asarray(position, dtype=np.float64)
    quaternion_xyzw = np.asarray(quaternion_xyzw, dtype=np.float64)

    if position.shape != (3,):
        raise ValueError(f"position must have shape (3,), got {position.shape}")

    if quaternion_xyzw.shape != (4,):
        raise ValueError(
            "quaternion_xyzw must have shape (4,), "
            f"got {quaternion_xyzw.shape}"
        )

    norm = float(np.linalg.norm(quaternion_xyzw))
    if norm <= 0.0:
        raise ValueError("Quaternion has zero norm.")

    quaternion_xyzw = quaternion_xyzw / norm

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_quat(quaternion_xyzw).as_matrix()
    T[:3, 3] = position
    return T


# =====================================================================
# Point cloud / table estimation
# =====================================================================

def depth_to_camera_point_cloud(
    depth: np.ndarray,
    K: np.ndarray,
    min_depth: float,
    max_depth: float,
    border_fraction: float = 0.05,
) -> np.ndarray:
    depth = np.asarray(depth, dtype=np.float32)
    K = np.asarray(K, dtype=np.float64)

    if depth.ndim != 2:
        raise ValueError("depth must have shape (H, W).")

    h, w = depth.shape

    valid = (
        np.isfinite(depth)
        & (depth >= min_depth)
        & (depth <= max_depth)
    )

    border_y = int(round(h * border_fraction))
    border_x = int(round(w * border_fraction))

    if border_y > 0:
        valid[:border_y, :] = False
        valid[h - border_y :, :] = False

    if border_x > 0:
        valid[:, :border_x] = False
        valid[:, w - border_x :] = False

    v, u = np.where(valid)

    if len(u) == 0:
        raise RuntimeError("Depth image produced zero valid scene points.")

    z = depth[v, u].astype(np.float64)

    fx = float(K[0, 0])
    fy = float(K[1, 1])
    cx = float(K[0, 2])
    cy = float(K[1, 2])

    x = (u.astype(np.float64) - cx) * z / fx
    y = (v.astype(np.float64) - cy) * z / fy

    return np.stack([x, y, z], axis=1)


def transform_points(
    points: np.ndarray,
    T_target_source: np.ndarray,
) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    T = np.asarray(T_target_source, dtype=np.float64)

    return (
        points @ T[:3, :3].T
        + T[:3, 3]
    )


def estimate_horizontal_table_plane(
    points_base: np.ndarray,
    seed: int = 42,
    iterations: int = 1200,
    distance_threshold_m: float = 0.003,
    min_abs_normal_z: float = 0.80,
) -> tuple[np.ndarray, float, np.ndarray, np.ndarray]:
    """
    Fit the dominant approximately-horizontal plane:
        normal . p + d = 0

    Returns:
        normal_base (3,), d, origin_base (3,), inlier_mask (N,)
    """
    points = np.asarray(points_base, dtype=np.float64)

    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_base must have shape (N, 3).")

    if len(points) < 3:
        raise RuntimeError("Not enough scene points for table estimation.")

    rng = np.random.default_rng(seed)

    best_mask = None
    best_count = 0

    for _ in range(iterations):
        idx = rng.choice(
            len(points),
            size=3,
            replace=False,
        )

        p0, p1, p2 = points[idx]

        normal = np.cross(
            p1 - p0,
            p2 - p0,
        )

        norm = float(np.linalg.norm(normal))
        if norm < 1e-9:
            continue

        normal /= norm

        if abs(float(normal[2])) < min_abs_normal_z:
            continue

        d = -float(np.dot(normal, p0))

        distances = np.abs(
            points @ normal + d
        )

        mask = distances <= distance_threshold_m
        count = int(mask.sum())

        if count > best_count:
            best_count = count
            best_mask = mask

    if best_mask is None or best_count < 3:
        raise RuntimeError(
            "Could not estimate a horizontal table plane."
        )

    inlier_points = points[best_mask]

    origin = inlier_points.mean(axis=0)

    centered = inlier_points - origin

    _, _, vh = np.linalg.svd(
        centered,
        full_matrices=False,
    )

    normal = vh[-1]
    normal /= np.linalg.norm(normal)

    if normal[2] < 0.0:
        normal = -normal

    d = -float(
        np.dot(
            normal,
            origin,
        )
    )

    # Recompute inliers against the refined plane.
    distances = np.abs(
        points @ normal + d
    )

    refined_mask = (
        distances
        <= distance_threshold_m
    )

    refined_points = points[
        refined_mask
    ]

    if len(refined_points) < 3:
        raise RuntimeError(
            "Refined table plane has too few inliers."
        )

    origin = refined_points.mean(
        axis=0
    )

    d = -float(
        np.dot(
            normal,
            origin,
        )
    )

    return (
        normal.astype(np.float64),
        d,
        origin.astype(np.float64),
        refined_mask,
    )


def table_transform_from_plane(
    normal_base: np.ndarray,
    origin_base: np.ndarray,
) -> np.ndarray:
    z_axis = np.asarray(
        normal_base,
        dtype=np.float64,
    )
    z_axis /= np.linalg.norm(
        z_axis
    )

    if z_axis[2] < 0.0:
        z_axis = -z_axis

    x_reference = np.array(
        [1.0, 0.0, 0.0],
        dtype=np.float64,
    )

    x_axis = (
        x_reference
        - np.dot(
            x_reference,
            z_axis,
        )
        * z_axis
    )

    if np.linalg.norm(x_axis) < 1e-6:
        x_reference = np.array(
            [0.0, 1.0, 0.0],
            dtype=np.float64,
        )
        x_axis = (
            x_reference
            - np.dot(
                x_reference,
                z_axis,
            )
            * z_axis
        )

    x_axis /= np.linalg.norm(
        x_axis
    )

    y_axis = np.cross(
        z_axis,
        x_axis,
    )
    y_axis /= np.linalg.norm(
        y_axis
    )

    # Re-orthogonalize X.
    x_axis = np.cross(
        y_axis,
        z_axis,
    )
    x_axis /= np.linalg.norm(
        x_axis
    )

    T = np.eye(
        4,
        dtype=np.float64,
    )

    T[:3, :3] = np.column_stack(
        [
            x_axis,
            y_axis,
            z_axis,
        ]
    )

    T[:3, 3] = np.asarray(
        origin_base,
        dtype=np.float64,
    )

    return T


# =====================================================================
# Geometry diagnostics
# =====================================================================

def bbox_corners(
    minimum: np.ndarray,
    maximum: np.ndarray,
) -> np.ndarray:
    return np.asarray(
        [
            [x, y, z]
            for x in (minimum[0], maximum[0])
            for y in (minimum[1], maximum[1])
            for z in (minimum[2], maximum[2])
        ],
        dtype=np.float64,
    )


def evaluate_candidates(
    planner,
    grasps_base: np.ndarray,
    confidences: np.ndarray,
    branch_tags: np.ndarray,
    target_pc_base: np.ndarray,
    table_normal: np.ndarray,
    table_d: float,
) -> dict[str, np.ndarray]:
    grasps = np.asarray(
        grasps_base,
        dtype=np.float64,
    )

    confidences = np.asarray(
        confidences,
        dtype=np.float64,
    )

    tags = np.asarray(
        branch_tags,
        dtype=str,
    )

    target_centroid = np.asarray(
        target_pc_base,
        dtype=np.float64,
    ).mean(axis=0)

    local_bbox = bbox_corners(
        planner.ROBOTIQ_BBOX_MIN,
        planner.ROBOTIQ_BBOX_MAX,
    )

    minimum_clearances = np.empty(
        len(grasps),
        dtype=np.float64,
    )

    fingertip_clearances = np.empty(
        len(grasps),
        dtype=np.float64,
    )

    target_distances = np.empty(
        len(grasps),
        dtype=np.float64,
    )

    approach_tilts_deg = np.empty(
        len(grasps),
        dtype=np.float64,
    )

    fingertip_centers = np.empty(
        (len(grasps), 3),
        dtype=np.float64,
    )

    down = np.array(
        [0.0, 0.0, -1.0],
        dtype=np.float64,
    )

    for index, grasp in enumerate(grasps):
        R = grasp[:3, :3]
        t = grasp[:3, 3]

        bbox_base = (
            local_bbox @ R.T + t
        )

        minimum_clearances[index] = float(
            (
                bbox_base @ table_normal
                + table_d
            ).min()
        )

        approach = R[:, 2].copy()
        approach /= np.linalg.norm(
            approach
        )

        tip = (
            t
            + planner.FINGERTIP_DEPTH
            * approach
        )

        fingertip_centers[index] = tip

        fingertip_clearances[index] = float(
            np.dot(
                tip,
                table_normal,
            )
            + table_d
        )

        target_distances[index] = float(
            np.linalg.norm(
                tip
                - target_centroid
            )
        )

        approach_tilts_deg[index] = float(
            np.degrees(
                np.arccos(
                    np.clip(
                        np.dot(
                            approach,
                            down,
                        ),
                        -1.0,
                        1.0,
                    )
                )
            )
        )

    safe_mask = (
        minimum_clearances
        >= planner.TABLE_SAFETY_MARGIN
    )

    safe_obb_mask = (
        safe_mask
        & (tags == "obb")
    )

    return {
        "minimum_clearances": minimum_clearances,
        "fingertip_clearances": fingertip_clearances,
        "target_distances": target_distances,
        "approach_tilts_deg": approach_tilts_deg,
        "fingertip_centers": fingertip_centers,
        "safe_mask": safe_mask,
        "safe_obb_mask": safe_obb_mask,
        "confidences": confidences,
        "branch_tags": tags,
    }


def transform_poses(
    poses: np.ndarray,
    T_target_source: np.ndarray,
) -> np.ndarray:
    poses = np.asarray(
        poses,
        dtype=np.float64,
    )

    return np.stack(
        [
            T_target_source @ pose
            for pose in poses
        ],
        axis=0,
    )


# =====================================================================
# Visualization
# =====================================================================

def save_depth_preview(
    depth: np.ndarray,
    path: Path,
) -> None:
    depth = np.asarray(
        depth,
        dtype=np.float32,
    )

    valid = (
        np.isfinite(depth)
        & (depth > 0.0)
    )

    preview = np.zeros(
        depth.shape,
        dtype=np.uint8,
    )

    if valid.any():
        values = depth[valid]

        low = float(
            np.percentile(
                values,
                2.0,
            )
        )
        high = float(
            np.percentile(
                values,
                98.0,
            )
        )

        if high > low:
            normalized = (
                (depth - low)
                / (high - low)
            )

            normalized = np.clip(
                normalized,
                0.0,
                1.0,
            )

            preview[valid] = (
                255.0
                * (1.0 - normalized[valid])
            ).astype(np.uint8)

    Image.fromarray(
        preview
    ).save(
        path
    )


def project_camera_point(
    point_camera: np.ndarray,
    K: np.ndarray,
) -> tuple[int, int] | None:
    p = np.asarray(
        point_camera,
        dtype=np.float64,
    )

    if (
        p.shape != (3,)
        or not np.isfinite(p).all()
        or p[2] <= 0.0
    ):
        return None

    u = (
        K[0, 0]
        * p[0]
        / p[2]
        + K[0, 2]
    )

    v = (
        K[1, 1]
        * p[1]
        / p[2]
        + K[1, 2]
    )

    if not np.isfinite([u, v]).all():
        return None

    return (
        int(round(float(u))),
        int(round(float(v))),
    )


def draw_gripper_projection(
    grasp_camera: np.ndarray,
    K: np.ndarray,
    out_image: np.ndarray,
) -> None:
    grasp = np.asarray(
        grasp_camera,
        dtype=np.float64,
    )

    origin = grasp[:3, 3]
    x_axis = grasp[:3, 0]
    z_axis = grasp[:3, 2]

    fingertip_depth = 0.136
    half_opening = 0.085 / 2.0

    fingertip_center = (
        origin
        + fingertip_depth
        * z_axis
    )

    left_tip = (
        fingertip_center
        - half_opening
        * x_axis
    )

    right_tip = (
        fingertip_center
        + half_opening
        * x_axis
    )

    mid_depth = (
        fingertip_depth
        * 0.60
    )

    left_mid = (
        origin
        + mid_depth
        * z_axis
        - half_opening
        * x_axis
    )

    right_mid = (
        origin
        + mid_depth
        * z_axis
        + half_opening
        * x_axis
    )

    points = [
        project_camera_point(
            p,
            K,
        )
        for p in (
            origin,
            fingertip_center,
            left_tip,
            right_tip,
            left_mid,
            right_mid,
        )
    ]

    if any(
        p is None
        for p in points
    ):
        return

    (
        p_origin,
        p_center,
        p_left_tip,
        p_right_tip,
        p_left_mid,
        p_right_mid,
    ) = points

    cv2.line(
        out_image,
        p_origin,
        p_center,
        (255, 255, 0),
        3,
    )

    cv2.line(
        out_image,
        p_origin,
        p_left_mid,
        (255, 165, 0),
        3,
    )

    cv2.line(
        out_image,
        p_left_mid,
        p_left_tip,
        (255, 165, 0),
        5,
    )

    cv2.line(
        out_image,
        p_origin,
        p_right_mid,
        (255, 165, 0),
        3,
    )

    cv2.line(
        out_image,
        p_right_mid,
        p_right_tip,
        (255, 165, 0),
        5,
    )

    cv2.line(
        out_image,
        p_left_tip,
        p_right_tip,
        (0, 255, 255),
        3,
    )

    cv2.circle(
        out_image,
        p_origin,
        5,
        (255, 0, 0),
        -1,
    )

    cv2.circle(
        out_image,
        p_center,
        6,
        (0, 255, 0),
        -1,
    )


def draw_pipeline_overlay(
    image_rgb: np.ndarray,
    mask: np.ndarray,
    semantic_point: np.ndarray,
    selected_grasp_camera: np.ndarray,
    selected_index: int,
    selected_confidence: float,
    selected_clearance: float,
    selected_target_distance: float,
    output_path: Path,
    K: np.ndarray,
) -> None:
    output = np.asarray(
        image_rgb,
        dtype=np.uint8,
    ).copy()

    mask_color = np.zeros_like(
        output
    )
    mask_color[..., 1] = 255

    output = np.where(
        mask[..., None],
        (
            0.70 * output
            + 0.30 * mask_color
        ).astype(np.uint8),
        output,
    )

    u = int(
        round(
            float(
                semantic_point[0]
            )
        )
    )

    v = int(
        round(
            float(
                semantic_point[1]
            )
        )
    )

    cv2.circle(
        output,
        (u, v),
        7,
        (255, 0, 255),
        -1,
    )

    cv2.circle(
        output,
        (u, v),
        12,
        (255, 255, 255),
        2,
    )

    draw_gripper_projection(
        selected_grasp_camera,
        K,
        output,
    )

    lines = [
        f"selected={selected_index}",
        f"confidence={selected_confidence:.4f}",
        f"table clearance={selected_clearance * 1000.0:.2f} mm",
        f"target distance={selected_target_distance * 1000.0:.2f} mm",
    ]

    y = 28

    for line in lines:
        cv2.putText(
            output,
            line,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (255, 255, 255),
            3,
            cv2.LINE_AA,
        )

        cv2.putText(
            output,
            line,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (20, 20, 20),
            1,
            cv2.LINE_AA,
        )

        y += 25

    Image.fromarray(
        output
    ).save(
        output_path
    )


# =====================================================================
# Capture the REAL planner internals during one plan() call
# =====================================================================

def install_capture_wrappers(
    planner,
    captured: dict,
) -> None:
    # Capture SAM's actual first predict() call.
    predictor = planner._get_sam_predictor()
    original_sam_predict = predictor.predict

    def wrapped_sam_predict(*args, **kwargs):
        output = original_sam_predict(
            *args,
            **kwargs,
        )

        masks, scores, logits = output

        captured["sam_masks"] = np.asarray(
            masks
        ).copy()

        captured["sam_scores"] = np.asarray(
            scores,
            dtype=np.float64,
        ).copy()

        captured["sam_logits"] = np.asarray(
            logits
        ).copy()

        captured["sam_best_index"] = int(
            np.argmax(
                scores
            )
        )

        captured["target_mask"] = np.asarray(
            masks[
                captured["sam_best_index"]
            ],
            dtype=bool,
        ).copy()

        return output

    predictor.predict = wrapped_sam_predict

    original_predict_semantic = (
        planner._predict_semantic_point
    )

    def wrapped_predict_semantic(*args, **kwargs):
        point = original_predict_semantic(
            *args,
            **kwargs,
        )

        captured[
            "semantic_point_px"
        ] = np.asarray(
            point,
            dtype=np.float64,
        ).copy()

        return point

    planner._predict_semantic_point = (
        wrapped_predict_semantic
    )

    original_mask_to_pc = (
        planner._mask_to_camera_point_cloud
    )

    def wrapped_mask_to_pc(*args, **kwargs):
        pc = original_mask_to_pc(
            *args,
            **kwargs,
        )

        captured[
            "target_point_cloud_camera"
        ] = np.asarray(
            pc,
            dtype=np.float64,
        ).copy()

        return pc

    planner._mask_to_camera_point_cloud = (
        wrapped_mask_to_pc
    )

    original_generate = (
        planner._generate_grasp_candidates_base
    )

    def wrapped_generate(*args, **kwargs):
        output = original_generate(
            *args,
            **kwargs,
        )

        (
            grasps_base,
            confidences,
            branch_tags,
            target_pc_base,
        ) = output

        captured[
            "grasps_base"
        ] = np.asarray(
            grasps_base,
            dtype=np.float64,
        ).copy()

        captured[
            "confidences"
        ] = np.asarray(
            confidences,
            dtype=np.float64,
        ).copy()

        captured[
            "branch_tags"
        ] = np.asarray(
            branch_tags,
            dtype=str,
        ).copy()

        captured[
            "target_point_cloud_base"
        ] = np.asarray(
            target_pc_base,
            dtype=np.float64,
        ).copy()

        return output

    planner._generate_grasp_candidates_base = (
        wrapped_generate
    )

    original_select = (
        planner._select_safe_grasp
    )

    def wrapped_select(*args, **kwargs):
        index = original_select(
            *args,
            **kwargs,
        )

        captured[
            "selected_index"
        ] = int(
            index
        )

        return index

    planner._select_safe_grasp = (
        wrapped_select
    )


# =====================================================================
# Checks / reporting
# =====================================================================

def rotation_metrics(
    rotations: np.ndarray,
) -> tuple[float, float]:
    rotations = np.asarray(
        rotations,
        dtype=np.float64,
    )

    max_orth_error = 0.0
    max_det_error = 0.0

    for R in rotations:
        orth_error = float(
            np.linalg.norm(
                R.T @ R
                - np.eye(3)
            )
        )

        det_error = abs(
            float(
                np.linalg.det(R)
            )
            - 1.0
        )

        max_orth_error = max(
            max_orth_error,
            orth_error,
        )

        max_det_error = max(
            max_det_error,
            det_error,
        )

    return (
        max_orth_error,
        max_det_error,
    )


def write_candidates_csv(
    path: Path,
    diagnostics: dict[str, np.ndarray],
) -> None:
    with path.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as stream:
        writer = csv.writer(
            stream
        )

        writer.writerow(
            [
                "index",
                "branch",
                "confidence",
                "safe",
                "minimum_clearance_m",
                "fingertip_clearance_m",
                "target_distance_m",
                "approach_tilt_deg",
            ]
        )

        for index in range(
            len(
                diagnostics[
                    "confidences"
                ]
            )
        ):
            writer.writerow(
                [
                    index,
                    diagnostics[
                        "branch_tags"
                    ][index],
                    float(
                        diagnostics[
                            "confidences"
                        ][index]
                    ),
                    bool(
                        diagnostics[
                            "safe_mask"
                        ][index]
                    ),
                    float(
                        diagnostics[
                            "minimum_clearances"
                        ][index]
                    ),
                    float(
                        diagnostics[
                            "fingertip_clearances"
                        ][index]
                    ),
                    float(
                        diagnostics[
                            "target_distances"
                        ][index]
                    ),
                    float(
                        diagnostics[
                            "approach_tilts_deg"
                        ][index]
                    ),
                ]
            )


def main() -> None:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--pkl",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--step",
        type=int,
        default=7,
    )

    parser.add_argument(
        "--instruction",
        type=str,
        required=True,
    )

    parser.add_argument(
        "--base-to-table-transform",
        type=Path,
        default=None,
        help=(
            "Optional production base_link -> table_0 YAML. "
            "When supplied, this transform is passed to "
            "GraspGenXPlanner instead of the plane fitted "
            "from the depth image."
        ),
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
    )

    args = parser.parse_args()

    output_dir = (
        args.output_dir
        if args.output_dir is not None
        else (
            OUTPUT_ROOT
            / f"full_plan_step_{args.step:03d}"
        )
    )

    output_dir = (
        output_dir
        .expanduser()
        .resolve()
    )

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    print("=" * 78)
    print("GRASPGEN-X FULL OFFLINE PLAN INTEGRATION TEST")
    print("=" * 78)
    print("[TEST] PKL:", args.pkl)
    print("[TEST] Step:", args.step)
    print("[TEST] Instruction:", args.instruction)
    print("[TEST] Output:", output_dir)

    # -----------------------------------------------------------------
    # Load rollout frame
    # -----------------------------------------------------------------

    data = load_pickle(
        args.pkl
    )

    trajectory = data[
        "traj"
    ]

    step = get_trajectory_step(
        trajectory,
        args.step,
    )

    obs = step[
        "obs"
    ]

    (
        image_bgr,
        depth,
        rgb_key,
        depth_key,
    ) = get_gripper_rgbd(
        obs
    )

    image_rgb = cv2.cvtColor(
        image_bgr,
        cv2.COLOR_BGR2RGB,
    )

    if "eef_pos" not in obs:
        raise RuntimeError(
            "Rollout observation has no 'eef_pos'."
        )

    if "eef_quat" not in obs:
        raise RuntimeError(
            "Rollout observation has no 'eef_quat'."
        )

    current_tcp_position = np.asarray(
        obs["eef_pos"],
        dtype=np.float64,
    )

    current_tcp_orientation = np.asarray(
        obs["eef_quat"],
        dtype=np.float64,
    )

    T_base_tcp_current = pose_to_transform(
        current_tcp_position,
        current_tcp_orientation,
    )

    T_base_camera = (
        T_base_tcp_current
        @ T_TCP_CAMERA
    )

    Image.fromarray(
        image_rgb
    ).save(
        output_dir
        / "input_rgb.png"
    )

    np.save(
        output_dir
        / "input_depth.npy",
        depth,
    )

    save_depth_preview(
        depth,
        output_dir
        / "input_depth_preview.png",
    )

    # -----------------------------------------------------------------
    # Automatic table estimation from this frame
    # -----------------------------------------------------------------

    scene_pc_camera = (
        depth_to_camera_point_cloud(
            depth=depth,
            K=DEFAULT_K,
            min_depth=0.15,
            max_depth=0.60,
            border_fraction=0.05,
        )
    )

    scene_pc_base = (
        transform_points(
            scene_pc_camera,
            T_base_camera,
        )
    )

    (
        table_normal,
        table_d,
        table_origin,
        table_inlier_mask,
    ) = estimate_horizontal_table_plane(
        scene_pc_base,
        seed=42,
        iterations=1200,
        distance_threshold_m=0.003,
        min_abs_normal_z=0.80,
    )

    T_base_table = (
        table_transform_from_plane(
            table_normal,
            table_origin,
        )
    )

    fitted_T_base_table = (
        T_base_table.copy()
    )

    fitted_table_normal = (
        table_normal.copy()
    )

    fitted_table_d = float(
        table_d
    )

    fitted_table_origin = (
        table_origin.copy()
    )

    table_inlier_ratio = float(
        table_inlier_mask.mean()
    )

    table_inlier_points = scene_pc_base[
        table_inlier_mask
    ]

    np.savez_compressed(
        output_dir
        / "table_plane.npz",
        scene_point_cloud_base=(
            scene_pc_base.astype(
                np.float32
            )
        ),
        table_inlier_mask=(
            table_inlier_mask.astype(
                np.uint8
            )
        ),
        table_normal_base=(
            table_normal
        ),
        table_d=np.asarray(
            table_d,
            dtype=np.float64,
        ),
        table_origin_base=(
            table_origin
        ),
        T_base_table=(
            T_base_table
        ),
    )

    print()
    print("[TABLE] normal:", table_normal)
    print("[TABLE] d:", table_d)
    print(
        "[TABLE] inliers:",
        int(
            table_inlier_mask.sum()
        ),
        "/",
        len(scene_pc_base),
    )
    print(
        "[TABLE] ratio:",
        f"{100.0 * table_inlier_ratio:.2f}%",
    )
    print(
        "[TABLE] mean:",
        table_origin,
    )
    print(
        "[TABLE] inlier Z:",
        float(
            table_inlier_points[
                :,
                2,
            ].min()
        ),
        "->",
        float(
            table_inlier_points[
                :,
                2,
            ].max()
        ),
    )

    # -----------------------------------------------------------------
    # Select the table transform actually passed to GraspGen-X.
    #
    # If a production base_link -> table_0 transform is supplied,
    # use exactly that transform for grasp safety evaluation.
    #
    # The depth-fitted plane above remains diagnostic only.
    # -----------------------------------------------------------------

    table_source = "depth_fit"

    if args.base_to_table_transform is not None:
        T_base_table = load_transform_yaml(
            args.base_to_table_transform
        )

        table_source = "production_tf"

        table_normal = (
            T_base_table[
                :3,
                2,
            ].copy()
        )

        table_normal /= np.linalg.norm(
            table_normal
        )

        if table_normal[2] < 0.0:
            table_normal = -table_normal

        table_origin = (
            T_base_table[
                :3,
                3,
            ].copy()
        )

        table_d = -float(
            np.dot(
                table_normal,
                table_origin,
            )
        )

        normal_angle_deg = float(
            np.degrees(
                np.arccos(
                    np.clip(
                        np.dot(
                            fitted_table_normal,
                            table_normal,
                        ),
                        -1.0,
                        1.0,
                    )
                )
            )
        )

        fitted_origin_to_runtime_plane = abs(
            float(
                np.dot(
                    table_normal,
                    fitted_table_origin,
                )
                + table_d
            )
        )

        print()
        print(
            "[TABLE] Using production TF for "
            "GraspGen-X collision checks."
        )
        print(
            "[TABLE] Transform YAML:",
            args.base_to_table_transform,
        )
        print(
            "[TABLE] Runtime normal:",
            table_normal,
        )
        print(
            "[TABLE] Runtime origin:",
            table_origin,
        )
        print(
            "[TABLE] Runtime d:",
            table_d,
        )
        print(
            "[TABLE] Fitted/runtime normal angle:",
            f"{normal_angle_deg:.6f} deg",
        )
        print(
            "[TABLE] Fitted origin -> runtime plane:",
            f"{fitted_origin_to_runtime_plane * 1000.0:.3f} mm",
        )

        np.savez_compressed(
            output_dir
            / "runtime_table_transform.npz",
            T_base_table=T_base_table,
            table_normal_base=table_normal,
            table_origin_base=table_origin,
            table_d=np.asarray(
                table_d,
                dtype=np.float64,
            ),
            fitted_T_base_table=(
                fitted_T_base_table
            ),
            fitted_table_normal_base=(
                fitted_table_normal
            ),
            fitted_table_origin_base=(
                fitted_table_origin
            ),
            fitted_table_d=np.asarray(
                fitted_table_d,
                dtype=np.float64,
            ),
        )

    # -----------------------------------------------------------------
    # Build the REAL backend and capture one real plan() execution
    # -----------------------------------------------------------------

    planner = build_grasp_planner(
        {
            "backend": "graspgenx",
        }
    )

    captured: dict = {}

    install_capture_wrappers(
        planner,
        captured,
    )

    result = planner.plan(
        rgb_image=image_rgb,
        depth_image=depth,
        camera_matrix=DEFAULT_K,
        T_base_camera=T_base_camera,
        T_base_table=T_base_table,
        current_tcp_position=(
            current_tcp_position
        ),
        current_tcp_orientation=(
            current_tcp_orientation
        ),
        grasp_instruction=(
            args.instruction
        ),
        artifacts_dir=(
            output_dir
        ),
    )

    required_capture_keys = {
        "semantic_point_px",
        "sam_masks",
        "sam_scores",
        "sam_best_index",
        "target_mask",
        "target_point_cloud_camera",
        "target_point_cloud_base",
        "grasps_base",
        "confidences",
        "branch_tags",
        "selected_index",
    }

    missing_capture = (
        required_capture_keys
        - set(
            captured
        )
    )

    if missing_capture:
        raise RuntimeError(
            "Missing captured planner diagnostics: "
            f"{sorted(missing_capture)}"
        )

    semantic_point = captured[
        "semantic_point_px"
    ]

    sam_scores = captured[
        "sam_scores"
    ]

    best_mask_index = int(
        captured[
            "sam_best_index"
        ]
    )

    target_mask = captured[
        "target_mask"
    ]

    target_pc_camera = captured[
        "target_point_cloud_camera"
    ]

    target_pc_base = captured[
        "target_point_cloud_base"
    ]

    grasps_base = captured[
        "grasps_base"
    ]

    confidences = captured[
        "confidences"
    ]

    branch_tags = captured[
        "branch_tags"
    ]

    selected_index = int(
        captured[
            "selected_index"
        ]
    )

    # -----------------------------------------------------------------
    # Candidate diagnostics
    # -----------------------------------------------------------------

    candidate_diag = evaluate_candidates(
        planner=planner,
        grasps_base=grasps_base,
        confidences=confidences,
        branch_tags=branch_tags,
        target_pc_base=target_pc_base,
        table_normal=table_normal,
        table_d=table_d,
    )

    selected_grasp_base = (
        grasps_base[
            selected_index
        ]
    )

    selected_confidence = float(
        confidences[
            selected_index
        ]
    )

    selected_branch = str(
        branch_tags[
            selected_index
        ]
    )

    selected_clearance = float(
        candidate_diag[
            "minimum_clearances"
        ][
            selected_index
        ]
    )

    selected_tip_clearance = float(
        candidate_diag[
            "fingertip_clearances"
        ][
            selected_index
        ]
    )

    selected_target_distance = float(
        candidate_diag[
            "target_distances"
        ][
            selected_index
        ]
    )

    selected_approach_tilt = float(
        candidate_diag[
            "approach_tilts_deg"
        ][
            selected_index
        ]
    )

    selected_tip_base = (
        candidate_diag[
            "fingertip_centers"
        ][
            selected_index
        ]
    )

    selected_closing_base = (
        selected_grasp_base[
            :3,
            0,
        ]
    )

    selected_approach_base = (
        selected_grasp_base[
            :3,
            2,
        ]
    )

    target_centroid_camera = (
        target_pc_camera.mean(
            axis=0
        )
    )

    target_centroid_base = (
        target_pc_base.mean(
            axis=0
        )
    )

    target_centroid_table_clearance = float(
        np.dot(
            target_centroid_base,
            table_normal,
        )
        + table_d
    )

    # GraspGen-X gripper base -> robot tcp_link.
    T_gripper_tcp = np.eye(
        4,
        dtype=np.float64,
    )
    T_gripper_tcp[
        2,
        3,
    ] = planner.FINGERTIP_DEPTH

    T_base_tcp_expected = (
        selected_grasp_base
        @ T_gripper_tcp
    )

    T_base_tcp_result = np.eye(
        4,
        dtype=np.float64,
    )
    T_base_tcp_result[
        :3,
        :3,
    ] = Rotation.from_quat(
        result.tcp_orientation_base
    ).as_matrix()
    T_base_tcp_result[
        :3,
        3,
    ] = result.tcp_position_base

    current_to_target_distance = float(
        np.linalg.norm(
            result.tcp_position_base
            - current_tcp_position
        )
    )

    T_camera_base = np.linalg.inv(
        T_base_camera
    )

    grasps_camera = (
        transform_poses(
            grasps_base,
            T_camera_base,
        )
    )

    selected_grasp_camera = (
        grasps_camera[
            selected_index
        ]
    )

    # -----------------------------------------------------------------
    # Numeric checks
    # -----------------------------------------------------------------

    checks: dict[str, dict] = {}

    def check(
        name: str,
        condition: bool,
        value=None,
        expected=None,
    ) -> None:
        passed = bool(
            condition
        )

        checks[
            name
        ] = {
            "passed": passed,
            "value": (
                value.tolist()
                if isinstance(
                    value,
                    np.ndarray,
                )
                else value
            ),
            "expected": expected,
        }

        print(
            "[PASS]"
            if passed
            else "[FAIL]",
            name,
            ""
            if value is None
            else f"value={value}",
        )

    # Transform / calibration sanity.
    camera_orth_error, camera_det_error = (
        rotation_metrics(
            T_base_camera[
                None,
                :3,
                :3,
            ]
        )
    )

    check(
        "T_base_camera_is_rigid",
        (
            camera_orth_error < 1e-6
            and camera_det_error < 1e-6
        ),
        {
            "orth_error": camera_orth_error,
            "det_error": camera_det_error,
        },
        "< 1e-6",
    )

    table_orth_error, table_det_error = (
        rotation_metrics(
            T_base_table[
                None,
                :3,
                :3,
            ]
        )
    )

    check(
        "T_base_table_is_rigid",
        (
            table_orth_error < 1e-6
            and table_det_error < 1e-6
        ),
        {
            "orth_error": table_orth_error,
            "det_error": table_det_error,
        },
        "< 1e-6",
    )

    check(
        "table_normal_is_near_vertical",
        abs(
            float(
                table_normal[2]
            )
        ) >= 0.95,
        table_normal,
        "|nz| >= 0.95",
    )

    check(
        "table_inlier_ratio_is_plausible",
        table_inlier_ratio >= 0.50,
        table_inlier_ratio,
        ">= 0.50",
    )

    # Semantic / segmentation.
    height, width = depth.shape

    check(
        "semantic_point_inside_image",
        (
            0.0
            <= semantic_point[0]
            < width
            and 0.0
            <= semantic_point[1]
            < height
        ),
        semantic_point,
        f"x in [0,{width}), y in [0,{height})",
    )

    check(
        "sam_returned_masks",
        len(sam_scores) > 0,
        int(
            len(
                sam_scores
            )
        ),
        "> 0",
    )

    check(
        "sam_best_index_matches_argmax",
        (
            best_mask_index
            == int(
                np.argmax(
                    sam_scores
                )
            )
        ),
        best_mask_index,
        "argmax(scores)",
    )

    check(
        "sam_mask_has_points",
        int(
            target_mask.sum()
        ) >= 100,
        int(
            target_mask.sum()
        ),
        ">= 100 px",
    )

    # Target PC.
    check(
        "target_pc_camera_nonempty",
        len(
            target_pc_camera
        ) >= 100,
        int(
            len(
                target_pc_camera
            )
        ),
        ">= 100",
    )

    check(
        "target_pc_camera_finite",
        np.isfinite(
            target_pc_camera
        ).all(),
    )

    check(
        "target_pc_base_finite",
        np.isfinite(
            target_pc_base
        ).all(),
    )

    check(
        "target_pc_depth_in_range",
        (
            float(
                target_pc_camera[
                    :,
                    2,
                ].min()
            )
            >= planner.MIN_DEPTH
            - 1e-6
            and float(
                target_pc_camera[
                    :,
                    2,
                ].max()
            )
            <= planner.MAX_DEPTH
            + 1e-6
        ),
        {
            "min": float(
                target_pc_camera[
                    :,
                    2,
                ].min()
            ),
            "max": float(
                target_pc_camera[
                    :,
                    2,
                ].max()
            ),
        },
        [
            planner.MIN_DEPTH,
            planner.MAX_DEPTH,
        ],
    )

    check(
        "target_centroid_is_above_table",
        (
            target_centroid_table_clearance
            > 0.0
            and target_centroid_table_clearance
            < 0.10
        ),
        target_centroid_table_clearance,
        "0 < clearance < 0.10 m",
    )

    # Candidate output.
    check(
        "graspgenx_returned_candidates",
        len(
            grasps_base
        ) > 0,
        int(
            len(
                grasps_base
            )
        ),
        "> 0",
    )

    check(
        "candidate_lengths_match",
        (
            len(
                grasps_base
            )
            == len(
                confidences
            )
            == len(
                branch_tags
            )
        ),
        [
            len(
                grasps_base
            ),
            len(
                confidences
            ),
            len(
                branch_tags
            ),
        ],
        "all equal",
    )

    check(
        "diff_branch_present",
        np.any(
            branch_tags
            == "diff"
        ),
        int(
            np.sum(
                branch_tags
                == "diff"
            )
        ),
        "> 0",
    )

    check(
        "obb_branch_present",
        np.any(
            branch_tags
            == "obb"
        ),
        int(
            np.sum(
                branch_tags
                == "obb"
            )
        ),
        "> 0",
    )

    grasps_orth_error, grasps_det_error = (
        rotation_metrics(
            grasps_base[
                :,
                :3,
                :3,
            ]
        )
    )

    check(
        "all_grasp_rotations_valid",
        (
            grasps_orth_error < 1e-3
            and grasps_det_error < 1e-3
        ),
        {
            "max_orth_error": grasps_orth_error,
            "max_det_error": grasps_det_error,
        },
        "< 1e-3",
    )

    obb_mask = (
        branch_tags
        == "obb"
    )

    if np.any(
        obb_mask
    ):
        obb_approaches = (
            grasps_base[
                obb_mask,
                :3,
                2,
            ]
        )

        max_obb_down_error = float(
            np.max(
                np.linalg.norm(
                    obb_approaches
                    - np.array(
                        [
                            0.0,
                            0.0,
                            -1.0,
                        ]
                    ),
                    axis=1,
                )
            )
        )

    else:
        max_obb_down_error = float(
            "inf"
        )

    check(
        "obb_approaches_are_base_top_down",
        max_obb_down_error < 1e-4,
        max_obb_down_error,
        "< 1e-4 from [0,0,-1]",
    )

    safe_count = int(
        candidate_diag[
            "safe_mask"
        ].sum()
    )

    safe_obb_count = int(
        candidate_diag[
            "safe_obb_mask"
        ].sum()
    )

    check(
        "safe_candidate_exists",
        safe_count > 0,
        safe_count,
        "> 0",
    )

    check(
        "safe_obb_candidate_exists",
        safe_obb_count > 0,
        safe_obb_count,
        "> 0",
    )

    check(
        "selected_candidate_is_safe",
        bool(
            candidate_diag[
                "safe_mask"
            ][
                selected_index
            ]
        ),
        selected_clearance,
        (
            f">= {planner.TABLE_SAFETY_MARGIN} m"
        ),
    )

    check(
        "selected_candidate_is_obb",
        selected_branch == "obb",
        selected_branch,
        "obb",
    )

    check(
        "selected_table_clearance",
        (
            selected_clearance
            >= planner.TABLE_SAFETY_MARGIN
        ),
        selected_clearance,
        (
            f">= {planner.TABLE_SAFETY_MARGIN} m"
        ),
    )

    check(
        "selected_fingertip_above_table",
        selected_tip_clearance
        >= planner.TABLE_SAFETY_MARGIN,
        selected_tip_clearance,
        (
            f">= {planner.TABLE_SAFETY_MARGIN} m"
        ),
    )

    check(
        "selected_grasp_is_near_target",
        selected_target_distance
        <= 0.030,
        selected_target_distance,
        "<= 0.030 m",
    )

    check(
        "selected_approach_is_top_down",
        selected_approach_tilt
        <= 1e-3,
        selected_approach_tilt,
        "<= 0.001 deg",
    )

    check(
        "selected_closing_axis_is_horizontal",
        abs(
            float(
                selected_closing_base[
                    2
                ]
            )
        )
        <= 1e-4,
        selected_closing_base,
        "|closing_z| <= 1e-4",
    )

    # plan() result consistency.
    check(
        "plan_semantic_point_matches_pipeline",
        np.allclose(
            result.semantic_point_px,
            semantic_point,
            atol=1e-6,
        ),
    )

    check(
        "plan_confidence_matches_selected",
        np.isclose(
            result.confidence,
            selected_confidence,
            atol=1e-6,
        ),
        result.confidence,
        selected_confidence,
    )

    check(
        "plan_approach_matches_selected",
        np.allclose(
            result.approach_direction_base,
            selected_approach_base,
            atol=1e-6,
        ),
        result.approach_direction_base,
        selected_approach_base.tolist(),
    )

    check(
        "tcp_position_matches_136mm_conversion",
        np.allclose(
            result.tcp_position_base,
            T_base_tcp_expected[
                :3,
                3,
            ],
            atol=1e-6,
        ),
        result.tcp_position_base,
        T_base_tcp_expected[
            :3,
            3,
        ].tolist(),
    )

    result_quat_norm = float(
        np.linalg.norm(
            result.tcp_orientation_base
        )
    )

    check(
        "tcp_quaternion_is_normalized",
        abs(
            result_quat_norm
            - 1.0
        )
        < 1e-6,
        result_quat_norm,
        "1.0",
    )

    check(
        "tcp_orientation_matches_grasp_rotation",
        np.allclose(
            T_base_tcp_result[
                :3,
                :3,
            ],
            selected_grasp_base[
                :3,
                :3,
            ],
            atol=1e-6,
        ),
    )

    check(
        "tcp_position_equals_fingertip_center",
        np.allclose(
            result.tcp_position_base,
            selected_tip_base,
            atol=1e-6,
        ),
        result.tcp_position_base,
        selected_tip_base.tolist(),
    )

    check(
        "current_to_target_distance_sane",
        (
            np.isfinite(
                current_to_target_distance
            )
            and current_to_target_distance
            < 0.50
        ),
        current_to_target_distance,
        "< 0.50 m",
    )

    # -----------------------------------------------------------------
    # Artifacts
    # -----------------------------------------------------------------

    draw_pipeline_overlay(
        image_rgb=image_rgb,
        mask=target_mask,
        semantic_point=semantic_point,
        selected_grasp_camera=(
            selected_grasp_camera
        ),
        selected_index=(
            selected_index
        ),
        selected_confidence=(
            selected_confidence
        ),
        selected_clearance=(
            selected_clearance
        ),
        selected_target_distance=(
            selected_target_distance
        ),
        output_path=(
            output_dir
            / "selected_grasp_overlay.png"
        ),
        K=DEFAULT_K,
    )

    semantic_overlay = image_rgb.copy()
    mask_color = np.zeros_like(
        semantic_overlay
    )
    mask_color[..., 1] = 255

    semantic_overlay = np.where(
        target_mask[..., None],
        (
            0.70
            * semantic_overlay
            + 0.30
            * mask_color
        ).astype(
            np.uint8
        ),
        semantic_overlay,
    )

    cv2.circle(
        semantic_overlay,
        (
            int(
                round(
                    float(
                        semantic_point[
                            0
                        ]
                    )
                )
            ),
            int(
                round(
                    float(
                        semantic_point[
                            1
                        ]
                    )
                )
            ),
        ),
        8,
        (
            255,
            0,
            255,
        ),
        -1,
    )

    Image.fromarray(
        semantic_overlay
    ).save(
        output_dir
        / "semantic_sam_overlay.png"
    )

    np.savez_compressed(
        output_dir
        / "graspgenx_full_result.npz",
        semantic_point_px=(
            semantic_point.astype(
                np.float32
            )
        ),
        sam_scores=(
            sam_scores.astype(
                np.float32
            )
        ),
        sam_masks=(
            captured[
                "sam_masks"
            ].astype(
                np.uint8
            )
        ),
        sam_logits=(
            captured[
                "sam_logits"
            ].astype(
                np.float32
            )
        ),
        sam_best_index=np.asarray(
            best_mask_index,
            dtype=np.int32,
        ),
        target_mask=(
            target_mask.astype(
                np.uint8
            )
        ),
        target_point_cloud_camera=(
            target_pc_camera.astype(
                np.float32
            )
        ),
        target_point_cloud_base=(
            target_pc_base.astype(
                np.float32
            )
        ),
        target_centroid_camera=(
            target_centroid_camera
        ),
        target_centroid_base=(
            target_centroid_base
        ),
        T_base_tcp_current=(
            T_base_tcp_current
        ),
        T_tcp_camera=(
            T_TCP_CAMERA
        ),
        T_base_camera=(
            T_base_camera
        ),
        T_base_table=(
            T_base_table
        ),
        table_normal_base=(
            table_normal
        ),
        table_d=np.asarray(
            table_d,
            dtype=np.float64,
        ),
        grasps_base=(
            grasps_base.astype(
                np.float32
            )
        ),
        grasps_camera=(
            grasps_camera.astype(
                np.float32
            )
        ),
        confidences=(
            confidences.astype(
                np.float32
            )
        ),
        branch_tags=(
            branch_tags.astype(
                "<U8"
            )
        ),
        minimum_clearances_m=(
            candidate_diag[
                "minimum_clearances"
            ]
        ),
        fingertip_clearances_m=(
            candidate_diag[
                "fingertip_clearances"
            ]
        ),
        target_distances_m=(
            candidate_diag[
                "target_distances"
            ]
        ),
        approach_tilts_deg=(
            candidate_diag[
                "approach_tilts_deg"
            ]
        ),
        safe_mask=(
            candidate_diag[
                "safe_mask"
            ].astype(
                np.uint8
            )
        ),
        selected_index=np.asarray(
            selected_index,
            dtype=np.int32,
        ),
        selected_grasp_base=(
            selected_grasp_base
        ),
        selected_grasp_camera=(
            selected_grasp_camera
        ),
        selected_tcp_pose_base=(
            T_base_tcp_result
        ),
        selected_fingertip_base=(
            selected_tip_base
        ),
    )

    write_candidates_csv(
        output_dir
        / "candidates.csv",
        candidate_diag,
    )

    result_payload = {
        "pkl": str(
            args.pkl
        ),
        "step": int(
            args.step
        ),
        "instruction": (
            args.instruction
        ),
        "rgb_key": (
            rgb_key
        ),
        "depth_key": (
            depth_key
        ),
        "rgb_shape": list(
            image_rgb.shape
        ),
        "depth_shape": list(
            depth.shape
        ),
        "current_tcp_position_base": (
            current_tcp_position.tolist()
        ),
        "current_tcp_orientation_xyzw": (
            current_tcp_orientation.tolist()
        ),
        "T_base_camera": (
            T_base_camera.tolist()
        ),
        "table": {
            "source": table_source,
            "T_base_table": (
                T_base_table.tolist()
            ),
            "normal_base": (
                table_normal.tolist()
            ),
            "d": float(
                table_d
            ),
            "origin_base": (
                table_origin.tolist()
            ),
            "inlier_count": int(
                table_inlier_mask.sum()
            ),
            "scene_point_count": int(
                len(
                    scene_pc_base
                )
            ),
            "inlier_ratio": (
                table_inlier_ratio
            ),
            "inlier_z_min": float(
                table_inlier_points[
                    :,
                    2,
                ].min()
            ),
            "inlier_z_max": float(
                table_inlier_points[
                    :,
                    2,
                ].max()
            ),
        },
        "semantic_point_px": (
            semantic_point.tolist()
        ),
        "sam_scores": (
            sam_scores.tolist()
        ),
        "sam_best_index": (
            best_mask_index
        ),
        "sam_mask_pixels": int(
            target_mask.sum()
        ),
        "target_point_cloud_points": int(
            len(
                target_pc_camera
            )
        ),
        "target_centroid_camera": (
            target_centroid_camera.tolist()
        ),
        "target_centroid_base": (
            target_centroid_base.tolist()
        ),
        "target_centroid_table_clearance_m": (
            target_centroid_table_clearance
        ),
        "candidate_count": int(
            len(
                grasps_base
            )
        ),
        "diff_count": int(
            np.sum(
                branch_tags
                == "diff"
            )
        ),
        "obb_count": int(
            np.sum(
                branch_tags
                == "obb"
            )
        ),
        "safe_count": (
            safe_count
        ),
        "safe_obb_count": (
            safe_obb_count
        ),
        "selected": {
            "index": (
                selected_index
            ),
            "branch": (
                selected_branch
            ),
            "confidence": (
                selected_confidence
            ),
            "grasp_pose_base": (
                selected_grasp_base.tolist()
            ),
            "tcp_pose_base": (
                T_base_tcp_result.tolist()
            ),
            "tcp_position_base": (
                result.tcp_position_base.tolist()
            ),
            "tcp_orientation_xyzw": (
                result.tcp_orientation_base.tolist()
            ),
            "approach_direction_base": (
                result.approach_direction_base.tolist()
            ),
            "closing_direction_base": (
                selected_closing_base.tolist()
            ),
            "minimum_table_clearance_m": (
                selected_clearance
            ),
            "fingertip_table_clearance_m": (
                selected_tip_clearance
            ),
            "target_distance_m": (
                selected_target_distance
            ),
            "approach_tilt_deg": (
                selected_approach_tilt
            ),
            "current_to_target_distance_m": (
                current_to_target_distance
            ),
        },
        "rotation_diagnostics": {
            "grasp_max_orthonormality_error": (
                grasps_orth_error
            ),
            "grasp_max_determinant_error": (
                grasps_det_error
            ),
        },
        "checks": checks,
    }

    with (
        output_dir
        / "result.json"
    ).open(
        "w",
        encoding="utf-8",
    ) as stream:
        json.dump(
            result_payload,
            stream,
            indent=2,
        )

    failures = [
        name
        for name, payload
        in checks.items()
        if not payload[
            "passed"
        ]
    ]

    # -----------------------------------------------------------------
    # Summary
    # -----------------------------------------------------------------

    print()
    print("=" * 78)
    print("FULL PLAN RESULT")
    print("=" * 78)
    print(
        "Semantic point:",
        semantic_point,
    )
    print(
        "SAM scores:",
        sam_scores,
    )
    print(
        "SAM mask pixels:",
        int(
            target_mask.sum()
        ),
    )
    print(
        "Target centroid BASE:",
        target_centroid_base,
    )
    print(
        "Candidates:",
        len(
            grasps_base
        ),
        "| diff:",
        int(
            np.sum(
                branch_tags
                == "diff"
            )
        ),
        "| obb:",
        int(
            np.sum(
                branch_tags
                == "obb"
            )
        ),
    )
    print(
        "Safe:",
        safe_count,
        "| safe OBB:",
        safe_obb_count,
    )
    print(
        "Selected:",
        selected_index,
        selected_branch,
        "confidence=",
        f"{selected_confidence:.4f}",
    )
    print(
        "Minimum table clearance:",
        f"{selected_clearance * 1000.0:.2f} mm",
    )
    print(
        "Fingertip table clearance:",
        f"{selected_tip_clearance * 1000.0:.2f} mm",
    )
    print(
        "Target distance:",
        f"{selected_target_distance * 1000.0:.2f} mm",
    )
    print(
        "Approach tilt:",
        f"{selected_approach_tilt:.6f} deg",
    )
    print(
        "TCP position BASE:",
        result.tcp_position_base,
    )
    print(
        "TCP quaternion xyzw:",
        result.tcp_orientation_base,
    )
    print(
        "Current -> target:",
        f"{current_to_target_distance * 1000.0:.2f} mm",
    )
    print(
        "Checks:",
        len(
            checks
        )
        - len(
            failures
        ),
        "/",
        len(
            checks
        ),
        "passed",
    )
    print(
        "Artifacts:",
        output_dir,
    )
    print("=" * 78)

    if failures:
        raise AssertionError(
            "Offline GraspGen-X plan validation failed: "
            + ", ".join(
                failures
            )
        )

    print(
        "[PASS] Full offline GraspGen-X plan validated."
    )


if __name__ == "__main__":
    main()
