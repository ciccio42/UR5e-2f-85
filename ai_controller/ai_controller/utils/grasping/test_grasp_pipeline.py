#!/usr/bin/env python3
from __future__ import annotations
import argparse
import json
import shutil
import sys
import time
from pathlib import Path
import cv2
import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy.spatial import cKDTree
# =====================================================================
# Paths
# =====================================================================
REPO_ROOT = Path(
    "/home/ros2_ws/src/UR5e-2f-85"
)
SEEDO_DIR = (
    REPO_ROOT
    / "ai_controller"
    / "ai_controller"
    / "models"
    / "seedo_controller"
)
if str(SEEDO_DIR) not in sys.path:
    sys.path.insert(
        0,
        str(SEEDO_DIR),
    )
from inspect_pkl import (
    load_pickle,
    get_trajectory_step,
)
M2T2_CONFIG = (
    REPO_ROOT
    / "ai_controller"
    / "ai_controller"
    / "utils"
    / "grasping"
    / "m2t2"
    / "config"
    / "m2t2_config.yaml"
)
GRASPMOLMO_CONFIG = (
    REPO_ROOT
    / "ai_controller"
    / "ai_controller"
    / "utils"
    / "grasping"
    / "graspmolmo"
    / "config"
    / "graspmolmo_config.yaml"
)
OUTPUT_DIR = (
    REPO_ROOT
    / ".runtime"
    / "grasp_pipeline_test"
)
# =====================================================================
# Eye-in-hand intrinsics
# =====================================================================
DEFAULT_K = np.array(
    [
        [
            363.8071594238281,
            0.0,
            335.86553955078125,
        ],
        [
            0.0,
            363.8071594238281,
            183.81210327148438,
        ],
        [
            0.0,
            0.0,
            1.0,
        ],
    ],
    dtype=np.float64,
)
# =====================================================================
# TCP -> camera optical
# =====================================================================
T_TCP_CAMERA = np.array(
    [
        [
            0.999262529,
            -0.0383979045,
            0.0,
            -0.0209004103,
        ],
        [
            0.0383979045,
            0.999262529,
            0.0,
            -0.0681027558,
        ],
        [
            0.0,
            0.0,
            1.0,
            -0.1455,
        ],
        [
            0.0,
            0.0,
            0.0,
            1.0,
        ],
    ],
    dtype=np.float64,
)
# =====================================================================
# GraspMolmo geometry
# =====================================================================
GRASP_VOLUME_SIZE = np.array(
    [
        0.082,
        0.01,
        0.112 - 0.066,
    ],
    dtype=np.float32,
)
GRASP_VOLUME_CENTER = np.array(
    [
        0.0,
        0.0,
        (0.066 + 0.112) / 2.0,
    ],
    dtype=np.float32,
)
DRAW_POINTS = np.array(
    [
        [0.041, 0.0, 0.112],
        [0.041, 0.0, 0.066],
        [-0.041, 0.0, 0.066],
        [-0.041, 0.0, 0.112],
    ],
    dtype=np.float32,
)
# =====================================================================
# Rollout RGB-D
# =====================================================================
def get_gripper_rgbd(
    obs: dict,
) -> tuple[np.ndarray, np.ndarray, str, str]:
    """
    Support both rollout formats.
    SeeDo rollout:
        eye_in_hand_image
        eye_in_hand_depth
    Colleague / OSVI-AWDA rollout:
        camera_gripper_image
        camera_gripper_depth
    """
    candidates = [
        (
            "eye_in_hand_image",
            "eye_in_hand_depth",
        ),
        (
            "camera_gripper_image",
            "camera_gripper_depth",
        ),
    ]
    for image_key, depth_key in candidates:
        if (
            image_key in obs
            and depth_key in obs
        ):
            image = np.asarray(
                obs[image_key]
            )
            depth = np.asarray(
                obs[depth_key],
                dtype=np.float32,
            )
            print(
                "[TEST] Gripper RGB key:",
                image_key,
            )
            print(
                "[TEST] Gripper depth key:",
                depth_key,
            )
            return (
                image,
                depth,
                image_key,
                depth_key,
            )
    raise RuntimeError(
        "No supported gripper RGB-D pair found. "
        "Expected either "
        "'eye_in_hand_image' + 'eye_in_hand_depth' "
        "or "
        "'camera_gripper_image' + 'camera_gripper_depth'."
    )
# =====================================================================
# Transform utilities
# =====================================================================
def quaternion_to_rotation(
    quaternion: np.ndarray,
) -> np.ndarray:
    quaternion = np.asarray(
        quaternion,
        dtype=np.float64,
    )
    x, y, z, w = quaternion
    norm = np.linalg.norm(
        quaternion
    )
    if norm <= 1e-12:
        raise ValueError(
            "Invalid zero quaternion."
        )
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return np.array(
        [
            [
                1 - 2 * (y * y + z * z),
                2 * (x * y - z * w),
                2 * (x * z + y * w),
            ],
            [
                2 * (x * y + z * w),
                1 - 2 * (x * x + z * z),
                2 * (y * z - x * w),
            ],
            [
                2 * (x * z - y * w),
                2 * (y * z + x * w),
                1 - 2 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )
def pose_to_transform(
    position: np.ndarray,
    quaternion: np.ndarray,
) -> np.ndarray:
    transform = np.eye(
        4,
        dtype=np.float64,
    )
    transform[
        :3,
        :3,
    ] = quaternion_to_rotation(
        quaternion
    )
    transform[
        :3,
        3,
    ] = np.asarray(
        position,
        dtype=np.float64,
    )
    return transform
def transform_points(
    points: np.ndarray,
    transform: np.ndarray,
) -> np.ndarray:
    points = np.asarray(
        points,
        dtype=np.float64,
    )
    return (
        points
        @ transform[
            :3,
            :3,
        ].T
        + transform[
            :3,
            3,
        ]
    )
def transform_poses(
    poses: np.ndarray,
    transform: np.ndarray,
) -> np.ndarray:
    if len(poses) == 0:
        return np.empty(
            (
                0,
                4,
                4,
            ),
            dtype=np.float32,
        )
    return (
        transform[
            None,
            ...
        ]
        @ poses
    ).astype(
        np.float32
    )
# =====================================================================
# Calibration
# =====================================================================
def load_camera_matrix(
    camera_info_path: Path | None,
) -> np.ndarray:
    if camera_info_path is None:
        print(
            "[TEST] Using built-in gripper camera intrinsics."
        )
        return DEFAULT_K.copy()
    with camera_info_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        data = yaml.safe_load(
            stream
        )
    return np.asarray(
        data["k"],
        dtype=np.float64,
    ).reshape(
        3,
        3,
    )
def load_base_to_table(
    path: Path,
) -> np.ndarray:
    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        data = yaml.safe_load(
            stream
        )
    transform = np.eye(
        4,
        dtype=np.float64,
    )
    transform[
        :3,
        :3,
    ] = np.asarray(
        data["rotation"],
        dtype=np.float64,
    )
    transform[
        :3,
        3,
    ] = np.asarray(
        data["translation"],
        dtype=np.float64,
    )
    return transform
# =====================================================================
# Depth preview
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
    if not np.any(valid):
        Image.fromarray(
            preview
        ).save(
            path
        )
        return
    values = depth[
        valid
    ]
    low = float(
        np.percentile(
            values,
            1,
        )
    )
    high = float(
        np.percentile(
            values,
            99,
        )
    )
    print(
        "[TEST] Depth preview range:",
        f"{low:.6f} -> {high:.6f} m",
    )
    if high > low:
        normalized = (
            depth
            - low
        ) / (
            high
            - low
        )
        normalized = np.clip(
            normalized,
            0.0,
            1.0,
        )
        preview[
            valid
        ] = (
            normalized[
                valid
            ]
            * 255.0
        ).astype(
            np.uint8
        )
    Image.fromarray(
        preview
    ).save(
        path
    )
# =====================================================================
# Point cloud
# =====================================================================
def depth_to_point_cloud(
    depth: np.ndarray,
    K: np.ndarray,
    min_depth: float,
    max_depth: float,
    bottom_ignore_px: int,
) -> tuple[np.ndarray, np.ndarray]:
    depth = np.asarray(
        depth,
        dtype=np.float64,
    )
    height, width = (
        depth.shape
    )
    fx = K[
        0,
        0,
    ]
    fy = K[
        1,
        1,
    ]
    cx = K[
        0,
        2,
    ]
    cy = K[
        1,
        2,
    ]
    u, v = np.meshgrid(
        np.arange(
            width,
            dtype=np.float64,
        ),
        np.arange(
            height,
            dtype=np.float64,
        ),
        indexing="xy",
    )
    valid = (
        np.isfinite(
            depth
        )
        & (
            depth
            >= min_depth
        )
        & (
            depth
            <= max_depth
        )
    )
    if bottom_ignore_px > 0:
        valid[
            max(
                0,
                height - bottom_ignore_px,
            ):
            ,
        ] = False
    z = depth[
        valid
    ]
    x = (
        u[
            valid
        ]
        - cx
    ) * z / fx
    y = (
        v[
            valid
        ]
        - cy
    ) * z / fy
    point_cloud = np.stack(
        [
            x,
            y,
            z,
        ],
        axis=1,
    )
    return (
        point_cloud.astype(
            np.float32
        ),
        valid,
    )
# =====================================================================
# Automatic table Z correction
# =====================================================================
def estimate_table_z_offset(
    point_cloud_table: np.ndarray,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
) -> float:
    points = np.asarray(
        point_cloud_table,
        dtype=np.float64,
    )
    mask = (
        (
            points[
                :,
                0
            ]
            >= x_min
        )
        & (
            points[
                :,
                0
            ]
            <= x_max
        )
        & (
            points[
                :,
                1
            ]
            >= y_min
        )
        & (
            points[
                :,
                1
            ]
            <= y_max
        )
        & (
            points[
                :,
                2
            ]
            >= -0.10
        )
        & (
            points[
                :,
                2
            ]
            <= 0.20
        )
    )
    z_values = points[
        mask,
        2,
    ]
    if z_values.size < 1000:
        raise RuntimeError(
            "Not enough points to estimate table Z."
        )
    bin_width = 0.002
    z_min = float(
        z_values.min()
    )
    z_max = float(
        z_values.max()
    )
    bins = np.arange(
        z_min,
        z_max
        + bin_width,
        bin_width,
    )
    histogram, edges = np.histogram(
        z_values,
        bins=bins,
    )
    peak_index = int(
        np.argmax(
            histogram
        )
    )
    peak_center = (
        edges[
            peak_index
        ]
        + edges[
            peak_index + 1
        ]
    ) / 2.0
    refinement_mask = (
        np.abs(
            z_values
            - peak_center
        )
        <= 0.006
    )
    refined = z_values[
        refinement_mask
    ]
    if refined.size == 0:
        return float(
            peak_center
        )
    return float(
        np.median(
            refined
        )
    )
# =====================================================================
# Grasp orientation metrics
# =====================================================================
def wrist_rotation_deg(
    grasp_aligned: np.ndarray,
    current_tcp_aligned: np.ndarray,
) -> float:
    """
    Minimum planar wrist rotation between the current TCP orientation
    and a candidate M2T2 grasp orientation in the aligned table frame.
    The first grasp rotation column is M2T2's contact direction.
    Opposite planar directions are treated as equivalent because a
    parallel gripper is symmetric under a 180 degree rotation around
    its approach axis.
    Returns an angle in [0, 90] degrees.
    """
    candidate_axis_xy = np.asarray(
        grasp_aligned[:2, 0],
        dtype=np.float64,
    )
    current_axis_xy = np.asarray(
        current_tcp_aligned[:2, 0],
        dtype=np.float64,
    )
    candidate_norm = np.linalg.norm(
        candidate_axis_xy
    )
    current_norm = np.linalg.norm(
        current_axis_xy
    )
    if (
        candidate_norm < 1e-8
        or current_norm < 1e-8
    ):
        return float("inf")
    candidate_axis_xy /= candidate_norm
    current_axis_xy /= current_norm
    cosine = np.clip(
        abs(
            np.dot(
                candidate_axis_xy,
                current_axis_xy,
            )
        ),
        0.0,
        1.0,
    )
    return float(
        np.degrees(
            np.arccos(
                cosine
            )
        )
    )
def approach_tilt_deg(
    grasp_table: np.ndarray,
) -> float:
    """
    Angle between grasp approach axis and vertical-down
    in the aligned table frame.
    0 deg  -> perfectly top-down
    90 deg -> horizontal approach
    """
    approach = np.asarray(
        grasp_table[:3, 2],
        dtype=np.float64,
    )
    norm = np.linalg.norm(
        approach
    )
    if norm < 1e-8:
        return float("inf")
    approach /= norm
    vertical_down = np.array(
        [0.0, 0.0, -1.0],
        dtype=np.float64,
    )
    cosine = np.clip(
        np.dot(
            approach,
            vertical_down,
        ),
        -1.0,
        1.0,
    )
    return float(
        np.degrees(
            np.arccos(
                cosine
            )
        )
    )
# =====================================================================
# Projection
# =====================================================================
def project_points(
    points_camera: np.ndarray,
    K: np.ndarray,
) -> np.ndarray:
    points_camera = np.asarray(
        points_camera,
        dtype=np.float64,
    )
    pixels = np.full(
        (
            points_camera.shape[0],
            2,
        ),
        np.nan,
        dtype=np.float64,
    )
    valid = (
        np.isfinite(
            points_camera
        ).all(
            axis=1
        )
        & (
            points_camera[
                :,
                2
            ]
            > 0.0
        )
    )
    if not np.any(
        valid
    ):
        return pixels
    projected = (
        points_camera[
            valid
        ]
        @ K.T
    )
    projected = (
        projected[
            :,
            :2
        ]
        / projected[
            :,
            2:3
        ]
    )
    pixels[
        valid
    ] = projected
    return pixels
# =====================================================================
# Workspace visualization
# =====================================================================
def draw_workspace(
    image: Image.Image,
    points_camera: np.ndarray,
    K: np.ndarray,
) -> Image.Image:
    visualization = (
        image.copy()
    )
    draw = ImageDraw.Draw(
        visualization
    )
    pixels = project_points(
        points_camera,
        K,
    )
    if len(
        pixels
    ) > 12000:
        indexes = np.linspace(
            0,
            len(
                pixels
            )
            - 1,
            12000,
        ).astype(
            np.int64
        )
        pixels = pixels[
            indexes
        ]
    width, height = (
        image.size
    )
    for x, y in pixels:
        if not (
            np.isfinite(
                x
            )
            and np.isfinite(
                y
            )
        ):
            continue
        if (
            0 <= x < width
            and 0 <= y < height
        ):
            draw.ellipse(
                (
                    x - 1,
                    y - 1,
                    x + 1,
                    y + 1,
                ),
                fill="lime",
            )
    return visualization
# =====================================================================
# M2T2 visualization
# =====================================================================
def draw_m2t2_candidates(
    image: Image.Image,
    contacts_camera: np.ndarray,
    confidence: np.ndarray,
    K: np.ndarray,
) -> Image.Image:
    visualization = (
        image.copy()
    )
    draw = ImageDraw.Draw(
        visualization
    )
    pixels = project_points(
        contacts_camera,
        K,
    )
    width, height = (
        image.size
    )
    max_points = 2000
    if len(
        confidence
    ) > max_points:
        indexes = np.argsort(
            confidence
        )[
            -max_points:
        ]
    else:
        indexes = np.arange(
            len(
                confidence
            )
        )
    for index in indexes:
        x, y = pixels[
            index
        ]
        if not (
            np.isfinite(
                x
            )
            and np.isfinite(
                y
            )
        ):
            continue
        if not (
            0 <= x < width
            and 0 <= y < height
        ):
            continue
        draw.ellipse(
            (
                x - 2,
                y - 2,
                x + 2,
                y + 2,
            ),
            fill="red",
            outline="white",
            width=1,
        )
    if len(
        confidence
    ) > 0:
        best_index = int(
            np.argmax(
                confidence
            )
        )
        x, y = pixels[
            best_index
        ]
        if (
            np.isfinite(
                x
            )
            and np.isfinite(
                y
            )
            and 0 <= x < width
            and 0 <= y < height
        ):
            radius = 12
            draw.line(
                (
                    x - radius,
                    y,
                    x + radius,
                    y,
                ),
                fill="blue",
                width=3,
            )
            draw.line(
                (
                    x,
                    y - radius,
                    x,
                    y + radius,
                ),
                fill="blue",
                width=3,
            )
    return visualization
# =====================================================================
# GraspMolmo representative points
# =====================================================================
def get_grasp_points_numpy(
    point_cloud: np.ndarray,
    grasps: np.ndarray,
) -> np.ndarray:
    point_cloud = np.asarray(
        point_cloud,
        dtype=np.float32,
    )
    grasps = np.asarray(
        grasps,
        dtype=np.float32,
    )

    min_position = (
        GRASP_VOLUME_CENTER
        - GRASP_VOLUME_SIZE / 2.0
    )
    max_position = (
        GRASP_VOLUME_CENTER
        + GRASP_VOLUME_SIZE / 2.0
    )

    # Circumscribed sphere of the grasp volume. Every point that can
    # satisfy the exact oriented-box test must lie inside this sphere.
    query_radius = float(
        np.linalg.norm(
            GRASP_VOLUME_SIZE / 2.0
        )
    ) + 1e-6

    # Build the spatial index once instead of scanning the whole cloud
    # independently for every grasp.
    tree = cKDTree(
        point_cloud
    )

    grasp_points = np.empty(
        (
            len(grasps),
            3,
        ),
        dtype=np.float32,
    )

    for i, grasp in enumerate(
        grasps
    ):
        rotation = grasp[
            :3,
            :3,
        ]
        translation = grasp[
            :3,
            3,
        ]

        # Center of the grasp volume expressed in camera coordinates.
        volume_center = (
            translation
            + rotation
            @ GRASP_VOLUME_CENTER
        )

        nearby_indices = tree.query_ball_point(
            volume_center,
            query_radius,
        )

        reference = (
            translation
            + rotation[
                :,
                2
            ]
            * 0.066
        )

        selected_point = None

        if nearby_indices:
            nearby_indices = np.asarray(
                nearby_indices,
                dtype=np.int64,
            )
            nearby_points = point_cloud[
                nearby_indices
            ]

            # Preserve the exact original oriented-box test, but apply
            # it only to spatially nearby points.
            local_points = (
                nearby_points
                - translation
            ) @ rotation

            inside = np.all(
                (
                    local_points
                    >= min_position
                )
                & (
                    local_points
                    <= max_position
                ),
                axis=1,
            )

            if np.any(
                inside
            ):
                candidate_points = nearby_points[
                    inside
                ]
                squared_distance = np.sum(
                    (
                        candidate_points
                        - reference
                    )
                    ** 2,
                    axis=1,
                )
                selected_point = candidate_points[
                    int(
                        np.argmin(
                            squared_distance
                        )
                    )
                ]

        # Preserve the original fallback exactly in meaning:
        # if the grasp volume contains no scene point, select the
        # globally nearest point to the reference. KD-tree makes this
        # O(log N) instead of scanning the whole cloud.
        if selected_point is None:
            _, nearest_index = tree.query(
                reference,
                k=1,
            )
            selected_point = point_cloud[
                int(
                    nearest_index
                )
            ]

        grasp_points[
            i
        ] = selected_point

        if (
            (i + 1) % 250 == 0
            or i + 1 == len(
                grasps
            )
        ):
            print(
                "[MATCH] Representative points:",
                f"{i + 1}/{len(grasps)}",
            )

    return grasp_points

# =====================================================================
# Combined visualization
# =====================================================================
def draw_grasp(
    draw: ImageDraw.ImageDraw,
    grasp_camera: np.ndarray,
    K: np.ndarray,
) -> None:
    points_3d = (
        DRAW_POINTS
        @ grasp_camera[
            :3,
            :3,
        ].T
        + grasp_camera[
            :3,
            3,
        ]
    )
    points_2d = project_points(
        points_3d,
        K,
    )
    if not np.isfinite(
        points_2d
    ).all():
        return
    points_2d = (
        points_2d
        .round()
        .astype(
            int
        )
    )
    for index in range(
        len(
            points_2d
        )
        - 1
    ):
        draw.line(
            [
                tuple(
                    points_2d[
                        index
                    ]
                ),
                tuple(
                    points_2d[
                        index + 1
                    ]
                ),
            ],
            fill="lime",
            width=4,
        )
def draw_combined_result(
    image: Image.Image,
    semantic_point: np.ndarray,
    grasp_pixels: np.ndarray,
    selected_index: int,
    selected_grasp_camera: np.ndarray,
    K: np.ndarray,
    semantic_radius: float,
) -> Image.Image:
    visualization = (
        image.copy()
    )
    draw = ImageDraw.Draw(
        visualization
    )
    width, height = (
        image.size
    )
    for x, y in grasp_pixels:
        if not (
            np.isfinite(
                x
            )
            and np.isfinite(
                y
            )
        ):
            continue
        if (
            0 <= x < width
            and 0 <= y < height
        ):
            draw.ellipse(
                (
                    x - 2,
                    y - 2,
                    x + 2,
                    y + 2,
                ),
                fill="red",
            )
    sx = float(
        semantic_point[
            0
        ]
    )
    sy = float(
        semantic_point[
            1
        ]
    )
    draw.ellipse(
        (
            sx - semantic_radius,
            sy - semantic_radius,
            sx + semantic_radius,
            sy + semantic_radius,
        ),
        outline="blue",
        width=2,
    )
    draw.ellipse(
        (
            sx - 7,
            sy - 7,
            sx + 7,
            sy + 7,
        ),
        outline="blue",
        width=4,
    )
    gx = float(
        grasp_pixels[
            selected_index,
            0,
        ]
    )
    gy = float(
        grasp_pixels[
            selected_index,
            1,
        ]
    )
    draw.line(
        (
            sx,
            sy,
            gx,
            gy,
        ),
        fill="cyan",
        width=2,
    )
    draw.ellipse(
        (
            gx - 7,
            gy - 7,
            gx + 7,
            gy + 7,
        ),
        fill="lime",
        outline="black",
        width=2,
    )
    draw_grasp(
        draw,
        selected_grasp_camera,
        K,
    )
    return visualization
# =====================================================================
# Main
# =====================================================================
def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--pkl",
        type=Path,
        default=Path(
            "/scene_capture/traj_000.pkl"
        ),
    )
    parser.add_argument(
        "--step",
        type=int,
        default=19,
    )
    parser.add_argument(
        "--extract-only",
        action="store_true",
        help=(
            "Extract and save gripper RGB/depth "
            "and stop before geometry/M2T2."
        ),
    )
    parser.add_argument(
        "--base-to-table",
        type=Path,
        default=Path(
            "/scene_capture/nut/scena_1/"
            "base_to_table_transform.yaml"
        ),
    )
    parser.add_argument(
        "--camera-info",
        type=Path,
        default=None,
    )
    parser.add_argument(
        "--num-runs",
        type=int,
        default=5,
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed used by M2T2 and supported local semantic inference.",
    )
    parser.add_argument(
        "--confidence-threshold",
        type=float,
        default=0.50,
    )
    parser.add_argument(
        "--semantic-radius-px",
        type=float,
        default=20.0,
    )
    parser.add_argument(
        "--max-approach-tilt-deg",
        type=float,
        default=20.0,
    )
    parser.add_argument(
        "--max-wrist-rotation-deg",
        type=float,
        default=20.0,
    )
    parser.add_argument(
        "--task",
        type=str,
        default=None,
    )
    parser.add_argument(
        "--depth-min",
        type=float,
        default=0.15,
    )
    parser.add_argument(
        "--depth-max",
        type=float,
        default=0.60,
    )
    parser.add_argument(
        "--bottom-ignore-px",
        type=int,
        default=30,
    )
    parser.add_argument(
        "--table-z-offset",
        type=float,
        default=None,
    )
    parser.add_argument(
        "--surface-snap",
        type=float,
        default=0.0,
    )
    args = parser.parse_args()
    # =============================================================
    # Clean output directory
    # =============================================================
    if OUTPUT_DIR.exists():
        shutil.rmtree(
            OUTPUT_DIR
        )
    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )
    print(
        "=" * 78
    )
    print(
        "GENERAL EYE-IN-HAND GRASP TEST"
    )
    print(
        "=" * 78
    )
    print(
        "[TEST] PKL:",
        args.pkl,
    )
    print(
        "[TEST] Step:",
        args.step,
    )
    # =============================================================
    # Load rollout
    # =============================================================
    data = load_pickle(
        args.pkl
    )
    trajectory = data[
        "traj"
    ]
    print(
        "[TEST] Trajectory length:",
        len(
            trajectory
        ),
    )
    step = get_trajectory_step(
        trajectory,
        args.step,
    )
    obs = step[
        "obs"
    ]
    # =============================================================
    # Load gripper RGB-D
    # =============================================================
    (
        image_bgr,
        depth,
        image_key,
        depth_key,
    ) = get_gripper_rgbd(
        obs
    )
    if (
        image_bgr.ndim != 3
        or image_bgr.shape[
            2
        ] != 3
    ):
        raise RuntimeError(
            "Invalid gripper RGB shape: "
            f"{image_bgr.shape}"
        )
    if depth.ndim != 2:
        raise RuntimeError(
            "Invalid gripper depth shape: "
            f"{depth.shape}"
        )
    # Dataset camera images are BGR/OpenCV.
    image_rgb = cv2.cvtColor(
        image_bgr,
        cv2.COLOR_BGR2RGB,
    )
    image = Image.fromarray(
        image_rgb
    )
    image.save(
        OUTPUT_DIR
        / "input_rgb.png"
    )
    np.save(
        OUTPUT_DIR
        / "input_depth.npy",
        depth,
    )
    save_depth_preview(
        depth,
        OUTPUT_DIR
        / "input_depth_preview.png",
    )
    # =============================================================
    # Depth statistics
    # =============================================================
    valid_depth = depth[
        np.isfinite(
            depth
        )
        & (
            depth > 0.0
        )
    ]
    print()
    print(
        "[TEST] RGB shape:",
        image_rgb.shape,
    )
    print(
        "[TEST] Depth shape:",
        depth.shape,
    )
    print(
        "[TEST] Valid depth pixels:",
        valid_depth.size,
        "/",
        depth.size,
    )
    if valid_depth.size > 0:
        print(
            "[TEST] Depth min:",
            float(
                valid_depth.min()
            ),
        )
        print(
            "[TEST] Depth max:",
            float(
                valid_depth.max()
            ),
        )
        for percentile in (
            1,
            5,
            25,
            50,
            75,
            95,
            99,
        ):
            print(
                f"[TEST] Depth p{percentile}:",
                float(
                    np.percentile(
                        valid_depth,
                        percentile,
                    )
                ),
            )
    # =============================================================
    # Save basic metadata
    # =============================================================
    basic_metadata = {
        "pkl": str(
            args.pkl
        ),
        "step": int(
            args.step
        ),
        "rgb_key": image_key,
        "depth_key": depth_key,
        "rgb_shape": list(
            image_rgb.shape
        ),
        "depth_shape": list(
            depth.shape
        ),
        "valid_depth_pixels": int(
            valid_depth.size
        ),
    }
    with (
        OUTPUT_DIR
        / "input_metadata.json"
    ).open(
        "w",
        encoding="utf-8",
    ) as stream:
        json.dump(
            basic_metadata,
            stream,
            indent=2,
        )
    # =============================================================
    # Extract-only mode
    # =============================================================
    if args.extract_only:
        print()
        print(
            "=" * 78
        )
        print(
            "[PASS] RGB-D extraction completed."
        )
        print(
            "[TEST] Output directory:",
            OUTPUT_DIR,
        )
        print()
        print(
            "[TEST] Files:"
        )
        print(
            "       input_rgb.png"
        )
        print(
            "       input_depth.npy"
        )
        print(
            "       input_depth_preview.png"
        )
        print(
            "       input_metadata.json"
        )
        print(
            "=" * 78
        )
        return
    # =============================================================
    # Remaining fields required for geometry
    # =============================================================
    required_geometry_fields = (
        "eef_pos",
        "eef_quat",
    )
    missing = [
        key
        for key
        in required_geometry_fields
        if key not in obs
    ]
    if missing:
        raise RuntimeError(
            "Missing rollout geometry fields: "
            + ", ".join(
                missing
            )
        )
    # =============================================================
    # Intrinsics
    # =============================================================
    K = load_camera_matrix(
        args.camera_info
    )
    print()
    print(
        "[TEST] Camera K:"
    )
    print(
        K
    )
    # =============================================================
    # Historical TCP pose
    # =============================================================
    eef_position = np.asarray(
        obs[
            "eef_pos"
        ],
        dtype=np.float64,
    )
    eef_quaternion = np.asarray(
        obs[
            "eef_quat"
        ],
        dtype=np.float64,
    )
    T_base_tcp = pose_to_transform(
        eef_position,
        eef_quaternion,
    )
    T_base_camera = (
        T_base_tcp
        @ T_TCP_CAMERA
    )
    print()
    print(
        "[TEST] EEF position:",
        eef_position,
    )
    print(
        "[TEST] EEF quaternion:",
        eef_quaternion,
    )
    # =============================================================
    # Base -> table
    # =============================================================
    T_base_table = (
        load_base_to_table(
            args.base_to_table
        )
    )
    T_table_base = (
        np.linalg.inv(
            T_base_table
        )
    )
    T_table_camera_raw = (
        T_table_base
        @ T_base_camera
    )
    # =============================================================
    # Point cloud
    # =============================================================
    (
        point_cloud_camera,
        depth_valid_mask,
    ) = depth_to_point_cloud(
        depth=depth,
        K=K,
        min_depth=args.depth_min,
        max_depth=args.depth_max,
        bottom_ignore_px=(
            args.bottom_ignore_px
        ),
    )
    print()
    print(
        "[TEST] Camera point cloud:",
        point_cloud_camera.shape,
    )
    point_cloud_table_raw = (
        transform_points(
            point_cloud_camera,
            T_table_camera_raw,
        )
    )
    print()
    print(
        "[TEST] RAW table Z percentiles:"
    )
    for percentile in (
        1,
        5,
        25,
        50,
        75,
        95,
        99,
    ):
        print(
            f"       p{percentile}:",
            float(
                np.percentile(
                    point_cloud_table_raw[
                        :,
                        2
                    ],
                    percentile,
                )
            ),
        )
    # =============================================================
    # Estimate table offset
    # =============================================================
    workspace_x_min = -0.45
    workspace_x_max = 0.45
    workspace_y_min = -0.35
    workspace_y_max = 0.30
    if args.table_z_offset is None:
        table_z_offset = (
            estimate_table_z_offset(
                point_cloud_table_raw,
                x_min=workspace_x_min,
                x_max=workspace_x_max,
                y_min=workspace_y_min,
                y_max=workspace_y_max,
            )
        )
        table_z_source = (
            "automatic"
        )
    else:
        table_z_offset = float(
            args.table_z_offset
        )
        table_z_source = (
            "manual"
        )
    print()
    print(
        "[TEST] Table Z offset:",
        f"{table_z_offset:.6f} m",
        f"({table_z_source})",
    )
    # =============================================================
    # Build aligned table frame
    # =============================================================
    T_aligned_table = np.eye(
        4,
        dtype=np.float64,
    )
    T_aligned_table[
        2,
        3,
    ] = -table_z_offset
    T_table_aligned = (
        np.linalg.inv(
            T_aligned_table
        )
    )
    T_aligned_camera = (
        T_aligned_table
        @ T_table_camera_raw
    )
    T_camera_aligned = (
        np.linalg.inv(
            T_aligned_camera
        )
    )
    T_base_aligned = (
        T_base_table
        @ T_table_aligned
    )
    T_aligned_base = (
        np.linalg.inv(
            T_base_aligned
        )
    )
    T_aligned_tcp = (
        T_aligned_base
        @ T_base_tcp
    )
    point_cloud_aligned = (
        transform_points(
            point_cloud_camera,
            T_aligned_camera,
        )
        .astype(
            np.float32
        )
    )
    print()
    print(
        "[TEST] ALIGNED Z percentiles:"
    )
    for percentile in (
        1,
        5,
        25,
        50,
        75,
        95,
        99,
    ):
        print(
            f"       p{percentile}:",
            float(
                np.percentile(
                    point_cloud_aligned[
                        :,
                        2
                    ],
                    percentile,
                )
            ),
        )
    # =============================================================
    # Workspace crop
    # =============================================================
    workspace_mask = (
        (
            point_cloud_aligned[
                :,
                0
            ]
            >= workspace_x_min
        )
        & (
            point_cloud_aligned[
                :,
                0
            ]
            <= workspace_x_max
        )
        & (
            point_cloud_aligned[
                :,
                1
            ]
            >= workspace_y_min
        )
        & (
            point_cloud_aligned[
                :,
                1
            ]
            <= workspace_y_max
        )
        & (
            point_cloud_aligned[
                :,
                2
            ]
            >= -0.03
        )
        & (
            point_cloud_aligned[
                :,
                2
            ]
            <= 0.25
        )
    )
    point_cloud_m2t2 = (
        point_cloud_aligned[
            workspace_mask
        ]
        .copy()
    )
    if len(
        point_cloud_m2t2
    ) == 0:
        raise RuntimeError(
            "Workspace crop removed all points."
        )
    # =============================================================
    # Optional surface snapping
    # =============================================================
    snapped_points = 0
    if args.surface_snap > 0.0:
        surface_mask = (
            np.abs(
                point_cloud_m2t2[
                    :,
                    2
                ]
            )
            < args.surface_snap
        )
        snapped_points = int(
            surface_mask.sum()
        )
        point_cloud_m2t2[
            surface_mask,
            2,
        ] = 0.0
    print()
    print(
        "[TEST] M2T2 point cloud:",
        point_cloud_m2t2.shape,
    )
    print(
        "[TEST] Surface snap:",
        args.surface_snap,
    )
    print(
        "[TEST] Snapped points:",
        snapped_points,
    )
    print(
        "[TEST] M2T2 bounds:"
    )
    print(
        "       min:",
        point_cloud_m2t2.min(
            axis=0
        ),
    )
    print(
        "       max:",
        point_cloud_m2t2.max(
            axis=0
        ),
    )
    # =============================================================
    # Workspace visualization
    # =============================================================
    workspace_camera = (
        transform_points(
            point_cloud_m2t2,
            T_camera_aligned,
        )
    )
    workspace_image = (
        draw_workspace(
            image,
            workspace_camera,
            K,
        )
    )
    workspace_image.save(
        OUTPUT_DIR
        / "workspace.png"
    )
    # =============================================================
    # M2T2
    # =============================================================
    from ai_controller.utils.grasping.m2t2.client import (
        M2T2Client,
    )
    print()
    print(
        "[TEST] Connecting to M2T2..."
    )
    m2t2_client = (
        M2T2Client(
            config_path=M2T2_CONFIG
        )
    )
    print(
        "[TEST] Running M2T2..."
    )
    start = (
        time.perf_counter()
    )
    print(
        "[TEST] M2T2 seed:",
        args.seed,
    )
    (
        grasps_aligned,
        contacts_aligned,
        confidence,
    ) = m2t2_client.predict_grasps(
        point_cloud_m2t2,
        num_runs=args.num_runs,
        seed=args.seed,
    )
    elapsed = (
        time.perf_counter()
        - start
    )
    print(
        "[TEST] Grasps:",
        grasps_aligned.shape,
    )
    print(
        "[TEST] Contacts:",
        contacts_aligned.shape,
    )
    print(
        "[TEST] Confidence:",
        confidence.shape,
    )
    print(
        "[TEST] M2T2 time:",
        f"{elapsed:.3f} s",
    )
    if len(
        grasps_aligned
    ) == 0:
        raise RuntimeError(
            "M2T2 returned zero grasps."
        )
    # =============================================================
    # Convert M2T2 outputs
    # =============================================================
    grasps_camera = (
        transform_poses(
            grasps_aligned,
            T_camera_aligned,
        )
    )
    contacts_camera = (
        transform_points(
            contacts_aligned,
            T_camera_aligned,
        )
        .astype(
            np.float32
        )
    )
    grasps_base = (
        transform_poses(
            grasps_aligned,
            T_base_aligned,
        )
    )
    contacts_base = (
        transform_points(
            contacts_aligned,
            T_base_aligned,
        )
        .astype(
            np.float32
        )
    )
    m2t2_image = (
        draw_m2t2_candidates(
            image,
            contacts_camera,
            confidence,
            K,
        )
    )
    m2t2_image.save(
        OUTPUT_DIR
        / "m2t2_candidates.png"
    )
    best_m2t2_index = int(
        np.argmax(
            confidence
        )
    )
    print()
    print(
        "[M2T2] Best candidate:"
    )
    print(
        "       index:",
        best_m2t2_index,
    )
    print(
        "       confidence:",
        float(
            confidence[
                best_m2t2_index
            ]
        ),
    )
    # =============================================================
    # Result storage
    # =============================================================
    result_data = {
        "pkl": str(
            args.pkl
        ),
        "step": int(
            args.step
        ),
        "rgb_key": image_key,
        "depth_key": depth_key,
        "table_z_offset_m": float(
            table_z_offset
        ),
        "num_runs": int(
            args.num_runs
        ),
        "m2t2_candidates": int(
            len(
                confidence
            )
        ),
        "m2t2_best_index": int(
            best_m2t2_index
        ),
        "m2t2_best_confidence": float(
            confidence[
                best_m2t2_index
            ]
        ),
        "seed": int(
            args.seed
        ),
    }
    npz_data = {
        "K": K,
        "T_base_tcp": T_base_tcp,
        "T_tcp_camera": T_TCP_CAMERA,
        "T_base_camera": T_base_camera,
        "T_base_table": T_base_table,
        "T_table_camera_raw": (
            T_table_camera_raw
        ),
        "table_z_offset_m": np.asarray(
            table_z_offset
        ),
        "T_aligned_camera": (
            T_aligned_camera
        ),
        "T_camera_aligned": (
            T_camera_aligned
        ),
        "T_base_aligned": (
            T_base_aligned
        ),
        "T_aligned_tcp": (
            T_aligned_tcp
        ),
        "point_cloud_camera": (
            point_cloud_camera
        ),
        "point_cloud_table_raw": (
            point_cloud_table_raw
        ),
        "point_cloud_aligned": (
            point_cloud_aligned
        ),
        "point_cloud_m2t2": (
            point_cloud_m2t2
        ),
        "grasps_aligned": (
            grasps_aligned
        ),
        "contacts_aligned": (
            contacts_aligned
        ),
        "grasps_camera": (
            grasps_camera
        ),
        "contacts_camera": (
            contacts_camera
        ),
        "grasps_base": (
            grasps_base
        ),
        "contacts_base": (
            contacts_base
        ),
        "confidence": (
            confidence
        ),
        "seed": np.asarray(
            args.seed,
            dtype=np.int64,
        ),
    }
    # =============================================================
    # GraspMolmo + M2T2 matching
    # =============================================================
    if args.task is not None:
        print()
        print("=" * 78)
        print("GRASPMOLMO + M2T2")
        print("=" * 78)
        print("[TEST] Task:", args.task)

        from ai_controller.utils.grasping.graspmolmo.client import (
            GraspMolmoClient,
        )

        semantic_client = GraspMolmoClient(
            config_path=GRASPMOLMO_CONFIG
        )

        print(
            "[TEST] GraspMolmo seed:",
            args.seed,
        )

        start = time.perf_counter()

        semantic_point = semantic_client.predict_point(
            image_rgb,
            args.task,
            verbosity=1,
            timeout=60.0,
            seed=args.seed,
        )

        elapsed = (
            time.perf_counter()
            - start
        )

        if semantic_point is None:
            raise RuntimeError(
                "GraspMolmo returned no point."
            )

        semantic_point = np.asarray(
            semantic_point,
            dtype=np.float32,
        )

        print(
            "[Semantic] Point:",
            semantic_point,
        )
        print(
            "[Semantic] Time:",
            f"{elapsed:.3f} s",
        )

        result_data[
            "semantic_backend"
        ] = "graspmolmo"
        # ---------------------------------------------------------
        # Selection policy:
        #
        # 1. M2T2 confidence threshold
        # 2. GraspMolmo semantic radius
        # 3. Top-down approach constraint
        # 4. Minimum wrist rotation from the current TCP orientation
        # 5. Higher M2T2 confidence, then smaller semantic distance
        # ---------------------------------------------------------
        confidence_mask = (
            confidence
            >= args.confidence_threshold
        )
        candidate_indices = np.flatnonzero(
            confidence_mask
        )
        if len(candidate_indices) == 0:
            raise RuntimeError(
                "No M2T2 candidate survived "
                "the confidence threshold."
            )

        filtered_grasps_camera = grasps_camera[
            candidate_indices
        ]
        filtered_grasps_aligned = grasps_aligned[
            candidate_indices
        ]
        filtered_confidence = confidence[
            candidate_indices
        ]

        print()
        print(
            "[MATCH] Confidence threshold:",
            args.confidence_threshold,
        )
        print(
            "[MATCH] Candidates after confidence filter:",
            len(candidate_indices),
        )

        # ---------------------------------------------------------
        # Cheap geometric pre-filter.
        #
        # These metrics depend only on the 6-DoF grasp pose, so
        # compute them before the expensive representative-point
        # matching. Grasps that cannot satisfy the robot geometry
        # constraints never need a representative point.
        # ---------------------------------------------------------
        geometry_start = time.perf_counter()

        wrist_rotations = np.array(
            [
                wrist_rotation_deg(
                    grasp,
                    T_aligned_tcp,
                )
                for grasp in filtered_grasps_aligned
            ],
            dtype=np.float32,
        )
        approach_tilts = np.array(
            [
                approach_tilt_deg(
                    grasp
                )
                for grasp in filtered_grasps_aligned
            ],
            dtype=np.float32,
        )

        geometric_mask = (
            (
                approach_tilts
                <= args.max_approach_tilt_deg
            )
            & (
                wrist_rotations
                <= args.max_wrist_rotation_deg
            )
        )
        geometric_local_indices = np.flatnonzero(
            geometric_mask
        )

        geometry_elapsed = (
            time.perf_counter()
            - geometry_start
        )

        print()
        print(
            "[MATCH] Max approach tilt:",
            f"{args.max_approach_tilt_deg:.1f} deg",
        )
        print(
            "[MATCH] Max wrist rotation:",
            f"{args.max_wrist_rotation_deg:.1f} deg",
        )
        print(
            "[MATCH] Candidates after geometric filter:",
            len(geometric_local_indices),
        )
        print(
            "[MATCH] Geometric filter time:",
            f"{geometry_elapsed:.3f} s",
        )

        if len(geometric_local_indices) == 0:
            raise RuntimeError(
                "No confidence-compatible grasp with "
                f"tilt <= {args.max_approach_tilt_deg:.1f} deg "
                f"and wrist rotation <= "
                f"{args.max_wrist_rotation_deg:.1f} deg."
            )

        # ---------------------------------------------------------
        # Expensive semantic matching.
        #
        # Keep arrays aligned with candidate_indices for backward
        # compatibility, but compute representative points only for
        # candidates that already passed the geometric constraints.
        # ---------------------------------------------------------
        grasp_points_camera = np.full(
            (
                len(candidate_indices),
                3,
            ),
            np.nan,
            dtype=np.float32,
        )
        grasp_pixels = np.full(
            (
                len(candidate_indices),
                2,
            ),
            np.nan,
            dtype=np.float64,
        )
        distances = np.full(
            len(candidate_indices),
            np.inf,
            dtype=np.float64,
        )

        print(
            "[MATCH] Computing representative points "
            f"for {len(geometric_local_indices)} geometric candidates..."
        )

        representative_start = time.perf_counter()

        geometric_grasp_points_camera = (
            get_grasp_points_numpy(
                point_cloud_camera,
                filtered_grasps_camera[
                    geometric_local_indices
                ],
            )
        )
        geometric_grasp_pixels = project_points(
            geometric_grasp_points_camera,
            K,
        )
        geometric_distances = np.linalg.norm(
            geometric_grasp_pixels
            - semantic_point[
                None,
                :
            ],
            axis=1,
        )

        geometric_valid_projection = (
            np.isfinite(
                geometric_grasp_pixels
            ).all(
                axis=1
            )
            & (
                geometric_grasp_points_camera[
                    :,
                    2
                ]
                > 0.0
            )
        )
        geometric_distances[
            ~geometric_valid_projection
        ] = np.inf

        grasp_points_camera[
            geometric_local_indices
        ] = geometric_grasp_points_camera
        grasp_pixels[
            geometric_local_indices
        ] = geometric_grasp_pixels
        distances[
            geometric_local_indices
        ] = geometric_distances

        representative_elapsed = (
            time.perf_counter()
            - representative_start
        )

        print(
            "[MATCH] Representative points time:",
            f"{representative_elapsed:.3f} s",
        )

        semantic_mask = np.zeros(
            len(candidate_indices),
            dtype=bool,
        )
        semantic_mask[
            geometric_local_indices
        ] = (
            geometric_valid_projection
            & (
                geometric_distances
                <= args.semantic_radius_px
            )
        )
        semantic_indices = np.flatnonzero(
            semantic_mask
        )

        print()
        print(
            "[MATCH] Semantic radius:",
            f"{args.semantic_radius_px:.1f} px",
        )
        print(
            "[MATCH] Semantic + geometric candidates:",
            len(semantic_indices),
        )

        result_data[
            "task"
        ] = args.task
        result_data[
            "semantic_point_px"
        ] = semantic_point.tolist()
        result_data[
            "confidence_candidates"
        ] = int(
            len(candidate_indices)
        )
        result_data[
            "geometric_candidates"
        ] = int(
            len(geometric_local_indices)
        )
        result_data[
            "semantic_candidates"
        ] = int(
            len(semantic_indices)
        )
        result_data[
            "geometry_filter_time_s"
        ] = float(
            geometry_elapsed
        )
        result_data[
            "representative_points_time_s"
        ] = float(
            representative_elapsed
        )

        npz_data[
            "semantic_point"
        ] = semantic_point
        npz_data[
            "candidate_indices"
        ] = candidate_indices
        npz_data[
            "geometric_mask"
        ] = geometric_mask
        npz_data[
            "geometric_local_indices"
        ] = geometric_local_indices
        npz_data[
            "grasp_points_camera"
        ] = grasp_points_camera
        npz_data[
            "grasp_pixels"
        ] = grasp_pixels
        npz_data[
            "semantic_distances_px"
        ] = distances
        npz_data[
            "wrist_rotations_deg"
        ] = wrist_rotations
        npz_data[
            "approach_tilts_deg"
        ] = approach_tilts

        if len(semantic_indices) == 0:
            finite_distances = distances[
                np.isfinite(
                    distances
                )
            ]
            nearest = (
                float(
                    finite_distances.min()
                )
                if len(finite_distances)
                else float("inf")
            )

            print()
            print(
                "[RESULT] No semantic-compatible "
                "M2T2 grasp after geometric filtering."
            )
            print(
                "[RESULT] Nearest geometric candidate:",
                f"{nearest:.2f} px",
            )

            result_data[
                "nearest_grasp_distance_px"
            ] = nearest
            result_data[
                "selected_grasp"
            ] = None
        else:
            viable_indices = semantic_indices

            print()
            print(
                "[MATCH] Semantic candidates geometry:"
            )

            semantic_ranking = np.argsort(
                -filtered_confidence[
                    semantic_indices
                ]
            )

            for rank, order_index in enumerate(
                semantic_ranking[:20],
                start=1,
            ):
                local_index = int(
                    semantic_indices[
                        order_index
                    ]
                )
                original_index = int(
                    candidate_indices[
                        local_index
                    ]
                )

                print(
                    f"        #{rank}: "
                    f"M2T2 index={original_index}, "
                    f"confidence="
                    f"{filtered_confidence[local_index]:.6f}, "
                    f"distance="
                    f"{distances[local_index]:.2f} px, "
                    f"tilt="
                    f"{approach_tilts[local_index]:.2f} deg, "
                    f"wrist_rotation="
                    f"{wrist_rotations[local_index]:.2f} deg"
                )

            diagnostic_ranking = np.lexsort(
                (
                    distances[
                        viable_indices
                    ],
                    -filtered_confidence[
                        viable_indices
                    ],
                    wrist_rotations[
                        viable_indices
                    ],
                )
            )
            ranked_viable = viable_indices[
                diagnostic_ranking
            ]

            print()
            print(
                "[MATCH] Top viable grasps:"
            )

            for rank, local_index in enumerate(
                ranked_viable[:10],
                start=1,
            ):
                original_index = int(
                    candidate_indices[
                        local_index
                    ]
                )

                print(
                    f"        #{rank}: "
                    f"M2T2 index={original_index}, "
                    f"confidence="
                    f"{filtered_confidence[local_index]:.6f}, "
                    f"distance="
                    f"{distances[local_index]:.2f} px, "
                    f"tilt="
                    f"{approach_tilts[local_index]:.2f} deg, "
                    f"wrist_rotation="
                    f"{wrist_rotations[local_index]:.2f} deg"
                )

            ranking = np.lexsort(
                (
                    distances[
                        viable_indices
                    ],
                    -filtered_confidence[
                        viable_indices
                    ],
                    wrist_rotations[
                        viable_indices
                    ],
                )
            )

            selected_local_index = int(
                viable_indices[
                    ranking[
                        0
                    ]
                ]
            )
            selected_wrist_rotation = float(
                wrist_rotations[
                    selected_local_index
                ]
            )
            selected_tilt = float(
                approach_tilts[
                    selected_local_index
                ]
            )
            selected_original_index = int(
                candidate_indices[
                    selected_local_index
                ]
            )
            selected_confidence = float(
                filtered_confidence[
                    selected_local_index
                ]
            )
            selected_distance = float(
                distances[
                    selected_local_index
                ]
            )
            selected_grasp_camera = grasps_camera[
                selected_original_index
            ]
            selected_grasp_aligned = grasps_aligned[
                selected_original_index
            ]
            selected_grasp_base = grasps_base[
                selected_original_index
            ]
            selected_pixel = grasp_pixels[
                selected_local_index
            ]

            print()
            print(
                "[RESULT] Selected grasp:"
            )
            print(
                "         M2T2 index:",
                selected_original_index,
            )
            print(
                "         confidence:",
                selected_confidence,
            )
            print(
                "         semantic distance:",
                f"{selected_distance:.3f} px",
            )
            print(
                "         approach tilt:",
                f"{selected_tilt:.2f} deg",
            )
            print(
                "         wrist rotation:",
                f"{selected_wrist_rotation:.2f} deg",
            )
            print(
                "         semantic point:",
                semantic_point,
            )
            print(
                "         grasp pixel:",
                selected_pixel,
            )

            combined_image = draw_combined_result(
                image=image,
                semantic_point=semantic_point,
                grasp_pixels=grasp_pixels,
                selected_index=selected_local_index,
                selected_grasp_camera=selected_grasp_camera,
                K=K,
                semantic_radius=args.semantic_radius_px,
            )
            combined_image.save(
                OUTPUT_DIR
                / "combined_selection.png"
            )

            result_data[
                "selected_m2t2_index"
            ] = selected_original_index
            result_data[
                "selected_confidence"
            ] = selected_confidence
            result_data[
                "selected_distance_px"
            ] = selected_distance
            result_data[
                "selected_approach_tilt_deg"
            ] = selected_tilt
            result_data[
                "selected_wrist_rotation_deg"
            ] = selected_wrist_rotation
            result_data[
                "selected_grasp_pixel"
            ] = selected_pixel.tolist()
            result_data[
                "selected_grasp_base"
            ] = selected_grasp_base.tolist()

            npz_data[
                "selected_m2t2_index"
            ] = np.asarray(
                selected_original_index
            )
            npz_data[
                "selected_grasp_camera"
            ] = selected_grasp_camera
            npz_data[
                "selected_grasp_aligned"
            ] = selected_grasp_aligned
            npz_data[
                "selected_grasp_base"
            ] = selected_grasp_base
    # =============================================================
    # Save results
    # =============================================================
    np.savez_compressed(
        OUTPUT_DIR
        / "results.npz",
        **npz_data,
    )
    with (
        OUTPUT_DIR
        / "metadata.json"
    ).open(
        "w",
        encoding="utf-8",
    ) as stream:
        json.dump(
            result_data,
            stream,
            indent=2,
        )
    print()
    print(
        "=" * 78
    )
    print(
        "[PASS] Test completed."
    )
    print(
        "[TEST] Output:",
        OUTPUT_DIR,
    )
    print(
        "=" * 78
    )
if __name__ == "__main__":
    main()
