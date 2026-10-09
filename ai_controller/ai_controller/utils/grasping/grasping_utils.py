from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from PIL import Image, ImageDraw

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

def _get_finger_collision_counts(
    point_cloud: np.ndarray,
    grasps: np.ndarray,
    opening_m: float,
    finger_thickness_m: float,
    finger_width_m: float,
    finger_z_min_m: float,
    finger_z_max_m: float,
    margin_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Count scene points intersecting the left and right finger volumes.

    The point cloud and grasps must use the same coordinate frame.

    Grasp local frame:
        X -> gripper closing direction
        Y -> finger width direction
        Z -> approach direction
    """

    point_cloud = np.asarray(
        point_cloud,
        dtype=np.float32,
    )

    grasps = np.asarray(
        grasps,
        dtype=np.float32,
    )

    left_counts = np.zeros(
        len(grasps),
        dtype=np.int32,
    )

    right_counts = np.zeros(
        len(grasps),
        dtype=np.int32,
    )

    half_opening = (
        opening_m / 2.0
    )

    half_finger_width = (
        finger_width_m / 2.0
    )

    left_x_min = (
        -half_opening
        - finger_thickness_m
        - margin_m
    )

    left_x_max = (
        -half_opening
        + margin_m
    )

    right_x_min = (
        half_opening
        - margin_m
    )

    right_x_max = (
        half_opening
        + finger_thickness_m
        + margin_m
    )

    y_limit = (
        half_finger_width
        + margin_m
    )

    z_min = (
        finger_z_min_m
        - margin_m
    )

    z_max = (
        finger_z_max_m
        + margin_m
    )

    for index, grasp in enumerate(
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

        local_points = (
            point_cloud
            - translation
        ) @ rotation

        x = local_points[:, 0]
        y = local_points[:, 1]
        z = local_points[:, 2]

        common_mask = (
            (np.abs(y) <= y_limit)
            & (z >= z_min)
            & (z <= z_max)
        )

        left_mask = (
            common_mask
            & (x >= left_x_min)
            & (x <= left_x_max)
        )

        right_mask = (
            common_mask
            & (x >= right_x_min)
            & (x <= right_x_max)
        )

        left_counts[index] = int(
            np.count_nonzero(
                left_mask
            )
        )

        right_counts[index] = int(
            np.count_nonzero(
                right_mask
            )
        )

    return (
        left_counts,
        right_counts,
    )

def _save_depth_preview(
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

    if high > low:
        normalized = (
            depth - low
        ) / (
            high - low
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


def _draw_m2t2_candidates(
    image: Image.Image,
    contacts_camera: np.ndarray,
    confidence: np.ndarray,
    K: np.ndarray,
) -> Image.Image:
    visualization = image.copy()

    draw = ImageDraw.Draw(
        visualization
    )

    pixels = _project_points(
        contacts_camera,
        K,
    )

    width, height = image.size

    max_points = 2000

    if len(confidence) > max_points:
        indexes = np.argsort(
            confidence
        )[
            -max_points:
        ]
    else:
        indexes = np.arange(
            len(confidence)
        )

    for index in indexes:
        x, y = pixels[
            index
        ]

        if not (
            np.isfinite(x)
            and np.isfinite(y)
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

    if len(confidence) > 0:
        best_index = int(
            np.argmax(
                confidence
            )
        )

        x, y = pixels[
            best_index
        ]

        if (
            np.isfinite(x)
            and np.isfinite(y)
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


def _draw_grasp(
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

    points_2d = _project_points(
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
        len(points_2d) - 1
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


def _draw_combined_result(
    image: Image.Image,
    semantic_point: np.ndarray,
    grasp_pixels: np.ndarray,
    selected_index: int,
    selected_grasp_camera: np.ndarray,
    K: np.ndarray,
    semantic_radius: float,
) -> Image.Image:
    visualization = image.copy()

    draw = ImageDraw.Draw(
        visualization
    )

    width, height = image.size

    for x, y in grasp_pixels:
        if not (
            np.isfinite(x)
            and np.isfinite(y)
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
        semantic_point[0]
    )

    sy = float(
        semantic_point[1]
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

    _draw_grasp(
        draw,
        selected_grasp_camera,
        K,
    )

    return visualization

def _draw_m2t2_candidates(
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

    pixels = _project_points(
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


def _draw_grasp(
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

    points_2d = _project_points(
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


def _draw_combined_result(
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

    # ---------------------------------------------------------
    # Candidate grasp representative points
    # ---------------------------------------------------------

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

    # ---------------------------------------------------------
    # GraspMolmo semantic point + semantic radius
    # ---------------------------------------------------------

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

    # ---------------------------------------------------------
    # Selected candidate
    # ---------------------------------------------------------

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

    # ---------------------------------------------------------
    # Selected 6-DoF grasp
    # ---------------------------------------------------------

    _draw_grasp(
        draw,
        selected_grasp_camera,
        K,
    )

    return visualization


def _transform_points(
    points: np.ndarray,
    transform: np.ndarray,
) -> np.ndarray:
    points = np.asarray(
        points,
        dtype=np.float64,
    )

    return (
        points
        @ transform[:3, :3].T
        + transform[:3, 3]
    )


def _transform_poses(
    poses: np.ndarray,
    transform: np.ndarray,
) -> np.ndarray:
    poses = np.asarray(
        poses,
        dtype=np.float64,
    )

    if len(poses) == 0:
        return np.empty(
            (0, 4, 4),
            dtype=np.float64,
        )

    return (
        transform[None, ...]
        @ poses
    )


def _pose_to_transform(
    position: np.ndarray,
    quaternion: np.ndarray,
) -> np.ndarray:
    position = np.asarray(
        position,
        dtype=np.float64,
    )

    quaternion = np.asarray(
        quaternion,
        dtype=np.float64,
    )

    if position.shape != (3,):
        raise ValueError(
            "TCP position must have shape (3,)."
        )

    if quaternion.shape != (4,):
        raise ValueError(
            "TCP quaternion must have shape (4,)."
        )

    transform = np.eye(
        4,
        dtype=np.float64,
    )

    transform[:3, :3] = (
        Rotation
        .from_quat(
            quaternion
        )
        .as_matrix()
    )

    transform[:3, 3] = position

    return transform


def _depth_to_point_cloud(
    depth: np.ndarray,
    K: np.ndarray,
    min_depth: float,
    max_depth: float,
    bottom_ignore_px: int,
) -> np.ndarray:
    depth = np.asarray(
        depth,
        dtype=np.float64,
    )

    if depth.ndim != 2:
        raise ValueError(
            "depth must have shape (H, W)."
        )

    height, width = depth.shape

    fx = float(K[0, 0])
    fy = float(K[1, 1])
    cx = float(K[0, 2])
    cy = float(K[1, 2])

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
        np.isfinite(depth)
        & (depth >= min_depth)
        & (depth <= max_depth)
    )

    if bottom_ignore_px > 0:
        valid[
            max(
                0,
                height - bottom_ignore_px,
            ):
        ] = False

    z = depth[valid]

    x = (
        u[valid]
        - cx
    ) * z / fx

    y = (
        v[valid]
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

    return point_cloud.astype(
        np.float32
    )


def _estimate_table_z_offset(
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
        (points[:, 0] >= x_min)
        & (points[:, 0] <= x_max)
        & (points[:, 1] >= y_min)
        & (points[:, 1] <= y_max)
        & (points[:, 2] >= -0.10)
        & (points[:, 2] <= 0.20)
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

    bins = np.arange(
        float(z_values.min()),
        float(z_values.max())
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
        edges[peak_index]
        + edges[peak_index + 1]
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


def _wrist_rotation_deg(
    grasp_aligned: np.ndarray,
    current_tcp_aligned: np.ndarray,
) -> float:
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


def _approach_tilt_deg(
    grasp_aligned: np.ndarray,
) -> float:
    approach = np.asarray(
        grasp_aligned[:3, 2],
        dtype=np.float64,
    )

    norm = np.linalg.norm(
        approach
    )

    if norm < 1e-8:
        return float("inf")

    approach /= norm

    vertical_down = np.array(
        [
            0.0,
            0.0,
            -1.0,
        ],
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


def _project_points(
    points_camera: np.ndarray,
    K: np.ndarray,
) -> np.ndarray:
    points_camera = np.asarray(
        points_camera,
        dtype=np.float64,
    )

    pixels = np.full(
        (
            len(points_camera),
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
            points_camera[:, 2]
            > 0.0
        )
    )

    if not np.any(valid):
        return pixels

    projected = (
        points_camera[valid]
        @ K.T
    )

    projected = (
        projected[:, :2]
        / projected[:, 2:3]
    )

    pixels[valid] = projected

    return pixels


def _get_grasp_points(
    point_cloud: np.ndarray,
    grasps: np.ndarray,
) -> np.ndarray:
    """
    Compute the representative scene point used by the working
    GraspMolmo-M2T2 matching test.
    """

    point_cloud = np.asarray(
        point_cloud,
        dtype=np.float32,
    )

    grasps = np.asarray(
        grasps,
        dtype=np.float32,
    )

    if len(point_cloud) == 0:
        raise ValueError(
            "Cannot compute grasp points from an empty point cloud."
        )

    min_position = (
        GRASP_VOLUME_CENTER
        - GRASP_VOLUME_SIZE / 2.0
    )

    max_position = (
        GRASP_VOLUME_CENTER
        + GRASP_VOLUME_SIZE / 2.0
    )

    query_radius = float(
        np.linalg.norm(
            GRASP_VOLUME_SIZE / 2.0
        )
    ) + 1e-6

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

    for index, grasp in enumerate(
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

        volume_center = (
            translation
            + rotation
            @ GRASP_VOLUME_CENTER
        )

        nearby_indices = (
            tree.query_ball_point(
                volume_center,
                query_radius,
            )
        )

        reference = (
            translation
            + rotation[:, 2]
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

            if np.any(inside):
                candidate_points = (
                    nearby_points[
                        inside
                    ]
                )

                squared_distance = np.sum(
                    (
                        candidate_points
                        - reference
                    )
                    ** 2,
                    axis=1,
                )

                selected_point = (
                    candidate_points[
                        int(
                            np.argmin(
                                squared_distance
                            )
                        )
                    ]
                )

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
            index
        ] = selected_point

    return grasp_points

