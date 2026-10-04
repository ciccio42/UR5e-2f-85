import math
from typing import Literal

import yaml
import numpy as np

from results import (
    StructuredSceneObject,
    StructuredSceneRelation,
)

def load_camera_calibration(calibration_path):
    """Load estimated_camera_positions.yaml: {camera_name: {position, orientation_matrix}}.

    Positions/orientations are expressed with respect to the raw ArUco marker
    origin/axes as returned by cv2.aruco's pose estimation (placed at the table
    center), per zed_camera/zed_camera_calibration/scripts/interactive_aruco_calibration.py.
    This is NOT necessarily the same frame as the ``table_0`` TF frame - see
    ARUCO_TO_TABLE0_ROTATION below for the fixed offset between the two.
    """
    with open(calibration_path, 'r') as f:
        raw = yaml.safe_load(f)

    calibration = {}
    for camera_name, entry in raw.items():
        calibration[camera_name] = {
            'position': np.array(entry['position'], dtype=np.float64),
            'orientation_matrix': np.array(entry['orientation_matrix'], dtype=np.float64),
        }
    return calibration

def robust_depth_at(depth_image, u, v, window=5):
    """Median depth (meters) in a small window around (u, v), ignoring NaN/inf/<=0."""
    h, w = depth_image.shape[:2]
    half = window // 2
    u0, u1 = max(0, u - half), min(w, u + half + 1)
    v0, v1 = max(0, v - half), min(h, v + half + 1)
    patch = np.asarray(depth_image[v0:v1, u0:u1], dtype=np.float64).flatten()
    valid = patch[np.isfinite(patch) & (patch > 0.0)]
    if valid.size == 0:
        return None
    return float(np.median(valid))

def deproject_pixel(u, v, depth, camera_matrix):
    """Pinhole deprojection: pixel (u, v) + depth (m) -> 3D point in the camera
    optical frame (X right, Y down, Z forward), using intrinsics K."""
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    x = (u - cx) * depth / fx
    y = (v - cy) * depth / fy
    return np.array([x, y, depth], dtype=np.float64)

def camera_point_to_aruco(point_cam, camera_calib_entry):
    """Apply the camera->ArUco extrinsic transform loaded from the calibration
    yaml (camera position/orientation expressed in the raw ArUco marker frame)."""
    R_cm = camera_calib_entry['orientation_matrix']
    t_cm = camera_calib_entry['position']
    return R_cm @ point_cam + t_cm

# Fixed extrinsic offset between the raw ArUco marker origin/axes (as returned by
# cv2.aruco's pose estimation, i.e. the frame estimated_camera_positions.yaml is
# expressed in) and the table_0 TF frame: zero translation, quaternion (x, y, z, w)
# = (0, 0, 1, 0), i.e. a 180 degree rotation about Z (X and Y flip, Z unchanged).
ARUCO_TO_TABLE0_TRANSLATION = np.zeros(3)
ARUCO_TO_TABLE0_ROTATION = np.array([
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
    [0.0, 0.0, 1.0],
])

def aruco_point_to_table0(point_aruco):
    """Apply the fixed ArUco-origin -> table_0 transform (see
    ARUCO_TO_TABLE0_ROTATION above)."""
    return ARUCO_TO_TABLE0_ROTATION @ point_aruco + ARUCO_TO_TABLE0_TRANSLATION

DirectionMode = Literal[4, 8]

SpatialRelation = Literal[
    "LEFT",
    "RIGHT",
    "UP",
    "DOWN",
    "UP_LEFT",
    "UP_RIGHT",
    "DOWN_LEFT",
    "DOWN_RIGHT",
]


def spatial_relation(
    center_a: tuple[float, float],
    center_b: tuple[float, float],
    directions: DirectionMode = 8,
) -> SpatialRelation:
    """Return the qualitative spatial relation of A relative to B.

    The relation is obtained from the angle of the vector connecting
    the center of B to the center of A.

    Image coordinates follow the standard convention:
    x increases to the right and y increases downward.

    Args:
        center_a: SAM centroid of object A in image pixel coordinates.
        center_b: SAM centroid of object B in image pixel coordinates.
        directions: Number of qualitative directions to use.
            4 -> LEFT, RIGHT, UP, DOWN
            8 -> also includes the four diagonal directions.

    Returns:
        The qualitative spatial relation of A relative to B.
    """
    if directions not in (4, 8):
        raise ValueError(
            f"Unsupported number of directions: {directions}. "
            "Expected 4 or 8."
        )

    a_x, a_y = center_a
    b_x, b_y = center_b

    dx = a_x - b_x
    dy = a_y - b_y

    if dx == 0.0 and dy == 0.0:
        raise ValueError(
            "Cannot determine a spatial relation between "
            "objects with identical centers."
        )

    angle = (
        math.degrees(
            math.atan2(dy, dx)
        )
        + 360.0
    ) % 360.0

    if directions == 4:
        if angle < 45.0 or angle >= 315.0:
            return "RIGHT"

        if angle < 135.0:
            return "DOWN"

        if angle < 225.0:
            return "LEFT"

        return "UP"

    # 8-direction representation.
    if angle < 22.5 or angle >= 337.5:
        return "RIGHT"

    if angle < 67.5:
        return "DOWN_RIGHT"

    if angle < 112.5:
        return "DOWN"

    if angle < 157.5:
        return "DOWN_LEFT"

    if angle < 202.5:
        return "LEFT"

    if angle < 247.5:
        return "UP_LEFT"

    if angle < 292.5:
        return "UP"

    return "UP_RIGHT"

def build_spatial_relations(
    objects: tuple[StructuredSceneObject, ...],
    directions: DirectionMode = 8,
) -> tuple[StructuredSceneRelation, ...]:
    """Build all directed pairwise spatial relations between scene objects."""

    if directions not in (4, 8):
        raise ValueError(
            f"Unsupported number of directions: {directions}. "
            "Expected 4 or 8."
        )

    object_ids = [
        obj.object_id
        for obj in objects
    ]

    if len(object_ids) != len(set(object_ids)):
        raise ValueError(
            "Structured scene objects must have unique object IDs."
        )

    relations: list[StructuredSceneRelation] = []

    for subject in objects:
        for reference in objects:
            if subject.object_id == reference.object_id:
                continue

            relation = spatial_relation(
                center_a=subject.center,
                center_b=reference.center,
                directions=directions,
            )

            relations.append(
                StructuredSceneRelation(
                    subject_object_id=subject.object_id,
                    reference_object_id=reference.object_id,
                    relation=relation,
                )
            )

    return tuple(relations)