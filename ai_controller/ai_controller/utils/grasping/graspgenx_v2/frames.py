
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def validate_transform(T, name="transform"):
    """Validate a rigid transform T_parent_child."""
    T = np.asarray(T, dtype=np.float64)

    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError(f"{name}: invalid shape or values")

    if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-6):
        raise ValueError(f"{name}: invalid homogeneous row")

    R = T[:3, :3]

    if not np.allclose(R.T @ R, np.eye(3), atol=1e-4):
        raise ValueError(f"{name}: rotation not orthonormal")

    if not np.isclose(np.linalg.det(R), 1.0, atol=1e-4):
        raise ValueError(f"{name}: rotation determinant != 1")

    return T


def from_xyz_rpy(xyz, rpy):
    """Build a transform using URDF RPY conventions."""
    x, y, z = map(float, xyz)
    roll, pitch, yaw = map(float, rpy)

    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rx = np.array([
        [1, 0, 0],
        [0, cr, -sr],
        [0, sr, cr],
    ])

    Ry = np.array([
        [cp, 0, sp],
        [0, 1, 0],
        [-sp, 0, cp],
    ])

    Rz = np.array([
        [cy, -sy, 0],
        [sy, cy, 0],
        [0, 0, 1],
    ])

    T = np.eye(4)
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = [x, y, z]

    return validate_transform(T)


def fixed_chain_transform(urdf_path, parent_link, child_link):
    """
    Read a chain of fixed joints from an expanded URDF.

    Returns T_parent_child, mapping child coordinates
    into the parent frame.
    """
    root = ET.parse(Path(urdf_path)).getroot()

    joints_by_child = {
        joint.find("child").get("link"): joint
        for joint in root.findall("joint")
        if joint.find("child") is not None
    }

    T = np.eye(4)
    current = child_link

    for _ in range(len(joints_by_child) + 1):
        if current == parent_link:
            return validate_transform(T)

        joint = joints_by_child.get(current)

        if joint is None:
            raise ValueError(
                f"No joint chain: {parent_link} -> {child_link}"
            )

        if joint.get("type") != "fixed":
            raise ValueError(
                f"Non-fixed joint: {joint.get('name')}"
            )

        origin = joint.find("origin")

        if origin is None:
            xyz_text = "0 0 0"
            rpy_text = "0 0 0"
        else:
            xyz_text = origin.get("xyz", "0 0 0")
            rpy_text = origin.get("rpy", "0 0 0")

        xyz = [float(v) for v in xyz_text.split()]
        rpy = [float(v) for v in rpy_text.split()]

        T = from_xyz_rpy(xyz, rpy) @ T

        current = joint.find("parent").get("link")

    raise ValueError("Joint chain contains a cycle")


def transform_points(T, points):
    """Transform an Nx3 point cloud into the parent frame."""
    T = validate_transform(T)
    points = np.asarray(points, dtype=np.float64)

    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("Expected an Nx3 point cloud")

    return points @ T[:3, :3].T + T[:3, 3]


def table_surface(T_base_table, offset_m):
    """
    Return a physical tabletop plane in base_link.

    offset_m is measured upward from table_0.
    """
    T_base_table = validate_transform(
        T_base_table,
        "T_base_table"
    )

    normal = T_base_table[:3, 2].copy()

    if normal[2] < 0:
        normal *= -1

    origin = (
        T_base_table[:3, 3]
        + float(offset_m) * normal
    )

    return origin, normal
