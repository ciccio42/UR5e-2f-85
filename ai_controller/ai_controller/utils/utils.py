import importlib
import math
import pickle
import sys
from pathlib import Path
import numpy as np

# Mirrors dataset_collector_pkg/utils.py field names so rollouts saved here
# share the same obs schema as recorded demonstration trajectories.
EEF_POS_NAME = 'eef_pos'
EEF_QUAT_NAME = 'eef_quat'
JOINT_POS_NAME = 'joint_pos'
JOINT_VEL_NAME = 'joint_vel'
GRIPPER_QPOS_NAME = 'gripper_qpos'
GRIPPER_QVEL_NAME = 'gripper_qvel'

def _quat2mat(quat):
    """(x, y, z, w) quaternion -> 3x3 rotation matrix.

    Ported from real_ur5e_pick_place_delta_removed_0_5_10_15/utils.py's quat2mat,
    to exactly match the convention used when generating this model's training data.
    """
    inds = np.array([3, 0, 1, 2])
    q = np.asarray(quat, dtype=np.float64).copy()[inds]
    n = np.dot(q, q)
    if n < np.finfo(float).eps * 4.0:
        return np.identity(3)
    q *= math.sqrt(2.0 / n)
    q2 = np.outer(q, q)
    return np.array([
        [1.0 - q2[2, 2] - q2[3, 3], q2[1, 2] - q2[3, 0], q2[1, 3] + q2[2, 0]],
        [q2[1, 2] + q2[3, 0], 1.0 - q2[1, 1] - q2[3, 3], q2[2, 3] - q2[1, 0]],
        [q2[1, 3] - q2[2, 0], q2[2, 3] + q2[1, 0], 1.0 - q2[1, 1] - q2[2, 2]],
    ])


def _mat2euler_sxyz(rmat):
    """3x3 rotation matrix -> (roll, pitch, yaw) static/extrinsic XYZ Euler angles
    (radians). Ported from the same utils.py's mat2euler(axes='sxyz')."""
    eps = np.finfo(float).eps * 4.0
    M = np.asarray(rmat, dtype=np.float64)[:3, :3]
    cy = math.sqrt(M[0, 0] * M[0, 0] + M[1, 0] * M[1, 0])
    if cy > eps:
        roll = math.atan2(M[2, 1], M[2, 2])
        pitch = math.atan2(-M[2, 0], cy)
        yaw = math.atan2(M[1, 0], M[0, 0])
    else:
        roll = math.atan2(-M[1, 2], M[1, 1])
        pitch = math.atan2(-M[2, 0], cy)
        yaw = 0.0
    return np.array([roll, pitch, yaw])


def _normalize_angle(angle, tol=1e-1):
    """Ported from ur5e_pick_place.py's PickPlaceEnv.normalize_angle."""
    norm = (angle + np.pi) % (2 * np.pi) - np.pi
    if np.isclose(norm, -np.pi, atol=tol):
        norm = np.pi
    return norm

def _euler2quat(roll, pitch, yaw):
    """(roll, pitch, yaw) static/extrinsic XYZ Euler angles -> (x, y, z, w) quaternion.

    Ported from real_ur5e_pick_place_delta_removed_0_5_10_15/utils.py's euler2quat,
    to exactly match the convention used when generating this model's training data.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return np.array([x, y, z, w], dtype=np.float64)
