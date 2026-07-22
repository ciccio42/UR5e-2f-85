"""Small geometry helpers shared by the scripted pick-place primitives.

Poses are represented as a (position, orientation) pair of numpy arrays:
position is [x, y, z] and orientation is a quaternion [x, y, z, w].
"""
import math
from collections import namedtuple

import numpy as np

# A gripper/object pose: position (3,) + orientation quaternion xyzw (4,).
GripperPose = namedtuple('GripperPose', ['position', 'orientation'])


def slerp(q0, q1, t):
    """Spherical linear interpolation between two xyzw quaternions."""
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)

    dot = np.dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    dot = min(1.0, max(-1.0, dot))

    # Close enough: fall back to linear interpolation + normalize.
    if dot > 0.9995:
        result = q0 + t * (q1 - q0)
        return result / np.linalg.norm(result)

    theta_0 = math.acos(dot)
    theta = theta_0 * t
    q2 = q1 - q0 * dot
    q2 = q2 / np.linalg.norm(q2)
    return q0 * math.cos(theta) + q2 * math.sin(theta)


def compute_num_steps(distance, min_step):
    """Number of waypoints so each step covers >= min_step (except when the
    whole move is already shorter than min_step, which is done in one step)."""
    if distance <= min_step:
        return 1
    return max(1, int(math.floor(distance / min_step)))


def build_linear_waypoints(start_pos, start_quat, end_pos, end_quat, min_step):
    """Return a list of GripperPose waypoints from start to end (start excluded,
    end included), linearly interpolating position and slerp-ing orientation,
    such that each step displaces the TCP by at least ``min_step`` meters.
    """
    start_pos = np.asarray(start_pos, dtype=np.float64)
    end_pos = np.asarray(end_pos, dtype=np.float64)
    distance = float(np.linalg.norm(end_pos - start_pos))

    n_steps = compute_num_steps(distance, min_step)
    waypoints = []
    for i in range(1, n_steps + 1):
        t = i / n_steps
        pos = start_pos + (end_pos - start_pos) * t
        quat = slerp(start_quat, end_quat, t)
        waypoints.append(GripperPose(pos, quat))
    return waypoints
