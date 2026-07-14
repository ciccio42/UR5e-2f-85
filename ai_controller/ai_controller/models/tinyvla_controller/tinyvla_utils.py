"""
Math helpers + constants for TinyVLAController, ported from
VLA-Bench/robosuite_test/models/tinyvla.py, robosuite_test/robosuite_utils.py
and tinyvla_utils.py (repo: ~/Desktop/Multi-Task-LFD/repo/VLA-Bench).

Only the pieces TinyVLA's inference path actually needs are kept (the source
tinyvla_utils.py also has quaternion/6D-rotation conversions unused here).
"""

import numpy as np
import torch
from PIL import Image

SCALE_FACTOR = 0.05

# Rotates from the EEF frame used by ai_controller's robot-state capture into
# the gripper frame TinyVLA was trained on (a 90-degree rotation about Z).
R_EE_TO_GRIPPER = np.array([
    [0.0, -1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
])

TINYVLA_IMAGE_SIZE = 224

# Crop margins [top, bottom, left, right] applied to the raw front-camera
# image before resizing, matching VLA-Bench/robosuite_test/robosuite_utils.py's
# TASK_CROP - NOT the same numbers as openvla_controller/openvla_utils.py's
# TASK_CROP, since this checkpoint was trained with a different crop window.
TASK_CROP = {
    'pick_place': [20, 25, 80, 75],
    'nut_assembly': [20, 25, 80, 75],
    'stack_block': [20, 25, 80, 75],
    'press_button': [10, 10, 70, 70],
}


def crop_front_image(image: np.ndarray, task_name: str) -> np.ndarray:
    """Crop the raw front-camera image to TinyVLA's training field-of-view, then
    resize to 224x224. NOT applied to the eye-in-hand image (see prepare_observation
    in tinyvla.py, mirroring the reference's prepare_observation)."""
    crop_params = TASK_CROP.get(task_name)
    if crop_params is None:
        return image

    top, bottom_margin, left, right_margin = crop_params
    height, width = image.shape[:2]
    box_h = height - top - bottom_margin
    box_w = width - left - right_margin
    image = image[top:top + box_h, left:left + box_w]
    image = np.array(
        Image.fromarray(image).resize((TINYVLA_IMAGE_SIZE, TINYVLA_IMAGE_SIZE)), dtype=np.uint8
    )
    return image


def normalize_angle(angle, tol=1e-1):
    """Normalize angle to (-pi, pi], where -pi wraps to pi."""
    norm = (angle + np.pi) % (2 * np.pi) - np.pi
    if np.isclose(norm, -np.pi, atol=tol):
        norm = np.pi
    return norm


def euler_to_axis_angle(aa):
    """(roll, pitch, yaw) -> axis-angle vector, ported verbatim from
    VLA-Bench/tinyvla_utils.py's euler_to_axis_angle."""
    roll, pitch, yaw = aa[0], aa[1], aa[2]

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)],
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)],
    ])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1],
    ])
    R = Rz @ Ry @ Rx

    angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))
    if np.isclose(angle, 0):
        return np.zeros(3)

    rx = R[2, 1] - R[1, 2]
    ry = R[0, 2] - R[2, 0]
    rz = R[1, 0] - R[0, 1]
    axis = np.array([rx, ry, rz]) / (2 * np.sin(angle))
    return axis * angle


# ---------------------------------------------------------------------------
# 6D rotation representation -> Euler angles (Zhou et al., CVPR 2019), ported
# from VLA-Bench/tinyvla_utils.py - only the subset needed to decode the
# action head's rot_6d orientation output.
# ---------------------------------------------------------------------------

def _index_from_letter(letter: str) -> int:
    return {"X": 0, "Y": 1, "Z": 2}[letter]


def _angle_from_tan(axis, other_axis, data, horizontal, tait_bryan):
    i1, i2 = {"X": (2, 1), "Y": (0, 2), "Z": (1, 0)}[axis]
    if horizontal:
        i2, i1 = i1, i2
    even = (axis + other_axis) in ["XY", "YZ", "ZX"]
    if horizontal == even:
        return torch.atan2(data[..., i1], data[..., i2])
    if tait_bryan:
        return torch.atan2(-data[..., i2], data[..., i1])
    return torch.atan2(data[..., i2], -data[..., i1])


def rotation_6d_to_matrix(d6: torch.Tensor) -> torch.Tensor:
    """Gram-Schmidt orthogonalization per Zhou et al., Section B."""
    import torch.nn.functional as F

    a1, a2 = d6[..., :3], d6[..., 3:]
    b1 = F.normalize(a1, dim=-1)
    b2 = a2 - (b1 * a2).sum(-1, keepdim=True) * b1
    b2 = F.normalize(b2, dim=-1)
    b3 = torch.cross(b1, b2, dim=-1)
    return torch.stack((b1, b2, b3), dim=-2)


def matrix_to_euler_angles(matrix: torch.Tensor, convention: str) -> torch.Tensor:
    i0 = _index_from_letter(convention[0])
    i2 = _index_from_letter(convention[2])
    tait_bryan = i0 != i2
    if tait_bryan:
        central_angle = torch.asin(matrix[..., i0, i2] * (-1.0 if i0 - i2 in [-1, 2] else 1.0))
    else:
        central_angle = torch.acos(matrix[..., i0, i0])

    o = (
        _angle_from_tan(convention[0], convention[1], matrix[..., i2], False, tait_bryan),
        central_angle,
        _angle_from_tan(convention[2], convention[1], matrix[..., i0, :], True, tait_bryan),
    )
    return torch.stack(o, -1)


def rot_6d_to_euler_angles(rot_6d, convention="XYZ"):
    """Converts a tensor with rot_6d representation to Euler angles."""
    rot_mat = rotation_6d_to_matrix(rot_6d)
    return matrix_to_euler_angles(rot_mat, convention=convention)


# Per-task-ID language prompt used by load_command() when the user doesn't type
# a custom one. Duplicated from openvla_controller/openvla_utils.py's
# COMMAND_TEMPLATE (same physical task IDs, not model-specific) rather than
# imported from it, since that module pulls in tensorflow/prismatic at import
# time - importing it here would make tinyvla_controller depend on the
# OpenVLA-OFT install even when only TinyVLA is used.
COMMAND_TEMPLATE = {
    '00': "Pick the green box and place it into the first bin",
    '01': "Pick the green box and place it into the second bin",
    '02': "Pick the green box and place it into the third bin",
    '03': "Pick the green box and place it into the fourth bin",
    '04': "Pick the yellow box and place it into the first bin",
    '05': "Pick the yellow box and place it into the second bin",
    '06': "Pick the yellow box and place it into the third bin",
    '07': "Pick the yellow box and place it into the fourth bin",
    '08': "Pick the blue box and place it into the first bin",
    '09': "Pick the blue box and place it into the second bin",
    '10': "Pick the blue box and place it into the third bin",
    '11': "Pick the blue box and place it into the fourth bin",
    '12': "Pick the red box and place it into the first bin",
    '13': "Pick the red box and place it into the second bin",
    '14': "Pick the red box and place it into the third bin",
    '15': "Pick the red box and place it into the fourth bin",
}
