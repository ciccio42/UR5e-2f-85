#!/usr/bin/env python3
"""Evaluate the UR5e Mimic Video action head on recorded trajectories.

The test samples a predefined set of recorded trajectories, loads the fine-tuned
Cosmos V2W + W2A action head pipeline once, and feeds each trajectory as 5-frame
windows plus the current low-dimensional robot state. It then plots the full
ground-truth trajectory together with all predicted action chunks that can be
compared with available trajectory frames.

The plot projects base_link EEF positions into the zed_front image using the
saved front-camera extrinsics, CameraInfo intrinsics and the preprocessing
resize/padding used to build workspace_rgb.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


SCRIPT_PATH = Path(__file__).resolve()
WORKSPACE_DIR = SCRIPT_PATH.parents[2]
REPO_ROOT = SCRIPT_PATH.parents[3]
MIMIC_MODEL_DIR = WORKSPACE_DIR / "external" / "mimic-video" / "model"
_CONFIG_ENV = os.environ.get("MIMIC_VIDEO_CONTROLLER_CONFIG")
_CONTROLLER_DIR_CANDIDATES = [
    Path(_CONFIG_ENV).expanduser().parent if _CONFIG_ENV else None,
    REPO_ROOT / "ai_controller" / "ai_controller" / "models" / "mimic_video_controller",
    Path("/home/ros2_ws/src/ai_controller/ai_controller/models/mimic_video_controller"),
]
MIMIC_CONTROLLER_DIR = next(
    (path for path in _CONTROLLER_DIR_CANDIDATES if path is not None and path.exists()),
    _CONTROLLER_DIR_CANDIDATES[1],
)

DEFAULT_CONFIG_PATH = MIMIC_CONTROLLER_DIR / "mimic_video_config.yaml"
DEFAULT_DATA_DIR = WORKSPACE_DIR / "processed_data" / "ur5e_pick_place_action"
DEFAULT_OUTPUT_DIR = WORKSPACE_DIR / "test" / "action" / "outputs"
DEFAULT_CHECKPOINT_DIR = WORKSPACE_DIR / "checkpoints"
DEFAULT_DATASET_STATS = DEFAULT_CHECKPOINT_DIR / "dataset_statistics" / "ur5e_action.json"
DEFAULT_ACTION_MODEL = (
    DEFAULT_CHECKPOINT_DIR
    / "posttraining"
    / "world2action"
    / "w2a_ur5e_videmb_v2w_ur5e_finetuned_lr1.000e-04_layer20_bsz1_accum12_train"
    / "checkpoints"
    / "model"
    / "iter_000010000.pt"
)
DEFAULT_EPISODES = (
    "ep_000000",
    "ep_000015",
    "ep_000020",
    "ep_000034",
    "ep_000039",
    "ep_000040",
    "ep_000055",
    "ep_000060",
    "ep_000074",
    "ep_000079",
    "ep_000080",
    "ep_000095",
    "ep_000100",
    "ep_000114",
    "ep_000119",
    "ep_000120",
    "ep_000135",
    "ep_000140",
    "ep_000154",
    "ep_000159",
    "ep_000160",
    "ep_000175",
    "ep_000180",
    "ep_000194",
    "ep_000199",
    "ep_000200",
    "ep_000215",
    "ep_000220",
    "ep_000234",
    "ep_000239",
    "ep_000240",
    "ep_000255",
    "ep_000260",
    "ep_000274",
    "ep_000279",
    "ep_000280",
    "ep_000295",
    "ep_000300",
    "ep_000314",
    "ep_000319",
    "ep_000320",
    "ep_000335",
    "ep_000340",
    "ep_000354",
    "ep_000359",
    "ep_000360",
    "ep_000375",
    "ep_000380",
    "ep_000394",
    "ep_000399",
    "ep_000400",
    "ep_000415",
    "ep_000420",
    "ep_000434",
    "ep_000439",
    "ep_000440",
    "ep_000455",
    "ep_000460",
    "ep_000474",
    "ep_000479",
)

FPS = 10
CONTEXT_FRAMES = 5
ACTION_HORIZON = 15
MODEL_HEIGHT = 480
MODEL_WIDTH = 640
RAW_IMAGE_WIDTH = 672
RAW_IMAGE_HEIGHT = 376
WORKSPACE_IMAGE_WIDTH = 320
WORKSPACE_IMAGE_HEIGHT = 240
CAMERA_FX = 363.8398742675781
CAMERA_FY = 363.8398742675781
CAMERA_CX = 337.5829162597656
CAMERA_CY = 178.53465270996094
T_BASE_CAMERA = (
    (-0.9995271385, 0.0017188839, -0.0307008924, 0.0258804482),
    (0.0210472614, 0.7661305314, -0.6423402694, 1.0325620478),
    (0.0224167827, -0.6426827011, -0.7658044356, 0.4595237268),
    (0.0, 0.0, 0.0, 1.0),
)
STATE_DIM = 10
PROMPT_SHAPE = (1, 512, 1024)
PLOT_AXES = {
    "xy": (0, 1),
    "xz": (0, 2),
    "yz": (1, 2),
}


@dataclass(frozen=True)
class RuntimeConfig:
    experiment_name: str
    video_model_path: Path
    action_model_path: Path
    dataset_statistics_path: Path
    seed: int
    use_cuda_graphs: bool


@dataclass(frozen=True)
class WindowResult:
    window_index: int
    context_start: int
    anchor_index: int
    gt_horizon: int
    output_image: Path
    output_npz: Path
    position_rmse_m: float
    final_position_error_m: float
    mean_rotation_error_deg: float
    final_rotation_error_deg: float
    gripper_accuracy: float


@dataclass(frozen=True)
class PredictionWindow:
    window_index: int
    context_start: int
    anchor_index: int
    pred_action_chunk: object
    gt_action_chunk: object
    pred_absolute: object
    gt_absolute: object


@dataclass(frozen=True)
class EvaluationResult:
    rows: list[WindowResult]
    trajectory_image: Path
    trajectory_npz: Path
    gt_release_index: int | None
    first_pred_release_index: int | None


def prepend_sys_path(path: Path) -> None:
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)


def setup_python_path(checkpoint_dir: Path) -> None:
    os.environ.setdefault("MIMIC_VIDEO_WORKSPACE", str(WORKSPACE_DIR))
    os.environ.setdefault("MIMIC_VIDEO_DATASET_STATISTICS_PATH", str(DEFAULT_DATASET_STATS))
    os.environ.setdefault("COSMOS_PREDICT2_ARGS", f"--checkpoints {checkpoint_dir}")
    prepend_sys_path(MIMIC_MODEL_DIR)
    prepend_sys_path(MIMIC_CONTROLLER_DIR)


def expand_path(path: str | Path) -> Path:
    expanded = os.path.expanduser(os.path.expandvars(str(path)))
    if "$" in expanded:
        raise EnvironmentError(f"Unresolved environment variable in path: {path}")
    return Path(expanded).resolve()


def load_runtime_config(args: argparse.Namespace) -> RuntimeConfig:
    import yaml

    config_path = expand_path(args.config)
    if not config_path.is_file():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    with config_path.open("r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    if not isinstance(config, dict):
        raise ValueError(f"Invalid YAML config: {config_path}")

    video_model_path = expand_path(args.video_model_path or config["video_model_path"])
    action_model_path = expand_path(args.action_model_path)
    dataset_statistics_path = expand_path(
        args.dataset_statistics_path or config["dataset_statistics_path"]
    )

    for label, path in (
        ("video model", video_model_path),
        ("action model", action_model_path),
        ("dataset statistics", dataset_statistics_path),
    ):
        if not path.is_file():
            raise FileNotFoundError(f"Missing {label}: {path}")

    return RuntimeConfig(
        experiment_name=str(config["experiment_name"]),
        video_model_path=video_model_path,
        action_model_path=action_model_path,
        dataset_statistics_path=dataset_statistics_path,
        seed=int(args.seed if args.seed is not None else config.get("seed", 0)),
        use_cuda_graphs=bool(args.use_cuda_graphs or config.get("use_cuda_graphs", False)),
    )


def resolve_episode_path(data_dir: Path, episode: str) -> Path:
    candidate = Path(episode)
    if candidate.is_file():
        return candidate.resolve()

    stem = candidate.stem if candidate.suffix else episode
    if not stem.startswith("ep_"):
        stem = f"ep_{int(stem):06d}"

    path = data_dir / f"{stem}.safetensors"
    if not path.is_file():
        raise FileNotFoundError(f"Episode not found: {path}")
    return path.resolve()


def required_episode_keys() -> tuple[str, ...]:
    return (
        "workspace_rgb",
        "eef_pos_lowdim",
        "eef_rot_lowdim",
        "gripper_lowdim",
        "eef_pos_ref_delta_lowdim",
        "eef_rot_ref_delta_lowdim",
        "gripper_action_lowdim",
        "language_instruction",
        "language_embedding",
    )


def load_episode(path: Path) -> dict:
    import safetensors.numpy as st

    data = st.load_file(path)
    missing = [key for key in required_episode_keys() if key not in data]
    if missing:
        raise KeyError(f"Missing keys in {path}: {missing}")

    num_frames = len(data["workspace_rgb"])
    for key in (
        "eef_pos_lowdim",
        "eef_rot_lowdim",
        "gripper_lowdim",
        "eef_pos_ref_delta_lowdim",
        "eef_rot_ref_delta_lowdim",
        "gripper_action_lowdim",
    ):
        if len(data[key]) != num_frames:
            raise ValueError(
                f"Length mismatch in {path.name}: workspace_rgb={num_frames}, "
                f"{key}={len(data[key])}"
            )
    return data


def decode_language(data: dict) -> str:
    import numpy as np

    raw = np.asarray(data["language_instruction"], dtype=np.uint8).reshape(-1).tobytes()
    return raw.decode("utf-8", errors="replace").strip()


def read_prompt_embedding(data: dict):
    import numpy as np
    import torch

    embedding = np.asarray(data["language_embedding"], dtype=np.float32)
    if embedding.ndim == 2:
        embedding = embedding[None]
    if tuple(embedding.shape) != PROMPT_SHAPE:
        raise ValueError(
            f"Unexpected language_embedding shape: {tuple(embedding.shape)}, "
            f"expected {PROMPT_SHAPE}"
        )
    return torch.from_numpy(np.ascontiguousarray(embedding))


def make_context_tensor(frames):
    import numpy as np
    import torch
    import torch.nn.functional as F

    frames = np.asarray(frames)
    if frames.ndim != 4 or frames.shape[-1] != 3 or frames.dtype != np.uint8:
        raise ValueError(
            "workspace_rgb context must be uint8 and shaped (T, H, W, 3); "
            f"got {frames.shape} {frames.dtype}"
        )

    tensor = torch.from_numpy(np.ascontiguousarray(frames)).permute(0, 3, 1, 2).float()
    tensor = F.interpolate(
        tensor,
        size=(MODEL_HEIGHT, MODEL_WIDTH),
        mode="bilinear",
        align_corners=False,
        antialias=True,
    )
    tensor = 2.0 * (tensor / 255.0 - 0.5)
    return tensor.permute(1, 0, 2, 3).unsqueeze(0).contiguous()


def make_state_tensor(data: dict, anchor_index: int):
    import numpy as np
    import torch
    from mimic_video_utils import build_lowdim_state

    position = np.asarray(data["eef_pos_lowdim"][anchor_index], dtype=np.float64)
    quaternion = np.asarray(data["eef_rot_lowdim"][anchor_index], dtype=np.float64)
    gripper = float(np.asarray(data["gripper_lowdim"][anchor_index]).reshape(-1)[0])
    state = build_lowdim_state(position, quaternion, gripper)
    if state.shape != (STATE_DIM,):
        raise ValueError(f"Unexpected state shape: {state.shape}")
    return torch.from_numpy(state[None, None].astype(np.float32))


def rotation_targets_to_6d(rotations):
    import numpy as np
    from mimic_video_utils import rotation_matrix_to_6d

    rotations = np.asarray(rotations, dtype=np.float32)
    if rotations.ndim == 2 and rotations.shape[1] == 6:
        return np.ascontiguousarray(rotations)
    if rotations.ndim == 2 and rotations.shape[1] == 9:
        rotations = rotations.reshape(len(rotations), 3, 3)
    if rotations.ndim == 3 and rotations.shape[1:] == (3, 3):
        return np.stack([rotation_matrix_to_6d(rotation) for rotation in rotations])
    raise ValueError(f"Unsupported rotation target shape: {rotations.shape}")


def make_gt_action_chunk(data: dict, anchor_index: int, horizon: int) -> object:
    import numpy as np

    num_actions = len(data["eef_pos_ref_delta_lowdim"])
    end = min(anchor_index + horizon, num_actions)
    if end <= anchor_index:
        raise ValueError(f"No GT actions available from anchor {anchor_index}")

    pos_delta = np.asarray(data["eef_pos_ref_delta_lowdim"][anchor_index:end], dtype=np.float32)
    rot_6d = rotation_targets_to_6d(data["eef_rot_ref_delta_lowdim"][anchor_index:end])
    gripper = np.asarray(data["gripper_action_lowdim"][anchor_index:end], dtype=np.float32)
    gripper = gripper.reshape(len(gripper), -1)[:, :1]

    if len(pos_delta) != len(rot_6d) or len(pos_delta) != len(gripper):
        raise ValueError(
            f"GT action components have inconsistent lengths from anchor {anchor_index}: "
            f"pos={len(pos_delta)}, rot={len(rot_6d)}, gripper={len(gripper)}"
        )
    return np.ascontiguousarray(np.concatenate((pos_delta, rot_6d, gripper), axis=1))


def absolute_chunk(action_chunk, data: dict, anchor_index: int):
    import numpy as np
    from mimic_video_utils import action_chunk_to_absolute_poses

    position = np.asarray(data["eef_pos_lowdim"][anchor_index], dtype=np.float64)
    quaternion = np.asarray(data["eef_rot_lowdim"][anchor_index], dtype=np.float64)
    gripper = float(np.asarray(data["gripper_lowdim"][anchor_index]).reshape(-1)[0])
    absolute, _ = action_chunk_to_absolute_poses(
        action_chunk=action_chunk,
        current_position=position,
        current_quaternion=quaternion,
        gripper_closed=gripper >= 0.5,
    )
    return absolute


def orientation_errors_deg(pred_quat, gt_quat):
    import numpy as np
    from scipy.spatial.transform import Rotation

    pred_rot = Rotation.from_quat(pred_quat)
    gt_rot = Rotation.from_quat(gt_quat)
    delta = pred_rot * gt_rot.inv()
    return np.degrees(delta.magnitude())


def compute_metrics(pred_abs, gt_abs) -> dict[str, float]:
    import numpy as np

    if len(pred_abs) != len(gt_abs):
        raise ValueError(f"Metric inputs must have the same length, got {len(pred_abs)} and {len(gt_abs)}")

    pos_error = np.linalg.norm(pred_abs[:, :3] - gt_abs[:, :3], axis=1)
    rot_error = orientation_errors_deg(pred_abs[:, 3:7], gt_abs[:, 3:7])
    gripper_accuracy = np.mean(pred_abs[:, 7] == gt_abs[:, 7])
    return {
        "position_rmse_m": float(np.sqrt(np.mean(pos_error**2))),
        "final_position_error_m": float(pos_error[-1]),
        "mean_rotation_error_deg": float(np.mean(rot_error)),
        "final_rotation_error_deg": float(rot_error[-1]),
        "gripper_accuracy": float(gripper_accuracy),
    }


def gripper_closed(values, threshold: float = 0.5):
    import numpy as np

    values = np.asarray(values, dtype=np.float32).reshape(-1)
    return values >= threshold


def absolute_gripper_closed(absolute_actions):
    import numpy as np

    absolute_actions = np.asarray(absolute_actions, dtype=np.float32)
    if absolute_actions.ndim != 2 or absolute_actions.shape[1] < 8:
        raise ValueError(f"Expected absolute action shape (H, >=8), got {absolute_actions.shape}")
    return absolute_actions[:, 7] >= 127.5


def state_transition_indices(closed_states) -> list[int]:
    import numpy as np

    closed_states = np.asarray(closed_states, dtype=bool)
    if len(closed_states) < 2:
        return []
    return (np.flatnonzero(closed_states[1:] != closed_states[:-1]) + 1).astype(int).tolist()


def first_release_index(closed_states) -> int | None:
    import numpy as np

    closed_states = np.asarray(closed_states, dtype=bool)
    if len(closed_states) < 2:
        return None
    releases = np.flatnonzero(closed_states[:-1] & ~closed_states[1:]) + 1
    return int(releases[0]) if len(releases) else None


def gt_closed_states(data: dict):
    return gripper_closed(data["gripper_lowdim"])


def gt_release_index(data: dict) -> int | None:
    return first_release_index(gt_closed_states(data))


def prediction_release_indices(
    predictions: list[PredictionWindow],
    data: dict,
    plot_limit_index: int | None,
) -> list[int]:
    import numpy as np

    releases: list[int] = []
    gt_closed = gt_closed_states(data)
    for prediction in predictions:
        initial_closed = bool(gt_closed[prediction.anchor_index])
        horizon = plot_horizon(prediction, plot_limit_index)
        pred_closed = absolute_gripper_closed(prediction.pred_absolute[:horizon])
        sequence = np.concatenate((np.array([initial_closed], dtype=bool), pred_closed))
        local_releases = np.flatnonzero(sequence[:-1] & ~sequence[1:])
        releases.extend(int(prediction.anchor_index + local_index) for local_index in local_releases)
    return sorted(releases)


def plot_horizon(prediction: PredictionWindow, limit_index: int | None) -> int:
    horizon = len(prediction.gt_absolute)
    if limit_index is not None:
        horizon = min(horizon, max(0, limit_index - prediction.anchor_index + 1))
    return horizon


def valid_anchor_indices(
    num_frames: int,
    start_frame: int,
    context_frames: int,
    stride_frames: int,
    max_windows: int | None,
    stop_frame: int | None,
) -> list[tuple[int, int]]:
    if start_frame < 0:
        raise ValueError("--start-frame must be >= 0")
    if context_frames <= 0:
        raise ValueError("--context-frames must be positive")
    if stride_frames <= 0:
        raise ValueError("--stride-frames must be positive")

    windows: list[tuple[int, int]] = []
    context_start = start_frame
    while True:
        anchor = context_start + context_frames - 1
        if context_start < 0 or anchor >= num_frames:
            break
        if stop_frame is not None and anchor > stop_frame:
            break
        windows.append((context_start, anchor))
        if max_windows is not None and len(windows) >= max_windows:
            break
        context_start += stride_frames

    return windows


def axis_bounds(data: dict, pred_abs, gt_abs, axes: tuple[int, int]):
    import numpy as np

    episode_positions = np.asarray(data["eef_pos_lowdim"], dtype=np.float32)[:, list(axes)]
    chunk_positions = np.concatenate((pred_abs[:, list(axes)], gt_abs[:, list(axes)]), axis=0)
    points = np.concatenate((episode_positions, chunk_positions), axis=0)
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    span = np.maximum(maxs - mins, 1e-4)
    margin = span * 0.08
    return mins - margin, maxs + margin


def map_points_to_panel(points, axes, bounds, rect):
    import numpy as np

    x0, y0, x1, y1 = rect
    mins, maxs = bounds
    xy = np.asarray(points, dtype=np.float32)[:, list(axes)]
    norm = (xy - mins) / np.maximum(maxs - mins, 1e-6)
    px = x0 + norm[:, 0] * (x1 - x0)
    py = y1 - norm[:, 1] * (y1 - y0)
    return np.stack((px, py), axis=1).round().astype(int)


def draw_polyline(image, points, color, thickness: int = 2) -> None:
    import cv2

    for start, end in zip(points[:-1], points[1:], strict=False):
        cv2.line(
            image,
            tuple(start.tolist()),
            tuple(end.tolist()),
            color,
            thickness,
            lineType=cv2.LINE_AA,
        )
    for index, point in enumerate(points):
        radius = 4 if index in (0, len(points) - 1) else 3
        cv2.circle(image, tuple(point.tolist()), radius, color, -1, lineType=cv2.LINE_AA)


def draw_state_colored_polyline(
    image,
    points,
    closed_states,
    open_color,
    closed_color,
    thickness: int,
) -> None:
    import cv2

    if len(points) != len(closed_states):
        raise ValueError(f"points/states mismatch: {len(points)} != {len(closed_states)}")
    for index in range(1, len(points)):
        color = closed_color if closed_states[index] else open_color
        cv2.line(
            image,
            tuple(points[index - 1].tolist()),
            tuple(points[index].tolist()),
            color,
            thickness,
            lineType=cv2.LINE_AA,
        )


def trajectory_axis_bounds(
    data: dict,
    predictions: list[PredictionWindow],
    axes: tuple[int, int],
    plot_limit_index: int | None,
):
    import numpy as np

    all_points = [np.asarray(data["eef_pos_lowdim"], dtype=np.float32)[:, list(axes)]]
    for prediction in predictions:
        horizon = plot_horizon(prediction, plot_limit_index)
        if horizon == 0:
            continue
        all_points.append(
            np.asarray(prediction.pred_absolute[:horizon], dtype=np.float32)[:, list(axes)]
        )
    points = np.concatenate(all_points, axis=0)
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    span = np.maximum(maxs - mins, 1e-4)
    margin = span * 0.08
    return mins - margin, maxs + margin


def put_label(image, text: str, point, color, offset: tuple[int, int]) -> None:
    import cv2

    x, y = int(point[0] + offset[0]), int(point[1] + offset[1])
    cv2.putText(
        image,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (20, 20, 20),
        3,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        color,
        1,
        cv2.LINE_AA,
    )


def draw_transition_marker(image, point, label: str, color, offset: tuple[int, int]) -> None:
    import cv2

    center = tuple(point.tolist())
    cv2.circle(image, center, 8, (255, 255, 255), -1, lineType=cv2.LINE_AA)
    cv2.circle(image, center, 8, color, 2, lineType=cv2.LINE_AA)
    put_label(image, label, point, color, offset)


def preprocessing_resize_padding() -> tuple[float, int, int, int, int]:
    scale = min(
        WORKSPACE_IMAGE_WIDTH / RAW_IMAGE_WIDTH,
        WORKSPACE_IMAGE_HEIGHT / RAW_IMAGE_HEIGHT,
    )
    resized_width = round(RAW_IMAGE_WIDTH * scale)
    resized_height = round(RAW_IMAGE_HEIGHT * scale)
    left = (WORKSPACE_IMAGE_WIDTH - resized_width) // 2
    top = (WORKSPACE_IMAGE_HEIGHT - resized_height) // 2
    return scale, left, top, resized_width, resized_height


def project_base_points_to_png(points):
    import numpy as np

    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"Expected base_link points shaped (N, 3), got {points.shape}")
    if len(points) == 0:
        return np.empty((0, 2), dtype=np.int32), np.empty((0,), dtype=bool)

    t_base_camera = np.asarray(T_BASE_CAMERA, dtype=np.float64)
    t_camera_base = np.linalg.inv(t_base_camera)
    homogeneous = np.concatenate((points, np.ones((len(points), 1), dtype=np.float64)), axis=1)
    camera_points = (t_camera_base @ homogeneous.T).T[:, :3]

    z = camera_points[:, 2]
    valid = z > 1e-6
    safe_z = np.where(valid, z, 1.0)
    u_raw = CAMERA_FX * camera_points[:, 0] / safe_z + CAMERA_CX
    v_raw = CAMERA_FY * camera_points[:, 1] / safe_z + CAMERA_CY
    valid &= (u_raw >= 0.0) & (u_raw < RAW_IMAGE_WIDTH)
    valid &= (v_raw >= 0.0) & (v_raw < RAW_IMAGE_HEIGHT)

    scale, left, top, resized_width, resized_height = preprocessing_resize_padding()
    u_workspace = u_raw * scale + left
    v_workspace = v_raw * scale + top
    valid &= (u_workspace >= left) & (u_workspace < left + resized_width)
    valid &= (v_workspace >= top) & (v_workspace < top + resized_height)

    u_png = u_workspace * (MODEL_WIDTH / WORKSPACE_IMAGE_WIDTH)
    v_png = v_workspace * (MODEL_HEIGHT / WORKSPACE_IMAGE_HEIGHT)
    valid &= (u_png >= 0.0) & (u_png < MODEL_WIDTH)
    valid &= (v_png >= 0.0) & (v_png < MODEL_HEIGHT)

    return np.stack((u_png, v_png), axis=1).round().astype(np.int32), valid


def draw_projected_state_polyline(
    image,
    points,
    valid,
    closed_states,
    open_color,
    closed_color,
    thickness: int,
) -> None:
    import cv2

    if len(points) != len(valid) or len(points) != len(closed_states):
        raise ValueError(
            f"projection/state mismatch: points={len(points)}, valid={len(valid)}, "
            f"states={len(closed_states)}"
        )
    for index in range(1, len(points)):
        if not (valid[index - 1] and valid[index]):
            continue
        color = closed_color if closed_states[index] else open_color
        cv2.line(
            image,
            tuple(points[index - 1].tolist()),
            tuple(points[index].tolist()),
            color,
            thickness,
            lineType=cv2.LINE_AA,
        )


def draw_projected_transition_marker(
    image,
    point,
    is_valid: bool,
    label: str,
    color,
    offset: tuple[int, int],
) -> None:
    if is_valid:
        draw_transition_marker(image, point, label, color, offset)


def draw_gripper_legend(image, left: int, top: int, colors: dict[str, tuple[int, int, int]]) -> None:
    import cv2

    items = (
        ("GT open", colors["gt_open"]),
        ("GT closed", colors["gt_closed"]),
        ("Pred open", colors["pred_open"]),
        ("Pred closed", colors["pred_closed"]),
    )
    x = left
    for label, color in items:
        cv2.line(image, (x, top), (x + 24, top), color, 4, lineType=cv2.LINE_AA)
        cv2.putText(
            image,
            label,
            (x + 30, top + 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (245, 245, 245),
            1,
            cv2.LINE_AA,
        )
        x += 132


def render_trajectory_overlay(
    frame,
    data: dict,
    predictions: list[PredictionWindow],
    plane: str,
    output_path: Path,
    release_index: int | None,
    first_pred_release: int | None,
    plot_limit_index: int | None,
) -> None:
    import cv2
    import numpy as np
    import imageio.v2 as imageio

    frame = np.asarray(frame, dtype=np.uint8)
    image = cv2.resize(frame, (MODEL_WIDTH, MODEL_HEIGHT), interpolation=cv2.INTER_LINEAR)
    image = np.ascontiguousarray(image)

    colors = {
        "gt_open": (255, 210, 60),
        "gt_closed": (80, 235, 125),
        "pred_open": (0, 165, 255),
        "pred_closed": (255, 80, 255),
    }
    gt_positions = np.asarray(data["eef_pos_lowdim"], dtype=np.float32)[:, :3]
    gt_closed = gt_closed_states(data)
    gt_points, gt_valid = project_base_points_to_png(gt_positions)

    draw_projected_state_polyline(
        image,
        gt_points,
        gt_valid,
        gt_closed,
        colors["gt_open"],
        colors["gt_closed"],
        thickness=3,
    )

    for prediction in predictions:
        anchor = prediction.anchor_index
        start_pos = gt_positions[anchor][None]
        comparable_horizon = plot_horizon(prediction, plot_limit_index)
        if comparable_horizon == 0:
            continue
        pred_abs_plot = prediction.pred_absolute[:comparable_horizon]
        pred_positions = np.concatenate((start_pos, pred_abs_plot[:, :3]), axis=0)
        pred_closed = np.concatenate(
            (
                np.array([gt_closed[anchor]], dtype=bool),
                absolute_gripper_closed(pred_abs_plot),
            )
        )
        pred_points, pred_valid = project_base_points_to_png(pred_positions)
        draw_projected_state_polyline(
            image,
            pred_points,
            pred_valid,
            pred_closed,
            colors["pred_open"],
            colors["pred_closed"],
            thickness=2,
        )
        if pred_valid[0]:
            cv2.circle(image, tuple(pred_points[0].tolist()), 4, (220, 220, 220), -1, lineType=cv2.LINE_AA)

    for transition_index in state_transition_indices(gt_closed):
        label = "GT closed" if gt_closed[transition_index] else "GT open"
        color = colors["gt_closed"] if gt_closed[transition_index] else colors["gt_open"]
        draw_projected_transition_marker(
            image,
            gt_points[transition_index],
            bool(gt_valid[transition_index]),
            label,
            color,
            (10, -10),
        )

    for prediction in predictions:
        anchor = prediction.anchor_index
        comparable_horizon = plot_horizon(prediction, plot_limit_index)
        if comparable_horizon == 0:
            continue
        pred_abs_plot = prediction.pred_absolute[:comparable_horizon]
        pred_closed = np.concatenate(
            (
                np.array([gt_closed[anchor]], dtype=bool),
                absolute_gripper_closed(pred_abs_plot),
            )
        )
        pred_positions = np.concatenate((gt_positions[anchor][None], pred_abs_plot[:, :3]), axis=0)
        pred_points, pred_valid = project_base_points_to_png(pred_positions)
        for local_index in state_transition_indices(pred_closed):
            label = "Pred closed" if pred_closed[local_index] else "Pred open"
            color = colors["pred_closed"] if pred_closed[local_index] else colors["pred_open"]
            draw_projected_transition_marker(
                image,
                pred_points[local_index],
                bool(pred_valid[local_index]),
                label,
                color,
                (10, 16),
            )

    banner = image.copy()
    cv2.rectangle(banner, (0, 0), (MODEL_WIDTH, 104), (10, 10, 10), -1)
    image = cv2.addWeighted(banner, 0.62, image, 0.38, 0.0)

    title = f"zed_front projection  windows={len(predictions)}"
    release_text = (
        f"GT release frame {release_index}" if release_index is not None else "GT release not found"
    )
    pred_release_text = (
        f"first Pred release step {first_pred_release}"
        if first_pred_release is not None
        else "Pred release not found"
    )
    cv2.putText(
        image,
        title,
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (245, 245, 245),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        f"{release_text} | {pred_release_text}",
        (20, 58),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        (245, 245, 245),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        "base_link -> zed_front pixels, then preprocessing resize/padding",
        (20, 82),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (245, 245, 245),
        1,
        cv2.LINE_AA,
    )
    draw_gripper_legend(image, 20, 100, colors)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    imageio.imwrite(output_path, image)


def render_overlay(
    frame,
    data: dict,
    pred_abs,
    gt_abs,
    anchor_index: int,
    window_index: int,
    plane: str,
    metrics: dict[str, float],
    output_path: Path,
) -> None:
    import cv2
    import numpy as np
    import imageio.v2 as imageio

    frame = np.asarray(frame, dtype=np.uint8)
    image = cv2.resize(frame, (MODEL_WIDTH, MODEL_HEIGHT), interpolation=cv2.INTER_LINEAR)
    image = np.ascontiguousarray(image)

    start_pos = np.asarray(data["eef_pos_lowdim"][anchor_index], dtype=np.float32)[None]
    pred_path = np.concatenate((start_pos, pred_abs[:, :3]), axis=0)
    gt_path = np.concatenate((start_pos, gt_abs[:, :3]), axis=0)
    pred_px, pred_valid = project_base_points_to_png(pred_path)
    gt_px, gt_valid = project_base_points_to_png(gt_path)

    colors = {
        "gt_open": (255, 210, 60),
        "gt_closed": (80, 235, 125),
        "pred_open": (0, 165, 255),
        "pred_closed": (255, 80, 255),
    }
    text_color = (245, 245, 245)
    muted = (205, 205, 205)

    gt_closed_sequence = np.concatenate(
        (
            np.array([gt_closed_states(data)[anchor_index]], dtype=bool),
            absolute_gripper_closed(gt_abs),
        )
    )
    pred_closed_sequence = np.concatenate(
        (
            np.array([gt_closed_states(data)[anchor_index]], dtype=bool),
            absolute_gripper_closed(pred_abs),
        )
    )
    draw_projected_state_polyline(
        image,
        gt_px,
        gt_valid,
        gt_closed_sequence,
        colors["gt_open"],
        colors["gt_closed"],
        thickness=3,
    )
    draw_projected_state_polyline(
        image,
        pred_px,
        pred_valid,
        pred_closed_sequence,
        colors["pred_open"],
        colors["pred_closed"],
        thickness=2,
    )
    if gt_valid[0]:
        cv2.circle(image, tuple(gt_px[0].tolist()), 6, (255, 255, 255), -1, lineType=cv2.LINE_AA)

    for transition_index in state_transition_indices(gt_closed_sequence):
        label = "GT closed" if gt_closed_sequence[transition_index] else "GT open"
        color = colors["gt_closed"] if gt_closed_sequence[transition_index] else colors["gt_open"]
        draw_projected_transition_marker(
            image,
            gt_px[transition_index],
            bool(gt_valid[transition_index]),
            label,
            color,
            (10, -10),
        )
    for transition_index in state_transition_indices(pred_closed_sequence):
        label = "Pred closed" if pred_closed_sequence[transition_index] else "Pred open"
        color = colors["pred_closed"] if pred_closed_sequence[transition_index] else colors["pred_open"]
        draw_projected_transition_marker(
            image,
            pred_px[transition_index],
            bool(pred_valid[transition_index]),
            label,
            color,
            (10, 16),
        )

    banner = image.copy()
    cv2.rectangle(banner, (0, 0), (MODEL_WIDTH, 96), (10, 10, 10), -1)
    image = cv2.addWeighted(banner, 0.62, image, 0.38, 0.0)

    left, top = 20, 26
    cv2.putText(
        image,
        f"window {window_index}  anchor {anchor_index}  zed_front projection",
        (left, top),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        text_color,
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        f"pos RMSE {metrics['position_rmse_m']:.4f} m",
        (left, top + 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        muted,
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        f"rot mean {metrics['mean_rotation_error_deg']:.1f} deg  grip {metrics['gripper_accuracy']:.2f}",
        (left, top + 46),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        muted,
        1,
        cv2.LINE_AA,
    )
    draw_gripper_legend(image, 20, 88, colors)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    imageio.imwrite(output_path, image)


def write_window_npz(output_path: Path, **arrays) -> None:
    import numpy as np

    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output_path, **arrays)


def write_trajectory_npz(
    output_path: Path,
    data: dict,
    predictions: list[PredictionWindow],
    release_index: int | None,
    first_pred_release: int | None,
    horizon: int,
) -> None:
    import numpy as np

    pred_absolute = np.stack([prediction.pred_absolute for prediction in predictions])
    pred_action_chunks = np.stack([prediction.pred_action_chunk for prediction in predictions])
    gt_absolute = np.full((len(predictions), horizon, 8), np.nan, dtype=np.float32)
    gt_action_chunks = np.full((len(predictions), horizon, 10), np.nan, dtype=np.float32)
    gt_horizons = np.empty((len(predictions),), dtype=np.int64)
    anchor_indices = np.empty((len(predictions),), dtype=np.int64)
    context_starts = np.empty((len(predictions),), dtype=np.int64)

    for index, prediction in enumerate(predictions):
        gt_horizon = len(prediction.gt_absolute)
        gt_absolute[index, :gt_horizon] = prediction.gt_absolute
        gt_action_chunks[index, :gt_horizon] = prediction.gt_action_chunk
        gt_horizons[index] = gt_horizon
        anchor_indices[index] = prediction.anchor_index
        context_starts[index] = prediction.context_start

    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output_path,
        pred_absolute=pred_absolute,
        pred_action_chunks=pred_action_chunks,
        gt_absolute=gt_absolute,
        gt_action_chunks=gt_action_chunks,
        gt_horizons=gt_horizons,
        anchor_indices=anchor_indices,
        context_starts=context_starts,
        gt_positions=np.asarray(data["eef_pos_lowdim"], dtype=np.float32),
        gt_quaternions=np.asarray(data["eef_rot_lowdim"], dtype=np.float32),
        gt_gripper_closed=gt_closed_states(data),
        gt_release_index=np.array(-1 if release_index is None else release_index, dtype=np.int64),
        first_pred_release_index=np.array(
            -1 if first_pred_release is None else first_pred_release,
            dtype=np.int64,
        ),
    )


def write_metrics_csv(path: Path, rows: Iterable[WindowResult]) -> None:
    rows = list(rows)
    if not rows:
        return

    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "window_index",
        "context_start",
        "anchor_index",
        "gt_horizon",
        "position_rmse_m",
        "final_position_error_m",
        "mean_rotation_error_deg",
        "final_rotation_error_deg",
        "gripper_accuracy",
        "output_image",
        "output_npz",
    ]
    with path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    "window_index": row.window_index,
                    "context_start": row.context_start,
                    "anchor_index": row.anchor_index,
                    "gt_horizon": row.gt_horizon,
                    "position_rmse_m": row.position_rmse_m,
                    "final_position_error_m": row.final_position_error_m,
                    "mean_rotation_error_deg": row.mean_rotation_error_deg,
                    "final_rotation_error_deg": row.final_rotation_error_deg,
                    "gripper_accuracy": row.gripper_accuracy,
                    "output_image": str(row.output_image),
                    "output_npz": str(row.output_npz),
                }
            )


def write_summary(path: Path, config: RuntimeConfig, episode_path: Path, result: EvaluationResult) -> None:
    rows = result.rows
    payload = {
        "episode": str(episode_path),
        "experiment_name": config.experiment_name,
        "video_model_path": str(config.video_model_path),
        "action_model_path": str(config.action_model_path),
        "dataset_statistics_path": str(config.dataset_statistics_path),
        "num_windows": len(rows),
        "gt_release_index": result.gt_release_index,
        "first_pred_release_index": result.first_pred_release_index,
        "trajectory_image": str(result.trajectory_image),
        "trajectory_npz": str(result.trajectory_npz),
        "mean_position_rmse_m": (
            sum(row.position_rmse_m for row in rows) / len(rows) if rows else None
        ),
        "mean_rotation_error_deg": (
            sum(row.mean_rotation_error_deg for row in rows) / len(rows) if rows else None
        ),
        "mean_gripper_accuracy": (
            sum(row.gripper_accuracy for row in rows) / len(rows) if rows else None
        ),
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def evaluate_windows(
    policy,
    data: dict,
    episode_path: Path,
    runtime: RuntimeConfig,
    output_dir: Path,
    plane: str,
    context_frames: int,
    stride_frames: int,
    horizon: int,
    max_windows: int | None,
    start_frame: int,
    stop_at_release: bool,
) -> EvaluationResult:
    import numpy as np
    import torch

    release_index = gt_release_index(data)
    stop_frame = release_index if stop_at_release else None
    plot_limit_index = stop_frame
    windows = valid_anchor_indices(
        num_frames=len(data["workspace_rgb"]),
        start_frame=start_frame,
        context_frames=context_frames,
        stride_frames=stride_frames,
        max_windows=max_windows,
        stop_frame=stop_frame,
    )
    if not windows:
        raise ValueError(
            f"No valid windows for {episode_path.name}: frames={len(data['workspace_rgb'])}, "
            f"start={start_frame}, context={context_frames}, stride={stride_frames}, "
            f"stop_frame={stop_frame}"
        )

    command = decode_language(data)
    prompt_embedding = read_prompt_embedding(data)
    run_dir = output_dir / runtime.action_model_path.stem / episode_path.stem
    run_dir.mkdir(parents=True, exist_ok=True)

    if release_index is None:
        print("  GT release: not found; predictions will be sampled through the trajectory tail")
    else:
        print(f"  GT release: frame {release_index}")
    if stop_at_release:
        print("  Sampling mode: stop at GT release")
    else:
        print("  Sampling mode: full trajectory tail")

    rows: list[WindowResult] = []
    predictions: list[PredictionWindow] = []
    for window_index, (context_start, anchor_index) in enumerate(windows):
        context = data["workspace_rgb"][context_start : anchor_index + 1]
        input_vid = make_context_tensor(context)
        state = make_state_tensor(data, anchor_index)
        gt_chunk = make_gt_action_chunk(data, anchor_index, horizon)

        print(
            f"[{window_index + 1}/{len(windows)}] "
            f"context={context_start}:{anchor_index} "
            f"gt={anchor_index}:{anchor_index + len(gt_chunk) - 1} "
            f"pred={anchor_index}:{anchor_index + horizon - 1}"
        )
        pred_chunk = policy.predict(
            input_vid=input_vid,
            state=state,
            command=command,
            prompt_embedding=prompt_embedding,
            seed=runtime.seed,
            use_cuda_graphs=runtime.use_cuda_graphs,
        )
        pred_chunk_np = pred_chunk.detach().float().cpu().numpy()
        if pred_chunk_np.ndim == 3:
            if pred_chunk_np.shape[0] != 1:
                raise RuntimeError(f"Expected batch size 1, got {pred_chunk_np.shape}")
            pred_chunk_np = pred_chunk_np[0]
        pred_horizon = pred_chunk_np.shape[0]
        if pred_horizon != horizon:
            raise RuntimeError(
                f"Pipeline returned horizon {pred_horizon}, but the test expected {horizon}"
            )
        pred_abs = absolute_chunk(pred_chunk_np, data, anchor_index)
        gt_abs = absolute_chunk(gt_chunk, data, anchor_index)
        comparable_horizon = plot_horizon(
            PredictionWindow(
                window_index=window_index,
                context_start=context_start,
                anchor_index=anchor_index,
                pred_action_chunk=pred_chunk_np,
                gt_action_chunk=gt_chunk,
                pred_absolute=pred_abs,
                gt_absolute=gt_abs,
            ),
            plot_limit_index,
        )
        if comparable_horizon <= 0:
            raise RuntimeError(
                f"Window anchor {anchor_index} has no comparable samples for the selected range"
            )
        metrics = compute_metrics(pred_abs[:comparable_horizon], gt_abs[:comparable_horizon])

        output_image = run_dir / f"{episode_path.stem}_anchor_{anchor_index:06d}_projection.png"
        output_npz = run_dir / f"{episode_path.stem}_anchor_{anchor_index:06d}.npz"
        render_overlay(
            frame=data["workspace_rgb"][anchor_index],
            data=data,
            pred_abs=pred_abs[:comparable_horizon],
            gt_abs=gt_abs[:comparable_horizon],
            anchor_index=anchor_index,
            window_index=window_index,
            plane=plane,
            metrics=metrics,
            output_path=output_image,
        )
        write_window_npz(
            output_npz,
            pred_action_chunk=pred_chunk_np,
            gt_action_chunk=gt_chunk,
            pred_absolute=pred_abs,
            gt_absolute=gt_abs,
            context_start=np.array(context_start, dtype=np.int64),
            anchor_index=np.array(anchor_index, dtype=np.int64),
        )

        predictions.append(
            PredictionWindow(
                window_index=window_index,
                context_start=context_start,
                anchor_index=anchor_index,
                pred_action_chunk=pred_chunk_np,
                gt_action_chunk=gt_chunk,
                pred_absolute=pred_abs,
                gt_absolute=gt_abs,
            )
        )
        rows.append(
            WindowResult(
                window_index=window_index,
                context_start=context_start,
                anchor_index=anchor_index,
                gt_horizon=comparable_horizon,
                output_image=output_image,
                output_npz=output_npz,
                **metrics,
            )
        )

        del input_vid, state, pred_chunk
        torch.cuda.empty_cache()

    pred_release_indices = prediction_release_indices(predictions, data, plot_limit_index)
    first_pred_release = pred_release_indices[0] if pred_release_indices else None
    overlay_frame_index = release_index if stop_at_release and release_index is not None else len(data["workspace_rgb"]) - 1
    trajectory_image = run_dir / f"{episode_path.stem}_trajectory_projection.png"
    trajectory_npz = run_dir / f"{episode_path.stem}_trajectory.npz"
    render_trajectory_overlay(
        frame=data["workspace_rgb"][overlay_frame_index],
        data=data,
        predictions=predictions,
        plane=plane,
        output_path=trajectory_image,
        release_index=release_index,
        first_pred_release=first_pred_release,
        plot_limit_index=plot_limit_index,
    )
    write_trajectory_npz(
        trajectory_npz,
        data=data,
        predictions=predictions,
        release_index=release_index,
        first_pred_release=first_pred_release,
        horizon=horizon,
    )

    return EvaluationResult(
        rows=rows,
        trajectory_image=trajectory_image,
        trajectory_npz=trajectory_npz,
        gt_release_index=release_index,
        first_pred_release_index=first_pred_release,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR)
    parser.add_argument(
        "--episode",
        dest="episodes",
        action="append",
        default=None,
        help=(
            "Episode stem, numeric id, or full .safetensors path. Repeat the option "
            "to select multiple episodes. Default: the predefined 60-episode evaluation set."
        ),
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--checkpoint-dir",
        type=Path,
        default=DEFAULT_CHECKPOINT_DIR,
        help="Base Mimic Video checkpoint dir used by COSMOS_PREDICT2_ARGS.",
    )
    parser.add_argument(
        "--video-model-path",
        type=Path,
        default=None,
        help="Override Cosmos V2W checkpoint. Defaults to mimic_video_config.yaml.",
    )
    parser.add_argument(
        "--action-model-path",
        type=Path,
        default=DEFAULT_ACTION_MODEL,
        help="W2A checkpoint to evaluate. Defaults to iter_000010000.pt.",
    )
    parser.add_argument(
        "--dataset-statistics-path",
        type=Path,
        default=None,
        help="Override UR5e action statistics JSON. Defaults to config/env.",
    )
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--context-frames", type=int, default=CONTEXT_FRAMES, choices=(1, 5))
    parser.add_argument("--horizon", type=int, default=ACTION_HORIZON)
    parser.add_argument("--stride-frames", type=int, default=ACTION_HORIZON)
    parser.add_argument(
        "--max-windows",
        type=int,
        default=None,
        help="Limit the number of windows. Default: all stride windows through the trajectory tail.",
    )
    parser.add_argument(
        "--stop-at-release",
        action="store_true",
        help="Use the first GT closed->open gripper transition as the last sampled anchor.",
    )
    parser.add_argument(
        "--plane",
        choices=tuple(PLOT_AXES),
        default="xy",
        help="Legacy option from the base_link chart overlay; ignored by camera projection.",
    )
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--use-cuda-graphs", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    checkpoint_dir = expand_path(args.checkpoint_dir)
    setup_python_path(checkpoint_dir)

    data_dir = expand_path(args.data_dir)
    output_dir = expand_path(args.output_dir)
    episode_specs = args.episodes if args.episodes is not None else DEFAULT_EPISODES
    episode_paths = [resolve_episode_path(data_dir, episode) for episode in episode_specs]
    runtime = load_runtime_config(args)

    from mimic_video import MimicVideoPolicy

    print("Loading Mimic Video policy once")
    print(f"  video:    {runtime.video_model_path}")
    print(f"  action:   {runtime.action_model_path}")
    print(f"  episodes: {len(episode_paths)}")
    print(f"  output:   {output_dir}")
    policy = MimicVideoPolicy(
        experiment_name=runtime.experiment_name,
        video_model_path=runtime.video_model_path,
        action_model_path=runtime.action_model_path,
        dataset_statistics_path=runtime.dataset_statistics_path,
        language_conditioning="precomputed",
        text_encoder_path=None,
    )

    total_windows = 0
    for episode_index, episode_path in enumerate(episode_paths, start=1):
        data = load_episode(episode_path)
        print("\n" + "=" * 80)
        print(f"Episode {episode_index}/{len(episode_paths)}: {episode_path}")
        print(f"Frames: {len(data['workspace_rgb'])}")
        print(f"Prompt: {decode_language(data)}")
        print("=" * 80)

        result = evaluate_windows(
            policy=policy,
            data=data,
            episode_path=episode_path,
            runtime=runtime,
            output_dir=output_dir,
            plane=args.plane,
            context_frames=args.context_frames,
            stride_frames=args.stride_frames,
            horizon=args.horizon,
            max_windows=args.max_windows,
            start_frame=args.start_frame,
            stop_at_release=args.stop_at_release,
        )

        run_dir = output_dir / runtime.action_model_path.stem / episode_path.stem
        metrics_path = run_dir / "metrics.csv"
        summary_path = run_dir / "summary.json"
        write_metrics_csv(metrics_path, result.rows)
        write_summary(summary_path, runtime, episode_path, result)
        total_windows += len(result.rows)

        print(f"Completed {episode_path.stem}")
        print(f"  windows: {len(result.rows)}")
        print(f"  trajectory image: {result.trajectory_image}")
        print(f"  trajectory npz:   {result.trajectory_npz}")
        print(f"  metrics: {metrics_path}")
        print(f"  summary: {summary_path}")

    print("\nDone.")
    print(f"  episodes: {len(episode_paths)}")
    print(f"  windows:  {total_windows}")
    print(f"  output:   {output_dir / runtime.action_model_path.stem}")


if __name__ == "__main__":
    main()
