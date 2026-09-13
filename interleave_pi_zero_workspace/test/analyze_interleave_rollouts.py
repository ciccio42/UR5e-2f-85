#!/usr/bin/env python3
"""
Analyze and render Interleave-Pi0 real-robot rollouts.

Expected input:
  ai_controller/saved_rollouts/interleave_pi0_controller/pick_place/
    task_XX/traj_YYY.pkl
    task_XX/traj_YYY.json

MODES
-----
inspect
  Inspect one rollout and print EEF/gripper/projection diagnostics.

  Example:
    python analyze_interleave_rollouts.py inspect \
      --savers-dir /PATH/TO/DIR/CONTAINING/savers.py \
      --task 04 --traj 1 --print-gripper-sequence

stats
  Recursively analyze JSON/PKL pairs and create, beside this script:
    rollouts.csv
    summary.csv
    annotations.csv  (template, if missing)

  Example:
    python analyze_interleave_rollouts.py stats \
      --savers-dir /PATH/TO/DIR/CONTAINING/savers.py

render
  For each rollout create:
    render/task_XX/traj_YYY.mp4
    render/task_XX/traj_YYY_trajectory.png

  MP4 = recorded front-camera frame sequence + Interleave instruction panel.
        No EEF trajectory and no gripper markers are drawn on the video.

  PNG = first camera frame + complete measured EEF trajectory + CLOSE/OPEN
        markers derived ONLY from measured gripper_qpos.

  Example, first test on one rollout:
    python analyze_interleave_rollouts.py render \
      --savers-dir /PATH/TO/DIR/CONTAINING/savers.py \
      --config /PATH/TO/interleave_pi0_config.yaml \
      --task 04 --traj 1 --fps 5

  Then all rollouts:
    python analyze_interleave_rollouts.py render \
      --savers-dir /PATH/TO/DIR/CONTAINING/savers.py \
      --config /PATH/TO/interleave_pi0_config.yaml \
      --fps 5

NOTES
-----
- camera_front_image is RGB.
- The saved frames are one observation per controller step, not a continuous
  real-time camera recording. --fps controls playback speed.
- The observation at step t is acquired before action[t] is executed. Therefore,
  if the final action commands OPEN and the episode immediately terminates, the
  physical reopening may not exist in any later saved gripper_qpos. In that case
  no OPEN marker is drawn: the script never invents an unobserved state.
- T_BASE_CAMERA is interpreted as T_base<-camera. Projection from base_link to
  image therefore uses inverse(T_BASE_CAMERA).
"""

from __future__ import annotations

import argparse
import csv
import importlib
import json
import math
import pickle
import re
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

try:
    import yaml
except ImportError as exc:
    raise RuntimeError("PyYAML is required to read the Interleave YAML config.") from exc


# =============================================================================
# PATHS
# =============================================================================

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]

DEFAULT_ROLLOUTS_ROOT = (
    REPO_ROOT
    / "ai_controller"
    / "saved_rollouts"
    / "interleave_pi0_controller"
    / "pick_place"
)
DEFAULT_OUTPUT_DIR = SCRIPT_DIR
DEFAULT_INSTRUCTION_IMAGES_DIR = SCRIPT_DIR / "instruction_images"


# =============================================================================
# CAMERA
# =============================================================================

RAW_IMAGE_WIDTH = 672
RAW_IMAGE_HEIGHT = 376
CAMERA_FX = 363.8398742675781
CAMERA_FY = 363.8398742675781
CAMERA_CX = 337.5829162597656
CAMERA_CY = 178.53465270996094

# T_base<-camera
T_BASE_CAMERA = np.array(
    [
        [-0.9995271385, 0.0017188839, -0.0307008924, 0.0258804482],
        [0.0210472614, 0.7661305314, -0.6423402694, 1.0325620478],
        [0.0224167827, -0.6426827011, -0.7658044356, 0.4595237268],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
T_CAMERA_BASE = np.linalg.inv(T_BASE_CAMERA)


# =============================================================================
# VISUAL STYLE
# =============================================================================

PANEL_HEIGHT = 150
PANEL_MARGIN = 16
INSTRUCTION_IMAGE_SIZE = 72

# RGB colors
COLOR_TRAJECTORY = (255, 215, 0)
COLOR_CLOSE = (255, 80, 40)
COLOR_OPEN = (40, 160, 255)
COLOR_TEXT = (20, 20, 20)
COLOR_MUTED = (90, 90, 90)
COLOR_PANEL = (248, 248, 248)


# =============================================================================
# SAVERS / PKL
# =============================================================================

def ensure_savers_importable(savers_dir: Path | None) -> None:
    """Make savers.Trajectory importable before pickle.load()."""
    if savers_dir is not None:
        savers_dir = savers_dir.expanduser().resolve()
        if not (savers_dir / "savers.py").is_file():
            raise FileNotFoundError(
                f"--savers-dir must be the directory containing savers.py: {savers_dir}"
            )
        if str(savers_dir) not in sys.path:
            sys.path.insert(0, str(savers_dir))

    try:
        importlib.import_module("savers")
        return
    except ModuleNotFoundError:
        pass

    try:
        from ament_index_python.packages import get_package_share_directory
        scripts_dir = Path(get_package_share_directory("dataset_collector_pkg")) / "scripts"
        if scripts_dir.is_dir():
            sys.path.insert(0, str(scripts_dir))
            importlib.import_module("savers")
            return
    except Exception:
        pass

    raise RuntimeError(
        "Cannot import savers.Trajectory. Pass --savers-dir with the directory "
        "that directly contains savers.py, or export it in PYTHONPATH."
    )


def load_rollout(pkl_path: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    with pkl_path.open("rb") as f:
        payload = pickle.load(f)

    if not isinstance(payload, dict) or "traj" not in payload:
        raise ValueError(f"Unexpected rollout structure: {pkl_path}")

    traj = payload["traj"]
    steps = [traj.get(t) for t in range(len(traj))]

    for i, step in enumerate(steps):
        if not isinstance(step, dict) or "obs" not in step or "action" not in step:
            raise ValueError(f"Unexpected Trajectory.get({i}) structure in {pkl_path}")

    return payload, steps


# =============================================================================
# DATA EXTRACTION
# =============================================================================

def decode_rgb_image(value: Any) -> np.ndarray:
    arr = np.asarray(value)

    if arr.ndim == 1:
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError("Could not decode compressed image")
        arr = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    if arr.ndim != 3 or arr.shape[2] != 3:
        raise ValueError(f"Expected HxWx3 image, got {arr.shape}")

    if arr.dtype != np.uint8:
        arr = np.clip(arr, 0, 255).astype(np.uint8)

    return arr


def extract_front_frames(steps: list[dict[str, Any]]) -> list[np.ndarray]:
    frames = []
    for i, step in enumerate(steps):
        obs = step["obs"]
        if "camera_front_image" not in obs:
            raise KeyError(f"Step {i}: camera_front_image missing")
        frames.append(decode_rgb_image(obs["camera_front_image"]))
    return frames


def extract_eef_positions(steps: list[dict[str, Any]]) -> np.ndarray:
    values = []
    for i, step in enumerate(steps):
        v = np.asarray(step["obs"].get("eef_pos"), dtype=np.float64).reshape(-1)
        if v.shape != (3,):
            raise ValueError(f"Step {i}: eef_pos shape={v.shape}, expected (3,)")
        values.append(v)
    return np.stack(values)


def extract_gripper_qpos(steps: list[dict[str, Any]]) -> np.ndarray:
    values = []
    for i, step in enumerate(steps):
        v = np.asarray(step["obs"].get("gripper_qpos"), dtype=np.float64).reshape(-1)
        if v.size == 0:
            raise ValueError(f"Step {i}: empty gripper_qpos")
        values.append(float(v[0]))
    return np.asarray(values, dtype=np.float64)


def extract_actions(steps: list[dict[str, Any]]) -> np.ndarray:
    actions = [np.asarray(step["action"], dtype=np.float64).reshape(-1) for step in steps]
    widths = {a.size for a in actions}
    if len(widths) != 1:
        raise ValueError(f"Inconsistent action dimensions: {sorted(widths)}")
    return np.stack(actions)


def trajectory_length_m(eef_positions: np.ndarray) -> float:
    if len(eef_positions) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(eef_positions, axis=0), axis=1).sum())


# =============================================================================
# GRIPPER EVENTS
# =============================================================================

def _first_index(mask: np.ndarray, start: int = 0) -> int | None:
    idx = np.flatnonzero(mask[start:])
    return None if idx.size == 0 else int(start + idx[0])


def detect_measured_gripper_events(
    qpos: np.ndarray,
    open_threshold: float | None = None,
    close_threshold: float | None = None,
) -> dict[str, Any]:
    """
    Detect physical events only from measured gripper_qpos.

    We first require an observed open state. This intentionally ignores a stale
    high qpos that may be present at rollout start while the initial opening is
    still settling. Then we find the first close and the first subsequent reopen.
    """
    q = np.asarray(qpos, dtype=np.float64)
    finite = q[np.isfinite(q)]

    if finite.size == 0:
        return {
            "qpos_min": math.nan,
            "qpos_max": math.nan,
            "open_threshold": math.nan,
            "close_threshold": math.nan,
            "initial_open_step": None,
            "close_step": None,
            "open_step": None,
        }

    qmin = float(np.min(finite))
    qmax = float(np.max(finite))
    low = float(np.percentile(finite, 10))
    high = float(np.percentile(finite, 90))
    span = high - low

    if open_threshold is None:
        open_threshold = low + 0.30 * span
    if close_threshold is None:
        close_threshold = low + 0.70 * span

    open_threshold = float(open_threshold)
    close_threshold = float(close_threshold)

    if not open_threshold < close_threshold:
        raise ValueError(
            f"open threshold must be < close threshold: "
            f"{open_threshold} >= {close_threshold}"
        )

    initial_open_step = _first_index(q <= open_threshold)
    close_step = None
    open_step = None

    if initial_open_step is not None:
        close_step = _first_index(q >= close_threshold, initial_open_step + 1)
    if close_step is not None:
        open_step = _first_index(q <= open_threshold, close_step + 1)

    return {
        "qpos_min": qmin,
        "qpos_max": qmax,
        "open_threshold": open_threshold,
        "close_threshold": close_threshold,
        "initial_open_step": initial_open_step,
        "close_step": close_step,
        "open_step": open_step,
    }


def detect_command_events(actions: np.ndarray) -> dict[str, int | None]:
    """Diagnostic only. These are never used for PNG OPEN/CLOSE markers."""
    g = actions[:, -1]
    close_step = _first_index(g >= 127.5)
    open_step = None
    if close_step is not None:
        open_step = _first_index(g <= 127.5, close_step + 1)
    return {"close_command_step": close_step, "open_command_step": open_step}


# =============================================================================
# CAMERA PROJECTION
# =============================================================================

def project_base_points_to_pixels(points_base: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    points = np.asarray(points_base, dtype=np.float64)
    homogeneous = np.concatenate(
        [points, np.ones((len(points), 1), dtype=np.float64)], axis=1
    )
    points_camera = (T_CAMERA_BASE @ homogeneous.T).T[:, :3]

    z = points_camera[:, 2]
    valid_depth = z > 1e-6
    pixels = np.full((len(points), 2), np.nan, dtype=np.float64)

    pixels[valid_depth, 0] = (
        CAMERA_FX * points_camera[valid_depth, 0] / z[valid_depth] + CAMERA_CX
    )
    pixels[valid_depth, 1] = (
        CAMERA_FY * points_camera[valid_depth, 1] / z[valid_depth] + CAMERA_CY
    )
    return pixels, valid_depth


def projection_diagnostics(pixels: np.ndarray, valid_depth: np.ndarray) -> dict[str, float]:
    inside = (
        valid_depth
        & np.isfinite(pixels[:, 0])
        & np.isfinite(pixels[:, 1])
        & (pixels[:, 0] >= 0)
        & (pixels[:, 0] < RAW_IMAGE_WIDTH)
        & (pixels[:, 1] >= 0)
        & (pixels[:, 1] < RAW_IMAGE_HEIGHT)
    )
    n = len(pixels)
    return {
        "projection_positive_depth_pct": 100.0 * float(valid_depth.sum()) / n if n else 0.0,
        "projection_inside_image_pct": 100.0 * float(inside.sum()) / n if n else 0.0,
    }


# =============================================================================
# INTERLEAVE CONFIG / INSTRUCTION IMAGES
# =============================================================================

def load_tasks(config_path: Path) -> dict[str, dict[str, Any]]:
    with config_path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    tasks = cfg.get("tasks")
    if not isinstance(tasks, dict):
        raise ValueError(f"No tasks mapping in {config_path}")
    return {str(k).zfill(2): v for k, v in tasks.items()}


def resolve_instruction_images(
    task_cfg: dict[str, Any],
    config_path: Path,
    instruction_images_dir: Path,
) -> list[Path]:
    raw = task_cfg.get("instruction_images")
    if raw is None:
        raw = task_cfg.get("instruction_image")
    if raw is None:
        return []
    if isinstance(raw, str):
        raw = [raw]

    resolved = []
    for item in raw:
        rel = Path(str(item))
        candidates = [
            instruction_images_dir / rel.name,
            SCRIPT_DIR / rel,
            config_path.parent / rel,
            rel.expanduser(),
        ]
        found = next((p.resolve() for p in candidates if p.is_file()), None)
        if found is None:
            raise FileNotFoundError(
                f"Instruction image {item!r} not found. Tried: "
                + ", ".join(str(p) for p in candidates)
            )
        resolved.append(found)
    return resolved


# =============================================================================
# TEXT / PANEL DRAWING
# =============================================================================

def load_font(size: int, bold: bool = False):
    candidates = [
        Path("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"),
        Path("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/dejavu/DejaVuSans.ttf"),
    ]
    for path in candidates:
        if path.is_file():
            return ImageFont.truetype(str(path), size=size)
    return ImageFont.load_default()


def text_size(draw, text, font):
    b = draw.textbbox((0, 0), text, font=font)
    return b[2] - b[0], b[3] - b[1]


def build_instruction_panel(
    width: int,
    prompt: str,
    instruction_images: list[Image.Image],
    task_id: str,
    traj_number: int,
) -> Image.Image:
    panel = Image.new("RGB", (width, PANEL_HEIGHT), COLOR_PANEL)
    draw = ImageDraw.Draw(panel)
    title_font = load_font(17, bold=True)
    prompt_font = load_font(21)
    meta_font = load_font(14)

    draw.text((PANEL_MARGIN, 8), "Interleaved instruction", fill=COLOR_TEXT, font=title_font)
    meta = f"Task {task_id}  |  Trajectory {traj_number:03d}"
    mw, _ = text_size(draw, meta, meta_font)
    draw.text((width - PANEL_MARGIN - mw, 10), meta, fill=COLOR_MUTED, font=meta_font)

    segments = prompt.split("<image>")
    expected = len(segments) - 1
    if expected != len(instruction_images):
        raise ValueError(
            f"Prompt has {expected} <image> placeholders but {len(instruction_images)} images"
        )

    tokens: list[tuple[str, Any]] = []
    for i, segment in enumerate(segments):
        for word in segment.split():
            tokens.append(("text", word + " "))
        if i < len(instruction_images):
            tokens.append(("image", instruction_images[i]))

    x, y = PANEL_MARGIN, 43
    line_h = INSTRUCTION_IMAGE_SIZE + 4

    for kind, value in tokens:
        if kind == "text":
            tw, th = text_size(draw, value, prompt_font)
            if x + tw > width - PANEL_MARGIN:
                x = PANEL_MARGIN
                y += line_h
                line_h = 30
            draw.text((x, y + max(0, (line_h - th) // 2)), value, fill=COLOR_TEXT, font=prompt_font)
            x += tw
        else:
            img = value.copy().convert("RGB")
            img.thumbnail((INSTRUCTION_IMAGE_SIZE, INSTRUCTION_IMAGE_SIZE), Image.Resampling.LANCZOS)
            canvas = Image.new("RGB", (INSTRUCTION_IMAGE_SIZE, INSTRUCTION_IMAGE_SIZE), (255, 255, 255))
            ox = (INSTRUCTION_IMAGE_SIZE - img.width) // 2
            oy = (INSTRUCTION_IMAGE_SIZE - img.height) // 2
            canvas.paste(img, (ox, oy))
            if x + INSTRUCTION_IMAGE_SIZE > width - PANEL_MARGIN:
                x = PANEL_MARGIN
                y += line_h
            panel.paste(canvas, (x, y))
            draw.rectangle(
                [x, y, x + INSTRUCTION_IMAGE_SIZE - 1, y + INSTRUCTION_IMAGE_SIZE - 1],
                outline=(130, 130, 130), width=1,
            )
            x += INSTRUCTION_IMAGE_SIZE + 8

    return panel


# =============================================================================
# RENDERERS
# =============================================================================

def render_video(
    output_path: Path,
    frames_rgb: list[np.ndarray],
    prompt: str,
    instruction_paths: list[Path],
    task_id: str,
    traj_number: int,
    fps: float,
) -> None:
    first = frames_rgb[0]
    h, w = first.shape[:2]
    instruction_images = [Image.open(p).convert("RGB") for p in instruction_paths]
    panel = np.asarray(
        build_instruction_panel(w, prompt, instruction_images, task_id, traj_number),
        dtype=np.uint8,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        float(fps),
        (w, h + PANEL_HEIGHT),
    )
    if not writer.isOpened():
        raise RuntimeError(f"Could not create video: {output_path}")

    try:
        for frame in frames_rgb:
            if frame.shape[:2] != (h, w):
                frame = cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA)
            composed = np.vstack([panel, frame])
            writer.write(cv2.cvtColor(composed, cv2.COLOR_RGB2BGR))
    finally:
        writer.release()


def draw_marker(draw, xy, label, color):
    x, y = int(round(xy[0])), int(round(xy[1]))
    r = 7
    draw.ellipse([x-r, y-r, x+r, y+r], fill=color, outline=(255,255,255), width=2)
    font = load_font(15, bold=True)
    tx, ty = x + 10, max(2, y - 10)
    b = draw.textbbox((tx, ty), label, font=font)
    draw.rounded_rectangle([b[0]-3, b[1]-3, b[2]+3, b[3]+3], radius=3, fill=(255,255,255))
    draw.text((tx, ty), label, fill=color, font=font)


def render_trajectory_png(
    output_path: Path,
    first_frame_rgb: np.ndarray,
    eef_positions: np.ndarray,
    close_step: int | None,
    open_step: int | None,
) -> dict[str, float]:
    pixels, valid_depth = project_base_points_to_pixels(eef_positions)
    diag = projection_diagnostics(pixels, valid_depth)

    image = Image.fromarray(first_frame_rgb).convert("RGB")
    draw = ImageDraw.Draw(image)

    for i in range(1, len(pixels)):
        p0, p1 = pixels[i-1], pixels[i]
        if not (valid_depth[i-1] and valid_depth[i]):
            continue
        if not (np.all(np.isfinite(p0)) and np.all(np.isfinite(p1))):
            continue
        draw.line([(float(p0[0]), float(p0[1])), (float(p1[0]), float(p1[1]))], fill=COLOR_TRAJECTORY, width=4)

    if close_step is not None and 0 <= close_step < len(pixels):
        p = pixels[close_step]
        if valid_depth[close_step] and np.all(np.isfinite(p)):
            draw_marker(draw, p, "CLOSE", COLOR_CLOSE)

    if open_step is not None and 0 <= open_step < len(pixels):
        p = pixels[open_step]
        if valid_depth[open_step] and np.all(np.isfinite(p)):
            draw_marker(draw, p, "OPEN", COLOR_OPEN)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)
    return diag


# =============================================================================
# DISCOVERY
# =============================================================================

TASK_RE = re.compile(r"task_(\d+)$")
TRAJ_RE = re.compile(r"traj_(\d+)$")


def discover_pkls(root: Path, task: str | None = None, traj: int | None = None) -> list[Path]:
    out = []
    for path in sorted(root.glob("task_*/traj_*.pkl")):
        mt = TASK_RE.match(path.parent.name)
        mr = TRAJ_RE.match(path.stem)
        if not mt or not mr:
            continue
        task_id = mt.group(1).zfill(2)
        traj_number = int(mr.group(1))
        if task is not None and task_id != str(task).zfill(2):
            continue
        if traj is not None and traj_number != int(traj):
            continue
        out.append(path)
    return out


def discover_jsons(root: Path) -> list[Path]:
    return sorted(root.glob("task_*/traj_*.json"))


# =============================================================================
# STATS
# =============================================================================

def classify_outcome(data: dict[str, Any]) -> str:
    try:
        triple = (
            int(data["object_reached"]),
            int(data["object_picked"]),
            int(data["object_placed"]),
        )
    except Exception:
        return "Inconsistent"

    if triple == (1, 1, 1):
        return "Success"
    if triple == (0, 0, 0):
        return "Reach failure"
    if triple == (1, 0, 0):
        return "Pick failure"
    if triple == (1, 1, 0):
        return "Place failure"
    return "Inconsistent"


def read_annotations(path: Path) -> dict[tuple[str, int], dict[str, str]]:
    if not path.exists():
        with path.open("w", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow(["task_id", "traj_number", "note", "error_type", "bin_confusion"])
        print(f"[stats] Created annotation template: {path}")
        return {}

    result = {}
    with path.open("r", newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if not row.get("task_id") or not row.get("traj_number"):
                continue
            key = (str(row["task_id"]).zfill(2), int(row["traj_number"]))
            result[key] = {k: v for k, v in row.items() if k not in {"task_id", "traj_number"}}
    return result


def analyze_pkl(path: Path, open_thr, close_thr) -> dict[str, Any]:
    payload, steps = load_rollout(path)
    eef = extract_eef_positions(steps)
    qpos = extract_gripper_qpos(steps)
    actions = extract_actions(steps)
    measured = detect_measured_gripper_events(qpos, open_thr, close_thr)
    commanded = detect_command_events(actions)
    pixels, valid = project_base_points_to_pixels(eef)
    proj = projection_diagnostics(pixels, valid)

    return {
        "pkl_path": str(path),
        "task_id_pkl": str(payload.get("task_id", "")).zfill(2),
        "traj_number_pkl": payload.get("traj_number", ""),
        "num_steps": len(steps),
        "eef_path_length_m": trajectory_length_m(eef),
        "gripper_qpos_min": measured["qpos_min"],
        "gripper_qpos_max": measured["qpos_max"],
        "gripper_open_threshold": measured["open_threshold"],
        "gripper_close_threshold": measured["close_threshold"],
        "gripper_initial_open_step_observed": measured["initial_open_step"],
        "gripper_close_step_observed": measured["close_step"],
        "gripper_open_step_observed": measured["open_step"],
        "gripper_close_command_step": commanded["close_command_step"],
        "gripper_open_command_step": commanded["open_command_step"],
        **proj,
    }


def safe_pct(num, den):
    return math.nan if not den else 100.0 * float(num) / float(den)


def summarize(rows: list[dict[str, Any]], scope: str) -> dict[str, Any]:
    cats = ["Success", "Reach failure", "Pick failure", "Place failure", "Inconsistent"]
    c = {cat: sum(r["category"] == cat for r in rows) for cat in cats}
    failures = c["Reach failure"] + c["Pick failure"] + c["Place failure"]
    reached = sum(int(r.get("object_reached", 0) or 0) == 1 for r in rows)
    picked = sum(int(r.get("object_picked", 0) or 0) == 1 for r in rows)
    placed = sum(int(r.get("object_placed", 0) or 0) == 1 for r in rows)
    n = len(rows)

    return {
        "scope": scope,
        "n_rollouts": n,
        "success": c["Success"],
        "reach_failure": c["Reach failure"],
        "pick_failure": c["Pick failure"],
        "place_failure": c["Place failure"],
        "inconsistent": c["Inconsistent"],
        "total_failures": failures,
        "success_rate_pct": safe_pct(c["Success"], n),
        "failure_rate_pct": safe_pct(failures, n),
        "place_failure_share_of_failures_pct": safe_pct(c["Place failure"], failures),
        "reach_rate_pct": safe_pct(reached, n),
        "pick_given_reach_pct": safe_pct(picked, reached),
        "place_given_pick_pct": safe_pct(placed, picked),
        "aborted_count": sum(int(r.get("aborted", 0) or 0) == 1 for r in rows),
        "trajectory_complete_count": sum(int(r.get("trajectory_complete", 0) or 0) == 1 for r in rows),
        "reached_wrong_sum": sum(int(r.get("reached_wrong", 0) or 0) for r in rows),
        "picked_wrong_sum": sum(int(r.get("picked_wrong", 0) or 0) for r in rows),
        "place_wrong_correct_bin_sum": sum(int(r.get("place_wrong_correct_bin", 0) or 0) for r in rows),
        "place_wrong_wrong_bin_sum": sum(int(r.get("place_wrong_wrong_bin", 0) or 0) for r in rows),
    }


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields, seen = [], set()
    for row in rows:
        for k in row:
            if k not in seen:
                fields.append(k)
                seen.add(k)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)


def run_stats(args):
    ensure_savers_importable(args.savers_dir)
    root = args.rollouts_root.resolve()
    outdir = args.output_dir.resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    annotations = read_annotations(outdir / "annotations.csv")

    rows = []
    for jp in discover_jsons(root):
        mt = TASK_RE.match(jp.parent.name)
        mr = TRAJ_RE.match(jp.stem)
        if not mt or not mr:
            continue
        task_id = mt.group(1).zfill(2)
        traj_number = int(mr.group(1))
        with jp.open("r", encoding="utf-8") as f:
            outcome = json.load(f)

        row = {"task_id": task_id, "traj_number": traj_number, "json_path": str(jp), **outcome}
        row["category"] = classify_outcome(outcome)

        pp = jp.with_suffix(".pkl")
        row["has_pkl"] = int(pp.is_file())
        if pp.is_file():
            try:
                row.update(analyze_pkl(pp, args.gripper_open_threshold, args.gripper_close_threshold))
            except Exception as exc:
                row["pkl_analysis_error"] = repr(exc)

        if (task_id, traj_number) in annotations:
            row.update(annotations[(task_id, traj_number)])
        rows.append(row)

    rows.sort(key=lambda r: (r["task_id"], int(r["traj_number"])))
    write_csv(outdir / "rollouts.csv", rows)

    summary_rows = [summarize(rows, "global")]
    for task_id in sorted({r["task_id"] for r in rows}):
        summary_rows.append(summarize([r for r in rows if r["task_id"] == task_id], f"task_{task_id}"))
    write_csv(outdir / "summary.csv", summary_rows)

    print(f"[stats] rollouts: {len(rows)}")
    print(f"[stats] wrote: {outdir / 'rollouts.csv'}")
    print(f"[stats] wrote: {outdir / 'summary.csv'}")
    print("\nGLOBAL SUMMARY")
    g = summary_rows[0]
    for k in [
        "n_rollouts", "success", "reach_failure", "pick_failure", "place_failure",
        "inconsistent", "total_failures", "success_rate_pct",
        "place_failure_share_of_failures_pct", "pick_given_reach_pct",
        "place_given_pick_pct", "aborted_count",
    ]:
        v = g[k]
        print(f"  {k}: {v:.2f}" if isinstance(v, float) else f"  {k}: {v}")


# =============================================================================
# INSPECT
# =============================================================================

def run_inspect(args):
    ensure_savers_importable(args.savers_dir)

    if args.pkl:
        path = args.pkl.resolve()
    else:
        matches = discover_pkls(args.rollouts_root.resolve(), args.task, args.traj)
        if not matches:
            raise FileNotFoundError("No matching rollout")
        path = matches[0]

    payload, steps = load_rollout(path)
    frames = extract_front_frames(steps)
    eef = extract_eef_positions(steps)
    qpos = extract_gripper_qpos(steps)
    actions = extract_actions(steps)
    measured = detect_measured_gripper_events(qpos, args.gripper_open_threshold, args.gripper_close_threshold)
    commanded = detect_command_events(actions)
    pixels, valid = project_base_points_to_pixels(eef)
    proj = projection_diagnostics(pixels, valid)

    print("=" * 80)
    print("PKL:", path)
    print("task_id:", str(payload.get("task_id", "")).zfill(2))
    print("traj_number:", payload.get("traj_number"))
    print("steps:", len(steps))
    print("frame shape:", frames[0].shape)
    print("action shape:", actions.shape)
    print("EEF path length [m]:", f"{trajectory_length_m(eef):.4f}")
    print("\nMEASURED GRIPPER")
    for k in ["qpos_min", "qpos_max", "open_threshold", "close_threshold", "initial_open_step", "close_step", "open_step"]:
        print(f"  {k}: {measured[k]}")
    print("\nCOMMAND DIAGNOSTICS (not used for PNG markers)")
    print("  close_command_step:", commanded["close_command_step"])
    print("  open_command_step:", commanded["open_command_step"])
    print("\nPROJECTION")
    print("  positive depth [%]:", f"{proj['projection_positive_depth_pct']:.2f}")
    print("  inside image [%]:", f"{proj['projection_inside_image_pct']:.2f}")

    if args.print_gripper_sequence:
        print("\nSTEP | measured_qpos | action_gripper")
        print("-----+---------------+---------------")
        for i, q in enumerate(qpos):
            print(f"{i:4d} | {q:13.6f} | {actions[i, -1]:13.3f}")


# =============================================================================
# RENDER
# =============================================================================

def run_render(args):
    ensure_savers_importable(args.savers_dir)
    if args.config is None:
        raise ValueError("render requires --config /path/to/interleave_pi0_config.yaml")

    config = args.config.resolve()
    tasks = load_tasks(config)
    pkls = discover_pkls(args.rollouts_root.resolve(), args.task, args.traj)
    if args.limit is not None:
        pkls = pkls[:args.limit]
    if not pkls:
        raise FileNotFoundError("No rollout PKLs found")

    render_root = args.output_dir.resolve() / "render"
    print(f"[render] selected {len(pkls)} rollout(s)")

    for n, pp in enumerate(pkls, 1):
        payload, steps = load_rollout(pp)
        task_id = str(payload.get("task_id", "")).zfill(2)
        traj_number = int(payload.get("traj_number"))
        task_cfg = tasks[task_id]
        prompt = str(task_cfg["prompt"])
        instruction_paths = resolve_instruction_images(
            task_cfg, config, args.instruction_images_dir.resolve()
        )

        frames = extract_front_frames(steps)
        eef = extract_eef_positions(steps)
        qpos = extract_gripper_qpos(steps)
        measured = detect_measured_gripper_events(qpos, args.gripper_open_threshold, args.gripper_close_threshold)

        task_dir = render_root / f"task_{task_id}"
        video_path = task_dir / f"traj_{traj_number:03d}.mp4"
        png_path = task_dir / f"traj_{traj_number:03d}_trajectory.png"

        render_video(video_path, frames, prompt, instruction_paths, task_id, traj_number, args.fps)
        diag = render_trajectory_png(
            png_path, frames[0], eef, measured["close_step"], measured["open_step"]
        )

        print(
            f"[render {n:02d}/{len(pkls):02d}] task={task_id} traj={traj_number:03d} "
            f"steps={len(steps)} close={measured['close_step']} open={measured['open_step']} "
            f"inside={diag['projection_inside_image_pct']:.1f}%"
        )

        if measured["close_step"] is not None and measured["open_step"] is None:
            print("  NOTE: physical CLOSE is observed, but no later measured OPEN exists; no OPEN marker drawn.")
        if diag["projection_inside_image_pct"] < 50.0:
            print("  WARNING: <50% of projected EEF points are inside the image; verify T_BASE_CAMERA convention/calibration.")

    print("[render] output:", render_root)


# =============================================================================
# CLI
# =============================================================================

def build_parser():
    p = argparse.ArgumentParser(description="Analyze/render Interleave-Pi0 real-robot rollouts")
    p.add_argument("mode", choices=["inspect", "stats", "render"])
    p.add_argument("--rollouts-root", type=Path, default=DEFAULT_ROLLOUTS_ROOT)
    p.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    p.add_argument("--savers-dir", type=Path, default="/home/asus-mivia/Desktop/Multi-Task-LFD/repo/VLA-Bench/robosuite_test/tasks/training/multi_task_il/datasets", help="Directory directly containing savers.py")
    p.add_argument("--config", type=Path, default=None, help="Interleave inference YAML with tasks")
    p.add_argument("--instruction-images-dir", type=Path, default=DEFAULT_INSTRUCTION_IMAGES_DIR)
    p.add_argument("--task", type=str, default=None)
    p.add_argument("--traj", type=int, default=None)
    p.add_argument("--pkl", type=Path, default=None)
    p.add_argument("--fps", type=float, default=5.0)
    p.add_argument("--limit", type=int, default=None)
    p.add_argument("--gripper-open-threshold", type=float, default=None)
    p.add_argument("--gripper-close-threshold", type=float, default=None)
    p.add_argument("--print-gripper-sequence", action="store_true")
    return p


def main():
    args = build_parser().parse_args()
    if args.fps <= 0:
        raise ValueError("--fps must be > 0")

    if args.mode == "inspect":
        run_inspect(args)
    elif args.mode == "stats":
        run_stats(args)
    else:
        run_render(args)


if __name__ == "__main__":
    main()
