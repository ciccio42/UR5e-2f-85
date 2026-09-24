#!/usr/bin/env python3
"""
Analyze and render Interleave-Pi0 real-robot rollouts.

Expected rollout layout
-----------------------
ai_controller/saved_rollouts/interleave_pi0_controller/pick_place/
    task_XX/
        traj_YYY.pkl
        traj_YYY.json

Default outputs
---------------
interleave_pi_zero_workspace/test/
    annotations.csv
    rollouts.csv
    summary.csv
    render/
        task_XX/
            traj_YYY.mp4
            traj_YYY_trajectory.png

Modes
-----
inspect:
  python analyze_interleave_rollouts.py inspect \
    --savers-dir /PATH/TO/DIRECTORY/CONTAINING/savers.py \
    --task 04 --traj 1 --print-gripper-sequence

stats:
  python analyze_interleave_rollouts.py stats \
    --savers-dir /PATH/TO/DIRECTORY/CONTAINING/savers.py

render one rollout:
  python analyze_interleave_rollouts.py render \
    --savers-dir /PATH/TO/DIRECTORY/CONTAINING/savers.py \
    --config /PATH/TO/interleave_pi0.yaml \
    --task 04 --traj 1 --fps 5

render all:
  python analyze_interleave_rollouts.py render \
    --savers-dir /PATH/TO/DIRECTORY/CONTAINING/savers.py \
    --config /PATH/TO/interleave_pi0.yaml \
    --fps 5

Video format
------------
Videos are written as simple MP4 files with OpenCV's mp4v codec.
No external ffmpeg executable is required.

Image convention
----------------
camera_front_image stored in the rollout PKLs is BGR.
This script converts BGR -> RGB once when loading frames.
For VideoWriter, the final composed RGB frame is converted back to BGR.

Trajectory PNG
--------------
The PNG uses the SECOND front-camera frame as background. The plotted EEF path
also starts from the second saved state. CLOSE is derived from measured
gripper_qpos. RELEASE/OPEN is derived from measured gripper_qpos when available;
if the rollout is truncated immediately after the final open command, the release
is still marked at the EEF pose of that command and explicitly labeled OPEN (CMD).

The generated video likewise starts from the SECOND saved camera frame.
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
import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]

DEFAULT_ROLLOUTS_ROOT = (
    REPO_ROOT
    / "ai_controller"
    / "saved_rollouts"
    / "interleave_pi0_controller"
    / "pick_place"
    / "grounding_bin"
)

DEFAULT_OUTPUT_DIR = SCRIPT_DIR
DEFAULT_INSTRUCTION_IMAGES_DIR = SCRIPT_DIR / "instruction_images"

RAW_IMAGE_WIDTH = 672
RAW_IMAGE_HEIGHT = 376

CAMERA_FX = 363.8398742675781
CAMERA_FY = 363.8398742675781
CAMERA_CX = 337.5829162597656
CAMERA_CY = 178.53465270996094

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

PANEL_HEIGHT = 150
PANEL_MARGIN = 16
INSTRUCTION_IMAGE_SIZE = 72

COLOR_PANEL = (248, 248, 248)
COLOR_TEXT = (20, 20, 20)
COLOR_MUTED = (90, 90, 90)
COLOR_TRAJECTORY = (255, 215, 0)
COLOR_CLOSE = (220, 50, 40)
COLOR_OPEN = (40, 120, 230)


def ensure_savers_importable(savers_dir: Path | None) -> None:
    if savers_dir is not None:
        directory = savers_dir.expanduser().resolve()
        if not (directory / "savers.py").is_file():
            raise FileNotFoundError(
                f"--savers-dir must directly contain savers.py: {directory}"
            )
        if str(directory) not in sys.path:
            sys.path.insert(0, str(directory))

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
        "Cannot import savers.Trajectory. Pass --savers-dir pointing to "
        "the directory that contains savers.py."
    )


def load_rollout(pkl_path: Path):
    with pkl_path.open("rb") as f:
        payload = pickle.load(f)

    if not isinstance(payload, dict) or "traj" not in payload:
        raise ValueError(f"Unexpected rollout structure in {pkl_path}")

    traj = payload["traj"]
    steps = [traj.get(t) for t in range(len(traj))]
    return payload, traj, steps


def decode_front_image_to_rgb(value: Any) -> np.ndarray:
    array = np.asarray(value)

    if array.ndim == 1:
        array = cv2.imdecode(array, cv2.IMREAD_COLOR)
        if array is None:
            raise ValueError("cv2.imdecode failed")

    if array.ndim != 3 or array.shape[2] != 3:
        raise ValueError(f"Expected HxWx3 image, got {array.shape}")

    if array.dtype != np.uint8:
        array = np.clip(array, 0, 255).astype(np.uint8)

    # PKL stores BGR -> convert exactly once to RGB.
    return cv2.cvtColor(array, cv2.COLOR_BGR2RGB)


def extract_front_frames(steps):
    return [
        decode_front_image_to_rgb(step["obs"]["camera_front_image"])
        for step in steps
    ]


def extract_eef_positions(steps):
    out = []
    for i, step in enumerate(steps):
        value = np.asarray(step["obs"]["eef_pos"], dtype=np.float64).reshape(-1)
        if value.shape != (3,):
            raise ValueError(f"Step {i}: invalid eef_pos shape {value.shape}")
        out.append(value)
    return np.stack(out, axis=0)


def extract_gripper_qpos(steps):
    out = []
    for i, step in enumerate(steps):
        value = np.asarray(step["obs"]["gripper_qpos"], dtype=np.float64).reshape(-1)
        if value.size < 1:
            raise ValueError(f"Step {i}: empty gripper_qpos")
        out.append(float(value[0]))
    return np.asarray(out)


def extract_actions(steps):
    actions = [
        np.asarray(step["action"], dtype=np.float64).reshape(-1)
        for step in steps
    ]
    return np.stack(actions, axis=0)


def first_true(mask: np.ndarray, start: int = 0):
    idx = np.flatnonzero(mask[start:])
    return None if idx.size == 0 else int(start + idx[0])


def detect_measured_gripper_events(
    qpos: np.ndarray,
    open_threshold: float | None = None,
    close_threshold: float | None = None,
):
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

    low = float(np.percentile(finite, 10))
    high = float(np.percentile(finite, 90))
    span = high - low

    if open_threshold is None:
        open_threshold = low + 0.30 * span
    if close_threshold is None:
        close_threshold = low + 0.70 * span

    if open_threshold >= close_threshold:
        raise ValueError("open_threshold must be lower than close_threshold")

    initial_open = first_true(q <= open_threshold)
    close_step = None
    open_step = None

    if initial_open is not None:
        close_step = first_true(q >= close_threshold, initial_open + 1)

    if close_step is not None:
        open_step = first_true(q <= open_threshold, close_step + 1)

    return {
        "qpos_min": float(np.min(finite)),
        "qpos_max": float(np.max(finite)),
        "open_threshold": float(open_threshold),
        "close_threshold": float(close_threshold),
        "initial_open_step": initial_open,
        "close_step": close_step,
        "open_step": open_step,
    }


def detect_commanded_gripper_events(actions):
    g = actions[:, -1]
    close_step = first_true(g >= 127.5)
    open_step = None if close_step is None else first_true(g <= 127.5, close_step + 1)
    return {
        "close_command_step": close_step,
        "open_command_step": open_step,
    }


def project_base_points_to_pixels(points_base):
    points = np.asarray(points_base, dtype=np.float64)
    hom = np.concatenate(
        [points, np.ones((len(points), 1), dtype=np.float64)],
        axis=1,
    )
    cam = (T_CAMERA_BASE @ hom.T).T[:, :3]
    z = cam[:, 2]
    valid = z > 1e-6

    pixels = np.full((len(points), 2), np.nan, dtype=np.float64)
    pixels[valid, 0] = CAMERA_FX * cam[valid, 0] / z[valid] + CAMERA_CX
    pixels[valid, 1] = CAMERA_FY * cam[valid, 1] / z[valid] + CAMERA_CY

    return pixels, valid


def projection_diagnostics(pixels, valid_depth):
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
        "projection_positive_depth_pct": 100.0 * np.sum(valid_depth) / n if n else 0.0,
        "projection_inside_image_pct": 100.0 * np.sum(inside) / n if n else 0.0,
    }


def load_tasks_from_config(config_path: Path):
    with config_path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    tasks = cfg.get("tasks")
    if not isinstance(tasks, dict):
        raise ValueError(f"No tasks mapping in {config_path}")

    return {str(k).zfill(2): v for k, v in tasks.items()}


def resolve_instruction_images(task_cfg, config_path, instruction_images_dir):
    raw = task_cfg.get("instruction_images")

    if raw is None:
        raw = task_cfg.get("instruction_image")
        raw = [] if raw is None else [raw]
    elif isinstance(raw, str):
        raw = [raw]

    resolved = []
    for item in raw:
        item_path = Path(str(item))
        candidates = [
            instruction_images_dir / item_path.name,
            SCRIPT_DIR / item_path,
            config_path.parent / item_path,
            item_path.expanduser(),
        ]
        found = next((p.resolve() for p in candidates if p.is_file()), None)
        if found is None:
            raise FileNotFoundError(f"Instruction image not found: {item}")
        resolved.append(found)

    return resolved


def get_font(size: int, bold: bool = False):
    candidates = [
        Path("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf")
        if bold else Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"),
        Path("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf")
        if bold else Path("/usr/share/fonts/dejavu/DejaVuSans.ttf"),
    ]

    for p in candidates:
        if p.is_file():
            return ImageFont.truetype(str(p), size=size)

    return ImageFont.load_default()


def text_size(draw, text, font):
    box = draw.textbbox((0, 0), text, font=font)
    return box[2] - box[0], box[3] - box[1]


def build_instruction_panel(width, prompt, instruction_images, task_id, traj_number):
    panel = Image.new("RGB", (width, PANEL_HEIGHT), COLOR_PANEL)
    draw = ImageDraw.Draw(panel)

    title_font = get_font(17, True)
    prompt_font = get_font(21, False)
    meta_font = get_font(14, False)

    draw.text(
        (PANEL_MARGIN, 8),
        "Interleaved instruction",
        fill=COLOR_TEXT,
        font=title_font,
    )

    meta = f"Task {task_id} | Trajectory {traj_number:03d}"
    meta_w, _ = text_size(draw, meta, meta_font)
    draw.text(
        (width - PANEL_MARGIN - meta_w, 10),
        meta,
        fill=COLOR_MUTED,
        font=meta_font,
    )

    segments = prompt.split("<image>")
    if len(segments) - 1 != len(instruction_images):
        raise ValueError(
            f"Prompt expects {len(segments)-1} instruction image(s), "
            f"got {len(instruction_images)}"
        )

    tokens = []
    for i, segment in enumerate(segments):
        for word in segment.split():
            tokens.append(("text", word + " "))
        if i < len(instruction_images):
            tokens.append(("image", instruction_images[i]))

    x = PANEL_MARGIN
    y = 43
    line_height = 30

    for kind, value in tokens:
        if kind == "text":
            tw, th = text_size(draw, value, prompt_font)
            if x + tw > width - PANEL_MARGIN:
                x = PANEL_MARGIN
                y += line_height
                line_height = 30

            draw.text(
                (x, y + max(0, (line_height - th) // 2)),
                value,
                fill=COLOR_TEXT,
                font=prompt_font,
            )
            x += tw
            line_height = max(line_height, th + 4)

        else:
            image = value.copy().convert("RGB")
            image.thumbnail(
                (INSTRUCTION_IMAGE_SIZE, INSTRUCTION_IMAGE_SIZE),
                Image.Resampling.LANCZOS,
            )

            canvas = Image.new(
                "RGB",
                (INSTRUCTION_IMAGE_SIZE, INSTRUCTION_IMAGE_SIZE),
                (255, 255, 255),
            )

            ox = (INSTRUCTION_IMAGE_SIZE - image.width) // 2
            oy = (INSTRUCTION_IMAGE_SIZE - image.height) // 2
            canvas.paste(image, (ox, oy))

            if x + INSTRUCTION_IMAGE_SIZE > width - PANEL_MARGIN:
                x = PANEL_MARGIN
                y += line_height
                line_height = INSTRUCTION_IMAGE_SIZE + 4

            panel.paste(canvas, (x, y))
            draw.rectangle(
                [x, y, x + INSTRUCTION_IMAGE_SIZE - 1, y + INSTRUCTION_IMAGE_SIZE - 1],
                outline=(130, 130, 130),
                width=1,
            )

            x += INSTRUCTION_IMAGE_SIZE + 8
            line_height = max(line_height, INSTRUCTION_IMAGE_SIZE + 4)

    return panel


def render_video(
    output_path: Path,
    frames_rgb,
    prompt,
    instruction_image_paths,
    task_id,
    traj_number,
    fps,
):
    """
    Simple MP4 using OpenCV + mp4v.
    """
    if not frames_rgb:
        raise ValueError("No frames to render")

    first = frames_rgb[0]
    h, w = first.shape[:2]

    instruction_images = [
        Image.open(path).convert("RGB")
        for path in instruction_image_paths
    ]

    panel = build_instruction_panel(
        w,
        prompt,
        instruction_images,
        task_id,
        traj_number,
    )
    panel_rgb = np.asarray(panel, dtype=np.uint8)

    out_h = PANEL_HEIGHT + h
    output_path.parent.mkdir(parents=True, exist_ok=True)

    writer = cv2.VideoWriter(
        str(output_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        float(fps),
        (w, out_h),
    )

    if not writer.isOpened():
        raise RuntimeError(f"Could not open mp4v VideoWriter: {output_path}")

    try:
        for frame_rgb in frames_rgb:
            if frame_rgb.shape[:2] != (h, w):
                frame_rgb = cv2.resize(
                    frame_rgb,
                    (w, h),
                    interpolation=cv2.INTER_AREA,
                )

            composed_rgb = np.vstack([panel_rgb, frame_rgb])

            # OpenCV writer receives BGR.
            composed_bgr = cv2.cvtColor(
                composed_rgb,
                cv2.COLOR_RGB2BGR,
            )

            writer.write(composed_bgr)

    finally:
        writer.release()

    if not output_path.exists() or output_path.stat().st_size == 0:
        raise RuntimeError(f"Empty/invalid output video: {output_path}")


def draw_marker(draw, xy, label, color):
    x, y = int(round(xy[0])), int(round(xy[1]))
    r = 7

    draw.ellipse(
        [x-r, y-r, x+r, y+r],
        fill=color,
        outline=(255, 255, 255),
        width=2,
    )

    font = get_font(15, True)
    tx, ty = x + 10, max(2, y - 10)
    box = draw.textbbox((tx, ty), label, font=font)
    pad = 3

    draw.rounded_rectangle(
        [box[0]-pad, box[1]-pad, box[2]+pad, box[3]+pad],
        radius=3,
        fill=(255, 255, 255),
    )

    draw.text((tx, ty), label, fill=color, font=font)


def render_trajectory_png(
    output_path,
    background_frame_rgb,
    eef_positions,
    close_step,
    open_step,
    open_is_commanded=False,
):
    pixels, valid_depth = project_base_points_to_pixels(eef_positions)
    diagnostics = projection_diagnostics(pixels, valid_depth)

    image = Image.fromarray(background_frame_rgb).convert("RGB")
    draw = ImageDraw.Draw(image)

    for i in range(1, len(pixels)):
        if not (valid_depth[i-1] and valid_depth[i]):
            continue

        p0 = pixels[i-1]
        p1 = pixels[i]

        if not (np.all(np.isfinite(p0)) and np.all(np.isfinite(p1))):
            continue

        draw.line(
            [(float(p0[0]), float(p0[1])), (float(p1[0]), float(p1[1]))],
            fill=COLOR_TRAJECTORY,
            width=4,
        )

    if close_step is not None and 0 <= close_step < len(pixels):
        p = pixels[close_step]
        if valid_depth[close_step] and np.all(np.isfinite(p)):
            draw_marker(draw, tuple(p), "CLOSE", COLOR_CLOSE)

    if open_step is not None and 0 <= open_step < len(pixels):
        p = pixels[open_step]
        if valid_depth[open_step] and np.all(np.isfinite(p)):
            label = "OPEN" if open_is_commanded else "OPEN"
            draw_marker(draw, tuple(p), label, COLOR_OPEN)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)

    return diagnostics


TASK_RE = re.compile(r"task_(\d+)$")
TRAJ_RE = re.compile(r"traj_(\d+)$")


def discover_pkls(rollouts_root, task_filter=None, traj_filter=None):
    selected = []

    for path in sorted(rollouts_root.glob("task_*/traj_*.pkl")):
        tm = TASK_RE.match(path.parent.name)
        rm = TRAJ_RE.match(path.stem)

        if not tm or not rm:
            continue

        task_id = tm.group(1).zfill(2)
        traj_number = int(rm.group(1))

        if task_filter is not None and task_id != str(task_filter).zfill(2):
            continue
        if traj_filter is not None and traj_number != int(traj_filter):
            continue

        selected.append(path)

    return selected


def classify_outcome(data):
    try:
        reached = int(data["object_reached"])
        picked = int(data["object_picked"])
        placed = int(data["object_placed"])
    except (KeyError, TypeError, ValueError):
        return "Inconsistent"

    if (reached, picked, placed) == (1, 1, 1):
        return "Success"
    if reached == 0:
        return "Reach failure"
    if (reached, picked, placed) == (1, 0, 0):
        return "Pick failure"
    if (reached, picked, placed) == (1, 1, 0):
        return "Place failure"

    return "Inconsistent"


def safe_int(value, default=0):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def safe_pct(a, b):
    return math.nan if not b else 100.0 * float(a) / float(b)


def trajectory_length_m(eef):
    if len(eef) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(eef, axis=0), axis=1).sum())


def analyze_pkl(pkl_path, open_threshold, close_threshold):
    payload, _, steps = load_rollout(pkl_path)

    eef = extract_eef_positions(steps)
    qpos = extract_gripper_qpos(steps)
    actions = extract_actions(steps)

    measured = detect_measured_gripper_events(
        qpos,
        open_threshold,
        close_threshold,
    )
    commanded = detect_commanded_gripper_events(actions)

    pixels, valid = project_base_points_to_pixels(eef)
    projection = projection_diagnostics(pixels, valid)

    return {
        "pkl_path": str(pkl_path),
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
        **projection,
    }


def read_annotations(path):
    if not path.exists():
        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "task_id",
                "traj_number",
                "note",
                "error_type",
                "bin_confusion",
            ])
        print(f"[stats] created {path}")
        return {}

    out = {}
    with path.open("r", newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if not row.get("task_id") or not row.get("traj_number"):
                continue
            key = (str(row["task_id"]).zfill(2), int(row["traj_number"]))
            out[key] = {
                k: v
                for k, v in row.items()
                if k not in {"task_id", "traj_number"}
            }

    return out


def summarize_rows(rows, scope):
    """
    Calcola statistiche aggregate globali o per singolo task.

    Categorie mutuamente esclusive:
      - Success:
            reached=1, picked=1, placed=1

      - Reach failure:
            reached=0

      - Pick failure:
            reached=1, picked=0

      - Place failure:
            reached=1, picked=1, placed=0

      - Inconsistent:
            qualsiasi altra combinazione anomala

    Le percentuali di Success / Reach failure / Pick failure / Place failure
    sono calcolate rispetto al numero totale di rollout nello scope.

    Le metriche condizionate:
      pick_given_reach_pct
      place_given_pick_pct

    descrivono invece la probabilità di superare una fase sapendo che la fase
    precedente è stata completata.
    """

    n = len(rows)

    counts = {
        category: sum(
            row["category"] == category
            for row in rows
        )
        for category in [
            "Success",
            "Reach failure",
            "Pick failure",
            "Place failure",
            "Inconsistent",
        ]
    }

    # -------------------------------------------------------------------------
    # FALLIMENTI TOTALI
    # -------------------------------------------------------------------------

    total_failures = (
        counts["Reach failure"]
        + counts["Pick failure"]
        + counts["Place failure"]
    )

    # -------------------------------------------------------------------------
    # RISULTATI DELLE SINGOLE FASI
    # -------------------------------------------------------------------------

    reached_count = sum(
        safe_int(row.get("object_reached")) == 1
        for row in rows
    )

    picked_count = sum(
        safe_int(row.get("object_picked")) == 1
        for row in rows
    )

    placed_count = sum(
        safe_int(row.get("object_placed")) == 1
        for row in rows
    )

    # Numeratori condizionati espliciti.
    # In questo modo eventuali rollout "Inconsistent" non possono produrre
    # percentuali condizionate logicamente errate.

    picked_after_reach_count = sum(
        safe_int(row.get("object_reached")) == 1
        and safe_int(row.get("object_picked")) == 1
        for row in rows
    )

    placed_after_pick_count = sum(
        safe_int(row.get("object_picked")) == 1
        and safe_int(row.get("object_placed")) == 1
        for row in rows
    )

    # -------------------------------------------------------------------------
    # SUMMARY
    # -------------------------------------------------------------------------

    return {
        "scope": scope,
        "n_rollouts": n,

        # ---------------------------------------------------------------------
        # Conteggi per categoria
        # ---------------------------------------------------------------------

        "success": counts["Success"],
        "reach_failure": counts["Reach failure"],
        "pick_failure": counts["Pick failure"],
        "place_failure": counts["Place failure"],
        "inconsistent": counts["Inconsistent"],

        "total_failures": total_failures,

        # ---------------------------------------------------------------------
        # Percentuali rispetto a tutti i rollout
        # ---------------------------------------------------------------------

        "success_rate_pct": safe_pct(
            counts["Success"],
            n,
        ),

        "reach_failure_rate_pct": safe_pct(
            counts["Reach failure"],
            n,
        ),

        "pick_failure_rate_pct": safe_pct(
            counts["Pick failure"],
            n,
        ),

        "place_failure_rate_pct": safe_pct(
            counts["Place failure"],
            n,
        ),

        "failure_rate_pct": safe_pct(
            total_failures,
            n,
        ),

        "inconsistent_rate_pct": safe_pct(
            counts["Inconsistent"],
            n,
        ),

        # ---------------------------------------------------------------------
        # Distribuzione dei fallimenti
        # ---------------------------------------------------------------------

        "reach_failure_share_of_failures_pct": safe_pct(
            counts["Reach failure"],
            total_failures,
        ),

        "pick_failure_share_of_failures_pct": safe_pct(
            counts["Pick failure"],
            total_failures,
        ),

        "place_failure_share_of_failures_pct": safe_pct(
            counts["Place failure"],
            total_failures,
        ),

        # ---------------------------------------------------------------------
        # Successo delle singole fasi rispetto a tutti i rollout
        # ---------------------------------------------------------------------

        "reach_rate_pct": safe_pct(
            reached_count,
            n,
        ),

        "pick_rate_pct": safe_pct(
            picked_count,
            n,
        ),

        "place_rate_pct": safe_pct(
            placed_count,
            n,
        ),

        # ---------------------------------------------------------------------
        # Metriche condizionate
        #
        # P(Pick | Reach)
        # P(Place | Pick)
        # ---------------------------------------------------------------------

        "pick_given_reach_pct": safe_pct(
            picked_after_reach_count,
            reached_count,
        ),

        "place_given_pick_pct": safe_pct(
            placed_after_pick_count,
            picked_count,
        ),

        # ---------------------------------------------------------------------
        # Conteggi intermedi, utili per verificare le percentuali
        # ---------------------------------------------------------------------

        "reached_count": reached_count,
        "picked_count": picked_count,
        "placed_count": placed_count,

        "picked_after_reach_count": picked_after_reach_count,
        "placed_after_pick_count": placed_after_pick_count,
    }

def write_csv(path, rows):
    fieldnames = []
    seen = set()

    for row in rows:
        for key in row:
            if key not in seen:
                seen.add(key)
                fieldnames.append(key)

    with path.open(
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as f:
        writer = csv.DictWriter(
            f,
            fieldnames=fieldnames,
            delimiter=";",
        )

        writer.writeheader()
        writer.writerows(rows)


def run_stats(args):
    ensure_savers_importable(args.savers_dir)

    root = args.rollouts_root.resolve()
    out_dir = args.output_dir.resolve()
    annotations = read_annotations(out_dir / "annotations.csv")

    rows = []

    for json_path in sorted(root.glob("task_*/traj_*.json")):
        tm = TASK_RE.match(json_path.parent.name)
        rm = TRAJ_RE.match(json_path.stem)

        if not tm or not rm:
            continue

        task_id = tm.group(1).zfill(2)
        traj_number = int(rm.group(1))

        with json_path.open("r", encoding="utf-8") as f:
            outcome = json.load(f)

        row = {
            "task_id": task_id,
            "traj_number": traj_number,
            "json_path": str(json_path),
            **outcome,
        }

        row["category"] = classify_outcome(outcome)

        pkl_path = json_path.with_suffix(".pkl")
        row["has_pkl"] = int(pkl_path.is_file())

        if pkl_path.is_file():
            try:
                row.update(
                    analyze_pkl(
                        pkl_path,
                        args.gripper_open_threshold,
                        args.gripper_close_threshold,
                    )
                )
            except Exception as exc:
                row["pkl_analysis_error"] = repr(exc)

        if (task_id, traj_number) in annotations:
            row.update(annotations[(task_id, traj_number)])

        rows.append(row)

    rows.sort(key=lambda r: (r["task_id"], int(r["traj_number"])))

    write_csv(out_dir / "rollouts.csv", rows)

    summary = [summarize_rows(rows, "global")]

    for task_id in sorted({r["task_id"] for r in rows}):
        task_rows = [r for r in rows if r["task_id"] == task_id]
        summary.append(summarize_rows(task_rows, f"task_{task_id}"))

    write_csv(out_dir / "summary.csv", summary)

    print(f"[stats] wrote {out_dir / 'rollouts.csv'}")
    print(f"[stats] wrote {out_dir / 'summary.csv'}")


def run_inspect(args):
    ensure_savers_importable(args.savers_dir)

    if args.pkl:
        pkl_path = args.pkl.resolve()
    else:
        matches = discover_pkls(
            args.rollouts_root.resolve(),
            args.task,
            args.traj,
        )
        if not matches:
            raise FileNotFoundError("No rollout matched")
        pkl_path = matches[0]

    payload, _, steps = load_rollout(pkl_path)

    frames = extract_front_frames(steps)
    eef = extract_eef_positions(steps)
    qpos = extract_gripper_qpos(steps)
    actions = extract_actions(steps)

    measured = detect_measured_gripper_events(
        qpos,
        args.gripper_open_threshold,
        args.gripper_close_threshold,
    )
    commanded = detect_commanded_gripper_events(actions)

    pixels, valid = project_base_points_to_pixels(eef)
    projection = projection_diagnostics(pixels, valid)

    print("=" * 80)
    print("PKL:", pkl_path)
    print("task_id:", payload.get("task_id"))
    print("traj_number:", payload.get("traj_number"))
    print("steps:", len(steps))
    print("frame shape:", frames[0].shape)
    print("action shape:", actions.shape)
    print("EEF path length [m]:", f"{trajectory_length_m(eef):.4f}")
    print()
    print("MEASURED GRIPPER")
    for key, value in measured.items():
        print(f"  {key}: {value}")
    print()
    print("COMMAND DIAGNOSTICS")
    for key, value in commanded.items():
        print(f"  {key}: {value}")
    print()
    print("PROJECTION")
    for key, value in projection.items():
        print(f"  {key}: {value:.2f}")

    if args.print_gripper_sequence:
        print()
        print("STEP | measured_qpos | action_gripper")
        print("-----+---------------+---------------")
        for i, q in enumerate(qpos):
            print(f"{i:4d} | {q:13.6f} | {actions[i, -1]:13.3f}")

def run_render(args):
    ensure_savers_importable(args.savers_dir)

    if args.config is None:
        raise ValueError("render requires --config")

    config_path = args.config.resolve()
    tasks = load_tasks_from_config(config_path)

    pkls = discover_pkls(
        args.rollouts_root.resolve(),
        args.task,
        args.traj,
    )

    if args.limit is not None:
        pkls = pkls[:args.limit]

    if not pkls:
        raise FileNotFoundError("No rollout PKLs selected")

    render_root = args.output_dir.resolve() / "render"

    print(f"[render] selected {len(pkls)} rollout(s)")

    rendered = 0
    skipped = 0

    for idx, pkl_path in enumerate(pkls, 1):
        try:
            payload, _, steps = load_rollout(pkl_path)

            task_id = str(payload["task_id"]).zfill(2)
            traj_number = int(payload["traj_number"])

            task_cfg = tasks[task_id]
            prompt = str(task_cfg["prompt"])

            instruction_paths = resolve_instruction_images(
                task_cfg,
                config_path,
                args.instruction_images_dir.resolve(),
            )

            frames = extract_front_frames(steps)
            eef = extract_eef_positions(steps)
            qpos = extract_gripper_qpos(steps)
            actions = extract_actions(steps)

            if len(frames) < 2:
                raise ValueError(
                    f"Rollout task={task_id} traj={traj_number:03d} has fewer than 2 frames"
                )

            measured = detect_measured_gripper_events(
                qpos,
                args.gripper_open_threshold,
                args.gripper_close_threshold,
            )
            commanded = detect_commanded_gripper_events(actions)

            # Visualizations intentionally start from the SECOND saved step.
            visual_start_step = 1
            frames_visual = frames[visual_start_step:]
            eef_visual = eef[visual_start_step:]

            def to_visual_index(step):
                if step is None or step < visual_start_step:
                    return None
                return step - visual_start_step

            close_step_visual = to_visual_index(measured["close_step"])

            # Prefer a physically observed reopening. If the rollout is truncated
            # immediately after the open command, mark the release at the EEF pose
            # associated with that command and make the source explicit in the label.
            open_is_commanded = False
            open_step_original = measured["open_step"]

            if open_step_original is None and commanded["open_command_step"] is not None:
                open_step_original = commanded["open_command_step"]
                open_is_commanded = True

            open_step_visual = to_visual_index(open_step_original)

            task_dir = render_root / f"task_{task_id}"

            video_path = task_dir / f"traj_{traj_number:03d}.mp4"
            png_path = task_dir / f"traj_{traj_number:03d}_trajectory.png"

            render_video(
                video_path,
                frames_visual,
                prompt,
                instruction_paths,
                task_id,
                traj_number,
                args.fps,
            )

            projection = render_trajectory_png(
                png_path,
                frames_visual[0],
                eef_visual,
                close_step_visual,
                open_step_visual,
                open_is_commanded=open_is_commanded,
            )

            open_source = (
                "command_fallback"
                if open_is_commanded
                else ("measured" if measured["open_step"] is not None else "missing")
            )

            rendered += 1

            print(
                f"[render {idx:02d}/{len(pkls):02d}] "
                f"task={task_id} traj={traj_number:03d} "
                f"steps={len(steps)} visual_steps={len(frames_visual)} "
                f"close={measured['close_step']} "
                f"open={open_step_original}({open_source}) "
                f"inside={projection['projection_inside_image_pct']:.1f}%"
            )

        except Exception as exc:
            skipped += 1

            print(
                f"[SKIP {idx:02d}/{len(pkls):02d}] "
                f"{pkl_path.parent.name}/{pkl_path.name}: "
                f"{type(exc).__name__}: {exc}"
            )

            continue

    print()
    print("=" * 80)
    print("RENDER COMPLETE")
    print(f"  selected: {len(pkls)}")
    print(f"  rendered: {rendered}")
    print(f"  skipped:  {skipped}")
    print(f"  output:   {render_root}")


def build_parser():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "mode",
        choices=["inspect", "stats", "render"],
    )

    parser.add_argument(
        "--rollouts-root",
        type=Path,
        default=DEFAULT_ROLLOUTS_ROOT,
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
    )

    parser.add_argument(
        "--savers-dir",
        type=Path,
        default="/home/asus-mivia/Desktop/Multi-Task-LFD/repo/VLA-Bench/robosuite_test/tasks/training/multi_task_il/datasets",
    )

    parser.add_argument(
        "--config",
        type=Path,
        default="/home/asus-mivia/Desktop/Alex/UR5e-2f-85/ai_controller/ai_controller/models/interleave_pi0_controller/interleave_pi0_config.yaml",
    )


    parser.add_argument(
        "--instruction-images-dir",
        type=Path,
        default=DEFAULT_INSTRUCTION_IMAGES_DIR,
    )

    parser.add_argument(
        "--task",
        type=str,
        default=None,
    )

    parser.add_argument(
        "--traj",
        type=int,
        default=None,
    )

    parser.add_argument(
        "--pkl",
        type=Path,
        default=None,
    )

    parser.add_argument(
        "--fps",
        type=float,
        default=5.0,
    )

    parser.add_argument(
        "--limit",
        type=int,
        default=None,
    )

    parser.add_argument(
        "--gripper-open-threshold",
        type=float,
        default=None,
    )

    parser.add_argument(
        "--gripper-close-threshold",
        type=float,
        default=None,
    )

    parser.add_argument(
        "--print-gripper-sequence",
        action="store_true",
    )

    return parser


def main():
    args = build_parser().parse_args()

    if args.fps <= 0:
        raise ValueError("--fps must be > 0")

    if args.mode == "inspect":
        run_inspect(args)
    elif args.mode == "stats":
        run_stats(args)
    elif args.mode == "render":
        run_render(args)


if __name__ == "__main__":
    main()
