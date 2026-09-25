#!/usr/bin/env python3
"""TEMPORARY script: run REAL OSVI-AWDA inference offline (real checkpoint,
real AWDA forward pass) using:

  - the human-demo context already extracted and saved for a given rollout
    (osvi_awda_context_NNN/context_raw_*.png, produced by
    OSVIAWDAController.load_command() during that rollout), instead of
    needing the original demo dataset (which may not exist on this host);
  - the first front-camera frame stored in that rollout's traj_XXX.pkl
    (obs['camera_front_image'] at t=0) as the live scene.

It then draws the 5 predicted AWDA waypoints (with their primitive labels:
FREE / GRASP / CARRY / CARRY / DROP) on that observation frame and saves a
PNG, for use in flow-explanation figures.

This runs entirely on the host (no ROS, no Docker) - only requires an
environment with the OSVI-AWDA torch stack, e.g.:
    conda activate osvi_awda

The runtime config (configs/osvi_awda_config.yaml) hardcodes checkpoint_dir
as a container path (/home/ros2_ws/src/...), which is how the repo is
bind-mounted into the docker container. HostOSVIAWDAController below remaps
that prefix to the actual repo root so the checkpoint resolves on the host,
without touching the shared yaml config.

Usage:
    python3 run_inference_waypoints_overlay.py \\
        --pkl /path/to/traj_002.pkl \\
        --context-dir /path/to/osvi_awda_context_002 \\
        [--config configs/osvi_awda_config.yaml] \\
        [--output-dir /tmp/osvi_awda_waypoints_traj_002] \\
        [--savers-dir /path/to/dataset_collector_pkg/scripts]
"""
import argparse
import importlib
import pickle
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def find_repo_root() -> Path:
    # Note: the ROS package dir ai_controller/ itself contains a nested
    # ai_controller/ python package dir (ROS2 package_name/package_name
    # convention), so checking for "ai_controller/" alone is ambiguous and
    # can stop one level too high. Require the nested layout too.
    current = Path(__file__).resolve()
    for candidate in [Path.cwd().resolve(), current.parent, *current.parents]:
        if (candidate / "ai_controller" / "ai_controller").is_dir():
            return candidate
    raise RuntimeError("Cannot find repository root containing ai_controller/ai_controller/.")


REPO_ROOT = find_repo_root()
THIS_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = THIS_DIR / "configs" / "osvi_awda_config.yaml"
DEFAULT_SAVERS_DIR = REPO_ROOT / "dataset_collector" / "dataset_collector_pkg" / "scripts"
CONTAINER_PREFIX = "/home/ros2_ws/src/"


def import_controller_class():
    # The importable "ai_controller" package is the INNER directory (ROS2
    # ament_python's package_name/package_name layout), so the OUTER
    # ai_controller/ ROS package dir must be on sys.path, not the repo root.
    sys.path.insert(0, str(REPO_ROOT / "ai_controller"))
    module_name = "ai_controller.models.osvi_awda_controller.osvi_awda_controller"
    module = importlib.import_module(module_name)
    return module.OSVIAWDAController


def build_host_controller_class(base_class):
    """Subclass that remaps the docker bind-mount prefix used by
    checkpoint_dir in the shared runtime config to the real repo root on
    this host, without editing the (shared) yaml config file."""

    class HostOSVIAWDAController(base_class):
        def _resolve_path(self, value):
            text = str(value)
            if text.startswith(CONTAINER_PREFIX):
                text = str(REPO_ROOT / text[len(CONTAINER_PREFIX):])
            return super()._resolve_path(text)

    return HostOSVIAWDAController


# =====================================================================
# Context loading: rebuild context_tensor from already-extracted
# context_raw_*.png frames instead of controller.load_command(), which
# needs the original (container-only) human-demo dataset path.
# =====================================================================

def load_context_from_saved_frames(controller, context_dir: Path):
    raw_frames = sorted(context_dir.glob("context_raw_*.png"))
    if not raw_frames:
        raise FileNotFoundError(f"No context_raw_*.png found in {context_dir}")

    import torch  # local import: only needed once the controller/model exist

    processed = []
    for frame_path in raw_frames:
        frame = np.asarray(Image.open(frame_path).convert("RGB"), dtype=np.uint8)
        processed.append(controller._preprocess_frame(frame, source="demo"))

    context_array = np.stack(processed, axis=0)
    controller.context_tensor = (
        torch.from_numpy(context_array).unsqueeze(0).float().to(controller.device)
    )
    controller.context_source = str(context_dir)
    print(
        f"[run_inference] Loaded context from {len(raw_frames)} saved frames in "
        f"{context_dir}: {tuple(controller.context_tensor.shape)}"
    )


def load_first_frame_from_pkl(pkl_path: Path, savers_dir: Path):
    savers_dir = str(savers_dir)
    if savers_dir not in sys.path:
        sys.path.insert(0, savers_dir)
    import savers  # noqa: F401  (import check; unpickling needs it registered)

    with pkl_path.open("rb") as stream:
        data = pickle.load(stream)
    traj = data["traj"]
    if traj.T == 0:
        raise ValueError(f"{pkl_path} has an empty trajectory.")

    obs0 = traj.get(0)["obs"]
    # ai_controller_node.py stores camera_front_image as BGR
    # (cv2.cvtColor(images[0], cv2.COLOR_RGB2BGR) before traj.append()).
    # The controller expects RGB, matching get_synced_images()'s rgb8 decode.
    frame_bgr = obs0["camera_front_image"]
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    eef_pos = np.asarray(obs0.get("eef_pos", [0.0, 0.35, 0.20]), dtype=np.float64)
    return frame_rgb, eef_pos, data


# =====================================================================
# Waypoint pixel-space conversion + drawing (adapted from
# debug/test_osvi_awda_scene_offline.py, kept self-contained here).
# =====================================================================

PRIMITIVE_COLORS = {
    "free_space": (0, 120, 255),
    "grasp": (255, 0, 0),
    "carry": (255, 165, 0),
    "drop": (0, 200, 0),
}


def _normalized_waypoint_to_raw_pixel(waypoint, raw_width, raw_height, crop, model_width, model_height):
    top, bottom, left, right = [int(v) for v in crop]
    cropped_width = raw_width - left - right
    cropped_height = raw_height - top - bottom
    if cropped_width <= 0 or cropped_height <= 0:
        raise ValueError(f"Invalid crop {crop} for {raw_width}x{raw_height} image.")

    u, v = float(waypoint[0]), float(waypoint[1])
    x_model = (u + 1.0) * 0.5 * (model_width - 1)
    y_model = (1.0 - v) * 0.5 * (model_height - 1)

    x_crop = x_model / float(model_width - 1) * (cropped_width - 1) if model_width > 1 else 0.0
    y_crop = y_model / float(model_height - 1) * (cropped_height - 1) if model_height > 1 else 0.0

    x_raw = int(np.clip(round(left + x_crop), 0, raw_width - 1))
    y_raw = int(np.clip(round(top + y_crop), 0, raw_height - 1))
    return x_raw, y_raw


FONT_CANDIDATES = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
]


def _load_label_font(image_size):
    font_size = max(14, int(min(image_size) * 0.045))
    for candidate in FONT_CANDIDATES:
        if Path(candidate).is_file():
            return ImageFont.truetype(candidate, font_size)
    return ImageFont.load_default(size=font_size)


def draw_waypoints_on_image(image_rgb, pixel_points, waypoints, primitives, output_path):
    image = Image.fromarray(np.asarray(image_rgb, dtype=np.uint8)).convert("RGB")
    draw = ImageDraw.Draw(image)
    radius = max(5, int(min(image.width, image.height) * 0.012))
    font = _load_label_font((image.width, image.height))

    for i in range(len(pixel_points) - 1):
        draw.line([pixel_points[i], pixel_points[i + 1]], fill=(255, 255, 255), width=max(2, radius // 3))

    for i, ((x, y), waypoint, primitive) in enumerate(zip(pixel_points, waypoints, primitives), start=1):
        color = PRIMITIVE_COLORS.get(primitive, (200, 200, 200))
        draw.ellipse([x - radius, y - radius, x + radius, y + radius], fill=color, outline=(255, 255, 255), width=2)

        label = primitive.upper()
        text_x, text_y = x + radius + 4, y - radius - 2
        bbox = draw.textbbox((text_x, text_y), label, font=font)
        padding = 2
        draw.rectangle(
            [bbox[0] - padding, bbox[1] - padding, bbox[2] + padding, bbox[3] + padding],
            fill=(0, 0, 0),
        )
        draw.text((text_x, text_y), label, fill=(255, 255, 255), font=font)

        if primitive == "grasp":
            cross_radius = radius + 5
            draw.line([x - cross_radius, y, x + cross_radius, y], fill=(255, 0, 0), width=2)
            draw.line([x, y - cross_radius, x, y + cross_radius], fill=(255, 0, 0), width=2)

    image.save(output_path)
    print(f"[run_inference] Saved: {output_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pkl", required=True, help="traj_XXX.pkl to take the t=0 front-camera frame from")
    parser.add_argument("--context-dir", required=True, help="osvi_awda_context_NNN directory (context_raw_*.png)")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="OSVI-AWDA runtime YAML config")
    parser.add_argument("--output-dir", default=None, help="Where to save the overlay PNG + debug files")
    parser.add_argument("--savers-dir", default=str(DEFAULT_SAVERS_DIR), help="dataset_collector_pkg/scripts path")
    args = parser.parse_args()

    pkl_path = Path(args.pkl).expanduser().resolve()
    context_dir = Path(args.context_dir).expanduser().resolve()
    config_path = Path(args.config).expanduser().resolve()
    output_dir = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir
        else Path(f"/tmp/osvi_awda_waypoints_{pkl_path.stem}")
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 90)
    print("OSVI-AWDA OFFLINE INFERENCE FROM SAVED ROLLOUT")
    print("=" * 90)
    print("pkl:        ", pkl_path)
    print("context_dir:", context_dir)
    print("config:     ", config_path)
    print("output_dir: ", output_dir)

    OSVIAWDAController = import_controller_class()
    HostOSVIAWDAController = build_host_controller_class(OSVIAWDAController)

    print("\n[run_inference] Loading real AWDA controller/checkpoint...")
    controller = HostOSVIAWDAController(str(config_path), task_name="pick_place")
    controller.reset()

    print("\n[run_inference] Loading human-demo context from saved frames...")
    load_context_from_saved_frames(controller, context_dir)

    print("\n[run_inference] Loading first front-camera frame from the rollout pkl...")
    scene_rgb, eef_pos, meta = load_first_frame_from_pkl(pkl_path, Path(args.savers_dir))
    print("scene shape:", scene_rgb.shape, "dtype:", scene_rgb.dtype)

    raw_scene_path = output_dir / f"{pkl_path.stem}_first_frame.png"
    Image.fromarray(scene_rgb).save(raw_scene_path)
    print(f"[run_inference] Saved: {raw_scene_path}")

    robot_state = np.concatenate([eef_pos, np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)])
    input_data = ([scene_rgb], robot_state)

    print("\n[run_inference] Running REAL AWDA forward (t=0)...")
    controller.inference(input_data, t=0, save_path=str(output_dir))

    image_waypoints = np.asarray(controller.last_image_waypoints, dtype=np.float64)
    primitives = [item["primitive"] for item in controller.last_gripper_decisions]

    print("\nPredicted waypoints / primitives:")
    for i, (wp, primitive) in enumerate(zip(image_waypoints, primitives), start=1):
        print(f"  WP{i} [{primitive:>10s}] u={wp[0]:+.4f} v={wp[1]:+.4f} depth={wp[2]:+.4f} grasp_attr={wp[3]:+.4f}")

    model_width = int(controller.cfg.image.get("width", 180))
    model_height = int(controller.cfg.image.get("height", 100))
    crop = controller.cfg.image.get("crop", [0, 0, 0, 0])
    raw_height, raw_width = scene_rgb.shape[:2]

    raw_points = [
        _normalized_waypoint_to_raw_pixel(wp, raw_width, raw_height, crop, model_width, model_height)
        for wp in image_waypoints
    ]

    overlay_path = output_dir / f"{pkl_path.stem}_waypoints_overlay.png"
    draw_waypoints_on_image(scene_rgb, raw_points, image_waypoints, primitives, overlay_path)

    print(f"\n[run_inference] task_id={meta.get('task_id')} traj={meta.get('traj_number')} "
          f"completed={meta.get('completed')}")
    print("Done.")


if __name__ == "__main__":
    main()
