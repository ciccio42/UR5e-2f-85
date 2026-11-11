#!/usr/bin/env python3
"""Presentation videos of saved real-robot rollouts (one .mp4 per trajectory).

Two layouts:

  --mode text  (text-conditioned policies: VLA-JEPA with a static prompt, Cosmos+VLA-JEPA)
      every rollout step shows the front and gripper cameras side by side, with the text
      command used by the policy overlaid at the bottom. The command is read from
      traj_<n>/cosmos/caption.json when present (Cosmos runs), otherwise from the
      tasks[<id>].prompt map of --config (VLA-JEPA static prompt).

  --mode osvi  (OSVI-AWDA)
      1. the demonstration context frames (osvi_awda_context_<n>/context_raw_*.png),
      2. the frame with the predicted AWDA waypoints (left panel of
         traj_<n>/step_0/osvi_awda_debug_overlay_t000.png),
      3. the robot rollout, front and gripper cameras side by side.

Rollout frames come from the trajectory .pkl (camera_front_image / camera_gripper_image,
stored BGR by ai_controller_node.py). Output: <out_dir>/task_<id>/traj_<n>.mp4.

Examples:
  python3 render_rollout_videos.py --mode text \\
      --root .../Alex/.../saved_rollouts/vla_jepa_controller/pick_place \\
      --config .../Alex/.../models/vla_jepa_controller/vla_jepa_config.yaml --out_dir videos/vla_jepa
  python3 render_rollout_videos.py --mode text --root .../vla_jepa_controller/pick_place/cosmos \\
      --out_dir videos/cosmos_vla_jepa
  python3 render_rollout_videos.py --mode osvi --root .../osvi_awda_controller/pick_place \\
      --out_dir videos/osvi_awda
"""
import argparse
import glob
import importlib
import json
import os
import pickle
import re
import shutil
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

DEFAULT_SAVERS_DIRS = (
    "/home/ros2_ws/src/dataset_collector/dataset_collector_pkg/scripts",
    str(Path(__file__).resolve().parents[3] / "dataset_collector" / "dataset_collector_pkg" / "scripts"),
)
FONT = cv2.FONT_HERSHEY_SIMPLEX
# OSVI-AWDA debug overlay = front | 4px | gripper | 4px | depth, all panels the same width
OSVI_OVERLAY_SEPARATOR_PX = 4


class TrajectoryUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if module.startswith("multi_task_il") and name == "Trajectory":
            return importlib.import_module("savers").Trajectory
        return super().find_class(module, name)


def _add_savers_to_path(savers_dir):
    for candidate in ([savers_dir] if savers_dir else []) + list(DEFAULT_SAVERS_DIRS):
        if candidate and os.path.isdir(candidate):
            if candidate not in sys.path:
                sys.path.insert(0, candidate)
            importlib.import_module("savers")
            return
    raise RuntimeError("savers.py (dataset_collector_pkg/scripts) not found; pass --savers-dir")


class VideoWriter:
    """H.264 through an ffmpeg binary when available (PATH or imageio-ffmpeg), otherwise
    OpenCV's MPEG-4 writer (no H.264 encoder in this OpenCV build)."""

    def __init__(self, path, fps, size):
        self.path, self.size = str(path), size
        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg is None:
            try:
                import imageio_ffmpeg
                ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
            except ImportError:
                ffmpeg = None
        if ffmpeg:
            width, height = size
            self.proc = subprocess.Popen(
                [ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24",
                 "-s", f"{width}x{height}", "-r", str(fps), "-i", "-", "-c:v", "libx264",
                 "-pix_fmt", "yuv420p", "-crf", "20", self.path],
                stdin=subprocess.PIPE)
            self.writer = None
        else:
            self.proc = None
            self.writer = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*"mp4v"), fps, size)

    def write(self, frame, repeat=1):
        for _ in range(repeat):
            if self.proc is not None:
                self.proc.stdin.write(np.ascontiguousarray(frame).tobytes())
            else:
                self.writer.write(frame)

    def close(self):
        if self.proc is not None:
            self.proc.stdin.close()
            self.proc.wait()
        else:
            self.writer.release()


def _wrap(text, scale, thickness, max_width):
    lines, current = [], ""
    for word in text.split():
        candidate = f"{current} {word}".strip()
        if cv2.getTextSize(candidate, FONT, scale, thickness)[0][0] > max_width and current:
            lines.append(current)
            current = word
        else:
            current = candidate
    return lines + ([current] if current else [])


def _label(image, text, origin=(12, 30), scale=0.8):
    cv2.putText(image, text, origin, FONT, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, text, origin, FONT, scale, (255, 255, 255), 2, cv2.LINE_AA)


def _overlay_banner(image, text, scale=0.9, thickness=2, margin=14):
    """Semi-transparent black band at the bottom of image with text wrapped inside."""
    lines = _wrap(text, scale, thickness, image.shape[1] - 2 * margin)
    line_h = int(cv2.getTextSize("Ag", FONT, scale, thickness)[0][1] * 1.8)
    band_h = 2 * margin + line_h * len(lines)
    top = image.shape[0] - band_h
    band = image[top:].astype(np.float32)
    image[top:] = (band * 0.35).astype(np.uint8)
    for i, line in enumerate(lines):
        y = top + margin + line_h * (i + 1) - line_h // 4
        cv2.putText(image, line, (margin, y), FONT, scale, (255, 255, 255), thickness, cv2.LINE_AA)


def _side_by_side(front, gripper, step=None, total=None):
    height = min(front.shape[0], gripper.shape[0])

    def fit(img):
        if img.shape[0] == height:
            return img.copy()
        return cv2.resize(img, (int(round(img.shape[1] * height / img.shape[0])), height))

    front, gripper = fit(front), fit(gripper)
    _label(front, "Front camera")
    _label(gripper, "Gripper camera")
    frame = np.hstack([front, gripper])
    if step is not None:
        text = f"t = {step}/{total}"
        width = cv2.getTextSize(text, FONT, 0.8, 2)[0][0]
        _label(frame, text, (frame.shape[1] - width - 12, 30))
    return frame


def _on_canvas(image, size):
    """Center image (scaled to fit) on a black canvas of size (width, height)."""
    width, height = size
    scale = min(width / image.shape[1], height / image.shape[0])
    resized = cv2.resize(image, (int(image.shape[1] * scale), int(image.shape[0] * scale)))
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    y0, x0 = (height - resized.shape[0]) // 2, (width - resized.shape[1]) // 2
    canvas[y0:y0 + resized.shape[0], x0:x0 + resized.shape[1]] = resized
    return canvas, x0


def _label_in_margin(canvas, text, margin_width, scale=0.8, thickness=2):
    """Write text wrapped inside the left black margin left by _on_canvas."""
    lines = _wrap(text, scale, thickness, max(margin_width - 24, 50))
    line_h = int(cv2.getTextSize("Ag", FONT, scale, thickness)[0][1] * 1.8)
    for i, line in enumerate(lines):
        cv2.putText(canvas, line, (12, 30 + i * line_h), FONT, scale, (255, 255, 255), thickness, cv2.LINE_AA)


def _load_traj(pkl_path):
    with open(pkl_path, "rb") as stream:
        payload = TrajectoryUnpickler(stream).load()
    return payload["traj"] if isinstance(payload, dict) else payload


def _decode(image):
    """Rollout images are raw BGR arrays, or JPEG bytes (SeeDo) that decode to BGR."""
    image = np.asarray(image)
    return cv2.imdecode(image, cv2.IMREAD_COLOR) if image.ndim == 1 else image


def _rollout_frames(traj):
    frames = []
    for t in range(traj.T):
        obs = traj.get(t)["obs"]
        front = obs.get("camera_front_image")
        gripper = obs.get("camera_gripper_image", obs.get("eye_in_hand_image"))
        if front is None or gripper is None:
            continue
        frames.append(_side_by_side(_decode(front), _decode(gripper), t, traj.T - 1))
    return frames


def _text_command(traj_stem, task_id, prompts):
    caption_file = Path(traj_stem) / "cosmos" / "caption.json"
    if caption_file.is_file():
        return "Cosmos command: " + json.load(open(caption_file))["caption"]
    prompt = prompts.get(task_id)
    return f"Command: {prompt}" if prompt else None


TEXT_FPS = 8


def _read_video(path):
    capture, frames = cv2.VideoCapture(str(path)), []
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        frames.append(frame)
    capture.release()
    return frames


def _crop_black_padding(frames):
    """Remove the black bands added by cosmos_captioner's square padding (same crop for all)."""
    if not frames:
        return frames
    content = np.max(np.stack(frames[: min(10, len(frames))]), axis=(0, 3)) > 15
    rows, cols = np.where(content.any(axis=1))[0], np.where(content.any(axis=0))[0]
    if not len(rows) or not len(cols):
        return frames
    return [f[rows[0]:rows[-1] + 1, cols[0]:cols[-1] + 1] for f in frames]


def render_text(pkl_path, out_path, fps, prompts, demo_sec=5.0):
    task_id = Path(pkl_path).parent.name.split("_")[-1]
    stem = str(pkl_path)[:-len(".pkl")]
    command = _text_command(stem, task_id, prompts)
    if command is None:
        print(f"  skip {pkl_path}: no caption.json and task {task_id} not in --config")
        return False
    frames = _rollout_frames(_load_traj(pkl_path))
    if not frames:
        print(f"  skip {pkl_path}: no front/gripper images")
        return False
    size = (frames[0].shape[1], frames[0].shape[0])
    writer = VideoWriter(out_path, TEXT_FPS, size)

    # Cosmos runs: the human demonstration Cosmos captioned, then its caption
    demo_videos = sorted(Path(stem, "cosmos").glob("task_*_demo.mp4"))
    demo = _crop_black_padding(_read_video(demo_videos[0])) if demo_videos else []
    if demo:
        step = max(1, round(len(demo) / (demo_sec * TEXT_FPS)))
        for frame in demo[::step]:
            canvas, margin = _on_canvas(frame, size)
            _label_in_margin(canvas, "1. Human demonstration (input to Cosmos)", margin)
            writer.write(canvas)
        canvas, margin = _on_canvas(demo[-1], size)
        _label_in_margin(canvas, "2. Task description generated by Cosmos", margin)
        _overlay_banner(canvas, command)
        writer.write(canvas, repeat=round(3.0 * TEXT_FPS))

    prefix = "3. Robot rollout - " if demo else ""
    for frame in frames:
        _overlay_banner(frame, prefix + command)
        writer.write(frame, repeat=max(1, round(TEXT_FPS / fps)))
    writer.close()
    return True


def render_osvi(pkl_path, out_path, fps, context_sec, waypoint_sec):
    task_dir, stem = Path(pkl_path).parent, Path(pkl_path).stem
    traj_number = int(stem.split("_")[-1])
    frames = _rollout_frames(_load_traj(pkl_path))
    if not frames:
        print(f"  skip {pkl_path}: no front/gripper images")
        return False
    size = (frames[0].shape[1], frames[0].shape[0])
    writer = VideoWriter(out_path, fps, size)

    context_files = sorted(glob.glob(str(task_dir / f"osvi_awda_context_{traj_number:03d}" / "context_raw_*.png")))
    if not context_files:
        print(f"  warning {pkl_path}: no osvi_awda_context_{traj_number:03d}/context_raw_*.png")
    for i, path in enumerate(context_files):
        frame, margin = _on_canvas(cv2.imread(path), size)
        _label_in_margin(frame, f"1. Demonstration context frame {i + 1}/{len(context_files)}", margin)
        writer.write(frame, repeat=max(1, round(context_sec * fps)))

    overlay_file = task_dir / stem / "step_0" / "osvi_awda_debug_overlay_t000.png"
    if overlay_file.is_file():
        overlay = cv2.imread(str(overlay_file))
        panel_w = (overlay.shape[1] - 2 * OSVI_OVERLAY_SEPARATOR_PX) // 3
        frame, margin = _on_canvas(overlay[:, :panel_w], size)
        _label_in_margin(frame, "2. Predicted AWDA waypoints", margin)
        writer.write(frame, repeat=max(1, round(waypoint_sec * fps)))
    else:
        print(f"  warning {pkl_path}: no {overlay_file.relative_to(task_dir)}")

    for frame in frames:
        _label(frame, "3. Robot rollout", (12, frame.shape[0] - 16))
        writer.write(frame)
    writer.close()
    return True


SEEDO_FPS = 8


def _read_json(path):
    """json.load, skipping '#'-commented lines (some run_status.json files have them)."""
    try:
        with open(path, "r", encoding="utf-8") as stream:
            text = "\n".join(line for line in stream if not line.lstrip().startswith("#"))
        return json.loads(text)
    except (OSError, ValueError):
        return None


def _text_panel(width, height, title, lines, scale=0.6, mono=False):
    """Black panel with a title and wrapped body lines (cut when they overflow)."""
    panel = np.zeros((height, width, 3), dtype=np.uint8)
    font = cv2.FONT_HERSHEY_PLAIN if mono else FONT
    body_scale = scale * 1.6 if mono else scale
    _label(panel, title)
    line_h = int(cv2.getTextSize("Ag", font, body_scale, 1)[0][1] * 1.9)
    y = 70
    for raw in lines:
        for line in _wrap(raw, body_scale, 1, width - 36) if raw else [""]:
            if y > height - 10:
                return panel
            cv2.putText(panel, line, (18, y), font, body_scale, (230, 230, 230), 1, cv2.LINE_AA)
            y += line_h
    return panel


def _seedo_demo(test_dir, demo_root, rollout_task_id):
    """Demonstration frames (BGR) the planner saw, a label, and a function mapping the
    planner's frame indices onto these frames."""
    tracked = (sorted(test_dir.glob("visual_prompting/*tracked-h264.mp4"))
               or sorted(test_dir.glob("visual_prompting/*tracked.mp4")))
    if tracked:
        capture, frames = cv2.VideoCapture(str(tracked[0])), []
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            frames.append(frame)
        capture.release()
        if frames:
            return frames, "Human demonstration + object tracking", (lambda i: i), False

    # run_status.json is missing in some tests: fall back to the rollout's task and traj000,
    # the demo every other test of these campaigns used
    status = _read_json(test_dir / "run_status.json") or {}
    task_id = str(status.get("task_id") or rollout_task_id).zfill(2)
    match = re.search(r"traj_?(\d+)", str(status.get("demo_path", "")))
    demo_pkl = Path(demo_root) / f"task_{task_id}" / f"traj{int(match.group(1)) if match else 0:03d}.pkl"
    if not demo_pkl.is_file():
        return [], None, None, False
    traj = _load_traj(demo_pkl)
    # human-demo JPEGs decode to RGB (the collection-time encode swapped channels)
    frames = [_decode(traj.get(t)["obs"]["camera_front_image"])[:, :, ::-1].copy() for t in range(traj.T)]
    label = (f"Human demonstration task_{task_id}/{demo_pkl.stem} from the dataset (the video given "
             f"to the planner in this test was not saved)")
    return frames, label, None, True


def _seedo_keyframes(test_dir, demo_frames, mapping, approximate):
    """(pick image, place image, pick label, place label, footer text) or None."""
    selection = _read_json(test_dir / "keyframe_selection" / "vlm_keyframe_selection.json")
    if selection and selection.get("selection", {}).get("status") == "completed":
        sel = selection["selection"]
        images = [cv2.imread(str(test_dir / "keyframe_selection" / "candidate_frames" / f"frame_{sel[k]:06d}.jpg"))
                  for k in ("pick_frame", "place_frame")]
        if all(img is not None for img in images):
            footer = f"VLM: pick - {sel.get('pick_evidence', '')} Place - {sel.get('place_evidence', '')}"
            return (images[0], images[1], f"Pick keyframe (frame {sel['pick_frame']})",
                    f"Place keyframe (frame {sel['place_frame']})", footer)
    # keyframe indices refer to the planner's own input video: only shown when that video is
    # the one being displayed (not the dataset fallback)
    manifest = _read_json(test_dir / "action_planning" / "input_manifest.json") or {}
    roles = manifest.get("frame_roles") or {}
    if demo_frames and not approximate and "pick" in roles and "place" in roles:
        pick, place = int(roles["pick"]), int(roles["place"])
        if pick < len(demo_frames) and place < len(demo_frames):
            return (demo_frames[pick], demo_frames[place], f"Pick keyframe (frame {pick})",
                    f"Place keyframe (frame {place})", "Keyframes given to the action planner")
    return None


def _seedo_primitives(test_dir):
    plan = _read_json(test_dir / "motion_layer" / "motion_plan.json") or {}
    per_step, calls = [], []
    for i, primitive in enumerate(plan.get("primitives", [])):
        args = ", ".join(f"'{v}'" for v in (primitive.get("arguments") or {}).values())
        call = f"{primitive['name']}({args})"
        calls.append(call)
        per_step += [f"primitive {i + 1}/{len(plan['primitives'])}: {call}"] * len(primitive.get("actions", []))
    return per_step, calls


def render_seedo(pkl_path, out_path, fps, demo_root, demo_sec):
    test_dir = Path(str(pkl_path).split("/rollouts/")[0])
    frames = _rollout_frames(_load_traj(pkl_path))
    if not frames:
        print(f"  skip {pkl_path}: no front/gripper images")
        return False
    size = (frames[0].shape[1], frames[0].shape[0])
    width, height = size
    half = width // 2
    hold = lambda sec: max(1, round(sec * SEEDO_FPS))
    writer = VideoWriter(out_path, SEEDO_FPS, size)

    demo_frames, demo_label, mapping, approximate = _seedo_demo(
        test_dir, demo_root, Path(pkl_path).parent.name.split("_")[-1])
    if demo_frames:
        step = max(1, round(len(demo_frames) / (demo_sec * SEEDO_FPS)))
        for frame in demo_frames[::step]:
            canvas, margin = _on_canvas(frame, size)
            _label_in_margin(canvas, f"1. {demo_label}", margin)
            writer.write(canvas)
    else:
        print(f"  warning {pkl_path}: no demonstration video or dataset demo found")

    keyframes = _seedo_keyframes(test_dir, demo_frames, mapping, approximate)
    if keyframes:
        pick, place, pick_label, place_label, footer = keyframes
        frame = _side_by_side(cv2.resize(pick, (half, height)), cv2.resize(place, (width - half, height)))
        frame[:50] = 0
        _label(frame, "2. Keyframe selection")
        _label(frame, pick_label, (12, 80), 0.7)
        _label(frame, place_label, (half + 12, 80), 0.7)
        _overlay_banner(frame, footer, scale=0.6)
        writer.write(frame, repeat=hold(4.0))

    plan_txt = test_dir / "action_planning" / "action_plan.txt"
    program = test_dir / "lmp_generator" / "generated_program.py"
    if plan_txt.is_file() or program.is_file():
        plan_lines = plan_txt.read_text().strip().splitlines() if plan_txt.is_file() else ["(not saved)"]
        code = [line for line in (program.read_text().splitlines() if program.is_file() else ["(not saved)"])
                if not line.startswith("objects =")]
        frame = np.hstack([_text_panel(half, height, "3. Action plan (task planner)", plan_lines, 0.7),
                           _text_panel(width - half, height, "4. Generated program (LMP)", code, 0.75, mono=True)])
        writer.write(frame, repeat=hold(5.0))

    per_step, calls = _seedo_primitives(test_dir)
    overlay = cv2.imread(str(test_dir / "scene_perceiver" / "raw_scene_overlay.png"))
    if overlay is not None:
        info = ["Objects detected in the robot's scene, matched to the plan:"]
        targets = [c for c in calls if c.startswith(("pick(", "placing("))]
        info += [f"  {c}" for c in targets] or ["  (no pick/place primitive found)"]
        noise = _read_json(test_dir / "scene_perceiver" / "camera_pose_noise.json")
        if noise and noise.get("level") != "baseline" and "translation_error_norm_mm" in noise:
            info += ["", f"Camera calibration noise ({noise.get('level')}):",
                     f"  translation error {noise['translation_error_norm_mm']:.1f} mm, "
                     f"rotation error {noise['rotation_error_norm_deg']:.2f} deg"]
        elif noise:
            info += ["", "Camera calibration: no noise (baseline)"]
        left = cv2.resize(overlay, (half, height))
        _label(left, "5. Runtime scene perception")
        frame = np.hstack([left, _text_panel(width - half, height, "Scene -> plan grounding", info, 0.6)])
        writer.write(frame, repeat=hold(4.0))

    repeat = max(1, round(SEEDO_FPS / fps))
    for t, frame in enumerate(frames):
        text = "6. Robot execution" + (f" - {per_step[t]}" if t < len(per_step) else "")
        _overlay_banner(frame, text, scale=0.7)
        writer.write(frame, repeat=repeat)

    outcome = _read_json(str(pkl_path)[:-len(".pkl")] + ".json")
    if outcome:
        yes = lambda k: "yes" if outcome.get(k) else "no"
        text = (f"7. Outcome: reached {yes('object_reached')}, picked {yes('object_picked')}, "
                f"placed {yes('object_placed')}")
        if outcome.get("program_status") not in (None, "completed"):
            text += f" | program {outcome.get('program_status')} at {outcome.get('failure_stage')}"
        final = frames[-1].copy()
        final[-60:] = 0
        _overlay_banner(final, text, scale=0.75)
        writer.write(final, repeat=hold(3.0))
    writer.close()
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=("text", "osvi", "seedo"), required=True)
    parser.add_argument("--root", required=True,
                        help="Folder with task_<id>/traj_<n>.pkl; for --mode seedo, any folder containing "
                             "SeeDo test dirs (<test>/rollouts/seedo_controller/...), searched recursively.")
    parser.add_argument("--out_dir", required=True)
    parser.add_argument("--config", default=None,
                        help="--mode text: yaml with tasks[<id>].prompt (VLA-JEPA static prompt runs).")
    parser.add_argument("--tasks", default=None, help="Comma-separated task ids to render (default: all).")
    parser.add_argument("--fps", type=float, default=4.0, help="Rollout steps per second (default 4).")
    parser.add_argument("--context_sec", type=float, default=0.5, help="--mode osvi: seconds per context frame.")
    parser.add_argument("--waypoint_sec", type=float, default=2.5, help="--mode osvi: seconds on the waypoint frame.")
    parser.add_argument("--demo_root", default="/home/asus-mivia/Desktop/dataset/pick_place/human_rgb_pick_place",
                        help="--mode seedo: human-demo dataset (task_<id>/traj<n>.pkl), used when a test did "
                             "not save its tracked demonstration video.")
    parser.add_argument("--demo_sec", type=float, default=5.0, help="--mode seedo and Cosmos runs: demonstration duration in seconds.")
    parser.add_argument("--savers-dir", default=None)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    _add_savers_to_path(args.savers_dir)
    prompts = {}
    if args.config:
        with open(args.config, "r", encoding="utf-8") as stream:
            cfg = yaml.safe_load(stream) or {}
        prompts = {str(k).zfill(2): v["prompt"] for k, v in (cfg.get("tasks") or {}).items()}

    wanted = {t.zfill(2) for t in args.tasks.split(",")} if args.tasks else None
    if args.mode == "seedo":
        pkls = sorted(glob.glob(os.path.join(args.root, "**", "rollouts", "seedo_controller", "*", "task_*",
                                             "traj_*.pkl"), recursive=True))
    else:
        pkls = sorted(glob.glob(os.path.join(args.root, "task_*", "traj_*.pkl")))
    done = 0
    for pkl_path in pkls:
        task_name = Path(pkl_path).parent.name
        if wanted is not None and task_name.split("_")[-1] not in wanted:
            continue
        if args.mode == "seedo":
            test_rel = os.path.relpath(pkl_path.split("/rollouts/")[0], args.root).replace(" ", "_")
            out_path = Path(args.out_dir) / f"{test_rel}_{task_name}_{Path(pkl_path).stem}.mp4"
        else:
            out_path = Path(args.out_dir) / task_name / (Path(pkl_path).stem + ".mp4")
        if out_path.exists() and not args.overwrite:
            continue
        out_path.parent.mkdir(parents=True, exist_ok=True)
        print(f"{pkl_path} -> {out_path}")
        if args.mode == "text":
            ok = render_text(pkl_path, out_path, args.fps, prompts, args.demo_sec)
        elif args.mode == "seedo":
            ok = render_seedo(pkl_path, out_path, args.fps, args.demo_root, args.demo_sec)
        else:
            ok = render_osvi(pkl_path, out_path, args.fps, args.context_sec, args.waypoint_sec)
        done += int(ok)
    print(f"Rendered {done} videos into {args.out_dir}")


if __name__ == "__main__":
    main()
