#!/usr/bin/env python3
"""ROS-agnostic Cosmos-Reason2 task-instruction captioner.

Reproduces, for the real robot, the same pattern used in
Multi-Task-LFD/repo/VLA-Bench/robosuite_test/run_robosuite_eval.py +
vllm_utils.py: Cosmos captions a pre-recorded human-demonstration video
*once per episode/task* to produce a natural-language task instruction,
which then stays fixed for the whole episode. That script drives an
external vLLM server via a CLI subprocess; here Cosmos runs in-process via
`transformers` (same approach already validated on this GB10/DGX Spark in
cosmos-reason2/scripts/inference_sample.py - see docs/ai_controller_models/
video_captioning.md for the environment fix that makes that possible).

No ROS imports here - the caller (ai_controller_node.py) is responsible for
constructing a `CosmosCaptioner` once and calling `caption_video()`/
`render_demo_clip()` at task start, mirroring how it already owns
load_command()/demo_path for the other controllers.
"""
import glob
import importlib
import json
import os
import pickle
import sys
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import torch
import yaml
from PIL import Image

THIS_DIR = Path(__file__).resolve().parent
DEFAULT_PROMPT_YAML = (
    THIS_DIR / "Video-Captioning-Human-Demo" / "prompts" / "generalist_task_description.yaml"
)
DEFAULT_MODEL_NAME = "nvidia/Cosmos-Reason2-8B"
PIXELS_PER_TOKEN = 32 ** 2
_DEMO_RNG = np.random.default_rng()  # OS-entropy seeded, independent of np.random.seed

# Debug windows (see render_demo_clip/CosmosCaptioner.caption_video): shown
# on whatever DISPLAY the process inherits (the ur_robotiq_teleoperation
# container is already run with -e DISPLAY + the X11 socket mounted, same as
# osvi_awda_controller's waypoint-overlay debug window). Failures (headless
# run, no X server) are caught and logged once, matching that same pattern,
# rather than raising.
INPUT_VIDEO_WINDOW_NAME = "Cosmos: input video"
RESULT_WINDOW_NAME = "Cosmos: computed prompt"


class TrajectoryUnpickler(pickle.Unpickler):
    """Same remap used by cod_controller.py/osvi_awda debug tools: legacy
    demo pkls reference multi_task_il.Trajectory, which is actually
    dataset_collector_pkg's savers.Trajectory."""

    def find_class(self, module, name):
        if module.startswith("multi_task_il") and name == "Trajectory":
            return importlib.import_module("savers").Trajectory
        return super().find_class(module, name)


def _add_savers_to_path(savers_dir=None):
    if savers_dir is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            savers_dir = str(Path(get_package_share_directory("dataset_collector_pkg")) / "scripts")
        except Exception:
            savers_dir = "/home/ros2_ws/src/dataset_collector/dataset_collector_pkg/scripts"
    if savers_dir not in sys.path:
        sys.path.insert(0, savers_dir)
    importlib.import_module("savers")


def _pad_frames_to_square(frames_bgr):
    """Centers each frame in a black square canvas sized to max(h, w) before
    Cosmos sees it - same preprocessing step as VLA-Bench's
    Multi-Task-LFD/repo/VLA-Bench/robosuite_test/vllm_utils.py::
    pad_video_to_square (there: a torchvision [T,H,W,C] tensor op; here:
    equivalent numpy op on the same BGR frame list _encode_h264 consumes).
    Non-square front-camera frames would otherwise get their aspect ratio
    distorted by the vision processor's own resize-to-token-budget step."""
    if not frames_bgr:
        return frames_bgr
    h, w = frames_bgr[0].shape[:2]
    side = max(h, w)
    if side == h == w:
        return frames_bgr
    pad_h, pad_w = (side - h) // 2, (side - w) // 2
    padded = []
    for frame in frames_bgr:
        canvas = np.zeros((side, side, 3), dtype=frame.dtype)
        canvas[pad_h:pad_h + h, pad_w:pad_w + w] = frame
        padded.append(canvas)
    return padded


def render_demo_clip(demo_path, task_id, out_path, fps=10, savers_dir=None, max_frames=None,
                      show_window=True):
    """Render an mp4 from one human-demo trajectory's camera_front_image
    frames, for Cosmos to caption. Mirrors cod_controller.load_command's demo
    file selection (first *.pkl found for the task, matching this repo's
    existing convention) and select_random_frames's frame decoding: demo
    frames are stored as JPEG bytes whose cv2.imdecode(..., IMREAD_COLOR)
    output is - empirically verified against human_rgb_pick_place/task_10 -
    already correct RGB channel order (the encode side's own cv2.imencode
    call baked in the BGR<->RGB swap already), NOT standard OpenCV BGR. So:
    no cv2.cvtColor call here, only a channel reversal to feed ffmpeg's
    bgr24 pixel format. Frames are then square-padded (_pad_frames_to_square)
    before encoding, matching VLA-Bench's own preprocessing.

    If show_window is True, plays the frames back live in a cv2 window
    (INPUT_VIDEO_WINDOW_NAME) as they're decoded - i.e. while the video is
    being loaded/prepared for Cosmos. The window is left open (on the last
    frame) when this returns; CosmosCaptioner.caption_video() closes it once
    inference is done and replaces it with the first/last-frame+prompt
    result window.
    """
    from ai_controller.utils.generate_rollout_videos import _encode_h264

    task_folder = str(task_id)
    if not task_folder.startswith("task_"):
        task_folder = f"task_{task_folder.zfill(2)}"
    demo_files = sorted(glob.glob(os.path.join(demo_path, task_folder, "*.pkl")))
    if not demo_files:
        raise FileNotFoundError(f"No demo .pkl files found in {os.path.join(demo_path, task_folder)}")

    _add_savers_to_path(savers_dir)
    # own RNG: VLAJEPAController.load_command() calls seed_everything() (i.e.
    # np.random.seed) right before every /caption_task, which would make a
    # global np.random draw pick the same demo every episode
    demo_indx = int(_DEMO_RNG.integers(len(demo_files)))
    with open(demo_files[demo_indx], "rb") as stream:
        payload = TrajectoryUnpickler(stream).load()
    traj = payload["traj"]

    n_steps = traj.T if max_frames is None else min(traj.T, max_frames)
    wait_ms = max(1, int(1000 / fps))
    frames_bgr = []
    for t in range(n_steps):
        raw = traj.get(t)["obs"]["camera_front_image"]
        if isinstance(raw, np.ndarray) and raw.ndim == 1:
            rgb = cv2.imdecode(raw, cv2.IMREAD_COLOR)
        else:
            rgb = np.asarray(raw)
        bgr = rgb[:, :, ::-1]  # RGB -> BGR for _encode_h264 / cv2.imshow
        frames_bgr.append(bgr)

        if show_window:
            try:
                cv2.imshow(INPUT_VIDEO_WINDOW_NAME, bgr)
                cv2.waitKey(wait_ms)
            except cv2.error as exc:
                print(f"[cosmos_captioner] Input video preview window disabled: {exc}")
                show_window = False

    frames_bgr = _pad_frames_to_square(frames_bgr)

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    _encode_h264(frames_bgr, out_path, fps)
    return out_path, demo_files[demo_indx]


def _wrap_text(text, font, font_scale, thickness, max_width_px):
    """Greedy word-wrap so cv2.putText (which doesn't wrap on its own) fits
    within max_width_px."""
    words = text.split()
    lines, current = [], ""
    for word in words:
        candidate = f"{current} {word}".strip()
        (w, _h), _ = cv2.getTextSize(candidate, font, font_scale, thickness)
        if w > max_width_px and current:
            lines.append(current)
            current = word
        else:
            current = candidate
    if current:
        lines.append(current)
    return lines


def _build_caption_result_image(first_frame_bgr, last_frame_bgr, caption):
    """first frame | last frame, with the computed prompt overlaid in a
    banner underneath - the "before/after" pair Cosmos's caption is meant to
    describe, next to the text it actually produced."""
    height = max(first_frame_bgr.shape[0], last_frame_bgr.shape[0])

    def _pad_to_height(frame):
        if frame.shape[0] == height:
            return frame
        pad = np.zeros((height - frame.shape[0], frame.shape[1], 3), dtype=np.uint8)
        return np.vstack([frame, pad])

    first_frame_bgr = _pad_to_height(first_frame_bgr)
    last_frame_bgr = _pad_to_height(last_frame_bgr)
    combined = np.hstack([first_frame_bgr, last_frame_bgr])

    font = cv2.FONT_HERSHEY_SIMPLEX
    label_scale = 0.7
    cv2.putText(combined, "first frame", (12, 28), font, label_scale, (0, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(combined, "last frame", (first_frame_bgr.shape[1] + 12, 28), font, label_scale,
                (0, 255, 255), 2, cv2.LINE_AA)

    margin = 14
    font_scale = max(0.6, combined.shape[1] / 1400.0)
    thickness = max(1, int(round(font_scale * 2)))
    lines = _wrap_text(caption, font, font_scale, thickness, combined.shape[1] - 2 * margin)
    line_height = int(cv2.getTextSize("Ag", font, font_scale, thickness)[0][1] * 1.9)
    banner_height = margin * 2 + line_height * max(1, len(lines))

    banner = np.zeros((banner_height, combined.shape[1], 3), dtype=np.uint8)
    for i, line in enumerate(lines):
        y = margin + line_height * (i + 1) - line_height // 3
        cv2.putText(banner, line, (margin, y), font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)

    return np.vstack([combined, banner])


def _build_result_image_from_video(video_path, caption):
    cap = cv2.VideoCapture(str(video_path))
    ok_first, first_frame = cap.read()
    last_frame = first_frame
    while True:
        ok_next, frame = cap.read()
        if not ok_next:
            break
        last_frame = frame
    cap.release()
    if not ok_first:
        print(f"[cosmos_captioner] Could not read frames back from {video_path} for the result image")
        return None
    return _build_caption_result_image(first_frame, last_frame, caption)


def _postprocess_caption(caption):
    """Same normalization Multi-Task-LFD/repo/VLA-Bench/robosuite_test/
    vllm_utils.py::run_vllm_server applies to Cosmos's raw output before
    using it as a task instruction: digits -> ordinal words (VLA-JEPA's
    vla_jepa_config.yaml tasks: prompts spell out "first"/"second"/...
    rather than "1"/"2"), "three"/"four" -> "third"/"fourth", a redundant
    "compartment box" -> "box", and stripped trailing periods. Exact same
    order of replacements as the original."""
    if "1" in caption:
        caption = caption.replace("1", "first")
    if "2" in caption:
        caption = caption.replace("2", "second")
    if "3" in caption:
        caption = caption.replace("3", "third")
    if "4" in caption:
        caption = caption.replace("4", "fourth")
    if "four" in caption and "fourth" not in caption:
        caption = caption.replace("four", "fourth")
    if "three" in caption:
        caption = caption.replace("three", "third")
    if "compartment box" in caption:
        caption = caption.replace("compartment box", "box")
    caption = caption.replace(".", "")
    return caption


class CosmosCaptioner:
    """Lazily loads nvidia/Cosmos-Reason2-2B once and reuses it across
    tasks/episodes - loading is the expensive part (a few GB, one-shot),
    per-call captioning is cheap in comparison."""

    def __init__(self, model_name: str = DEFAULT_MODEL_NAME, device_map: str = "auto"):
        self.model_name = model_name
        self.device_map = device_map
        self.model = None
        self.processor = None
        self._display_failed = False

    def _ensure_loaded(self):
        if self.model is not None:
            return
        import transformers

        transformers.set_seed(0)
        self.model = transformers.Qwen3VLForConditionalGeneration.from_pretrained(
            self.model_name, dtype=torch.bfloat16, device_map=self.device_map, attn_implementation="sdpa"
        )
        self.processor = transformers.Qwen3VLProcessor.from_pretrained(self.model_name)
        min_vision_tokens, max_vision_tokens = 256, 8192
        size = {
            "shortest_edge": min_vision_tokens * PIXELS_PER_TOKEN,
            "longest_edge": max_vision_tokens * PIXELS_PER_TOKEN,
        }
        self.processor.image_processor.size = size
        self.processor.video_processor.size = size

    def warmup(self, num_frames: int = 8, side: int = 256):
        """Loads the weights and runs one throwaway generation on a tiny black
        clip, so CUDA context/kernel selection/allocator setup happen here
        instead of inside the first real caption request."""
        from ai_controller.utils.generate_rollout_videos import _encode_h264

        self._ensure_loaded()
        clip = Path("/tmp/cosmos_captioner/warmup.mp4")
        clip.parent.mkdir(parents=True, exist_ok=True)
        _encode_h264([np.zeros((side, side, 3), dtype=np.uint8)] * num_frames, clip, 4)
        self.caption_video(clip, max_new_tokens=4, show_result_window=False)

    def caption_video(
        self,
        video_path,
        prompt_yaml: Optional[str] = None,
        fps: int = 2,
        max_new_tokens: int = 64,
        temperature: float = 0.7,
        top_p: float = 0.8,
        top_k: int = 20,
        repetition_penalty: float = 1.0,
        show_result_window: bool = True,
        save_dir=None,
        extra_metadata: Optional[dict] = None,
    ) -> str:
        self._ensure_loaded()

        prompt_yaml = Path(prompt_yaml) if prompt_yaml else DEFAULT_PROMPT_YAML
        if not prompt_yaml.is_absolute():
            prompt_yaml = THIS_DIR / prompt_yaml  # e.g. vla_jepa_config.yaml's cosmos_prompt_yaml
        with open(prompt_yaml, "r", encoding="utf-8") as stream:
            prompt = yaml.safe_load(stream)
        
        
        conversation = [
            {"role": "system", "content": [{"type": "text", "text": prompt["system_prompt"]}]},
            {
                "role": "user",
                "content": [
                    {"type": "video", "video": str(video_path)},
                    {"type": "text", "text": prompt["user_prompt"]},
                ],
            },
        ]

        inputs = self.processor.apply_chat_template(
            conversation,
            tokenize=True,
            add_generation_prompt=True,
            return_dict=True,
            return_tensors="pt",
            fps=fps,
        )
        inputs = inputs.to(self.model.device)

        with torch.no_grad():
            # Sampling params match VLA-Bench's actual cosmos-reason2-inference
            # CLI invocation (--no-reasoning => cosmos_reason2_utils.script.
            # inference.SamplingOverrides.get_defaults(reasoning=False)):
            # top_p=0.8, top_k=20, repetition_penalty=1.0, temperature=0.7.
            # That defaults set also has presence_penalty=1.5, which
            # transformers.generate() has no equivalent for - omitted.
            generated_ids = self.model.generate(
                **inputs,
                max_new_tokens=max_new_tokens,
                do_sample=True,
                temperature=temperature,
                top_p=top_p,
                top_k=top_k,
                repetition_penalty=repetition_penalty,
            )
        generated_ids_trimmed = [
            out_ids[len(in_ids):] for in_ids, out_ids in zip(inputs.input_ids, generated_ids, strict=False)
        ]
        output_text = self.processor.batch_decode(
            generated_ids_trimmed, skip_special_tokens=True, clean_up_tokenization_spaces=False
        )
        raw_caption = output_text[0].strip()
        caption = _postprocess_caption(raw_caption)

        result_image = None
        if show_result_window or save_dir is not None:
            result_image = _build_result_image_from_video(video_path, caption)

        if save_dir is not None:
            save_dir = Path(save_dir)
            save_dir.mkdir(parents=True, exist_ok=True)
            if result_image is not None:
                cv2.imwrite(str(save_dir / "caption_result.png"), result_image)
            metadata = {
                "caption": caption,
                "raw_caption": raw_caption,
                "video_path": str(video_path),
                "prompt_yaml": str(prompt_yaml),
                "system_prompt": prompt["system_prompt"],
                "user_prompt": prompt["user_prompt"],
                "model_name": self.model_name,
                "fps": fps,
                "num_input_tokens": int(inputs.input_ids.shape[1]),
                "max_new_tokens": max_new_tokens,
                "temperature": temperature,
                "top_p": top_p,
                "top_k": top_k,
                "repetition_penalty": repetition_penalty,
                **(extra_metadata or {}),
            }
            with open(save_dir / "caption.json", "w", encoding="utf-8") as stream:
                json.dump(metadata, stream, indent=2)

        if show_result_window and result_image is not None:
            self._show_result_window(result_image)
        return caption

    def _show_result_window(self, result_image):
        """Closes the input-video preview window (see render_demo_clip) and
        opens a new one with the first/last frame + computed prompt image,
        once inference is done."""
        if self._display_failed:
            return
        try:
            try:
                cv2.destroyWindow(INPUT_VIDEO_WINDOW_NAME)
            except cv2.error:
                pass  # window was never shown (e.g. render_demo_clip(show_window=False))

            cv2.namedWindow(RESULT_WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(RESULT_WINDOW_NAME, result_image.shape[1], result_image.shape[0])
            cv2.imshow(RESULT_WINDOW_NAME, result_image)
            cv2.waitKey(1)
        except cv2.error as exc:
            print(f"[cosmos_captioner] Result window disabled: {exc}")
            self._display_failed = True
