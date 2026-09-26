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
DEFAULT_MODEL_NAME = "nvidia/Cosmos-Reason2-2B"
PIXELS_PER_TOKEN = 32 ** 2


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


def render_demo_clip(demo_path, task_id, out_path, fps=10, savers_dir=None, max_frames=None):
    """Render an mp4 from one human-demo trajectory's camera_front_image
    frames, for Cosmos to caption. Mirrors cod_controller.load_command's demo
    file selection (first *.pkl found for the task, matching this repo's
    existing convention) and select_random_frames's frame decoding: demo
    frames are stored as JPEG bytes whose cv2.imdecode(..., IMREAD_COLOR)
    output is - empirically verified against human_rgb_pick_place/task_10 -
    already correct RGB channel order (the encode side's own cv2.imencode
    call baked in the BGR<->RGB swap already), NOT standard OpenCV BGR. So:
    no cv2.cvtColor call here, only a channel reversal to feed ffmpeg's
    bgr24 pixel format.
    """
    from ai_controller.utils.generate_rollout_videos import _encode_h264

    task_folder = str(task_id)
    if not task_folder.startswith("task_"):
        task_folder = f"task_{task_folder.zfill(2)}"
    demo_files = sorted(glob.glob(os.path.join(demo_path, task_folder, "*.pkl")))
    if not demo_files:
        raise FileNotFoundError(f"No demo .pkl files found in {os.path.join(demo_path, task_folder)}")

    _add_savers_to_path(savers_dir)
    with open(demo_files[0], "rb") as stream:
        payload = TrajectoryUnpickler(stream).load()
    traj = payload["traj"]

    n_steps = traj.T if max_frames is None else min(traj.T, max_frames)
    frames_bgr = []
    for t in range(n_steps):
        raw = traj.get(t)["obs"]["camera_front_image"]
        if isinstance(raw, np.ndarray) and raw.ndim == 1:
            rgb = cv2.imdecode(raw, cv2.IMREAD_COLOR)
        else:
            rgb = np.asarray(raw)
        frames_bgr.append(rgb[:, :, ::-1])  # RGB -> BGR for _encode_h264

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    _encode_h264(frames_bgr, out_path, fps)
    return out_path, demo_files[0]


class CosmosCaptioner:
    """Lazily loads nvidia/Cosmos-Reason2-2B once and reuses it across
    tasks/episodes - loading is the expensive part (a few GB, one-shot),
    per-call captioning is cheap in comparison."""

    def __init__(self, model_name: str = DEFAULT_MODEL_NAME, device_map: str = "auto"):
        self.model_name = model_name
        self.device_map = device_map
        self.model = None
        self.processor = None

    def _ensure_loaded(self):
        if self.model is not None:
            return
        import transformers

        transformers.set_seed(0)
        self.model = transformers.Qwen3VLForConditionalGeneration.from_pretrained(
            self.model_name, dtype=torch.float16, device_map=self.device_map, attn_implementation="sdpa"
        )
        self.processor = transformers.Qwen3VLProcessor.from_pretrained(self.model_name)
        min_vision_tokens, max_vision_tokens = 256, 8192
        size = {
            "shortest_edge": min_vision_tokens * PIXELS_PER_TOKEN,
            "longest_edge": max_vision_tokens * PIXELS_PER_TOKEN,
        }
        self.processor.image_processor.size = size
        self.processor.video_processor.size = size

    def caption_video(
        self,
        video_path,
        prompt_yaml: Optional[str] = None,
        fps: int = 4,
        max_new_tokens: int = 64,
    ) -> str:
        self._ensure_loaded()

        prompt_yaml = Path(prompt_yaml) if prompt_yaml else DEFAULT_PROMPT_YAML
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
            generated_ids = self.model.generate(**inputs, max_new_tokens=max_new_tokens)
        generated_ids_trimmed = [
            out_ids[len(in_ids):] for in_ids, out_ids in zip(inputs.input_ids, generated_ids, strict=False)
        ]
        output_text = self.processor.batch_decode(
            generated_ids_trimmed, skip_special_tokens=True, clean_up_tokenization_spaces=False
        )
        return output_text[0].strip()
