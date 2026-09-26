#!/usr/bin/env python3
"""Thin HTTP client counterpart to server.py's VLA-JEPA + Cosmos bridge.

Runs in ai_controller_node.py's own (numpy<2, ROS/cv_bridge) system python -
see server.py's module docstring for why the split is necessary. Exposes the
same call surface ai_controller_node.py already uses for every other
controller (reset/load_command/inference), plus caption_task_with_cosmos()
for the Cosmos-generated task-description path, so ai_controller_node.py
only needs a one-line class swap to use this instead of importing
VLAJEPAController (and its heavy torch/lerobot/transformers deps) directly.
"""
import base64
import time

import cv2
import numpy as np
import requests
import yaml


class VLAJEPARemoteUnavailable(RuntimeError):
    pass


def _encode_image_b64(image_rgb):
    bgr = cv2.cvtColor(np.asarray(image_rgb, dtype=np.uint8), cv2.COLOR_RGB2BGR)
    ok, buf = cv2.imencode(".png", bgr)
    if not ok:
        raise ValueError("cv2.imencode failed for image")
    return base64.b64encode(buf.tobytes()).decode("ascii")


class VLAJEPAControllerClient:
    """Same construction signature as every other AIController subclass here
    (model_config path, task_name), so ai_controller_node.py's existing
    `self.controller = VLAJEPAControllerClient(self.model_config_path,
    self.task_name)` line needs no special-casing."""

    def __init__(self, model_config, task_name: str = "pick_place",
                 connect_retries: int = 5, connect_retry_wait: float = 3.0):
        with open(model_config, "r", encoding="utf-8") as stream:
            cfg = yaml.safe_load(stream) or {}
        host = cfg.get("server_host", "127.0.0.1")
        port = int(cfg.get("server_port", 8770))
        self.base_url = f"http://{host}:{port}"
        self.task_name = task_name
        self.command = None

        last_exc = None
        for attempt in range(connect_retries):
            try:
                resp = requests.get(f"{self.base_url}/health", timeout=5.0)
                resp.raise_for_status()
                print(f"[VLAJEPAControllerClient] Connected to {self.base_url}")
                return
            except requests.RequestException as exc:
                last_exc = exc
                if attempt < connect_retries - 1:
                    time.sleep(connect_retry_wait)
        raise VLAJEPARemoteUnavailable(
            f"Could not reach vla_jepa server.py at {self.base_url} after "
            f"{connect_retries} attempts. Start it first (separate venv with "
            "torch/lerobot/transformers - see docs/ai_controller_models/"
            f"video_captioning.md): python3 server.py --config {model_config}"
        ) from last_exc

    def reset(self):
        requests.post(f"{self.base_url}/reset", timeout=15.0).raise_for_status()
        self.command = None

    def load_command(self, demo_path, task_id, **kwargs):
        resp = requests.post(
            f"{self.base_url}/load_command",
            json={
                "demo_path": demo_path,
                "task_id": task_id,
                "save_demo_frames": bool(kwargs.get("save_demo_frames", False)),
                "traj_cnt": int(kwargs.get("traj_cnt", 0)),
                "save_path": kwargs.get("save_path"),
            },
            timeout=30.0,
        )
        resp.raise_for_status()
        self.command = resp.json()["command"]
        return self.command

    def caption_task_with_cosmos(self, demo_path, task_id, timeout: float = 180.0):
        """Overrides the static per-task prompt (already set by load_command)
        with a Cosmos-Reason2 caption of this task's human demo, for the rest
        of the episode - see server.py's /caption_task handler."""
        resp = requests.post(
            f"{self.base_url}/caption_task",
            json={"demo_path": demo_path, "task_id": task_id},
            timeout=timeout,
        )
        resp.raise_for_status()
        self.command = resp.json()["command"]
        return self.command

    def inference(self, input_data, t: int = 0, save_path=None):
        images, robot_state = input_data[0], input_data[1]
        resp = requests.post(
            f"{self.base_url}/inference",
            json={
                "images": [_encode_image_b64(img) for img in images],
                "robot_state": np.asarray(robot_state, dtype=np.float64).tolist(),
                "t": int(t),
            },
            timeout=60.0,
        )
        resp.raise_for_status()
        return resp.json()["actions"]
