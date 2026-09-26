#!/usr/bin/env python3
"""VLA-JEPA + Cosmos HTTP bridge server.

Why this exists: lerobot/transformers/scipy require numpy>=2, whose ABI is
incompatible with cv_bridge's compiled C++ extension (built against numpy
1.x) that ai_controller_node.py depends on for all ROS image decoding -
importing both in the same Python interpreter crashes cv_bridge on any call,
even a same-encoding passthrough (verified empirically on this host). The
two stacks cannot share a process, so this runs the real, heavy
VLAJEPAController + CosmosCaptioner in a separate venv/process
(/opt/vla_jepa_venv, numpy>=2) and exposes them over local HTTP.
vla_jepa_client.py (imported by ai_controller_node.py, numpy<2 system
python) is the thin client counterpart with a matching call interface.

This mirrors the split-process pattern already used by the cluster's
Multi-Task-LFD/repo/VLA-Bench/robosuite_test eval harness
(models/lerobot_policy.py <-> lerobot_policy_server.py), just over a local
loopback instead of a Slurm-allocated node.

Run (inside the venv that has torch/lerobot/transformers installed, see
docs/ai_controller_models/video_captioning.md):
    /opt/vla_jepa_venv/bin/python3 server.py \\
        --config vla_jepa_config.yaml [--host 127.0.0.1] [--port 8770]
"""
import argparse
import base64
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from flask import Flask, jsonify, request

THIS_DIR = Path(__file__).resolve().parent


def _add_repo_to_path():
    # ai_controller.models.vla_jepa_controller.* and
    # ai_controller.models.video_captioning.* both need the OUTER
    # ai_controller/ ROS package dir on sys.path (ROS2 ament_python's
    # package_name/package_name layout - see run_inference_waypoints_overlay.py
    # for the same fix applied to osvi_awda_controller's debug tooling).
    repo_ai_controller_dir = THIS_DIR.parents[1]  # .../ai_controller/ai_controller -> .../ai_controller
    outer = repo_ai_controller_dir.parent
    if str(outer) not in sys.path:
        sys.path.insert(0, str(outer))
    if str(THIS_DIR) not in sys.path:
        sys.path.insert(0, str(THIS_DIR))  # for vla_jepa.py/vla_jepa_utils.py's own bare imports


_add_repo_to_path()

from ai_controller.models.vla_jepa_controller.vla_jepa_controller import VLAJEPAController  # noqa: E402


def _decode_images(images_b64):
    images = []
    for b64 in images_b64:
        raw = base64.b64decode(b64)
        arr = np.frombuffer(raw, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        images.append(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    return images


def create_app(config_path: str, task_name: str = "pick_place"):
    app = Flask(__name__)

    print(f"[vla_jepa_server] Loading VLAJEPAController from {config_path} ...")
    controller = VLAJEPAController(config_path, task_name=task_name)
    controller.reset()
    print("[vla_jepa_server] Controller ready.")

    state = {"cosmos_captioner": None}

    @app.route("/health", methods=["GET"])
    def health():
        return jsonify({"status": "ok"})

    @app.route("/reset", methods=["POST"])
    def reset():
        controller.reset()
        return jsonify({"ok": True})

    @app.route("/load_command", methods=["POST"])
    def load_command():
        body = request.get_json(force=True)
        controller.load_command(
            body["demo_path"], body["task_id"],
            save_demo_frames=bool(body.get("save_demo_frames", False)),
            traj_cnt=int(body.get("traj_cnt", 0)),
            save_path=body.get("save_path"),
        )
        return jsonify({"command": controller.command})

    @app.route("/caption_task", methods=["POST"])
    def caption_task():
        from ai_controller.models.video_captioning.cosmos_captioner import (
            CosmosCaptioner, render_demo_clip)

        body = request.get_json(force=True)
        demo_path, task_id = body["demo_path"], body["task_id"]

        if state["cosmos_captioner"] is None:
            print("[vla_jepa_server] Loading Cosmos-Reason2 for task captioning...")
            state["cosmos_captioner"] = CosmosCaptioner()

        clip_path, demo_file = render_demo_clip(
            demo_path, task_id, f"/tmp/cosmos_captioner/task_{task_id}_demo.mp4")
        print(f"[vla_jepa_server] Rendered Cosmos input clip from {demo_file}: {clip_path}")
        caption = state["cosmos_captioner"].caption_video(clip_path)
        print(f"[vla_jepa_server] Cosmos task description: {caption!r}")
        controller.command = caption
        return jsonify({"command": caption, "demo_file": str(demo_file)})

    @app.route("/inference", methods=["POST"])
    def inference():
        body = request.get_json(force=True)
        images = _decode_images(body["images"])
        robot_state = np.asarray(body["robot_state"], dtype=np.float64)
        t = int(body.get("t", 0))

        out = controller.inference([images, robot_state], t=t)
        actions = [list(map(float, np.asarray(action, dtype=np.float64))) for action in out]
        return jsonify({"actions": actions})

    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(THIS_DIR / "vla_jepa_config.yaml"))
    parser.add_argument("--task-name", default="pick_place")
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=None)
    args = parser.parse_args()

    with open(args.config, "r", encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream) or {}
    host = args.host or cfg.get("server_host", "127.0.0.1")
    port = args.port or int(cfg.get("server_port", 8770))

    app = create_app(args.config, task_name=args.task_name)
    print(f"[vla_jepa_server] Listening on {host}:{port}")
    app.run(host=host, port=port, threaded=False)


if __name__ == "__main__":
    main()
