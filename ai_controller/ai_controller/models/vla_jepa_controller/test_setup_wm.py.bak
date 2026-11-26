#!/usr/bin/env python3
"""Read-only preflight for the WM-conditioned VLA-JEPA UR5e controller.

Requires real checkpoint and CUDA but NO ROS graph, robot, MoveIt or HTTP.
Runs 2 controller steps to check: new chunk WM conditioning at step 0,
then cached second action, with image history updated at both steps.

The original test_setup.py stays the BASELINE-only preflight.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import numpy as np
import yaml

from test_setup import (
    check_environment, check_huggingface_cache, check_cuda,
    check_lerobot_installation, configure_python_paths, require_file,
    require_directory, section,
)


def check_wm_checkpoint(config_path: Path) -> None:
    section("WM PREFLIGHT - config / checkpoint / processors")
    from safetensors import safe_open
    from lerobot.configs.policies import PreTrainedConfig

    runtime_cfg = yaml.safe_load(require_file(config_path, "WM YAML").read_text()) or {}
    if runtime_cfg.get("inference_mode") != "wm_conditioned":
        raise RuntimeError("Expected inference_mode: wm_conditioned")
    expected_runtime = {
        "device": "cuda", "seed": 1000,
        "front_camera_index": 0, "gripper_camera_index": 3,
        "input_color_order": "rgb", "dataset_action_scale": 0.05,
        "postprocessor_config_filename": "policy_postprocessor_ur5e.json",
        "gripper_open_position": 0.0, "gripper_closed_position": 255.0,
        "server_host": "127.0.0.1", "server_port": 8770,
    }
    for key, expected in expected_runtime.items():
        if runtime_cfg.get(key) != expected:
            raise RuntimeError(f"{key}: expected {expected!r}, got {runtime_cfg.get(key)!r}")
    if set(str(k) for k in runtime_cfg.get("tasks", {})) < {f"{i:02d}" for i in range(16)}:
        raise RuntimeError("Missing one or more baseline tasks 00..15")

    root = require_directory(Path(os.environ["VLA_JEPA_CHECKPOINT"]), "WM checkpoint")
    if Path(runtime_cfg["checkpoint_path"]).resolve() != root:
        raise RuntimeError("YAML checkpoint_path != VLA_JEPA_CHECKPOINT")
    for required in ("config.json", "model.safetensors", "policy_preprocessor.json",
                     "policy_postprocessor.json", "policy_postprocessor_ur5e.json"):
        require_file(root / required, required)
    cfg = PreTrainedConfig.from_pretrained(str(root))
    if cfg.type != "vla_jepa":
        raise RuntimeError(f"Unexpected policy type {cfg.type!r}")
    if (cfg.action_dim, cfg.chunk_size, cfg.n_action_steps) != (7,7,7):
        raise RuntimeError("Expected 7-dimensional actions, horizon and queue length=7")
    if (cfg.num_video_frames, cfg.num_world_model_views,
        cfg.num_action_tokens_per_timestep, cfg.num_embodied_action_tokens_per_instruction) != (8,2,8,32):
        raise RuntimeError("WM configuration inconsistent with stage-2 training")
    if cfg.jepa_tubelet_size != 2:
        raise RuntimeError("Expected V-JEPA2 tubelet_size=2")
    if set(cfg.input_features) != {
        "observation.images.exterior_1_left", "observation.images.exterior_2_left",
    }:
        raise RuntimeError("Unexpected model image features")
    if getattr(cfg, "clip_normalized_actions", None) is not False:
        raise RuntimeError("WM stage2 does not clip normalized actions")
    if not getattr(cfg, "enable_world_model", False):
        raise RuntimeError("Exported checkpoint config must retain the World Model")
    for field in ("pre_snap_gripper_action", "binarize_gripper_action", "use_relative_actions"):
        if bool(getattr(cfg, field, False)):
            raise RuntimeError(f"Expected {field}=false")

    with safe_open(root / "model.safetensors", framework="pt", device="cpu") as f:
        keys = f.keys()
        for prefix in ("model.qwen.", "model.action_model.",
                       "model.video_encoder.", "model.video_predictor."):
            if not any(key.startswith(prefix) for key in keys):
                raise RuntimeError(f"Missing trained checkpoint weight prefix {prefix}")

    pre_cfg = json.loads((root / "policy_preprocessor.json").read_text())
    rename_steps = [s for s in pre_cfg["steps"] if s.get("registry_name") == "rename_observations_processor"]
    if len(rename_steps) != 1 or rename_steps[0]["config"]["rename_map"] != {
        "observation.images.front": "observation.images.exterior_1_left",
        "observation.images.gripper": "observation.images.exterior_2_left",
    }:
        raise RuntimeError("Mismatch in camera mapping from training")

    post_cfg = json.loads((root / "policy_postprocessor_ur5e.json").read_text())
    post_names = [step.get("registry_name") for step in post_cfg["steps"]]
    if post_names != ["unnormalizer_processor", "absolute_actions_processor", "device_processor"]:
        raise RuntimeError(f"Unexpected UR5e postprocessor steps: {post_names}")
    if post_cfg["steps"][1].get("config", {}).get("enabled") is not False:
        raise RuntimeError("Absolute action conversion must stay disabled")
    for name, cfg_json in (("policy_preprocessor.json", pre_cfg),
                           ("policy_postprocessor_ur5e.json", post_cfg)):
        for step in cfg_json["steps"]:
            if step.get("state_file"):
                require_file(root / step["state_file"], f"{name} processor state")
    unnormalizer = post_cfg["steps"][0]["state_file"]
    with safe_open(root / unnormalizer, framework="pt", device="cpu") as f:
        lo = f.get_tensor("action.min").float().numpy()
        hi = f.get_tensor("action.max").float().numpy()
    if lo.shape != (7,) or hi.shape != (7,):
        raise RuntimeError("Action min/max must both be 7D")
    if not np.isclose(lo[6], 0.0, atol=1e-5) or not np.isclose(hi[6], 20.0, atol=1e-3):
        raise RuntimeError("UR5e gripper normalization must be 0..20")

    print("Checkpoint and processor checks: PASS", flush=True)
    print("  source:", root, flush=True)
    print("  predictor:", cfg.jepa_encoder_name, flush=True)
    print("  action steps:", cfg.n_action_steps, flush=True)
    print("  flow integration steps:", cfg.num_inference_timesteps, flush=True)
    print("  WM conditioning: 32 + 256 tokens", flush=True)


def check_full_wm_inference(config_path: Path) -> None:
    section("WM PREFLIGHT - real model / dummy action, NO robot")
    from ai_controller.models.vla_jepa_controller.vla_jepa_controller import VLAJEPAController
    controller = VLAJEPAController(str(config_path), task_name="pick_place")
    controller.reset()
    controller.load_command("", task_id="00")
    base_images = [np.zeros((376,672,3), dtype=np.uint8) for _ in range(4)]
    # A fixed, valid robot reference pose (never sent to MoveIt).
    state = np.array([-0.155520946, 0.348699940, 0.153280345,
                      0.999445204, 0.031616514, 0.002143805, 0.010251021, 0.0])
    runtime = controller._runtime
    if runtime._previous_views is not None:
        raise RuntimeError("Video history not empty after reset")

    action0 = np.asarray(controller.inference([base_images, state], t=0, save_path=None))
    if action0.shape != (1,8) or not np.isfinite(action0).all():
        raise RuntimeError(f"Invalid dummy action 0: {action0}")
    if controller.fixed_orientation_xyzw is not None and not np.allclose(
        action0[0, 3:7], controller.fixed_orientation_xyzw, atol=1e-7, rtol=0.0
    ):
        raise RuntimeError("WM target did not apply fixed_orientation_xyzw")
    if runtime.policy._wm_chunks_generated != 1:
        raise RuntimeError("Expected a fresh WM prediction on first chunk")
    if runtime.policy._wm_last_video_shape != (1,2,2,3,224,224):
        raise RuntimeError("Incorrect input video shape")
    if runtime.policy._wm_last_conditioning_shape != (1,288,2048):
        raise RuntimeError("Incorrect Qwen + WM conditioning shape")
    if runtime._previous_views is None:
        raise RuntimeError("Video history not captured")

    changed_images = [np.full((376,672,3), 16, dtype=np.uint8) for _ in range(4)]
    action1 = np.asarray(controller.inference([changed_images, state], t=1, save_path=None))
    if action1.shape != (1,8) or not np.isfinite(action1).all():
        raise RuntimeError(f"Invalid dummy action 1: {action1}")
    if controller.fixed_orientation_xyzw is not None and not np.allclose(
        action1[0, 3:7], controller.fixed_orientation_xyzw, atol=1e-7, rtol=0.0
    ):
        raise RuntimeError("WM cached action did not apply fixed_orientation_xyzw")
    if runtime.policy._wm_chunks_generated != 1:
        raise RuntimeError("Queue did not reuse cached action at second step")
    if not np.isclose(float(runtime._previous_views[0].mean()), 16 / 255, atol=1e-3):
        raise RuntimeError("Video history not updated on cached action")
    runtime.reset()
    if runtime._previous_views is not None or runtime.policy._wm_chunks_generated != 0:
        raise RuntimeError("WM runtime did not reset both queues and camera history")
    print("WM conditioned model loading, chunk #1, queue and temporal buffer: PASS")
    print("No ROS/MoveIt motion was requested.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--task-name", default="pick_place")
    args = parser.parse_args()
    config_path = Path(args.config).expanduser().resolve()
    configure_python_paths()
    check_environment()
    check_huggingface_cache()
    check_cuda()
    check_lerobot_installation()
    check_wm_checkpoint(config_path)
    check_full_wm_inference(config_path)
    print("FULL WM-CONDITIONED PREFLIGHT PASSED", flush=True)


if __name__ == "__main__":
    main()
