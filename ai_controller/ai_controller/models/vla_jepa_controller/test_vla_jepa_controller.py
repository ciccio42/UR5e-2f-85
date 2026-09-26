#!/usr/bin/env python3
"""Unit test: VLAJEPAController in isolation (no HTTP, no ROS).

Loads the real checkpoint and runs one inference step on synthetic images +
a fixed robot_state, asserting the output has the shape/dtype/ranges
ai_controller_node.py's vla_jepa_controller branch expects: a list containing
one 8-element float action [x, y, z, qx, qy, qz, qw, gripper], with the
quaternion normalized and the position close to the input eef_pos (a single
inference step should only move a small amount).

Run inside the venv that has torch/lerobot installed (see
docs/ai_controller_models/video_captioning.md):
    /opt/vla_jepa_venv/bin/python3 test_vla_jepa_controller.py \\
        [--config vla_jepa_config.yaml] [--task-id 10] \\
        [--demo-path /dataset/pick_place/human_rgb_pick_place]
"""
import argparse
import sys
from pathlib import Path

import numpy as np

THIS_DIR = Path(__file__).resolve().parent


def _add_repo_to_path():
    # THIS_DIR = .../ai_controller/ai_controller/models/vla_jepa_controller
    # parents[2] = the OUTER ai_controller/ ROS package dir (contains the
    # importable "ai_controller" python package, ROS2's package_name/
    # package_name layout - same fix as run_inference_waypoints_overlay.py).
    outer = THIS_DIR.parents[2]
    if str(outer) not in sys.path:
        sys.path.insert(0, str(outer))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=str(THIS_DIR / "vla_jepa_config.yaml"))
    parser.add_argument("--task-id", default="10")
    parser.add_argument("--demo-path", default="/dataset/pick_place/human_rgb_pick_place")
    args = parser.parse_args()

    _add_repo_to_path()
    from ai_controller.models.vla_jepa_controller.vla_jepa_controller import VLAJEPAController

    print(f"[TEST] Loading VLAJEPAController from {args.config} ...")
    controller = VLAJEPAController(args.config, task_name="pick_place")
    controller.reset()

    print(f"[TEST] load_command(task_id={args.task_id!r}) ...")
    controller.load_command(args.demo_path, args.task_id, save_demo_frames=False)
    assert isinstance(controller.command, str) and len(controller.command) > 0, (
        f"Expected a non-empty static prompt string, got {controller.command!r}")
    print(f"[TEST] command = {controller.command!r}")

    rng = np.random.default_rng(0)
    front = rng.integers(0, 255, (376, 672, 3), dtype=np.uint8)
    left = rng.integers(0, 255, (376, 672, 3), dtype=np.uint8)
    right = rng.integers(0, 255, (376, 672, 3), dtype=np.uint8)
    gripper = rng.integers(0, 255, (376, 672, 3), dtype=np.uint8)
    images = [front, left, right, gripper]
    eef_pos = np.array([0.0, 0.35, 0.20])
    robot_state = np.concatenate([eef_pos, [0.0, 0.0, 0.0, 1.0], [0.0]])  # xyz, qxyzw, gripper_closed

    print("[TEST] Running inference([images, robot_state], t=0) ...")
    out = controller.inference([images, robot_state], t=0)

    assert isinstance(out, list) and len(out) == 1, f"Expected a 1-element list of actions, got {out!r}"
    action = np.asarray(out[0], dtype=np.float64)
    assert action.shape == (8,), f"Expected an 8-element action, got shape {action.shape}"

    position, quat, gripper_cmd = action[:3], action[3:7], action[7]
    quat_norm = float(np.linalg.norm(quat))
    assert abs(quat_norm - 1.0) < 1e-3, f"Quaternion not normalized: norm={quat_norm}"
    position_delta = float(np.linalg.norm(position - eef_pos))
    assert position_delta < 0.2, (
        f"A single inference step moved {position_delta:.4f} m from the reference pose - "
        "expected a small correction, this looks like a units/frame bug")
    assert gripper_cmd in (0.0, 1.0) or 0.0 <= gripper_cmd <= 1.0, f"Unexpected gripper value: {gripper_cmd}"

    print(f"[TEST] action = {action.tolist()}")
    print(f"[TEST] position delta from reference pose: {position_delta:.4f} m, quaternion norm: {quat_norm:.6f}")
    print("[PASS] VLAJEPAController loads and produces a well-formed action.")


if __name__ == "__main__":
    main()
