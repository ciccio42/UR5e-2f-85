#!/usr/bin/env python3
"""Unit test: VLAJEPAControllerClient <-> server.py HTTP bridge.

Exercises the exact call surface ai_controller_node.py uses for
vla_jepa_controller: reset(), load_command() (static per-task prompt),
inference() (2-element input_data), and caption_task_with_cosmos() (the
use_cosmos_task_description:=true path). Requires server.py already running
(separate venv, see server.py's module docstring / docs/ai_controller_models/
video_captioning.md) - this test itself only needs requests/numpy/cv2, so it
runs fine in ai_controller_node.py's own numpy<2 system python.

Run:
    python3 test_vla_jepa_client_server.py [--config vla_jepa_config.yaml]
        [--task-id 10] [--demo-path /dataset/pick_place/human_rgb_pick_place]
        [--skip-cosmos]
"""
import argparse
from pathlib import Path

import numpy as np

THIS_DIR = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=str(THIS_DIR / "vla_jepa_config.yaml"))
    parser.add_argument("--task-id", default="10")
    parser.add_argument("--demo-path", default="/dataset/pick_place/human_rgb_pick_place")
    parser.add_argument("--skip-cosmos", action="store_true",
                         help="Skip the caption_task_with_cosmos check (it loads a second, "
                              "multi-GB model server-side and is slower)")
    args = parser.parse_args()

    from vla_jepa_client import VLAJEPAControllerClient

    print(f"[TEST] Connecting to vla_jepa server via {args.config} ...")
    client = VLAJEPAControllerClient(args.config)

    client.reset()
    assert client.command is None, f"Expected command to be cleared by reset(), got {client.command!r}"

    static_command = client.load_command(args.demo_path, args.task_id, save_demo_frames=False)
    assert isinstance(static_command, str) and len(static_command) > 0, (
        f"Expected a non-empty static prompt string, got {static_command!r}")
    assert client.command == static_command
    print(f"[TEST] static command (task {args.task_id}) = {static_command!r}")

    rng = np.random.default_rng(0)
    images = [rng.integers(0, 255, (376, 672, 3), dtype=np.uint8) for _ in range(4)]
    eef_pos = np.array([0.0, 0.35, 0.20])
    robot_state = np.concatenate([eef_pos, [0.0, 0.0, 0.0, 1.0], [0.0]])

    print("[TEST] inference([images, robot_state], t=0) ...")
    out = client.inference([images, robot_state], t=0)
    assert isinstance(out, list) and len(out) == 1, f"Expected a 1-element list of actions, got {out!r}"
    action = np.asarray(out[0], dtype=np.float64)
    assert action.shape == (8,), f"Expected an 8-element action, got shape {action.shape}"
    quat_norm = float(np.linalg.norm(action[3:7]))
    assert abs(quat_norm - 1.0) < 1e-3, f"Quaternion not normalized: norm={quat_norm}"
    print(f"[TEST] action = {action.tolist()}")

    if not args.skip_cosmos:
        print("[TEST] caption_task_with_cosmos(...) - this loads Cosmos-Reason2 server-side "
              "on first call, can take a while ...")
        cosmos_command = client.caption_task_with_cosmos(args.demo_path, args.task_id)
        assert isinstance(cosmos_command, str) and len(cosmos_command) > 0, (
            f"Expected a non-empty Cosmos caption, got {cosmos_command!r}")
        assert client.command == cosmos_command
        print(f"[TEST] Cosmos-generated command (task {args.task_id}) = {cosmos_command!r}")
        print(f"[TEST] (static prompt for comparison was: {static_command!r})")

    print("[PASS] VLAJEPAControllerClient <-> server.py round-trips correctly.")


if __name__ == "__main__":
    main()
