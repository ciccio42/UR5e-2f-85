#!/usr/bin/env python3
"""Unit test: cosmos_captioner.py (render_demo_clip + CosmosCaptioner) in isolation.

Renders an mp4 from a real human-demo trajectory (no VLA-JEPA involved) and
captions it with Cosmos-Reason2, asserting the caption is a non-empty
sentence roughly matching the "Pick the [OBJECT] and place it into the
[TARGET]" template. Also asserts the rendered clip's colors are correct
(mean color of the wood-colored table region is warm/brown, not the
blue-tinted inversion this camera_front_image JPEG decoding path had before
the RGB-order fix - see render_demo_clip's docstring).

Run inside the venv that has torch/transformers installed (see
docs/ai_controller_models/video_captioning.md):
    /opt/vla_jepa_venv/bin/python3 test_cosmos_captioner.py \\
        [--demo-path /dataset/pick_place/human_rgb_pick_place] [--task-id 10]
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

THIS_DIR = Path(__file__).resolve().parent


def _add_repo_to_path():
    outer = THIS_DIR.parents[2]  # .../ai_controller/ai_controller/models -> .../ai_controller
    if str(outer) not in sys.path:
        sys.path.insert(0, str(outer))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--demo-path", default="/dataset/pick_place/human_rgb_pick_place")
    parser.add_argument("--task-id", default="10")
    parser.add_argument("--out-dir", default="/tmp/cosmos_captioner_test")
    args = parser.parse_args()

    _add_repo_to_path()
    from ai_controller.models.video_captioning.cosmos_captioner import CosmosCaptioner, render_demo_clip

    print(f"[TEST] Rendering demo clip for task {args.task_id} from {args.demo_path} ...")
    clip_path, demo_file = render_demo_clip(
        args.demo_path, args.task_id, Path(args.out_dir) / f"task_{args.task_id}_demo.mp4", fps=10)
    assert clip_path.is_file() and clip_path.stat().st_size > 0, f"No mp4 produced at {clip_path}"
    print(f"[TEST] Rendered {clip_path} from {demo_file}")

    cap = cv2.VideoCapture(str(clip_path))
    ok, frame_bgr = cap.read()
    cap.release()
    assert ok, f"Could not read back a frame from {clip_path}"
    # A correctly RGB-ordered wood-table scene should read warm (R>=B) on
    # average; the pre-fix bug (treating already-corrected JPEG bytes as
    # standard OpenCV BGR) produced a cold blue-tinted image (B>R) instead.
    mean_b, mean_g, mean_r = frame_bgr.reshape(-1, 3).mean(axis=0)
    assert mean_r >= mean_b, (
        f"Rendered clip looks blue-tinted (mean R={mean_r:.1f} < mean B={mean_b:.1f}) - "
        "likely an RGB/BGR inversion in render_demo_clip's frame decoding")
    print(f"[TEST] Frame 0 mean BGR = ({mean_b:.1f}, {mean_g:.1f}, {mean_r:.1f}) - looks warm/correct")

    print("[TEST] Loading Cosmos-Reason2 and captioning the clip (first call is slow) ...")
    captioner = CosmosCaptioner()
    caption = captioner.caption_video(clip_path)
    assert isinstance(caption, str) and len(caption) > 0, f"Expected a non-empty caption, got {caption!r}"
    print(f"[TEST] caption = {caption!r}")

    lowered = caption.lower()
    assert "pick" in lowered or "place" in lowered, (
        f"Caption doesn't look like a pick-place instruction: {caption!r}")

    print("[PASS] render_demo_clip + CosmosCaptioner.caption_video produce a correct, sane caption.")


if __name__ == "__main__":
    main()
