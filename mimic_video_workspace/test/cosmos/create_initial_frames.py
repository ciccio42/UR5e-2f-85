from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import imageio.v2 as iio
import safetensors.torch as st
import torch


WORKSPACE_DIR = Path(__file__).resolve().parents[2]
DEFAULT_DATASET_DIR = WORKSPACE_DIR / "processed_data" / "ur5e_pick_place_video" / "video"
OUTPUT_DIR = WORKSPACE_DIR / "test" / "cosmos" / "initial_frames"

def write_first_frames_clip(input_video: Path, output_video: Path, num_frames: int, fps: int) -> None:
    # Creo un mini-video con i primi frame reali della traiettoria.
    reader = iio.get_reader(str(input_video))
    frames = []
    try:
        for frame in reader:
            frames.append(frame)
            if len(frames) == num_frames:
                break
    finally:
        reader.close()

    if not frames:
        raise ValueError(f"Empty video: {input_video}")

    # Se il video fosse piu corto del richiesto, ripeto l'ultimo frame.
    while len(frames) < num_frames:
        frames.append(frames[-1])

    output_video.parent.mkdir(parents=True, exist_ok=True)
    with iio.get_writer(str(output_video), codec="libx264", fps=fps, quality=8) as writer:
        for frame in frames:
            writer.append_data(frame)

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-video", type=Path, required=True)
    parser.add_argument("--num-conditional-frames", type=int, default=5, choices=[1, 5])
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--name", type=str, required=True)
    return parser.parse_args()

def main():

    args = parse_args()
    video_ep = args.input_video

    video_path = DEFAULT_DATASET_DIR / video_ep

    output_video = OUTPUT_DIR / f"{args.name}.mp4"
    write_first_frames_clip(video_path, output_video, args.num_conditional_frames, args.fps)
    
if __name__ == "__main__":
    main()