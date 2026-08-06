#!/usr/bin/env python3
"""Analyze frame-count distribution for preprocessed UR5e mp4 videos."""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path

import cv2


DEFAULT_VIDEO_DIR = Path(__file__).resolve().parents[1] / "processed_data" / "ur5e_pick_place_video" / "video"


def selected_videos(video_dir: Path, camera_suffix: str) -> list[Path]:
    # Gli mp4 finiscono con 0 per front camera e 1 per gripper camera.
    videos = sorted(video_dir.glob("*.mp4"))
    if camera_suffix == "all":
        return videos
    return [video for video in videos if video.stem.endswith(camera_suffix)]


def count_frames(video_path: Path, decode: bool) -> tuple[int, float]:
    # CAP_PROP_FRAME_COUNT e' veloce; decode=True conta davvero i frame leggendo il video.
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise ValueError(f"Cannot open video: {video_path}")

    fps = capture.get(cv2.CAP_PROP_FPS)
    if not decode:
        frames = int(round(capture.get(cv2.CAP_PROP_FRAME_COUNT)))
        capture.release()
        return frames, fps

    frames = 0
    while True:
        ok, _frame = capture.read()
        if not ok:
            break
        frames += 1
    capture.release()
    return frames, fps


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--video-dir", type=Path, default=DEFAULT_VIDEO_DIR)
    parser.add_argument("--camera-suffix", choices=["0", "1", "all"], default="0")
    parser.add_argument("--decode", action="store_true")
    parser.add_argument("--list-shorter-than", type=int)
    args = parser.parse_args()

    videos = selected_videos(args.video_dir, args.camera_suffix)
    if not videos:
        raise FileNotFoundError(f"No mp4 videos found in {args.video_dir}")

    frame_counts: list[tuple[Path, int, float]] = []
    for video in videos:
        frames, fps = count_frames(video, args.decode)
        frame_counts.append((video, frames, fps))

    counts = Counter(frames for _video, frames, _fps in frame_counts)
    fps_counts = Counter(round(fps, 3) for _video, _frames, fps in frame_counts)
    values = [frames for _video, frames, _fps in frame_counts]

    print(f"video_dir: {args.video_dir}")
    print(f"camera_suffix: {args.camera_suffix}")
    print(f"videos: {len(frame_counts)}")
    print(f"min_frames: {min(values)}")
    print(f"max_frames: {max(values)}")
    print(f"avg_frames: {sum(values) / len(values):.2f}")

    print("\nframe_count_distribution:")
    for frames, num_videos in sorted(counts.items()):
        print(f"  {frames}: {num_videos}")

    print("\nfps_distribution:")
    for fps, num_videos in sorted(fps_counts.items()):
        print(f"  {fps}: {num_videos}")

    if args.list_shorter_than is not None:
        print(f"\nvideos_shorter_than_{args.list_shorter_than}:")
        for video, frames, _fps in frame_counts:
            if frames < args.list_shorter_than:
                print(f"  {video.name}: {frames}")


if __name__ == "__main__":
    main()
