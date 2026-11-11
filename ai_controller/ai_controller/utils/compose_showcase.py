#!/usr/bin/env python3
"""Concatenate per-trajectory videos (from render_rollout_videos.py) into one showcase video.

The spec is a JSON list of sections, shown in order:
    [{"title": "SeeDo", "description": "...",
      "examples": [{"video": "seedo/....mp4", "caption": "green cube -> first bin"}, ...]}, ...]
Video paths are relative to the spec file. Each section starts with a title card, each example
with a short caption card; every clip is resampled to --fps without changing its playback speed.

Usage:
    python3 compose_showcase.py --spec rollout_videos/showcase_spec.json --out rollout_videos/showcase.mp4
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from render_rollout_videos import FONT, VideoWriter, _wrap  # noqa: E402


def _card(size, title, subtitle="", title_scale=1.4, subtitle_scale=0.75):
    width, height = size
    card = np.zeros((height, width, 3), dtype=np.uint8)
    blocks = [(line, title_scale, 3) for line in _wrap(title, title_scale, 3, width - 80)]
    blocks += [(line, subtitle_scale, 2) for line in _wrap(subtitle, subtitle_scale, 2, width - 160)]
    heights = [int(cv2.getTextSize("Ag", FONT, s, t)[0][1] * 2.0) for _, s, t in blocks]
    y = (height - sum(heights)) // 2
    for (line, scale, thickness), line_h in zip(blocks, heights):
        text_w = cv2.getTextSize(line, FONT, scale, thickness)[0][0]
        y += line_h
        cv2.putText(card, line, ((width - text_w) // 2, y - line_h // 4), FONT, scale,
                    (255, 255, 255), thickness, cv2.LINE_AA)
    return card


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--spec", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--fps", type=float, default=8.0)
    parser.add_argument("--title_sec", type=float, default=3.0)
    parser.add_argument("--caption_sec", type=float, default=1.5)
    args = parser.parse_args()

    spec_path = Path(args.spec)
    sections = json.loads(spec_path.read_text())
    size = None
    for section in sections:
        for example in section["examples"]:
            capture = cv2.VideoCapture(str(spec_path.parent / example["video"]))
            this = (int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)), int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            capture.release()
            if this == (0, 0):
                raise FileNotFoundError(example["video"])
            size = size or this

    writer = VideoWriter(args.out, args.fps, size)
    for section in sections:
        writer.write(_card(size, section["title"], section.get("description", "")),
                     repeat=round(args.title_sec * args.fps))
        for i, example in enumerate(section["examples"], 1):
            writer.write(_card(size, f"{section['title']} - example {i}/{len(section['examples'])}",
                               example.get("caption", ""), 1.0, 0.9),
                         repeat=round(args.caption_sec * args.fps))
            capture = cv2.VideoCapture(str(spec_path.parent / example["video"]))
            repeat = max(1, round(args.fps / (capture.get(cv2.CAP_PROP_FPS) or args.fps)))
            while True:
                ok, frame = capture.read()
                if not ok:
                    break
                if (frame.shape[1], frame.shape[0]) != size:
                    frame = cv2.resize(frame, size)
                writer.write(frame, repeat=repeat)
            capture.release()
    writer.close()
    print(f"Saved {args.out}")


if __name__ == "__main__":
    main()
