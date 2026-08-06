#!/usr/bin/env python3
"""Add precomputed text and video embeddings to UR5e action safetensors."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import safetensors.numpy as st_np
import safetensors.torch as st_torch
import torch


WORKSPACE_DIR = Path(__file__).resolve().parents[1]
PROCESSED_DIR = WORKSPACE_DIR / "processed_data"
DEFAULT_ACTION_DIR = PROCESSED_DIR / "ur5e_pick_place_action"
DEFAULT_LANGUAGE_EMBEDDINGS_DIR = PROCESSED_DIR / "ur5e_pick_place_video" / "language_embeddings"
DEFAULT_VIDEO_EMBEDDINGS_DIR = PROCESSED_DIR / "ur5e_pick_place_video" / "video_embeddings"
DEFAULT_VIDEO_SUFFIX = "0"
NS_PER_SEC = 1_000_000_000


def language_embedding_path(action_path: Path, language_embeddings_dir: Path) -> Path:
    # ep_000000.safetensors usa l'embedding testuale ep_000000.safetensors.
    return language_embeddings_dir / action_path.name


def video_embedding_path(action_path: Path, video_embeddings_dir: Path, video_suffix: str) -> Path:
    # Per l'action head usiamo la front camera: ep_000000.safetensors -> ep_0000000.safetensors.
    return video_embeddings_dir / f"{action_path.stem}{video_suffix}.safetensors"


def read_language_embedding(path: Path) -> np.ndarray:
    # Gli autori salvano encoded_text come tensore torch; lo porto a float32 numpy e aggiungo la dimensione episodio.
    encoded_text = st_torch.load_file(path)["encoded_text"]
    return np.ascontiguousarray(encoded_text.to(torch.float32).cpu().numpy()[None])


def video_embedding_timestamps(action_data: dict[str, np.ndarray], video_data: dict[str, np.ndarray]) -> np.ndarray:
    # Gli embedding video sono associati agli indici ancora usati dallo script di precompute.
    # Li riallineo ai timestamp di workspace_rgb, gestendo anche gli indici oltre la fine del video.
    video_len = int(video_data["video_len"])
    fps = float(video_data["fps"])
    idxs = video_data["video_embeddings_idxs"].astype(np.int64)
    workspace_timestamps = action_data["workspace_rgb_timestamps"]

    if len(action_data["workspace_rgb"]) != video_len:
        raise ValueError(
            f"Video/action length mismatch: workspace_rgb={len(action_data['workspace_rgb'])}, video_len={video_len}"
        )

    padding_idx = idxs - video_len + 1
    extrapolated = (
        workspace_timestamps[-1] + NS_PER_SEC * np.clip(padding_idx, 0, None) / fps
    ).astype(np.uint64)
    aligned = workspace_timestamps[np.clip(idxs, 0, video_len - 1)]
    return np.where(padding_idx > 0, extrapolated, aligned).astype(np.uint64)


def add_embeddings(
    action_path: Path,
    language_embeddings_dir: Path,
    video_embeddings_dir: Path,
    video_suffix: str,
    overwrite: bool,
) -> str:
    # Carico il safetensor action, aggiungo gli embedding mancanti e risalvo lo stesso file.
    action_data = st_np.load_file(action_path)
    has_language = "language_embedding" in action_data
    has_video = "workspace_rgb_embedding" in action_data

    if has_language and has_video and not overwrite:
        return f"skip: {action_path.name}"

    if overwrite or not has_language:
        lang_path = language_embedding_path(action_path, language_embeddings_dir)
        if not lang_path.exists():
            raise FileNotFoundError(lang_path)
        action_data["language_embedding"] = read_language_embedding(lang_path)
        action_data["language_embedding_timestamps"] = np.array([0], dtype=np.uint64)

    if overwrite or not has_video:
        vid_path = video_embedding_path(action_path, video_embeddings_dir, video_suffix)
        if not vid_path.exists():
            raise FileNotFoundError(vid_path)
        video_data = st_np.load_file(vid_path)
        action_data["workspace_rgb_embedding"] = np.ascontiguousarray(video_data["video_embeddings"])
        action_data["workspace_rgb_embedding_timestamps"] = video_embedding_timestamps(action_data, video_data)
        action_data["num_conditional_frames"] = np.array([int(video_data["num_conditional_frames"])])
        action_data["num_conditional_frames_timestamps"] = np.array([0], dtype=np.uint64)

    st_np.save_file(action_data, action_path)
    return f"ok: {action_path.name}"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--action-dir", type=Path, default=DEFAULT_ACTION_DIR)
    parser.add_argument("--language-embeddings-dir", type=Path, default=DEFAULT_LANGUAGE_EMBEDDINGS_DIR)
    parser.add_argument("--video-embeddings-dir", type=Path, default=DEFAULT_VIDEO_EMBEDDINGS_DIR)
    parser.add_argument("--video-suffix", default=DEFAULT_VIDEO_SUFFIX)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    action_paths = sorted(args.action_dir.glob("ep_*.safetensors"))
    if not action_paths:
        raise FileNotFoundError(f"No action safetensors found in {args.action_dir}")

    print(f"action_dir={args.action_dir.resolve()}")
    print(f"language_embeddings_dir={args.language_embeddings_dir.resolve()}")
    print(f"video_embeddings_dir={args.video_embeddings_dir.resolve()}")
    print(f"video_suffix={args.video_suffix}")

    for action_path in action_paths:
        print(add_embeddings(
            action_path,
            args.language_embeddings_dir,
            args.video_embeddings_dir,
            args.video_suffix,
            args.overwrite,
        ))

    print(f"done: {len(action_paths)} action safetensors")


if __name__ == "__main__":
    main()
