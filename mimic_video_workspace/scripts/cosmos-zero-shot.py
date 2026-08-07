#!/usr/bin/env python3
"""Run Cosmos zero-shot generation from the first frames and precomputed T5 embedding."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import imageio.v2 as iio
import safetensors.torch as st
import torch


WORKSPACE_DIR = Path(__file__).resolve().parents[1]
MIMIC_MODEL_DIR = WORKSPACE_DIR / "external" / "mimic-video" / "model"
DEFAULT_DATASET_DIR = WORKSPACE_DIR / "processed_data" / "ur5e_pick_place_video"
DEFAULT_OUTPUT_DIR = WORKSPACE_DIR / "outputs"
DEFAULT_CHECKPOINT_DIR = WORKSPACE_DIR / "checkpoints"


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


def find_language_embedding(input_video: Path, dataset_dir: Path, explicit_path: Path | None) -> Path:
    # I video multi-camera sono ep_0000000.mp4 / ep_0000001.mp4,
    # mentre l'embedding testuale e' ep_000000.safetensors.
    if explicit_path is not None:
        return explicit_path

    language_dir = dataset_dir / "language_embeddings"
    candidates = [
        language_dir / f"{input_video.stem}.safetensors",
        language_dir / f"{input_video.stem[:-1]}.safetensors",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate

    raise FileNotFoundError(
        f"No language embedding found for {input_video.name}. Tried: "
        + ", ".join(str(path) for path in candidates)
    )


def read_language_embedding(path: Path) -> torch.Tensor:
    # Gli autori salvano encoded_text senza dimensione batch: la aggiungo qui.
    encoded_text = st.load_file(path)["encoded_text"].to(torch.float32)
    if encoded_text.ndim == 2:
        encoded_text = encoded_text.unsqueeze(0)
    if encoded_text.ndim != 3:
        raise ValueError(f"Unexpected encoded_text shape in {path}: {tuple(encoded_text.shape)}")
    return encoded_text.contiguous()


def add_mimic_video_to_path(checkpoint_dir: Path) -> None:
    # Deve succedere prima degli import Cosmos, per far leggere il checkpoint dir corretto.
    os.environ.setdefault("COSMOS_PREDICT2_ARGS", f"--checkpoints {checkpoint_dir}")
    sys.path.insert(0, str(MIMIC_MODEL_DIR))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-video", type=Path, required=True)
    parser.add_argument("--dataset-dir", type=Path, default=DEFAULT_DATASET_DIR)
    parser.add_argument("--language-embedding", type=Path)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--checkpoint-dir", type=Path, default=DEFAULT_CHECKPOINT_DIR)
    parser.add_argument("--dit-path", type=Path)
    parser.add_argument("--num-conditional-frames", type=int, default=5, choices=[1, 5])
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--guidance", type=float, default=7.0)
    parser.add_argument("--num-sampling-step", type=int, default=35)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--use-cuda-graphs", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    add_mimic_video_to_path(args.checkpoint_dir.resolve())

    from cosmos_predict2.configs.config_video2world import get_cosmos_predict2_video2world_pipeline
    from cosmos_predict2.pipelines.video2world import Video2WorldPipeline, read_and_process_video
    from imaginaire.constants import get_cosmos_predict2_video2world_checkpoint
    from imaginaire.utils.io import save_image_or_video

    input_video = args.input_video.resolve()
    output_dir = args.output_dir.resolve()
    mini_video = output_dir / f"{input_video.stem}_first{args.num_conditional_frames}.mp4"
    output_video = output_dir / f"{input_video.stem}_zero_shot_first{args.num_conditional_frames}.mp4"
    language_embedding_path = find_language_embedding(input_video, args.dataset_dir.resolve(), args.language_embedding)

    print(f"input_video={input_video}")
    print(f"mini_video={mini_video}")
    print(f"language_embedding={language_embedding_path.resolve()}")
    print(f"output_video={output_video}")

    write_first_frames_clip(input_video, mini_video, args.num_conditional_frames, args.fps)
    prompt_embedding = read_language_embedding(language_embedding_path)

    config = get_cosmos_predict2_video2world_pipeline(model_size="2B", resolution="480", fps=10)
    config.guardrail_config.enabled = False

    dit_path = str(args.dit_path.resolve()) if args.dit_path else get_cosmos_predict2_video2world_checkpoint()
    pipe = Video2WorldPipeline.from_config(
        config=config,
        dit_path=dit_path,
        use_text_encoder=False,
        device="cuda",
        torch_dtype=torch.bfloat16,
    )

    num_latent_conditional_frames = pipe.tokenizer.get_latent_num_frames(args.num_conditional_frames)
    num_video_frames = pipe.tokenizer.get_pixel_num_frames(pipe.config.state_t)
    vid_input = read_and_process_video(
        str(mini_video),
        [480, 640],
        num_video_frames,
        num_latent_conditional_frames,
        resize=True,
    )

    video = pipe.generate_video(
        vid_input=vid_input,
        is_video_embedding=False,
        num_latent_conditional_frames=num_latent_conditional_frames,
        prompt_embedding=prompt_embedding,
        guidance=args.guidance,
        num_sampling_step=args.num_sampling_step,
        seed=args.seed,
        use_cuda_graphs=args.use_cuda_graphs,
    )

    save_image_or_video(video, str(output_video), fps=args.fps)
    print(f"saved: {output_video}")


if __name__ == "__main__":
    main()
