#!/usr/bin/env python3
"""Run Cosmos generation on all color/bin combinations using precomputed T5 embeddings
and prebuilt 5-frame initial videos.

Expected structure:
mimic_video_workspace/
├── checkpoints/
├── external/mimic-video/model/
└── test/cosmos/
    ├── initial_frames/
    │   ├── green.mp4
    │   ├── yellow.mp4
    │   ├── blue.mp4
    │   └── red.mp4
    ├── language_embeddings/
    │   ├── green_bin1.safetensors
    │   └── ...
    └── outputs/
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import safetensors.torch as st
import torch


WORKSPACE_DIR = Path(__file__).resolve().parents[2]
TEST_DIR = WORKSPACE_DIR / "test" / "cosmos"
MIMIC_MODEL_DIR = WORKSPACE_DIR / "external" / "mimic-video" / "model"
DEFAULT_OUTPUT_DIR = TEST_DIR / "outputs"
DEFAULT_CHECKPOINT_DIR = WORKSPACE_DIR / "checkpoints"

ALL_COLORS = ("green", "yellow", "blue", "red")
ALL_BINS = (1, 2, 3, 4)


def add_mimic_video_to_path(checkpoint_dir: Path) -> None:
    # Serve prima degli import Cosmos.
    os.environ.setdefault("COSMOS_PREDICT2_ARGS", f"--checkpoints {checkpoint_dir}")
    if str(MIMIC_MODEL_DIR) not in sys.path:
        sys.path.insert(0, str(MIMIC_MODEL_DIR))


def read_language_embedding(path: Path) -> torch.Tensor:
    # encoded_text è salvato senza dimensione batch: la aggiungiamo qui.
    encoded_text = st.load_file(path)["encoded_text"].to(torch.float32)

    if encoded_text.ndim == 2:
        encoded_text = encoded_text.unsqueeze(0)

    if encoded_text.ndim != 3:
        raise ValueError(
            f"Unexpected encoded_text shape in {path}: {tuple(encoded_text.shape)}"
        )

    return encoded_text.contiguous()


def build_test_cases(
    test_dir: Path,
    colors: list[str],
    bins: list[int],
) -> list[tuple[str, int, Path, Path]]:
    cases: list[tuple[str, int, Path, Path]] = []

    initial_dir = test_dir / "initial_frames"
    embedding_dir = test_dir / "language_embeddings"

    for color in colors:
        input_video = initial_dir / f"{color}.mp4"
        if not input_video.exists():
            raise FileNotFoundError(f"Missing initial video: {input_video}")

        for bin_idx in bins:
            embedding_path = embedding_dir / f"{color}_bin{bin_idx}.safetensors"
            if not embedding_path.exists():
                raise FileNotFoundError(
                    f"Missing language embedding: {embedding_path}"
                )

            cases.append((color, bin_idx, input_video, embedding_path))

    return cases


def sanitize_run_name(dit_path: Path) -> str:
    # best_000000988_fused.pt -> best_000000988_fused
    return dit_path.stem


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--dit-paths",
        type=Path,
        nargs="+",
        required=True,
        help="One or more fused .pt checkpoints to evaluate.",
    )
    parser.add_argument(
        "--test-dir",
        type=Path,
        default=TEST_DIR,
        help="Directory containing initial_frames/, language_embeddings/, outputs/.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Base output directory. A subfolder is created for each checkpoint.",
    )
    parser.add_argument(
        "--checkpoint-dir",
        type=Path,
        default=DEFAULT_CHECKPOINT_DIR,
        help="Base Mimic Video checkpoints directory (for tokenizer/text encoder paths).",
    )
    parser.add_argument(
        "--colors",
        type=str,
        nargs="*",
        default=list(ALL_COLORS),
        choices=list(ALL_COLORS),
        help="Subset of colors to test.",
    )
    parser.add_argument(
        "--bins",
        type=int,
        nargs="*",
        default=list(ALL_BINS),
        choices=list(ALL_BINS),
        help="Subset of bins to test.",
    )
    parser.add_argument(
        "--num-conditional-frames",
        type=int,
        default=5,
        choices=[1, 5],
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=10,
    )
    parser.add_argument(
        "--guidance",
        type=float,
        default=7.0,
    )
    parser.add_argument(
        "--num-sampling-step",
        type=int,
        default=35,
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="Same seed will be reused for every test case.",
    )
    parser.add_argument(
        "--use-cuda-graphs",
        action="store_true",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_args()

    test_dir = args.test_dir.resolve()
    output_dir = args.output_dir.resolve()
    checkpoint_dir = args.checkpoint_dir.resolve()

    add_mimic_video_to_path(checkpoint_dir)

    from cosmos_predict2.configs.config_video2world import (
        get_cosmos_predict2_video2world_pipeline,
    )
    from cosmos_predict2.pipelines.video2world import (
        Video2WorldPipeline,
        read_and_process_video,
    )
    from imaginaire.utils.io import save_image_or_video

    cases = build_test_cases(
        test_dir=test_dir,
        colors=args.colors,
        bins=args.bins,
    )

    print(f"Test dir: {test_dir}")
    print(f"Output base dir: {output_dir}")
    print(f"Checkpoint dir: {checkpoint_dir}")
    print(f"Number of test cases: {len(cases)}")

    for dit_path in args.dit_paths:
        dit_path = dit_path.resolve()
        if not dit_path.exists():
            raise FileNotFoundError(f"Checkpoint not found: {dit_path}")

        run_name = sanitize_run_name(dit_path)
        run_output_dir = output_dir / run_name
        run_output_dir.mkdir(parents=True, exist_ok=True)

        print("\n" + "=" * 80)
        print(f"Loading checkpoint: {dit_path}")
        print(f"Run output dir: {run_output_dir}")
        print("=" * 80)

        config = get_cosmos_predict2_video2world_pipeline(
            model_size="2B",
            resolution="480",
            fps=10,
        )
        config.guardrail_config.enabled = False

        pipe = Video2WorldPipeline.from_config(
            config=config,
            dit_path=str(dit_path),
            use_text_encoder=False,
            device="cuda",
            torch_dtype=torch.bfloat16,
        )

        num_latent_conditional_frames = pipe.tokenizer.get_latent_num_frames(
            args.num_conditional_frames
        )
        num_video_frames = pipe.tokenizer.get_pixel_num_frames(pipe.config.state_t)

        for color, bin_idx, input_video, embedding_path in cases:
            output_video = run_output_dir / f"{color}_bin{bin_idx}.mp4"

            print(f"\n[Checkpoint: {run_name}] {color} -> bin{bin_idx}")
            print(f"  input_video: {input_video}")
            print(f"  embedding:   {embedding_path}")
            print(f"  output:      {output_video}")

            prompt_embedding = read_language_embedding(embedding_path)

            vid_input = read_and_process_video(
                str(input_video),
                [480, 640],
                num_video_frames,
                num_latent_conditional_frames,
                resize=True,
            )

            # Nessun grafo autograd durante l'inferenza.
            with torch.inference_mode():
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
            print(f"  saved: {output_video}")

            # Pulizia minima per non accumulare tensori inutili.
            del prompt_embedding
            del vid_input
            del video

        # Scarico la pipeline prima di passare al checkpoint successivo.
        del pipe
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

    print("\nDone.")


if __name__ == "__main__":
    main()