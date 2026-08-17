#!/usr/bin/env python3
"""Carica Cosmos e verifica il primo effective batch UR5e bilanciato."""

import argparse
import gc
import inspect
import os
import sys
from itertools import chain
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = REPO_ROOT / "mimic_video_workspace"
MODEL_ROOT = WORKSPACE_ROOT / "external/mimic-video/model"
CHECKPOINT_ROOT = WORKSPACE_ROOT / "checkpoints"
if not (MODEL_ROOT / "cosmos_predict2/configs/config.py").is_file():
    raise FileNotFoundError(f"Codice MimicVideo non trovato in: {MODEL_ROOT}")
os.environ.setdefault("COSMOS_PREDICT2_ARGS", f'--checkpoints "{CHECKPOINT_ROOT}"')
sys.path.insert(0, str(MODEL_ROOT))
os.chdir(MODEL_ROOT)

import torch
import cosmos_predict2
from megatron.core import parallel_state

from cosmos_predict2.configs.config import make_config
from imaginaire.lazy_config import instantiate
from imaginaire.utils import distributed
from imaginaire.utils.config_helper import override


NUM_TASKS = 12
EPISODES_PER_TASK = 40

IMPORTED_MODEL_ROOT = Path(cosmos_predict2.__file__).resolve().parent.parent
if IMPORTED_MODEL_ROOT != MODEL_ROOT.resolve():
    raise RuntimeError(
        f"MimicVideo importato da {IMPORTED_MODEL_ROOT}, atteso {MODEL_ROOT.resolve()}"
    )


def initialize_model_parallel(config: Any) -> None:
    kwargs = {
        "pipeline_model_parallel_size": config.model_parallel.pipeline_model_parallel_size,
        "tensor_model_parallel_size": config.model_parallel.tensor_model_parallel_size,
        "context_parallel_size": config.model_parallel.context_parallel_size,
    }
    if "create_gloo_process_groups" in inspect.signature(parallel_state.initialize_model_parallel).parameters:
        kwargs["create_gloo_process_groups"] = False
    parallel_state.initialize_model_parallel(**kwargs)
    parallel_state.sequence_parallel = config.model_parallel.sequence_parallel


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch-size", type=int, choices=(1, 12), default=1)
    parser.add_argument("--rank", type=int, choices=(64, 128, 256), default=64)
    parser.add_argument("--lora-alpha", type=int, default=32)
    parser.add_argument("--epoch", type=int, default=0)
    parser.add_argument("--windows", type=int, default=1)
    return parser.parse_args()


def episode_info(dataset: Any, dataset_idx: int) -> tuple[str, int]:
    video_path = dataset._video_paths[dataset_idx]
    episode_stem = video_path.stem[:-1] if dataset._is_multi_img else video_path.stem
    episode_id = int(episode_stem.removeprefix("ep_"))
    return episode_stem, episode_id // EPISODES_PER_TASK


def print_batch_shapes(value: Any, prefix: str = "") -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            print_batch_shapes(child, f"{prefix}/{key}" if prefix else key)
    elif torch.is_tensor(value):
        print(f"  {prefix}: shape={tuple(value.shape)}, dtype={value.dtype}, device={value.device}")
    else:
        print(f"  {prefix}: {type(value).__name__}")


def main() -> None:
    args = parse_args()
    if args.windows <= 0:
        raise ValueError("--windows deve essere positivo.")

    experiment = (
        "v2w_ur5e_pick_place_videmb_"
        f"lora_rank{args.rank}_lr1.778e-04_bsz{args.batch_size}"
    )
    config = override(
        make_config(),
        [
            "--",
            f"experiment={experiment}",
            f"model.config.lora_alpha={args.lora_alpha}",
        ],
    )

    distributed.init()
    config.validate()
    config.freeze()
    initialize_model_parallel(config)

    model = instantiate(config.model)
    dataloader = instantiate(config.dataloader_train)

    grad_accum = config.trainer.grad_accum_iter
    physical_batch = dataloader.batch_size
    effective_batch = physical_batch * grad_accum

    print("\n========== CONFIGURAZIONE ==========")
    print(f"experiment:          {experiment}")
    print(f"physical_batch:      {physical_batch}")
    print(f"grad_accum_iter:     {grad_accum}")
    print(f"effective_batch:     {effective_batch}")
    print(f"num_workers:         {dataloader.num_workers}")
    print(f"prefetch_factor:     {dataloader.prefetch_factor}")
    print(f"in_order:            {dataloader.in_order}")
    print(f"max_iter:            {config.trainer.max_iter}")

    if effective_batch != NUM_TASKS:
        raise ValueError(f"L'effective batch deve essere {NUM_TASKS}, ricevuto {effective_batch}.")

    print("\n========== MODELLO ==========")
    total_parameters = sum(parameter.numel() for parameter in model.parameters())
    trainable_parameters = sum(parameter.numel() for parameter in model.parameters() if parameter.requires_grad)
    print(f"class:                {type(model).__name__}")
    print(f"parameters:           {total_parameters:,}")
    print(f"trainable_parameters: {trainable_parameters:,}")
    print(f"cuda_allocated_gib:   {torch.cuda.memory_allocated() / 1024**3:.2f}")
    print(f"cuda_reserved_gib:    {torch.cuda.memory_reserved() / 1024**3:.2f}")

    print("\n========== DATALOADER ==========")
    sampler = dataloader.sampler
    dataset = dataloader.dataset

    if not hasattr(sampler, "_get_ur5e_video_task_indices"):
        raise RuntimeError("Il sampler bilanciato Cosmos non risulta applicato.")

    task_indices = sampler._get_ur5e_video_task_indices()
    task_counts = [len(indices) for indices in task_indices]
    print(f"dataset_samples:      {len(dataset)}")
    print(f"sampler_samples:      {len(sampler)}")
    print(f"samples_per_task:     {task_counts}")
    print(f"physical_batches:     {len(dataloader)}")

    sampler.set_epoch(args.epoch)
    sampler.set_start_iter(0)
    batch_indices = iter(dataloader.batch_sampler)

    print("\n========== BILANCIAMENTO ==========")
    for window_idx in range(args.windows):
        microbatches = [list(next(batch_indices)) for _ in range(grad_accum)]
        indices = list(chain.from_iterable(microbatches))
        samples = [episode_info(dataset, index) for index in indices]
        task_ids = [task_id for _episode, task_id in samples]

        print(f"optimizer_window {window_idx}:")
        for micro_idx, microbatch in enumerate(microbatches):
            details = [
                f"idx={index} {episode} task={task_id}"
                for index, (episode, task_id) in zip(
                    microbatch,
                    [episode_info(dataset, index) for index in microbatch],
                    strict=True,
                )
            ]
            print(f"  microbatch {micro_idx:02d}: " + " | ".join(details))

        if sorted(task_ids) != list(range(NUM_TASKS)):
            raise AssertionError(f"Batch non bilanciato: task={task_ids}")
        print(f"  OK: task presenti {sorted(task_ids)}")

    sampler.set_epoch(args.epoch)
    sampler.set_start_iter(0)
    data_iterator = iter(dataloader)
    first_batch = None
    for _ in range(grad_accum):
        batch = next(data_iterator)
        if first_batch is None:
            first_batch = batch

    print("\n========== PRIMO BATCH REALE ==========")
    print_batch_shapes(first_batch)
    print("\nControllo completato: modello, configurazione e batch bilanciato sono validi.")

    if hasattr(data_iterator, "_shutdown_workers"):
        data_iterator._shutdown_workers()
    del first_batch, batch, data_iterator, dataloader, model
    gc.collect()
    torch.cuda.empty_cache()


if __name__ == "__main__":
    try:
        main()
    finally:
        if parallel_state.is_initialized():
            parallel_state.destroy_model_parallel()
        if torch.distributed.is_initialized():
            torch.distributed.destroy_process_group()
