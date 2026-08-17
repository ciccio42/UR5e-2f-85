#!/usr/bin/env python3
"""Verifica modello, dati, bilanciamento e memoria di Cosmos o action head."""

import argparse
import bisect
import gc
import inspect
import os
import shutil
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

from imaginaire.lazy_config import instantiate
from imaginaire.utils import distributed
from imaginaire.utils import misc
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
    parser.add_argument("target", choices=("cosmos", "action"))
    parser.add_argument("--batch-size", type=int, choices=(1, 12), default=1)
    parser.add_argument("--rank", type=int, choices=(64, 128, 256), default=64)
    parser.add_argument("--lora-alpha", type=int, default=32)
    parser.add_argument("--cosmos-checkpoint", type=Path)
    parser.add_argument("--epoch", type=int, default=0)
    parser.add_argument("--windows", type=int, default=1)
    return parser.parse_args()


def install_action_configs() -> None:
    source_root = WORKSPACE_ROOT / "configs/action_head/dataloading"
    target_root = MODEL_ROOT / "cosmos_predict2/configs/dataloading"
    relative_paths = (
        Path("ur5e_videmb.yaml"),
        Path("dataset/ur5e.yaml"),
        Path("dataset/transform/ur5e_to_ur5e_videmb.yaml"),
        Path("policy_io/ur5e_videmb.yaml"),
    )
    for relative_path in relative_paths:
        source = source_root / relative_path
        target = target_root / relative_path
        if not source.is_file():
            raise FileNotFoundError(source)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)


def ensure_patches_are_applied(target: str) -> None:
    if target == "cosmos":
        checks = {
            MODEL_ROOT / "cosmos_predict2/configs/defaults/data_video.py": "ur5e_pick_place_videmb",
            MODEL_ROOT / "cosmos_predict2/data/resumable_sampler.py": "_get_ur5e_video_task_indices",
        }
    else:
        checks = {
            MODEL_ROOT / "cosmos_predict2/configs/defaults/world2action_pipe.py": '"ur5e"',
            MODEL_ROOT / "cosmos_predict2/configs/defaults/world2action_model.py": "v2w_ur5e_finetuned",
            MODEL_ROOT / "cosmos_predict2/data/resumable_sampler.py": "_get_ur5e_action_task_indices",
        }

    missing = [str(path) for path, marker in checks.items() if marker not in path.read_text()]
    if missing:
        raise RuntimeError(
            f"Le patch {target} non risultano applicate ai file: {', '.join(missing)}"
        )


def cosmos_episode_info(dataset: Any, dataset_idx: int) -> tuple[str, int]:
    video_path = dataset._video_paths[dataset_idx]
    episode_stem = video_path.stem[:-1] if dataset._is_multi_img else video_path.stem
    episode_id = int(episode_stem.removeprefix("ep_"))
    return episode_stem, episode_id // EPISODES_PER_TASK


def action_episode_info(dataset: Any, dataset_idx: int) -> tuple[str, int]:
    chunk_reader = dataset._chunk_reader
    episode_idx = bisect.bisect_right(chunk_reader._cumulative_num_timesteps, dataset_idx) - 1
    episode_path = chunk_reader._episode_paths[episode_idx]
    episode_id = int(episode_path.stem.removeprefix("ep_"))
    return episode_path.stem, episode_id // EPISODES_PER_TASK


def episode_info(target: str, dataset: Any, dataset_idx: int) -> tuple[str, int]:
    if target == "cosmos":
        return cosmos_episode_info(dataset, dataset_idx)
    return action_episode_info(dataset, dataset_idx)


def print_batch_shapes(value: Any, prefix: str = "") -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            print_batch_shapes(child, f"{prefix}/{key}" if prefix else key)
    elif torch.is_tensor(value):
        print(f"  {prefix}: shape={tuple(value.shape)}, dtype={value.dtype}, device={value.device}")
    else:
        print(f"  {prefix}: {type(value).__name__}")


def process_rss_gib() -> float | None:
    status_path = Path("/proc/self/status")
    if not status_path.is_file():
        return None
    for line in status_path.read_text().splitlines():
        if line.startswith("VmRSS:"):
            return int(line.split()[1]) / 1024**2
    return None


def print_memory(label: str) -> None:
    torch.cuda.synchronize()
    free_bytes, total_bytes = torch.cuda.mem_get_info()
    rss = process_rss_gib()
    print(f"\n========== MEMORIA: {label} ==========")
    if rss is not None:
        print(f"process_rss_gib:      {rss:.2f}")
    print(f"cuda_allocated_gib:   {torch.cuda.memory_allocated() / 1024**3:.2f}")
    print(f"cuda_reserved_gib:    {torch.cuda.memory_reserved() / 1024**3:.2f}")
    print(f"cuda_peak_gib:        {torch.cuda.max_memory_allocated() / 1024**3:.2f}")
    print(f"cuda_free_gib:        {free_bytes / 1024**3:.2f}")
    print(f"cuda_total_gib:       {total_bytes / 1024**3:.2f}")


def validate_batch(target: str, batch: dict[str, Any], batch_size: int) -> None:
    if target == "cosmos":
        required_keys = {
            "video_embedding",
            "obs/language_embedding",
            "t5_text_mask",
            "fps",
            "padding_mask",
            "num_conditional_frames",
        }
    else:
        required_keys = {
            "obs/workspace_rgb_embedding",
            "obs/num_conditional_frames",
            "obs/language_embedding",
            "obs/lowdim_concat",
            "action/lowdim_concat",
        }

    missing_keys = required_keys - set(batch)
    if missing_keys:
        raise KeyError(f"Chiavi mancanti nel batch {target}: {sorted(missing_keys)}")

    if target == "action":
        obs_shape = tuple(batch["obs/lowdim_concat"].shape)
        action_shape = tuple(batch["action/lowdim_concat"].shape)
        if obs_shape != (batch_size, 1, 10):
            raise ValueError(f"Forma obs/lowdim_concat inattesa: {obs_shape}")
        if action_shape != (batch_size, 15, 10):
            raise ValueError(f"Forma action/lowdim_concat inattesa: {action_shape}")


def main() -> None:
    args = parse_args()
    if args.windows <= 0:
        raise ValueError("--windows deve essere positivo.")

    if args.target == "action":
        install_action_configs()
        checkpoint = args.cosmos_checkpoint or Path(
            os.environ.get(
                "UR5E_COSMOS_CKPT",
                CHECKPOINT_ROOT / "video_backbone/v2w_pretrained_cosmos.pt",
            )
        )
        if not checkpoint.is_absolute():
            checkpoint = WORKSPACE_ROOT / checkpoint
        if not checkpoint.is_file():
            raise FileNotFoundError(f"Checkpoint Cosmos non trovato: {checkpoint}")
        os.environ["UR5E_COSMOS_CKPT"] = str(checkpoint.resolve())

    ensure_patches_are_applied(args.target)

    from cosmos_predict2.configs.config import make_config

    if args.target == "cosmos":
        experiment = (
            "v2w_ur5e_pick_place_videmb_"
            f"lora_rank{args.rank}_lr1.778e-04_bsz{args.batch_size}"
        )
        overrides = [
            "--",
            f"experiment={experiment}",
            f"model.config.lora_alpha={args.lora_alpha}",
        ]
    else:
        experiment = (
            "w2a_ur5e_videmb_v2w_ur5e_finetuned_"
            f"lr1.000e-04_layer20_bsz{args.batch_size}"
        )
        overrides = ["--", f"experiment={experiment}"]

    config = override(make_config(), overrides)

    distributed.init()
    config.validate()
    config.freeze()
    initialize_model_parallel(config)

    model = instantiate(config.model)
    dataloader = instantiate(config.dataloader_train)
    dataset = dataloader.dataset
    model = model.to("cuda", memory_format=config.trainer.memory_format)

    if args.target == "action":
        print("\n========== STATISTICHE ACTION ==========")
        statistics = dataset.get_statistics()
        print(f"stats_id:             {dataset.stats_id}")
        print(f"normalized_keys:      {sorted(statistics)}")
        model.on_train_start(
            config.trainer.memory_format,
            statistics,
            dataset.stats_id,
        )
        del statistics

    grad_accum = config.trainer.grad_accum_iter
    physical_batch = dataloader.batch_size
    effective_batch = physical_batch * grad_accum

    print("\n========== CONFIGURAZIONE ==========")
    print(f"target:              {args.target}")
    print(f"experiment:          {experiment}")
    if args.target == "action":
        print(f"cosmos_checkpoint:   {config.model.config.video_dit_path}")
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
    print_memory("MODELLO")

    print("\n========== DATALOADER ==========")
    sampler = dataloader.sampler
    sampler_method_name = (
        "_get_ur5e_video_task_indices"
        if args.target == "cosmos"
        else "_get_ur5e_action_task_indices"
    )
    if not hasattr(sampler, sampler_method_name):
        raise RuntimeError(f"Il sampler bilanciato {args.target} non risulta applicato.")

    task_indices = getattr(sampler, sampler_method_name)()
    if task_indices is None:
        raise RuntimeError(f"Il sampler non ha riconosciuto il dataset {args.target} UR5e.")
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
        samples = [episode_info(args.target, dataset, index) for index in indices]
        task_ids = [task_id for _episode, task_id in samples]

        print(f"optimizer_window {window_idx}:")
        for micro_idx, microbatch in enumerate(microbatches):
            details = [
                f"idx={index} {episode} task={task_id}"
                for index, (episode, task_id) in zip(
                    microbatch,
                    [episode_info(args.target, dataset, index) for index in microbatch],
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
    first_batch = next(data_iterator)

    print("\n========== PRIMO BATCH REALE ==========")
    print_batch_shapes(first_batch)
    validate_batch(args.target, first_batch, physical_batch)
    print_memory("MODELLO + BATCH CPU")

    gpu_batch = misc.to(first_batch, device="cuda")
    print_memory("MODELLO + BATCH CUDA")
    print("\nControllo completato: modello, configurazione e batch bilanciato sono validi.")

    if hasattr(data_iterator, "_shutdown_workers"):
        data_iterator._shutdown_workers()
    del gpu_batch, first_batch, data_iterator, dataloader, model
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
