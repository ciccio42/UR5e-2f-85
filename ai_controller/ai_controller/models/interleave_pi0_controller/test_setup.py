#!/usr/bin/env python3

"""
Full preflight for the Interleave-Pi0 inference server.

This test does NOT communicate with MoveIt and cannot move the robot.

Checks:
    1. Python / pinned package versions.
    2. CUDA / GPU / Triton / ptxas.
    3. Real torch.compile CUDA execution.
    4. Open-Pi-Zero imports.
    5. Runtime config and assets.
    6. Full InterleavePi0Controller construction.
    7. Checkpoint + processor + instruction images + statistics.
    8. Real dummy inference and post-processing.
"""

from __future__ import annotations

import argparse
import importlib.metadata
import json
import os
import platform
import subprocess
import sys
from pathlib import Path

import numpy as np


THIS_DIR = Path(__file__).resolve().parent


# =============================================================================
# Helpers
# =============================================================================


def section(title: str) -> None:
    print()
    print("=" * 78)
    print(title)
    print("=" * 78)


def require_file(path: Path, name: str) -> Path:
    path = path.expanduser().resolve()

    if not path.is_file():
        raise FileNotFoundError(
            f"{name} not found: {path}"
        )

    return path


def require_directory(path: Path, name: str) -> Path:
    path = path.expanduser().resolve()

    if not path.is_dir():
        raise FileNotFoundError(
            f"{name} not found: {path}"
        )

    return path


def package_version(distribution: str) -> str:
    return importlib.metadata.version(distribution)


def check_version(
    distribution: str,
    expected: str,
) -> None:
    actual = package_version(distribution)

    print(f"{distribution:<24} {actual}")

    if actual != expected:
        raise RuntimeError(
            f"{distribution}: expected {expected}, got {actual}"
        )


# =============================================================================
# Python paths
# =============================================================================


def configure_python_paths() -> Path:
    """
    Make both ai_controller and open-pi-zero importable.

    OPEN_PI_ZERO must be supplied by run_server.sh / runtime environment.
    """

    open_pi_zero_value = os.environ.get(
        "OPEN_PI_ZERO",
        "",
    ).strip()

    if not open_pi_zero_value:
        raise EnvironmentError(
            "OPEN_PI_ZERO is not set."
        )

    open_pi_zero = require_directory(
        Path(open_pi_zero_value),
        "OPEN_PI_ZERO",
    )

    # Layout:
    #
    # ai_controller/
    # └── ai_controller/
    #     └── models/
    #         └── interleave_pi0_controller/
    #
    # package_root = outer ai_controller directory.
    package_dir = THIS_DIR.parents[1]
    package_root = package_dir.parent

    for path in (
        open_pi_zero,
        package_root,
        THIS_DIR,
    ):
        path_string = str(path)

        if path_string not in sys.path:
            sys.path.insert(
                0,
                path_string,
            )

    return open_pi_zero


# =============================================================================
# 1. Environment
# =============================================================================


def check_environment() -> None:
    section("PREFLIGHT 1/5 - Python / package versions")

    print("Python executable :", sys.executable)
    print("Python version    :", sys.version.split()[0])
    print("Architecture      :", platform.machine())
    print()

    if sys.version_info[:2] != (3, 10):
        raise RuntimeError(
            f"Expected Python 3.10, got {sys.version}"
        )

    if platform.machine() != "aarch64":
        raise RuntimeError(
            f"Expected aarch64, got {platform.machine()}"
        )

    # Versions shared with training.
    check_version("torch", "2.8.0+cu129")
    check_version("triton", "3.4.0")
    check_version("transformers", "4.47.1")
    check_version("tokenizers", "0.21.4")
    check_version("huggingface-hub", "0.36.2")

    check_version("numpy", "1.26.4")
    check_version("scipy", "1.11.4")
    check_version("bitsandbytes", "0.49.0")

    check_version("hydra-core", "1.3.6")
    check_version("omegaconf", "2.3.1")
    check_version("einops", "0.8.2")
    check_version("Pillow", "12.3.0")
    check_version("sentencepiece", "0.2.0")
    check_version("safetensors", "0.8.0")

    check_version("PyYAML", "6.0.3")

    # Server-only packages.
    print()
    print("Server packages:")
    print(
        f"{'Flask':<24} "
        f"{package_version('Flask')}"
    )
    print(
        f"{'opencv-python-headless':<24} "
        f"{package_version('opencv-python-headless')}"
    )

    print()
    print("Python environment: OK")


# =============================================================================
# 2. CUDA / Triton
# =============================================================================


def check_cuda_and_triton() -> None:
    section("PREFLIGHT 2/5 - CUDA / Triton / torch.compile")

    import torch
    import triton

    print("PyTorch          :", torch.__version__)
    print("PyTorch CUDA     :", torch.version.cuda)
    print("Triton           :", triton.__version__)

    if torch.version.cuda != "12.9":
        raise RuntimeError(
            f"Expected torch CUDA 12.9, got {torch.version.cuda}"
        )

    ptxas_value = os.environ.get(
        "TRITON_PTXAS_PATH",
        "",
    ).strip()

    if not ptxas_value:
        raise EnvironmentError(
            "TRITON_PTXAS_PATH is not set."
        )

    ptxas = require_file(
        Path(ptxas_value),
        "CUDA 12.9 ptxas",
    )

    print("TRITON_PTXAS_PATH:", ptxas)

    result = subprocess.run(
        [str(ptxas), "--version"],
        check=True,
        capture_output=True,
        text=True,
    )

    ptxas_output = (
        result.stdout
        + result.stderr
    )

    print()
    print(ptxas_output.strip())

    if "V12.9.86" not in ptxas_output:
        raise RuntimeError(
            "Expected ptxas V12.9.86."
        )

    if not torch.cuda.is_available():
        raise RuntimeError(
            "torch.cuda.is_available() == False"
        )

    print()
    print("GPU              :", torch.cuda.get_device_name(0))
    print("Capability       :", torch.cuda.get_device_capability(0))

    # Simple real CUDA operation.
    x = torch.ones(
        4,
        device="cuda",
    )

    if float(x.sum().item()) != 4.0:
        raise RuntimeError(
            "Unexpected CUDA tensor result."
        )

    print("CUDA tensor      : OK")

    # -------------------------------------------------------------------------
    # Real torch.compile / Triton execution.
    #
    # torch.compile is lazy: compilation is triggered by the first execution.
    # This catches ptxas/Triton problems before loading the large VLA model.
    # -------------------------------------------------------------------------

    def compiled_function(tensor):
        return torch.sin(tensor) * tensor + 2.0

    compiled_function = torch.compile(
        compiled_function,
        fullgraph=True,
    )

    compile_input = torch.randn(
        4096,
        device="cuda",
        dtype=torch.float32,
    )

    compile_output = compiled_function(
        compile_input
    )

    torch.cuda.synchronize()

    if not bool(
        torch.isfinite(
            compile_output
        ).all().item()
    ):
        raise RuntimeError(
            "torch.compile produced non-finite values."
        )

    print("torch.compile     : OK")
    print("Triton / ptxas   : OK")


# =============================================================================
# 3. Open-Pi-Zero + assets
# =============================================================================


def check_open_pi_zero_and_assets(
    config_path: Path,
) -> None:
    section("PREFLIGHT 3/5 - Open-Pi-Zero / runtime assets")

    from omegaconf import OmegaConf
    from PIL import Image

    from src.model.vla.interleaved_pizero import (
        InterleavedPiZeroInference,
    )
    from src.model.vla.interleaved_processing import (
        InterleavedVLAProcessor,
    )

    print("InterleavedPiZeroInference: OK")
    print("InterleavedVLAProcessor:    OK")

    config_path = require_file(
        config_path,
        "Interleave runtime config",
    )

    cfg = OmegaConf.load(
        config_path
    )

    # This also resolves:
    #
    # ${oc.env:INTERLEAVE_PI0_CHECKPOINT}
    # ${oc.env:INTERLEAVE_PI0_PALIGEMMA}
    OmegaConf.resolve(cfg)

    config_dir = config_path.parent

    checkpoint_path = require_file(
        Path(str(cfg.checkpoint_path)),
        "Interleave checkpoint",
    )

    paligemma_path = require_directory(
        Path(str(cfg.pretrained_model_path)),
        "PaliGemma directory",
    )

    stats_path = Path(
        str(cfg.dataset_statistics_path)
    )

    if not stats_path.is_absolute():
        stats_path = config_dir / stats_path

    stats_path = require_file(
        stats_path,
        "Dataset statistics",
    )

    with stats_path.open(
        "r",
        encoding="utf-8",
    ) as handle:
        stats = json.load(handle)

    for group in (
        "action",
        "proprio",
    ):
        if group not in stats:
            raise RuntimeError(
                f"Missing statistics group: {group}"
            )

        for key in (
            "p01",
            "p99",
        ):
            if key not in stats[group]:
                raise RuntimeError(
                    f"Missing statistics key: {group}.{key}"
                )

            values = np.asarray(
                stats[group][key],
                dtype=np.float32,
            )

            if values.shape != (7,):
                raise RuntimeError(
                    f"{group}.{key} must have shape (7,), "
                    f"got {values.shape}"
                )

            if not np.all(
                np.isfinite(values)
            ):
                raise RuntimeError(
                    f"{group}.{key} contains non-finite values."
                )

        p01 = np.asarray(
            stats[group]["p01"],
            dtype=np.float32,
        )

        p99 = np.asarray(
            stats[group]["p99"],
            dtype=np.float32,
        )

        if np.any(
            p99 <= p01
        ):
            raise RuntimeError(
                f"{group}: every p99 must be greater than p01."
            )

    # Current UR5e runtime invariants.
    if float(
        cfg.action_scale_factor
    ) != 0.05:
        raise RuntimeError(
            "Expected action_scale_factor=0.05."
        )

    if float(
        cfg.gripper_action_closed_value
    ) != 20.0:
        raise RuntimeError(
            "Expected gripper_action_closed_value=20.0."
        )

    if cfg.final_action_clip_value is not None:
        raise RuntimeError(
            "final_action_clip_value must be null."
        )

    if int(cfg.cond_steps) != 1:
        raise RuntimeError(
            "Expected cond_steps=1."
        )

    if int(cfg.horizon_steps) != 4:
        raise RuntimeError(
            "Expected horizon_steps=4."
        )

    if int(cfg.action_dim) != 7:
        raise RuntimeError(
            "Expected action_dim=7."
        )

    if int(cfg.proprio_dim) != 7:
        raise RuntimeError(
            "Expected proprio_dim=7."
        )

    if not bool(
        cfg.get(
            "use_torch_compile",
            True,
        )
    ):
        raise RuntimeError(
            "use_torch_compile must be enabled for this runtime."
        )

    tasks = cfg.tasks

    if len(tasks) != 16:
        raise RuntimeError(
            f"Expected 16 tasks, found {len(tasks)}."
        )

    expected_task_ids = {
        f"{index:02d}"
        for index in range(16)
    }

    actual_task_ids = {
        str(task_id)
        for task_id in tasks.keys()
    }

    if actual_task_ids != expected_task_ids:
        raise RuntimeError(
            "Unexpected task IDs: "
            f"{sorted(actual_task_ids)}"
        )

    num_instruction_images = int(
        cfg.num_instruction_images
    )

    for task_id in sorted(
        expected_task_ids
    ):
        task = tasks[task_id]

        prompt = str(
            task.prompt
        )

        instruction_images = list(
            task.instruction_images
        )

        if len(
            instruction_images
        ) != num_instruction_images:
            raise RuntimeError(
                f"Task {task_id}: expected "
                f"{num_instruction_images} instruction images, "
                f"found {len(instruction_images)}."
            )

        # Current controller expects the YAML form <image>, which is later
        # converted to <image_placeholder> before the processor.
        placeholder_count = prompt.count(
            "<image>"
        )

        if placeholder_count != num_instruction_images:
            raise RuntimeError(
                f"Task {task_id}: prompt contains "
                f"{placeholder_count} <image> tags, "
                f"expected {num_instruction_images}."
            )

        for image_value in instruction_images:

            image_path = Path(
                str(image_value)
            )

            if not image_path.is_absolute():
                image_path = (
                    config_dir
                    / image_path
                )

            image_path = require_file(
                image_path,
                f"Task {task_id} instruction image",
            )

            with Image.open(
                image_path
            ) as image:

                if image.size != (
                    224,
                    224,
                ):
                    raise RuntimeError(
                        f"{image_path}: expected 224x224, "
                        f"got {image.size}."
                    )

    print()
    print("Config             :", config_path)
    print("Checkpoint         :", checkpoint_path)
    print("PaliGemma          :", paligemma_path)
    print("Statistics         :", stats_path)
    print("Tasks              : 16/16 OK")
    print(
        "Instruction images : "
        f"{num_instruction_images}/task OK"
    )
    print("Runtime assets     : OK")


# =============================================================================
# 4 + 5. Full model load / dummy inference
# =============================================================================


def check_full_inference(
    config_path: Path,
) -> None:
    section("PREFLIGHT 4/5 - Full model load")

    import torch

    from ai_controller.models.interleave_pi0_controller.interleave_pi0_controller import (
        InterleavePi0Controller,
    )

    print("Creating InterleavePi0Controller...")
    print("This loads the real checkpoint and processor.")
    print()

    controller = InterleavePi0Controller(
        model_config=str(config_path),
        task_name="pick_place",
    )

    controller.load_command(
        demo_path="",
        task_id="00",
    )

    print()
    print("Controller / checkpoint / task: OK")

    section("PREFLIGHT 5/5 - Real dummy inference")

    # Original front-camera dimensions.
    dummy_front_rgb = np.zeros(
        (
            376,
            672,
            3,
        ),
        dtype=np.uint8,
    )

    # [x, y, z, qx, qy, qz, qw, gripper_closed]
    dummy_robot_state = np.array(
        [
            -0.15552094619366708,
             0.34869994018501943,
             0.1532803451753288,
             0.9994452044624775,
             0.03161651380119412,
             0.0021438049655468088,
             0.010251021036213035,
             0.0,
        ],
        dtype=np.float64,
    )

    print(
        "Executing one real Interleave-Pi0 inference..."
    )

    output = controller.inference(
        input_data=[
            [
                dummy_front_rgb,
            ],
            dummy_robot_state,
        ],
        t=0,
        save_path=None,
    )

    output = np.asarray(
        output,
        dtype=np.float32,
    )

    if output.shape != (
        1,
        8,
    ):
        raise RuntimeError(
            "Unexpected controller output shape: "
            f"{output.shape}"
        )

    if not np.all(
        np.isfinite(output)
    ):
        raise RuntimeError(
            "Controller output contains non-finite values: "
            f"{output}"
        )

    quaternion = output[
        0,
        3:7,
    ]

    quaternion_norm = float(
        np.linalg.norm(
            quaternion
        )
    )

    if (
        not np.isfinite(
            quaternion_norm
        )
        or quaternion_norm < 0.99
        or quaternion_norm > 1.01
    ):
        raise RuntimeError(
            "Invalid output quaternion norm: "
            f"{quaternion_norm}"
        )

    print()
    print("Output shape      :", output.shape)
    print("Output action     :", output[0])
    print(
        "Quaternion norm   : "
        f"{quaternion_norm:.9f}"
    )

    del controller

    if torch.cuda.is_available():
        torch.cuda.empty_cache()

    print()
    print("=" * 78)
    print("FULL INTERLEAVE-Pi0 PREFLIGHT PASSED")
    print("=" * 78)


# =============================================================================
# Main
# =============================================================================


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
    )

    parser.add_argument(
        "--config",
        required=True,
        help="Interleave-Pi0 runtime YAML.",
    )

    args = parser.parse_args()

    config_path = Path(
        args.config
    ).expanduser().resolve()

    configure_python_paths()

    check_environment()
    check_cuda_and_triton()
    check_open_pi_zero_and_assets(
        config_path
    )
    check_full_inference(
        config_path
    )


if __name__ == "__main__":
    main()