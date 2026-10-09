#!/usr/bin/env python3

"""
Full preflight for the PI0.5 inference server.

This test does NOT communicate with ROS, MoveIt, ZED or the robot.

Checks:
    1. Python / package versions / venv.
    2. CUDA / DGX Spark GB10 execution.
    3. LeRobot editable installation and source path.
    4. PI0.5 checkpoint metadata and serialized processor state.
    5. PI05Controller construction.
    6. One real dummy inference and final target validation.

Run through:
    ./run_server.sh --preflight-only
"""

from __future__ import annotations

import argparse
import gc
import importlib.metadata
import json
import os
import platform
import sys
from pathlib import Path

import numpy as np


THIS_DIR = Path(__file__).resolve().parent


def section(title: str) -> None:
    print()
    print("=" * 78)
    print(title)
    print("=" * 78)


def require_file(path: Path, name: str) -> Path:
    path = path.expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"{name} not found: {path}")
    return path


def require_directory(path: Path, name: str) -> Path:
    path = path.expanduser().resolve()
    if not path.is_dir():
        raise FileNotFoundError(f"{name} not found: {path}")
    return path


def package_version(distribution: str) -> str:
    return importlib.metadata.version(distribution)


def check_exact_version(distribution: str, expected: str) -> None:
    actual = package_version(distribution)
    print(f"{distribution:<28} {actual}")
    if actual != expected:
        raise RuntimeError(
            f"{distribution}: expected {expected}, got {actual}"
        )


def configure_python_paths() -> None:
    # /home/ros2_ws/src/ai_controller
    package_dir = THIS_DIR.parents[1]
    package_root = package_dir.parent

    for path in (package_root, THIS_DIR):
        value = str(path)
        if value not in sys.path:
            sys.path.insert(0, value)


# =============================================================================
# 1. Environment
# =============================================================================

def check_environment() -> None:
    section("PREFLIGHT 1/6 - Python / package versions")

    print("Python executable :", sys.executable)
    print("Python version    :", sys.version.split()[0])
    print("sys.prefix        :", sys.prefix)
    print("sys.base_prefix   :", sys.base_prefix)
    print("Architecture      :", platform.machine())
    print()

    if sys.version_info[:2] != (3, 12):
        raise RuntimeError(f"Expected Python 3.12, got {sys.version}")

    if sys.prefix == sys.base_prefix:
        raise RuntimeError(
            "test_setup.py is not running inside pi05_venv."
        )

    if platform.machine() != "aarch64":
        raise RuntimeError(
            f"Expected aarch64 on DGX Spark, got {platform.machine()}"
        )

    expected = {
        "torch": "2.11.0+cu128",
        "torchvision": "0.26.0+cu128",
        "numpy": "2.2.6",
        "scipy": "1.17.1",
        "transformers": "5.5.4",
        "tokenizers": "0.22.2",
        "huggingface-hub": "1.14.0",
        "Pillow": "12.2.0",
        "packaging": "25.0",
        "PyYAML": "6.0.3",
        "safetensors": "0.7.0",
        "draccus": "0.10.0",
        "gymnasium": "1.3.0",
        "opencv-python-headless": "4.13.0.92",
        "requests": "2.34.0",
        "tqdm": "4.67.3",
        "setuptools": "80.10.2",
        "httpx": "0.28.1",
        "httpcore": "1.0.9",
        "Flask": "3.1.3",
        "einops": "0.8.2",
        "lerobot": "0.5.2",
    }

    for distribution, version in expected.items():
        check_exact_version(distribution, version)

    print()
    print("Python environment: OK")


# =============================================================================
# 2. CUDA / GB10
# =============================================================================

def check_cuda() -> None:
    section("PREFLIGHT 2/6 - CUDA / DGX Spark GB10")

    import torch

    print("PyTorch          :", torch.__version__)
    print("PyTorch CUDA     :", torch.version.cuda)
    print("CUDA available   :", torch.cuda.is_available())

    if not torch.cuda.is_available():
        raise RuntimeError("torch.cuda.is_available() == False")

    device_name = torch.cuda.get_device_name(0)
    capability = torch.cuda.get_device_capability(0)

    print("GPU              :", device_name)
    print("Capability       :", capability)

    if capability != (12, 1):
        raise RuntimeError(
            f"Expected GB10 compute capability (12, 1), got {capability}"
        )

    # Basic CUDA kernel.
    x = torch.ones((1024, 1024), device="cuda", dtype=torch.float32)
    y = x @ x
    torch.cuda.synchronize()

    if not bool(torch.isfinite(y).all().item()):
        raise RuntimeError("CUDA matrix multiplication produced non-finite values.")

    print("CUDA matmul      : OK")

    # Useful smoke test for the sm_121 / CUDA-build issue.
    z = torch.linspace(
        0.99,
        1.01,
        steps=4096,
        device="cuda",
        dtype=torch.float32,
    )
    reduction = torch.prod(z)
    torch.cuda.synchronize()

    if not bool(torch.isfinite(reduction).item()):
        raise RuntimeError("CUDA reduction produced a non-finite value.")

    print("CUDA reduction   : OK")
    print("CUDA runtime     : OK")

    del x, y, z, reduction
    torch.cuda.empty_cache()


# =============================================================================
# 3. LeRobot
# =============================================================================

def check_lerobot_installation() -> None:
    section("PREFLIGHT 3/6 - LeRobot editable installation")

    import lerobot

    lerobot_root = Path(
        os.environ.get("PI05_LEROBOT_ROOT", "/opt/pi05/lerobot")
    ).resolve()

    expected_source_root = (
        lerobot_root / "src" / "lerobot"
    ).resolve()

    require_directory(expected_source_root, "LeRobot source directory")

    actual_path = Path(lerobot.__file__).resolve()

    print("LeRobot version  :", package_version("lerobot"))
    print("LeRobot module   :", actual_path)
    print("Expected source  :", expected_source_root)

    if package_version("lerobot") != "0.5.2":
        raise RuntimeError(
            f"Expected LeRobot 0.5.2, got {package_version('lerobot')}"
        )

    try:
        actual_path.relative_to(expected_source_root)
    except ValueError as exc:
        raise RuntimeError(
            "pi05_venv is not using the mounted LeRobot source.\n"
            f"Expected under: {expected_source_root}\n"
            f"Actual:         {actual_path}"
        ) from exc

    print()
    print("LeRobot editable source: OK")


# =============================================================================
# 4. Checkpoint
# =============================================================================

def check_checkpoint(config_path: Path) -> None:
    section("PREFLIGHT 4/6 - PI0.5 checkpoint / processors")

    import yaml
    from safetensors import safe_open
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies.factory import get_policy_class
    from lerobot.policies.pi05.modeling_pi05 import PI05Policy

    config_path = require_file(config_path, "PI0.5 runtime config")

    with config_path.open("r", encoding="utf-8") as handle:
        runtime_cfg = yaml.safe_load(handle) or {}

    checkpoint_path = require_directory(
        Path(os.environ.get("PI05_CHECKPOINT", "/opt/pi05/checkpoint")),
        "PI0.5 checkpoint",
    )

    configured_checkpoint = runtime_cfg.get("checkpoint_path")
    if configured_checkpoint:
        configured_checkpoint = Path(str(configured_checkpoint)).expanduser().resolve()
        if configured_checkpoint != checkpoint_path:
            raise RuntimeError(
                "Checkpoint path mismatch:\n"
                f"  environment: {checkpoint_path}\n"
                f"  YAML:        {configured_checkpoint}"
            )

    required_files = (
        "config.json",
        "model.safetensors",
        "policy_preprocessor.json",
        "policy_postprocessor.json",
        "policy_preprocessor_step_3_normalizer_processor.safetensors",
        "policy_postprocessor_step_0_unnormalizer_processor.safetensors",
    )

    for filename in required_files:
        require_file(checkpoint_path / filename, filename)

    cfg = PreTrainedConfig.from_pretrained(checkpoint_path)

    if cfg.type != "pi05":
        raise RuntimeError(f"Expected policy type 'pi05', got {cfg.type!r}")

    if int(cfg.chunk_size) != 50:
        raise RuntimeError(f"Expected chunk_size=50, got {cfg.chunk_size}")

    if int(cfg.n_action_steps) != 50:
        raise RuntimeError(
            f"Expected checkpoint n_action_steps=50, got {cfg.n_action_steps}"
        )

    if int(cfg.num_inference_steps) != 10:
        raise RuntimeError(
            f"Expected num_inference_steps=10, got {cfg.num_inference_steps}"
        )

    if tuple(cfg.image_resolution) != (224, 224):
        raise RuntimeError(
            f"Expected image_resolution=(224, 224), got {cfg.image_resolution}"
        )

    if int(cfg.max_state_dim) != 32:
        raise RuntimeError(
            f"Expected max_state_dim=32, got {cfg.max_state_dim}"
        )

    if int(cfg.max_action_dim) != 32:
        raise RuntimeError(
            f"Expected max_action_dim=32, got {cfg.max_action_dim}"
        )

    if bool(cfg.use_relative_actions):
        raise RuntimeError("Expected use_relative_actions=False")

    if str(cfg.dtype) != "bfloat16":
        raise RuntimeError(f"Expected dtype='bfloat16', got {cfg.dtype!r}")

    if tuple(cfg.output_features["action"].shape) != (7,):
        raise RuntimeError(
            f"Expected action shape (7,), got {cfg.output_features['action'].shape}"
        )

    policy_cls = get_policy_class(cfg.type)
    if policy_cls is not PI05Policy:
        raise RuntimeError(f"Unexpected policy class: {policy_cls}")

    # Serialized camera rename.
    with (checkpoint_path / "policy_preprocessor.json").open(
        "r", encoding="utf-8"
    ) as handle:
        pre_cfg = json.load(handle)

    rename_steps = [
        step
        for step in pre_cfg["steps"]
        if step.get("registry_name") == "rename_observations_processor"
    ]

    if len(rename_steps) != 1:
        raise RuntimeError("Expected exactly one rename_observations_processor.")

    rename_map = rename_steps[0]["config"]["rename_map"]

    expected_rename = {
        "observation.images.front": "observation.images.base_0_rgb",
        "observation.images.gripper": "observation.images.left_wrist_0_rgb",
    }

    if rename_map != expected_rename:
        raise RuntimeError(f"Unexpected camera rename map: {rename_map}")

    # Raw proprioceptive state statistics must be 13D.
    pre_state = (
        checkpoint_path
        / "policy_preprocessor_step_3_normalizer_processor.safetensors"
    )

    with safe_open(pre_state, framework="pt", device="cpu") as handle:
        keys = set(handle.keys())

        state_key = None
        for candidate in (
            "observation.state.mean",
            "observation.state.q50",
            "observation.state.min",
        ):
            if candidate in keys:
                state_key = candidate
                break

        if state_key is None:
            raise RuntimeError(
                "Could not find observation.state statistics in preprocessor state."
            )

        state_stats = handle.get_tensor(state_key)

    if tuple(state_stats.shape) != (13,):
        raise RuntimeError(
            f"Expected raw observation.state statistics shape (13,), "
            f"got {tuple(state_stats.shape)}"
        )

    # Action statistics.
    post_state = (
        checkpoint_path
        / "policy_postprocessor_step_0_unnormalizer_processor.safetensors"
    )

    with safe_open(post_state, framework="pt", device="cpu") as handle:
        keys = set(handle.keys())

        for required in ("action.min", "action.max"):
            if required not in keys:
                raise RuntimeError(f"Missing action statistic: {required}")

        action_min = handle.get_tensor("action.min").float().cpu().numpy()
        action_max = handle.get_tensor("action.max").float().cpu().numpy()

    if action_min.shape != (7,) or action_max.shape != (7,):
        raise RuntimeError(
            f"Unexpected action statistics shapes: "
            f"{action_min.shape}, {action_max.shape}"
        )

    if not np.isclose(action_min[6], 0.0, atol=1e-5):
        raise RuntimeError(
            f"Expected gripper action.min=0, got {action_min[6]}"
        )

    if not np.isclose(action_max[6], 20.0, atol=1e-3):
        raise RuntimeError(
            f"Expected gripper action.max=20, got {action_max[6]}"
        )

    print("Checkpoint       :", checkpoint_path)
    print("Policy type      :", cfg.type)
    print("Chunk size       :", cfg.chunk_size)
    print("Action steps     :", cfg.n_action_steps)
    print("Inference steps  :", cfg.num_inference_steps)
    print("Image resolution :", cfg.image_resolution)
    print("Raw state dim    :", state_stats.shape[0])
    print("Camera rename    : OK")
    print("action.min       :", action_min)
    print("action.max       :", action_max)
    print()
    print("Checkpoint metadata: OK")


# =============================================================================
# 5 + 6. Full controller / dummy inference
# =============================================================================

def check_full_inference(config_path: Path, task_name: str) -> None:
    section("PREFLIGHT 5/6 - Full PI0.5 controller load")

    import torch
    from scipy.spatial.transform import Rotation
    from ai_controller.models.pi05_controller.pi05_controller import PI05Controller

    print("Creating PI05Controller...")
    print("This loads the real checkpoint and serialized processors.")
    print()

    controller = PI05Controller(
        model_config=str(config_path),
        task_name=task_name,
    )

    controller.reset()

    # Baseline task ID; adjust in YAML if task IDs are named differently.
    controller.load_command(
        demo_path="",
        task_id="00",
    )

    print()
    print("Controller / checkpoint / task: OK")

    section("PREFLIGHT 6/6 - Real dummy inference")

    # AIControllerNode camera order:
    #   0 front, 1 left, 2 right, 3 gripper
    dummy_images = [
        np.zeros((376, 672, 3), dtype=np.uint8),
        np.zeros((376, 672, 3), dtype=np.uint8),
        np.zeros((376, 672, 3), dtype=np.uint8),
        np.zeros((376, 672, 3), dtype=np.uint8),
    ]

    # Representative UR5e state.
    joints = np.array(
        [
            2.243230104446411,
            -1.776084065437317,
            1.6363073587417603,
            -2.017521619796753,
            4.7088189125061035,
            0.002170085906982422,
        ],
        dtype=np.float64,
    )

    eef_position = np.array(
        [-0.0062, 0.5672, 0.0610],
        dtype=np.float64,
    )

    eef_rpy = np.array(
        [3.1215, -0.0032, 0.0420],
        dtype=np.float64,
    )

    eef_quaternion = Rotation.from_euler(
        "xyz",
        eef_rpy,
    ).as_quat()

    dummy_state = {
        "joint_positions": joints,
        "gripper_qpos": np.array([0.0], dtype=np.float64),
        "eef_position": eef_position,
        "eef_quaternion": eef_quaternion,
        "gripper_closed": False,
    }

    print("Executing one real PI0.5 inference...")

    output = controller.inference(
        input_data=[dummy_images, dummy_state],
        t=0,
        save_path=None,
    )

    output = np.asarray(output, dtype=np.float64)

    if output.ndim != 2 or output.shape[1] != 8 or output.shape[0] < 1:
        raise RuntimeError(
            f"Expected controller output Nx8, got {output.shape}"
        )

    if not np.all(np.isfinite(output)):
        raise RuntimeError(
            f"Controller output contains non-finite values:\n{output}"
        )

    for index, action in enumerate(output):
        quat_norm = float(np.linalg.norm(action[3:7]))
        if not 0.99 <= quat_norm <= 1.01:
            raise RuntimeError(
                f"Action {index}: invalid quaternion norm {quat_norm}"
            )

        if float(action[7]) not in (0.0, 255.0):
            raise RuntimeError(
                f"Action {index}: unexpected gripper command {action[7]}"
            )

    print()
    print("Output shape      :", output.shape)
    print("First action      :", output[0])
    print("Quaternion norm   :", np.linalg.norm(output[0, 3:7]))
    print("Gripper command   :", output[0, 7])

    del controller
    gc.collect()

    if torch.cuda.is_available():
        torch.cuda.empty_cache()

    print()
    print("=" * 78)
    print("FULL PI0.5 PREFLIGHT PASSED")
    print("=" * 78)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument(
        "--config",
        required=True,
        help="PI0.5 runtime YAML.",
    )

    parser.add_argument(
        "--task-name",
        default="pick_place",
        help="Controller task family.",
    )

    args = parser.parse_args()

    config_path = Path(args.config).expanduser().resolve()

    configure_python_paths()

    check_environment()
    check_cuda()
    check_lerobot_installation()
    check_checkpoint(config_path)
    check_full_inference(
        config_path,
        task_name=args.task_name,
    )


if __name__ == "__main__":
    main()
