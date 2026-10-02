#!/usr/bin/env python3

"""
Full preflight for the VLA-JEPA inference server.

This test does NOT communicate with ROS, MoveIt, ZED or the robot.
It does NOT require server.py to already be running.

Checks:

    1. Python / package versions / isolated venv.
    2. Hugging Face cache and local Qwen availability.
    3. CUDA 13 / GB10 / NVRTC execution.
    4. LeRobot editable installation and source location.
    5. VLA-JEPA checkpoint metadata and serialized processors.
    6. Runtime YAML and UR5e preprocessing/postprocessing.
    7. Full VLAJEPAController construction.
    8. Real dummy inference and final UR5e target validation.

The preflight can therefore be executed with the robot, MoveIt, ZED and
every other runtime container switched off.
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


def check_exact_version(
    distribution: str,
    expected: str,
) -> None:
    actual = package_version(distribution)

    print(f"{distribution:<24} {actual}")

    if actual != expected:
        raise RuntimeError(
            f"{distribution}: expected {expected}, got {actual}"
        )


def check_version_range(
    distribution: str,
    requirement: str,
) -> None:
    from packaging.specifiers import SpecifierSet
    from packaging.version import Version

    actual = package_version(distribution)

    print(f"{distribution:<24} {actual}")

    if Version(actual) not in SpecifierSet(requirement):
        raise RuntimeError(
            f"{distribution}: expected {requirement}, got {actual}"
        )


# =============================================================================
# Python paths
# =============================================================================


def configure_python_paths() -> None:
    """
    Make the ROS Python package and the controller-local modules importable.

    LeRobot itself should NOT need to be manually added here:
    it must come from the editable installation inside the VLA-JEPA venv.
    """

    package_dir = THIS_DIR.parents[1]
    package_root = package_dir.parent

    for path in (
        package_root,
        THIS_DIR,
    ):
        path_string = str(path)

        if path_string not in sys.path:
            sys.path.insert(
                0,
                path_string,
            )


# =============================================================================
# 1. Environment
# =============================================================================


def check_environment() -> None:
    section("PREFLIGHT 1/8 - Python / package versions")

    print("Python executable :", sys.executable)
    print("Python version    :", sys.version.split()[0])
    print("sys.prefix        :", sys.prefix)
    print("sys.base_prefix   :", sys.base_prefix)
    print("Architecture      :", platform.machine())
    print()

    if sys.version_info[:2] != (3, 12):
        raise RuntimeError(
            f"Expected Python 3.12, got {sys.version}"
        )

    if sys.prefix == sys.base_prefix:
        raise RuntimeError(
            "test_setup.py is not running inside the VLA-JEPA venv."
        )

    if platform.machine() != "aarch64":
        raise RuntimeError(
            f"Expected aarch64 on DGX Spark, got {platform.machine()}"
        )

    # Exact versions intentionally pinned by the Spark runtime.
    check_exact_version(
        "torch",
        "2.11.0+cu130",
    )
    check_exact_version(
        "torchvision",
        "0.26.0+cu130",
    )

    # Dependency ranges used when the VLA-JEPA venv was created.
    check_version_range(
        "numpy",
        ">=2.0.0,<2.3.0",
    )
    check_version_range(
        "opencv-python",
        ">=4.9.0,<4.14.0",
    )
    check_version_range(
        "Pillow",
        ">=10.0.0,<13.0.0",
    )
    check_version_range(
        "einops",
        ">=0.8.0,<0.9.0",
    )
    check_exact_version(
        "draccus",
        "0.10.0",
    )
    check_version_range(
        "huggingface-hub",
        ">=1.0.0,<2.0.0",
    )
    check_version_range(
        "requests",
        ">=2.32.0,<3.0.0",
    )
    check_version_range(
        "gymnasium",
        ">=1.1.1,<2.0.0",
    )
    check_version_range(
        "safetensors",
        ">=0.4.3,<1.0.0",
    )
    check_version_range(
        "packaging",
        ">=24.2,<26.0",
    )
    check_version_range(
        "transformers",
        ">=5.4.0,<5.6.0",
    )
    check_version_range(
        "diffusers",
        ">=0.27.2,<0.36.0",
    )
    check_version_range(
        "qwen-vl-utils",
        ">=0.0.11,<0.1.0",
    )
    check_version_range(
        "peft",
        ">=0.18.0,<1.0.0",
    )

    # Presence checks for runtime packages without an exact pin.
    for distribution in (
        "scipy",
        "accelerate",
        "omegaconf",
        "PyYAML",
        "Flask",
        "scikit-learn",
    ):
        print(
            f"{distribution:<24} "
            f"{package_version(distribution)}"
        )

    print()
    print("Python environment: OK")


# =============================================================================
# 2. Hugging Face cache
# =============================================================================


def check_huggingface_cache() -> None:
    section("PREFLIGHT 2/8 - Hugging Face cache")

    hf_home_value = os.environ.get(
        "HF_HOME",
        "",
    ).strip()

    hf_hub_cache_value = os.environ.get(
        "HF_HUB_CACHE",
        "",
    ).strip()

    if not hf_home_value:
        raise EnvironmentError(
            "HF_HOME is not set."
        )

    if not hf_hub_cache_value:
        raise EnvironmentError(
            "HF_HUB_CACHE is not set."
        )

    hf_home = require_directory(
        Path(hf_home_value),
        "HF_HOME",
    )

    hub_cache = require_directory(
        Path(hf_hub_cache_value),
        "HF_HUB_CACHE",
    )

    from huggingface_hub import snapshot_download
    from huggingface_hub import constants as hf_constants

    resolved_hf_home = Path(
        hf_constants.HF_HOME
    ).resolve()

    resolved_hub_cache = Path(
        hf_constants.HF_HUB_CACHE
    ).resolve()

    print("HF_HOME env       :", hf_home)
    print("HF_HOME resolved  :", resolved_hf_home)
    print("HF_HUB_CACHE env  :", hub_cache)
    print("HF_HUB_CACHE lib  :", resolved_hub_cache)

    if resolved_hf_home != hf_home:
        raise RuntimeError(
            "huggingface_hub resolved a different HF_HOME:\n"
            f"  env:      {hf_home}\n"
            f"  resolved: {resolved_hf_home}"
        )

    if resolved_hub_cache != hub_cache:
        raise RuntimeError(
            "huggingface_hub resolved a different HF_HUB_CACHE:\n"
            f"  env:      {hub_cache}\n"
            f"  resolved: {resolved_hub_cache}"
        )

    qwen_repo = "Qwen/Qwen3-VL-2B-Instruct"

    qwen_cache_dir = (
        hub_cache
        / "models--Qwen--Qwen3-VL-2B-Instruct"
    )

    require_directory(
        qwen_cache_dir,
        "Qwen3-VL-2B Hugging Face cache",
    )

    # Stronger than checking only the directory name:
    # ask huggingface_hub to resolve the snapshot with networking disabled.
    qwen_snapshot = snapshot_download(
        repo_id=qwen_repo,
        cache_dir=str(hub_cache),
        local_files_only=True,
    )

    qwen_snapshot = require_directory(
        Path(qwen_snapshot),
        "Local Qwen snapshot",
    )

    print()
    print("Qwen repository   :", qwen_repo)
    print("Qwen snapshot     :", qwen_snapshot)

    # Informational only.
    #
    # The current VLA-JEPA runtime explicitly disables the latent world model
    # before constructing the inference policy, so V-JEPA2 is not required by
    # the current real-robot inference path.
    vjepa_cache_dir = (
        hub_cache
        / "models--facebook--vjepa2-vitl-fpc64-256"
    )

    print(
        "V-JEPA2 cache     :",
        (
            str(vjepa_cache_dir)
            if vjepa_cache_dir.is_dir()
            else "not present (not required by current inference path)"
        ),
    )

    print()
    print("Hugging Face cache: OK")


# =============================================================================
# 3. CUDA / GB10
# =============================================================================


def check_cuda() -> None:
    section("PREFLIGHT 3/8 - CUDA 13 / GB10 / NVRTC")

    import torch
    import torchvision

    print("PyTorch          :", torch.__version__)
    print("Torchvision      :", torchvision.__version__)
    print("PyTorch CUDA     :", torch.version.cuda)

    if torch.__version__ != "2.11.0+cu130":
        raise RuntimeError(
            f"Expected torch 2.11.0+cu130, got {torch.__version__}"
        )

    if torchvision.__version__ != "0.26.0+cu130":
        raise RuntimeError(
            "Expected torchvision 0.26.0+cu130, "
            f"got {torchvision.__version__}"
        )

    if torch.version.cuda != "13.0":
        raise RuntimeError(
            f"Expected CUDA 13.0 PyTorch build, got {torch.version.cuda}"
        )

    if not torch.cuda.is_available():
        raise RuntimeError(
            "torch.cuda.is_available() == False"
        )

    device_name = torch.cuda.get_device_name(0)
    capability = torch.cuda.get_device_capability(0)

    print("GPU              :", device_name)
    print("Capability       :", capability)

    if capability != (12, 1):
        raise RuntimeError(
            "Expected DGX Spark GB10 compute capability (12, 1), "
            f"got {capability}"
        )

    # Basic CUDA execution.
    matrix = torch.ones(
        (1024, 1024),
        device="cuda",
        dtype=torch.float32,
    )

    result = matrix @ matrix

    if not bool(
        torch.isfinite(result).all().item()
    ):
        raise RuntimeError(
            "CUDA matrix multiplication produced non-finite values."
        )

    print("CUDA matmul      : OK")

    # Reduction kernel used as NVRTC smoke test on GB10/sm_121.
    reduction_input = torch.linspace(
        0.99,
        1.01,
        steps=4096,
        device="cuda",
        dtype=torch.float32,
    )

    reduction_output = torch.prod(
        reduction_input
    )

    torch.cuda.synchronize()

    if not bool(
        torch.isfinite(reduction_output).item()
    ):
        raise RuntimeError(
            "CUDA reduction produced a non-finite value."
        )

    print("NVRTC reduction  : OK")
    print("CUDA runtime     : OK")

    del matrix
    del result
    del reduction_input
    del reduction_output

    torch.cuda.empty_cache()


# =============================================================================
# 4. LeRobot editable installation
# =============================================================================


def check_lerobot_installation() -> None:
    section("PREFLIGHT 4/8 - LeRobot editable installation")

    import lerobot

    actual_path = Path(
        lerobot.__file__
    ).resolve()

    expected_root = (
        THIS_DIR
        / "external"
        / "lerobot"
    ).resolve()

    expected_source_root = (
        expected_root
        / "src"
        / "lerobot"
    ).resolve()

    require_directory(
        expected_source_root,
        "LeRobot source directory",
    )

    print("LeRobot version   :", package_version("lerobot"))
    print("LeRobot module    :", actual_path)
    print("Expected source   :", expected_source_root)

    if package_version("lerobot") != "0.5.2":
        raise RuntimeError(
            "Expected LeRobot 0.5.2, got "
            f"{package_version('lerobot')}"
        )

    try:
        actual_path.relative_to(
            expected_source_root
        )
    except ValueError as exc:
        raise RuntimeError(
            "The VLA-JEPA venv is not using the LeRobot source "
            "mounted in the current repository.\n"
            f"Expected under:\n  {expected_source_root}\n"
            f"Actual:\n  {actual_path}"
        ) from exc

    print()
    print("LeRobot editable source: OK")


# =============================================================================
# 5. Checkpoint + serialized processors
# =============================================================================


def check_checkpoint(
    config_path: Path,
) -> None:
    section("PREFLIGHT 5/8 - VLA-JEPA checkpoint / processors")

    import yaml

    from safetensors import safe_open

    from lerobot.configs.policies import (
        PreTrainedConfig,
    )

    from lerobot.policies.factory import (
        get_policy_class,
    )

    from lerobot.policies.vla_jepa.modeling_vla_jepa import (
        VLAJEPAPolicy,
    )

    config_path = require_file(
        config_path,
        "VLA-JEPA runtime config",
    )

    with config_path.open(
        "r",
        encoding="utf-8",
    ) as handle:
        controller_cfg = yaml.safe_load(
            handle
        ) or {}

    checkpoint_env = os.environ.get(
        "VLA_JEPA_CHECKPOINT",
        "",
    ).strip()

    if not checkpoint_env:
        raise EnvironmentError(
            "VLA_JEPA_CHECKPOINT is not set."
        )

    checkpoint_path = require_directory(
        Path(checkpoint_env),
        "VLA-JEPA checkpoint",
    )

    configured_checkpoint = Path(
        str(
            controller_cfg.get(
                "checkpoint_path",
                "",
            )
        )
    ).expanduser().resolve()

    if configured_checkpoint != checkpoint_path:
        raise RuntimeError(
            "Checkpoint path mismatch:\n"
            f"  run_server.sh: {checkpoint_path}\n"
            f"  YAML:          {configured_checkpoint}"
        )

    required_files = (
        "config.json",
        "model.safetensors",
        "policy_preprocessor.json",
        "policy_postprocessor.json",
        "policy_postprocessor_ur5e.json",
        "policy_preprocessor_step_3_normalizer_processor.safetensors",
        "policy_postprocessor_step_2_unnormalizer_processor.safetensors",
    )

    for filename in required_files:
        require_file(
            checkpoint_path / filename,
            filename,
        )

    # -------------------------------------------------------------------------
    # LeRobot policy configuration
    # -------------------------------------------------------------------------

    cfg = PreTrainedConfig.from_pretrained(
        checkpoint_path
    )

    if cfg.type != "vla_jepa":
        raise RuntimeError(
            f"Expected policy type 'vla_jepa', got {cfg.type!r}"
        )

    if int(cfg.action_dim) != 7:
        raise RuntimeError(
            f"Expected action_dim=7, got {cfg.action_dim}"
        )

    if int(cfg.chunk_size) != 7:
        raise RuntimeError(
            f"Expected chunk_size=7, got {cfg.chunk_size}"
        )

    if int(cfg.n_action_steps) != 7:
        raise RuntimeError(
            f"Expected n_action_steps=7, got {cfg.n_action_steps}"
        )

    if int(cfg.num_inference_timesteps) != 4:
        raise RuntimeError(
            "Expected num_inference_timesteps=4, got "
            f"{cfg.num_inference_timesteps}"
        )

    if tuple(cfg.resize_images_to) != (
        224,
        224,
    ):
        raise RuntimeError(
            "Expected resize_images_to=(224, 224), got "
            f"{cfg.resize_images_to}"
        )

    expected_input_features = {
        "observation.images.exterior_1_left",
        "observation.images.exterior_2_left",
    }

    actual_input_features = set(
        cfg.input_features.keys()
    )

    if actual_input_features != expected_input_features:
        raise RuntimeError(
            "Unexpected VLA-JEPA input features: "
            f"{sorted(actual_input_features)}"
        )

    if "observation.state" in actual_input_features:
        raise RuntimeError(
            "The current UR5e VLA-JEPA checkpoint must not "
            "declare observation.state as a policy input."
        )

    for key in (
        "action",
        "action.world",
    ):
        if key not in cfg.output_features:
            raise RuntimeError(
                f"Missing output feature {key!r}"
            )

        shape = tuple(
            cfg.output_features[key].shape
        )

        if shape != (7,):
            raise RuntimeError(
                f"{key} must have shape (7,), got {shape}"
            )

    policy_cls = get_policy_class(
        cfg.type
    )

    if policy_cls is not VLAJEPAPolicy:
        raise RuntimeError(
            f"Unexpected policy class: {policy_cls}"
        )

    # -------------------------------------------------------------------------
    # Serialized preprocessor
    # -------------------------------------------------------------------------

    preprocessor_path = (
        checkpoint_path
        / "policy_preprocessor.json"
    )

    with preprocessor_path.open(
        "r",
        encoding="utf-8",
    ) as handle:
        pre_cfg = json.load(handle)

    rename_steps = [
        step
        for step in pre_cfg["steps"]
        if step.get("registry_name")
        == "rename_observations_processor"
    ]

    if len(rename_steps) != 1:
        raise RuntimeError(
            "Expected exactly one rename_observations_processor."
        )

    rename_map = rename_steps[0][
        "config"
    ][
        "rename_map"
    ]

    expected_rename_map = {
        "observation.images.front":
            "observation.images.exterior_1_left",
        "observation.images.gripper":
            "observation.images.exterior_2_left",
    }

    if rename_map != expected_rename_map:
        raise RuntimeError(
            f"Unexpected camera rename map: {rename_map}"
        )

    # -------------------------------------------------------------------------
    # Serialized postprocessor
    # -------------------------------------------------------------------------

    postprocessor_filename = str(
        controller_cfg.get(
            "postprocessor_config_filename",
            "policy_postprocessor.json",
        )
    )

    postprocessor_path = require_file(
        checkpoint_path
        / postprocessor_filename,
        "Configured VLA-JEPA postprocessor",
    )

    with postprocessor_path.open(
        "r",
        encoding="utf-8",
    ) as handle:
        post_cfg = json.load(handle)

    post_names = [
        step.get("registry_name")
        for step in post_cfg["steps"]
    ]

    expected_post_names = [
        "vla_jepa_clip_actions",
        "unnormalizer_processor",
        "device_processor",
    ]

    if post_names != expected_post_names:
        raise RuntimeError(
            "Unexpected postprocessor pipeline: "
            f"{post_names}"
        )

    # -------------------------------------------------------------------------
    # Processor state files
    # -------------------------------------------------------------------------

    state_files: list[str] = []

    for processor_cfg in (
        pre_cfg,
        post_cfg,
    ):
        for step in processor_cfg["steps"]:
            state_file = step.get(
                "state_file"
            )

            if state_file is not None:
                state_files.append(
                    state_file
                )

    for state_file in state_files:
        require_file(
            checkpoint_path / state_file,
            f"Processor state {state_file}",
        )

    # -------------------------------------------------------------------------
    # UR5e action statistics
    # -------------------------------------------------------------------------

    unnormalizer_steps = [
        step
        for step in post_cfg["steps"]
        if step.get("registry_name")
        == "unnormalizer_processor"
    ]

    if len(unnormalizer_steps) != 1:
        raise RuntimeError(
            "Expected exactly one unnormalizer_processor."
        )

    post_state_path = require_file(
        checkpoint_path
        / unnormalizer_steps[0]["state_file"],
        "Action unnormalizer state",
    )

    with safe_open(
        post_state_path,
        framework="pt",
        device="cpu",
    ) as handle:

        keys = set(
            handle.keys()
        )

        for required_key in (
            "action.min",
            "action.max",
            "action.mean",
            "action.std",
        ):
            if required_key not in keys:
                raise RuntimeError(
                    f"Missing statistic {required_key}"
                )

        action_min = (
            handle.get_tensor("action.min")
            .float()
            .cpu()
            .numpy()
        )

        action_max = (
            handle.get_tensor("action.max")
            .float()
            .cpu()
            .numpy()
        )

    if action_min.shape != (7,):
        raise RuntimeError(
            f"action.min has shape {action_min.shape}"
        )

    if action_max.shape != (7,):
        raise RuntimeError(
            f"action.max has shape {action_max.shape}"
        )

    if not np.all(np.isfinite(action_min)):
        raise RuntimeError(
            "action.min contains non-finite values."
        )

    if not np.all(np.isfinite(action_max)):
        raise RuntimeError(
            "action.max contains non-finite values."
        )

    if not np.isclose(
        action_min[6],
        0.0,
        atol=1e-5,
    ):
        raise RuntimeError(
            f"Expected gripper action.min=0, got {action_min[6]}"
        )

    if not np.isclose(
        action_max[6],
        20.0,
        atol=1e-3,
    ):
        raise RuntimeError(
            f"Expected gripper action.max=20, got {action_max[6]}"
        )

    print("Checkpoint        :", checkpoint_path)
    print("Policy type       :", cfg.type)
    print("Action dim        :", cfg.action_dim)
    print("Chunk size        :", cfg.chunk_size)
    print("Action steps      :", cfg.n_action_steps)
    print("Inference steps   :", cfg.num_inference_timesteps)
    print("Input features    :", sorted(actual_input_features))
    print("Camera rename     : OK")
    print("Postprocessor     :", postprocessor_filename)
    print("Processor states  : OK")
    print("action.min        :", action_min)
    print("action.max        :", action_max)

    print()
    print("Checkpoint metadata: OK")


# =============================================================================
# 6. Runtime config + UR5e utils
# =============================================================================


def check_controller_config_and_utils(
    config_path: Path,
) -> None:
    section("PREFLIGHT 6/8 - Runtime config / UR5e preprocessing")

    import torch
    import yaml

    from ai_controller.models.vla_jepa_controller.vla_jepa_utils import (
        ACTION_DIM,
        DATASET_ACTION_SCALE,
        FRONT_CROP_MARGINS,
        IMAGE_SIZE,
        build_lerobot_observation,
        delta_action_to_absolute_target,
        process_front_image,
        process_gripper_image,
    )

    config_path = require_file(
        config_path,
        "VLA-JEPA runtime config",
    )

    with config_path.open(
        "r",
        encoding="utf-8",
    ) as handle:
        cfg = yaml.safe_load(
            handle
        ) or {}

    # -------------------------------------------------------------------------
    # Runtime YAML invariants
    # -------------------------------------------------------------------------

    expected_values = {
        "device": "cuda",
        "seed": 1000,
        "front_camera_index": 0,
        "gripper_camera_index": 3,
        "input_color_order": "rgb",
        "dataset_action_scale": 0.05,
        "postprocessor_config_filename": "policy_postprocessor_ur5e.json",
        "gripper_open_position": 0.0,
        "gripper_closed_position": 255.0,
        "server_host": "127.0.0.1",
        "server_port": 8770,
        "grasp_z_offset_m": -0.02,
    }

    for key, expected in expected_values.items():

        if key not in cfg:
            raise RuntimeError(
                f"Missing runtime config key: {key}"
            )

        actual = cfg[key]

        if isinstance(expected, float):
            if not np.isclose(
                float(actual),
                expected,
                atol=1e-9,
            ):
                raise RuntimeError(
                    f"{key}: expected {expected}, got {actual}"
                )
        elif actual != expected:
            raise RuntimeError(
                f"{key}: expected {expected!r}, got {actual!r}"
            )

    if ACTION_DIM != 7:
        raise RuntimeError(
            f"Expected ACTION_DIM=7, got {ACTION_DIM}"
        )

    if not np.isclose(
        DATASET_ACTION_SCALE,
        0.05,
    ):
        raise RuntimeError(
            "Expected DATASET_ACTION_SCALE=0.05."
        )

    # -------------------------------------------------------------------------
    # Tasks
    #
    # 00..15 are the baseline task set.
    # Additional OOD/generalization task IDs are allowed.
    # -------------------------------------------------------------------------

    tasks = cfg.get(
        "tasks",
        {},
    )

    expected_baseline_ids = {
        f"{index:02d}"
        for index in range(16)
    }

    actual_task_ids = {
        str(task_id)
        for task_id in tasks.keys()
    }

    missing_baseline = (
        expected_baseline_ids
        - actual_task_ids
    )

    if missing_baseline:
        raise RuntimeError(
            "Missing baseline task IDs: "
            f"{sorted(missing_baseline)}"
        )

    for task_id, task_cfg in tasks.items():

        prompt = str(
            (task_cfg or {}).get(
                "prompt",
                "",
            )
        ).strip()

        if not prompt:
            raise RuntimeError(
                f"Task {task_id}: empty prompt."
            )

    # -------------------------------------------------------------------------
    # Image preprocessing
    # -------------------------------------------------------------------------

    dummy_front = np.zeros(
        (376, 672, 3),
        dtype=np.uint8,
    )

    dummy_gripper = np.zeros(
        (376, 672, 3),
        dtype=np.uint8,
    )

    front = process_front_image(
        dummy_front,
        input_color_order="rgb",
    )

    gripper = process_gripper_image(
        dummy_gripper,
        input_color_order="rgb",
    )

    expected_shape = (
        3,
        IMAGE_SIZE,
        IMAGE_SIZE,
    )

    for name, image in (
        ("front", front),
        ("gripper", gripper),
    ):

        if tuple(image.shape) != expected_shape:
            raise RuntimeError(
                f"{name} shape = {tuple(image.shape)}, "
                f"expected {expected_shape}"
            )

        if image.dtype != torch.float32:
            raise RuntimeError(
                f"{name} dtype = {image.dtype}"
            )

        if (
            float(image.min()) < 0.0
            or float(image.max()) > 1.0
        ):
            raise RuntimeError(
                f"{name} image is not in [0, 1]."
            )

    observation = build_lerobot_observation(
        front_image=front,
        gripper_image=gripper,
        task=tasks["00"]["prompt"],
    )

    expected_observation_keys = {
        "observation.images.front",
        "observation.images.gripper",
        "task",
    }

    if set(observation.keys()) != expected_observation_keys:
        raise RuntimeError(
            "Unexpected raw LeRobot observation keys: "
            f"{observation.keys()}"
        )

    # -------------------------------------------------------------------------
    # Dataset scale / geometric postprocessing
    # -------------------------------------------------------------------------

    reference_position = np.array(
        [
            0.10,
            0.20,
            0.30,
        ],
        dtype=np.float32,
    )

    reference_quaternion = np.array(
        [
            0.0,
            0.0,
            0.0,
            1.0,
        ],
        dtype=np.float32,
    )

    # +1.0 in dataset action space must become +0.05 m physically.
    test_action = np.array(
        [
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],
        dtype=np.float32,
    )

    (
        position,
        quaternion,
        _gripper_state,
    ) = delta_action_to_absolute_target(
        postprocessed_action=test_action,
        reference_position=reference_position,
        reference_quaternion_xyzw=reference_quaternion,
        currently_closed=False,
    )

    expected_position = np.array(
        [
            0.15,
            0.20,
            0.30,
        ],
        dtype=np.float32,
    )

    if not np.allclose(
        position,
        expected_position,
        atol=1e-6,
    ):
        raise RuntimeError(
            "Dataset action scale test failed:\n"
            f"  expected: {expected_position}\n"
            f"  actual:   {position}"
        )

    quaternion_norm = float(
        np.linalg.norm(
            quaternion
        )
    )

    if not np.isclose(
        quaternion_norm,
        1.0,
        atol=1e-6,
    ):
        raise RuntimeError(
            f"Quaternion output is not normalized: {quaternion_norm}"
        )

    print("Config            :", config_path)
    print("Image size        :", IMAGE_SIZE)
    print("Front crop        :", FRONT_CROP_MARGINS)
    print("Action dimension  :", ACTION_DIM)
    print("Dataset scale     :", DATASET_ACTION_SCALE)
    print("Defined tasks     :", len(tasks))
    print("Baseline tasks    : 16/16 OK")
    print("Front preprocessing  : OK")
    print("Gripper preprocessing: OK")
    print("LeRobot observation  : OK")
    print("Delta -> absolute    : OK")

    print()
    print("Runtime config / UR5e utils: OK")


# =============================================================================
# 7 + 8. Full model load / real dummy inference
# =============================================================================


def check_full_inference(
    config_path: Path,
    task_name: str,
) -> None:
    section("PREFLIGHT 7/8 - Full VLA-JEPA model load")

    import torch

    from ai_controller.models.vla_jepa_controller.vla_jepa_controller import (
        VLAJEPAController,
    )

    print("Creating VLAJEPAController...")
    print("This loads the real checkpoint, Qwen backbone and processors.")
    print()

    controller = VLAJEPAController(
        model_config=str(config_path),
        task_name=task_name,
    )

    controller.reset()

    controller.load_command(
        demo_path="",
        task_id="00",
    )

    print()
    print("Controller / checkpoint / task: OK")

    section("PREFLIGHT 8/8 - Real dummy inference")

    # Complete camera list expected by AIControllerNode:
    #
    #   0 -> front
    #   1 -> left
    #   2 -> right
    #   3 -> gripper
    #
    # VLA-JEPA uses only 0 and 3.

    dummy_front_rgb = np.zeros(
        (376, 672, 3),
        dtype=np.uint8,
    )

    dummy_left_rgb = np.zeros(
        (376, 672, 3),
        dtype=np.uint8,
    )

    dummy_right_rgb = np.zeros(
        (376, 672, 3),
        dtype=np.uint8,
    )

    dummy_gripper_rgb = np.zeros(
        (376, 672, 3),
        dtype=np.uint8,
    )

    dummy_images = [
        dummy_front_rgb,
        dummy_left_rgb,
        dummy_right_rgb,
        dummy_gripper_rgb,
    ]

    # [x, y, z, qx, qy, qz, qw, gripper_closed]
    #
    # The pose is used by the UR5e postprocessing stage to transform
    # VLA-JEPA delta actions into absolute MoveIt targets.
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

    print("Executing one real VLA-JEPA inference...")

    output = controller.inference(
        input_data=[
            dummy_images,
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
            "Controller output contains non-finite values:\n"
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
            f"Invalid output quaternion norm: {quaternion_norm}"
        )

    gripper = float(
        output[
            0,
            7,
        ]
    )

    if gripper not in (
        0.0,
        255.0,
    ):
        raise RuntimeError(
            "Unexpected final gripper command: "
            f"{gripper}"
        )

    print()
    print("Output shape      :", output.shape)
    print("Output action     :", output[0])
    print("Quaternion norm   :", f"{quaternion_norm:.9f}")
    print("Gripper command   :", gripper)

    del controller

    gc.collect()

    if torch.cuda.is_available():
        torch.cuda.empty_cache()

    print()
    print("=" * 78)
    print("FULL VLA-JEPA PREFLIGHT PASSED")
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
        help="VLA-JEPA runtime YAML.",
    )

    parser.add_argument(
        "--task-name",
        default="pick_place",
        help="Controller task family.",
    )

    args = parser.parse_args()

    config_path = Path(
        args.config
    ).expanduser().resolve()

    configure_python_paths()

    check_environment()
    check_huggingface_cache()
    check_cuda()
    check_lerobot_installation()
    check_checkpoint(
        config_path
    )
    check_controller_config_and_utils(
        config_path
    )
    check_full_inference(
        config_path,
        task_name=args.task_name,
    )


if __name__ == "__main__":
    main()