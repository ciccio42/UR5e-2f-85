#!/usr/bin/env python3
"""
Download the allenai/GraspMolmo checkpoint into the Hugging Face cache
used by this project.

Intended repository location:
    ai_controller/ai_controller/utils/grasping/graspmolmo/scripts/download_checkpoint.py

The runtime loads:
    allenai/GraspMolmo

The cache is stored under:
    <repo>/.runtime/huggingface
"""

from __future__ import annotations

import os
from pathlib import Path

MODEL_ID = "allenai/GraspMolmo"


def find_repo_root() -> Path:
    script = Path(__file__).resolve()

    for parent in script.parents:
        grasping_dir = (
            parent
            / "ai_controller"
            / "ai_controller"
            / "utils"
            / "grasping"
        )
        if grasping_dir.is_dir():
            return parent

    raise RuntimeError(
        "Could not locate the UR5e-2f-85 repository root. "
        "Place this script under "
        "ai_controller/ai_controller/utils/grasping/graspmolmo/scripts/."
    )


def main() -> None:
    repo_root = find_repo_root()

    hf_home = repo_root / ".runtime" / "huggingface"
    hf_hub_cache = hf_home / "hub"

    hf_hub_cache.mkdir(
        parents=True,
        exist_ok=True,
    )

    os.environ["HF_HOME"] = str(hf_home)
    os.environ["HF_HUB_CACHE"] = str(hf_hub_cache)

    try:
        from huggingface_hub import snapshot_download
    except ImportError as exc:
        raise RuntimeError(
            "huggingface_hub is not installed. "
            "Run this script with the GraspMolmo environment, for example:\n"
            "  /opt/graspmolmo_venv/bin/python download_checkpoint.py"
        ) from exc

    print(f"[GraspMolmo] Model: {MODEL_ID}")
    print(f"[GraspMolmo] HF_HOME: {hf_home}")
    print("[GraspMolmo] Downloading checkpoint...")

    snapshot_path = snapshot_download(
        repo_id=MODEL_ID,
        cache_dir=str(hf_hub_cache),
    )

    print("[GraspMolmo] Download completed.")
    print(f"[GraspMolmo] Snapshot: {snapshot_path}")
    print()
    print("Use the same cache when starting the server:")
    print(f"  export HF_HOME={hf_home}")


if __name__ == "__main__":
    main()
