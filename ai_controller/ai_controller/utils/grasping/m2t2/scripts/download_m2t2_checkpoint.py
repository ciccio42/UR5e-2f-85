#!/usr/bin/env python3
"""
Download the generic M2T2 checkpoint used by this project.

Intended repository location:
    ai_controller/ai_controller/utils/grasping/m2t2/scripts/download_checkpoint.py

Output:
    <repo>/.runtime/m2t2_checkpoints/m2t2.pth
"""

from __future__ import annotations

import shutil
import urllib.request
from pathlib import Path

CHECKPOINT_URL = (
    "https://huggingface.co/wentao-yuan/m2t2/"
    "resolve/main/m2t2.pth?download=true"
)
CHECKPOINT_NAME = "m2t2.pth"


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
        "ai_controller/ai_controller/utils/grasping/m2t2/scripts/."
    )


def main() -> None:
    repo_root = find_repo_root()

    checkpoint_dir = (
        repo_root
        / ".runtime"
        / "m2t2_checkpoints"
    )
    checkpoint_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    destination = checkpoint_dir / CHECKPOINT_NAME
    temporary = destination.with_suffix(".pth.part")

    if destination.is_file():
        print("[M2T2] Checkpoint already exists:")
        print(f"  {destination}")
        return

    print("[M2T2] Downloading generic checkpoint:")
    print(f"  {CHECKPOINT_URL}")
    print("[M2T2] Destination:")
    print(f"  {destination}")

    try:
        request = urllib.request.Request(
            CHECKPOINT_URL,
            headers={
                "User-Agent": "UR5e-2f-85-checkpoint-downloader",
            },
        )

        with urllib.request.urlopen(request) as response, temporary.open("wb") as output:
            shutil.copyfileobj(
                response,
                output,
                length=1024 * 1024,
            )

        temporary.replace(destination)

    except Exception:
        temporary.unlink(missing_ok=True)
        raise

    print("[M2T2] Download completed.")
    print(f"[M2T2] Checkpoint: {destination}")


if __name__ == "__main__":
    main()
