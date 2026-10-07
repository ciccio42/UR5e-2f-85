#!/usr/bin/env python3
"""
Thin HTTP client for the GraspMolmo backend.

This client runs in the ROS / SeeDo Python environment and communicates
with the GraspMolmo server running in its dedicated virtual environment.

The client has no dependency on torch, transformers or GraspMolmo itself.
"""

from __future__ import annotations

import base64
import io
import time
from pathlib import Path

import numpy as np
import requests
import yaml
from PIL import Image


THIS_DIR = Path(__file__).resolve().parent


class GraspMolmoRemoteUnavailable(RuntimeError):
    """Raised when the GraspMolmo HTTP server cannot be reached."""


def _encode_image_b64(
    image_rgb: np.ndarray,
) -> str:
    """
    Encode an RGB uint8 image as base64 PNG.
    """

    image_rgb = np.asarray(
        image_rgb,
        dtype=np.uint8,
    )

    if (
        image_rgb.ndim != 3
        or image_rgb.shape[2] != 3
    ):
        raise ValueError(
            "image_rgb must have shape (H, W, 3), "
            f"got {image_rgb.shape}"
        )

    image = Image.fromarray(
        image_rgb,
        mode="RGB",
    )

    buffer = io.BytesIO()

    image.save(
        buffer,
        format="PNG",
    )

    return base64.b64encode(
        buffer.getvalue()
    ).decode("ascii")


class GraspMolmoClient:
    """
    Thin HTTP client for task-oriented grasp-point prediction.
    """

    def __init__(
        self,
        config_path: str | None = None,
        connect_retries: int = 5,
        connect_retry_wait: float = 3.0,
    ) -> None:

        if config_path is None:
            config_path = str(
                THIS_DIR
                / "config"
                / "graspmolmo_config.yaml"
            )

        with open(
            config_path,
            "r",
            encoding="utf-8",
        ) as stream:
            cfg = yaml.safe_load(stream) or {}

        host = cfg.get(
            "server_host",
            "127.0.0.1",
        )

        port = int(
            cfg.get(
                "server_port",
                8780,
            )
        )

        self.base_url = (
            f"http://{host}:{port}"
        )

        last_exc = None

        for attempt in range(
            connect_retries
        ):
            try:
                response = requests.get(
                    f"{self.base_url}/health",
                    timeout=5.0,
                )

                response.raise_for_status()

                print(
                    "[GraspMolmoClient] "
                    f"Connected to {self.base_url}"
                )

                return

            except requests.RequestException as exc:
                last_exc = exc

                if (
                    attempt
                    < connect_retries - 1
                ):
                    time.sleep(
                        connect_retry_wait
                    )

        raise GraspMolmoRemoteUnavailable(
            "Could not reach GraspMolmo server at "
            f"{self.base_url} after "
            f"{connect_retries} attempts. "
            "Start graspmolmo/server.py first."
        ) from last_exc

    def predict_point(
        self,
        rgb: np.ndarray,
        task: str,
        verbosity: int = 0,
        timeout: float = 60.0,
        seed: int | None = None,
    ) -> np.ndarray | None:
        """
        Predict the task-oriented grasp point.

        Returns
        -------
        np.ndarray | None
            Pixel coordinates [x, y], or None if no valid point was predicted.
        """

        task = str(task).strip()

        if not task:
            raise ValueError(
                "task must not be empty."
            )

        payload = {
            "image": _encode_image_b64(
                rgb
            ),
            "task": task,
            "verbosity": int(
                verbosity
            ),
        }

        if seed is not None:
            payload["seed"] = int(
                seed
            )

        response = requests.post(
            f"{self.base_url}/predict_point",
            json=payload,
            timeout=timeout,
        )

        response.raise_for_status()

        point = response.json().get(
            "point"
        )

        if point is None:
            return None

        point = np.asarray(
            point,
            dtype=np.float32,
        )

        if point.shape != (2,):
            raise ValueError(
                "Invalid point returned by "
                "GraspMolmo server: "
                f"{point}"
            )

        return point