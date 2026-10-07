#!/usr/bin/env python3
"""
Thin HTTP client for the M2T2 grasp proposal backend.
"""

from __future__ import annotations

import base64
import io
import time
from pathlib import Path

import numpy as np
import requests
import yaml


THIS_DIR = Path(__file__).resolve().parent


class M2T2RemoteUnavailable(RuntimeError):
    pass


def _encode_numpy_b64(
    array: np.ndarray,
) -> str:
    """
    Encode a NumPy array as a base64-encoded .npy payload.
    """

    array = np.asarray(
        array,
        dtype=np.float32,
    )

    buffer = io.BytesIO()

    np.save(
        buffer,
        array,
        allow_pickle=False,
    )

    return base64.b64encode(
        buffer.getvalue()
    ).decode("ascii")


class M2T2Client:
    """
    Thin HTTP client for generic M2T2 grasp proposals.
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
                / "m2t2_config.yaml"
            )

        with open(
            config_path,
            "r",
            encoding="utf-8",
        ) as stream:
            cfg = yaml.safe_load(
                stream
            ) or {}

        host = cfg.get(
            "server_host",
            "127.0.0.1",
        )

        port = int(
            cfg.get(
                "server_port",
                8781,
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
                    "[M2T2Client] Connected to "
                    f"{self.base_url}"
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

        raise M2T2RemoteUnavailable(
            "Could not reach M2T2 server at "
            f"{self.base_url} after "
            f"{connect_retries} attempts."
        ) from last_exc

    def predict_grasps(
        self,
        point_cloud: np.ndarray,
        num_runs: int | None = None,
        timeout: float = 60.0,
        seed=None,
    ) -> tuple[
        np.ndarray,
        np.ndarray,
        np.ndarray,
    ]:

        point_cloud = np.asarray(
            point_cloud,
            dtype=np.float32,
        )

        if (
            point_cloud.ndim != 2
            or point_cloud.shape[1] < 3
        ):
            raise ValueError(
                "point_cloud must have shape "
                f"(N, 3) or (N, >=3), got "
                f"{point_cloud.shape}"
            )

        payload = {
            "point_cloud": _encode_numpy_b64(
                point_cloud
            ),
        }

        if num_runs is not None:
            payload["num_runs"] = int(
                num_runs
            )

        if seed is not None:
            payload["seed"] = int(
                seed
            )

        response = requests.post(
            f"{self.base_url}/predict_grasps",
            json=payload,
            timeout=timeout,
        )

        response.raise_for_status()

        result = response.json()

        grasps = np.asarray(
            result["grasps"],
            dtype=np.float32,
        )

        contacts = np.asarray(
            result["contacts"],
            dtype=np.float32,
        )

        confidence = np.asarray(
            result["confidence"],
            dtype=np.float32,
        )

        if grasps.size == 0:
            grasps = np.empty(
                (0, 4, 4),
                dtype=np.float32,
            )

        if contacts.size == 0:
            contacts = np.empty(
                (0, 3),
                dtype=np.float32,
            )

        if confidence.size == 0:
            confidence = np.empty(
                (0,),
                dtype=np.float32,
            )

        return (
            grasps,
            contacts,
            confidence,
        )