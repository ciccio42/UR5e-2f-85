#!/usr/bin/env python3
"""
HTTP server for the M2T2 grasp proposal backend.

The server runs inside the dedicated M2T2 virtual environment and keeps
the model resident on the GPU.

Point clouds are transferred as base64-encoded NumPy .npy payloads to avoid
sending hundreds of thousands of XYZ values as JSON numbers.

Run:

    /opt/m2t2_venv/bin/python server.py \
        --config config/m2t2_config.yaml
"""

from __future__ import annotations

import argparse
import base64
import io
import sys
from pathlib import Path

import numpy as np
import yaml
from flask import Flask, jsonify, request


THIS_DIR = Path(__file__).resolve().parent


def _add_package_to_path() -> None:
    """
    Add the outer ROS Python package directory to sys.path.

    Layout:

        ai_controller/
            ai_controller/
                utils/
                    grasping/
                        m2t2/
    """

    outer_ai_controller = THIS_DIR.parents[3]

    if str(outer_ai_controller) not in sys.path:
        sys.path.insert(
            0,
            str(outer_ai_controller),
        )


_add_package_to_path()


from ai_controller.utils.grasping.m2t2.runtime import M2T2Runtime  # noqa: E402


def _decode_numpy_b64(
    encoded: str,
) -> np.ndarray:
    """
    Decode a base64-encoded NumPy .npy payload.
    """

    try:
        raw = base64.b64decode(
            encoded,
            validate=True,
        )
    except Exception as exc:
        raise ValueError(
            "Invalid base64 point cloud payload."
        ) from exc

    try:
        with io.BytesIO(raw) as buffer:
            array = np.load(
                buffer,
                allow_pickle=False,
            )
    except Exception as exc:
        raise ValueError(
            "Invalid NumPy point cloud payload."
        ) from exc

    array = np.asarray(
        array,
        dtype=np.float32,
    )

    if (
        array.ndim != 2
        or array.shape[1] < 3
    ):
        raise ValueError(
            "point_cloud must have shape (N, 3) "
            f"or (N, >=3), got {array.shape}"
        )

    return array[:, :3]


def create_app(
    config_path: str,
) -> Flask:

    app = Flask(__name__)

    with open(
        config_path,
        "r",
        encoding="utf-8",
    ) as stream:
        cfg = yaml.safe_load(
            stream
        ) or {}

    checkpoint_path = cfg.get(
        "checkpoint_path"
    )

    model_config_path = cfg.get(
        "model_config_path"
    )

    print(
        "[m2t2_server] Loading M2T2 runtime..."
    )

    runtime_kwargs = {}

    if checkpoint_path:
        runtime_kwargs[
            "checkpoint_path"
        ] = checkpoint_path

    if model_config_path:
        runtime_kwargs[
            "config_path"
        ] = model_config_path

    runtime = M2T2Runtime(
        **runtime_kwargs
    )

    runtime.load()

    print(
        "[m2t2_server] M2T2 runtime ready."
    )

    @app.route(
        "/health",
        methods=["GET"],
    )
    def health():
        return jsonify(
            {
                "status": "ok",
                "backend": "m2t2",
            }
        )

    @app.route(
        "/predict_grasps",
        methods=["POST"],
    )
    def predict_grasps():

        try:
            body = request.get_json(
                force=True
            )

            if (
                not isinstance(body, dict)
                or "point_cloud" not in body
            ):
                raise ValueError(
                    "Missing 'point_cloud' field."
                )

            point_cloud = _decode_numpy_b64(
                body["point_cloud"]
            )

            num_runs = body.get(
                "num_runs"
            )

            if num_runs is not None:
                num_runs = int(
                    num_runs
                )

            seed = body.get(
                "seed"
            )

            if seed is not None:
                seed = int(
                    seed
                )

            print(
                "[m2t2_server] "
                f"num_runs={num_runs}, seed={seed}"
            )

            grasps, contacts, confidence = (
                runtime.predict_grasps(
                    point_cloud,
                    num_runs=num_runs,
                    seed=seed,
                )
            )

            return jsonify(
                {
                    "grasps": grasps.tolist(),
                    "contacts": contacts.tolist(),
                    "confidence": confidence.tolist(),
                    "num_grasps": int(
                        grasps.shape[0]
                    ),
                }
            )

        except ValueError as exc:
            return jsonify(
                {
                    "error": str(exc)
                }
            ), 400

        except Exception as exc:
            print(
                "[m2t2_server] Inference error:",
                repr(exc),
            )

            return jsonify(
                {
                    "error": str(exc)
                }
            ), 500

    return app


def main() -> None:

    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "--config",
        default=str(
            THIS_DIR
            / "config"
            / "m2t2_config.yaml"
        ),
    )

    parser.add_argument(
        "--host",
        default=None,
    )

    parser.add_argument(
        "--port",
        type=int,
        default=None,
    )

    args = parser.parse_args()

    with open(
        args.config,
        "r",
        encoding="utf-8",
    ) as stream:
        cfg = yaml.safe_load(
            stream
        ) or {}

    host = (
        args.host
        or cfg.get(
            "server_host",
            "127.0.0.1",
        )
    )

    port = (
        args.port
        or int(
            cfg.get(
                "server_port",
                8781,
            )
        )
    )

    app = create_app(
        args.config
    )

    print(
        f"[m2t2_server] Listening on "
        f"{host}:{port}"
    )

    app.run(
        host=host,
        port=port,
        threaded=False,
    )


if __name__ == "__main__":
    main()