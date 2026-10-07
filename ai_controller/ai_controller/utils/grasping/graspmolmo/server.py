#!/usr/bin/env python3
"""
GraspMolmo HTTP bridge server.

GraspMolmo runs in its own Python virtual environment because its
PyTorch / Transformers dependency stack must remain isolated from
the ROS / SeeDo Python environment.

The heavy GraspMolmo model is loaded once at server startup and kept
resident on the GPU. ROS / SeeDo processes communicate with this
server over local HTTP.

Run inside the Docker container with the dedicated GraspMolmo venv:

    /opt/graspmolmo_venv/bin/python \
    server.py \
    --config config/graspmolmo_config.yaml

The server intentionally contains no ROS dependencies.
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
from PIL import Image


THIS_DIR = Path(__file__).resolve().parent


def _add_package_to_path() -> None:
    """
    Add the outer ai_controller ROS package directory to sys.path.

    Layout:

        ai_controller/
        └── ai_controller/
            └── utils/
                └── grasping/
                    └── graspmolmo/
    """

    package_root = THIS_DIR.parents[3]

    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))

_add_package_to_path()


from ai_controller.utils.grasping.graspmolmo.runtime import (
    GraspMolmoRuntime,
)


def _decode_image(image_b64: str) -> np.ndarray:
    """
    Decode a base64-encoded PNG/JPEG image into an RGB uint8 array.
    """

    try:
        raw = base64.b64decode(
            image_b64,
            validate=True,
        )
    except Exception as exc:
        raise ValueError(
            "Invalid base64 image payload."
        ) from exc

    try:
        image = Image.open(io.BytesIO(raw)).convert("RGB")
    except Exception as exc:
        raise ValueError(
            "Unable to decode image payload."
        ) from exc

    return np.asarray(
        image,
        dtype=np.uint8,
    )


def create_app() -> Flask:
    """
    Create the Flask application and load GraspMolmo once.

    The server does not begin listening until the model has finished
    loading, so /health becomes available only when inference is ready.
    """

    app = Flask(__name__)

    print(
        "[graspmolmo_server] Loading GraspMolmo runtime..."
    )

    runtime = GraspMolmoRuntime()
    runtime.load()

    print(
        "[graspmolmo_server] GraspMolmo runtime ready."
    )

    @app.route(
        "/health",
        methods=["GET"],
    )
    def health():
        return jsonify(
            {
                "status": "ok",
                "backend": "graspmolmo",
            }
        )

    @app.route(
        "/predict_point",
        methods=["POST"],
    )
    def predict_point():
        body = request.get_json(
            force=True,
        )

        if "image" not in body:
            return jsonify(
                {
                    "error": (
                        "Missing required field: image"
                    )
                }
            ), 400

        if "task" not in body:
            return jsonify(
                {
                    "error": (
                        "Missing required field: task"
                    )
                }
            ), 400

        task = str(
            body["task"]
        ).strip()

        if not task:
            return jsonify(
                {
                    "error": (
                        "Task must not be empty."
                    )
                }
            ), 400

        try:
            rgb = _decode_image(
                body["image"]
            )

            verbosity = int(
                body.get(
                    "verbosity",
                    0,
                )
            )

            seed = body.get(
                "seed"
            )

            if seed is not None:
                seed = int(
                    seed
                )

            if verbosity >= 1:
                print(
                    "[graspmolmo_server] "
                    f"seed={seed}"
                )

            point = runtime.predict_point(
                rgb=rgb,
                task=task,
                verbosity=verbosity,
                seed=seed,
            )

        except ValueError as exc:
            return jsonify(
                {
                    "error": str(exc),
                }
            ), 400

        except Exception as exc:
            print(
                "[graspmolmo_server] "
                f"Inference error: {exc}"
            )

            return jsonify(
                {
                    "error": (
                        "GraspMolmo inference failed."
                    )
                }
            ), 500

        if point is None:
            return jsonify(
                {
                    "point": None,
                }
            )

        return jsonify(
            {
                "point": [
                    float(point[0]),
                    float(point[1]),
                ],
            }
        )

    return app


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
    )

    parser.add_argument(
        "--config",
        default=str(
            THIS_DIR
            / "config"
            / "graspmolmo_config.yaml"
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

    config_path = Path(
        args.config
    )

    if config_path.exists():
        with config_path.open(
            "r",
            encoding="utf-8",
        ) as stream:
            cfg = (
                yaml.safe_load(stream)
                or {}
            )
    else:
        cfg = {}

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
                8780,
            )
        )
    )

    app = create_app()

    print(
        "[graspmolmo_server] "
        f"Listening on {host}:{port}"
    )

    app.run(
        host=host,
        port=port,
        threaded=False,
    )


if __name__ == "__main__":
    main()