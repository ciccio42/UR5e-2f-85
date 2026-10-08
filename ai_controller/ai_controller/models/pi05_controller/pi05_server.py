#!/usr/bin/env python3
"""
PI0.5 HTTP bridge server.

Why this exists
---------------
The ROS2 ai_controller_node.py process uses cv_bridge and the system ROS
Python environment, while PI0.5 is executed through LeRobot in its own
Python environment with torch / transformers / scipy / CUDA dependencies.

To keep those software stacks isolated, the real PI05Controller runs in
this separate process and is exposed to ai_controller_node.py through a
small local HTTP API.

The matching pi05_client.py runs inside the ROS process and exposes the
same controller interface used by AIControllerNode:

    reset()
    load_command()
    inference()

Architecture
------------

    ai_controller_node.py
            |
            |  HTTP
            v
    pi05_client.py
            |
            v
    pi05_server.py
            |
            v
    PI05Controller
            |
            +--> pi05_utils.py
            |
            +--> PI05Runtime / LeRobot
            |
            +--> PI0.5 checkpoint

The server owns the heavy policy process and loads the model only once.

Run
---

Inside the Python environment containing PI0.5 / LeRobot dependencies:

    python3 pi05_server.py \
        --config pi05_config.yaml

Optional overrides:

    python3 pi05_server.py \
        --config pi05_config.yaml \
        --host 127.0.0.1 \
        --port 8771
"""

from __future__ import annotations

import argparse
import base64
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml
from flask import Flask, jsonify, request


# =============================================================================
# PATH SETUP
# =============================================================================

THIS_DIR = Path(__file__).resolve().parent


def _add_repo_to_path() -> None:
    """
    Make the outer ROS ai_controller package importable.

    Expected repository layout:

        ai_controller/
        └── ai_controller/
            └── models/
                └── pi05_controller/
                    ├── pi05.py
                    ├── pi05_utils.py
                    ├── pi05_controller.py
                    ├── pi05_server.py
                    └── pi05_client.py
    """

    # THIS_DIR:
    #
    #   .../ai_controller/ai_controller/models/pi05_controller
    #
    # THIS_DIR.parents[1]:
    #
    #   .../ai_controller/ai_controller
    #
    # outer:
    #
    #   .../ai_controller
    #
    repo_ai_controller_dir = THIS_DIR.parents[1]
    outer = repo_ai_controller_dir.parent

    if str(outer) not in sys.path:
        sys.path.insert(
            0,
            str(outer),
        )

    # pi05_controller.py currently uses local imports such as:
    #
    #   from pi05 import PI05Runtime
    #   from pi05_utils import ...
    #
    # Therefore make this directory directly importable as well.
    if str(THIS_DIR) not in sys.path:
        sys.path.insert(
            0,
            str(THIS_DIR),
        )


_add_repo_to_path()


from ai_controller.models.pi05_controller.pi05_controller import (  # noqa: E402
    PI05Controller,
)


# =============================================================================
# IMAGE TRANSPORT
# =============================================================================


def _decode_images(
    images_b64: list[str],
) -> list[np.ndarray]:
    """
    Decode images sent by PI05ControllerClient.

    Transport format:

        RGB uint8 NumPy image
            ↓ client
        RGB -> BGR
            ↓
        PNG encode
            ↓
        base64
            ↓ HTTP
        base64 decode
            ↓
        PNG decode BGR
            ↓
        BGR -> RGB
            ↓
        RGB uint8 NumPy image

    The PI05Controller therefore receives the same RGB representation that
    AIControllerNode originally acquired through cv_bridge.
    """

    if not isinstance(
        images_b64,
        list,
    ):
        raise TypeError(
            "'images' must be a list."
        )

    images: list[np.ndarray] = []

    for index, encoded in enumerate(
        images_b64
    ):

        if not isinstance(
            encoded,
            str,
        ):
            raise TypeError(
                f"images[{index}] must be a base64 string."
            )

        try:
            raw = base64.b64decode(
                encoded
            )

        except Exception as exc:
            raise ValueError(
                f"Could not decode base64 image {index}."
            ) from exc

        arr = np.frombuffer(
            raw,
            dtype=np.uint8,
        )

        bgr = cv2.imdecode(
            arr,
            cv2.IMREAD_COLOR,
        )

        if bgr is None:
            raise ValueError(
                f"cv2.imdecode failed for image {index}."
            )

        rgb = cv2.cvtColor(
            bgr,
            cv2.COLOR_BGR2RGB,
        )

        images.append(
            rgb
        )

    return images


# =============================================================================
# PI0.5 STATE TRANSPORT
# =============================================================================


def _decode_pi05_states(
    states_json: dict[str, Any],
) -> dict[str, Any]:
    """
    Reconstruct the PI0.5 robot-side state produced by
    AIControllerNode._build_pi05_state().

    Expected HTTP representation:

        {
            "joint_positions": [q1, ..., q6],
            "gripper_qpos": float,
            "eef_position": [x, y, z],
            "eef_quaternion": [qx, qy, qz, qw],
            "gripper_closed": bool
        }

    This is NOT yet observation.state.

    PI05Controller / pi05_utils later construct:

        [
            q1, q2, q3, q4, q5, q6,
            gripper,
            x, y, z, roll, pitch, yaw,
        ]

    and the serialized LeRobot processor performs the checkpoint
    normalization afterwards.
    """

    if not isinstance(
        states_json,
        dict,
    ):
        raise TypeError(
            "'states' must be a dictionary."
        )

    required = (
        "joint_positions",
        "gripper_qpos",
        "eef_position",
        "eef_quaternion",
        "gripper_closed",
    )

    missing = [
        key
        for key in required
        if key not in states_json
    ]

    if missing:
        raise KeyError(
            "PI0.5 states payload is missing fields: "
            f"{missing}"
        )

    joint_positions = np.asarray(
        states_json[
            "joint_positions"
        ],
        dtype=np.float64,
    )

    eef_position = np.asarray(
        states_json[
            "eef_position"
        ],
        dtype=np.float64,
    )

    eef_quaternion = np.asarray(
        states_json[
            "eef_quaternion"
        ],
        dtype=np.float64,
    )

    gripper_qpos = float(
        states_json[
            "gripper_qpos"
        ]
    )

    gripper_closed = bool(
        states_json[
            "gripper_closed"
        ]
    )

    # -------------------------------------------------------------------------
    # Shape validation
    # -------------------------------------------------------------------------

    if joint_positions.shape != (
        6,
    ):
        raise ValueError(
            "PI0.5 joint_positions must have shape (6,), "
            f"got {joint_positions.shape}."
        )

    if eef_position.shape != (
        3,
    ):
        raise ValueError(
            "PI0.5 eef_position must have shape (3,), "
            f"got {eef_position.shape}."
        )

    if eef_quaternion.shape != (
        4,
    ):
        raise ValueError(
            "PI0.5 eef_quaternion must have shape (4,), "
            f"got {eef_quaternion.shape}."
        )

    # -------------------------------------------------------------------------
    # Numerical validation
    # -------------------------------------------------------------------------

    if not np.all(
        np.isfinite(
            joint_positions
        )
    ):
        raise ValueError(
            "PI0.5 joint_positions contains non-finite values."
        )

    if not np.all(
        np.isfinite(
            eef_position
        )
    ):
        raise ValueError(
            "PI0.5 eef_position contains non-finite values."
        )

    if not np.all(
        np.isfinite(
            eef_quaternion
        )
    ):
        raise ValueError(
            "PI0.5 eef_quaternion contains non-finite values."
        )

    if not np.isfinite(
        gripper_qpos
    ):
        raise ValueError(
            "PI0.5 gripper_qpos is not finite."
        )

    if np.linalg.norm(
        eef_quaternion
    ) < 1e-8:
        raise ValueError(
            "PI0.5 eef_quaternion has near-zero norm."
        )

    return {
        "joint_positions":
            joint_positions,

        # Deliberately RAW.
        #
        # No normalization is performed here. The value is inserted into the
        # 13D observation.state by PI05Controller and normalized exactly once
        # by the serialized LeRobot checkpoint preprocessor.
        "gripper_qpos":
            gripper_qpos,

        "eef_position":
            eef_position,

        "eef_quaternion":
            eef_quaternion,

        "gripper_closed":
            gripper_closed,
    }


# =============================================================================
# FLASK APPLICATION
# =============================================================================


def create_app(
    config_path: str,
    task_name: str = "pick_place",
) -> Flask:
    """
    Create the PI0.5 HTTP service and load the controller once.

    Model loading happens BEFORE app.run(), therefore /health becomes
    available only after the PI0.5 checkpoint is completely ready.
    """

    app = Flask(
        __name__
    )

    # -------------------------------------------------------------------------
    # Heavy model initialization
    # -------------------------------------------------------------------------

    print(
        "[pi05_server] Loading PI05Controller "
        f"from {config_path} ..."
    )

    controller = PI05Controller(
        config_path,
        task_name=task_name,
    )

    controller.reset()

    print(
        "[pi05_server] Controller ready."
    )

    # =========================================================================
    # HEALTH
    # =========================================================================

    @app.route(
        "/health",
        methods=["GET"],
    )
    def health():

        runtime = getattr(
            controller,
            "_runtime",
            None,
        )

        return jsonify(
            {
                "status": "ok",
                "controller": "pi05",
                "model_loaded": runtime is not None,
            }
        )


    # =========================================================================
    # RESET
    # =========================================================================

    @app.route(
        "/reset",
        methods=["POST"],
    )
    def reset():
        """
        Reset episode-specific PI0.5 state.

        In particular this clears the internal LeRobot action queue.
        """

        controller.reset()

        return jsonify(
            {
                "ok": True,
            }
        )


    # =========================================================================
    # LOAD LANGUAGE COMMAND
    # =========================================================================

    @app.route(
        "/load_command",
        methods=["POST"],
    )
    def load_command():

        body = request.get_json(
            force=True
        )

        if not isinstance(
            body,
            dict,
        ):
            raise TypeError(
                "JSON body must be a dictionary."
            )

        if "demo_path" not in body:
            raise KeyError(
                "Missing 'demo_path' in /load_command request."
            )

        if "task_id" not in body:
            raise KeyError(
                "Missing 'task_id' in /load_command request."
            )

        command = controller.load_command(
            body[
                "demo_path"
            ],
            task_id=body[
                "task_id"
            ],
        )

        return jsonify(
            {
                "command":
                    controller.command
                    if controller.command is not None
                    else command,
            }
        )


    # =========================================================================
    # INFERENCE
    # =========================================================================

    @app.route(
        "/inference",
        methods=["POST"],
    )
    def inference():
        """
        Perform one PI0.5 controller step.

        Expected request JSON:

            {
                "images": [
                    "<base64 PNG>",
                    "<base64 PNG>",
                    "<base64 PNG>",
                    "<base64 PNG>"
                ],

                "states": {
                    "joint_positions": [6 floats],
                    "gripper_qpos": float,
                    "eef_position": [3 floats],
                    "eef_quaternion": [4 floats],
                    "gripper_closed": bool
                },

                "t": 0,
                "save_path": "/optional/path"
            }

        The controller receives the same standard interface used by the ROS
        node:

            controller.inference(
                input_data=[images, states],
                ...
            )

        and returns:

            [
                [
                    x,
                    y,
                    z,
                    qx,
                    qy,
                    qz,
                    qw,
                    gripper_position
                ]
            ]
        """

        body = request.get_json(
            force=True
        )

        if not isinstance(
            body,
            dict,
        ):
            raise TypeError(
                "JSON body must be a dictionary."
            )

        if "images" not in body:
            raise KeyError(
                "Missing 'images' in /inference request."
            )

        if "states" not in body:
            raise KeyError(
                "Missing 'states' in /inference request."
            )

        # ---------------------------------------------------------------------
        # Decode client payload
        # ---------------------------------------------------------------------

        images = _decode_images(
            body[
                "images"
            ]
        )

        states = _decode_pi05_states(
            body[
                "states"
            ]
        )

        t = int(
            body.get(
                "t",
                0,
            )
        )

        save_path = body.get(
            "save_path"
        )

        # ---------------------------------------------------------------------
        # Real controller inference
        # ---------------------------------------------------------------------

        out = controller.inference(
            input_data=[
                images,
                states,
            ],
            t=t,
            save_path=save_path,
        )

        # ---------------------------------------------------------------------
        # NumPy / tensors -> JSON-compatible representation
        # ---------------------------------------------------------------------

        actions = []

        for action_index, action in enumerate(
            out
        ):

            action_array = np.asarray(
                action,
                dtype=np.float64,
            )

            if action_array.shape != (
                8,
            ):
                raise RuntimeError(
                    "PI05Controller returned an unexpected action "
                    f"shape at index {action_index}: "
                    f"{action_array.shape}."
                )

            if not np.all(
                np.isfinite(
                    action_array
                )
            ):
                raise RuntimeError(
                    "PI05Controller returned non-finite action "
                    f"at index {action_index}: {action_array}"
                )

            actions.append(
                [
                    float(value)
                    for value in action_array
                ]
            )

        return jsonify(
            {
                "actions":
                    actions,
            }
        )


    return app


# =============================================================================
# MAIN
# =============================================================================


def main() -> None:

    parser = argparse.ArgumentParser(
        description=__doc__,
    )

    parser.add_argument(
        "--config",
        default=str(
            THIS_DIR
            / "pi05_config.yaml"
        ),
        help=(
            "Path to the PI0.5 runtime/controller YAML configuration."
        ),
    )

    parser.add_argument(
        "--task-name",
        default="pick_place",
    )

    parser.add_argument(
        "--host",
        default=None,
        help=(
            "HTTP bind address. "
            "If omitted, server_host from the YAML is used."
        ),
    )

    parser.add_argument(
        "--port",
        type=int,
        default=None,
        help=(
            "HTTP port. "
            "If omitted, server_port from the YAML is used."
        ),
    )

    args = parser.parse_args()

    # -------------------------------------------------------------------------
    # Server configuration
    # -------------------------------------------------------------------------

    with open(
        args.config,
        "r",
        encoding="utf-8",
    ) as stream:

        cfg = (
            yaml.safe_load(
                stream
            )
            or {}
        )

    host = (
        args.host
        or cfg.get(
            "server_host",
            "127.0.0.1",
        )
    )

    port = (
        args.port
        if args.port is not None
        else int(
            cfg.get(
                "server_port",
                8771,
            )
        )
    )

    # -------------------------------------------------------------------------
    # Load PI0.5 and start service
    # -------------------------------------------------------------------------

    app = create_app(
        args.config,
        task_name=args.task_name,
    )

    print(
        f"[pi05_server] Listening on "
        f"{host}:{port}"
    )

    # Single threaded deliberately:
    #
    # PI05Controller / PI05Runtime own mutable episode state, most notably
    # the internal LeRobot action queue. Concurrent inference/reset requests
    # must therefore not modify that state simultaneously.
    app.run(
        host=host,
        port=port,
        threaded=False,
    )


if __name__ == "__main__":
    main()