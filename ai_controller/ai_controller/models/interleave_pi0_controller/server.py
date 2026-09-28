#!/usr/bin/env python3
"""
Interleave-Pi0 HTTP bridge server.

The real InterleavePi0Controller runs in a dedicated Conda environment,
separated from ai_controller_node.py's ROS Python environment.

The server owns the complete Interleave runtime:

    raw RGB images + robot state
        -> InterleavePi0Controller.pre_process()
        -> Interleave-Pi0 inference
        -> InterleavePi0Controller.post_process()
        -> internal action buffer
        -> one absolute robot action

ai_controller_node.py communicates with this process through the thin
InterleavePi0ControllerClient.

The HTTP image transport uses lossless PNG encoding. The decoded RGB uint8
image is passed unchanged to InterleavePi0Controller, which then applies the
same crop/resize/model preprocessing used by the current runtime.

Run from the Interleave Conda environment, for example:

    python server.py \
        --config interleave_pi0_grounding_bin_config.yaml

The process also requires the same runtime environment variables used by the
current Interleave config, in particular:

    INTERLEAVE_PI0_CHECKPOINT
    INTERLEAVE_PI0_PALIGEMMA

and the open-pi-zero source tree must be available on PYTHONPATH.
"""

from __future__ import annotations

import argparse
import base64
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from flask import Flask, jsonify, request


THIS_DIR = Path(__file__).resolve().parent


# =============================================================================
# PYTHON PATH
# =============================================================================


def _add_repo_to_path() -> None:
    """
    Make the ROS ai_controller package and the controller-local modules
    importable when server.py is launched directly.

    Directory layout:

        ai_controller/
        └── ai_controller/
            └── models/
                └── interleave_pi0_controller/
                    ├── server.py
                    ├── interleave_pi0_controller.py
                    ├── interleave_pi0.py
                    └── interleave_pi0_utils.py

    `interleave_pi0_controller.py` still uses local imports such as:

        from interleave_pi0 import ...
        from interleave_pi0_utils import ...

    so THIS_DIR must also be on sys.path.

    The external open-pi-zero source directory is intentionally NOT resolved
    here. It must be exposed by the server launcher / Conda runtime through
    PYTHONPATH, exactly as part of the model runtime environment.
    """

    # .../ai_controller/ai_controller
    package_dir = THIS_DIR.parents[1]

    # .../ai_controller
    package_root = package_dir.parent

    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))

    if str(THIS_DIR) not in sys.path:
        sys.path.insert(0, str(THIS_DIR))


_add_repo_to_path()


from ai_controller.models.interleave_pi0_controller.interleave_pi0_controller import (  # noqa: E402
    InterleavePi0Controller,
)


# =============================================================================
# IMAGE TRANSPORT
# =============================================================================


def _decode_image_b64(image_b64: str) -> np.ndarray:
    """
    Decode one losslessly transported image.

    Expected client pipeline:

        RGB uint8
            -> RGB to BGR
            -> cv2.imencode(".png")
            -> base64

    Server pipeline:

        base64
            -> PNG bytes
            -> cv2.imdecode()
            -> BGR to RGB
            -> RGB uint8

    """

    if not isinstance(image_b64, str):
        raise TypeError(
            "Encoded image must be a base64 string, "
            f"got {type(image_b64).__name__}."
        )

    raw = base64.b64decode(
        image_b64,
        validate=True,
    )

    encoded = np.frombuffer(
        raw,
        dtype=np.uint8,
    )

    bgr = cv2.imdecode(
        encoded,
        cv2.IMREAD_COLOR,
    )

    if bgr is None:
        raise ValueError(
            "cv2.imdecode failed for an image received by the "
            "Interleave-Pi0 server."
        )

    rgb = cv2.cvtColor(
        bgr,
        cv2.COLOR_BGR2RGB,
    )

    if rgb.dtype != np.uint8:
        raise RuntimeError(
            f"Decoded image has unexpected dtype {rgb.dtype}."
        )

    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise RuntimeError(
            "Decoded image must have shape (H, W, 3), "
            f"got {rgb.shape}."
        )

    return rgb


def _decode_images(images_b64) -> list[np.ndarray]:
    """
    Decode the list of RGB images sent by the client.

    Interleave currently uses images[0] as the live front-camera image.
    Keeping a list here preserves the same input interface already used by
    InterleavePi0Controller:

        input_data = [images, robot_state]
    """

    if not isinstance(images_b64, list):
        raise TypeError(
            "Request field 'images' must be a list."
        )

    if not images_b64:
        raise ValueError(
            "Interleave-Pi0 requires at least the front-camera image."
        )

    return [
        _decode_image_b64(image_b64)
        for image_b64 in images_b64
    ]


# =============================================================================
# FLASK APP
# =============================================================================


def create_app(
    config_path: str,
    task_name: str = "pick_place",
) -> Flask:
    """
    Create the Flask application and one persistent Interleave controller.

    The controller is instantiated exactly once.

    Therefore the following state remains server-side for the lifetime of the
    process:

        - model and processor;
        - loaded checkpoint;
        - dataset statistics;
        - current task / command;
        - instruction images;
        - gripper state;
        - internal action buffer.
    """

    app = Flask(__name__)

    print(
        "[interleave_pi0_server] "
        f"Loading InterleavePi0Controller from {config_path} ..."
    )

    controller = InterleavePi0Controller(
        config_path,
        task_name=task_name,
    )

    # Start from a clean rollout state while keeping model/processor loaded.
    controller.reset()

    print(
        "[interleave_pi0_server] "
        "Controller ready."
    )

    # -------------------------------------------------------------------------
    # HEALTH
    # -------------------------------------------------------------------------

    @app.route("/health", methods=["GET"])
    def health():
        return jsonify(
            {
                "status": "ok",
                "controller": "interleave_pi0",
            }
        )

    # -------------------------------------------------------------------------
    # RESET
    # -------------------------------------------------------------------------

    @app.route("/reset", methods=["POST"])
    def reset():
        """
        Reset rollout-specific controller state.

        The Interleave model itself remains loaded.
        """

        controller.reset()

        return jsonify(
            {
                "ok": True,
            }
        )

    # -------------------------------------------------------------------------
    # LOAD COMMAND
    # -------------------------------------------------------------------------

    @app.route("/load_command", methods=["POST"])
    def load_command():
        """
        Select the Interleave task.

        The controller remains responsible for:
            - task lookup in the YAML;
            - prompt preparation;
            - instruction-image loading;
            - dataset-statistics loading;
            - action-buffer invalidation.
        """

        body = request.get_json(
            force=True,
        )

        if "task_id" not in body:
            raise KeyError(
                "Missing required request field 'task_id'."
            )

        task_id = body["task_id"]

        controller.load_command(
            demo_path="",
            task_id=task_id,
        )

        return jsonify(
            {
                "command": controller.command,
                "task_id": controller.current_task_id,
            }
        )

    # -------------------------------------------------------------------------
    # INFERENCE
    # -------------------------------------------------------------------------

    @app.route("/inference", methods=["POST"])
    def inference():
        """
        Execute one call to InterleavePi0Controller.inference().

        IMPORTANT:
        one HTTP request corresponds to one controller action.

        If the controller's internal action buffer is empty, this call performs:

            decode transport
                -> pre_process()
                -> Interleave-Pi0 inference
                -> post_process()
                -> fill action buffer
                -> return action[0]

        If buffered actions remain:

            decode transport
                -> controller.inference()
                -> consume next buffered action

        The controller therefore retains exactly the same action-buffer
        semantics as the current non-HTTP implementation.
        """

        body = request.get_json(
            force=True,
        )

        if "images" not in body:
            raise KeyError(
                "Missing required request field 'images'."
            )

        if "robot_state" not in body:
            raise KeyError(
                "Missing required request field 'robot_state'."
            )

        # ---------------------------------------------------------------------
        # Transport decoding
        # ---------------------------------------------------------------------

        images = _decode_images(
            body["images"]
        )

        robot_state = np.asarray(
            body["robot_state"],
            dtype=np.float64,
        )

        if robot_state.shape != (8,):
            raise ValueError(
                "robot_state must have shape (8,), "
                "[x, y, z, qx, qy, qz, qw, gripper_closed], "
                f"got {robot_state.shape}."
            )

        if not np.all(
            np.isfinite(robot_state)
        ):
            raise ValueError(
                "robot_state contains non-finite values."
            )

        t = int(
            body.get(
                "t",
                0,
            )
        )

        save_path = body.get(
            "save_path",
            None,
        )

        # ---------------------------------------------------------------------
        # Complete Interleave controller call
        # ---------------------------------------------------------------------
        #
        # pre_process + inference + post_process + action-buffer handling all
        # remain inside InterleavePi0Controller.
        #
        # The server does not reproduce any model-specific computation.
        # ---------------------------------------------------------------------

        out = controller.inference(
            input_data=[
                images,
                robot_state,
            ],
            t=t,
            save_path=save_path,
        )

        # Current controller contract:
        #
        #     [
        #         [x, y, z, qx, qy, qz, qw, gripper]
        #     ]
        #
        # Convert explicitly to ordinary Python floats before JSON encoding.
        actions = [
            [
                float(value)
                for value in np.asarray(
                    action,
                    dtype=np.float64,
                )
            ]
            for action in out
        ]

        return jsonify(
            {
                "actions": actions,
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
            / "interleave_pi0_config.yaml"
        ),
        help="Interleave-Pi0 runtime YAML.",
    )

    parser.add_argument(
        "--task-name",
        default="pick_place",
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

    # Read only server_host/server_port here.
    #
    # The actual Interleave configuration is loaded and resolved by
    # InterleavePi0Policy through OmegaConf when the controller is created.
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

    # Keep VLA-JEPA on 8770 and reserve 8771 for Interleave.
    port = (
        args.port
        or int(
            cfg.get(
                "server_port",
                8771,
            )
        )
    )

    app = create_app(
        args.config,
        task_name=args.task_name,
    )

    print(
        "[interleave_pi0_server] "
        f"Listening on {host}:{port}"
    )

    # The controller is stateful:
    #
    #   - current task
    #   - gripper state
    #   - action_buffer
    #   - action_idx
    #
    # Requests must therefore be processed sequentially.
    app.run(
        host=host,
        port=port,
        threaded=False,
    )


if __name__ == "__main__":
    main()