#!/usr/bin/env python3
"""
Thin HTTP client counterpart to pi05_server.py.

This module runs inside ai_controller_node.py's ROS / cv_bridge Python
environment.

The heavy PI0.5 / LeRobot policy is NOT imported here. It runs in
pi05_server.py inside a separate Python environment.

The client exposes the same interface expected by AIControllerNode:

    reset()
    load_command()
    inference()

Architecture
------------

    AIControllerNode
        |
        v
    PI05ControllerClient
        |
        | HTTP
        v
    pi05_server.py
        |
        v
    PI05Controller
        |
        v
    PI05Runtime / LeRobot / PI0.5
"""

from __future__ import annotations

import base64
import time

import cv2
import numpy as np
import requests
import yaml


# =============================================================================
# EXCEPTIONS
# =============================================================================


class PI05RemoteUnavailable(RuntimeError):
    """Raised when the PI0.5 inference server cannot be reached."""

    pass


# =============================================================================
# IMAGE SERIALIZATION
# =============================================================================


def _encode_image_b64(
    image_rgb,
) -> str:
    """
    Encode one RGB uint8 image as a base64 PNG.

    AIControllerNode provides RGB NumPy images.

    Transport:

        RGB uint8
            ↓
        BGR
            ↓
        PNG encode
            ↓
        base64 string

    pi05_server.py reverses this transformation and reconstructs RGB.
    """

    image = np.asarray(
        image_rgb
    )

    if (
        image.ndim != 3
        or image.shape[2] != 3
    ):
        raise ValueError(
            "PI0.5 image must have shape (H, W, 3), "
            f"got {image.shape}."
        )

    if image.dtype != np.uint8:
        raise TypeError(
            "PI0.5 image must have dtype uint8, "
            f"got {image.dtype}."
        )

    bgr = cv2.cvtColor(
        image,
        cv2.COLOR_RGB2BGR,
    )

    ok, buffer = cv2.imencode(
        ".png",
        bgr,
    )

    if not ok:
        raise ValueError(
            "cv2.imencode failed for PI0.5 image."
        )

    return base64.b64encode(
        buffer.tobytes()
    ).decode(
        "ascii"
    )


# =============================================================================
# PI0.5 STATE SERIALIZATION
# =============================================================================


def _serialize_pi05_states(
    states,
) -> dict:
    """
    Convert AIControllerNode's PI0.5 state dictionary to JSON-compatible
    Python types.

    Expected input:

        {
            "joint_positions": np.ndarray shape (6,),
            "gripper_qpos": float,
            "eef_position": np.ndarray shape (3,),
            "eef_quaternion": np.ndarray shape (4,),
            "gripper_closed": bool,
        }

    Important
    ---------
    No model preprocessing or normalization is performed here.

    In particular:

        gripper_qpos

    is transferred unchanged.

    The quaternion is also transported unchanged from ROS/TF. It will later
    be converted to Euler XYZ by PI05Controller/pi05_utils before the raw
    13D observation.state is constructed.

    LeRobot subsequently applies the checkpoint QUANTILES normalization.
    """

    if not isinstance(
        states,
        dict,
    ):
        raise TypeError(
            "PI0.5 states must be a dictionary, "
            f"got {type(states).__name__}."
        )

    required_keys = (
        "joint_positions",
        "gripper_qpos",
        "eef_position",
        "eef_quaternion",
        "gripper_closed",
    )

    missing = [
        key
        for key in required_keys
        if key not in states
    ]

    if missing:
        raise KeyError(
            "PI0.5 states missing required fields: "
            f"{missing}"
        )

    # -------------------------------------------------------------------------
    # Joint positions
    # -------------------------------------------------------------------------

    joint_positions = np.asarray(
        states[
            "joint_positions"
        ],
        dtype=np.float64,
    )

    if joint_positions.shape != (
        6,
    ):
        raise ValueError(
            "PI0.5 joint_positions must have shape (6,), "
            f"got {joint_positions.shape}."
        )

    if not np.all(
        np.isfinite(
            joint_positions
        )
    ):
        raise ValueError(
            "PI0.5 joint_positions contains non-finite values."
        )

    # -------------------------------------------------------------------------
    # Gripper proprioception
    #
    # Already a scalar in AIControllerNode._build_pi05_state().
    # No normalization is applied here.
    # -------------------------------------------------------------------------

    gripper_qpos = float(
        states[
            "gripper_qpos"
        ]
    )

    if not np.isfinite(
        gripper_qpos
    ):
        raise ValueError(
            "PI0.5 gripper_qpos is not finite."
        )

    # -------------------------------------------------------------------------
    # EEF position
    # -------------------------------------------------------------------------

    eef_position = np.asarray(
        states[
            "eef_position"
        ],
        dtype=np.float64,
    )

    if eef_position.shape != (
        3,
    ):
        raise ValueError(
            "PI0.5 eef_position must have shape (3,), "
            f"got {eef_position.shape}."
        )

    if not np.all(
        np.isfinite(
            eef_position
        )
    ):
        raise ValueError(
            "PI0.5 eef_position contains non-finite values."
        )

    # -------------------------------------------------------------------------
    # EEF quaternion
    #
    # Robot-side ROS/TF representation:
    #
    #     [qx, qy, qz, qw]
    #
    # This is NOT passed directly to the neural policy.
    # PI05Controller/pi05_utils later converts it to:
    #
    #     [roll, pitch, yaw]
    #
    # before constructing observation.state.
    # -------------------------------------------------------------------------

    eef_quaternion = np.asarray(
        states[
            "eef_quaternion"
        ],
        dtype=np.float64,
    )

    if eef_quaternion.shape != (
        4,
    ):
        raise ValueError(
            "PI0.5 eef_quaternion must have shape (4,), "
            f"got {eef_quaternion.shape}."
        )

    if not np.all(
        np.isfinite(
            eef_quaternion
        )
    ):
        raise ValueError(
            "PI0.5 eef_quaternion contains non-finite values."
        )

    if float(
        np.linalg.norm(
            eef_quaternion
        )
    ) < 1e-8:
        raise ValueError(
            "PI0.5 eef_quaternion has near-zero norm."
        )

    # -------------------------------------------------------------------------
    # Logical gripper state
    #
    # This is NOT part of the model's proprioceptive input.
    # It is used by PI05Controller for output gripper hysteresis.
    # -------------------------------------------------------------------------

    gripper_closed = bool(
        states[
            "gripper_closed"
        ]
    )

    return {
        "joint_positions":
            joint_positions.tolist(),

        "gripper_qpos":
            gripper_qpos,

        "eef_position":
            eef_position.tolist(),

        "eef_quaternion":
            eef_quaternion.tolist(),

        "gripper_closed":
            gripper_closed,
    }


# =============================================================================
# CLIENT
# =============================================================================


class PI05ControllerClient:
    """
    Thin HTTP client with the same interface expected by AIControllerNode.

    Construction:

        PI05ControllerClient(
            model_config_path,
            task_name,
        )

    The configuration file is only read here to retrieve the HTTP server
    address and timeout values. The real PI0.5 controller reads the same YAML
    independently inside pi05_server.py.
    """

    def __init__(
        self,
        model_config,
        task_name: str = "pick_place",
        connect_retries: int = 5,
        connect_retry_wait: float = 3.0,
    ):

        # ---------------------------------------------------------------------
        # HTTP configuration
        # -------------------------------------------------------------------------

        with open(
            model_config,
            "r",
            encoding="utf-8",
        ) as stream:

            cfg = (
                yaml.safe_load(
                    stream
                )
                or {}
            )

        host = cfg.get(
            "server_host",
            "127.0.0.1",
        )

        port = int(
            cfg.get(
                "server_port",
                8771,
            )
        )

        self.base_url = (
            f"http://{host}:{port}"
        )

        self.task_name = (
            task_name
        )

        self.command = None

        self.inference_timeout = float(
            cfg.get(
                "server_inference_timeout",
                120.0,
            )
        )

        # ---------------------------------------------------------------------
        # Wait for server availability
        # -------------------------------------------------------------------------

        last_exception = None

        for attempt in range(
            connect_retries
        ):

            try:

                response = requests.get(
                    f"{self.base_url}/health",
                    timeout=5.0,
                )

                response.raise_for_status()

                payload = response.json()

                if payload.get(
                    "status"
                ) != "ok":
                    raise PI05RemoteUnavailable(
                        "PI0.5 server returned an unexpected "
                        f"health response: {payload}"
                    )

                print(
                    "[PI05ControllerClient] "
                    f"Connected to {self.base_url}"
                )

                return

            except (
                requests.RequestException,
                PI05RemoteUnavailable,
            ) as exc:

                last_exception = exc

                if attempt < (
                    connect_retries - 1
                ):

                    print(
                        "[PI05ControllerClient] "
                        "Server not ready "
                        f"(attempt {attempt + 1}/"
                        f"{connect_retries}). "
                        f"Retrying in {connect_retry_wait}s..."
                    )

                    time.sleep(
                        connect_retry_wait
                    )

        raise PI05RemoteUnavailable(
            "Could not reach PI0.5 server at "
            f"{self.base_url} after "
            f"{connect_retries} attempts. "
            "Start pi05_server.py first, for example:\n"
            f"  python3 pi05_server.py --config {model_config}"
        ) from last_exception


    # =========================================================================
    # RESET
    # =========================================================================

    def reset(
        self,
    ):
        """
        Reset episode-specific PI0.5 state on the server.

        In particular, the server-side PI05Runtime clears the LeRobot action
        queue.
        """

        response = requests.post(
            f"{self.base_url}/reset",
            timeout=15.0,
        )

        response.raise_for_status()

        self.command = None


    # =========================================================================
    # COMMAND
    # =========================================================================

    def load_command(
        self,
        demo_path,
        task_id,
        **kwargs,
    ):
        """
        Ask the server-side PI05Controller to select the prompt associated
        with task_id.

        demo_path is kept in the interface for compatibility with
        AIControllerNode and the other controllers. PI0.5 currently uses the
        static task prompt configured in pi05_config.yaml.
        """

        del kwargs

        response = requests.post(
            f"{self.base_url}/load_command",
            json={
                "demo_path":
                    str(demo_path),

                "task_id":
                    str(task_id),
            },
            timeout=30.0,
        )

        response.raise_for_status()

        payload = response.json()

        if "command" not in payload:
            raise RuntimeError(
                "PI0.5 server /load_command response "
                "does not contain 'command'."
            )

        self.command = str(
            payload[
                "command"
            ]
        )

        return self.command


    # =========================================================================
    # INFERENCE
    # =========================================================================

    def inference(
        self,
        input_data,
        t: int = 0,
        save_path=None,
    ):
        """
        Perform one remote PI0.5 controller step.

        Expected input_data:

            [
                images,
                states,
            ]

        where:

            images:
                list of RGB uint8 NumPy camera frames.

            states:
                {
                    "joint_positions": shape (6,),
                    "gripper_qpos": float,
                    "eef_position": shape (3,),
                    "eef_quaternion": shape (4,),
                    "gripper_closed": bool,
                }

        The server returns a list of absolute UR5e targets:

            [
                [
                    x,
                    y,
                    z,
                    qx,
                    qy,
                    qz,
                    qw,
                    gripper_position,
                ]
            ]
        """

        if (
            not isinstance(
                input_data,
                (list, tuple),
            )
            or len(input_data) != 2
        ):
            raise ValueError(
                "PI0.5 client expects input_data=[images, states]."
            )

        images, states = (
            input_data
        )

        if not isinstance(
            images,
            (list, tuple),
        ):
            raise TypeError(
                "PI0.5 images must be a list or tuple."
            )

        # Current node has four cameras:
        #
        #   0 front
        #   1 left
        #   2 right
        #   3 gripper
        #
        # All are transported unchanged. PI05Controller selects front and
        # gripper using the indexes configured in pi05_config.yaml.
        encoded_images = [
            _encode_image_b64(
                image
            )
            for image in images
        ]

        serialized_states = (
            _serialize_pi05_states(
                states
            )
        )

        payload = {
            "images":
                encoded_images,

            "states":
                serialized_states,

            "t":
                int(t),

            "save_path":
                (
                    str(save_path)
                    if save_path is not None
                    else None
                ),
        }

        response = requests.post(
            f"{self.base_url}/inference",
            json=payload,
            timeout=self.inference_timeout,
        )

        response.raise_for_status()

        result = response.json()

        if "actions" not in result:
            raise RuntimeError(
                "PI0.5 server /inference response "
                "does not contain 'actions'."
            )

        actions = result[
            "actions"
        ]

        if not isinstance(
            actions,
            list,
        ):
            raise TypeError(
                "PI0.5 server returned invalid 'actions' type: "
                f"{type(actions).__name__}."
            )

        # ---------------------------------------------------------------------
        # Validate server output before giving it back to AIControllerNode.
        # -------------------------------------------------------------------------

        validated_actions = []

        for index, action in enumerate(
            actions
        ):

            action_array = np.asarray(
                action,
                dtype=np.float64,
            )

            if action_array.shape != (
                8,
            ):
                raise ValueError(
                    "PI0.5 server returned action "
                    f"{index} with shape {action_array.shape}; "
                    "expected (8,)."
                )

            if not np.all(
                np.isfinite(
                    action_array
                )
            ):
                raise ValueError(
                    "PI0.5 server returned non-finite "
                    f"action {index}: {action_array}"
                )

            # Keep the controller convention already used by the node:
            #
            # list[list[float]]
            #
            validated_actions.append(
                [
                    float(value)
                    for value in action_array
                ]
            )

        return validated_actions