#!/usr/bin/env python3
"""
Thin HTTP client for the Interleave-Pi0 bridge server.

This module runs inside ai_controller_node.py's ROS/system Python environment.

Its responsibilities are limited to:
    - connecting to the Interleave-Pi0 HTTP server;
    - losslessly encoding RGB images as PNG for transport;
    - serializing robot state and request metadata;
    - forwarding reset/load_command/inference calls;
    - returning the absolute robot actions produced by the remote controller.

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


class InterleavePi0RemoteUnavailable(RuntimeError):
    """Raised when the Interleave-Pi0 server cannot be reached."""


# =============================================================================
# IMAGE TRANSPORT
# =============================================================================


def _encode_image_b64(image_rgb) -> str:
    """
    Encode one RGB uint8 image losslessly as PNG + base64.

    Transport pipeline:

        RGB uint8
            -> RGB to BGR
            -> PNG encode
            -> base64 string

    The server performs the exact inverse transformation:

        base64
            -> PNG decode
            -> BGR to RGB

    PNG is used only as a transport representation. No JPEG compression is
    introduced by the HTTP bridge.
    """

    image = np.asarray(image_rgb)

    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(
            "Interleave-Pi0 client expected an RGB image with shape "
            f"(H, W, 3), got {image.shape}."
        )

    if image.dtype != np.uint8:
        raise TypeError(
            "Interleave-Pi0 client expected an RGB uint8 image, "
            f"got dtype {image.dtype}."
        )

    # OpenCV encoders expect BGR convention.
    bgr = cv2.cvtColor(
        image,
        cv2.COLOR_RGB2BGR,
    )

    ok, encoded = cv2.imencode(
        ".png",
        bgr,
    )

    if not ok:
        raise ValueError(
            "cv2.imencode('.png') failed for an image sent to the "
            "Interleave-Pi0 server."
        )

    return base64.b64encode(
        encoded.tobytes()
    ).decode(
        "ascii"
    )


# =============================================================================
# CLIENT
# =============================================================================


class InterleavePi0ControllerClient:
    """
    Thin client exposing the same public call surface used by AIControllerNode.

    Construction intentionally matches the real controller:

        InterleavePi0ControllerClient(
            model_config,
            task_name,
        )

    so AIControllerNode only needs to instantiate this class instead of
    InterleavePi0Controller.

    The client does NOT maintain the Interleave action buffer. The remote
    InterleavePi0Controller remains the sole owner of:

        action_buffer
        action_idx
        command
        instruction_images
        gripper state
        model / processor state
    """

    def __init__(
        self,
        model_config: str,
        task_name: str = "pick_place",
        connect_retries: int = 5,
        connect_retry_wait: float = 3.0,
    ) -> None:

        # ---------------------------------------------------------------------
        # Server configuration
        # ---------------------------------------------------------------------

        with open(
            model_config,
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
                8771,
            )
        )

        self.base_url = (
            f"http://{host}:{port}"
        )

        self.task_name = task_name

        # Shadow state only.
        #
        # The authoritative command remains inside the remote controller.
        self.command = None

        # First torch.compile-backed inference may be significantly slower than
        # subsequent calls, therefore use a generous default timeout.
        # self.inference_timeout = float(
        #     cfg.get(
        #         "server_inference_timeout",
        #         180.0,
        #     )
        # )

        # self.load_command_timeout = float(
        #     cfg.get(
        #         "server_load_command_timeout",
        #         30.0,
        #     )
        # )

        # self.reset_timeout = float(
        #     cfg.get(
        #         "server_reset_timeout",
        #         15.0,
        #     )
        # )

        # Reuse the local HTTP connection across controller steps.
        self.session = requests.Session()

        # ---------------------------------------------------------------------
        # Server availability check
        # ---------------------------------------------------------------------

        last_exc = None

        for attempt in range(
            connect_retries
        ):
            try:
                response = self.session.get(
                    f"{self.base_url}/health",
                    timeout=5.0,
                )

                response.raise_for_status()

                print(
                    "[InterleavePi0ControllerClient] "
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

        raise InterleavePi0RemoteUnavailable(
            "Could not reach the Interleave-Pi0 server at "
            f"{self.base_url} after {connect_retries} attempts. "
            "Start server.py first inside the dedicated Interleave "
            "Conda environment."
        ) from last_exc

    # =========================================================================
    # RESET
    # =========================================================================

    def reset(self) -> None:
        """
        Reset rollout-specific state in the remote controller.

        Model, processor and checkpoint remain loaded server-side.
        """
        try:
            response = self.session.post(
                f"{self.base_url}/reset",
                #timeout=self.reset_timeout,
            )

            response.raise_for_status()
        except requests.RequestException as exc:
            raise RuntimeError(
                f"Interleave-Pi0 server reset request failed: {exc}"
            ) from exc

        self.command = None

    # =========================================================================
    # LOAD COMMAND
    # =========================================================================

    def load_command(
        self,
        demo_path,
        task_id,
        **kwargs,
    ):
        """
        Forward task selection to the remote InterleavePi0Controller.

        The server-side controller remains responsible for:
            - YAML task lookup;
            - prompt preparation;
            - instruction-image loading;
            - dataset-statistics loading;
            - action-buffer reset.
        """

        try:
            response = self.session.post(
                f"{self.base_url}/load_command",
                json={
                    "task_id": task_id,
                },
                #timeout=self.load_command_timeout,
            )

            response.raise_for_status()
        except requests.RequestException as exc:
            raise RuntimeError(
                f"Interleave-Pi0 server load_command request failed: {exc}"
            ) from exc

        

        payload = response.json()

        self.command = payload[
            "command"
        ]

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
        Request one Interleave controller action.

        input_data:
            [images, robot_state]

        images:
            RGB uint8 images collected by AIControllerNode.

        robot_state:
            [x, y, z, qx, qy, qz, qw, gripper_closed]

        t:
            global ROS rollout step.

            It does NOT identify the action inside the Interleave chunk.
            Chunk progression is owned entirely by the remote controller through
            its action_buffer/action_idx state.

        Returns:
            [
                [x, y, z, qx, qy, qz, qw, gripper]
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
                "input_data must be [images, robot_state]."
            )

        images, robot_state = (
            input_data
        )

        if images is None:
            raise ValueError(
                "Interleave-Pi0 requires camera images."
            )

        images = list(
            images
        )

        if not images:
            raise ValueError(
                "Interleave-Pi0 requires at least the front-camera image."
            )

        # ---------------------------------------------------------------------
        # Robot state
        # ---------------------------------------------------------------------

        robot_state = np.asarray(
            robot_state,
            dtype=np.float64,
        )

        if robot_state.shape != (8,):
            raise ValueError(
                "robot_state must have shape (8,), "
                "[x, y, z, qx, qy, qz, qw, gripper_closed], "
                f"got {robot_state.shape}."
            )

        if not np.all(
            np.isfinite(
                robot_state
            )
        ):
            raise ValueError(
                "robot_state contains non-finite values."
            )

        # ---------------------------------------------------------------------
        # HTTP request
        # ---------------------------------------------------------------------
        #
        # Important:
        # the client deliberately does NOT inspect whether the remote action
        # buffer is empty.
        #
        # Every AIControllerNode inference call is forwarded to the server.
        # The real InterleavePi0Controller decides whether to:
        #
        #   - execute a new model query, or
        #   - consume the next buffered action.
        # ---------------------------------------------------------------------

        payload = {
            "images": [
                _encode_image_b64(
                    images[0]
                )
            ],

            "robot_state":
                robot_state.tolist(),

            "t":
                int(t),

            "save_path": (
                str(save_path)
                if save_path is not None
                else None
            ),
        }

        try:
            response = self.session.post(
                f"{self.base_url}/inference",
                json=payload,
            )
            response.raise_for_status()

        except requests.RequestException as exc:
            raise RuntimeError(
                f"Interleave-Pi0 server inference request failed: {exc}"
            ) from exc


        response_payload = (
            response.json()
        )

        if "actions" not in response_payload:
            raise RuntimeError(
                "Interleave-Pi0 server response does not contain "
                "'actions'."
            )

        actions = response_payload[
            "actions"
        ]

        # Current Interleave controller contract:
        #
        #     [
        #         [x, y, z, qx, qy, qz, qw, gripper]
        #     ]
        #
        # Keep the same ordinary-Python-list representation expected by
        # AIControllerNode.
        if (
            not isinstance(
                actions,
                list,
            )
            or len(actions) != 1
        ):
            raise RuntimeError(
                "Unexpected Interleave-Pi0 server action response: "
                f"{actions!r}"
            )

        action = np.asarray(
            actions[0],
            dtype=np.float64,
        )

        if action.shape != (8,):
            raise RuntimeError(
                "Interleave-Pi0 server returned an action with "
                f"shape {action.shape}, expected (8,)."
            )

        if not np.all(
            np.isfinite(
                action
            )
        ):
            raise RuntimeError(
                "Interleave-Pi0 server returned non-finite action values."
            )

        return [
            [
                float(value)
                for value in action
            ]
        ]

    # =========================================================================
    # CLEANUP
    # =========================================================================

    def close(self) -> None:
        """
        Close the client's HTTP session.

        This does not stop the remote model server.
        """

        self.session.close()