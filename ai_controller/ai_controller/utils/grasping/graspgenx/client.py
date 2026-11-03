from __future__ import annotations

import os
import subprocess
import tempfile
from pathlib import Path
import json
import select
import threading
import time

import numpy as np


THIS_DIR = Path(__file__).resolve().parent


class GraspGenXRemoteUnavailable(RuntimeError):
    pass


class GraspGenXRemoteClient:
    """
    Thin bridge from the SeeDo environment to the official
    GraspGen-X ZMQ client running inside the dedicated
    GraspGen-X Python environment.

    The SeeDo environment does not need graspgenx, pyzmq,
    msgpack, or other GraspGen-X dependencies installed.
    """

    WORKER_RESPONSE_PREFIX = (
        "__GRASPGENX_WORKER_RESPONSE__"
    )

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 8782,
        gripper_name: str = "robotiq_2f_85",
        python_executable: str = (
            "/opt/graspgenx_venv/bin/python"
        ),
        graspgenx_root: str = "/opt/GraspGenX",
        checkpoint_dir: str = (
            "/opt/graspgenx_checkpoints"
        ),
        gripper_cfg_dir: str = (
            "/opt/graspgenx_gripper_descriptions"
        ),
    ) -> None:
        self.host = host
        self.port = int(port)
        self.gripper_name = gripper_name

        self.python_executable = Path(
            python_executable
        )
        self.graspgenx_root = Path(
            graspgenx_root
        )

        self.checkpoint_dir = Path(
            checkpoint_dir
        )
        self.gripper_cfg_dir = Path(
            gripper_cfg_dir
        )

        self.worker_path = (
            THIS_DIR / "worker.py"
        )

        self._worker_process = None
        self._worker_lock = threading.Lock()

    def _build_env(self) -> dict[str, str]:
        env = os.environ.copy()

        # Do not expose the SeeDo Python environment to
        # the GraspGen-X worker.
        env.pop(
            "PYTHONPATH",
            None,
        )

        env["PYTHONNOUSERSITE"] = "1"

        env[
            "GRASPGENX_CHECKPOINT_DIR"
        ] = str(
            self.checkpoint_dir
        )

        env[
            "GRASPGENX_GRIPPER_CFG_DIR"
        ] = str(
            self.gripper_cfg_dir
        )

        return env

    def _stop_worker(
        self,
    ) -> None:

        process = self._worker_process

        self._worker_process = None

        if process is None:
            return

        try:
            if process.stdin is not None:
                process.stdin.close()

        except Exception:
            pass

        try:
            process.wait(
                timeout=2.0
            )

        except subprocess.TimeoutExpired:

            try:
                process.kill()

            except Exception:
                pass

            try:
                process.wait(
                    timeout=2.0
                )

            except Exception:
                pass


    def _ensure_worker(
        self,
    ) -> subprocess.Popen:

        process = self._worker_process

        if (
            process is not None
            and process.poll() is None
        ):
            return process


        self._stop_worker()


        command = [
            str(self.python_executable),
            str(self.worker_path),

            "--persistent",

            "--host",
            self.host,

            "--port",
            str(self.port),
        ]


        try:
            process = subprocess.Popen(
                command,

                cwd=str(
                    self.graspgenx_root
                ),

                env=self._build_env(),

                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,

                text=True,
                bufsize=1,
            )

        except OSError as exc:
            raise GraspGenXRemoteUnavailable(
                "Could not start the persistent "
                "GraspGen-X client worker."
            ) from exc


        self._worker_process = process

        return process


    def _request_worker(
        self,
        request: dict,
        timeout: float,
    ) -> dict:

        with self._worker_lock:

            process = self._ensure_worker()

            if (
                process.stdin is None
                or process.stdout is None
            ):
                self._stop_worker()

                raise GraspGenXRemoteUnavailable(
                    "Persistent GraspGen-X worker "
                    "pipes are unavailable."
                )


            try:
                process.stdin.write(
                    json.dumps(
                        request
                    )
                    + "\n"
                )

                process.stdin.flush()

            except Exception as exc:
                self._stop_worker()

                raise GraspGenXRemoteUnavailable(
                    "Could not send request to "
                    "persistent GraspGen-X worker."
                ) from exc


            deadline = (
                time.monotonic()
                + float(timeout)
            )

            worker_output = []


            while True:

                remaining = (
                    deadline
                    - time.monotonic()
                )

                if remaining <= 0.0:

                    self._stop_worker()

                    raise GraspGenXRemoteUnavailable(
                        "Persistent GraspGen-X worker "
                        "timed out."
                    )


                ready, _, _ = select.select(
                    [process.stdout],
                    [],
                    [],
                    remaining,
                )


                if not ready:

                    self._stop_worker()

                    raise GraspGenXRemoteUnavailable(
                        "Persistent GraspGen-X worker "
                        "timed out."
                    )


                line = process.stdout.readline()


                if line == "":

                    return_code = (
                        process.poll()
                    )

                    self._stop_worker()

                    raise GraspGenXRemoteUnavailable(
                        "Persistent GraspGen-X worker "
                        "terminated unexpectedly "
                        f"with code {return_code}.\n"
                        + "\n".join(
                            worker_output[-50:]
                        )
                    )


                line = line.rstrip(
                    "\r\n"
                )


                if line.startswith(
                    self.WORKER_RESPONSE_PREFIX
                ):

                    payload_text = line[
                        len(
                            self.WORKER_RESPONSE_PREFIX
                        ):
                    ]

                    try:
                        payload = json.loads(
                            payload_text
                        )

                    except json.JSONDecodeError as exc:
                        raise GraspGenXRemoteUnavailable(
                            "Persistent GraspGen-X worker "
                            "returned invalid JSON."
                        ) from exc


                    if not payload.get(
                        "ok",
                        False,
                    ):
                        raise GraspGenXRemoteUnavailable(
                            "GraspGen-X client worker failed: "
                            + str(
                                payload.get(
                                    "error",
                                    "unknown error",
                                )
                            )
                        )


                    return payload


                if line:
                    worker_output.append(
                        line
                    )


    def __del__(
        self,
    ):

        try:
            self._stop_worker()

        except Exception:
            pass

    def health(
        self,
        timeout: float = 20.0,
    ) -> bool:

        result = self._request_worker(
            {
                "action": "health",
            },
            timeout=timeout,
        )

        health_result = result.get(
            "result"
        )

        return (
            isinstance(
                health_result,
                dict,
            )
            and health_result.get(
                "status"
            ) == "ok"
        )

    def predict_grasps(
        self,
        point_cloud: np.ndarray,
        num_grasps: int = 200,
        grasp_threshold: float = -1.0,
        topk_num_grasps: int = 100,
        timeout: float = 120.0,
    ) -> tuple[
        np.ndarray,
        np.ndarray,
    ]:

        point_cloud = np.asarray(
            point_cloud,
            dtype=np.float32,
        )

        if (
            point_cloud.ndim != 2
            or point_cloud.shape[1] != 3
        ):
            raise ValueError(
                "point_cloud must have shape "
                f"(N, 3), got "
                f"{point_cloud.shape}"
            )

        if len(point_cloud) == 0:
            raise ValueError(
                "point_cloud must not be empty."
            )

        with tempfile.TemporaryDirectory(
            prefix="graspgenx_"
        ) as tmp_dir:

            tmp_dir = Path(
                tmp_dir
            )

            input_path = (
                tmp_dir / "request.npz"
            )

            output_path = (
                tmp_dir / "response.npz"
            )

            np.savez(
                input_path,
                point_cloud=point_cloud,
            )

            self._request_worker(
                {
                    "action": "infer",

                    "input":
                        str(input_path),

                    "output":
                        str(output_path),

                    "gripper_name":
                        self.gripper_name,

                    "num_grasps":
                        int(num_grasps),

                    "grasp_threshold":
                        float(grasp_threshold),

                    "topk_num_grasps":
                        int(topk_num_grasps),
                },
                timeout=timeout,
            )

            if not output_path.exists():
                raise GraspGenXRemoteUnavailable(
                    "GraspGen-X worker completed "
                    "without producing a response."
                )

            with np.load(
                output_path,
                allow_pickle=False,
            ) as data:

                grasps = np.asarray(
                    data["grasps"],
                    dtype=np.float32,
                )

                confidences = np.asarray(
                    data["confidences"],
                    dtype=np.float32,
                )

        if grasps.size == 0:
            grasps = np.empty(
                (0, 4, 4),
                dtype=np.float32,
            )

        if confidences.size == 0:
            confidences = np.empty(
                (0,),
                dtype=np.float32,
            )

        return (
            grasps,
            confidences,
        )



    def predict_object_grasps(
        self,
        point_cloud: np.ndarray,
        sweep_volume_params: dict,
        planner: str = "graspmoe",
        num_grasps: int = 200,
        grasp_threshold: float = -1.0,
        topk_num_grasps: int = -1,
        obb_density: str = "sparse",
        obb_position_spacing_m: float = 0.01,
        timeout: float = 180.0,
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
            or point_cloud.shape[1] != 3
        ):
            raise ValueError(
                "point_cloud must have shape "
                f"(N, 3), got "
                f"{point_cloud.shape}"
            )

        if len(point_cloud) == 0:
            raise ValueError(
                "point_cloud must not be empty."
            )

        if planner not in (
            "graspmoe",
            "diffusion",
        ):
            raise ValueError(
                "planner must be either "
                "'graspmoe' or 'diffusion'."
            )

        required_keys = (
            "extents_open",
            "offset_open",
            "extents_mid",
            "offset_mid",
        )

        missing = [
            key
            for key in required_keys
            if key not in sweep_volume_params
        ]

        if missing:
            raise ValueError(
                "Missing sweep-volume fields: "
                f"{missing}"
            )

        extents_open = np.asarray(
            sweep_volume_params[
                "extents_open"
            ],
            dtype=np.float32,
        )

        offset_open = np.asarray(
            sweep_volume_params[
                "offset_open"
            ],
            dtype=np.float32,
        )

        extents_mid = np.asarray(
            sweep_volume_params[
                "extents_mid"
            ],
            dtype=np.float32,
        )

        offset_mid = np.asarray(
            sweep_volume_params[
                "offset_mid"
            ],
            dtype=np.float32,
        )

        for name, value in (
            (
                "extents_open",
                extents_open,
            ),
            (
                "offset_open",
                offset_open,
            ),
            (
                "extents_mid",
                extents_mid,
            ),
            (
                "offset_mid",
                offset_mid,
            ),
        ):
            if value.shape != (3,):
                raise ValueError(
                    f"{name} must have shape (3,), "
                    f"got {value.shape}."
                )

        gripper_type = int(
            sweep_volume_params.get(
                "gripper_type",
                0,
            )
        )

        fingertip_depth = (
            sweep_volume_params.get(
                "fingertip_depth",
                None,
            )
        )

        with tempfile.TemporaryDirectory(
            prefix="graspgenx_"
        ) as tmp_dir:

            tmp_dir = Path(
                tmp_dir
            )

            input_path = (
                tmp_dir / "request.npz"
            )

            output_path = (
                tmp_dir / "response.npz"
            )

            request_data = {
                "point_cloud": point_cloud,
                "extents_open": extents_open,
                "offset_open": offset_open,
                "extents_mid": extents_mid,
                "offset_mid": offset_mid,
                "gripper_type": np.asarray(
                    gripper_type,
                    dtype=np.int32,
                ),
            }

            if fingertip_depth is not None:
                request_data[
                    "fingertip_depth"
                ] = np.asarray(
                    float(
                        fingertip_depth
                    ),
                    dtype=np.float32,
                )

            np.savez(
                input_path,
                **request_data,
            )

            self._request_worker(
                {
                    "action": "infer_object",

                    "input":
                        str(input_path),

                    "output":
                        str(output_path),

                    "planner":
                        planner,

                    "num_grasps":
                        int(num_grasps),

                    "grasp_threshold":
                        float(grasp_threshold),

                    "topk_num_grasps":
                        int(topk_num_grasps),

                    "obb_density":
                        str(obb_density),

                    "obb_position_spacing_m":
                        float(
                            obb_position_spacing_m
                        ),
                },
                timeout=timeout,
            )

            if not output_path.exists():
                raise GraspGenXRemoteUnavailable(
                    "GraspGen-X worker completed "
                    "without producing a response."
                )

            with np.load(
                output_path,
                allow_pickle=False,
            ) as data:

                grasps = np.asarray(
                    data["grasps"],
                    dtype=np.float32,
                )

                confidences = np.asarray(
                    data["confidences"],
                    dtype=np.float32,
                )

                branch_tags = np.asarray(
                    data["branch_tags"],
                    dtype=str,
                )

        if grasps.size == 0:
            grasps = np.empty(
                (0, 4, 4),
                dtype=np.float32,
            )

        if confidences.size == 0:
            confidences = np.empty(
                (0,),
                dtype=np.float32,
            )

        if branch_tags.size == 0:
            branch_tags = np.empty(
                (0,),
                dtype="<U8",
            )

        if len(grasps) != len(confidences):
            raise GraspGenXRemoteUnavailable(
                "GraspGen-X returned inconsistent "
                "grasp/confidence lengths."
            )

        if len(branch_tags) != len(grasps):
            raise GraspGenXRemoteUnavailable(
                "GraspGen-X returned inconsistent "
                "branch-tag lengths."
            )

        return (
            grasps,
            confidences,
            branch_tags,
        )