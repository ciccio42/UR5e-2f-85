from __future__ import annotations

import os
import subprocess
import tempfile
from pathlib import Path

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

    def _run_worker(
        self,
        arguments: list[str],
        timeout: float,
    ) -> subprocess.CompletedProcess[str]:

        command = [
            str(self.python_executable),
            str(self.worker_path),
            *arguments,
        ]

        try:
            result = subprocess.run(
                command,
                cwd=str(
                    self.graspgenx_root
                ),
                env=self._build_env(),
                capture_output=True,
                text=True,
                timeout=timeout,
                check=False,
            )

        except (
            OSError,
            subprocess.TimeoutExpired,
        ) as exc:
            raise GraspGenXRemoteUnavailable(
                "Could not execute the "
                "GraspGen-X client worker."
            ) from exc

        if result.returncode != 0:
            raise GraspGenXRemoteUnavailable(
                "GraspGen-X client worker failed.\n"
                f"stdout:\n{result.stdout}\n"
                f"stderr:\n{result.stderr}"
            )

        return result

    def health(
        self,
        timeout: float = 20.0,
    ) -> bool:

        result = self._run_worker(
            [
                "--action",
                "health",
                "--host",
                self.host,
                "--port",
                str(self.port),
            ],
            timeout=timeout,
        )

        return '"status": "ok"' in result.stdout

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

            np.savez_compressed(
                input_path,
                point_cloud=point_cloud,
            )

            self._run_worker(
                [
                    "--action",
                    "infer",
                    "--host",
                    self.host,
                    "--port",
                    str(self.port),
                    "--input",
                    str(input_path),
                    "--output",
                    str(output_path),
                    "--gripper-name",
                    self.gripper_name,
                    "--num-grasps",
                    str(num_grasps),
                    "--grasp-threshold",
                    str(grasp_threshold),
                    "--topk-num-grasps",
                    str(topk_num_grasps),
                ],
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

            np.savez_compressed(
                input_path,
                **request_data,
            )

            self._run_worker(
                [
                    "--action",
                    "infer_object",
                    "--host",
                    self.host,
                    "--port",
                    str(self.port),
                    "--input",
                    str(input_path),
                    "--output",
                    str(output_path),
                    "--planner",
                    planner,
                    "--num-grasps",
                    str(num_grasps),
                    "--grasp-threshold",
                    str(grasp_threshold),
                    "--topk-num-grasps",
                    str(topk_num_grasps),
                    "--obb-density",
                    str(obb_density),
                    "--obb-position-spacing-m",
                    str(obb_position_spacing_m),
                ],
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