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

    def filter_collision_free_grasps(
        self,
        grasps_base: np.ndarray,
        background_scene_point_cloud_base: np.ndarray,
        target_point_cloud_base: np.ndarray,
        sweep_volume_params: dict,
        collision_threshold: float = 0.002,
        target_collision_threshold: float = 0.002,
        num_collision_samples: int = 2000,
        max_scene_points: int = 8192,
        timeout: float = 180.0,
    ) -> tuple[
        np.ndarray,
        np.ndarray,
        np.ndarray,
    ]:
        """
        Check candidate grasps with the real Robotiq collision mesh.

        Background/other-object geometry is checked with GraspGen-X's
        official mesh-based collision filter.

        For the target itself, points inside the intended open sweep volume
        are allowed contact points. Target points outside that volume remain
        obstacles, so another part of the same object cannot be crossed by
        a finger/palm.
        """
        grasps_base = np.asarray(
            grasps_base,
            dtype=np.float32,
        )

        background_scene_point_cloud_base = np.asarray(
            background_scene_point_cloud_base,
            dtype=np.float32,
        )

        target_point_cloud_base = np.asarray(
            target_point_cloud_base,
            dtype=np.float32,
        )

        if (
            grasps_base.ndim != 3
            or grasps_base.shape[1:] != (4, 4)
        ):
            raise ValueError(
                "grasps_base must have shape (K, 4, 4), "
                f"got {grasps_base.shape}."
            )

        for name, cloud in (
            (
                "background_scene_point_cloud_base",
                background_scene_point_cloud_base,
            ),
            (
                "target_point_cloud_base",
                target_point_cloud_base,
            ),
        ):
            if (
                cloud.ndim != 2
                or cloud.shape[1] != 3
            ):
                raise ValueError(
                    f"{name} must have shape (N, 3), "
                    f"got {cloud.shape}."
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

        with tempfile.TemporaryDirectory(
            prefix="graspgenx_collision_"
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
                grasps=grasps_base,
                background_scene_point_cloud=(
                    background_scene_point_cloud_base
                ),
                target_point_cloud=(
                    target_point_cloud_base
                ),
                extents_open=extents_open,
                offset_open=offset_open,
            )

            self._run_worker(
                [
                    "--action",
                    "collision_filter",
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
                    "--collision-threshold",
                    str(collision_threshold),
                    "--target-collision-threshold",
                    str(target_collision_threshold),
                    "--num-collision-samples",
                    str(num_collision_samples),
                    "--max-scene-points",
                    str(max_scene_points),
                ],
                timeout=timeout,
            )

            if not output_path.exists():
                raise GraspGenXRemoteUnavailable(
                    "GraspGen-X collision worker completed "
                    "without producing a response."
                )

            with np.load(
                output_path,
                allow_pickle=False,
            ) as data:

                collision_free = np.asarray(
                    data["collision_free"],
                    dtype=bool,
                )

                background_free = np.asarray(
                    data["background_free"],
                    dtype=bool,
                )

                target_free = np.asarray(
                    data["target_free"],
                    dtype=bool,
                )

        expected_shape = (
            len(grasps_base),
        )

        for name, mask in (
            (
                "collision_free",
                collision_free,
            ),
            (
                "background_free",
                background_free,
            ),
            (
                "target_free",
                target_free,
            ),
        ):
            if mask.shape != expected_shape:
                raise GraspGenXRemoteUnavailable(
                    f"{name} has shape {mask.shape}; "
                    f"expected {expected_shape}."
                )

        return (
            collision_free,
            background_free,
            target_free,
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