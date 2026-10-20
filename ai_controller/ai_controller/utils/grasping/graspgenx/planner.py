"""GraspGen-X planner with semantic and geometric grasp selection."""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import tempfile

import numpy as np
from scipy.spatial.transform import Rotation

from ai_controller.utils.grasping.graspgenx.inference_core import (
    GraspGenXInferenceCore,
)


class GraspGenXPlanner(GraspGenXInferenceCore):
    # SeeDo MotionLayer consumes the standard TCP grasp interface.
    backend_name = "graspgenx"

    def __init__(
        self,
        *,
        worker_python: str = "/opt/graspgenx_venv/bin/python",
        table_margin_mm: float = 0.0,
        semantic_radius_px: float = 50.0,
        worker_timeout_s: int = 300,
        **v1_kwargs,
    ):
        super().__init__(
            semantic_radius_px=semantic_radius_px,
            **v1_kwargs,
        )

        self.worker_python = Path(
            worker_python
        )

        self.table_margin_mm = float(
            table_margin_mm
        )

        self.geometry_semantic_radius_px = float(
            semantic_radius_px
        )

        if not (
            0 < self.geometry_semantic_radius_px <= 1000
        ):
            raise ValueError(
                "Invalid semantic radius"
            )

        self.worker_timeout_s = int(
            worker_timeout_s
        )

        self._selected_rotation = None
        self._tcp_orientation = None
        self._last_details = None

    def _compute_plan(self, *args, **kwargs):
        # Capture the PRE-grasp TCP orientation; index selection needs this.
        if "current_tcp_orientation" in kwargs:
            current_quat = kwargs["current_tcp_orientation"]
        elif len(args) >= 7:
            current_quat = args[6]
        else:
            raise ValueError("GraspGen-X requires current_tcp_orientation")

        self._tcp_orientation = np.asarray(
            current_quat, dtype=np.float64,
        ).reshape(4)
        self._selected_rotation = None
        self._last_details = None

        # Re-runs REAL GraspMolmo, SAM, GraspGen-X.
        # Use the geometry selector for candidate selection.
        result = super().plan(*args, **kwargs)
        details = self._last_details

        if details is None or self._selected_rotation is None:
            raise RuntimeError("GraspGen-X geometry worker did not return a selection")

        # The inference core saves the raw candidate before possible pi_z symmetry.
        # Preserve it, but save the actual geometry-checked pose separately.
        checked_gripper = np.asarray(
            details["T_base_gripper"], dtype=np.float64
        )

        expected_tcp_pos = (
            checked_gripper[:3, 3]
            + self.FINGERTIP_DEPTH * checked_gripper[:3, 2]
        )

        actual_tcp_pos = np.asarray(
            result.tcp_position_base, dtype=np.float64
        )

        actual_tcp_R = Rotation.from_quat(
            np.asarray(
                result.tcp_orientation_base,
                dtype=np.float64,
            )
        ).as_matrix()

        if (
            not np.allclose(
                expected_tcp_pos,
                actual_tcp_pos,
                atol=1e-5,
            )
            or not np.allclose(
                checked_gripper[:3, :3],
                actual_tcp_R,
                atol=1e-5,
            )
        ):
            raise RuntimeError(
                "Selected gripper pose and returned TCP pose disagree"
                "offline plan rejected"
            )

        artifact_root = kwargs.get("artifacts_dir")
        if artifact_root is None and len(args) >= 9:
            artifact_root = args[8]

        if artifact_root is not None:
            root = Path(artifact_root).expanduser().resolve()
            root.mkdir(parents=True, exist_ok=True)

            report = {
                "status": "OK",
                "backend": "graspgenx",
                "note": (
                    "Grasp pose selected by the GraspGen-X "
                    "geometry selector."
                ),
                "selection": details,
                "selected_gripper_pose_base": checked_gripper.tolist(),
                "selected_tcp_position_base": actual_tcp_pos.tolist(),
                "selected_tcp_quaternion_xyzw": (
                    np.asarray(
                        result.tcp_orientation_base,
                        dtype=float,
                    ).tolist()
                ),
                "current_tcp_quaternion_xyzw": (
                    self._tcp_orientation.tolist()
                ),
            }

            (root / "graspgenx_geometry_selection.json").write_text(
                json.dumps(report, indent=2),
                encoding="utf-8",
            )

        return result

    def plan(self, *args, **kwargs):
        """Run the GraspGen-X planner."""
        return self._compute_plan(
            *args,
            **kwargs,
        )

    def _select_safe_grasp(
        self, *, grasps_base, confidences, branch_tags, T_base_table,
        semantic_point_px, T_base_camera, camera_matrix,
        target_point_cloud_base, background_scene_point_cloud_base,
    ):
        inputs = dict(
            grasps_base=np.asarray(
                grasps_base, dtype=np.float64
            ),
            confidences=np.asarray(
                confidences, dtype=np.float64
            ),
            branch_tags=np.asarray(
                branch_tags
            ).astype("U32"),
            T_base_table=np.asarray(
                T_base_table, dtype=np.float64
            ),
            semantic_point_px=np.asarray(
                semantic_point_px, dtype=np.float64
            ),
            T_base_camera=np.asarray(
                T_base_camera, dtype=np.float64
            ),
            camera_matrix=np.asarray(
                camera_matrix, dtype=np.float64
            ),
            target_points_base=np.asarray(
                target_point_cloud_base, dtype=np.float64
            ),
            background_points_base=np.asarray(
                background_scene_point_cloud_base,
                dtype=np.float64,
            ),
            current_tcp_orientation=self._tcp_orientation,
        )

        if not self.worker_python.is_file():
            raise FileNotFoundError(
                "GraspGen-X worker Python is missing: "
                f"{self.worker_python}"
            )

        with tempfile.TemporaryDirectory(
            prefix="graspgenx_"
        ) as tmp:
            inp = Path(tmp) / "input.npz"
            out = Path(tmp) / "choice.json"

            np.savez_compressed(inp, **inputs)

            package_root = Path(__file__).resolve().parents[4]

            env = os.environ.copy()
            env["PYTHONPATH"] = os.pathsep.join(
                [
                    str(package_root),
                    env.get("PYTHONPATH", ""),
                ]
            )
            env["PYTHONDONTWRITEBYTECODE"] = "1"

            cmd = [
                str(self.worker_python),
                "-m",
                "ai_controller.utils.grasping.graspgenx.runtime_worker",
                "--input",
                str(inp),
                "--output",
                str(out),
                "--table-margin-mm",
                str(self.table_margin_mm),
                "--semantic-radius-px",
                str(self.geometry_semantic_radius_px),
            ]

            process = subprocess.run(
                cmd,
                env=env,
                capture_output=True,
                text=True,
                timeout=self.worker_timeout_s,
                check=False,
            )

            if process.stdout:
                print(process.stdout, flush=True)

            if process.returncode != 0:
                raise RuntimeError(
                    "[GRASPGENX] Worker failed or rejected all grasps:\n"
                    + process.stderr[-7000:]
                )

            details = json.loads(
                out.read_text(encoding="utf-8")
            )

        idx = int(details["index"])
        checked_pose = np.asarray(
            details["T_base_gripper"],
            dtype=np.float64,
        )

        if idx < 0 or idx >= len(inputs["grasps_base"]):
            raise RuntimeError("Invalid GraspGen-X candidate index")

        if (
            checked_pose.shape != (4, 4)
            or not np.isfinite(checked_pose).all()
        ):
            raise RuntimeError(
                "Invalid GraspGen-X selected gripper transform"
            )

        self._selected_rotation = (
            checked_pose[:3, :3].copy()
        )
        self._last_details = details

        print(
            "[GRASPGENX] geometric selection:",
            json.dumps({
                k: v
                for k, v in details.items()
                if k != "T_base_gripper"
            }),
            flush=True,
        )

        return idx

    def _resolve_parallel_gripper_symmetry(
        self, *, grasp_rotation_base, current_tcp_orientation,
    ):
        """Resolve the symmetry of the gripper in parallel with the current TCP orientation."""
        # The geometry selector has already chosen the orientation.
        if self._selected_rotation is None:
            raise RuntimeError(
                "GraspGen-X rotation not supplied by worker"
            )

        if not np.allclose(
            self._selected_rotation[:, 2],
            np.asarray(grasp_rotation_base)[:, 2],
            atol=1e-5,
        ):
            raise RuntimeError(
                "GraspGen-X rotation changes the grasp approach axis"
            )

        return self._selected_rotation.copy()
