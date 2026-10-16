
"""GraspGen-X V2 backend, sharing V1 inference but using V2 geometry.

plan(): compatible with SeeDo, blocked by default after inference.
plan_offline(): for an explicit test harness, returns the computed pose without
                changing the protection used by SeeDo.

No robot movement happens inside this planner.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import tempfile

import numpy as np
from scipy.spatial.transform import Rotation

from ai_controller.utils.grasping.graspgenx.planner import GraspGenXPlanner


class GraspGenXV2Planner(GraspGenXPlanner):
    # SeeDo MotionLayer handles V1 and V2 using the same TCP grasp interface.
    backend_name = "graspgenx"

    def __init__(
        self, *, v2_driver_urdf: str, v2_robotiq_mesh_share: str,
        v2_worker_python: str = "/opt/graspgenx_venv/bin/python",
        v2_gripper_open_q: float = 0.1,
        v2_table_margin_mm: float = 5.0,
        v2_max_expensive_checks: int = 80,
        v2_worker_timeout_s: int = 300,
        v2_unsafe_enable_robot_commands: bool = False,
        **v1_kwargs,
    ):
        super().__init__(**v1_kwargs)
        self.v2_driver_urdf = Path(v2_driver_urdf)
        self.v2_robotiq_mesh_share = Path(v2_robotiq_mesh_share)
        self.v2_worker_python = Path(v2_worker_python)
        self.v2_gripper_open_q = float(v2_gripper_open_q)
        self.v2_table_margin_mm = float(v2_table_margin_mm)
        self.v2_max_expensive_checks = int(v2_max_expensive_checks)
        self.v2_worker_timeout_s = int(v2_worker_timeout_s)
        self.v2_unsafe_enable_robot_commands = bool(
            v2_unsafe_enable_robot_commands
        )
        self._v2_selected_rotation = None
        self._v2_tcp_orientation = None
        self._v2_last_details = None

    def _compute_plan(self, *args, **kwargs):
        # Capture the PRE-grasp TCP orientation; index selection needs this.
        if "current_tcp_orientation" in kwargs:
            current_quat = kwargs["current_tcp_orientation"]
        elif len(args) >= 7:
            current_quat = args[6]
        else:
            raise ValueError("V2 requires current_tcp_orientation")

        self._v2_tcp_orientation = np.asarray(
            current_quat, dtype=np.float64,
        ).reshape(4)
        self._v2_selected_rotation = None
        self._v2_last_details = None

        # Re-runs REAL GraspMolmo, SAM, GraspGen-X.
        # Uses V2 override for geometric selection.
        result = super().plan(*args, **kwargs)
        details = self._v2_last_details

        if details is None or self._v2_selected_rotation is None:
            raise RuntimeError("V2 worker did not return a selection")

        # V1 saves the raw candidate before possible pi_z symmetry.
        # Preserve it, but save the ACTUAL checked V2 pose separately.
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
                "V2 checked gripper pose and returned TCP pose disagree; "
                "offline plan rejected"
            )

        artifact_root = kwargs.get("artifacts_dir")
        if artifact_root is None and len(args) >= 9:
            artifact_root = args[8]

        if artifact_root is not None:
            root = Path(artifact_root).expanduser().resolve()
            root.mkdir(parents=True, exist_ok=True)

            report = {
                "status": "OFFLINE_GEOMETRIC_PLAN_NOT_ROBOT_SAFE",
                "backend": "graspgenx_v2",
                "note": (
                    "Use selected_gripper_pose_base below: the V1 raw "
                    "candidate artifact may show a different pi_z symmetry. "
                    "No robot movement is authorized by this report."
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
                    self._v2_tcp_orientation.tolist()
                ),
            }

            (root / "v2_geometry_selection.json").write_text(
                json.dumps(report, indent=2),
                encoding="utf-8",
            )

        return result

    def plan(self, *args, **kwargs):
        """Normal SeeDo entry point: blocked unless expressly overridden."""
        result = self._compute_plan(*args, **kwargs)

        if not self.v2_unsafe_enable_robot_commands:
            raise RuntimeError(
                "[GRASPGENX_V2] Offline artifacts saved, but plan is blocked "
                "for SeeDo execution. Use plan_offline() in a standalone "
                "test process to inspect the predicted pose."
            )

        return result

    def plan_offline(self, *args, **kwargs):
        """Explicit inference-only entry point for standalone saved-frame tests.

        Never call this from the SeeDo controller: it returns a pose object.
        The normal plan() block remains in force.
        """
        if self.v2_unsafe_enable_robot_commands:
            raise RuntimeError(
                "Disable robot-command override for offline testing"
            )

        return self._compute_plan(*args, **kwargs)

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
            current_tcp_orientation=self._v2_tcp_orientation,
        )

        for path in (
            self.v2_driver_urdf,
            self.v2_worker_python,
        ):
            if not path.is_file():
                raise FileNotFoundError(
                    f"V2 dependency missing: {path}"
                )

        if not self.v2_robotiq_mesh_share.is_dir():
            raise FileNotFoundError(
                "V2 Robotiq package missing: "
                f"{self.v2_robotiq_mesh_share}"
            )

        with tempfile.TemporaryDirectory(
            prefix="graspgenx_v2_"
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
                str(self.v2_worker_python),
                "-m",
                "ai_controller.utils.grasping.graspgenx_v2.runtime_worker",
                "--input", str(inp),
                "--output", str(out),
                "--urdf", str(self.v2_driver_urdf),
                "--mesh-share", str(self.v2_robotiq_mesh_share),
                "--open-q", str(self.v2_gripper_open_q),
                "--table-margin-mm", str(self.v2_table_margin_mm),
                "--max-expensive-checks", str(self.v2_max_expensive_checks),
            ]

            process = subprocess.run(
                cmd,
                env=env,
                capture_output=True,
                text=True,
                timeout=self.v2_worker_timeout_s,
                check=False,
            )

            if process.stdout:
                print(process.stdout, flush=True)

            if process.returncode != 0:
                raise RuntimeError(
                    "[GRASPGENX_V2] Worker failed or rejected all grasps:\n"
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
            raise RuntimeError("Invalid V2 candidate index")

        if (
            checked_pose.shape != (4, 4)
            or not np.isfinite(checked_pose).all()
        ):
            raise RuntimeError(
                "Invalid V2 selected gripper transform"
            )

        self._v2_selected_rotation = (
            checked_pose[:3, :3].copy()
        )
        self._v2_last_details = details

        print(
            "[GRASPGENX_V2] geometric selection:",
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
        # V1 asks for a symmetry after selection.
        # V2 has already checked and selected it.
        if self._v2_selected_rotation is None:
            raise RuntimeError(
                "V2 rotation not supplied by worker"
            )

        if not np.allclose(
            self._v2_selected_rotation[:, 2],
            np.asarray(grasp_rotation_base)[:, 2],
            atol=1e-5,
        ):
            raise RuntimeError(
                "V2 rotation changes the grasp approach axis"
            )

        return self._v2_selected_rotation.copy()
