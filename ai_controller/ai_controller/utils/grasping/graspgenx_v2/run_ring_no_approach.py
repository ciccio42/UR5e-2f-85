"""Offline-only GraspGen-X V2 ring preview, approach_target disabled.

Requires the companion runtime_core.py installed in the ROS2 workspace.
Never runs the physical robot or gripper.
"""
from pathlib import Path
from unittest.mock import patch
import os
import sys

os.environ["GRASPGENX_V2_OFFLINE_DIAGNOSTIC"] = "1"
os.environ["GRASPGENX_V2_DIAG_SKIP_APPROACH_TARGET"] = "1"
os.environ.pop("GRASPGENX_V2_DIAG_APPROACH_TIP_MM", None)

from ai_controller.utils.grasping.graspgenx_v2.backend import GraspGenXV2Planner
from ai_controller.utils.grasping.graspgenx_v2 import test_v2_full_inference as test

ROOT = Path("/test_real/graspgenx_v2_full")
ROOT.mkdir(parents=True, exist_ok=True)
for i in range(1, 1000):
    output = ROOT / f"ring_no_approach_{i:03d}"
    preview = ROOT / f"ring_no_approach_{i:03d}_pose_preview.json"
    if not output.exists() and not preview.exists():
        break
else:
    raise RuntimeError("No unused output name")

os.environ["GRASPGENX_V2_DIAG_PREVIEW_FILE"] = str(preview)

original_init = GraspGenXV2Planner.__init__

def patched_init(self, *args, **kwargs):
    original_init(self, *args, **kwargs)
    if self.v2_unsafe_enable_robot_commands:
        raise RuntimeError(
            "Refusing an offline diagnostic with robot commands enabled. "
            "Set graspgenx_v2_unsafe_enable_robot_commands: false"
        )
    self.grasp_depth_offset_m = 0.0
    print("[TEST] depth offset=0 mm, robot commands disabled", flush=True)

sys.argv = [
    "test_v2_full_inference.py",
    "--config",
    "/home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml",
    "--pkl",
    "/test_real/test_task00_22/rollouts/seedo_controller/pick_place/task_00/traj_000.pkl",
    "--step", "19",
    "--instruction",
    "Pick up the gray ring by grasping its handle, not the circular ring body.",
    "--output-dir", str(output),
]

print("[TEST] approach_target BYPASSED for offline inspection only", flush=True)
print(f"[TEST] artifacts: {output}", flush=True)
print(f"[TEST] pose preview JSON: {preview}", flush=True)

try:
    with patch.object(GraspGenXV2Planner, "__init__", patched_init):
        code = test.main()
except Exception:
    if preview.exists():
        print(
            f"[TEST] Pose preview saved despite the test failure: {preview}",
            flush=True,
        )
    raise
else:
    if preview.exists():
        print(f"[TEST] Pose preview saved: {preview}", flush=True)
    raise SystemExit(code)
