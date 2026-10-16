# GraspGen-X V2 — runtime module, not a one-off audit

## Contents
- `runtime_core.py`: reusable scene/table/semantic/gripper candidate selection.
- `runtime_worker.py`: geometry worker launched in `/opt/graspgenx_venv`.
- `backend.py`: SeeDo backend, inherits original V1 perception/inference and returns an index plus validated orientation; retains the `graspgenx` Motion Layer interface.
- `install_factory_patch.py`: idempotent patch of ONLY `grasp_planner_factory.py`.
- `offline_replay.py`: optional single end-to-end frozen-scene regression test.

## Installation in existing branch

At the root of your repository:

```bash
cp runtime_core.py runtime_worker.py backend.py offline_replay.py \
  ai_controller/ai_controller/utils/grasping/graspgenx_v2/
python3 install_factory_patch.py
```

In `ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml`, under `grasp_planner:`:

```yaml
  backend: graspgenx_v2
  graspgenx_v2_driver_urdf: /test_real/urdf_audit/ur5e_driver_installed_audit.urdf
  graspgenx_v2_robotiq_mesh_share: /home/ros2_ws/install/robotiq_description/share/robotiq_description
  graspgenx_v2_worker_python: /opt/graspgenx_venv/bin/python
  graspgenx_v2_gripper_open_q: 0.1
  graspgenx_v2_table_margin_mm: 5.0
  graspgenx_v2_max_expensive_checks: 80
  graspgenx_v2_unsafe_enable_robot_commands: false
```

The V1 `graspgenx` backend is unchanged. Do not change URDF files.

**IMPORTANT**: `unsafe_enable_robot_commands` MUST remain false. The new
selector is not yet a complete robot-safe checker (mount, arm, full executed
path, closed-gripper contact, depth occlusions). SeeDo will compute/log/possibly
save a geometry-prefiltered plan and then stop before commanding the grasp.

## One smoke test (not a new object-specific algorithm)

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=ai_controller \
  /opt/graspgenx_venv/bin/python \
    -m ai_controller.utils.grasping.graspgenx_v2.offline_replay \
    --case-dir /test_real/test_task00_23 \
    --mesh-share /home/ros2_ws/install/robotiq_description/share/robotiq_description
```

The results may be a `NoValidGrasp` if observations do not provide reliable
geometry. No arbitrary retreat is applied. This tests the complete geometry
worker, not only a preselected grasp.

## Deliberate limitations
- The geometric prefilter cannot certify robot safety.
- It checks **all candidates for table/semantic** constraints, then applies
  signed-distance and a simplified straight 80mm pregrasp approach to the top
  N candidates (`max_expensive_checks`).
- It **does not add** a grasp retreat or a second depth offset. Candidates
  already include the base planner's configured depth adjustment.
- Table is robustly fit from current live depth, not `+40mm` hardcoded.
- Original `GraspGenXPlanner.plan` can keep writing artifacts for regression.
- By design, the system rejects cases with uncertain table/scene geometry.
