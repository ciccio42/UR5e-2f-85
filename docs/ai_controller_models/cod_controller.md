# COD-Controller

`cod_controller` (`ai_controller/ai_controller/models/cod_controller/`) runs the
COD (Conditioned-target-Object-Detector) MOSAIC double-policy from
[Multi-Task-LFD-Training-Framework](https://github.com/ciccio42/Multi-Task-LFD-Training-Framework)
(`mt_rep_double_policy.py`). At each control-loop step it feeds the frontal ZED
image plus a small set of human-demonstration context frames (sampled once
per task from a demo `.pkl` via `load_command`) to the policy, along with the
gripper/wrist camera image on its own embedding stream when the loaded
checkpoint was trained with `use_wrist_img=true`. The model is bundled in the
repo (no separate model repo to clone) but still depends on its own
[target-object-detector checkpoint](../../ai_controller/ai_controller/models/cod_controller/cond_target_obj_detector.py)
referenced from the policy config.

This assumes the Docker/container setup and UR-driver + `moveit_controller`
launch described in [AI-Controller](../ai_controller.md) are already running.

## Installation

Install the following inside the container (`docker exec -it
ur_robotiq_teleoperation_container bash`) - no extra repo to clone, `cod_controller`
only needs the shared AI-controller Python dependencies:

```bash
python3 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128 --break-system-packages
python3 -m pip install hydra-core omegaconf einops torchsummary tqdm --break-system-packages
```

### Checkpoint layout

`model_config_path` below must point at a `config.yaml` that:
- sits in the same folder as its `model_save-<step>.pt` weights (the
  controller automatically picks the highest-step checkpoint in that folder,
  see `CODController.load_model`);
- has `policy.target_obj_detector_path` / `policy.target_obj_detector_step`
  set to the target-object-detector checkpoint to load alongside the policy
  weights.

## Running

```bash
docker exec -it ur_robotiq_teleoperation_container bash
source install/setup.bash

# Run AI-Controller (COD-Default)
ros2 run ai_controller ai_controller_node --ros-args \
    -p move_robot:=True  \
    -p ai_controller_target:="cod_controller" \
    -p model_config_path:="/home/ros2_ws/src/ai_controller/checkpoint_folder/Real-1Task-pick_place-Simulated-Agent-Human-Demonstration-UR5e-Agent-MOSAIC-COD-SKIP-0-5-10-15-Batch24/config.yaml"
```

To replay a previously saved rollout instead of running the live policy, see
`replicate_rollout` in [AI-Controller](../ai_controller.md#launch).
