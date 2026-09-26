# OSVI-WM-Controller

`osvi_controller` (`ai_controller/ai_controller/models/osvi_controller/`) runs
the OSVI world-model-based policy ("OSVI-WM"). The model code and configs are
bundled directly under the model folder (`models/`, `configs/`,
`osvi_config.yaml`) - no external repo to clone.

This assumes the Docker/container setup and UR-driver + `moveit_controller`
launch described in [AI-Controller](../ai_controller.md) are already running.

## Installation

Install the following inside the container (`docker exec -it
ur_robotiq_teleoperation_container bash`):

```bash
cd /home/ros2_ws
colcon build --packages-select ai_controller --symlink-install
source install/setup.bash
python3 -m pip install einops hydra-core omegaconf torchsummary tqdm pyyaml matplotlib --break-system-packages
```

`osvi_config.yaml`'s `checkpoint_dir` / `checkpoint_step` must point at an
OSVI-WM checkpoint folder (e.g. under `ai_controller/checkpoint_folder/osvi_wm/`).

## Running

```bash
docker exec -it ur_robotiq_teleoperation_container bash
cd /home/ros2_ws
source install/setup.bash

ros2 run ai_controller ai_controller_node --ros-args \
  -p move_robot:=True \
  -p ai_controller_target:="osvi_controller" \
  -p model_config_path:="/home/ros2_ws/src/ai_controller/ai_controller/models/osvi_controller/osvi_config.yaml"
```
