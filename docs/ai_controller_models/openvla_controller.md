# OpenVLA-Controller

`openvla_controller` (`ai_controller/ai_controller/models/openvla_controller/`)
wraps [openvla-oft](https://github.com/ciccio42/openvla-oft) for closed-loop
control: it predicts a chunk of 8-dim actions (`[x, y, z, roll, pitch, yaw,
gripper]`, converted to a quaternion before being sent to the robot) from the
frontal camera image and the task instruction.

This assumes the Docker/container setup and UR-driver + `moveit_controller`
launch described in [AI-Controller](../ai_controller.md) are already running.

## Installation

Install the following inside the container (`docker exec -it
ur_robotiq_teleoperation_container bash`).

### Required
```bash
cd /home/ros2_ws/src/ai_controller/ai_controller/models/openvla_controller
git clone https://github.com/ciccio42/openvla-oft.git
source /home/ros2_ws/src/ai_controller/ai_controller/models/requirements/openvla_oft_installation.sh
cd openvla-oft
pip install -e . --break-system-packages

pip install --upgrade protobuf --break-system-packages

# unit test
cd /home/ros2_ws/src/ai_controller/ai_controller/models/openvla_controller
python3 test.py
```

### Optional - quantization (reduces GPU memory from ~14 GB to ~8 / ~4 GB)
```bash
pip install bitsandbytes --break-system-packages
```

## Running

```bash
docker exec -it ur_robotiq_teleoperation_container bash
source install/setup.bash

ros2 run ai_controller ai_controller_node --ros-args \
  -p move_robot:=True \
  -p ai_controller_target:=openvla_controller \
  -p model_config_path:=/home/ros2_ws/src/ai_controller/ai_controller/models/openvla_controller/openvla_config.yaml \
  -p task_name:=pick_place
```
