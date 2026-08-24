# AI-Controller

## UR-Container
```bash
export ROBOT_IP=172.16.174.59

docker build -t ur_robotiq_teleoperation . -f UR_Robotiq_Teleoperation
xhost +local:docker
docker run -it --rm \
  --gpus all \
  --privileged \
  --cap-add=SYS_NICE \
  --cpuset-cpus="0-19" \
  --network host \
  --ipc=host \
  --pid=host \
  --ulimit memlock=-1:-1 \
  --ulimit rtprio=99 \
  --shm-size=1g \
  --security-opt seccomp=unconfined \
  -e DISPLAY=$DISPLAY \
  -e ROBOT_IP=${ROBOT_IP} \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev/input:/dev/input \
  -v ${UR5e_2f_85_PATH}/ur5e_2f_85:/home/ros2_ws/src/ur5e_2f_85 \
  -v ${UR5e_2f_85_PATH}/dataset_collector:/home/ros2_ws/src/dataset_collector \
  -v ${UR5e_2f_85_PATH}/ai_controller:/home/ros2_ws/src/ai_controller \
  -v ${UR5e_2f_85_PATH}/moveit_controller:/home/ros2_ws/src/moveit_controller \
  -v ${UR5e_2f_85_PATH}/zed_camera/zed_camera_calibration:/home/ros2_ws/src/zed_camera/zed_camera_calibration:ro \
  -v ${UR5e_2f_85_PATH}/traj_tmp:/traj_tmp \
  -v /home/asus-mivia/Desktop/saved_trajectories:/home/saved_trajectories \
  -v /home/asus-mivia/Desktop/dataset:/dataset \
  --name ur_robotiq_teleoperation_container \
  ur_robotiq_teleoperation


# Only the first time
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=${ROBOT_IP} \
  target_filename:="/home/ros2_ws/src/ur5e_2f_85/real_robot_calibration.yaml"

```

## Zed Container
```bash
xhost +local:docker
docker run -it --rm \
  --gpus all \
  --privileged \
  --network host \
  --ipc=host \
  --pid=host \
  --device=/dev/bus/usb \
  -v /sys:/sys:ro \
  -v /run/udev:/run/udev:ro \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev:/dev \
  -v ${UR5e_2f_85_PATH}/zed_camera:/home/ros2_ws/src/zed_camera \
  --name zed_camera_container \
  5.3-ros2-devel-l4t-r38.4
```

## Launch
**Docker-1: Launch UR-Driver**
```bash
# Launch external-controller [REAL - With Gripper]
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=${ROBOT_IP} \
  use_tool_communication:=true \
  tool_voltage:=24 \
  tool_parity:=0 \
  tool_baud_rate:=115200 \
  tool_stop_bits:=1 \
  tool_rx_idle_chars:=1.5 \
  tool_tx_idle_chars:=3.5 \
  tool_device_name:=/tmp/ttyUR \
  kinematics_params_file:=/home/ros2_ws/src/ur5e_2f_85/real_robot_calibration.yaml \
  controllers_file:=/home/ros2_ws/src/ur5e_2f_85/ur5e_2f_85_description/config/ur5e_2f_85_controllers.yaml \
  description_launchfile:="/home/ros2_ws/src/ur5e_2f_85/ur5e_2f_85_description/launch/ur5e_2f_85_display_control.launch.py" \
  launch_rviz:=false

# Launch without the gripper
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=${ROBOT_IP} \
  use_tool_communication:=false 


docker exec -it ur_robotiq_teleoperation_container  bash
source install/setup.bash
ros2 launch ur5e_2f_85_moveit_config move_group_servo.launch.py launch_servo:=true

# Run moveit_controller
docker exec -it ur_robotiq_teleoperation_container  bash
source install/setup.bash
ros2 run moveit_controller moveit_controller_node --ros-args -p execute_trajectory:=True

# Run moveit_controller in PLAN-ONLY mode: every GoHome/GoToPose request is planned and
# published to /display_planned_path for RViz, but never executed on the robot
# (no ExecuteTrajectory call, no controller_manager switch). See "Simulate before you
# execute" in script_controller.md.
ros2 run moveit_controller moveit_controller_node --ros-args -p execute_trajectory:=False


docker exec -it ur_robotiq_teleoperation_container  bash
source install/setup.bash

# Run AI-Controller (COD-Default)
ros2 run ai_controller ai_controller_node --ros-args \
    -p move_robot:=True  \
    -p ai_controller_target:="cod_controller" \
    -p model_config_path:="/home/ros2_ws/src/ai_controller/checkpoint_folder/Real-1Task-pick_place-Simulated-Agent-Human-Demonstration-UR5e-Agent-MOSAIC-COD-SKIP-0-5-10-15-EYE-IN-HAND--Batch24/config.yaml"

# Run AI-Controller (Open-VLA)
pip install --upgrade protobuf --break-system-packages
ros2 run ai_controller ai_controller_node --ros-args \
    -p move_robot:=True  \
    -p ai_controller_target:="openvla_controller" \
    -p model_config_path:="/home/ros2_ws/src/ai_controller/ai_controller/models/openvla_controller/openvla_config.yaml"

# Run AI-Controller (TinyVLA)
export PYTHONPATH=$PYTHONPATH:/home/ros2_ws/src/ai_controller/ai_controller/models/tinyvla_controller/TinyVLA
export PYTHONPATH=$PYTHONPATH:/home/ros2_ws/src/ai_controller/ai_controller/models/tinyvla_controller/TinyVLA/llava-pythia
ros2 run ai_controller ai_controller_node --ros-args \
    -p move_robot:=True  \
    -p ai_controller_target:="tinyvla_controller" \
    -p model_config_path:="/home/ros2_ws/src/ai_controller/ai_controller/models/tinyvla_controller/tinyvla_config.yaml"

# Replicate saved trajectories
# add -p dry_run:=false to actually execute it once you trust the check
ros2 run ai_controller replicate_rollout --ros-args \
    -p rollout_path:=/home/ros2_ws/src/ai_controller/saved_rollouts/cod_controller/pick_place/task_01/traj_000.pkl \
    -p context_trajectory_path:=/home/ros2_ws/src/ai_controller/saved_rollouts/cod_controller/pick_place/task_01/context_000.pkl \
    -p save_video:=True
``` 

Script-Controller (scripted, click-to-target pick-place, no learned model) has its own
launch command and instructions in [Script-Controller](script_controller.md).

**Docker-2: Launch Zed-Camera Drivers**
```bash
ros2 launch zed_camera_driver zed_multi_camera.launch.py \
    camera_model:='zedm' \
    config_camera_path:=src/zed_camera/zed_camera_driver/config/camera_config.yaml \
    cameras_yaml:=src/zed_camera/zed_camera_driver/config/multi_cameras.yaml \
    rviz:=false
```

## OpenVLA Dependencies

Install the following inside the container (`docker exec -it ur_robotiq_teleoperation_container bash`).

### Required
```bash
cd  /home/ros2_ws/src/ai_controller/ai_controller/models/openvla_controller
git clone https://github.com/ciccio42/openvla-oft.git
source /home/ros2_ws/src/ai_controller/ai_controller/models/requirements/openvla_oft_installation.sh
cd openvla-oft
pip install -e . --break-system-packages

# unit test
cd /home/ros2_ws/src/ai_controller/ai_controller/models/openvla_controller
python3 test.py 
```

### Optional – quantization (reduces GPU memory from ~14 GB to ~8 / ~4 GB)
```bash
pip install bitsandbytes --break-system-packages
```

### Configuration
Set the ROS parameters to use OpenVLA:
```bash
ros2 run ai_controller ai_controller_node \
  --ros-args \
  -p ai_controller_target:=openvla_controller \
  -p model_config_path:=/home/ros2_ws/src/ai_controller/ai_controller/models/openvla_controller/openvla_config.yaml \
  -p task_name:=pick_place
```

## TinyVLA Dependencies

Install the following inside the container (`docker exec -it ur_robotiq_teleoperation_container bash`).
Reference (validated) inference code this controller is ported from lives at
`~/Desktop/Multi-Task-LFD/repo/VLA-Bench/robosuite_test/models/tinyvla.py`; the
original TinyVLA training repo is at `~/Desktop/Multi-Task-LFD/repo/TinyVLA`.

### Required
```bash
cd /home/ros2_ws/src/ai_controller/ai_controller/models/tinyvla_controller
git clone https://github.com/ciccio42/TinyVLA.git

# llava_pythia (model/tokenizer/image-processor code) and policy_heads (action
# head implementations: act / droid_diffusion / transformer_diffusion)
cd TinyVLA/llava-pythia && pip install -e . --break-system-packages
cd ../policy_heads && pip install -e . --break-system-packages

pip uninstall torch torchvision --break-system-packages
pip install "torch==2.7.0" "torchvision==0.22.0" --index-url https://download.pytorch.org/whl/cu128 --break-system-packages
pip install ipython --break-system-packages --ignore-installed psutil
pip install "diffusers==0.39.0" --break-system-packages
pip uninstall flash-attn -y  --break-system-packages


# pip install "numpy<2" opencv-python --force-reinstall --break-system-packages
# # pip install "huggingface-hub<1.0,>=0.19.3" --break-system-package
# pip install "deepspeed==0.18.1" --break-system-packages
# pip install "bitsandbytes==0.48.0" --break-system-packages
# pip install "sentencepiece==0.1.99" --break-system-packages
# pip install "timm==0.6.13" --break-system-packages
# pip install "torch==2.7.0" --break-system-packages
# pip install "torchvision==0.22.0" --break-system-packages
# pip install pyquaternion --break-system-packages --ignore-installed psutil

export PYTHONPATH=$PYTHONPATH:/home/ros2_ws/src/ai_controller/ai_controller/models/tinyvla_controller/TinyVLA
export PYTHONPATH=$PYTHONPATH:/home/ros2_ws/src/ai_controller/ai_controller/models/tinyvla_controller/TinyVLA/llava-pythia

# unit test
cd /home/ros2_ws/src/ai_controller/ai_controller/models/tinyvla_controller
python3 test.py \
        --config /home/ros2_ws/src/ai_controller/ai_controller/models/tinyvla_controller/tinyvla_config.yaml
```

Checkpoint layout expected by `tinyvla_config.yaml` (`model_path` / `model_base`):
a LoRA (or merged) checkpoint directory, its Llava-Pythia-1.3B base, and a
`dataset_stats.pkl` (qpos/action normalization stats) one level above
`model_path` - see the comments in `tinyvla_config.yaml` and
`TinyVLAPolicy.__init__` in `tinyvla.py`.

### Configuration
Set the ROS parameters to use TinyVLA:
```bash
ros2 run ai_controller ai_controller_node \
  --ros-args \
  -p ai_controller_target:=tinyvla_controller \
  -p model_config_path:=/home/ros2_ws/src/ai_controller/ai_controller/models/tinyvla_controller/tinyvla_config.yaml \
  -p task_name:=pick_place
```

## Dependencies to bring in docker
```bash
python3 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128 --break-system-packages
python3 -m pip install hydra-core omegaconf einops torchsummary tqdm --break-system-packages
```
