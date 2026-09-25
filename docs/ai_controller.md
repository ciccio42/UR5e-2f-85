# AI-Controller

## UR-Container
```bash
export ROBOT_IP=192.168.1.100

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
  -v ${UR5e_2f_85_PATH}/zed_camera/zed-ros2-description:/home/ros2_ws/src/zed-ros2-description\
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
  -v ${UR5e_2f_85_PATH}/zed_camera/zed_docker_cache/resources:/usr/local/zed/resources \
  -v ${UR5e_2f_85_PATH}/zed_camera/zed_docker_cache/settings:/usr/local/zed/settings \
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

# Run AI-Controller: see ai_controller_models/<model>.md (linked from "## Models"
# below) for that model's installation steps and the exact ros2 run command.

# Replicate saved trajectories
# add -p dry_run:=false to actually execute it once you trust the check
ros2 run ai_controller replicate_rollout --ros-args \
    -p rollout_path:=/home/ros2_ws/src/ai_controller/saved_rollouts/cod_controller/pick_place/task_01/traj_000.pkl \
    -p context_trajectory_path:=/home/ros2_ws/src/ai_controller/saved_rollouts/cod_controller/pick_place/task_01/context_000.pkl \
    -p save_video:=True
``` 

**Docker-2: Launch Zed-Camera Drivers**
docker exec -it zed_camera_container  bash
```bash
ros2 launch zed_camera_driver zed_multi_camera.launch.py \
    camera_model:='zedm' \
    config_camera_path:=src/zed_camera/zed_camera_driver/config/camera_config.yaml \
    cameras_yaml:=src/zed_camera/zed_camera_driver/config/multi_cameras.yaml \
    rviz:=false
```

## Models

Each AI-controller model has its own page with installation steps and the
exact `ros2 run` command to launch it (all of them assume the Docker/UR-driver/
`moveit_controller` setup above is already running):

- [COD-Controller](ai_controller_models/cod_controller.md)
- [OpenVLA-Controller](ai_controller_models/openvla_controller.md)
- [TinyVLA-Controller](ai_controller_models/tinyvla_controller.md)
- [OSVI-WM-Controller](ai_controller_models/osvi_controller.md)
- [OSVI-AWDA-Controller](ai_controller_models/osvi_awda_controller.md)

Script-Controller (scripted, click-to-target pick-place, no learned model) has its own
launch command and instructions in [Script-Controller](script_controller.md).

## Dependencies to bring in docker
```bash
python3 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128 --break-system-packages
python3 -m pip install hydra-core omegaconf einops torchsummary tqdm --break-system-packages
```
