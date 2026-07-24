# Dataset Collector
**UR-Docker**

### For simulation robot
```bash
# Build Docker - Only if you do not have the image
docker build -t ursim_e-series . -f UR_SIM
docker build -t ur_robotiq_teleoperation . -f UR_Robotiq_Teleoperation

# Create subnet
docker network create --subnet=192.168.56.0/24 ursim_net

# Run docker assigning IP, set ur5e
docker run --rm -it \
  -e ROBOT_MODEL=UR5e \
  --net ursim_net \
  --ip ${ROBOT_IP} \
  --privileged \
  --cap-add=NET_ADMIN \
  -p 5900:5900 -p 6080:6080 \
  -v ${UR5e_2f_85_PATH}/ur_programs:/ursim/programs \
  ursim_e-series


xhost +local:docker
docker run -it --rm \
  --gpus all \
  --privileged \
  --cap-add=SYS_NICE \
  --cpuset-cpus="0-1" \
  --network ursim_net \
  --ip ${HOST_IP} \
  --ipc=host \
  --pid=host \
  --ulimit memlock=-1:-1 \
  --ulimit rtprio=99 \
  --shm-size=1g \
  --security-opt seccomp=unconfined \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev/input:/dev/input \
  -v ${UR5e_2f_85_PATH}/ur5e_2f_85:/home/ros2_ws/src/ur5e_2f_85 \
  -v ${UR5e_2f_85_PATH}/dataset_collector:/home/ros2_ws/src/dataset_collector \
  -v ${UR5e_2f_85_PATH}/moveit_controller:/home/ros2_ws/src/moveit_controller \
  --name ur_robotiq_teleoperation_container \
  ur_robotiq_teleoperation
```

### For real-world robot
```bash
docker build -t ur_robotiq_teleoperation . -f UR_Robotiq_Teleoperation
xhost +local:docker
docker run -it --rm \
  --gpus all \
  --privileged \
  --cap-add=SYS_NICE \
  --cpuset-cpus="0-1" \
  --network host \
  --ipc=host \
  --pid=host \
  --ulimit memlock=-1:-1 \
  --ulimit rtprio=99 \
  --shm-size=1g \
  --security-opt seccomp=unconfined \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev/input:/dev/input \
  -v ${UR5e_2f_85_PATH}/ur5e_2f_85:/home/ros2_ws/src/ur5e_2f_85 \
  -v ${UR5e_2f_85_PATH}/dataset_collector:/home/ros2_ws/src/dataset_collector \
  -v ${UR5e_2f_85_PATH}/moveit_controller:/home/ros2_ws/src/moveit_controller \
  -v ${UR5e_2f_85_PATH}/traj_tmp:/traj_tmp \
  -v /home/asus-mivia/Desktop/saved_trajectories:/home/saved_trajectories \
  --name ur_robotiq_teleoperation_container \
  ur_robotiq_teleoperation


# Only the first time
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=${ROBOT_IP} \
  target_filename:="/home/ros2_ws/src/ur5e_2f_85/real_robot_calibration.yaml"

```

**Zed-Docker**
```bash
source zed-build-docker-image.sh

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

**Docker-1: Launch UR-Driver**
```bash
# Launch external-controller [SIM]
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=${ROBOT_IP} \
  kinematics_params_file:=/home/ros2_ws/src/ur5e_2f_85/sim_calibration.yaml \
  description_launchfile:="/home/ros2_ws/src/ur5e_2f_85/ur5e_2f_85_description/launch/ur5e_2f_85_display_control.launch.py" \
  launch_rviz:=false

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

# Launch Movegroup
docker exec -it ur_robotiq_teleoperation_container  bash
source install/setup.bash
ros2 launch ur5e_2f_85_moveit_config move_group_servo.launch.py launch_servo:=true

# Launch Teleoperation Node
docker exec -it ur_robotiq_teleoperation_container  bash
source install/setup.bash
ros2 launch ur5e_2f_85_teleoperation ur5e_teleoperation.launch.py

# Run moveit_controller 
docker exec -it ur_robotiq_teleoperation_container  bash
source install/setup.bash
ros2 run moveit_controller moveit_controller_node

# if you want to replicate a trajectory 
docker exec -it ur_robotiq_teleoperation_container  bash
source install/setup.bash
ros2 launch dataset_collector_pkg  replicate_trajectory.launch.py
```

**Docker-2: Launch Zed-Camera Drivers**
```bash
ros2 launch zed_camera_driver zed_multi_camera.launch.py \
    camera_model:='zedm' \
    config_camera_path:=src/zed_camera/zed_camera_driver/config/camera_config.yaml \
    cameras_yaml:=src/zed_camera/zed_camera_driver/config/multi_cameras.yaml \
    rviz:=false
```

**Docker-1: Launch Dataset-Collector**
```bash
docker exec -it ur_robotiq_teleoperation_container  bash
source install/setup.bash
ros2 run dataset_collector_pkg dataset_collector_node
```