# Docker for UR-ROS2

## ROS2 docker

**Build**
```bash
docker build -t ros2 . -f ros2Jazzy
``` 

**Run**
```bash
xhost +local:docker
docker run -it --rm \
  --gpus all \
  --privileged \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  ros2
``` 


## UR

**Build**
```bash
docker build -t ur_ros2 . -f URRos2
``` 


**Run**
```bash
xhost +local:docker
docker run -it --rm \
  --gpus all \
  --privileged \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  ur_ros2
``` 

## UR5e + Robotiq Gripper + Tavolo

**Build**
```bash
docker build -t ur_robotiq . -f UR_Robotiq 
``` 

**Run**
```bash
xhost +local:docker
docker run -it --rm \
  --gpus all \
  --privileged \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /home/mivia/Scrivania/Ur5e/ros2/ur_ros2/UR5e-2f-85/ur5e_2f_85:/home/ros2_ws/src/ur5e_2f_85 \
  ur_robotiq
``` 

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --packages-select ur5e_2f_85_description ur5e_2f_85_moveit_config 
source install/setup.bash 
# test moveit demo
ros2 launch ur5e_2f_85_moveit_config demo.launch.py 
```

## Bringup robot

### For simulation robot
```bash
# Build Docker - Only if you do not have the image
docker build -t ursim_e-series . -f UR_SIM

# Create subnet
docker network create --subnet=192.168.56.0/24 ursim_net

# Run docker assigning IP, set ur5e
docker run --rm -it \
  -e ROBOT_MODEL=UR5e \
  --net ursim_net \
  --ip 192.168.56.101 \
  --privileged \
  --cap-add=NET_ADMIN \
  -p 5900:5900 -p 6080:6080 \
  -v /home/mivia/Scrivania/Ur5e/ros2/ur_ros2/UR5e-2f-85/ur_programs:/ursim/programs \
  ursim_e-series

xhost +local:docker
docker run -it --rm \
  --gpus all \
  --privileged \
  --cap-add=SYS_NICE \
  --cpuset-cpus="0-1" \
  --network ursim_net \
  --ip 192.168.56.102 \
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
  -v /home/mivia/Scrivania/Ur5e/ros2/ur_ros2/UR5e-2f-85/ur5e_2f_85:/home/ros2_ws/src/ur5e_2f_85 \
  ur_robotiq

```

```bash
# Bringup without moveit
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash 

# Calibration (only the first time)
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=192.168.56.101 \
  target_filename:="/home/ros2_ws/src/ur5e_2f_85/sim_calibration.yaml"


# Launch without Tool
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.56.101 \
  kinematics_params_file:=/home/ros2_ws/src/ur5e_2f_85/sim_calibration.yaml \
  description_launchfile:="/home/ros2_ws/src/ur5e_2f_85/ur5e_2f_85_description/launch/ur5e_2f_85_display_control.launch.py"

# Launch movegroup
ros2 launch ur5e_2f_85_moveit_config move_group.launch.py

```

## ZED Cameras docker
```bash
# Build docker
source zed-build-docker-image.sh

xhost +local:docker
docker run -it --rm \
  --gpus all \
  --privileged \
  --network ursim_net \
  --ip 192.168.56.103 \
  --ipc=host \
  --pid=host \
  --device=/dev/bus/usb \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev:/dev \
  -v /home/mivia/Scrivania/Ur5e/ros2/ur_ros2/UR5e-2f-85/zed_camera:/home/ros2_ws/src/zed_camera \
  --name zed_camera_container \
  5.1-ros2-devel-cuda13.0-ubuntu24.04

# Test camera
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm
# RVIZ
ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zedm

# Aruco detection
ros2 launch zed_camera_calibration zed_camera_calibration.launch.py camera_model:=zedm config_camera_path:=src/zed_camera/zed_camera_calibration/config/camera_config.yaml
```



# Usefull commands
```bash
docker exec -it <ID_OR_NAME> bash

# Clean docker build cache
docker builder prune --all

```

# ToDo
[X] Description file
[X] Moveit Config
[X] Test with simulated UR5e robot
[] Integrate Cameras
  [] Calibration tool
[] Integrate Teleoperation
[] Integrate DatasetCollection
