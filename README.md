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

## UR5e + Robotiq Gripper + Table

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
  --name ur_robotiq_container \
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

# Launch multi-camera 
#ros2 launch zed_multi_camera zed_multi_camera.launch.py \
#      cam_names:='[zed_front,zed_left,zed_right]' \
#      cam_models:='[zedm,zedm,zedm]' \
#      cam_serials:='[16450494,11990492,15689351]'

ros2 launch zed_camera_calibration zed_multi_camera_calibration.launch.py camera_model:=zedm config_camera_path:=src/zed_camera/zed_camera_calibration/config/camera_config.yaml rviz:=false

# Run interactive calibration
ros2 launch zed_camera_driver zed_multi_camera.launch.py \
    camera_model:='zedm' \
    config_camera_path:=src/zed_camera/zed_camera_calibration/config/camera_config.yaml \
    cameras_yaml:=src/zed_camera/zed_camera_calibration/config/multi_cameras.yaml \
    rviz:=true

ros2 run zed_camera_calibration interactive_aruco_calibration.py \
    --ros-args \
    -p cameras_config:=src/zed_camera/zed_camera_calibration/config/multi_cameras.yaml \
    -p cameras_yaml:=src/zed_camera/zed_camera_calibration/config/camera_config.yaml \
    -p aruco_info:=src/zed_camera/zed_camera_calibration/config/aruco_frontal_camera.yaml

```

## UR5e + Robotiq Gripper + Table + Teleoperation

```bash
docker build -t ur_robotiq_teleoperation . -f UR_Robotiq_Teleoperation

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
  -v /dev/input:/dev/input \
  -v /home/mivia/Scrivania/Ur5e/ros2/ur_ros2/UR5e-2f-85/ur5e_2f_85:/home/ros2_ws/src/ur5e_2f_85 \
  --name ur_robotiq_teleoperation_container \
  ur_robotiq_teleoperation
```

```bash
# Run ur-sim
docker run --rm -it \
  -e ROBOT_MODEL=UR5e \
  --net ursim_net \
  --ip 192.168.56.101 \
  --privileged \
  --cap-add=NET_ADMIN \
  -p 5900:5900 -p 6080:6080 \
  -v /home/mivia/Scrivania/Ur5e/ros2/ur_ros2/UR5e-2f-85/ur_programs:/ursim/programs \
  ursim_e-series

# RUN ur-driver
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.56.101 \
  kinematics_params_file:=/home/ros2_ws/src/ur5e_2f_85/sim_calibration.yaml \
  description_launchfile:="/home/ros2_ws/src/ur5e_2f_85/ur5e_2f_85_description/launch/ur5e_2f_85_display_control.launch.py" \
  launch_rviz:=false

ros2 launch ur5e_2f_85_moveit_config move_group_servo.launch.py launch_servo:=true
ros2 launch ur5e_2f_85_teleoperation ur5e_teleoperation.launch.py
```

## Dataset Collector

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
  -v /dev/input:/dev/input \
  -v /home/mivia/Scrivania/Ur5e/ros2/ur_ros2/UR5e-2f-85/ur5e_2f_85:/home/ros2_ws/src/ur5e_2f_85 \
  -v /home/mivia/Scrivania/Ur5e/ros2/ur_ros2/UR5e-2f-85/dataset_collector:/home/ros2_ws/src/dataset_collector \
  -v /home/mivia/Scrivania/Ur5e/ros2/ur_ros2/UR5e-2f-85/moveit_controller:/home/ros2_ws/src/moveit_controller \
  --name ur_robotiq_teleoperation_container \
  ur_robotiq_teleoperation
```

**Zed-Docker**
```bash
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
```

**Docker-1: Launch UR-Driver**
```bash
# Launch external-controller
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.56.101 \
  kinematics_params_file:=/home/ros2_ws/src/ur5e_2f_85/sim_calibration.yaml \
  description_launchfile:="/home/ros2_ws/src/ur5e_2f_85/ur5e_2f_85_description/launch/ur5e_2f_85_display_control.launch.py" \
  launch_rviz:=false

# Launch Movegroup
docker exec -it ur_robotiq_teleoperation_container  bash
ros2 launch ur5e_2f_85_moveit_config move_group_servo.launch.py launch_servo:=true

# Launch Teleoperation Node
docker exec -it ur_robotiq_teleoperation_container  bash
ros2 launch ur5e_2f_85_teleoperation ur5e_teleoperation.launch.py
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

```



# Usefull commands
```bash
docker exec -it <ID_OR_NAME> bash

# Clean docker build cache
docker builder prune --all
# Clean dandling image
docker image prune -f
```

# ToDo
* [X] Description file 

* [X] Moveit Config

* [X] Test with simulated UR5e robot

* [X] Integrate Cameras
  + [X] Calibration tool
  + [X] Multicamera

* [X] Integrate Teleoperation
* [] Integrate DatasetCollection
  + [] Moveit home position
  + [] Save trajectories
* [] Controllers
  + [] Dataset trajectory reply
