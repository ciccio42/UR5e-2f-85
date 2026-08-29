# Starting the Real SeeDo Setup

## 1. ZED Cameras

On the host system:

```bash
cd /home/asus-mivia/Desktop/Angelo/UR5e-2f-85
```

Start the ZED container:

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
  -v "$(pwd)/zed_camera:/home/ros2_ws/src/zed_camera" \
  --name zed_camera_container \
  5.3-ros2-devel-l4t-r38.4
```

Inside the container, start the multi-camera driver:

```bash
ros2 launch zed_camera_driver zed_multi_camera.launch.py \
  camera_model:='zedm' \
  config_camera_path:=src/zed_camera/zed_camera_driver/config/camera_config.yaml \
  cameras_yaml:=src/zed_camera/zed_camera_driver/config/multi_cameras.yaml \
  rviz:=false
```

---

## 2. SeeDo Container + UR Driver

On the host system:

```bash
cd /home/asus-mivia/Desktop/Angelo/UR5e-2f-85
```

Start the SeeDo container:

```bash
export ROBOT_IP=172.16.174.59

docker run -it --rm \
  --name seedo_ros2_container \
  --gpus all \
  --privileged \
  --cap-add=SYS_NICE \
  --network host \
  --ipc=host \
  --pid=host \
  --ulimit memlock=-1:-1 \
  --ulimit rtprio=99 \
  --shm-size=1g \
  --security-opt seccomp=unconfined \
  -e DISPLAY=$DISPLAY \
  -e ROBOT_IP=${ROBOT_IP} \
  -e OPENAI_API_KEY="${OPENAI_API_KEY}" \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -e BUILD_SEEDO_ROS=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev/input:/dev/input \
  -v "$(pwd)":/home/ros2_ws/src/UR5e-2f-85 \
  -v "$(pwd)/../test_dataset":/test_dataset:ro \
  -v "$(pwd)/../seedo_tests":/seedo_tests \
  -v "$(pwd)/../test_isaac":/test_isaac \
  -v "$(pwd)/../test_real":/test_real \
  -v "$(pwd)/../scene_capture":/scene_capture \
  seedo_ros2:latest
```

Inside the container:

```bash
source /home/ros2_ws/install/setup.bash

ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=172.16.174.59 \
  use_tool_communication:=true \
  tool_voltage:=24 \
  tool_parity:=0 \
  tool_baud_rate:=115200 \
  tool_stop_bits:=1 \
  tool_rx_idle_chars:=1.5 \
  tool_tx_idle_chars:=3.5 \
  tool_device_name:=/tmp/ttyUR \
  kinematics_params_file:=/home/ros2_ws/src/UR5e-2f-85/ur5e_2f_85/real_robot_calibration.yaml \
  controllers_file:=/home/ros2_ws/src/UR5e-2f-85/ur5e_2f_85/ur5e_2f_85_description/config/ur5e_2f_85_controllers.yaml \
  description_launchfile:="/home/ros2_ws/src/UR5e-2f-85/ur5e_2f_85/ur5e_2f_85_description/launch/ur5e_2f_85_display_control.launch.py" \
  launch_rviz:=false
```

---

## 3. Enable External Control

On the UR tablet, start the **External Control** program.

---

## 4. `table_0` Static TF

Open a new terminal:

```bash
docker exec -it seedo_ros2_container bash

source /home/ros2_ws/install/setup.bash

ros2 run tf2_ros static_transform_publisher \
  --x 0.00 --y 0.612 --z -0.120 \
  --qx 0.000 --qy 0.000 --qz 1.000 --qw 0.000 \
  --frame-id base_link \
  --child-frame-id table_0
```

---

## 5. MoveIt

Open a new terminal:

```bash
docker exec -it seedo_ros2_container bash

source /home/ros2_ws/install/setup.bash

ros2 launch ur5e_2f_85_moveit_config move_group_servo.launch.py \
  launch_servo:=true
```

---

## 6. MoveIt Controller

Open a new terminal:

```bash
docker exec -it seedo_ros2_container bash

source /home/ros2_ws/install/setup.bash

ros2 run moveit_controller moveit_controller_node \
  --ros-args \
  -p execute_trajectory:=true
```

---

## 7. Run the SeeDo Test

Open a new terminal:

```bash
docker exec -it seedo_ros2_container bash

export QT_QPA_PLATFORM=offscreen
bash /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/run_seedo_real_test.sh
```
