# primo terminale
## UR-Container 
```bash
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





## Zed Container secondo terminale
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


# primo terminale
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




  ### Secondo terminale
  
**Docker-2: Launch Zed-Camera Drivers**
```bash
ros2 launch zed_camera_driver zed_multi_camera.launch.py \
    camera_model:='zedm' \
    config_camera_path:=src/zed_camera/zed_camera_driver/config/camera_config.yaml \
    cameras_yaml:=src/zed_camera/zed_camera_driver/config/multi_cameras.yaml \
    rviz:=false
```



# Terzo terminale

docker exec -it ur_robotiq_teleoperation_container  bash
# poi
source install/setup.bash
ros2 launch ur5e_2f_85_moveit_config move_group_servo.launch.py launch_servo:=true


# Quarto terminale
# Run moveit_controller
docker exec -it ur_robotiq_teleoperation_container  bash
# poi
source install/setup.bash
ros2 run moveit_controller moveit_controller_node


# Quinto terminale
cd Alex/UR5e-2f-85
export UR5e_2f_85_PATH="$PWD"

bash ai_controller/ai_controller/models/mimic_video_controller/run_mimic_video_container.sh


ros2 run ai_controller ai_controller_node --ros-args \
  -p move_robot:=True \
  -p ai_controller_target:=mimic_video_controller \
  -p model_config_path:="$MIMIC_VIDEO_CONTROLLER_CONFIG" \
  -p task_name:=pick_place \
  -p "camera_topic:=['/zed_front/zed_node/rgb/color/rect/image']"