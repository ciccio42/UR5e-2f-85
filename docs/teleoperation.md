# Teleoperation

## UR5e + Robotiq Gripper + Table + Teleoperation

```bash
docker build -t ur_robotiq_teleoperation . -f UR_Robotiq_Teleoperation

xhost +local:docker
docker run -it --rm \
  --gpus all \
  --privileged \
  --cap-add=SYS_NICE \
  --cpuset-cpus="0-1" \
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
  --name ur_robotiq_teleoperation_container \
  ur_robotiq_teleoperation
```

```bash
# Run ur-sim
docker run --rm -it \
  -e ROBOT_MODEL=UR5e \
  --net ursim_net \
  --ip ${ROBOT_IP} \
  --privileged \
  --cap-add=NET_ADMIN \
  -p 5900:5900 -p 6080:6080 \
  -v ${UR5e_2f_85_PATH}/ur_programs:/ursim/programs \
  ursim_e-series

# RUN ur-driver
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=${ROBOT_IP} \
  kinematics_params_file:=/home/ros2_ws/src/ur5e_2f_85/real_robot_calibration.yaml \
  description_launchfile:="/home/ros2_ws/src/ur5e_2f_85/ur5e_2f_85_description/launch/ur5e_2f_85_display_control.launch.py" \
  launch_rviz:=false

ros2 launch ur5e_2f_85_moveit_config move_group_servo.launch.py launch_servo:=true
ros2 launch ur5e_2f_85_teleoperation ur5e_teleoperation.launch.py
```