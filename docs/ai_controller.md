# AI-Controller

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
  -v ${UR5e_2f_85_PATH}/ai_controller:/home/ros2_ws/src/ai_controller \
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

## Dependencies to bring in docker
```bash
python3 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128 --break-system-packages
python3 -m pip install hydra-core omegaconf einops torchsummary tqdm --break-system-packages
```
