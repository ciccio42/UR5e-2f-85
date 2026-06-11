# Bringup Robot

## For simulation robot
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
  -v ${UR5e_2f_85_PATH}/ur_programs:/ursim/programs \
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
  -v ${UR5e_2f_85_PATH}/ur5e_2f_85:/home/ros2_ws/src/ur5e_2f_85 \
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