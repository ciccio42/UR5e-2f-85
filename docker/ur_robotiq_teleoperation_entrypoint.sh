#!/bin/bash

# Build
source "/opt/ros/$ROS_DISTRO/setup.bash" 
apt-get update && apt-get install -y ros-$ROS_DISTRO-moveit-py
source "/opt/ros/$ROS_DISTRO/setup.bash" 
ros2 daemon stop && ros2 daemon start
python3 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128 --break-system-packages
python3 -m pip install hydra-core omegaconf einops torchsummary tqdm --break-system-packages
colcon build --packages-select ur5e_2f_85_description \
                                ur5e_2f_85_moveit_config \
                                ur5e_2f_85_teleoperation_msg \
                                ur5e_2f_85_teleoperation \
                                dataset_collector_pkg \
                                moveit_controller \
                                moveit_controller_srvs \
                                ai_controller

# setup ros environment
source "$ROS_WS/install/setup.bash"

exec "$@"