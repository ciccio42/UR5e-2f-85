#!/bin/bash

# Build
apt update && apt install -y iputils-ping
source "/opt/ros/$ROS_DISTRO/setup.bash" 
colcon build --packages-select ur5e_2f_85_description ur5e_2f_85_moveit_config

# setup ros environment
source "$ROS_WS/install/setup.bash"

exec "$@"