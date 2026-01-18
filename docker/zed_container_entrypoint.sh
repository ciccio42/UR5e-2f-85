#!/bin/bash

# Build
apt update && apt install -y iputils-ping
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# setup ros environment
source /root/.bashrc
source "/opt/ros/$ROS_DISTRO/setup.bash" 
source "$ROS_WS/install/setup.bash"

exec "$@"