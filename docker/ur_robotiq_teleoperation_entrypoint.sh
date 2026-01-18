#!/bin/bash

# Build
source "/opt/ros/$ROS_DISTRO/setup.bash" 
chmod 777 -R ur5e_2f_85_teleoperation
colcon build --packages-select ur5e_2f_85_description ur5e_2f_85_moveit_config ur5e_2f_85_teleoperation

# setup ros environment
source "$ROS_WS/install/setup.bash"

exec "$@"