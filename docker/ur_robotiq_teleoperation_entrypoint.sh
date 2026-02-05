#!/bin/bash

# Build
source "/opt/ros/$ROS_DISTRO/setup.bash" 
apt install -y ros-$ROS_DISTRO-moveit-py
source "/opt/ros/$ROS_DISTRO/setup.bash" 
colcon build --packages-select ur5e_2f_85_description \
                                ur5e_2f_85_moveit_config \
                                ur5e_2f_85_teleoperation_msg \
                                ur5e_2f_85_teleoperation \
                                dataset_collector_pkg \
                                moveit_controller \
                                moveit_controller_srvs

# setup ros environment
source "$ROS_WS/install/setup.bash"

exec "$@"