#!/bin/bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"
cd "${ROS_WS}"

if [ -d "${ROS_WS}/src" ] && \
   find "${ROS_WS}/src" -name package.xml -print -quit | grep -q .; then
    echo "[entrypoint] ROS 2 packages found"

    if [ "${BUILD_ROS_WS:-0}" = "1" ]; then
        echo "[entrypoint] Building ROS 2 workspace..."
        colcon build --symlink-install
    else
        echo "[entrypoint] Skipping build. Set BUILD_ROS_WS=1 to build at startup."
    fi
fi

if [ -f "${ROS_WS}/install/setup.bash" ]; then
    source "${ROS_WS}/install/setup.bash"
fi

exec "$@"