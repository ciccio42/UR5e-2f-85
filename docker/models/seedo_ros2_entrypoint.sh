#!/bin/bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

cd "${ROS_WS}"

UR_APPLICATION_ROOT="${ROS_WS}/src/UR5e-2f-85"

if [ -d "${UR_APPLICATION_ROOT}" ]; then
    export PYTHONPATH="\
${UR_APPLICATION_ROOT}/ai_controller:\
${UR_APPLICATION_ROOT}/ai_controller/ai_controller/models/seedo_controller/models/SeeDo:\
${UR_APPLICATION_ROOT}/ai_controller/ai_controller/models/seedo_controller/models/SeeDo/VLM_CaP:\
/opt/models:\
/opt/models/GroundingDINO:\
${PYTHONPATH:-}"
fi

if [ -d "${ROS_WS}/src" ] && \
   find "${ROS_WS}/src" -name package.xml -print -quit | grep -q .; then

    echo "[entrypoint] ROS 2 packages found"

    if [ "${BUILD_ROS_WS:-0}" = "1" ]; then
        echo "[entrypoint] Building full ROS 2 workspace..."
        colcon build --symlink-install

    elif [ "${BUILD_SEEDO_ROS:-0}" = "1" ]; then
        echo "[entrypoint] Building ROS 2 packages required by SeeDo..."

        export PYTHON_EXECUTABLE=/usr/bin/python3

        colcon build \
            --symlink-install \
            --cmake-args \
                -DPython3_EXECUTABLE=/usr/bin/python3 \
            --packages-select \
            moveit_controller_srvs \
            moveit_controller \
            ur5e_2f_85_teleoperation_msg \
            dataset_collector_pkg \
            ai_controller \
            ur5e_2f_85_description \
            ur5e_2f_85_moveit_config

    else
        echo "[entrypoint] Skipping ROS 2 workspace build."
        echo "[entrypoint] Set BUILD_ROS_WS=1 for full build."
        echo "[entrypoint] Set BUILD_SEEDO_ROS=1 for SeeDo dependencies."
    fi
fi

if [ -f "${ROS_WS}/install/setup.bash" ]; then
    source "${ROS_WS}/install/setup.bash"
fi

if [ -f "/opt/topic_based_ws/install/setup.bash" ]; then
    source "/opt/topic_based_ws/install/setup.bash"
fi

exec "$@"