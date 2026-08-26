#!/usr/bin/env bash

set -o pipefail

source /home/ros2_ws/install/setup.bash

export PYTHONPATH="${PYTHONPATH:-}:/opt/seedo-venv/lib/python3.12/site-packages"

REPO_ROOT="/home/ros2_ws/src/UR5e-2f-85"

SCRIPT_DIR="${REPO_ROOT}/ai_controller/ai_controller/models/seedo_controller"
SCENE_CONFIGURATOR="${SCRIPT_DIR}/configure_isaac_scene.py"

REAL_TEST_ROOT="/test_isaac"

TASK=""
TRAJECTORY=""
CONFIGURE_SCENE=false

# ============================================================
# OPTIONAL ISAAC SCENE CONFIGURATION ARGUMENTS
# ============================================================

while [[ $# -gt 0 ]]; do
    case "$1" in
        --task)
            if [[ $# -lt 2 ]]; then
                echo "ERROR: --task requires a value."
                exit 1
            fi

            TASK="$2"
            CONFIGURE_SCENE=true
            shift 2
            ;;

        --trajectory)
            if [[ $# -lt 2 ]]; then
                echo "ERROR: --trajectory requires a value."
                exit 1
            fi

            TRAJECTORY="$2"
            CONFIGURE_SCENE=true
            shift 2
            ;;

        *)
            echo "Unknown argument: $1"
            echo "Usage: $0 [--task 00-15] [--trajectory 000-039]"
            exit 1
            ;;
    esac
done

# ============================================================
# SCENE CONFIGURATION SELECTION
# ============================================================

if [[ "${CONFIGURE_SCENE}" == false ]]; then
    read -rp "Do you want to configure the scene? [y/n]: " CONFIGURE_REPLY

    case "${CONFIGURE_REPLY}" in
        y|Y|yes|YES|Yes|s|S|si|SI|Si)
            CONFIGURE_SCENE=true
            ;;
        *)
            CONFIGURE_SCENE=false
            ;;
    esac
fi

# ============================================================
# ISAAC SCENE TASK / TRAJECTORY
# ============================================================

if [[ "${CONFIGURE_SCENE}" == true ]]; then

    if [[ -z "${TASK}" ]]; then
        read -rp "Isaac scene task [00-15]: " TASK
    fi

    if [[ -z "${TRAJECTORY}" ]]; then
        read -rp "Isaac scene trajectory [000-039]: " TRAJECTORY
    fi

    if ! [[ "${TASK}" =~ ^[0-9]{1,2}$ ]] || (( 10#${TASK} > 15 )); then
        echo "ERROR: task must be between 00 and 15."
        exit 1
    fi

    if ! [[ "${TRAJECTORY}" =~ ^[0-9]{1,3}$ ]] || (( 10#${TRAJECTORY} > 39 )); then
        echo "ERROR: trajectory must be between 000 and 039."
        exit 1
    fi

    printf -v TASK "%02d" "$((10#${TASK}))"
    printf -v TRAJECTORY "%03d" "$((10#${TRAJECTORY}))"

    echo "================================================="
    echo "Configuring Isaac scene"
    echo "Task:       ${TASK}"
    echo "Trajectory: ${TRAJECTORY}"
    echo "================================================="

    python3 "${SCENE_CONFIGURATOR}" \
        --task "$((10#${TASK}))" \
        --trajectory "$((10#${TRAJECTORY}))"

    SCENE_CONFIG_EXIT_CODE=$?

    if [[ ${SCENE_CONFIG_EXIT_CODE} -ne 0 ]]; then
        echo
        echo "ERROR: Isaac scene configuration failed."
        echo "SeeDo test will NOT be started."
        exit "${SCENE_CONFIG_EXIT_CODE}"
    fi

    echo
    echo "Isaac scene configured successfully."
    echo

else

    echo "================================================="
    echo "Isaac scene configuration SKIPPED"
    echo "Using the current scene."
    echo "================================================="
    echo

fi

# ============================================================
# SELECT DEMONSTRATION TASK
#
# This is intentionally independent from the Isaac scene task.
# It determines:
#   - demonstration video
#   - test folder name
#
# Examples:
#   first run task 00  -> test_task00_1
#   second run task 00 -> test_task00_2
#   first run task 03  -> test_task03_1
# ============================================================

read -rp "Enter demonstration task ID [00-15]: " TASK_ID_INPUT

if ! [[ "${TASK_ID_INPUT}" =~ ^[0-9]+$ ]]; then
    echo "ERROR: task ID must be numeric."
    exit 1
fi

TASK_ID_NUM="$((10#${TASK_ID_INPUT}))"

if (( TASK_ID_NUM < 0 || TASK_ID_NUM > 15 )); then
    echo "ERROR: task ID must be between 00 and 15."
    exit 1
fi

TASK_ID="$(printf '%02d' "${TASK_ID_NUM}")"

DEMO_PATH="/test_dataset/pick_place/human_rgb_pick_place/task_${TASK_ID}/traj000/converted/traj000-h264-30fps_modified.mp4"

if [[ ! -f "${DEMO_PATH}" ]]; then
    echo "ERROR: demonstration video not found:"
    echo "${DEMO_PATH}"
    exit 1
fi

# ============================================================
# AUTOMATIC TEST NUMBER
# ============================================================

TEST_INDEX=1

while [[ -e "${REAL_TEST_ROOT}/test_task${TASK_ID}_${TEST_INDEX}" ]]; do
    ((TEST_INDEX++))
done

TEST_NAME="test_task${TASK_ID}_${TEST_INDEX}"
RUN_DIR="${REAL_TEST_ROOT}/${TEST_NAME}"

TIMESTAMP="$(date '+%Y%m%d_%H%M%S_%3N')"

ACTION_PLAN_SOURCE="/seedo_tests/ai_controller_node_interactive/action_planning"

CONFIG_PATH="${REPO_ROOT}/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml"

PRECOMPUTED_PLAN="${ACTION_PLAN_SOURCE}/action_plan.json"

mkdir -p "${RUN_DIR}"
mkdir -p "${RUN_DIR}/action_planning"
mkdir -p "${RUN_DIR}/rollouts"

echo "================================================="
echo "SeeDo Isaac Sim test"
echo "Demo task ID: ${TASK_ID}"
echo "Test number: ${TEST_INDEX}"
echo "Test name: ${TEST_NAME}"
echo "Run directory: ${RUN_DIR}"
echo "Demo video: ${DEMO_PATH}"

if [[ "${CONFIGURE_SCENE}" == true ]]; then
    echo "Isaac scene task: ${TASK}"
    echo "Isaac trajectory: ${TRAJECTORY}"
else
    echo "Isaac scene: current scene"
fi

echo "================================================="

# ============================================================
# SAVE ACTION PLANNING ARTIFACTS
# ============================================================

cp -a \
    "${ACTION_PLAN_SOURCE}/." \
    "${RUN_DIR}/action_planning/"

# ============================================================
# SAVE CONFIGURATION
# ============================================================

cp \
    "${CONFIG_PATH}" \
    "${RUN_DIR}/seedo_controller.yaml"

# ============================================================
# RUN SEEDO
# ============================================================

set +e

# Precomputed action-plan version:
# 
ros2 run ai_controller ai_controller_node \
    --ros-args \
    -p ai_controller_target:=seedo_controller \
    -p model_config_path:="${CONFIG_PATH}" \
    -p task_name:=pick_place \
    -p demo_path:="${DEMO_PATH}" \
    -p seedo_precomputed_action_plan_path:=/test_isaac/test_task04_1/action_planning/action_plan.json \
    -p seedo_artifacts_dir:="${RUN_DIR}" \
    -p save_rollout_path:="${RUN_DIR}/rollouts" \
    -p move_robot:=true \
    -p seedo_execute_gripper:=true \
    -p camera_topic:="['/zed_front/zed_node/rgb/color/rect/image','/zed_left/zed_node/rgb/color/rect/image','/zed_right/zed_node/rgb/color/rect/image','/zed_front/zed_node/rgb/color/rect/image']" \
    -p seedo_record_depth_topics:="['/zed_front/zed_node/depth/depth_registered','/zed_left/zed_node/depth/depth_registered','/zed_right/zed_node/depth/depth_registered','/zed_front/zed_node/depth/depth_registered']" \
    2>&1 | tee "${RUN_DIR}/console.log"

# ros2 run ai_controller ai_controller_node \
#     --ros-args \
#     -p ai_controller_target:=seedo_controller \
#     -p model_config_path:="${CONFIG_PATH}" \
#     -p task_name:=pick_place \
#     -p demo_path:="${DEMO_PATH}" \
#     -p seedo_artifacts_dir:="${RUN_DIR}" \
#     -p save_rollout_path:="${RUN_DIR}/rollouts" \
#     -p move_robot:=true \
#     -p seedo_execute_gripper:=true \
#     -p camera_topic:="['/zed_front/zed_node/rgb/color/rect/image','/zed_left/zed_node/rgb/color/rect/image','/zed_right/zed_node/rgb/color/rect/image','/zed_front/zed_node/rgb/color/rect/image']" \
#     -p seedo_record_depth_topics:="['/zed_front/zed_node/depth/depth_registered','/zed_left/zed_node/depth/depth_registered','/zed_right/zed_node/depth/depth_registered','/zed_front/zed_node/depth/depth_registered']" \
#     2>&1 | tee "${RUN_DIR}/console.log"

EXIT_CODE=${PIPESTATUS[0]}

set -e

# ============================================================
# SAVE RUN STATUS
# ============================================================

cat > "${RUN_DIR}/run_status.json" <<EOF
{
    "timestamp": "${TIMESTAMP}",
    "test_name": "${TEST_NAME}",
    "task_id": "${TASK_ID}",
    "test_index": ${TEST_INDEX},
    "demo_path": "${DEMO_PATH}",
    "isaac_scene_configured": ${CONFIGURE_SCENE},
    "isaac_scene_task": "${TASK}",
    "isaac_scene_trajectory": "${TRAJECTORY}",
    "exit_code": ${EXIT_CODE},
    "mode": "isaac_sim"
}
EOF

echo
echo "================================================="
echo "Test finished with exit code: ${EXIT_CODE}"
echo "Artifacts saved in:"
echo "${RUN_DIR}"
echo "================================================="

exit "${EXIT_CODE}"