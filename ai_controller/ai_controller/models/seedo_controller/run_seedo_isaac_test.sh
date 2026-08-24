#!/usr/bin/env bash

set -o pipefail

source /home/ros2_ws/install/setup.bash
export PYTHONPATH="${PYTHONPATH:-}:/opt/seedo-venv/lib/python3.12/site-packages"

REPO_ROOT="/home/ros2_ws/src/UR5e-2f-85"

SCRIPT_DIR="${REPO_ROOT}/ai_controller/ai_controller/models/seedo_controller"
SCENE_CONFIGURATOR="${SCRIPT_DIR}/configure_isaac_scene.py"

TASK=""
TRAJECTORY=""
CONFIGURE_SCENE=false

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

# If no task/trajectory was passed, ask whether the scene
# should be configured at all.
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
# TASK / TRAJECTORY
# ============================================================

if [[ "${CONFIGURE_SCENE}" == true ]]; then

    if [[ -z "${TASK}" ]]; then
        read -rp "Task [00-15]: " TASK
    fi

    if [[ -z "${TRAJECTORY}" ]]; then
        read -rp "Trajectory [000-039]: " TRAJECTORY
    fi

    # Validate task.
    if ! [[ "${TASK}" =~ ^[0-9]{1,2}$ ]] || (( 10#${TASK} > 15 )); then
        echo "ERROR: task must be between 00 and 15."
        exit 1
    fi

    # Validate trajectory.
    if ! [[ "${TRAJECTORY}" =~ ^[0-9]{1,3}$ ]] || (( 10#${TRAJECTORY} > 39 )); then
        echo "ERROR: trajectory must be between 000 and 039."
        exit 1
    fi

    # Normalize formatting.
    printf -v TASK "%02d" "$((10#${TASK}))"
    printf -v TRAJECTORY "%03d" "$((10#${TRAJECTORY}))"

fi

OUTPUT_ROOT="/test_isaac"

if [[ "${CONFIGURE_SCENE}" == true ]]; then

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

TIMESTAMP="$(date '+%Y%m%d_%H%M%S_%3N')"
RUN_DIR="${OUTPUT_ROOT}/${TIMESTAMP}"

ACTION_PLAN_SOURCE="/seedo_tests/ai_controller_node_interactive/action_planning"
CONFIG_PATH="${REPO_ROOT}/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml"
PRECOMPUTED_PLAN="${ACTION_PLAN_SOURCE}/action_plan.json"

mkdir -p "${RUN_DIR}/action_planning"
mkdir -p "${RUN_DIR}/rollouts"

echo "================================================="
echo "SeeDo Isaac Sim test"
echo "Run directory: ${RUN_DIR}"
echo "================================================="

cp -a "${ACTION_PLAN_SOURCE}/." "${RUN_DIR}/action_planning/"
cp "${CONFIG_PATH}" "${RUN_DIR}/seedo_controller.yaml"

set +e

# ros2 run ai_controller ai_controller_node \
#     --ros-args \
#     -p ai_controller_target:=seedo_controller \
#     -p model_config_path:="${CONFIG_PATH}" \
#     -p task_name:=pick_place \
#     -p demo_path:=/test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps_modified.mp4 \
#     -p seedo_precomputed_action_plan_path:="${PRECOMPUTED_PLAN}" \
#     -p seedo_artifacts_dir:="${RUN_DIR}" \
#     -p save_rollout_path:="${RUN_DIR}/rollouts" \
#     -p move_robot:=true \
#     -p seedo_execute_gripper:=true \
#     2>&1 | tee "${RUN_DIR}/console.log"

ros2 run ai_controller ai_controller_node \
    --ros-args \
    -p ai_controller_target:=seedo_controller \
    -p model_config_path:="${CONFIG_PATH}" \
    -p task_name:=pick_place \
    -p demo_path:=/test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps_modified.mp4 \
    -p seedo_artifacts_dir:="${RUN_DIR}" \
    -p save_rollout_path:="${RUN_DIR}/rollouts" \
    -p move_robot:=true \
    -p seedo_execute_gripper:=true \
    2>&1 | tee "${RUN_DIR}/console.log"

EXIT_CODE=${PIPESTATUS[0]}

set -e

cat > "${RUN_DIR}/run_status.json" <<EOF
{
    "timestamp": "${TIMESTAMP}",
    "task": "${TASK}",
    "trajectory": "${TRAJECTORY}",
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