#!/usr/bin/env bash

set -o pipefail

source /home/ros2_ws/install/setup.bash

export PYTHONPATH="${PYTHONPATH:-}:/opt/seedo-venv/lib/python3.12/site-packages"

REPO_ROOT="/home/ros2_ws/src/UR5e-2f-85"

TIMESTAMP="$(date '+%Y%m%d_%H%M%S_%3N')"

REAL_TEST_ROOT="/test_real"

# ----------------------------------------------------------------------
# Select task ID and automatically determine the next test number.
#
# Examples:
#   first run of task 00  -> test_task00_1
#   second run of task 00 -> test_task00_2
#   first run of task 03  -> test_task03_1
# ----------------------------------------------------------------------

read -rp "Enter task ID [00-15]: " TASK_ID_INPUT

if ! [[ "${TASK_ID_INPUT}" =~ ^[0-9]+$ ]]; then
    echo "ERROR: task ID must be numeric."
    exit 1
fi

TASK_ID="$(printf '%02d' "$((10#${TASK_ID_INPUT}))")"

DEMO_PATH="/test_dataset/pick_place/human_rgb_pick_place/task_${TASK_ID}/traj000/converted/traj000-h264-30fps_modified.mp4"

if [[ ! -f "${DEMO_PATH}" ]]; then
    echo "ERROR: demonstration video not found:"
    echo "${DEMO_PATH}"
    exit 1
fi

TEST_INDEX=1

while [[ -e "${REAL_TEST_ROOT}/test_task${TASK_ID}_${TEST_INDEX}" ]]; do
    ((TEST_INDEX++))
done

TEST_NAME="test_task${TASK_ID}_${TEST_INDEX}"
RUN_DIR="${REAL_TEST_ROOT}/${TEST_NAME}"

ACTION_PLAN_SOURCE="/seedo_tests/ai_controller_node_interactive/action_planning"

CONFIG_PATH="${REPO_ROOT}/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml"

PRECOMPUTED_PLAN="${ACTION_PLAN_SOURCE}/action_plan.json"

mkdir -p "${RUN_DIR}"
mkdir -p "${RUN_DIR}/action_planning"
mkdir -p "${RUN_DIR}/rollouts"

echo "================================================="
echo "SeeDo robot test"
echo "Task ID: ${TASK_ID}"
echo "Test number: ${TEST_INDEX}"
echo "Test name: ${TEST_NAME}"
echo "Run directory: ${RUN_DIR}"
echo "Demo video: ${DEMO_PATH}"
echo "================================================="

# ----------------------------------------------------------------------
# Save the Action Planning artifacts used as input.
# In precomputed mode these are not regenerated, so preserve the exact
# artifacts that produced the plan used in this execution.
# ----------------------------------------------------------------------

cp -a \
    "${ACTION_PLAN_SOURCE}/." \
    "${RUN_DIR}/action_planning/"

# ----------------------------------------------------------------------
# Save the exact SeeDo configuration used for this experiment.
# ----------------------------------------------------------------------

cp \
    "${CONFIG_PATH}" \
    "${RUN_DIR}/seedo_controller.yaml"

set +e

# ros2 run ai_controller ai_controller_node \
#     --ros-args \
#     -p ai_controller_target:=seedo_controller \
#     -p model_config_path:="${CONFIG_PATH}" \
#     -p task_name:=pick_place \
#     -p demo_path:="${DEMO_PATH}" \
#     -p seedo_precomputed_action_plan_path:=/test_real/test_solo_verde/test_task00_1/action_planning/action_plan.json \
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
    -p demo_path:="${DEMO_PATH}" \
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
    "test_name": "${TEST_NAME}",
    "task_id": "${TASK_ID}",
    "test_index": ${TEST_INDEX},
    "demo_path": "${DEMO_PATH}",
    "exit_code": ${EXIT_CODE}
}
EOF

echo
echo "================================================="
echo "Test finished with exit code: ${EXIT_CODE}"
echo "Artifacts saved in:"
echo "${RUN_DIR}"
echo "================================================="

exit "${EXIT_CODE}"