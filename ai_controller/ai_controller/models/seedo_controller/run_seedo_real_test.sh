#!/usr/bin/env bash

set -o pipefail

source /home/ros2_ws/install/setup.bash

export PYTHONPATH="${PYTHONPATH:-}:/opt/seedo-venv/lib/python3.12/site-packages"

REPO_ROOT="/home/ros2_ws/src/UR5e-2f-85"

TIMESTAMP="$(date '+%Y%m%d_%H%M%S_%3N')"

REAL_TEST_ROOT="/test_real"
RUN_DIR="${REAL_TEST_ROOT}/${TIMESTAMP}"

ACTION_PLAN_SOURCE="/seedo_tests/ai_controller_node_interactive/action_planning"

CONFIG_PATH="${REPO_ROOT}/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml"

PRECOMPUTED_PLAN="${ACTION_PLAN_SOURCE}/action_plan.json"

mkdir -p "${RUN_DIR}"
mkdir -p "${RUN_DIR}/action_planning"
mkdir -p "${RUN_DIR}/rollouts"

echo "================================================="
echo "SeeDo real robot test"
echo "Run directory: ${RUN_DIR}"
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

# ----------------------------------------------------------------------
# Save repository state for reproducibility.
# ----------------------------------------------------------------------

git -C "${REPO_ROOT}" rev-parse HEAD \
    > "${RUN_DIR}/git_commit.txt"

git -C "${REPO_ROOT}" diff \
    > "${RUN_DIR}/local_changes.patch"

# ----------------------------------------------------------------------
# Execute SeeDo.
#
# seedo_artifacts_dir:
#   scene_perceiver/
#   scene_interpreter/
#   lmp_generator/
#   motion_layer/
#
# save_rollout_path:
#   executed trajectory .pkl + result .json
#
# stdout/stderr:
#   console.log
# ----------------------------------------------------------------------

set +e

ros2 run ai_controller ai_controller_node \
    --ros-args \
    -p ai_controller_target:=seedo_controller \
    -p model_config_path:="${CONFIG_PATH}" \
    -p task_name:=pick_place \
    -p demo_path:=/test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps_modified.mp4 \
    -p seedo_precomputed_action_plan_path:="${PRECOMPUTED_PLAN}" \
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