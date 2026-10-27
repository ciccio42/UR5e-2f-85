#!/usr/bin/env bash

set -o pipefail


# =============================================================================
# SCRIPT MARKERS
# =============================================================================

SCRIPT_NAME="RUN SEEDO ISAAC TEST"


print_start_marker() {
    echo
    echo "===================================================================================================="
    echo "START: ${SCRIPT_NAME}"
    echo "===================================================================================================="
    echo
}


print_end_marker() {
    local exit_code=$?

    echo
    echo "===================================================================================================="
    echo "END: ${SCRIPT_NAME}"
    echo "Exit code: ${exit_code}"
    echo "===================================================================================================="
    echo
}


fail() {
    echo
    echo "ERROR: $*" >&2
    exit 1
}


trap print_end_marker EXIT

print_start_marker


# =============================================================================
# ROS / PYTHON ENVIRONMENT
# =============================================================================

ROS_SETUP="/home/ros2_ws/install/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
    fail "ROS setup not found: ${ROS_SETUP}"
fi

source "${ROS_SETUP}"

export PYTHONPATH="${PYTHONPATH:-}:/opt/seedo-venv/lib/python3.12/site-packages"


# =============================================================================
# PATHS
# =============================================================================

REPO_ROOT="/home/ros2_ws/src/UR5e-2f-85"

SCRIPT_DIR="${REPO_ROOT}/ai_controller/ai_controller/models/seedo_controller"

SCENE_CONFIGURATOR="${SCRIPT_DIR}/configure_isaac_scene.py"

CONFIG_ROOT="${SCRIPT_DIR}/config"
DEFAULT_CONFIG_PATH="${CONFIG_ROOT}/seedo_controller.yaml"

REAL_TEST_ROOT="/test_isaac"


if [[ ! -f "${SCENE_CONFIGURATOR}" ]]; then
    fail "Scene configurator not found: ${SCENE_CONFIGURATOR}"
fi


# =============================================================================
# SUPPORTED ENVIRONMENTS
# =============================================================================

ENV_PICK_PLACE="pick_place"
ENV_NUT_ASSEMBLY="nut_assembly"

ENV_TYPE=""

CONFIGURE_SCENE=""

SCENE_TASK=""
SCENE_TRAJECTORY=""

DEMO_TASK=""
DEMO_PATH=""

CONFIG_PATH=""

PRECOMPUTED_PLAN=""
PRECOMPUTED_DEMO_SCENE=""
PRECOMPUTED_MODE=false


# =============================================================================
# CAMERA TOPICS
# =============================================================================

CAMERA_TOPICS="['/zed_front/zed_node/rgb/color/rect/image','/zed_left/zed_node/rgb/color/rect/image','/zed_right/zed_node/rgb/color/rect/image','/zed_gripper/zed_node/rgb/color/rect/image']"

DEPTH_TOPICS="['/zed_front/zed_node/depth/depth_registered','/zed_left/zed_node/depth/depth_registered','/zed_right/zed_node/depth/depth_registered','/zed_gripper/zed_node/depth/depth_registered']"


# =============================================================================
# HELP
# =============================================================================

print_usage() {
    cat <<EOF

Usage:

  $0 [options]

Environment:

  --env-type pick_place|nut_assembly

Scene configuration:

  --configure-scene
  --skip-scene-config

  --scene-task N
  --scene-trajectory N

Backward-compatible aliases:

  --task N
  --trajectory N

Demonstration:

  --demo-task N
  --demo-path PATH

Configuration:

  --config PATH

  Default:
    ${DEFAULT_CONFIG_PATH}

Optional precomputed demonstration artifacts:

  --precomputed-plan PATH
  --precomputed-demo-scene PATH

Notes:

  - If --config is omitted, the Solution config is used.
  - Supplying --precomputed-plan enables precomputed mode.
  - --precomputed-demo-scene is required only when the selected config uses
    perception_mode: generalized.

Examples:

  Pick-and-Place:

    $0 \
      --env-type pick_place \
      --scene-task 0 \
      --scene-trajectory 3 \
      --demo-task 0

  Nut Assembly:

    $0 \
      --env-type nut_assembly \
      --scene-task 4 \
      --demo-task 4 \
      --demo-path /path/to/nut/video.mp4

  Reuse current Isaac scene:

    $0 \
      --env-type nut_assembly \
      --skip-scene-config \
      --demo-task 4 \
      --demo-path /path/to/nut/video.mp4

EOF
}


# =============================================================================
# ARGUMENT PARSING
# =============================================================================

while [[ $# -gt 0 ]]; do

    case "$1" in

        --env-type)

            if [[ $# -lt 2 ]]; then
                fail "--env-type requires a value."
            fi

            ENV_TYPE="$2"

            shift 2
            ;;


        --configure-scene)

            CONFIGURE_SCENE=true

            shift
            ;;


        --skip-scene-config)

            CONFIGURE_SCENE=false

            shift
            ;;


        --scene-task|--task)

            if [[ $# -lt 2 ]]; then
                fail "$1 requires a value."
            fi

            SCENE_TASK="$2"

            shift 2
            ;;


        --scene-trajectory|--trajectory)

            if [[ $# -lt 2 ]]; then
                fail "$1 requires a value."
            fi

            SCENE_TRAJECTORY="$2"

            shift 2
            ;;


        --demo-task)

            if [[ $# -lt 2 ]]; then
                fail "--demo-task requires a value."
            fi

            DEMO_TASK="$2"

            shift 2
            ;;


        --demo-path)

            if [[ $# -lt 2 ]]; then
                fail "--demo-path requires a value."
            fi

            DEMO_PATH="$2"

            shift 2
            ;;


        --config)

            if [[ $# -lt 2 ]]; then
                fail "--config requires a value."
            fi

            CONFIG_PATH="$2"

            shift 2
            ;;


        --precomputed-plan)

            if [[ $# -lt 2 ]]; then
                fail "--precomputed-plan requires a value."
            fi

            PRECOMPUTED_PLAN="$2"

            shift 2
            ;;

        --precomputed-demo-scene)

            if [[ $# -lt 2 ]]; then
                fail "--precomputed-demo-scene requires a value."
            fi

            PRECOMPUTED_DEMO_SCENE="$2"

            shift 2
            ;;


        -h|--help)

            print_usage
            exit 0
            ;;


        *)

            echo "Unknown argument: $1" >&2

            print_usage

            exit 1
            ;;

    esac

done


# =============================================================================
# GENERIC INPUT UTILITIES
# =============================================================================

normalize_env_type() {

    case "$1" in

        pick_place|pick|p|P)
            echo "${ENV_PICK_PLACE}"
            ;;

        nut_assembly|nut|n|N)
            echo "${ENV_NUT_ASSEMBLY}"
            ;;

        *)
            return 1
            ;;

    esac
}


validate_unsigned_integer() {

    local value="$1"

    [[ "${value}" =~ ^[0-9]+$ ]]
}


validate_range() {

    local value="$1"
    local minimum="$2"
    local maximum="$3"
    local label="$4"

    if ! validate_unsigned_integer "${value}"; then
        fail "${label} must be numeric."
    fi

    local numeric_value=$((10#${value}))

    if (( numeric_value < minimum || numeric_value > maximum )); then

        fail "${label} must be between ${minimum} and ${maximum}."

    fi
}


read_perception_mode() {

    local config_path="$1"
    local value

    value="$(
        awk '
            /^[[:space:]]*perception_mode[[:space:]]*:/ {
                line = $0
                sub(/^[^:]*:[[:space:]]*/, "", line)
                sub(/[[:space:]]*#.*/, "", line)
                gsub(/[[:space:]\047\042]/, "", line)
                print tolower(line)
                exit
            }
        ' "${config_path}"
    )"

    case "${value}" in
        generalized|prior_guided)
            printf '%s\n' "${value}"
            ;;
        *)
            return 1
            ;;
    esac
}


# =============================================================================
# CONFIGURATION
# =============================================================================

if [[ -z "${CONFIG_PATH}" ]]; then
    CONFIG_PATH="${DEFAULT_CONFIG_PATH}"
fi

if [[ ! -f "${CONFIG_PATH}" ]]; then
    fail "SeeDo configuration not found: ${CONFIG_PATH}"
fi

PERCEPTION_MODE="$(read_perception_mode "${CONFIG_PATH}")" \
    || fail "Could not read a supported top-level perception_mode from: ${CONFIG_PATH}"

echo
echo "[PASS] SeeDo configuration found:"
echo "       ${CONFIG_PATH}"
echo "       perception_mode=${PERCEPTION_MODE}"
echo


# =============================================================================
# SELECT ENVIRONMENT
# =============================================================================

if [[ -z "${ENV_TYPE}" ]]; then

    echo "Select SeeDo environment:"
    echo
    echo "  1) pick_place"
    echo "  2) nut_assembly"
    echo

    read -rp "Environment [1/2]: " ENV_REPLY

    case "${ENV_REPLY}" in

        1)
            ENV_TYPE="${ENV_PICK_PLACE}"
            ;;

        2)
            ENV_TYPE="${ENV_NUT_ASSEMBLY}"
            ;;

        *)

            NORMALIZED_ENV="$(
                normalize_env_type "${ENV_REPLY}"
            )" || fail "Invalid environment '${ENV_REPLY}'."

            ENV_TYPE="${NORMALIZED_ENV}"
            ;;

    esac

else

    NORMALIZED_ENV="$(
        normalize_env_type "${ENV_TYPE}"
    )" || fail "Invalid environment '${ENV_TYPE}'."

    ENV_TYPE="${NORMALIZED_ENV}"

fi


echo
echo "Selected environment: ${ENV_TYPE}"
echo


# =============================================================================
# DETERMINE WHETHER ISAAC SCENE MUST BE CONFIGURED
# =============================================================================

if [[ -n "${SCENE_TASK}" || -n "${SCENE_TRAJECTORY}" ]]; then

    if [[ "${CONFIGURE_SCENE}" == "false" ]]; then
        fail \
            "Scene task/trajectory was provided together with " \
            "--skip-scene-config."
    fi

    CONFIGURE_SCENE=true

fi


if [[ -z "${CONFIGURE_SCENE}" ]]; then

    read -rp \
        "Do you want to configure the Isaac scene? [y/n]: " \
        CONFIGURE_REPLY

    case "${CONFIGURE_REPLY}" in

        y|Y|yes|YES|Yes|s|S|si|SI|Si)
            CONFIGURE_SCENE=true
            ;;

        *)
            CONFIGURE_SCENE=false
            ;;

    esac

fi


# =============================================================================
# ISAAC SCENE CONFIGURATION
# =============================================================================

if [[ "${CONFIGURE_SCENE}" == true ]]; then

    echo
    echo "----------------------------------------------------------------------------------------------------"
    echo "ISAAC SCENE CONFIGURATION"
    echo "----------------------------------------------------------------------------------------------------"
    echo

    # -------------------------------------------------------------------------
    # Scene task
    # -------------------------------------------------------------------------

    if [[ -z "${SCENE_TASK}" ]]; then

        if [[ "${ENV_TYPE}" == "${ENV_PICK_PLACE}" ]]; then

            read -rp \
                "Isaac scene task [00-15]: " \
                SCENE_TASK

        else

            read -rp \
                "Isaac scene task [00-08]: " \
                SCENE_TASK

        fi

    fi


    # -------------------------------------------------------------------------
    # Scene trajectory
    # -------------------------------------------------------------------------

    if [[ -z "${SCENE_TRAJECTORY}" ]]; then

        if [[ "${ENV_TYPE}" == "${ENV_PICK_PLACE}" ]]; then

            read -rp \
                "Isaac scene trajectory [000-039]: " \
                SCENE_TRAJECTORY

        else

            SCENE_TRAJECTORY="0"

            echo \
                "[INFO] Nut Assembly currently has only trajectory 000."

        fi

    fi


    # -------------------------------------------------------------------------
    # Environment-specific validation
    # -------------------------------------------------------------------------

    if [[ "${ENV_TYPE}" == "${ENV_PICK_PLACE}" ]]; then

        validate_range \
            "${SCENE_TASK}" \
            0 \
            15 \
            "Pick-and-Place scene task"

        validate_range \
            "${SCENE_TRAJECTORY}" \
            0 \
            39 \
            "Pick-and-Place scene trajectory"

    else

        validate_range \
            "${SCENE_TASK}" \
            0 \
            8 \
            "Nut-Assembly scene task"

        validate_range \
            "${SCENE_TRAJECTORY}" \
            0 \
            0 \
            "Nut-Assembly scene trajectory"

    fi


    SCENE_TASK_NUM="$((10#${SCENE_TASK}))"
    SCENE_TRAJECTORY_NUM="$((10#${SCENE_TRAJECTORY}))"

    printf -v SCENE_TASK "%02d" \
        "${SCENE_TASK_NUM}"

    printf -v SCENE_TRAJECTORY "%03d" \
        "${SCENE_TRAJECTORY_NUM}"


    echo
    echo "================================================="
    echo "Configuring Isaac scene"
    echo "================================================="
    echo "Environment: ${ENV_TYPE}"
    echo "Task:        ${SCENE_TASK}"
    echo "Trajectory:  ${SCENE_TRAJECTORY}"
    echo "================================================="
    echo


    python3 "${SCENE_CONFIGURATOR}" \
        --env-type "${ENV_TYPE}" \
        --task "${SCENE_TASK_NUM}" \
        --trajectory "${SCENE_TRAJECTORY_NUM}"

    SCENE_CONFIG_EXIT_CODE=$?


    if [[ ${SCENE_CONFIG_EXIT_CODE} -ne 0 ]]; then

        fail \
            "Isaac scene configuration failed. " \
            "SeeDo test will NOT be started."

    fi


    echo
    echo "[PASS] Isaac scene configured successfully."
    echo

else

    echo
    echo "================================================="
    echo "Isaac scene configuration SKIPPED"
    echo "Environment: ${ENV_TYPE}"
    echo "Using the current Isaac scene."
    echo "================================================="
    echo

fi


# =============================================================================
# SELECT DEMONSTRATION TASK
#
# Deliberately independent from the Isaac scene task.
#
# This allows:
#
#   - same demonstration on different simulated layouts;
#   - generalization experiments;
#   - explicit mismatch tests when desired.
# =============================================================================

if [[ -z "${DEMO_TASK}" ]]; then

    if [[ "${ENV_TYPE}" == "${ENV_PICK_PLACE}" ]]; then

        read -rp \
            "Enter Pick-and-Place demonstration task ID [00-15]: " \
            DEMO_TASK

    else

        read -rp \
            "Enter Nut-Assembly demonstration task ID [00-08]: " \
            DEMO_TASK

    fi

fi


if [[ "${ENV_TYPE}" == "${ENV_PICK_PLACE}" ]]; then

    validate_range \
        "${DEMO_TASK}" \
        0 \
        15 \
        "Pick-and-Place demonstration task"

else

    validate_range \
        "${DEMO_TASK}" \
        0 \
        8 \
        "Nut-Assembly demonstration task"

fi


DEMO_TASK_NUM="$((10#${DEMO_TASK}))"

printf -v DEMO_TASK_ID "%02d" \
    "${DEMO_TASK_NUM}"


# =============================================================================
# DEMONSTRATION VIDEO
# =============================================================================

if [[ -z "${DEMO_PATH}" ]]; then

    if [[ "${ENV_TYPE}" == "${ENV_PICK_PLACE}" ]]; then

        DEMO_PATH="/test_dataset/pick_place/human_rgb_pick_place/task_${DEMO_TASK_ID}/traj000/converted/traj000-h264-30fps.mp4"

    else

        DEMO_PATH="/dataset_nut/task_${DEMO_TASK_ID}/traj_000_camera_front.mp4"

    fi

fi


if [[ -z "${DEMO_PATH}" ]]; then
    fail "Demonstration video path is empty."
fi


if [[ ! -f "${DEMO_PATH}" ]]; then

    echo
    echo "Demonstration video not found:"
    echo
    echo "  ${DEMO_PATH}"
    echo

    fail "Invalid demonstration video."

fi


echo
echo "[PASS] Demonstration video found:"
echo "       ${DEMO_PATH}"
echo


# =============================================================================
# OPTIONAL PRECOMPUTED DEMONSTRATION ARTIFACTS
# =============================================================================

if [[ -n "${PRECOMPUTED_DEMO_SCENE}" && -z "${PRECOMPUTED_PLAN}" ]]; then

    fail "--precomputed-demo-scene requires --precomputed-plan."

fi


if [[ -n "${PRECOMPUTED_PLAN}" ]]; then

    PRECOMPUTED_MODE=true

    if [[ ! -f "${PRECOMPUTED_PLAN}" ]]; then
        fail \
            "Precomputed action plan not found: ${PRECOMPUTED_PLAN}"
    fi

    if [[ "${PERCEPTION_MODE}" == "generalized" ]]; then

        if [[ -z "${PRECOMPUTED_DEMO_SCENE}" ]]; then
            fail \
                "Generalized precomputed execution requires " \
                "--precomputed-demo-scene."
        fi

    fi

    if [[ -n "${PRECOMPUTED_DEMO_SCENE}" && ! -f "${PRECOMPUTED_DEMO_SCENE}" ]]; then
        fail \
            "Precomputed demo structured scene not found: ${PRECOMPUTED_DEMO_SCENE}"
    fi

    echo
    echo "[INFO] Precomputed demonstration mode enabled."
    echo "[INFO] Action plan:"
    echo "       ${PRECOMPUTED_PLAN}"

    if [[ -n "${PRECOMPUTED_DEMO_SCENE}" ]]; then
        echo "[INFO] Demo structured scene:"
        echo "       ${PRECOMPUTED_DEMO_SCENE}"
    else
        echo "[INFO] Demo structured scene: not required (${PERCEPTION_MODE})."
    fi

    echo

else

    PRECOMPUTED_MODE=false

    echo
    echo "[INFO] Demonstration pipeline will be executed normally."
    echo

fi


# =============================================================================
# AUTOMATIC TEST NUMBER
# =============================================================================

TEST_INDEX=1

TEST_PREFIX="test_${ENV_TYPE}_task${DEMO_TASK_ID}"


while [[ -e "${REAL_TEST_ROOT}/${TEST_PREFIX}_${TEST_INDEX}" ]]; do

    ((TEST_INDEX++))

done


TEST_NAME="${TEST_PREFIX}_${TEST_INDEX}"

RUN_DIR="${REAL_TEST_ROOT}/${TEST_NAME}"


TIMESTAMP="$(
    date '+%Y%m%d_%H%M%S_%3N'
)"


mkdir -p "${RUN_DIR}"
mkdir -p "${RUN_DIR}/action_planning"
mkdir -p "${RUN_DIR}/demo_structured_scene"
mkdir -p "${RUN_DIR}/rollouts"

if [[ "${SEEDO_GUI_TELEMETRY:-}" == "1" ]]; then

    printf \
        '__SEEDO_PIPELINE_EVENT__{"event":"artifacts_root","path":"%s"}\n' \
        "${RUN_DIR}"

fi

# =============================================================================
# TEST SUMMARY
# =============================================================================

echo
echo "===================================================================================================="
echo "SEEDO ISAAC SIM TEST"
echo "===================================================================================================="

echo "Environment:       ${ENV_TYPE}"
echo "Demo task ID:      ${DEMO_TASK_ID}"
echo "Test number:       ${TEST_INDEX}"
echo "Test name:         ${TEST_NAME}"
echo "Run directory:     ${RUN_DIR}"
echo "Demo video:        ${DEMO_PATH}"
echo "Configuration:     ${CONFIG_PATH}"
echo "Perception mode:   ${PERCEPTION_MODE}"

if [[ "${CONFIGURE_SCENE}" == true ]]; then

    echo "Isaac scene task:  ${SCENE_TASK}"
    echo "Isaac trajectory:  ${SCENE_TRAJECTORY}"

else

    echo "Isaac scene:       current scene"

fi


if [[ "${PRECOMPUTED_MODE}" == true ]]; then

    echo "Precomputed mode:  enabled"
    echo "Action plan:       ${PRECOMPUTED_PLAN}"

    if [[ -n "${PRECOMPUTED_DEMO_SCENE}" ]]; then
        echo "Demo scene:        ${PRECOMPUTED_DEMO_SCENE}"
    else
        echo "Demo scene:        not required"
    fi

else

    echo "Precomputed mode:  disabled"
    echo "Demo pipeline:     generated at runtime"

fi


echo "===================================================================================================="
echo


# =============================================================================
# SAVE PRE-RUN ARTIFACTS
# =============================================================================

# Save only the exact precomputed artifacts explicitly selected for this run.
# Full runs generate their action-planning artifacts directly in RUN_DIR.
if [[ "${PRECOMPUTED_MODE}" == true ]]; then

    cp \
        "${PRECOMPUTED_PLAN}" \
        "${RUN_DIR}/action_planning/action_plan.json"

    echo "[PASS] Precomputed action plan copied into run artifacts."

    if [[ -n "${PRECOMPUTED_DEMO_SCENE}" ]]; then

        cp \
            "${PRECOMPUTED_DEMO_SCENE}" \
            "${RUN_DIR}/demo_structured_scene/demo_structured_scene.json"

        echo "[PASS] Precomputed demo structured scene copied into run artifacts."

    fi

fi


cp \
    "${CONFIG_PATH}" \
    "${RUN_DIR}/seedo_controller.yaml"


# Save the launcher used for the experiment when possible.

if [[ -f "$0" ]]; then

    cp \
        "$0" \
        "${RUN_DIR}/run_seedo_isaac_test.sh"

fi


# =============================================================================
# BUILD ROS PARAMETERS
# =============================================================================

ROS_ARGS=(

    --ros-args

    -p
    "ai_controller_target:=seedo_controller"

    -p
    "model_config_path:=${CONFIG_PATH}"

    -p
    "task_name:=${ENV_TYPE}"

    -p
    "demo_path:=${DEMO_PATH}"

    -p
    "seedo_artifacts_dir:=${RUN_DIR}"

    -p
    "save_rollout_path:=${RUN_DIR}/rollouts"

    -p
    "move_robot:=true"

    -p
    "seedo_execute_gripper:=true"

    -p
    "camera_topic:=${CAMERA_TOPICS}"

    -p
    "seedo_record_depth_topics:=${DEPTH_TOPICS}"
)


if [[ "${PRECOMPUTED_MODE}" == true ]]; then

    ROS_ARGS+=(
        -p
        "seedo_precomputed_action_plan_path:=${PRECOMPUTED_PLAN}"
    )

    if [[ -n "${PRECOMPUTED_DEMO_SCENE}" ]]; then

        ROS_ARGS+=(
            -p
            "seedo_precomputed_demo_structured_scene_path:=${PRECOMPUTED_DEMO_SCENE}"
        )

    fi

fi


# =============================================================================
# RUN SEEDO
# =============================================================================

echo
echo "----------------------------------------------------------------------------------------------------"
echo "STARTING SEEDO CONTROLLER"
echo "----------------------------------------------------------------------------------------------------"
echo

echo "task_name:       ${ENV_TYPE}"
echo "demo_path:       ${DEMO_PATH}"
echo "model_config:    ${CONFIG_PATH}"
echo "perception_mode: ${PERCEPTION_MODE}"
echo

echo "RGB cameras:"
echo "  /zed_front/zed_node/rgb/color/rect/image"
echo "  /zed_left/zed_node/rgb/color/rect/image"
echo "  /zed_right/zed_node/rgb/color/rect/image"
echo "  /zed_gripper/zed_node/rgb/color/rect/image"

echo

echo "Depth cameras:"
echo "  /zed_front/zed_node/depth/depth_registered"
echo "  /zed_left/zed_node/depth/depth_registered"
echo "  /zed_right/zed_node/depth/depth_registered"
echo "  /zed_gripper/zed_node/depth/depth_registered"

echo


set +e


ros2 run \
    ai_controller \
    ai_controller_node \
    "${ROS_ARGS[@]}" \
    2>&1 | tee "${RUN_DIR}/console.log"


EXIT_CODE=${PIPESTATUS[0]}


set -e


# =============================================================================
# SAVE RUN STATUS
# =============================================================================

cat > "${RUN_DIR}/run_status.json" <<EOF
{
    "timestamp": "${TIMESTAMP}",
    "test_name": "${TEST_NAME}",
    "env_type": "${ENV_TYPE}",
    "demo_task_id": "${DEMO_TASK_ID}",
    "demo_path": "${DEMO_PATH}",
    "config_path": "${CONFIG_PATH}",
    "perception_mode": "${PERCEPTION_MODE}",
    "isaac_scene_configured": ${CONFIGURE_SCENE},
    "isaac_scene_task": "${SCENE_TASK}",
    "isaac_scene_trajectory": "${SCENE_TRAJECTORY}",
    "precomputed_mode": ${PRECOMPUTED_MODE},
    "precomputed_action_plan": "${PRECOMPUTED_PLAN}",
    "precomputed_demo_structured_scene": "${PRECOMPUTED_DEMO_SCENE}",
    "exit_code": ${EXIT_CODE},
    "mode": "isaac_sim"
}
EOF


# =============================================================================
# FINAL SUMMARY
# =============================================================================

echo
echo "===================================================================================================="
echo "SEEDO TEST FINISHED"
echo "===================================================================================================="

echo "Environment: ${ENV_TYPE}"
echo "Exit code:   ${EXIT_CODE}"
echo "Artifacts:"
echo "  ${RUN_DIR}"

echo "===================================================================================================="
echo


exit "${EXIT_CODE}"