#!/usr/bin/env bash

set -Eeuo pipefail


# =============================================================================
# VLA-JEPA inference server launcher
#
# Uses the persistent VLA-JEPA venv mounted inside the ROS container.
#
# Normal execution:
#
#   ./run_server.sh
#
# Full preflight + server:
#
#   ./run_server.sh --preflight
#
# Full preflight only:
#
#   ./run_server.sh --preflight-only
#
# Different config:
#
#   ./run_server.sh --config vla_jepa_config.yaml
#
# Cosmos task description:
#
#   ./run_server.sh --use-cosmos-task-description
#
# The preflight:
#   - does NOT contact the ROS graph;
#   - does NOT contact MoveIt;
#   - does NOT require ZED containers;
#   - cannot move the robot;
#   - does NOT require server.py to already be running;
#   - loads the real VLA-JEPA checkpoint;
#   - executes one real dummy inference locally.
# =============================================================================


THIS_DIR="$(
    cd "$(dirname "${BASH_SOURCE[0]}")"
    pwd
)"


# =============================================================================
# Runtime paths
# =============================================================================

VENV_DIR="${VLA_JEPA_VENV_DIR:-${THIS_DIR}/venv/vla_jepa_venv}"
PYTHON="${VENV_DIR}/bin/python"

VLA_JEPA_CHECKPOINT="${VLA_JEPA_CHECKPOINT:-/opt/vla_jepa/checkpoint}"
VLA_JEPA_HF_HOME="${VLA_JEPA_HF_HOME:-/opt/vla_jepa/huggingface}"

export VLA_JEPA_CHECKPOINT
export VLA_JEPA_HF_HOME


# =============================================================================
# Hugging Face cache
#
# The old dedicated VLA-JEPA image used:
#
#   HF_HOME=/workspace/checkpoints/huggingface
#
# In the new shared ROS container the same cache is mounted at:
#
#   /opt/vla_jepa/huggingface
#
# HF_HUB_CACHE and HUGGINGFACE_HUB_CACHE are made explicit so all
# Hugging Face / Transformers / LeRobot code resolves the same cache.
# =============================================================================

export HF_HOME="${VLA_JEPA_HF_HOME}"
export HF_HUB_CACHE="${HF_HOME}/hub"
export HUGGINGFACE_HUB_CACHE="${HF_HOME}/hub"


# =============================================================================
# Defaults / arguments
# =============================================================================

DEFAULT_CONFIG_NAME="${VLA_JEPA_CONFIG_NAME:-vla_jepa_config.yaml}"

CONFIG_VALUE="${DEFAULT_CONFIG_NAME}"

RUN_PREFLIGHT=0
PREFLIGHT_ONLY=0
USE_COSMOS=0

TASK_NAME="pick_place"
HOST_VALUE=""
PORT_VALUE=""


# =============================================================================
# Helpers
# =============================================================================

die()
{
    echo
    echo "ERROR: $*" >&2
    echo
    exit 1
}


usage()
{
    cat <<EOF
Usage:

  $0 [options]

Options:

  --config FILE
        Runtime YAML.
        Relative paths are resolved from:
          ${THIS_DIR}

  --preflight
        Run the complete VLA-JEPA dummy-inference preflight
        before starting the Flask server.

  --preflight-only
        Run the complete preflight and exit without starting Flask.

  --task-name NAME
        Task family passed to server.py.
        Default: pick_place

  --host HOST
        Override server host from YAML.

  --port PORT
        Override server port from YAML.

  --use-cosmos-task-description
        Preload Cosmos through server.py.

  -h, --help
        Show this message.
EOF
}


# =============================================================================
# Arguments
# =============================================================================

while [[ $# -gt 0 ]]; do

    case "$1" in

        --config)
            [[ $# -ge 2 ]] || die "--config requires a value."
            CONFIG_VALUE="$2"
            shift 2
            ;;

        --preflight)
            RUN_PREFLIGHT=1
            shift
            ;;

        --preflight-only)
            RUN_PREFLIGHT=1
            PREFLIGHT_ONLY=1
            shift
            ;;

        --task-name)
            [[ $# -ge 2 ]] || die "--task-name requires a value."
            TASK_NAME="$2"
            shift 2
            ;;

        --host)
            [[ $# -ge 2 ]] || die "--host requires a value."
            HOST_VALUE="$2"
            shift 2
            ;;

        --port)
            [[ $# -ge 2 ]] || die "--port requires a value."
            PORT_VALUE="$2"
            shift 2
            ;;

        --use-cosmos-task-description)
            USE_COSMOS=1
            shift
            ;;

        -h|--help)
            usage
            exit 0
            ;;

        *)
            die "Unknown argument: $1"
            ;;

    esac

done


# =============================================================================
# Venv
# =============================================================================

if [[ ! -x "${PYTHON}" ]]; then

    cat >&2 <<EOF

ERROR: VLA-JEPA venv does not exist:

    ${VENV_DIR}

Expected Python:

    ${PYTHON}

Make sure the venv bind mount is present in the container.

EOF

    exit 1
fi


# =============================================================================
# Config
# =============================================================================

if [[ "${CONFIG_VALUE}" = /* ]]; then
    CONFIG_PATH="${CONFIG_VALUE}"
else
    CONFIG_PATH="${THIS_DIR}/${CONFIG_VALUE}"
fi

[[ -f "${CONFIG_PATH}" ]] || \
    die "Config not found: ${CONFIG_PATH}"

CONFIG_PATH="$(
    realpath "${CONFIG_PATH}"
)"

VLA_JEPA_CONTROLLER_CONFIG="${CONFIG_PATH}"
export VLA_JEPA_CONTROLLER_CONFIG


# =============================================================================
# Required source files
# =============================================================================

for required_file in \
    "${THIS_DIR}/server.py" \
    "${THIS_DIR}/test_setup.py" \
    "${THIS_DIR}/vla_jepa_controller.py" \
    "${THIS_DIR}/vla_jepa.py" \
    "${THIS_DIR}/vla_jepa_utils.py"; do

    [[ -f "${required_file}" ]] || \
        die "Required source file not found: ${required_file}"

done


# =============================================================================
# Checkpoint
# =============================================================================

[[ -d "${VLA_JEPA_CHECKPOINT}" ]] || \
    die "Checkpoint directory not found: ${VLA_JEPA_CHECKPOINT}"

for checkpoint_file in \
    config.json \
    model.safetensors \
    policy_preprocessor.json \
    policy_postprocessor.json \
    policy_postprocessor_ur5e.json \
    policy_preprocessor_step_3_normalizer_processor.safetensors \
    policy_postprocessor_step_2_unnormalizer_processor.safetensors; do

    [[ -f "${VLA_JEPA_CHECKPOINT}/${checkpoint_file}" ]] || \
        die "Checkpoint file not found: ${VLA_JEPA_CHECKPOINT}/${checkpoint_file}"

done


# =============================================================================
# Hugging Face cache
# =============================================================================

[[ -d "${HF_HOME}" ]] || \
    die "Hugging Face cache not mounted: ${HF_HOME}"

[[ -d "${HF_HUB_CACHE}" ]] || \
    die "Hugging Face hub cache not found: ${HF_HUB_CACHE}"


# =============================================================================
# LeRobot source
# =============================================================================

LEROBOT_ROOT="${THIS_DIR}/external/lerobot"

[[ -d "${LEROBOT_ROOT}/src/lerobot" ]] || \
    die "LeRobot source tree not found: ${LEROBOT_ROOT}"


# =============================================================================
# Python path
#
# Layout:
#
#   ai_controller/
#   └── ai_controller/
#       └── models/
#           └── vla_jepa_controller/
#
# ../../.. = outer ROS ai_controller package root.
# =============================================================================

AI_CONTROLLER_ROOT="$(
    cd "${THIS_DIR}/../../.."
    pwd
)"

export PYTHONPATH="${AI_CONTROLLER_ROOT}:${THIS_DIR}${PYTHONPATH:+:${PYTHONPATH}}"


# =============================================================================
# Runtime environment
# =============================================================================

export PYTHONUNBUFFERED=1
export PYTHONDONTWRITEBYTECODE=1
export TOKENIZERS_PARALLELISM=false

# Do not create a pip cache during runtime/debug operations.
export PIP_NO_CACHE_DIR=1


# =============================================================================
# Persist selected runtime paths for debugging / other shells
# =============================================================================

RUNTIME_ENV_FILE="/tmp/vla_jepa_runtime.env"

{
    printf 'export VLA_JEPA_CONTROLLER_CONFIG=%q\n' "${VLA_JEPA_CONTROLLER_CONFIG}"
    printf 'export VLA_JEPA_CHECKPOINT=%q\n' "${VLA_JEPA_CHECKPOINT}"
    printf 'export HF_HOME=%q\n' "${HF_HOME}"
    printf 'export HF_HUB_CACHE=%q\n' "${HF_HUB_CACHE}"
    printf 'export HUGGINGFACE_HUB_CACHE=%q\n' "${HUGGINGFACE_HUB_CACHE}"
} > "${RUNTIME_ENV_FILE}"


# =============================================================================
# Summary
# =============================================================================

echo
echo "======================================================================"
echo " VLA-JEPA runtime"
echo "======================================================================"

echo "Python:"
echo "  ${PYTHON}"
echo

echo "Config:"
echo "  ${CONFIG_PATH}"
echo

echo "Checkpoint:"
echo "  ${VLA_JEPA_CHECKPOINT}"
echo

echo "LeRobot:"
echo "  ${LEROBOT_ROOT}"
echo

echo "HF_HOME:"
echo "  ${HF_HOME}"
echo

echo "HF_HUB_CACHE:"
echo "  ${HF_HUB_CACHE}"
echo

echo "Task name:"
echo "  ${TASK_NAME}"

echo "======================================================================"
echo


# =============================================================================
# Optional full preflight
# =============================================================================

if [[ "${RUN_PREFLIGHT}" == "1" ]]; then

    echo "Running full VLA-JEPA preflight..."
    echo
    echo "Hugging Face is forced offline during the preflight."
    echo "This verifies that the mounted checkpoint/cache are sufficient."
    echo

    env \
        HF_HUB_OFFLINE=1 \
        TRANSFORMERS_OFFLINE=1 \
        "${PYTHON}" \
            "${THIS_DIR}/test_setup.py" \
            --config "${CONFIG_PATH}" \
            --task-name "${TASK_NAME}"

    if [[ "${PREFLIGHT_ONLY}" == "1" ]]; then
        echo
        echo "Preflight completed. Server not started."
        exit 0
    fi

fi


# =============================================================================
# Start server
# =============================================================================

SERVER_ARGS=(
    --config "${CONFIG_PATH}"
    --task-name "${TASK_NAME}"
)

if [[ -n "${HOST_VALUE}" ]]; then
    SERVER_ARGS+=(--host "${HOST_VALUE}")
fi

if [[ -n "${PORT_VALUE}" ]]; then
    SERVER_ARGS+=(--port "${PORT_VALUE}")
fi

if [[ "${USE_COSMOS}" == "1" ]]; then
    SERVER_ARGS+=(--use-cosmos-task-description)
fi


echo
echo "Starting VLA-JEPA Flask server..."
echo

exec "${PYTHON}" \
    "${THIS_DIR}/server.py" \
    "${SERVER_ARGS[@]}"