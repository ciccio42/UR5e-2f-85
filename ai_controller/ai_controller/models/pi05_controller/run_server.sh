#!/usr/bin/env bash

set -Eeuo pipefail

# =============================================================================
# PI0.5 inference server launcher
#
# Normal execution:
#   ./run_server.sh
#
# Full preflight + server:
#   ./run_server.sh --preflight
#
# Full preflight only:
#   ./run_server.sh --preflight-only
#
# Different config:
#   ./run_server.sh --config pi05_config.yaml
# =============================================================================

THIS_DIR="$(
    cd "$(dirname "${BASH_SOURCE[0]}")"
    pwd
)"

# ----------------------------------------------------------------------
# DGX Spark GB10 / sm_121 NVRTC compatibility
#
# PI0.5 keeps the training-compatible PyTorch build:
#     torch 2.11.0+cu128
#
# CUDA 12.8 NVRTC cannot compile JIT kernels for sm_121.
# We therefore override only NVRTC with CUDA 12.9, which supports sm_121.
# ----------------------------------------------------------------------

NVRTC_DIR="${THIS_DIR}/venv/nvrtc_12_9"

if [[ ! -f "${NVRTC_DIR}/libnvrtc.so.12" ]]; then
    echo "Missing PI0.5 NVRTC library: ${NVRTC_DIR}/libnvrtc.so.12" >&2
    exit 1
fi

if [[ ! -f "${NVRTC_DIR}/libnvrtc-builtins.so.12.9" ]]; then
    echo "Missing PI0.5 NVRTC builtins: ${NVRTC_DIR}/libnvrtc-builtins.so.12.9" >&2
    exit 1
fi

export LD_LIBRARY_PATH="${NVRTC_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export LD_PRELOAD="${NVRTC_DIR}/libnvrtc.so.12${LD_PRELOAD:+:${LD_PRELOAD}}"

# -----------------------------------------------------------------------------
# Runtime paths
# -----------------------------------------------------------------------------

VENV_DIR="${PI05_VENV_DIR:-${THIS_DIR}/venv/pi05_venv}"
PYTHON="${VENV_DIR}/bin/python"

PI05_CHECKPOINT="${PI05_CHECKPOINT:-/opt/pi05/checkpoint}"
PI05_LEROBOT_ROOT="${PI05_LEROBOT_ROOT:-/opt/pi05/lerobot}"
PI05_PALIGEMMA_PATH="${PI05_PALIGEMMA_PATH:-/opt/pi05/paligemma-3b-pt-224}"

export PI05_PALIGEMMA_PATH
export PI05_CHECKPOINT
export PI05_LEROBOT_ROOT


export PYTHONUNBUFFERED=1
export PYTHONDONTWRITEBYTECODE=1
export TOKENIZERS_PARALLELISM=false
export PIP_NO_CACHE_DIR=1

# -----------------------------------------------------------------------------
# Defaults
# -----------------------------------------------------------------------------

CONFIG_VALUE="${PI05_CONFIG_NAME:-pi05_config.yaml}"
TASK_NAME="pick_place"
RUN_PREFLIGHT=0
PREFLIGHT_ONLY=0
HOST_VALUE=""
PORT_VALUE=""

die() {
    echo "ERROR: $*" >&2
    exit 1
}

usage() {
    cat <<EOF
Usage:
  $0 [options]

Options:
  --config FILE
      Runtime YAML. Relative paths are resolved from:
        ${THIS_DIR}

  --preflight
      Run the full PI0.5 preflight before starting the server.

  --preflight-only
      Run the full PI0.5 preflight and exit.

  --task-name NAME
      Task family passed to pi05_server.py.
      Default: pick_place

  --host HOST
      Override server host.

  --port PORT
      Override server port.

  -h, --help
      Show this message.
EOF
}

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
        -h|--help)
            usage
            exit 0
            ;;
        *)
            die "Unknown argument: $1"
            ;;
    esac
done

# -----------------------------------------------------------------------------
# Required runtime
# -----------------------------------------------------------------------------

[[ -x "${PYTHON}" ]] || die "PI0.5 venv Python not found: ${PYTHON}"

if [[ "${CONFIG_VALUE}" = /* ]]; then
    CONFIG_PATH="${CONFIG_VALUE}"
else
    CONFIG_PATH="${THIS_DIR}/${CONFIG_VALUE}"
fi

[[ -f "${CONFIG_PATH}" ]] || die "Config not found: ${CONFIG_PATH}"
CONFIG_PATH="$(realpath "${CONFIG_PATH}")"

export PI05_CONTROLLER_CONFIG="${CONFIG_PATH}"

for required_file in \
    "${THIS_DIR}/pi05_server.py" \
    "${THIS_DIR}/test_setup.py" \
    "${THIS_DIR}/pi05_controller.py" \
    "${THIS_DIR}/pi05.py" \
    "${THIS_DIR}/pi05_utils.py"; do
    [[ -f "${required_file}" ]] || die "Required source file not found: ${required_file}"
done

[[ -d "${PI05_CHECKPOINT}" ]] || die "Checkpoint directory not found: ${PI05_CHECKPOINT}"
[[ -d "${PI05_PALIGEMMA_PATH}" ]] || \
    die "PaliGemma directory not found: ${PI05_PALIGEMMA_PATH}"

for checkpoint_file in \
    config.json \
    model.safetensors \
    policy_preprocessor.json \
    policy_postprocessor.json \
    policy_preprocessor_step_3_normalizer_processor.safetensors \
    policy_postprocessor_step_0_unnormalizer_processor.safetensors; do
    [[ -f "${PI05_CHECKPOINT}/${checkpoint_file}" ]] || \
        die "Checkpoint file not found: ${PI05_CHECKPOINT}/${checkpoint_file}"
done

[[ -d "${PI05_LEROBOT_ROOT}/src/lerobot" ]] || \
    die "LeRobot source not mounted at: ${PI05_LEROBOT_ROOT}"


# ai_controller package root:
# /home/ros2_ws/src/ai_controller
AI_CONTROLLER_ROOT="$(
    cd "${THIS_DIR}/../../.."
    pwd
)"

export PYTHONPATH="${AI_CONTROLLER_ROOT}:${THIS_DIR}${PYTHONPATH:+:${PYTHONPATH}}"

# -----------------------------------------------------------------------------
# Summary
# -----------------------------------------------------------------------------

echo
echo "======================================================================"
echo " PI0.5 runtime"
echo "======================================================================"
echo "Python:      ${PYTHON}"
echo "Config:      ${CONFIG_PATH}"
echo "Checkpoint:  ${PI05_CHECKPOINT}"
echo "LeRobot:     ${PI05_LEROBOT_ROOT}"
echo "PaliGemma:   ${PI05_PALIGEMMA_PATH}"
echo "Task:        ${TASK_NAME}"
echo "======================================================================"
echo

# -----------------------------------------------------------------------------
# Optional preflight
# -----------------------------------------------------------------------------

if [[ "${RUN_PREFLIGHT}" == "1" ]]; then
    echo "Running PI0.5 preflight..."
    echo

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

# -----------------------------------------------------------------------------
# Start server
# -----------------------------------------------------------------------------

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

echo
echo "Starting PI0.5 Flask server..."
echo

exec "${PYTHON}" \
    "${THIS_DIR}/pi05_server.py" \
    "${SERVER_ARGS[@]}"
