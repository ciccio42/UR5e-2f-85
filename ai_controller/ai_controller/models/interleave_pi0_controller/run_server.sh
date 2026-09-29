#!/usr/bin/env bash
set -Eeuo pipefail

# =============================================================================
# Interleave-Pi0 inference server launcher
#
# Uses the persistent Conda environment created once by:
#
#   environment/create_env.sh
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
#   ./run_server.sh \
#       --config interleave_pi0_config.yaml
# =============================================================================

# =============================================================================
# Interleave workspace
# =============================================================================

INTERLEAVE_WORKSPACE="${INTERLEAVE_WORKSPACE:-/workspace}"

OPEN_PI_ZERO="${OPEN_PI_ZERO:-${INTERLEAVE_WORKSPACE}/external/Interleave-VLA/open-pi-zero}"

INTERLEAVE_PI0_PALIGEMMA="${INTERLEAVE_PI0_PALIGEMMA:-${INTERLEAVE_WORKSPACE}/checkpoints/paligemma/paligemma-3b-pt-224}"

export INTERLEAVE_WORKSPACE
export OPEN_PI_ZERO
export INTERLEAVE_PI0_PALIGEMMA

THIS_DIR="$(
    cd "$(dirname "${BASH_SOURCE[0]}")"
    pwd
)"

ENV_PREFIX="${THIS_DIR}/environment/.conda_env"
PYTHON="${ENV_PREFIX}/bin/python"

DEFAULT_CONFIG_NAME="${INTERLEAVE_PI0_CONFIG_NAME:-interleave_pi0_grounding_bin_config.yaml}"

CONFIG_VALUE="${DEFAULT_CONFIG_NAME}"

RUN_PREFLIGHT=0
PREFLIGHT_ONLY=0


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
      Run the complete dummy-inference preflight before starting Flask.

  --preflight-only
      Run the complete preflight and exit without starting Flask.

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
# Conda environment
# =============================================================================


if [[ ! -x "${PYTHON}" ]]; then

    cat >&2 <<EOF

ERROR: Interleave Conda environment does not exist:

    ${ENV_PREFIX}

Create it once with:

    environment/create_env.sh

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


if [[ ! -f "${CONFIG_PATH}" ]]; then
    die "Config not found: ${CONFIG_PATH}"
fi


CONFIG_PATH="$(
    realpath "${CONFIG_PATH}"
)"


# =============================================================================
# Persist selected runtime config for the ROS client
# =============================================================================

INTERLEAVE_PI0_CONTROLLER_CONFIG="${CONFIG_PATH}"
export INTERLEAVE_PI0_CONTROLLER_CONFIG

RUNTIME_ENV_FILE="/tmp/interleave_pi0_runtime.env"

printf 'export INTERLEAVE_PI0_CONTROLLER_CONFIG=%q\n' \
    "${INTERLEAVE_PI0_CONTROLLER_CONFIG}" \
    > "${RUNTIME_ENV_FILE}"

# =============================================================================
# Checkpoint selection
#
# Il checkpoint di default viene scelto in base al config.
#
# È comunque possibile sovrascriverlo esplicitamente:
#
#   INTERLEAVE_PI0_CHECKPOINT=/path/custom.pt ./run_server.sh ...
# =============================================================================

CONFIG_BASENAME="$(basename "${CONFIG_PATH}")"


if [[ -z "${INTERLEAVE_PI0_CHECKPOINT:-}" ]]; then

    case "${CONFIG_BASENAME}" in

        interleave_pi0_grounding_bin_config.yaml)
            INTERLEAVE_PI0_CHECKPOINT="${INTERLEAVE_WORKSPACE}/checkpoints/posttraining/bin_grounding/step66240.pt"
            ;;

        interleave_pi0_config.yaml)
            INTERLEAVE_PI0_CHECKPOINT="${INTERLEAVE_WORKSPACE}/checkpoints/posttraining/box_only/step66240.pt"
            ;;

        *)
            die \
                "No default checkpoint is defined for config '${CONFIG_BASENAME}'. " \
                "Set INTERLEAVE_PI0_CHECKPOINT explicitly."
            ;;

    esac

fi


export INTERLEAVE_PI0_CHECKPOINT

# =============================================================================
# Required runtime assets
#
# Their exact container paths are deliberately NOT hard-coded here.
# They will depend on the final bind mounts used for:
#
#   - Interleave-VLA / open-pi-zero
#   - checkpoint
#   - PaliGemma
# =============================================================================

# =============================================================================
# Required runtime assets
# =============================================================================

[[ -n "${OPEN_PI_ZERO:-}" ]] || \
    die "OPEN_PI_ZERO is not set."

[[ -n "${INTERLEAVE_PI0_CHECKPOINT:-}" ]] || \
    die "INTERLEAVE_PI0_CHECKPOINT is not set."

[[ -n "${INTERLEAVE_PI0_PALIGEMMA:-}" ]] || \
    die "INTERLEAVE_PI0_PALIGEMMA is not set."

[[ -d "${OPEN_PI_ZERO}" ]] || \
    die "OPEN_PI_ZERO not found: ${OPEN_PI_ZERO}"

[[ -f "${INTERLEAVE_PI0_CHECKPOINT}" ]] || \
    die "Checkpoint not found: ${INTERLEAVE_PI0_CHECKPOINT}"

[[ -d "${INTERLEAVE_PI0_PALIGEMMA}" ]] || \
    die "PaliGemma not found: ${INTERLEAVE_PI0_PALIGEMMA}"


# =============================================================================
# Python path
#
# THIS_DIR:
#   .../ai_controller/ai_controller/models/interleave_pi0_controller
#
# ../../..:
#   outer ai_controller package root
# =============================================================================


AI_CONTROLLER_ROOT="$(
    cd "${THIS_DIR}/../../.."
    pwd
)"


export PYTHONPATH="${OPEN_PI_ZERO}:${AI_CONTROLLER_ROOT}:${THIS_DIR}${PYTHONPATH:+:${PYTHONPATH}}"


# =============================================================================
# CUDA 12.9 compiler tools from the Conda environment
#
# The ROS container itself has CUDA 12.8 under /usr/local/cuda.
#
# Interleave must NOT use:
#
#     /usr/local/cuda/bin/ptxas
#
# We explicitly locate the CUDA 12.9 compiler installed into .conda_env.
# =============================================================================


PTXAS="$(
    find "${ENV_PREFIX}" \
        -type f \
        -name ptxas \
        -perm -u+x \
        -print \
        -quit
)"


NVCC="$(
    find "${ENV_PREFIX}" \
        -type f \
        -name nvcc \
        -perm -u+x \
        -print \
        -quit
)"


[[ -n "${PTXAS}" ]] || \
    die "ptxas not found inside ${ENV_PREFIX}"

[[ -n "${NVCC}" ]] || \
    die "nvcc not found inside ${ENV_PREFIX}"


PTXAS_OUTPUT="$(
    "${PTXAS}" --version 2>&1
)"

NVCC_OUTPUT="$(
    "${NVCC}" --version 2>&1
)"


grep -q "V12.9.86" <<< "${PTXAS_OUTPUT}" || \
    die "Expected ptxas V12.9.86."

grep -q "V12.9.86" <<< "${NVCC_OUTPUT}" || \
    die "Expected nvcc V12.9.86."


CUDA_BIN="$(
    dirname "${NVCC}"
)"

CUDA_ROOT="$(
    cd "${CUDA_BIN}/.."
    pwd
)"


export CUDA_HOME="${CUDA_ROOT}"
export CUDA_PATH="${CUDA_ROOT}"

export PATH="${CUDA_BIN}:${PATH}"


export TRITON_PTXAS_PATH="${PTXAS}"
export TRITON_PTXAS_BLACKWELL_PATH="${PTXAS}"


# Same convention used by the validated Interleave Spark runtime.
export TORCH_CUDA_ARCH_LIST="12.0+PTX"


# =============================================================================
# Runtime environment
# =============================================================================


export PYTHONUNBUFFERED=1
export PYTHONDONTWRITEBYTECODE=1
export TOKENIZERS_PARALLELISM=false

# Avoid accidentally building a pip cache during runtime/debug operations.
export PIP_NO_CACHE_DIR=1


# =============================================================================
# Summary
# =============================================================================


echo
echo "======================================================================"
echo " Interleave-Pi0 runtime"
echo "======================================================================"
echo "Python:"
echo "  ${PYTHON}"
echo
echo "Config:"
echo "  ${CONFIG_PATH}"
echo
echo "Open-Pi-Zero:"
echo "  ${OPEN_PI_ZERO}"
echo
echo "Checkpoint:"
echo "  ${INTERLEAVE_PI0_CHECKPOINT}"
echo
echo "PaliGemma:"
echo "  ${INTERLEAVE_PI0_PALIGEMMA}"
echo
echo "CUDA_HOME:"
echo "  ${CUDA_HOME}"
echo
echo "ptxas:"
echo "  ${PTXAS}"
echo
echo "nvcc:"
echo "  ${NVCC}"
echo "======================================================================"
echo


# =============================================================================
# Optional full preflight
# =============================================================================


if [[ "${RUN_PREFLIGHT}" == "1" ]]; then

    "${PYTHON}" \
        "${THIS_DIR}/test_setup.py" \
        --config "${CONFIG_PATH}"

    if [[ "${PREFLIGHT_ONLY}" == "1" ]]; then
        echo
        echo "Preflight completed. Server not started."
        exit 0
    fi

fi


# =============================================================================
# Start server
# =============================================================================


echo
echo "Starting Interleave-Pi0 Flask server..."
echo


exec "${PYTHON}" \
    "${THIS_DIR}/server.py" \
    --config "${CONFIG_PATH}" \
    --task-name pick_place