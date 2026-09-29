#!/usr/bin/env bash
set -Eeuo pipefail

# =============================================================================
# Interleave-Pi0 Conda environment creation
#
# Questo script:
#   - viene eseguito DENTRO ur_robotiq_teleoperation_container;
#   - usa il Conda già installato sulla Spark host e montato nel container;
#   - crea un environment persistente nella directory del controller;
#   - usa environment.yml come unica sorgente delle dipendenze;
#   - evita la cache pip;
#   - usa una package cache Conda locale e temporanea;
#   - verifica Python e la toolchain CUDA 12.9 installata nell'environment.
#
# Normalmente deve essere eseguito UNA SOLA VOLTA.
# =============================================================================


# -----------------------------------------------------------------------------
# Paths
# -----------------------------------------------------------------------------

THIS_DIR="$(
    cd "$(dirname "${BASH_SOURCE[0]}")"
    pwd
)"

ENV_FILE="${THIS_DIR}/environment.yaml"
ENV_PREFIX="${THIS_DIR}/.conda_env"

CONDA_PKGS_DIR="${THIS_DIR}/.conda_pkgs"
TMP_DIR="${THIS_DIR}/.tmp"


# -----------------------------------------------------------------------------
# Conda
#
# Il Conda della Spark host viene montato nel container mantenendo lo stesso
# path:
#
#   /home/asus-mivia/anaconda3
#
# È possibile sovrascrivere il path tramite:
#
#   CONDA_BIN=/altro/path/conda ./create_env.sh
# -----------------------------------------------------------------------------

CONDA_BIN="${CONDA_BIN:-/home/asus-mivia/anaconda3/bin/conda}"


# -----------------------------------------------------------------------------
# Safety threshold
#
# È soltanto una protezione contro il riempimento accidentale del filesystem.
# Non rappresenta una stima esatta della dimensione finale dell'environment.
#
# Si può modificare, se necessario:
#
#   MIN_FREE_GB=30 ./create_env.sh
# -----------------------------------------------------------------------------

MIN_FREE_GB="${MIN_FREE_GB:-25}"


# =============================================================================
# Utility
# =============================================================================

die()
{
    echo
    echo "ERROR: $*" >&2
    echo
    exit 1
}


print_header()
{
    echo
    echo "======================================================================"
    echo " $1"
    echo "======================================================================"
}


# =============================================================================
# Initial checks
# =============================================================================

print_header "Interleave-Pi0 Conda environment"

echo "Environment file : ${ENV_FILE}"
echo "Environment path : ${ENV_PREFIX}"
echo "Conda executable : ${CONDA_BIN}"
echo "Conda pkg cache  : ${CONDA_PKGS_DIR}"
echo


# -----------------------------------------------------------------------------
# Architecture
# -----------------------------------------------------------------------------

ARCH="$(uname -m)"

echo "Architecture     : ${ARCH}"

if [[ "${ARCH}" != "aarch64" ]]; then
    die "Expected aarch64, found ${ARCH}"
fi


# -----------------------------------------------------------------------------
# environment.yml
# -----------------------------------------------------------------------------

if [[ ! -f "${ENV_FILE}" ]]; then
    die "environment.yaml not found: ${ENV_FILE}"
fi


# -----------------------------------------------------------------------------
# Host Conda mount
# -----------------------------------------------------------------------------

if [[ ! -x "${CONDA_BIN}" ]]; then
    cat >&2 <<EOF

ERROR: Conda is not available inside this container.

Expected:

    ${CONDA_BIN}

The host Anaconda installation must be mounted into the container, e.g.:

    -v /home/asus-mivia/anaconda3:/home/asus-mivia/anaconda3:ro

EOF
    exit 1
fi


echo
"${CONDA_BIN}" --version


# -----------------------------------------------------------------------------
# Do not overwrite an existing environment
# -----------------------------------------------------------------------------

if [[ -e "${ENV_PREFIX}" ]]; then
    cat >&2 <<EOF

ERROR: the Interleave Conda environment already exists:

    ${ENV_PREFIX}

This script intentionally does not overwrite an existing environment.

If the existing environment is valid, do not run this script again.

If it must be rebuilt, remove it explicitly first.

EOF
    exit 1
fi


# =============================================================================
# Disk-space check
# =============================================================================

print_header "Disk space"

df -h "${THIS_DIR}"

AVAILABLE_KB="$(
    df -Pk "${THIS_DIR}" \
        | awk 'NR==2 {print $4}'
)"

REQUIRED_KB="$(( MIN_FREE_GB * 1024 * 1024 ))"

echo
echo "Required safety margin: ${MIN_FREE_GB} GiB"

if (( AVAILABLE_KB < REQUIRED_KB )); then
    AVAILABLE_GB="$(( AVAILABLE_KB / 1024 / 1024 ))"

    die \
        "Only approximately ${AVAILABLE_GB} GiB are available. " \
        "Free more disk space before creating the environment."
fi


# =============================================================================
# Temporary directories / caches
# =============================================================================

print_header "Preparing temporary directories"

mkdir -p \
    "${CONDA_PKGS_DIR}" \
    "${TMP_DIR}"


# Conda uses an isolated package cache belonging only to this environment
# creation. This prevents writes to the host Anaconda package cache.
export CONDA_PKGS_DIRS="${CONDA_PKGS_DIR}"


# pip must not retain downloaded wheels.
export PIP_NO_CACHE_DIR=1
export PIP_DISABLE_PIP_VERSION_CHECK=1


# Keep temporary installation files local and easy to remove.
export TMPDIR="${TMP_DIR}"


# =============================================================================
# Cleanup
# =============================================================================

CREATION_STARTED=0


cleanup()
{
    exit_code=$?

    echo

    # Temporary files are never needed after the command finishes.
    rm -rf "${TMP_DIR}" || true

    if [[ ${exit_code} -ne 0 ]]; then

        echo "Environment creation FAILED."

        if [[ "${CREATION_STARTED}" == "1" ]]; then
            echo
            echo "Removing partial environment:"
            echo "  ${ENV_PREFIX}"

            rm -rf "${ENV_PREFIX}" || true
        fi

        echo
        echo "Removing temporary Conda package cache:"
        echo "  ${CONDA_PKGS_DIR}"

        rm -rf "${CONDA_PKGS_DIR}" || true

        echo
        df -h "${THIS_DIR}" || true

        exit "${exit_code}"
    fi
}

trap cleanup EXIT


# =============================================================================
# Environment creation
# =============================================================================

print_header "Creating Conda environment"

echo
echo "This can take a long time."
echo "PyTorch CUDA packages are large."
echo

CREATION_STARTED=1

"${CONDA_BIN}" env create \
    --prefix "${ENV_PREFIX}" \
    --file "${ENV_FILE}" \
    --yes


# =============================================================================
# Basic validation
# =============================================================================

print_header "Basic environment validation"

PYTHON="${ENV_PREFIX}/bin/python"

if [[ ! -x "${PYTHON}" ]]; then
    die "Python was not created at ${PYTHON}"
fi


"${PYTHON}" - <<'PY'
import platform
import sys

print("Python executable :", sys.executable)
print("Python version    :", sys.version.split()[0])
print("Architecture      :", platform.machine())

assert sys.version_info[:2] == (3, 10), sys.version
assert platform.machine() == "aarch64", platform.machine()

print()
print("Python environment: OK")
PY


# =============================================================================
# Locate CUDA compiler tools
# =============================================================================

print_header "CUDA 12.9 compiler tools"

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


if [[ -z "${PTXAS}" ]]; then
    die "ptxas was not found inside the Conda environment"
fi

if [[ -z "${NVCC}" ]]; then
    die "nvcc was not found inside the Conda environment"
fi


echo "ptxas: ${PTXAS}"
"${PTXAS}" --version

echo

echo "nvcc:   ${NVCC}"
"${NVCC}" --version


PTXAS_VERSION="$(
    "${PTXAS}" --version 2>&1 \
        | grep -o 'V12\.9\.[0-9]*' \
        | head -n1
)"

NVCC_VERSION="$(
    "${NVCC}" --version 2>&1 \
        | grep -o 'V12\.9\.[0-9]*' \
        | head -n1
)"


if [[ "${PTXAS_VERSION}" != "V12.9.86" ]]; then
    die \
        "Expected ptxas V12.9.86, found '${PTXAS_VERSION:-unknown}'"
fi

if [[ "${NVCC_VERSION}" != "V12.9.86" ]]; then
    die \
        "Expected nvcc V12.9.86, found '${NVCC_VERSION:-unknown}'"
fi


echo
echo "CUDA compiler tools: OK"


# =============================================================================
# Lightweight package-version validation
#
# Questo NON sostituisce preflight.py.
# Non carichiamo ancora checkpoint né eseguiamo inferenza.
# =============================================================================

print_header "Python package versions"

"${PYTHON}" - <<'PY'
import bitsandbytes
import einops
import hydra
import numpy
import omegaconf
import PIL
import safetensors
import scipy
import torch
import transformers
import triton

print(f"torch:          {torch.__version__}")
print(f"torch CUDA:     {torch.version.cuda}")
print(f"triton:         {triton.__version__}")
print(f"transformers:   {transformers.__version__}")
print(f"numpy:          {numpy.__version__}")
print(f"scipy:          {scipy.__version__}")
print(f"bitsandbytes:   {bitsandbytes.__version__}")
print(f"hydra-core:     {hydra.__version__}")
print(f"omegaconf:      {omegaconf.__version__}")
print(f"einops:         {einops.__version__}")
print(f"Pillow:         {PIL.__version__}")
print(f"safetensors:    {safetensors.__version__}")

assert torch.__version__ == "2.8.0+cu129", torch.__version__
assert torch.version.cuda == "12.9", torch.version.cuda
assert triton.__version__ == "3.4.0", triton.__version__
assert transformers.__version__ == "4.47.1", transformers.__version__
assert numpy.__version__ == "1.26.4", numpy.__version__
assert scipy.__version__ == "1.11.4", scipy.__version__
assert bitsandbytes.__version__ == "0.49.0", bitsandbytes.__version__
assert hydra.__version__ == "1.3.6", hydra.__version__
assert omegaconf.__version__ == "2.3.1", omegaconf.__version__
assert einops.__version__ == "0.8.2", einops.__version__
assert PIL.__version__ == "12.3.0", PIL.__version__
assert safetensors.__version__ == "0.8.0", safetensors.__version__

print()
print("Pinned package versions: OK")
PY


# =============================================================================
# Environment size
# =============================================================================

print_header "Environment size"

du -sh "${ENV_PREFIX}"

echo
df -h "${THIS_DIR}"


# =============================================================================
# Remove installation cache
# =============================================================================

print_header "Removing installation cache"

echo "Removing:"
echo "  ${CONDA_PKGS_DIR}"

rm -rf "${CONDA_PKGS_DIR}"


# =============================================================================
# Completed
# =============================================================================

print_header "Interleave-Pi0 environment created successfully"

echo "Environment:"
echo
echo "  ${ENV_PREFIX}"
echo
echo "Python:"
echo
echo "  ${ENV_PREFIX}/bin/python"
echo
echo "ptxas:"
echo
echo "  ${PTXAS}"
echo
echo "nvcc:"
echo
echo "  ${NVCC}"
echo
echo "The full model validation will be performed later by preflight.py."
echo

CREATION_STARTED=0