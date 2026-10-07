#!/usr/bin/env bash

set -euo pipefail

# =============================================================================
# M2T2 environment installation
# =============================================================================

PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"

VENV_DIR="${VENV_DIR:-/opt/m2t2_venv}"
SOURCE_DIR="${SOURCE_DIR:-/opt/M2T2}"

M2T2_REPO="https://github.com/NVlabs/M2T2.git"
M2T2_COMMIT="be2e5f6feae2f961615ab0f7b643f0f98b0c3a50"

CUDA_HOME="${CUDA_HOME:-/usr/local/cuda-13.0}"

# Keep this environment isolated from ROS / SeeDo.
unset VIRTUAL_ENV
unset PYTHONPATH
unset PYTHONHOME

export PYTHONNOUSERSITE=1

# NVIDIA GB10 = compute capability 12.1.
export TORCH_CUDA_ARCH_LIST="12.1"

export CUDA_HOME
export PATH="${CUDA_HOME}/bin:${PATH}"
export LD_LIBRARY_PATH="${CUDA_HOME}/lib64:${LD_LIBRARY_PATH:-}"


echo "============================================================"
echo " M2T2 installation"
echo "============================================================"
echo "Python:          ${PYTHON_BIN}"
echo "Venv:            ${VENV_DIR}"
echo "Source:          ${SOURCE_DIR}"
echo "CUDA_HOME:       ${CUDA_HOME}"
echo "CUDA arch:       ${TORCH_CUDA_ARCH_LIST}"
echo


# -----------------------------------------------------------------------------
# Check environment
# -----------------------------------------------------------------------------

if ! command -v "${PYTHON_BIN}" >/dev/null 2>&1; then
    echo "ERROR: ${PYTHON_BIN} not found."
    exit 1
fi

if [ ! -x "${CUDA_HOME}/bin/nvcc" ]; then
    echo "ERROR: nvcc not found under ${CUDA_HOME}/bin."
    exit 1
fi

"${PYTHON_BIN}" --version
"${CUDA_HOME}/bin/nvcc" --version


# -----------------------------------------------------------------------------
# Create isolated virtual environment
# -----------------------------------------------------------------------------

if [ ! -x "${VENV_DIR}/bin/python" ]; then
    echo "[M2T2] Creating virtual environment..."
    "${PYTHON_BIN}" -m venv "${VENV_DIR}"
else
    echo "[M2T2] Virtual environment already exists."
fi

VENV_PYTHON="${VENV_DIR}/bin/python"
VENV_PIP="${VENV_DIR}/bin/pip"

"${VENV_PYTHON}" -m pip install --upgrade \
    pip \
    setuptools \
    wheel


# -----------------------------------------------------------------------------
# Clone M2T2
# -----------------------------------------------------------------------------

if [ ! -d "${SOURCE_DIR}/.git" ]; then
    echo "[M2T2] Cloning repository..."
    git clone "${M2T2_REPO}" "${SOURCE_DIR}"
else
    echo "[M2T2] Repository already present."
fi

git config --global --add safe.directory "${SOURCE_DIR}"

cd "${SOURCE_DIR}"

git fetch origin
git checkout "${M2T2_COMMIT}"


# -----------------------------------------------------------------------------
# Install PyTorch for NVIDIA GB10 / CUDA 13
# -----------------------------------------------------------------------------

echo "[M2T2] Installing CUDA 13 PyTorch stack..."

"${VENV_PIP}" install \
    "torch==2.11.0+cu130" \
    "torchvision==0.26.0+cu130" \
    --index-url https://download.pytorch.org/whl/cu130


# -----------------------------------------------------------------------------
# Build tools
# -----------------------------------------------------------------------------

echo "[M2T2] Installing build tools..."

"${VENV_PIP}" install \
    ninja \
    packaging


# -----------------------------------------------------------------------------
# M2T2 Python dependencies
#
# We install them explicitly instead of letting setup.py resolve torch again.
# -----------------------------------------------------------------------------

echo "[M2T2] Installing runtime dependencies..."

"${VENV_PIP}" install \
    "numpy==1.26.4" \
    h5py \
    hydra-core \
    matplotlib \
    meshcat \
    scikit-learn \
    scipy \
    tensorboard \
    trimesh \
    flask \
    pyyaml


# -----------------------------------------------------------------------------
# Compile PointNet++ CUDA extension
# -----------------------------------------------------------------------------

echo "[M2T2] Building PointNet++ CUDA extension for sm_121..."

cd "${SOURCE_DIR}"

"${VENV_PIP}" install \
    ./pointnet2_ops \
    --no-build-isolation \
    --no-deps


# -----------------------------------------------------------------------------
# Install M2T2 itself
# -----------------------------------------------------------------------------

echo "[M2T2] Installing M2T2 package..."

"${VENV_PIP}" install \
    -e . \
    --no-build-isolation \
    --no-deps


# -----------------------------------------------------------------------------
# Import checks
# -----------------------------------------------------------------------------

echo "[M2T2] Running import checks..."

"${VENV_PYTHON}" - <<'PY'
import torch
import m2t2
import pointnet2_ops

print("PyTorch:", torch.__version__)
print("CUDA available:", torch.cuda.is_available())

if not torch.cuda.is_available():
    raise RuntimeError("CUDA is not available.")

print("GPU:", torch.cuda.get_device_name(0))
print("Capability:", torch.cuda.get_device_capability(0))

print("M2T2 import: OK")
print("pointnet2_ops import: OK")
PY


echo
echo "============================================================"
echo " M2T2 environment ready"
echo "============================================================"
echo
echo "Python:"
echo "  ${VENV_PYTHON}"
echo
echo "Source:"
echo "  ${SOURCE_DIR}"