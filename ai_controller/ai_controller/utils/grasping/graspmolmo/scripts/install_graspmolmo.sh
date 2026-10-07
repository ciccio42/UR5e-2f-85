#!/usr/bin/env bash

set -euo pipefail

# =============================================================================
# GraspMolmo environment installation
# =============================================================================

VENV_DIR="${VENV_DIR:-/opt/graspmolmo_venv}"
SOURCE_DIR="${SOURCE_DIR:-/opt/GraspMolmo}"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"

GRASPMOLMO_REPO="https://github.com/abhaybd/GraspMolmo.git"

# Pin della versione che stiamo usando per rendere il setup riproducibile.
GRASPMOLMO_COMMIT="73990b5c5121ac5ab5fd402453bfeeefd270d153"

# Keep the GraspMolmo environment fully isolated from the ROS/SeeDo Python
unset VIRTUAL_ENV
unset PYTHONPATH
unset PYTHONHOME

export PYTHONNOUSERSITE=1


echo "============================================================"
echo " GraspMolmo installation"
echo "============================================================"
echo "Python:      ${PYTHON_BIN}"
echo "Venv:        ${VENV_DIR}"
echo "Source:      ${SOURCE_DIR}"
echo


# -----------------------------------------------------------------------------
# Check Python
# -----------------------------------------------------------------------------

if ! command -v "${PYTHON_BIN}" >/dev/null 2>&1; then
    echo "ERROR: ${PYTHON_BIN} not found."
    exit 1
fi

"${PYTHON_BIN}" --version


# -----------------------------------------------------------------------------
# Create virtual environment
# -----------------------------------------------------------------------------

if [ ! -x "${VENV_DIR}/bin/python" ]; then
    echo "[GraspMolmo] Creating virtual environment..."
    "${PYTHON_BIN}" -m venv "${VENV_DIR}"
else
    echo "[GraspMolmo] Virtual environment already exists."
fi


VENV_PYTHON="${VENV_DIR}/bin/python"
VENV_PIP="${VENV_DIR}/bin/pip"

"${VENV_PYTHON}" -m pip install --upgrade pip setuptools wheel


# -----------------------------------------------------------------------------
# Clone GraspMolmo
# -----------------------------------------------------------------------------

if [ ! -d "${SOURCE_DIR}/.git" ]; then
    echo "[GraspMolmo] Cloning repository..."
    git clone "${GRASPMOLMO_REPO}" "${SOURCE_DIR}"
else
    echo "[GraspMolmo] Repository already present."
fi

cd "${SOURCE_DIR}"

git config --global --add safe.directory "${SOURCE_DIR}"

git fetch origin
git checkout "${GRASPMOLMO_COMMIT}"


# -----------------------------------------------------------------------------
# Install GraspMolmo inference dependencies
# -----------------------------------------------------------------------------

echo "[GraspMolmo] Installing inference dependencies..."

# -----------------------------------------------------------------------------
# PyTorch for NVIDIA GB10 / CUDA 13
# -----------------------------------------------------------------------------

echo "[GraspMolmo] Installing CUDA 13 PyTorch stack..."

"${VENV_PIP}" install \
    "torch==2.11.0+cu130" \
    "torchvision==0.26.0+cu130" \
    --index-url https://download.pytorch.org/whl/cu130


# -----------------------------------------------------------------------------
# GraspMolmo inference dependencies
#
# Install manually so the upstream setup.py cannot replace our CUDA-compatible
# PyTorch stack.
# -----------------------------------------------------------------------------

echo "[GraspMolmo] Installing inference dependencies..."

"${VENV_PIP}" install \
    "numpy~=1.26.4" \
    "Pillow~=10.2.0" \
    "tqdm~=4.67.1" \
    "transformers~=4.52.4" \
    "accelerate~=1.7.0" \
    "tensorflow==2.18.0" \
    "safetensors" \
    "einops"


# -----------------------------------------------------------------------------
# Install GraspMolmo itself without resolving dependencies again
# -----------------------------------------------------------------------------

echo "[GraspMolmo] Installing GraspMolmo package..."

"${VENV_PIP}" install -e . --no-deps


# -----------------------------------------------------------------------------
# Flask server dependencies
# -----------------------------------------------------------------------------

echo "[GraspMolmo] Installing HTTP server dependencies..."

"${VENV_PIP}" install flask


# -----------------------------------------------------------------------------
# Sanity check
# -----------------------------------------------------------------------------

echo "[GraspMolmo] Running import test..."

"${VENV_PYTHON}" - <<'PY'
from graspmolmo.inference.grasp_predictor import GraspMolmo

print("GraspMolmo import: OK")
PY


echo
echo "============================================================"
echo " GraspMolmo environment ready"
echo "============================================================"
echo
echo "Python:"
echo "  ${VENV_PYTHON}"
echo
echo "Source:"
echo "  ${SOURCE_DIR}"