# syntax=docker/dockerfile:1.7

ARG UV_VERSION=0.11.3


# ================================================================
# uv
# ================================================================
FROM ghcr.io/astral-sh/uv:${UV_VERSION} AS uv


# ================================================================
# Base CUDA per NVIDIA DGX Spark / GB10 / ARM64
#
# Questa è la stessa base già validata sulla Spark con Mimic Video.
# ================================================================
FROM nvidia/cuda:12.9.2-cudnn-devel-ubuntu24.04


# ================================================================
# Ambiente
# ================================================================
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    CUDA_HOME=/usr/local/cuda \
    CUDA_PATH=/usr/local/cuda \
    VIRTUAL_ENV=/opt/interleave-pizero-venv \
    UV_PROJECT_ENVIRONMENT=/opt/interleave-pizero-venv \
    UV_LINK_MODE=copy \
    UV_PYTHON=3.10 \
    PATH="/opt/interleave-pizero-venv/bin:/usr/local/cuda/bin:${PATH}" \
    TORCH_CUDA_ARCH_LIST="12.0+PTX" \
    MAX_JOBS=4 \
    TF_CPP_MIN_LOG_LEVEL=2 \
    TOKENIZERS_PARALLELISM=false


# ================================================================
# Dipendenze Linux
# ================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    ffmpeg \
    git \
    git-lfs \
    libgl1 \
    libglib2.0-0 \
    libjpeg-dev \
    libpng-dev \
    ninja-build \
    pkg-config \
    zlib1g-dev \
    && rm -rf /var/lib/apt/lists/* \
    && git lfs install


# ================================================================
# Installa uv
# ================================================================
COPY --from=uv /uv /uvx /usr/local/bin/


# ================================================================
# Python 3.10 + virtual environment
#
# Python 3.10 è la versione usata dagli autori di open-pi-zero.
# ================================================================
RUN uv python install 3.10 && \
    uv venv "${VIRTUAL_ENV}" --python 3.10


# ================================================================
# Tooling Python
# ================================================================
RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install \
        --python "${VIRTUAL_ENV}/bin/python" \
        setuptools==80.9.0 \
        wheel \
        packaging \
        pip


# ================================================================
# PyTorch CUDA 12.9 per ARM64 / GB10
#
# Questa combinazione è già stata testata sulla Spark:
#   torch       2.8.0
#   torchvision 0.23.0
#   CUDA        12.9
#
# Non installiamo torch==2.5.1 richiesto originariamente dagli
# autori perché non è la configurazione adatta al nostro GB10 ARM64.
# ================================================================
RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install \
        --python "${VIRTUAL_ENV}/bin/python" \
        torch==2.8.0 \
        torchvision==0.23.0 \
        --index-url https://download.pytorch.org/whl/cu129


# ================================================================
# Triton
#
# Compatibile con PyTorch 2.8 / CUDA 12.9.
# Può essere usato da torch.compile.
# ================================================================
RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install \
        --python "${VIRTUAL_ENV}/bin/python" \
        triton==3.4.0 \
        --index-url https://download.pytorch.org/whl


# ================================================================
# Dipendenze Interleave-VLA / open-pi-zero
#
# Manteniamo le versioni dichiarate dagli autori quando possibile.
#
# scipy non è richiesta dagli autori, ma serve al nostro builder
# UR5e per scipy.spatial.transform.Rotation.
# ================================================================
RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install \
        --python "${VIRTUAL_ENV}/bin/python" \
        numpy==1.26.4 \
        scipy==1.11.4 \
        protobuf==3.20.3 \
        tensorflow==2.15.0 \
        tensorflow_datasets==4.9.2 \
        transformers==4.47.1 \
        einops \
        "gsutil>=5.32" \
        hydra-core \
        imageio \
        matplotlib \
        omegaconf \
        pillow \
        pretty_errors \
        tqdm \
        wandb \
        sentencepiece==0.2.0


# ================================================================
# bitsandbytes
#
# Gli autori usano bitsandbytes==0.45.0.
# Per la Spark ARM64 / GB10 usiamo una release recente con supporto
# aarch64 e CUDA compatibile con il nostro stack.
# ================================================================
RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install \
        --python "${VIRTUAL_ENV}/bin/python" \
        bitsandbytes==0.49.0


# ================================================================
# Copia Interleave-VLA
#
# STRUTTURA HOST:
#
# interleave_pi_zero_workspace/
# ├── Docker/
# │   └── Dockerfile
# ├── external/
# │   └── Interleave-VLA/
# ├── checkpoints/
# ├── outputs/
# ├── processed_data/
# └── scripts/
#
# Il build context deve quindi essere:
# interleave_pi_zero_workspace/
# ================================================================
COPY external/Interleave-VLA /workspace/Interleave-VLA


# ================================================================
# Installa open-pi-zero
#
# --no-deps è intenzionale:
# il pyproject degli autori contiene pin incompatibili con il nostro
# stack Spark, in particolare torch e bitsandbytes.
#
# Le dipendenze necessarie sono già state installate sopra.
# ================================================================
WORKDIR /workspace/Interleave-VLA/open-pi-zero

RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install \
        --python "${VIRTUAL_ENV}/bin/python" \
        --editable . \
        --no-deps


# ================================================================
# Directory persistenti / runtime
#
# Queste directory verranno normalmente collegate alle omonime
# directory dell'host tramite volume mount.
# ================================================================
RUN mkdir -p \
    /workspace/processed_data \
    /workspace/checkpoints \
    /workspace/outputs


# ================================================================
# Path usati da Interleave / open-pi-zero
# ================================================================
ENV VLA_DATA_DIR=/workspace/processed_data \
    VLA_LOG_DIR=/workspace/outputs \
    HF_HOME=/workspace/checkpoints/huggingface \
    TRANSFORMERS_CACHE=/workspace/checkpoints/huggingface


# ================================================================
# Smoke test build-time
#
# Durante docker build non assumiamo che la GPU sia accessibile.
# Verifichiamo solamente gli import e le versioni fondamentali.
# ================================================================
RUN "${VIRTUAL_ENV}/bin/python" - <<'PY'
import platform

import numpy as np
import scipy
import tensorflow as tf
import tensorflow_datasets as tfds
import torch
import transformers
import bitsandbytes as bnb

from scipy.spatial.transform import Rotation

print("Architecture:", platform.machine())
print("NumPy:", np.__version__)
print("SciPy:", scipy.__version__)
print("TensorFlow:", tf.__version__)
print("TFDS:", tfds.__version__)
print("PyTorch:", torch.__version__)
print("Transformers:", transformers.__version__)
print("bitsandbytes:", bnb.__version__)

assert platform.machine() == "aarch64"
assert np.__version__ == "1.26.4"
assert tfds.__version__ == "4.9.2"
assert torch.__version__.startswith("2.8.")

R = Rotation.from_euler(
    "xyz",
    [0.0, 0.0, 0.0]
).as_matrix()

assert R.shape == (3, 3)

print("Interleave base environment: OK")
PY


# ================================================================
# Directory iniziale
# ================================================================
WORKDIR /workspace/Interleave-VLA/open-pi-zero


CMD ["/bin/bash"]