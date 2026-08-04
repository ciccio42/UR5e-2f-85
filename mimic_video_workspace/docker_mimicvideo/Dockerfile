# syntax=docker/dockerfile:1.7

ARG UV_VERSION=0.11.3

# Stage usato solamente per recuperare uv.
FROM ghcr.io/astral-sh/uv:${UV_VERSION} AS uv

# CUDA 12.9, cuDNN e compilatore nvcc per ARM64.
FROM nvidia/cuda:12.9.2-cudnn-devel-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    CUDA_HOME=/usr/local/cuda \
    CUDA_PATH=/usr/local/cuda \
    VIRTUAL_ENV=/opt/mimic-video-venv \
    UV_PROJECT_ENVIRONMENT=/opt/mimic-video-venv \
    UV_LINK_MODE=copy \
    UV_PYTHON=3.10 \
    PATH="/opt/mimic-video-venv/bin:/usr/local/cuda/bin:${PATH}" \
    TORCH_CUDA_ARCH_LIST="12.0+PTX" \
    FLASH_ATTN_CUDA_ARCHS="120" \
    NVTE_CUDA_ARCHS="120" \
    NVTE_FRAMEWORK=pytorch \
    MAX_JOBS=4 \
    NVCC_THREADS=2

# Dipendenze Linux necessarie per compilazione,
# immagini, video e pacchetti Python.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    ffmpeg \
    git \
    libgl1 \
    libglib2.0-0 \
    libjpeg-dev \
    libpng-dev \
    ninja-build \
    pkg-config \
    zlib1g-dev \
    && rm -rf /var/lib/apt/lists/*

# Installa uv nello stage CUDA finale.
COPY --from=uv /uv /uvx /usr/local/bin/

# Installa Python 3.10 e crea l'ambiente virtuale.
RUN uv python install 3.10 && \
    uv venv "${VIRTUAL_ENV}" --python 3.10

# Strumenti Python necessari per compilare le estensioni.
RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install \
        --python "${VIRTUAL_ENV}/bin/python" \
        setuptools==80.9.0 \
        wheel \
        packaging \
        ninja \
        cmake \
        pybind11 \
        psutil \
        pip

# PyTorch CUDA 12.9 per ARM64.
RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install \
        --python "${VIRTUAL_ENV}/bin/python" \
        torch==2.8.0 \
        torchvision==0.23.0 \
        --index-url https://download.pytorch.org/whl/cu129

# Triton richiesto dall'extra cu129 di Mimic Video.
RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install \
        --python "${VIRTUAL_ENV}/bin/python" \
        triton==3.4.0 \
        --index-url https://download.pytorch.org/whl

# Copia il codice ufficiale Mimic Video.
COPY external/mimic-video /workspace/mimic-video

WORKDIR /workspace/mimic-video/model

# Installa il progetto e tutte le dipendenze Python normali
# dichiarate nella sezione project.dependencies del pyproject.toml.
RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install \
        --python "${VIRTUAL_ENV}/bin/python" \
        --editable .

# Compila NVIDIA Apex contro PyTorch cu129 e CUDA 12.9.
RUN --mount=type=cache,target=/root/.cache/uv \
    APEX_CPP_EXT=1 \
    APEX_CUDA_EXT=1 \
    uv pip install \
        --python "${VIRTUAL_ENV}/bin/python" \
        --no-build-isolation \
        "git+https://github.com/NVIDIA/apex.git@12ec844e01121ef615eb6fe68a28715af5f33105"

# Compila FlashAttention 2.8.3 per Blackwell sm_120.
RUN --mount=type=cache,target=/root/.cache/uv \
    FLASH_ATTENTION_FORCE_BUILD=TRUE \
    uv pip install \
        --python "${VIRTUAL_ENV}/bin/python" \
        --no-build-isolation \
        --no-deps \
        flash-attn==2.8.3

# Compila Transformer Engine 1.13 con supporto PyTorch.
RUN --mount=type=cache,target=/root/.cache/uv \
    NVTE_FRAMEWORK=pytorch \
    uv pip install \
        --python "${VIRTUAL_ENV}/bin/python" \
        --no-build-isolation \
        "git+https://github.com/NVIDIA/TransformerEngine.git@v1.13"

# Controlla eventuali dipendenze mancanti o incompatibili.
RUN uv pip check \
    --python "${VIRTUAL_ENV}/bin/python"

WORKDIR /workspace/mimic-video/model

CMD ["/bin/bash"]