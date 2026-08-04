# syntax=docker/dockerfile:1.7

# Ambiente NVIDIA ufficiale per DGX Spark:
# - Ubuntu 24.04
# - CUDA 13.0.1
# - strumenti di sviluppo CUDA
FROM nvcr.io/nvidia/cuda:13.0.1-devel-ubuntu24.04

ARG UV_VERSION=0.11.3

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    CUDA_HOME=/usr/local/cuda \
    UV_PROJECT_ENVIRONMENT=/opt/mimic-video-venv \
    UV_LINK_MODE=copy \
    UV_PYTHON=3.10 \
    PATH="/opt/mimic-video-venv/bin:/usr/local/cuda/bin:${PATH}"

# Dipendenze Linux necessarie per:
# - compilare eventuali estensioni CUDA/Python;
# - elaborare immagini e video;
# - installare Mimic Video.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    ffmpeg \
    git \
    libgl1 \
    libglib2.0-0 \
    ninja-build \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# Installa uv dalla sua immagine ufficiale.
# Docker seleziona automaticamente la variante ARM64.
COPY --from=ghcr.io/astral-sh/uv:${UV_VERSION} \
    /uv \
    /uvx \
    /usr/local/bin/

# Installa Python 3.10 tramite uv.
RUN uv python install 3.10

# Directory del progetto Mimic Video nel container.
WORKDIR /workspace/mimic-video/model

# Copia prima solo il pyproject.toml.
# Questo permette a Docker di riutilizzare la cache delle dipendenze.
COPY external/mimic-video/model/pyproject.toml ./pyproject.toml

# Installa le dipendenze della configurazione Blackwell cu129.
# Non installa ancora il package locale perché il codice non è stato copiato.
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync \
        --python 3.10 \
        --extra cu129 \
        --no-install-project

# Copia tutta la repository ufficiale presente come submodule.
COPY external/mimic-video /workspace/mimic-video

# Completa l'installazione del progetto Mimic Video.
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync \
        --python 3.10 \
        --extra cu129

WORKDIR /workspace/mimic-video/model

CMD ["/bin/bash"]