# syntax=docker/dockerfile:1.7

ARG UV_VERSION=0.11.3

# Stage usato solo per recuperare i binari di uv.
FROM ghcr.io/astral-sh/uv:${UV_VERSION} AS uv

# Stage finale in cui sarà installato Mimic Video.
FROM nvcr.io/nvidia/cuda:13.0.1-devel-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    CUDA_HOME=/usr/local/cuda \
    UV_PROJECT_ENVIRONMENT=/opt/mimic-video-venv \
    UV_LINK_MODE=copy \
    UV_PYTHON=3.10 \
    PATH="/opt/mimic-video-venv/bin:/usr/local/cuda/bin:${PATH}"

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

# Copia uv nello stage CUDA finale.
COPY --from=uv /uv /uvx /usr/local/bin/

RUN uv python install 3.10

WORKDIR /workspace/mimic-video/model

# Prima installa le dipendenze per sfruttare la cache Docker.
COPY external/mimic-video/model/pyproject.toml ./pyproject.toml

RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync \
        --python 3.10 \
        --extra cu129 \
        --no-install-project

# Poi copia tutto il codice ufficiale.
COPY external/mimic-video /workspace/mimic-video

RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync \
        --python 3.10 \
        --extra cu129

WORKDIR /workspace/mimic-video/model

CMD ["/bin/bash"]