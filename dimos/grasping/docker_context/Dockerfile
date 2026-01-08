# GraspGen Docker Service
# Standalone container for running GraspGen inference
#
# Build:
#   cd dimos/dimos/grasping/docker_context
#   docker build -t dimos-graspgen .
#
# Run:
#   docker run --gpus all -p 8094:8094 --name dimos_graspgen dimos-graspgen

FROM nvidia/cuda:12.8.1-cudnn-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

# Install system dependencies
RUN apt-get update && apt-get install -y \
    wget \
    curl \
    git \
    build-essential \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    libglu1-mesa \
    libglu1-mesa-dev \
    libegl1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Miniconda
ENV CONDA_DIR=/opt/conda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p $CONDA_DIR && \
    rm ~/miniconda.sh
ENV PATH=$CONDA_DIR/bin:$PATH

# Initialize conda
RUN conda init bash

# Clone GraspGen repository
WORKDIR /workspace/third_party
RUN git clone https://github.com/NVlabs/GraspGen.git && \
    cd GraspGen && \
    git checkout main

# Copy pre-downloaded model checkpoints (downloaded locally to avoid network issues during build)
# Run this first: git clone https://huggingface.co/adithyamurali/GraspGenModels
COPY GraspGenModels/checkpoints /workspace/third_party/GraspGen/checkpoints

# Accept Conda Terms of Service for non-interactive build
# This is required by Anaconda since 2024
RUN /bin/bash -c "source /opt/conda/etc/profile.d/conda.sh && \
    conda config --set channel_priority flexible && \
    echo 'yes' | conda tos accept --override-channels --channel defaults 2>/dev/null || true"

# Create conda environment
RUN conda create -n graspgen python=3.10 -y && \
    conda clean -afy

WORKDIR /workspace/third_party/GraspGen

# Install PyTorch 2.7.0 with CUDA 12.8 for RTX 5060 (sm_120) support
# PyTorch 2.7.0 officially added Blackwell architecture support
RUN conda run -n graspgen pip install --no-cache-dir \
    torch==2.7.0 torchvision==0.22.0 \
    --index-url https://download.pytorch.org/whl/cu128

# Install GraspGen package
RUN conda run -n graspgen pip install --no-cache-dir -e .

# Build PointNet++ C++ extensions with sm_120 support
# PyTorch 2.7.0 + CUDA 12.8 natively support Blackwell (sm_120)
# Include architectures: Ampere (8.0, 8.6), Ada (8.9), Hopper (9.0), Blackwell (12.0)
RUN conda run -n graspgen bash -c "\
    export TORCH_CUDA_ARCH_LIST='8.0 8.6 8.9 9.0 12.0' && \
    export FORCE_CUDA=1 && \
    export CUDA_HOME=/usr/local/cuda && \
    cd pointnet2_ops && \
    pip install --no-build-isolation ."

# Install torch-scatter and torch-cluster for PyTorch 2.7.0
# Build from source with --no-build-isolation to access installed torch
RUN conda run -n graspgen pip install --no-cache-dir --no-build-isolation torch-scatter torch-cluster

# Install FastAPI and dependencies for the service wrapper
RUN conda run -n graspgen pip install --no-cache-dir \
    fastapi==0.109.0 \
    uvicorn[standard]==0.27.0 \
    pydantic==2.5.3 \
    numpy \
    trimesh \
    pillow \
    pyrender \
    imageio \
    scipy

# Copy service code
WORKDIR /workspace
COPY main.py /workspace/main.py

# Create directories
RUN mkdir -p /workspace/logs /workspace/debug

# Environment variables
ENV GRASPGEN_PATH=/workspace/third_party/GraspGen
ENV DEFAULT_GRIPPER=robotiq_2f_140
ENV LOG_DIR=/workspace/logs
ENV PYOPENGL_PLATFORM=egl
ENV PYTHONPATH=/workspace:/workspace/third_party

# CUDA runtime environment for PyTorch 2.7.0 with sm_120 support
ENV CUDA_LAUNCH_BLOCKING=0
ENV TORCH_CUDA_ARCH_LIST="8.0 8.6 8.9 9.0 12.0"
ENV PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True

# Expose port
EXPOSE 8094

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=60s --retries=3 \
    CMD curl -f http://localhost:8094/health || exit 1

# Run the service
CMD ["conda", "run", "--no-capture-output", "-n", "graspgen", "python", "-u", "/workspace/main.py"]
