FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    python3 \
    python3-pip \
    cmake \
    wget \
    ca-certificates \
    gcc-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    gdb-multiarch \
    openocd \
    binutils-multiarch \
    clang-format \
    clang-tidy \
    && pip3 install --no-cache-dir pre-commit \
    && rm -rf /var/lib/apt/lists/*
