# Docker Development Environment

This directory contains Docker configuration for VI-SLAM development.

## Quick Start

```bash
# Build and start the development container
cd docker
docker-compose up -d

# Enter the container
docker-compose exec vi-slam-dev bash

# Inside the container, build the PC client
cd /workspace/pc_client
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

## Container Contents

- Ubuntu 22.04 base image
- Build tools: GCC, CMake, Git
- Dependencies: OpenCV, Eigen3, ZeroMQ, Ceres Solver
- Python 3 with virtual environment

## Stopping the Container

```bash
docker-compose down
```

## Rebuilding After Changes

```bash
docker-compose build --no-cache
docker-compose up -d
```
