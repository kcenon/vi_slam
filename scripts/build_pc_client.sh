#!/bin/bash
# Build script for PC client

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
PC_CLIENT_DIR="$PROJECT_ROOT/pc_client"
BUILD_DIR="$PC_CLIENT_DIR/build"

# Build type (default: Release)
BUILD_TYPE="${1:-Release}"

echo "Building PC client ($BUILD_TYPE)..."

# Create build directory
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure
cmake -DCMAKE_BUILD_TYPE="$BUILD_TYPE" ..

# Build
cmake --build . --config "$BUILD_TYPE" -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

echo ""
echo "Build completed successfully!"
echo "Binary location: $BUILD_DIR/vi_slam_pc_client"
