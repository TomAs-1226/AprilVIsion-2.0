#!/bin/bash
#
# FRC Vision Mac Simulator - Build Script
#
# Usage:
#   ./scripts/build_mac.sh [debug|release]
#
# Default: release build
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"

# Parse arguments
BUILD_TYPE="Release"
if [[ "$1" == "debug" ]]; then
    BUILD_TYPE="Debug"
fi

echo "========================================"
echo "FRC Vision Mac Simulator Build"
echo "Build type: $BUILD_TYPE"
echo "========================================"
echo ""

# Check if dependencies are installed
if ! command -v cmake &>/dev/null; then
    echo "CMake not found. Run: ./scripts/install_mac.sh"
    exit 1
fi

if ! pkg-config --exists opencv4; then
    echo "OpenCV not found. Run: ./scripts/install_mac.sh"
    exit 1
fi

# Create build directory
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Get number of CPU cores
NPROC=$(sysctl -n hw.ncpu)

# Configure
echo "Configuring..."
cmake .. \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DBUILD_MAC_SIM=ON \
    -DBUILD_ORANGEPI=OFF

echo ""

# Build
echo "Building with $NPROC parallel jobs..."
make -j"$NPROC"

echo ""
echo "========================================"
echo "Build Complete!"
echo "========================================"
echo ""
echo "Run the simulator with:"
echo ""
echo "  $BUILD_DIR/frc_vision_sim"
echo ""
echo "Or use:"
echo ""
echo "  ./scripts/run_sim.sh"
echo ""
