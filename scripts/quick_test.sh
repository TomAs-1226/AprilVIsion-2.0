#!/bin/bash
#===============================================================================
# Quick Test Script - Test camera streaming with single camera
#
# Usage: ./scripts/quick_test.sh
#
# This runs the vision system with minimal configuration for testing.
#===============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"

echo "======================================"
echo "FRC Vision Quick Test"
echo "======================================"
echo ""

# Check if built
if [[ ! -f "${BUILD_DIR}/frc_vision" ]]; then
    echo "Build not found. Building first..."
    mkdir -p "${BUILD_DIR}"
    cd "${BUILD_DIR}"
    cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
    ninja -j$(nproc)
    cd "${SCRIPT_DIR}"
fi

# List available cameras
echo "Available cameras:"
ls -la /dev/video* 2>/dev/null || echo "No video devices found!"
echo ""

# Use main config (already configured for single camera)
CONFIG="${SCRIPT_DIR}/config/config.yml"

echo "Using config: $CONFIG"
echo ""
echo "Starting vision system..."
echo "Dashboard: http://localhost:5800"
echo "MJPEG stream: http://localhost:5800/cam0.mjpeg"
echo "Snapshot: http://localhost:5800/cam0.jpg"
echo "Debug test: http://localhost:5800/test/stream"
echo ""
echo "Press Ctrl+C to stop"
echo "======================================"
echo ""

# Run with verbose output
"${BUILD_DIR}/frc_vision" "$CONFIG"
