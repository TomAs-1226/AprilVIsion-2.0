#!/bin/bash
# FRC Vision Coprocessor - Run Script
#
# Usage: ./scripts/run.sh [config_path]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"
CONFIG_PATH="${1:-$BUILD_DIR/config/config.yml}"

# Check if build exists
if [ ! -f "$BUILD_DIR/frc_vision" ]; then
    echo "Binary not found. Building..."
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
    ninja
fi

# Check cameras
echo "Checking cameras..."
v4l2-ctl --list-devices 2>/dev/null || echo "No cameras found (v4l2-ctl not available)"
echo ""

# Set real-time priority if possible
NICE=""
if [ "$EUID" -eq 0 ] || which chrt >/dev/null 2>&1; then
    NICE="chrt -f 10"
fi

# Run the coprocessor
echo "Starting FRC Vision Coprocessor..."
echo "Config: $CONFIG_PATH"
echo "Press Ctrl+C to stop"
echo ""

cd "$BUILD_DIR"
exec $NICE ./frc_vision "$CONFIG_PATH"
