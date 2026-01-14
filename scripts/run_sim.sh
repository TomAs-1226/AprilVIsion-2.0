#!/bin/bash
#
# FRC Vision Mac Simulator - Run Script
#
# Usage:
#   ./scripts/run_sim.sh [options]
#
# Options:
#   --no-webcam    Disable webcam (synthetic only)
#   --config PATH  Use custom config file
#   --help         Show help
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"
EXECUTABLE="$BUILD_DIR/frc_vision_sim"

# Check if executable exists
if [[ ! -f "$EXECUTABLE" ]]; then
    echo "Simulator not built. Building now..."
    "$SCRIPT_DIR/build_mac.sh"
fi

# Run simulator with passed arguments
echo "Starting FRC Vision Mac Simulator..."
echo ""

cd "$BUILD_DIR"
exec ./frc_vision_sim "$@"
