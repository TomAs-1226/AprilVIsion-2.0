#!/bin/bash
#
# Fetch official WPILib AprilTag field layout
#
# Downloads the official 2024 CRESCENDO field layout from the WPILib repository
# and places it in the assets directory.
#
# Usage:
#   ./scripts/fetch_layout.sh
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
ASSETS_DIR="$PROJECT_DIR/assets"

# URLs for field layouts
CRESCENDO_2024_URL="https://raw.githubusercontent.com/wpilibsuite/allwpilib/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2024-crescendo.json"

echo "========================================"
echo "FRC AprilTag Field Layout Downloader"
echo "========================================"
echo ""

# Create assets directory if needed
mkdir -p "$ASSETS_DIR"

# Download 2024 Crescendo layout
echo "Downloading 2024 CRESCENDO field layout..."

if command -v curl &>/dev/null; then
    curl -fsSL "$CRESCENDO_2024_URL" -o "$ASSETS_DIR/2024-crescendo.json"
elif command -v wget &>/dev/null; then
    wget -q "$CRESCENDO_2024_URL" -O "$ASSETS_DIR/2024-crescendo.json"
else
    echo "Error: Neither curl nor wget found. Please install one of them."
    exit 1
fi

# Verify download
if [[ -f "$ASSETS_DIR/2024-crescendo.json" ]]; then
    echo ""
    echo "Successfully downloaded to: assets/2024-crescendo.json"

    # Count tags
    TAG_COUNT=$(grep -c '"ID"' "$ASSETS_DIR/2024-crescendo.json" || echo "?")
    echo "Field layout contains $TAG_COUNT AprilTags"
else
    echo "Error: Download failed"
    exit 1
fi

echo ""
echo "Done!"
