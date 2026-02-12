#!/bin/bash
#===============================================================================
# AprilVision 3.2 - Camera Detection Utility
#
# Lists all connected video devices and their capabilities.
# Use this to verify cameras are detected before configuring in the dashboard.
#
# Usage: ./scripts/detect_cameras.sh
#===============================================================================

CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo ""
echo -e "${CYAN}AprilVision 3.2 - Camera Detection${NC}"
echo "==================================="
echo ""

# Check if v4l-utils is installed
if ! command -v v4l2-ctl &> /dev/null; then
    echo -e "${YELLOW}v4l2-ctl not found. Install with: sudo apt-get install v4l-utils${NC}"
    exit 1
fi

# Count devices
CAM_COUNT=$(ls /dev/video* 2>/dev/null | wc -l)
echo -e "Video Devices Found: ${GREEN}${CAM_COUNT}${NC}"
echo "--------------"

# List all video devices
v4l2-ctl --list-devices 2>/dev/null || echo "No video devices found"

echo ""
echo "Device Details:"
echo "---------------"

for dev in /dev/video*; do
    if [[ -e "$dev" ]]; then
        echo ""
        echo "  $dev:"
        # Get device name
        name=$(v4l2-ctl -d "$dev" --info 2>/dev/null | grep "Card type" | sed 's/.*: //')
        if [[ -n "$name" ]]; then
            echo "    Name: $name"
        fi
        # Get supported formats
        formats=$(v4l2-ctl -d "$dev" --list-formats-ext 2>/dev/null | grep -E "Size|Interval" | head -6)
        if [[ -n "$formats" ]]; then
            echo "    Formats:"
            echo "$formats" | sed 's/^/      /'
        fi
    fi
done

echo ""
echo -e "${GREEN}Tip:${NC} Configure cameras through the AprilVision dashboard at :5801"
echo "     Camera names in the dashboard must match your robot code."
echo "     Default names: \"front\", \"left\", \"right\""
echo ""
