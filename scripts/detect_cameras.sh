#!/bin/bash
# AprilVision 2.0 - Detect available cameras
# Lists all video devices and their capabilities

echo "AprilVision 2.0 - Camera Detection"
echo "==================================="
echo ""

# Check if v4l-utils is installed
if ! command -v v4l2-ctl &> /dev/null; then
    echo "v4l2-ctl not found. Install with: sudo apt-get install v4l-utils"
    exit 1
fi

# List all video devices
echo "Video Devices:"
echo "--------------"
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
echo "Tip: Configure cameras through the PhotonVision web UI at :5801"
echo "     PhotonVision will auto-detect connected USB cameras."
