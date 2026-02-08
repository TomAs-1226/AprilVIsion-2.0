#!/bin/bash
#===============================================================================
# Camera Detection Script
# Lists all available video devices and their capabilities
#===============================================================================

echo "======================================"
echo "FRC Vision Camera Detection"
echo "======================================"
echo ""

echo "Video devices found:"
echo "--------------------"
for dev in /dev/video*; do
    if [[ -e "$dev" ]]; then
        # Get device info using v4l2-ctl if available
        if command -v v4l2-ctl &> /dev/null; then
            name=$(v4l2-ctl --device="$dev" --info 2>/dev/null | grep "Card type" | cut -d: -f2 | xargs)
            caps=$(v4l2-ctl --device="$dev" --info 2>/dev/null | grep "Device Caps" | cut -d: -f2 | xargs)

            if [[ -n "$name" ]]; then
                echo "$dev: $name"
                # Check if it's a capture device
                if [[ "$caps" == *"Video Capture"* ]]; then
                    echo "    [CAPTURE DEVICE - Use this one]"
                else
                    echo "    [Metadata/other - skip]"
                fi
            else
                echo "$dev: (unable to query)"
            fi
        else
            echo "$dev: (install v4l-utils for details)"
        fi
    fi
done

echo ""
echo "--------------------"
echo ""

# Count actual capture devices
capture_count=0
capture_devices=""
for dev in /dev/video*; do
    if [[ -e "$dev" ]] && command -v v4l2-ctl &> /dev/null; then
        caps=$(v4l2-ctl --device="$dev" --info 2>/dev/null | grep "Device Caps" | cut -d: -f2 | xargs)
        if [[ "$caps" == *"Video Capture"* ]]; then
            capture_count=$((capture_count + 1))
            capture_devices="$capture_devices $dev"
        fi
    fi
done

if [[ $capture_count -gt 0 ]]; then
    echo "Found $capture_count capture device(s):$capture_devices"
    echo ""
    echo "Recommended config/config.yml cameras section:"
    echo "-----------------------------------------------"

    idx=0
    for dev in $capture_devices; do
        dev_num=$(echo "$dev" | grep -o '[0-9]*$')
        case $idx in
            0) name="front"; yaw="0.0" ;;
            1) name="left"; yaw="90.0" ;;
            2) name="right"; yaw="-90.0" ;;
            *) name="cam$idx"; yaw="0.0" ;;
        esac

        echo "  - name: $name"
        echo "    device: $dev"
        echo "    width: 640"
        echo "    height: 480"
        echo "    fps: 30"
        echo "    exposure: -1"
        echo "    gain: -1"
        echo "    format: MJPG"
        echo "    intrinsics: cam${idx}_intrinsics.yml"
        echo "    extrinsics:"
        echo "      x: 0.0"
        echo "      y: 0.0"
        echo "      z: 0.50"
        echo "      roll: 0.0"
        echo "      pitch: 0.0"
        echo "      yaw: $yaw"
        echo ""

        idx=$((idx + 1))
    done
else
    echo "No capture devices found."
    echo "Make sure cameras are plugged in and run:"
    echo "  sudo apt install v4l-utils"
fi
