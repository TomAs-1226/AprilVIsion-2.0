#!/bin/bash
#===============================================================================
# Camera Calibration Script
#
# This script helps calibrate your camera using a ChArUco board.
#
# Steps:
# 1. Print the calibration board: http://localhost:5800/api/calibration/board
# 2. Run this script
# 3. Show the board to the camera from different angles
# 4. Press 'c' to capture each frame, 'q' when done
#
# Requirements: The vision system must be running (./setup.sh or ./scripts/quick_test.sh)
#===============================================================================

set -e

PORT=${1:-5800}
CAMERA=${2:-0}
OUTPUT=${3:-config/cam${CAMERA}_intrinsics.yml}

echo "======================================"
echo "FRC Vision Camera Calibration"
echo "======================================"
echo ""
echo "Camera: $CAMERA"
echo "Output: $OUTPUT"
echo "Server: http://localhost:$PORT"
echo ""

# Check if server is running
if ! curl -s "http://localhost:$PORT/health" > /dev/null 2>&1; then
    echo "ERROR: Vision server not running!"
    echo "Start it with: ./scripts/quick_test.sh"
    exit 1
fi

# Print the calibration board
echo "Step 1: Print the calibration board"
echo "  Download: http://localhost:$PORT/api/calibration/board"
echo "  Or use: curl http://localhost:$PORT/api/calibration/board > charuco_board.png"
echo ""

# Start calibration
echo "Step 2: Starting calibration session..."
RESPONSE=$(curl -s -X POST "http://localhost:$PORT/api/calibration/start" \
    -H "Content-Type: application/json" \
    -d '{"squares_x": 5, "squares_y": 7, "square_length": 0.04, "marker_length": 0.02}')

echo "  $RESPONSE"
echo ""

MIN_FRAMES=$(echo "$RESPONSE" | grep -o '"min_frames":[0-9]*' | cut -d: -f2)
RECOMMENDED=$(echo "$RESPONSE" | grep -o '"recommended_frames":[0-9]*' | cut -d: -f2)

echo "Step 3: Capture calibration frames"
echo "  Minimum frames needed: $MIN_FRAMES"
echo "  Recommended frames: $RECOMMENDED"
echo ""
echo "  Instructions:"
echo "  - Hold the ChArUco board in front of the camera"
echo "  - Move to different positions and angles"
echo "  - Cover the entire frame area"
echo "  - Tilt the board at various angles"
echo ""

FRAMES=0
while true; do
    echo -n "Press Enter to capture frame (or 'q' to finish): "
    read -r input

    if [[ "$input" == "q" || "$input" == "Q" ]]; then
        break
    fi

    RESPONSE=$(curl -s -X POST "http://localhost:$PORT/api/calibration/capture" \
        -H "Content-Type: application/json" \
        -d "{\"camera_id\": $CAMERA}")

    SUCCESS=$(echo "$RESPONSE" | grep -o '"success":[a-z]*' | cut -d: -f2)
    FRAMES=$(echo "$RESPONSE" | grep -o '"frames_captured":[0-9]*' | cut -d: -f2)

    if [[ "$SUCCESS" == "true" ]]; then
        echo "  ✓ Frame $FRAMES captured"
    else
        echo "  ✗ Frame rejected (board not detected clearly)"
    fi

    if [[ "$FRAMES" -ge "$RECOMMENDED" ]]; then
        echo ""
        echo "  You have enough frames for a good calibration!"
        echo -n "  Continue capturing or press 'q' to compute: "
    fi
done

echo ""

if [[ "$FRAMES" -lt "$MIN_FRAMES" ]]; then
    echo "ERROR: Not enough frames captured ($FRAMES < $MIN_FRAMES)"
    exit 1
fi

echo "Step 4: Computing calibration..."
RESPONSE=$(curl -s -X POST "http://localhost:$PORT/api/calibration/compute" \
    -H "Content-Type: application/json" \
    -d "{\"save_path\": \"$OUTPUT\"}")

echo "$RESPONSE" | python3 -m json.tool 2>/dev/null || echo "$RESPONSE"

SUCCESS=$(echo "$RESPONSE" | grep -o '"success":[a-z]*' | cut -d: -f2)

if [[ "$SUCCESS" == "true" ]]; then
    echo ""
    echo "======================================"
    echo "Calibration Complete!"
    echo "======================================"
    echo "Saved to: $OUTPUT"
    echo ""
    echo "Restart the vision system to use the new calibration."
else
    echo ""
    echo "ERROR: Calibration failed!"
    exit 1
fi
