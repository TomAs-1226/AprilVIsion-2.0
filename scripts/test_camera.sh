#!/bin/bash
# Quick camera test script

echo "=== Camera Debug ==="
echo ""

echo "1. Listing video devices:"
ls -la /dev/video* 2>/dev/null || echo "No video devices found!"
echo ""

echo "2. V4L2 device list:"
v4l2-ctl --list-devices 2>/dev/null || echo "v4l2-ctl not found"
echo ""

echo "3. Testing /dev/video0:"
if [ -e /dev/video0 ]; then
    echo "Device exists. Checking capabilities..."
    v4l2-ctl -d /dev/video0 --all 2>&1 | head -30
    echo ""
    echo "Supported formats:"
    v4l2-ctl -d /dev/video0 --list-formats-ext 2>&1 | head -40
else
    echo "/dev/video0 does not exist!"
fi
echo ""

echo "4. Quick capture test (5 frames):"
if [ -e /dev/video0 ]; then
    timeout 5 v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=5 2>&1 || echo "Capture test failed or timed out"
fi
echo ""

echo "5. Check if frc_vision service is running:"
systemctl status frc_vision --no-pager 2>/dev/null | head -10
echo ""

echo "6. Check frc_vision logs for errors:"
journalctl -u frc_vision --no-pager -n 20 2>/dev/null | grep -i "error\|fail\|camera\|video"
echo ""

echo "=== End Debug ==="
