#!/bin/bash
# FRC Vision Coprocessor - Dependency Installation Script
# For Orange Pi 5 (RK3588S) running Ubuntu-Rockchip (Joshua Riek)
#
# Usage: sudo ./scripts/install_deps.sh

set -e

echo "=========================================="
echo "FRC Vision Coprocessor - Dependency Setup"
echo "Orange Pi 5 (RK3588S) / Ubuntu-Rockchip"
echo "=========================================="

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root: sudo ./scripts/install_deps.sh"
    exit 1
fi

# Update package lists
echo "[1/6] Updating package lists..."
apt-get update

# Install build essentials
echo "[2/6] Installing build tools..."
apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    ninja-build

# Install OpenCV
echo "[3/6] Installing OpenCV..."
apt-get install -y \
    libopencv-dev \
    libopencv-videoio-dev \
    libopencv-imgcodecs-dev \
    libopencv-imgproc-dev \
    libopencv-calib3d-dev

# Install YAML-CPP
echo "[4/6] Installing yaml-cpp..."
apt-get install -y libyaml-cpp-dev

# Install V4L2 utilities (for camera debugging)
echo "[5/6] Installing camera utilities..."
apt-get install -y \
    v4l-utils \
    libv4l-dev

# Setup udev rules for camera permissions
echo "[6/6] Setting up camera permissions..."

# Create udev rule for video devices
cat > /etc/udev/rules.d/99-frc-vision.rules << 'EOF'
# FRC Vision Coprocessor - Camera Permissions
# Allow access to video devices without root

# USB cameras
KERNEL=="video[0-9]*", SUBSYSTEM=="video4linux", MODE="0666"
KERNEL=="video[0-9]*", SUBSYSTEM=="video4linux", GROUP="video"

# Grant access to plugdev group as well
KERNEL=="video[0-9]*", SUBSYSTEM=="video4linux", GROUP="plugdev", MODE="0666"
EOF

# Reload udev rules
udevadm control --reload-rules
udevadm trigger

# Add current user to video group
ACTUAL_USER="${SUDO_USER:-$USER}"
if [ -n "$ACTUAL_USER" ] && [ "$ACTUAL_USER" != "root" ]; then
    usermod -a -G video "$ACTUAL_USER"
    usermod -a -G plugdev "$ACTUAL_USER"
    echo "Added user '$ACTUAL_USER' to video and plugdev groups"
    echo "NOTE: Log out and back in for group changes to take effect"
fi

# Increase USB buffer size for better camera performance
echo "[*] Optimizing USB buffer size..."
if ! grep -q "usbcore.usbfs_memory_mb" /etc/default/grub 2>/dev/null; then
    echo 'GRUB_CMDLINE_LINUX_DEFAULT="$GRUB_CMDLINE_LINUX_DEFAULT usbcore.usbfs_memory_mb=1000"' >> /etc/default/grub
    echo "USB buffer size increased (will take effect after reboot)"
fi

# Set memory limits for real-time performance
cat > /etc/security/limits.d/frc-vision.conf << 'EOF'
# FRC Vision Coprocessor - Resource Limits
*               soft    memlock         unlimited
*               hard    memlock         unlimited
*               soft    rtprio          99
*               hard    rtprio          99
EOF

# Create installation directory
mkdir -p /opt/frc-vision
mkdir -p /etc/frc_vision

echo ""
echo "=========================================="
echo "Dependency installation complete!"
echo ""
echo "Next steps:"
echo "  1. Log out and log back in (for group permissions)"
echo "  2. Build the project:"
echo "     mkdir build && cd build"
echo "     cmake .. -GNinja"
echo "     ninja"
echo "  3. Run the coprocessor:"
echo "     ./frc_vision"
echo ""
echo "To install as a service:"
echo "  sudo cp deploy/frc_vision.service /etc/systemd/system/"
echo "  sudo systemctl enable frc_vision"
echo "  sudo systemctl start frc_vision"
echo "=========================================="
