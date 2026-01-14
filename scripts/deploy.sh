#!/bin/bash
# FRC Vision Coprocessor - Deployment Script
# Installs the built binary and config to /opt/frc-vision
#
# Usage: sudo ./scripts/deploy.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"
INSTALL_DIR="/opt/frc-vision"

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root: sudo ./scripts/deploy.sh"
    exit 1
fi

# Check if build exists
if [ ! -f "$BUILD_DIR/frc_vision" ]; then
    echo "Binary not found. Please build first:"
    echo "  mkdir build && cd build && cmake .. -GNinja && ninja"
    exit 1
fi

echo "Deploying FRC Vision Coprocessor..."

# Stop service if running
if systemctl is-active --quiet frc_vision; then
    echo "Stopping existing service..."
    systemctl stop frc_vision
fi

# Create install directory
mkdir -p "$INSTALL_DIR"
mkdir -p "$INSTALL_DIR/config"
mkdir -p "$INSTALL_DIR/web"

# Copy binary
echo "Installing binary..."
cp "$BUILD_DIR/frc_vision" "$INSTALL_DIR/"
chmod +x "$INSTALL_DIR/frc_vision"

# Copy config files
echo "Installing configuration..."
cp -r "$BUILD_DIR/config/"* "$INSTALL_DIR/config/" 2>/dev/null || \
cp -r "$PROJECT_DIR/config/"* "$INSTALL_DIR/config/"

# Copy web files
echo "Installing web assets..."
cp -r "$BUILD_DIR/web/"* "$INSTALL_DIR/web/" 2>/dev/null || \
cp -r "$PROJECT_DIR/web/"* "$INSTALL_DIR/web/"

# Install systemd service
echo "Installing systemd service..."
cp "$PROJECT_DIR/deploy/frc_vision.service" /etc/systemd/system/
systemctl daemon-reload

# Enable service
systemctl enable frc_vision

echo ""
echo "=========================================="
echo "Deployment complete!"
echo ""
echo "Installed to: $INSTALL_DIR"
echo ""
echo "Commands:"
echo "  Start:   sudo systemctl start frc_vision"
echo "  Stop:    sudo systemctl stop frc_vision"
echo "  Restart: sudo systemctl restart frc_vision"
echo "  Status:  sudo systemctl status frc_vision"
echo "  Logs:    sudo journalctl -u frc_vision -f"
echo ""
echo "Edit config: $INSTALL_DIR/config/config.yml"
echo "Dashboard:   http://$(hostname -I | awk '{print $1}'):5800"
echo "=========================================="
