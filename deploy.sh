#!/bin/bash
# =============================================================================
# AprilVision 3.2 - Deploy Script
# Copies bridge + dashboard files to /opt/aprilvision and restarts the service
#
# Usage:
#   sudo ./deploy.sh            # Deploy and restart service
#   sudo ./deploy.sh --no-restart  # Deploy files only
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="/opt/aprilvision"
SERVICE_NAME="aprilvision-dashboard"

echo "=============================================="
echo "  AprilVision 3.2 - Deploy"
echo "  Source: $SCRIPT_DIR"
echo "  Target: $INSTALL_DIR"
echo "=============================================="

# Check for root
if [ "$EUID" -ne 0 ]; then
    echo "ERROR: Please run with sudo"
    echo "  sudo $0 $*"
    exit 1
fi

# Create install directory
mkdir -p "$INSTALL_DIR/web"
mkdir -p "$INSTALL_DIR/snapshots"

# Copy bridge script
echo "[1/4] Copying bridge server..."
cp "$SCRIPT_DIR/scripts/aprilvision-bridge.py" "$INSTALL_DIR/bridge.py"
chmod +x "$INSTALL_DIR/bridge.py"

# Copy web files
echo "[2/4] Copying dashboard files..."
cp -r "$SCRIPT_DIR/web/"* "$INSTALL_DIR/web/"

# Install systemd service
echo "[3/4] Installing systemd service..."
cp "$SCRIPT_DIR/deploy/aprilvision-dashboard.service" /etc/systemd/system/
systemctl daemon-reload
systemctl enable "$SERVICE_NAME" 2>/dev/null || true

# Restart unless --no-restart
if [ "$1" != "--no-restart" ]; then
    echo "[4/4] Restarting service..."
    systemctl restart "$SERVICE_NAME" 2>/dev/null || true
    sleep 1
    if systemctl is-active --quiet "$SERVICE_NAME"; then
        echo ""
        echo "  Service is RUNNING"
        echo "  Dashboard: http://$(hostname -I | awk '{print $1}'):5801"
    else
        echo ""
        echo "  WARNING: Service did not start. Check logs:"
        echo "    journalctl -u $SERVICE_NAME -n 20"
    fi
else
    echo "[4/4] Skipping restart (--no-restart)"
fi

echo ""
echo "  Deploy complete!"
echo "  Files installed to: $INSTALL_DIR"
echo ""
echo "  Useful commands:"
echo "    sudo systemctl status $SERVICE_NAME"
echo "    sudo systemctl restart $SERVICE_NAME"
echo "    journalctl -u $SERVICE_NAME -f"
echo "=============================================="
