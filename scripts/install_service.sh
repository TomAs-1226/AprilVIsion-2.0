#!/bin/bash
# =============================================================================
# FRC Vision Coprocessor - Service Installation Script
# =============================================================================
#
# This script:
# 1. Creates a dedicated 'frcvision' user
# 2. Sets up /dev/video* permissions via udev rules
# 3. Deploys the binary and config to /opt/frc-vision
# 4. Installs and enables the systemd service
#
# Usage: sudo ./scripts/install_service.sh
#
# After installation:
#   systemctl status frc_vision    # Check status
#   journalctl -u frc_vision -f    # View logs
#   curl http://localhost:5800/api/status  # Check readiness
#
# =============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check root
if [ "$EUID" -ne 0 ]; then
    log_error "Please run as root: sudo $0"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"
INSTALL_DIR="/opt/frc-vision"
SERVICE_NAME="frc_vision"
SERVICE_USER="frcvision"
SERVICE_GROUP="frcvision"

log_info "FRC Vision Coprocessor - Service Installation"
log_info "=============================================="

# =============================================================================
# Step 1: Create dedicated user
# =============================================================================
log_info "Creating service user '$SERVICE_USER'..."

if id "$SERVICE_USER" &>/dev/null; then
    log_info "User '$SERVICE_USER' already exists"
else
    useradd --system --no-create-home --shell /usr/sbin/nologin "$SERVICE_USER"
    log_info "Created system user '$SERVICE_USER'"
fi

# Add user to video group for camera access
usermod -a -G video "$SERVICE_USER" 2>/dev/null || true
usermod -a -G plugdev "$SERVICE_USER" 2>/dev/null || true

# =============================================================================
# Step 2: Setup udev rules for camera permissions
# =============================================================================
log_info "Setting up camera permissions (udev rules)..."

cat > /etc/udev/rules.d/99-frc-vision-cameras.rules << 'EOF'
# FRC Vision Coprocessor - Camera Access Rules
# Grant video device access to frcvision user and video group

# USB Video devices (UVC cameras)
KERNEL=="video[0-9]*", SUBSYSTEM=="video4linux", MODE="0666", GROUP="video"

# Specifically for the frcvision service user
KERNEL=="video[0-9]*", SUBSYSTEM=="video4linux", OWNER="frcvision", MODE="0660"

# Grant plugdev group access as well
KERNEL=="video[0-9]*", SUBSYSTEM=="video4linux", GROUP="plugdev", MODE="0666"
EOF

# Reload udev rules
udevadm control --reload-rules
udevadm trigger --subsystem-match=video4linux

log_info "Udev rules installed and reloaded"

# =============================================================================
# Step 3: Check for binary
# =============================================================================
if [ ! -f "$BUILD_DIR/frc_vision" ]; then
    log_warn "Binary not found at $BUILD_DIR/frc_vision"
    log_info "Building now..."

    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    cmake "$PROJECT_DIR" -GNinja -DCMAKE_BUILD_TYPE=Release
    ninja

    if [ ! -f "$BUILD_DIR/frc_vision" ]; then
        log_error "Build failed! Cannot continue."
        exit 1
    fi
fi

# =============================================================================
# Step 4: Stop existing service if running
# =============================================================================
if systemctl is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
    log_info "Stopping existing service..."
    systemctl stop "$SERVICE_NAME"
fi

# =============================================================================
# Step 5: Deploy files
# =============================================================================
log_info "Deploying to $INSTALL_DIR..."

mkdir -p "$INSTALL_DIR"
mkdir -p "$INSTALL_DIR/config"
mkdir -p "$INSTALL_DIR/web"

# Copy binary
cp "$BUILD_DIR/frc_vision" "$INSTALL_DIR/"
chmod +x "$INSTALL_DIR/frc_vision"

# Copy config files
if [ -d "$BUILD_DIR/config" ]; then
    cp -r "$BUILD_DIR/config/"* "$INSTALL_DIR/config/" 2>/dev/null || true
fi
if [ -d "$PROJECT_DIR/config" ]; then
    cp -r "$PROJECT_DIR/config/"* "$INSTALL_DIR/config/" 2>/dev/null || true
fi

# Copy web files
if [ -d "$BUILD_DIR/web" ]; then
    cp -r "$BUILD_DIR/web/"* "$INSTALL_DIR/web/" 2>/dev/null || true
fi
if [ -d "$PROJECT_DIR/web" ]; then
    cp -r "$PROJECT_DIR/web/"* "$INSTALL_DIR/web/" 2>/dev/null || true
fi

# Set ownership
chown -R "$SERVICE_USER:$SERVICE_GROUP" "$INSTALL_DIR"
chmod -R 755 "$INSTALL_DIR"

log_info "Files deployed to $INSTALL_DIR"

# =============================================================================
# Step 6: Install systemd service
# =============================================================================
log_info "Installing systemd service..."

cp "$PROJECT_DIR/deploy/frc_vision.service" /etc/systemd/system/

# Reload systemd
systemctl daemon-reload

# Enable service for boot
systemctl enable "$SERVICE_NAME"

log_info "Service installed and enabled"

# =============================================================================
# Step 7: Configure system limits
# =============================================================================
log_info "Setting system limits for real-time performance..."

cat > /etc/security/limits.d/99-frc-vision.conf << EOF
# FRC Vision Coprocessor - Resource Limits
$SERVICE_USER    soft    memlock         unlimited
$SERVICE_USER    hard    memlock         unlimited
$SERVICE_USER    soft    rtprio          99
$SERVICE_USER    hard    rtprio          99
$SERVICE_USER    soft    nofile          65536
$SERVICE_USER    hard    nofile          65536
EOF

# =============================================================================
# Step 8: Start service
# =============================================================================
log_info "Starting service..."

systemctl start "$SERVICE_NAME"

# Wait a moment and check status
sleep 2

if systemctl is-active --quiet "$SERVICE_NAME"; then
    log_info "Service started successfully!"
else
    log_warn "Service may have issues. Check: journalctl -u $SERVICE_NAME"
fi

# =============================================================================
# Done!
# =============================================================================
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Installation Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Service: $SERVICE_NAME"
echo "User: $SERVICE_USER"
echo "Install Dir: $INSTALL_DIR"
echo ""
echo "Commands:"
echo "  systemctl status $SERVICE_NAME   # Check status"
echo "  systemctl restart $SERVICE_NAME  # Restart"
echo "  systemctl stop $SERVICE_NAME     # Stop"
echo "  journalctl -u $SERVICE_NAME -f   # View logs"
echo ""
echo "Readiness check:"
echo "  curl http://localhost:5800/api/status"
echo ""
echo "Dashboard:"
IP_ADDR=$(hostname -I 2>/dev/null | awk '{print $1}' || echo "localhost")
echo "  http://$IP_ADDR:5800"
echo ""
echo "Edit configuration:"
echo "  $INSTALL_DIR/config/config.yml"
echo ""

# Show quick status
echo "Current status:"
systemctl status "$SERVICE_NAME" --no-pager | head -10
