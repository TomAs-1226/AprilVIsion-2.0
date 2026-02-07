#!/bin/bash
#===============================================================================
# FRC Vision Coprocessor - Complete Setup Script
# One script to build, configure, and deploy everything
#
# Usage: ./setup.sh [OPTIONS]
#
# Options:
#   --team TEAM       Set team number (e.g., 1234)
#   --build-only      Only build, don't install service
#   --install-only    Only install service (requires prior build)
#   --dev             Development mode (no service install)
#   --clean           Clean build directory first
#   --help            Show this help
#
# Examples:
#   ./setup.sh --team 1234          # Full setup for team 1234
#   ./setup.sh --build-only         # Just build
#   ./setup.sh --clean --team 5678  # Clean build + setup for team 5678
#===============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
TEAM_NUMBER=""
BUILD_ONLY=false
INSTALL_ONLY=false
DEV_MODE=false
CLEAN_BUILD=false
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
INSTALL_DIR="/opt/frc-vision"

#===============================================================================
# Helper functions
#===============================================================================

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[OK]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_banner() {
    echo ""
    echo "╔═══════════════════════════════════════════════════════════════════╗"
    echo "║       FRC AprilTag Vision Coprocessor - Setup Script              ║"
    echo "║                    Orange Pi 5 (RK3588S)                          ║"
    echo "╚═══════════════════════════════════════════════════════════════════╝"
    echo ""
}

show_help() {
    head -25 "$0" | tail -21
    exit 0
}

check_root() {
    if [[ $EUID -eq 0 ]]; then
        log_error "Don't run this script as root. It will ask for sudo when needed."
        exit 1
    fi
}

check_arch() {
    ARCH=$(uname -m)
    if [[ "$ARCH" != "aarch64" ]]; then
        log_warn "This system is $ARCH, not aarch64 (ARM64)"
        log_warn "This software is designed for Orange Pi 5 / RK3588S"
        read -p "Continue anyway? [y/N] " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
}

#===============================================================================
# Parse arguments
#===============================================================================

while [[ $# -gt 0 ]]; do
    case $1 in
        --team)
            TEAM_NUMBER="$2"
            shift 2
            ;;
        --build-only)
            BUILD_ONLY=true
            shift
            ;;
        --install-only)
            INSTALL_ONLY=true
            shift
            ;;
        --dev)
            DEV_MODE=true
            shift
            ;;
        --clean)
            CLEAN_BUILD=true
            shift
            ;;
        --help|-h)
            show_help
            ;;
        *)
            log_error "Unknown option: $1"
            show_help
            ;;
    esac
done

#===============================================================================
# Step 1: Install Dependencies
#===============================================================================

install_dependencies() {
    log_info "Installing system dependencies..."

    # Check if we need to install packages
    PACKAGES_NEEDED=false

    for pkg in cmake ninja-build g++ libopencv-dev libyaml-cpp-dev; do
        if ! dpkg -s "$pkg" &> /dev/null; then
            PACKAGES_NEEDED=true
            break
        fi
    done

    if $PACKAGES_NEEDED; then
        log_info "Installing required packages (requires sudo)..."
        sudo apt-get update
        sudo apt-get install -y \
            build-essential \
            cmake \
            ninja-build \
            g++ \
            git \
            pkg-config \
            libopencv-dev \
            libyaml-cpp-dev \
            libssl-dev \
            v4l-utils \
            curl
        log_success "Dependencies installed"
    else
        log_success "All dependencies already installed"
    fi
}

#===============================================================================
# Step 2: Build
#===============================================================================

build_project() {
    log_info "Building FRC Vision Coprocessor..."

    if $CLEAN_BUILD && [[ -d "$BUILD_DIR" ]]; then
        log_info "Cleaning build directory..."
        rm -rf "$BUILD_DIR"
    fi

    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"

    log_info "Configuring with CMake..."
    cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release

    log_info "Building (this may take a few minutes on first build)..."
    ninja -j$(nproc)

    if [[ -f "${BUILD_DIR}/frc_vision" ]]; then
        log_success "Build successful: ${BUILD_DIR}/frc_vision"
    else
        log_error "Build failed - executable not found"
        exit 1
    fi

    cd "$SCRIPT_DIR"
}

#===============================================================================
# Step 3: Configure Team Number
#===============================================================================

configure_team() {
    if [[ -z "$TEAM_NUMBER" ]]; then
        log_info "No team number specified"
        read -p "Enter your FRC team number (e.g., 1234): " TEAM_NUMBER
    fi

    if [[ ! "$TEAM_NUMBER" =~ ^[0-9]+$ ]]; then
        log_error "Invalid team number: $TEAM_NUMBER"
        exit 1
    fi

    # Calculate roboRIO IP from team number
    if [[ ${#TEAM_NUMBER} -le 2 ]]; then
        ROBORIO_IP="10.0.${TEAM_NUMBER}.2"
    elif [[ ${#TEAM_NUMBER} -eq 3 ]]; then
        ROBORIO_IP="10.${TEAM_NUMBER:0:1}.${TEAM_NUMBER:1:2}.2"
    else
        ROBORIO_IP="10.${TEAM_NUMBER:0:2}.${TEAM_NUMBER:2:2}.2"
    fi

    log_info "Team $TEAM_NUMBER -> roboRIO IP: $ROBORIO_IP"

    # Update config file
    CONFIG_FILE="${SCRIPT_DIR}/config/config.yml"
    if [[ -f "$CONFIG_FILE" ]]; then
        log_info "Updating config with team $TEAM_NUMBER..."
        sed -i "s|nt_server:.*|nt_server: \"${ROBORIO_IP}\"           # Team $TEAM_NUMBER roboRIO|" "$CONFIG_FILE"
        log_success "Config updated: $CONFIG_FILE"
    else
        log_warn "Config file not found at $CONFIG_FILE"
    fi
}

#===============================================================================
# Step 4: Create System User and Permissions
#===============================================================================

setup_user_and_permissions() {
    log_info "Setting up system user and permissions (requires sudo)..."

    # Create frcvision user if it doesn't exist
    if ! id -u frcvision &>/dev/null; then
        log_info "Creating frcvision user..."
        sudo useradd -r -s /usr/sbin/nologin -d /opt/frc-vision -c "FRC Vision Coprocessor" frcvision
        log_success "User frcvision created"
    else
        log_success "User frcvision already exists"
    fi

    # Add frcvision to video group for camera access
    sudo usermod -aG video frcvision 2>/dev/null || true

    # Create udev rules for camera access
    UDEV_RULES="/etc/udev/rules.d/99-frc-vision.rules"
    log_info "Creating udev rules for camera access..."
    sudo tee "$UDEV_RULES" > /dev/null << 'EOF'
# FRC Vision Coprocessor - Camera Access Rules
# Give frcvision user access to all video devices

# V4L2 video devices
KERNEL=="video[0-9]*", MODE="0666", GROUP="video"

# USB cameras - set permissions
SUBSYSTEM=="usb", ATTR{idVendor}=="*",  MODE="0666"

# Trigger udev reload
EOF

    # Reload udev rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    log_success "Udev rules created"

    # Set resource limits for real-time performance
    LIMITS_FILE="/etc/security/limits.d/frc-vision.conf"
    log_info "Setting resource limits..."
    sudo tee "$LIMITS_FILE" > /dev/null << 'EOF'
# FRC Vision Coprocessor - Resource Limits
frcvision soft rtprio 99
frcvision hard rtprio 99
frcvision soft memlock unlimited
frcvision hard memlock unlimited
frcvision soft nice -20
frcvision hard nice -20
EOF
    log_success "Resource limits configured"
}

#===============================================================================
# Step 5: Install Files
#===============================================================================

install_files() {
    log_info "Installing to $INSTALL_DIR (requires sudo)..."

    # Create directories
    sudo mkdir -p "${INSTALL_DIR}"
    sudo mkdir -p "${INSTALL_DIR}/config"
    sudo mkdir -p "${INSTALL_DIR}/web"
    sudo mkdir -p "${INSTALL_DIR}/logs"

    # Copy binary
    if [[ -f "${BUILD_DIR}/frc_vision" ]]; then
        sudo cp "${BUILD_DIR}/frc_vision" "${INSTALL_DIR}/"
        sudo chmod +x "${INSTALL_DIR}/frc_vision"
        log_success "Binary installed"
    else
        log_error "Binary not found. Run with --build-only first or without --install-only"
        exit 1
    fi

    # Copy config files
    if [[ -d "${SCRIPT_DIR}/config" ]]; then
        sudo cp -r "${SCRIPT_DIR}/config/"* "${INSTALL_DIR}/config/"
        log_success "Config files installed"
    fi

    # Copy web files
    if [[ -d "${SCRIPT_DIR}/web" ]]; then
        sudo cp -r "${SCRIPT_DIR}/web/"* "${INSTALL_DIR}/web/"
        log_success "Web files installed"
    fi

    # Set ownership
    sudo chown -R frcvision:frcvision "${INSTALL_DIR}"

    log_success "Files installed to $INSTALL_DIR"
}

#===============================================================================
# Step 6: Install Systemd Service
#===============================================================================

install_service() {
    log_info "Installing systemd service..."

    SERVICE_FILE="/etc/systemd/system/frc_vision.service"

    sudo tee "$SERVICE_FILE" > /dev/null << 'EOF'
[Unit]
Description=FRC AprilTag Vision Coprocessor
Documentation=https://github.com/your-team/frc-vision
After=network.target
Wants=network.target

# Don't wait for network - we'll retry connection
StartLimitIntervalSec=0

[Service]
Type=simple
User=frcvision
Group=frcvision
WorkingDirectory=/opt/frc-vision

# Main executable with fast-start flag
ExecStart=/opt/frc-vision/frc_vision /opt/frc-vision/config/config.yml --fast-start

# Fast restart on failure
Restart=always
RestartSec=0.5
TimeoutStartSec=30
TimeoutStopSec=5

# Kill whole process group
KillMode=mixed
KillSignal=SIGTERM

# Resource limits
Nice=-5
LimitRTPRIO=99
LimitMEMLOCK=infinity
CPUSchedulingPolicy=rr
CPUSchedulingPriority=50

# Environment
Environment="MALLOC_ARENA_MAX=2"
Environment="OPENCV_VIDEOIO_DEBUG=0"

# Logging
StandardOutput=journal
StandardError=journal
SyslogIdentifier=frc_vision

[Install]
WantedBy=multi-user.target
EOF

    # Reload systemd
    sudo systemctl daemon-reload

    # Enable service
    sudo systemctl enable frc_vision

    log_success "Service installed and enabled"

    # Ask to start
    read -p "Start the service now? [Y/n] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Nn]$ ]]; then
        sudo systemctl start frc_vision
        sleep 2
        if systemctl is-active --quiet frc_vision; then
            log_success "Service started successfully"
        else
            log_warn "Service may still be starting. Check: sudo systemctl status frc_vision"
        fi
    fi
}

#===============================================================================
# Step 7: Verify Installation
#===============================================================================

verify_installation() {
    log_info "Verifying installation..."

    echo ""
    echo "Installation Summary:"
    echo "────────────────────────────────────────────────────────────"

    # Check binary
    if [[ -x "${INSTALL_DIR}/frc_vision" ]]; then
        echo -e "  Binary:     ${GREEN}✓${NC} ${INSTALL_DIR}/frc_vision"
    else
        echo -e "  Binary:     ${RED}✗${NC} Not found"
    fi

    # Check config
    if [[ -f "${INSTALL_DIR}/config/config.yml" ]]; then
        echo -e "  Config:     ${GREEN}✓${NC} ${INSTALL_DIR}/config/config.yml"
    else
        echo -e "  Config:     ${RED}✗${NC} Not found"
    fi

    # Check service
    if systemctl is-enabled --quiet frc_vision 2>/dev/null; then
        echo -e "  Service:    ${GREEN}✓${NC} Enabled"
    else
        echo -e "  Service:    ${YELLOW}○${NC} Not enabled"
    fi

    # Check if running
    if systemctl is-active --quiet frc_vision 2>/dev/null; then
        echo -e "  Status:     ${GREEN}✓${NC} Running"
    else
        echo -e "  Status:     ${YELLOW}○${NC} Not running"
    fi

    # Get IP address
    IP_ADDR=$(hostname -I | awk '{print $1}')

    echo "────────────────────────────────────────────────────────────"
    echo ""
    echo "Access:"
    echo "  Dashboard:  http://${IP_ADDR}:5800"
    echo "  Status API: http://${IP_ADDR}:5800/api/status"
    echo ""
    echo "Commands:"
    echo "  Start:      sudo systemctl start frc_vision"
    echo "  Stop:       sudo systemctl stop frc_vision"
    echo "  Status:     sudo systemctl status frc_vision"
    echo "  Logs:       journalctl -u frc_vision -f"
    echo ""

    if [[ -n "$TEAM_NUMBER" ]]; then
        echo "Team Configuration:"
        echo "  Team:       $TEAM_NUMBER"
        echo "  roboRIO IP: $ROBORIO_IP"
        echo ""
    fi
}

#===============================================================================
# Main
#===============================================================================

main() {
    print_banner
    check_root
    check_arch

    if $INSTALL_ONLY; then
        setup_user_and_permissions
        install_files
        install_service
        verify_installation
        exit 0
    fi

    # Full installation
    install_dependencies

    if ! $INSTALL_ONLY; then
        build_project
    fi

    if $BUILD_ONLY; then
        log_success "Build complete. Run './setup.sh --install-only' to install."
        exit 0
    fi

    if $DEV_MODE; then
        log_success "Development build complete."
        echo ""
        echo "Run manually with:"
        echo "  ./build/frc_vision config/config.yml"
        exit 0
    fi

    # Configure team number
    configure_team

    # Setup user and permissions
    setup_user_and_permissions

    # Install files
    install_files

    # Install service
    install_service

    # Verify
    verify_installation

    log_success "Setup complete!"
}

# Run main
main
