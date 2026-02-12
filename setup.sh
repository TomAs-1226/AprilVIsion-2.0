#!/bin/bash
#===============================================================================
# AprilVision 3.2 - Custom FRC Vision System
# Complete One-Script Setup
#
# This script downloads the detection engine, installs all dependencies, and
# sets up a complete vision system with a custom dashboard.
#
# Usage: ./setup.sh [OPTIONS]
#
# Options:
#   --team TEAM       Set team number (e.g., 1234)
#   --install-only    Only install service (skip engine download)
#   --deploy          Quick redeploy: copy files + restart services (no engine/Java)
#   --dev             Development mode (no service install)
#   --clean           Remove existing installation first
#   --help            Show this help
#
# Examples:
#   ./setup.sh --team 1234          # Full setup for team 1234
#   ./setup.sh --clean --team 5678  # Clean install for team 5678
#   ./setup.sh --deploy             # Redeploy dashboard files + restart
#   ./setup.sh --deploy --team 5805 # Redeploy + update team number
#===============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Engine version
PV_VERSION="v2026.2.2"
PV_JAR_NAME="photonvision-${PV_VERSION}-linuxarm64.jar"
PV_DOWNLOAD_URL="https://github.com/PhotonVision/photonvision/releases/download/${PV_VERSION}"

# Default values
TEAM_NUMBER=""
INSTALL_ONLY=false
DEV_MODE=false
CLEAN_INSTALL=false
DEPLOY_ONLY=false
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="/opt/photonvision"
DASHBOARD_DIR="/opt/aprilvision"

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
    echo -e "${CYAN}"
    cat << 'BANNER'
       _            _ _ __   ___     _             _____ ____
      / \   _ __  _(_) |\ \ / (_)___(_) ___  _ __ |___ /|___ \
     / _ \ | '_ \| '_| | \ V /| / __| |/ _ \| '_ \  |_ \  __) |
    / ___ \| |_) | | | |  | | | \__ \ | (_) | | | |___) |/ __/
   /_/   \_\ .__/|_| |_|  |_| |_|___/_|\___/|_| |_|____/|_____|
           |_|
BANNER
    echo -e "${NC}"
    echo -e "${GREEN}================================================================${NC}"
    echo -e "${GREEN}|${NC}  ${YELLOW}FRC Competition Vision System${NC}                              ${GREEN}|${NC}"
    echo -e "${GREEN}|${NC}  ${CYAN}AprilVision 3.2 Detection Engine${NC}                           ${GREEN}|${NC}"
    echo -e "${GREEN}|${NC}  Built by Team 1226 for FRC 2026 Season                     ${GREEN}|${NC}"
    echo -e "${GREEN}================================================================${NC}"
    echo ""
    echo -e "  ${YELLOW}What you get:${NC}"
    echo "    - AprilVision 3.2 AprilTag detection engine"
    echo "    - Multi-tag PnP on coprocessor for accurate poses"
    echo "    - Custom branded dashboard with system health monitoring"
    echo "    - Auto-start systemd services with crash recovery"
    echo "    - Pre-match health check + competition match mode"
    echo "    - Drop-in Java robot code with Vision Library (PhotonLib) integration"
    echo ""
}

show_help() {
    head -18 "$0" | tail -14
    exit 0
}

check_root() {
    if [[ $EUID -eq 0 ]]; then
        log_error "Don't run this script as root. It will ask for sudo when needed."
        exit 1
    fi
}

check_os_compatibility() {
    log_info "Checking OS compatibility..."

    # Check for Debian-based distro (required for apt-get)
    if ! command -v apt-get &> /dev/null; then
        log_error "This script requires a Debian-based Linux distribution (Ubuntu, Debian, Raspberry Pi OS, Armbian, etc.)"
        log_error "Your system does not have apt-get. Please install dependencies manually or use a supported distro."
        exit 1
    fi

    # Read distro info if available
    if [[ -f /etc/os-release ]]; then
        source /etc/os-release
        log_success "Detected OS: ${PRETTY_NAME:-$ID}"

        case "$ID" in
            ubuntu|debian|raspbian|linuxmint|pop|armbian)
                log_success "OS $ID is fully supported"
                ;;
            *)
                # Has apt-get but not a known distro - warn but continue
                if [[ "$ID_LIKE" == *"debian"* ]] || [[ "$ID_LIKE" == *"ubuntu"* ]]; then
                    log_success "OS $ID (Debian-based) should be compatible"
                else
                    log_warn "OS '$ID' is not officially tested. Proceeding since apt-get is available."
                    log_warn "If you encounter issues, try Ubuntu 22.04+ or Debian 12+"
                fi
                ;;
        esac
    else
        log_warn "Could not detect OS version. Proceeding since apt-get is available."
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
        --install-only)
            INSTALL_ONLY=true
            shift
            ;;
        --deploy)
            DEPLOY_ONLY=true
            shift
            ;;
        --dev)
            DEV_MODE=true
            shift
            ;;
        --clean)
            CLEAN_INSTALL=true
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
# Step 1: Detect Architecture
#===============================================================================

detect_architecture() {
    ARCH=$(uname -m)
    log_info "Detected architecture: $ARCH"

    case "$ARCH" in
        aarch64|arm64)
            PV_JAR_NAME="photonvision-${PV_VERSION}-linuxarm64.jar"
            JAVA_ARCH="arm64"
            log_success "ARM64 detected - using arm64 engine JAR"
            ;;
        x86_64|amd64)
            PV_JAR_NAME="photonvision-${PV_VERSION}-linuxx64.jar"
            JAVA_ARCH="amd64"
            log_success "x86_64 detected - using x64 engine JAR"
            ;;
        *)
            log_error "Unsupported architecture: $ARCH"
            log_error "AprilVision requires ARM64 or x86_64"
            exit 1
            ;;
    esac
}

#===============================================================================
# Step 2: Install System Dependencies
#===============================================================================

install_dependencies() {
    log_info "Installing system dependencies..."

    PACKAGES_NEEDED=false

    for pkg in curl wget unzip v4l-utils python3; do
        if ! dpkg -s "$pkg" &> /dev/null; then
            PACKAGES_NEEDED=true
            break
        fi
    done

    if $PACKAGES_NEEDED; then
        log_info "Installing required packages (requires sudo)..."
        sudo apt-get update
        sudo apt-get install -y \
            curl \
            wget \
            unzip \
            v4l-utils \
            python3 \
            libatomic1 \
            libglib2.0-0
        log_success "System dependencies installed"
    else
        log_success "System dependencies already installed"
    fi
}

#===============================================================================
# Step 3: Install Java 17
#===============================================================================

install_java() {
    log_info "Checking Java installation..."

    # Check if Java 17 is already installed
    if command -v java &> /dev/null; then
        JAVA_VER=$(java -version 2>&1 | head -n 1 | awk -F '"' '{print $2}' | cut -d'.' -f1)
        if [[ "$JAVA_VER" == "17" ]]; then
            log_success "Java 17 already installed"
            return
        else
            log_warn "Java $JAVA_VER found, but Java 17 is required"
        fi
    fi

    log_info "Installing Java 17 JDK (required by detection engine)..."
    sudo apt-get update
    sudo apt-get install -y openjdk-17-jdk-headless

    # Verify installation
    if command -v java &> /dev/null; then
        JAVA_VER=$(java -version 2>&1 | head -n 1)
        log_success "Java installed: $JAVA_VER"
    else
        log_error "Java installation failed"
        exit 1
    fi
}

#===============================================================================
# Step 4: Download Detection Engine
#===============================================================================

download_photonvision() {
    log_info "Setting up detection engine ${PV_VERSION}..."

    # Create installation directory
    sudo mkdir -p "${INSTALL_DIR}"

    # Check if JAR already exists
    if [[ -f "${INSTALL_DIR}/photonvision.jar" ]] && ! $CLEAN_INSTALL; then
        log_success "Engine JAR already present at ${INSTALL_DIR}/photonvision.jar"
        log_info "Use --clean to force re-download"
        return
    fi

    if $CLEAN_INSTALL && [[ -d "${INSTALL_DIR}" ]]; then
        log_info "Cleaning existing engine installation..."
        sudo rm -f "${INSTALL_DIR}/photonvision.jar"
    fi

    local DOWNLOAD_URL="${PV_DOWNLOAD_URL}/${PV_JAR_NAME}"
    log_info "Downloading engine from: ${DOWNLOAD_URL}"
    log_info "This may take a few minutes (JAR is ~115 MB)..."

    # Download with retry logic
    local retries=0
    local max_retries=4
    local wait_time=2

    while [[ $retries -lt $max_retries ]]; do
        if sudo wget -q --show-progress -O "${INSTALL_DIR}/photonvision.jar" "$DOWNLOAD_URL"; then
            log_success "Detection engine ${PV_VERSION} downloaded successfully"
            return
        fi

        retries=$((retries + 1))
        if [[ $retries -lt $max_retries ]]; then
            log_warn "Download failed, retrying in ${wait_time}s... (attempt $((retries + 1))/$max_retries)"
            sleep $wait_time
            wait_time=$((wait_time * 2))
        fi
    done

    log_error "Failed to download detection engine after $max_retries attempts"
    log_error "Download manually from: https://github.com/PhotonVision/photonvision/releases"
    log_error "Place the JAR at: ${INSTALL_DIR}/photonvision.jar"
    exit 1
}

#===============================================================================
# Step 5: Configure Team Number
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

    # Update our config file with team number
    CONFIG_FILE="${SCRIPT_DIR}/config/config.yml"
    if [[ -f "$CONFIG_FILE" ]]; then
        sed -i "s|team_number:.*|team_number: ${TEAM_NUMBER}|" "$CONFIG_FILE"
        sed -i "s|nt_server:.*|nt_server: \"${ROBORIO_IP}\"|" "$CONFIG_FILE"
        log_success "Config updated with team $TEAM_NUMBER"
    fi

    # Update engine network config
    # The engine uses its own settings directory
    sudo mkdir -p "${INSTALL_DIR}/photonvision_config/network"
    sudo tee "${INSTALL_DIR}/photonvision_config/network/networkSettings.json" > /dev/null << EOF
{
    "ntServerAddress": "${ROBORIO_IP}",
    "teamNumber": ${TEAM_NUMBER},
    "connectionType": 0,
    "staticIp": "",
    "hostname": "photonvision",
    "runNTServer": true,
    "shouldManage": false,
    "shouldPublishProto": true,
    "networkManagerIface": "",
    "setStaticCommand": "",
    "setDHCPcommand": ""
}
EOF
    log_success "Engine network configured for team $TEAM_NUMBER"
}

#===============================================================================
# Step 6: Create System User and Permissions
#===============================================================================

setup_user_and_permissions() {
    log_info "Setting up system user and permissions..."

    # Create photonvision user if it doesn't exist
    if ! id -u photonvision &>/dev/null; then
        log_info "Creating photonvision user..."
        sudo useradd -r -s /usr/sbin/nologin -d "${INSTALL_DIR}" -c "AprilVision Engine Service" photonvision
        log_success "User photonvision created"
    else
        log_success "User photonvision already exists"
    fi

    # Add to video group for camera access
    sudo usermod -aG video photonvision 2>/dev/null || true

    # Create udev rules for camera access
    UDEV_RULES="/etc/udev/rules.d/99-photonvision.rules"
    log_info "Creating udev rules for camera access..."
    sudo tee "$UDEV_RULES" > /dev/null << 'EOF'
# AprilVision 3.2 - Camera Access Rules
# Give engine user access to all video devices

# V4L2 video devices
KERNEL=="video[0-9]*", MODE="0666", GROUP="video"

# USB cameras - set permissions
SUBSYSTEM=="usb", ATTR{idVendor}=="*", MODE="0666"
EOF

    # Reload udev rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    log_success "Udev rules created"

    # Set resource limits for real-time performance
    LIMITS_FILE="/etc/security/limits.d/photonvision.conf"
    sudo tee "$LIMITS_FILE" > /dev/null << 'EOF'
# AprilVision 3.2 - Resource Limits
photonvision soft rtprio 99
photonvision hard rtprio 99
photonvision soft memlock unlimited
photonvision hard memlock unlimited
photonvision soft nice -20
photonvision hard nice -20
EOF
    log_success "Resource limits configured"
}

#===============================================================================
# Step 7: Install Files
#===============================================================================

install_files() {
    log_info "Installing AprilVision dashboard files..."

    # Create dashboard directories
    sudo mkdir -p "${DASHBOARD_DIR}"
    sudo mkdir -p "${DASHBOARD_DIR}/web"
    sudo mkdir -p "${DASHBOARD_DIR}/config"
    sudo mkdir -p "${DASHBOARD_DIR}/logs"

    # Copy web files
    if [[ -d "${SCRIPT_DIR}/web" ]]; then
        sudo cp -r "${SCRIPT_DIR}/web/"* "${DASHBOARD_DIR}/web/"
        log_success "Web dashboard files installed"
    fi

    # Copy config
    if [[ -d "${SCRIPT_DIR}/config" ]]; then
        sudo cp -r "${SCRIPT_DIR}/config/"* "${DASHBOARD_DIR}/config/"
        log_success "Config files installed"
    fi

    # Copy bridge script
    if [[ -f "${SCRIPT_DIR}/scripts/aprilvision-bridge.py" ]]; then
        sudo cp "${SCRIPT_DIR}/scripts/aprilvision-bridge.py" "${DASHBOARD_DIR}/bridge.py"
        sudo chmod +x "${DASHBOARD_DIR}/bridge.py"
        log_success "Dashboard bridge installed"
    fi

    # Set ownership
    sudo chown -R photonvision:photonvision "${INSTALL_DIR}"
    sudo chown -R photonvision:photonvision "${DASHBOARD_DIR}"

    log_success "All files installed"
}

#===============================================================================
# Step 8: Install Systemd Services
#===============================================================================

install_services() {
    log_info "Installing systemd services..."

    # Stop existing services if running
    sudo systemctl stop photonvision 2>/dev/null || true
    sudo systemctl stop aprilvision-dashboard 2>/dev/null || true

    # Detection engine service
    sudo tee "/etc/systemd/system/photonvision.service" > /dev/null << EOF
[Unit]
Description=AprilVision 3.2 - Detection Engine
After=network.target
Wants=network-online.target
StartLimitIntervalSec=0

[Service]
Type=simple
User=root
WorkingDirectory=${INSTALL_DIR}
ExecStart=/usr/bin/java \\
    -Xmx512m \\
    -jar ${INSTALL_DIR}/photonvision.jar \\
    -configDir ${INSTALL_DIR}/photonvision_config
Restart=always
RestartSec=2
TimeoutStartSec=60
TimeoutStopSec=10
Nice=-5
LimitRTPRIO=99
LimitMEMLOCK=infinity
Environment="JAVA_HOME=/usr/lib/jvm/java-17-openjdk-${JAVA_ARCH}"
StandardOutput=journal
StandardError=journal
SyslogIdentifier=aprilvision-engine

[Install]
WantedBy=multi-user.target
EOF

    # AprilVision dashboard service
    # Build ExecStart with optional --team flag
    local BRIDGE_CMD="/usr/bin/python3 ${DASHBOARD_DIR}/bridge.py --port 5801 --engine-port 5800 --web-dir ${DASHBOARD_DIR}/web"
    if [[ -n "$TEAM_NUMBER" && "$TEAM_NUMBER" != "0" && "$TEAM_NUMBER" != "0000" ]]; then
        BRIDGE_CMD="${BRIDGE_CMD} --team ${TEAM_NUMBER}"
    fi

    sudo tee "/etc/systemd/system/aprilvision-dashboard.service" > /dev/null << EOF
[Unit]
Description=AprilVision 3.2 - Dashboard Service
After=photonvision.service
Requires=photonvision.service

[Service]
Type=simple
User=photonvision
Group=photonvision
WorkingDirectory=${DASHBOARD_DIR}
ExecStart=${BRIDGE_CMD}
Restart=always
RestartSec=3
TimeoutStartSec=15
StandardOutput=journal
StandardError=journal
SyslogIdentifier=aprilvision-dashboard

[Install]
WantedBy=multi-user.target
EOF

    # Reload systemd
    sudo systemctl daemon-reload

    # Enable services
    sudo systemctl enable photonvision
    sudo systemctl enable aprilvision-dashboard

    log_success "Services installed and enabled"

    # Ask to start
    if ! $DEV_MODE; then
        read -p "Start the services now? [Y/n] " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Nn]$ ]]; then
            sudo systemctl start photonvision
            sleep 3
            sudo systemctl start aprilvision-dashboard
            sleep 2

            if systemctl is-active --quiet photonvision; then
                log_success "Detection engine service started"
            else
                log_warn "Detection engine may still be starting. Check: sudo systemctl status photonvision"
            fi

            if systemctl is-active --quiet aprilvision-dashboard; then
                log_success "Dashboard service started"
            else
                log_warn "Dashboard may still be starting. Check: sudo systemctl status aprilvision-dashboard"
            fi
        fi
    fi
}

#===============================================================================
# Step 9: Verify Installation
#===============================================================================

verify_installation() {
    log_info "Verifying installation..."

    echo ""
    echo "Installation Summary:"
    echo "------------------------------------------------------------"

    # Check engine JAR
    if [[ -f "${INSTALL_DIR}/photonvision.jar" ]]; then
        local jar_size=$(du -h "${INSTALL_DIR}/photonvision.jar" | awk '{print $1}')
        echo -e "  Engine JAR:        ${GREEN}OK${NC} (${jar_size})"
    else
        echo -e "  Engine JAR:        ${RED}MISSING${NC}"
    fi

    # Check Java
    if command -v java &> /dev/null; then
        echo -e "  Java:              ${GREEN}OK${NC} ($(java -version 2>&1 | head -n 1))"
    else
        echo -e "  Java:              ${RED}MISSING${NC}"
    fi

    # Check dashboard files
    if [[ -f "${DASHBOARD_DIR}/web/index.html" ]]; then
        echo -e "  Dashboard:         ${GREEN}OK${NC}"
    else
        echo -e "  Dashboard:         ${RED}MISSING${NC}"
    fi

    # Check services
    if systemctl is-enabled --quiet photonvision 2>/dev/null; then
        echo -e "  Engine Service:    ${GREEN}Enabled${NC}"
    else
        echo -e "  Engine Service:    ${YELLOW}Not enabled${NC}"
    fi

    if systemctl is-enabled --quiet aprilvision-dashboard 2>/dev/null; then
        echo -e "  Dashboard Service: ${GREEN}Enabled${NC}"
    else
        echo -e "  Dashboard Service: ${YELLOW}Not enabled${NC}"
    fi

    # Check if running
    if systemctl is-active --quiet photonvision 2>/dev/null; then
        echo -e "  Engine Status:     ${GREEN}Running${NC}"
    else
        echo -e "  Engine Status:     ${YELLOW}Not running${NC}"
    fi

    if systemctl is-active --quiet aprilvision-dashboard 2>/dev/null; then
        echo -e "  Dashboard Status:  ${GREEN}Running${NC}"
    else
        echo -e "  Dashboard Status:  ${YELLOW}Not running${NC}"
    fi

    # Get IP address
    IP_ADDR=$(hostname -I | awk '{print $1}')

    echo "------------------------------------------------------------"
    echo ""
    echo "Access Points:"
    echo "  Detection Engine:      http://${IP_ADDR}:5800"
    echo "  AprilVision Dashboard: http://${IP_ADDR}:5801"
    echo ""
    echo "Commands:"
    echo "  Start all:    sudo systemctl start photonvision aprilvision-dashboard"
    echo "  Stop all:     sudo systemctl stop photonvision aprilvision-dashboard"
    echo "  Engine status: sudo systemctl status photonvision"
    echo "  Engine logs:  journalctl -u photonvision -f"
    echo "  Dash logs:    journalctl -u aprilvision-dashboard -f"
    echo ""

    if [[ -n "$TEAM_NUMBER" ]]; then
        echo "Team Configuration:"
        echo "  Team:       $TEAM_NUMBER"
        echo "  roboRIO IP: $ROBORIO_IP"
        echo ""
    fi
}

#===============================================================================
# Step 10: Print Completion Message
#===============================================================================

print_completion() {
    echo ""
    echo -e "${GREEN}================================================================${NC}"
    echo -e "${GREEN}|${NC}                                                              ${GREEN}|${NC}"
    echo -e "${GREEN}|${NC}  ${YELLOW}AprilVision 3.2 - Setup Complete!${NC}                          ${GREEN}|${NC}"
    echo -e "${GREEN}|${NC}  ${CYAN}AprilVision 3.2 Detection Engine${NC}                            ${GREEN}|${NC}"
    echo -e "${GREEN}|${NC}                                                              ${GREEN}|${NC}"
    echo -e "${GREEN}================================================================${NC}"
    echo ""
    echo -e "${BLUE}Next Steps:${NC}"
    echo ""
    echo "  1. Open the AprilVision dashboard to configure cameras:"
    echo "     ${YELLOW}http://<this-device-ip>:5801${NC}"
    echo "     - Add cameras and set resolution (640x480 @ 30fps)"
    echo "     - Select AprilTag pipeline with tag36h11"
    echo "     - Run camera calibration for accurate poses"
    echo ""
    echo "  2. Run a health check to verify everything:"
    echo "     ${YELLOW}./scripts/health_check.sh${NC}"
    echo ""
    echo "  3. Add Vision Library (PhotonLib) to your robot project's build.gradle:"
    echo "     ${YELLOW}implementation 'org.photonvision:photonlib-java:${PV_VERSION}'${NC}"
    echo ""
    echo "  4. Copy robot code examples from:"
    echo "     ${YELLOW}robot-code-examples/${NC}"
    echo "     - VisionSubsystem.java  (multi-camera subsystem)"
    echo "     - AlignToTagCommand.java (auto-align to tags)"
    echo "     - RobotContainerExample.java (full wiring)"
    echo ""
    echo -e "${BLUE}Competition Day:${NC}"
    echo "  - Run health check:  ${YELLOW}./scripts/health_check.sh${NC}"
    echo "  - Enable match mode: ${YELLOW}./scripts/match_mode.sh enable${NC}"
    echo "  - After practice:    ${YELLOW}./scripts/match_mode.sh disable${NC}"
    echo ""
    echo -e "${BLUE}Useful Commands:${NC}"
    echo "  Start:   sudo systemctl start photonvision aprilvision-dashboard"
    echo "  Stop:    sudo systemctl stop photonvision aprilvision-dashboard"
    echo "  Status:  sudo systemctl status photonvision"
    echo "  Logs:    journalctl -u photonvision -f"
    echo ""
}

#===============================================================================
# Main
#===============================================================================

main() {
    print_banner

    # Quick deploy mode: just copy files + restart (no engine/Java/user setup)
    if $DEPLOY_ONLY; then
        log_info "Deploy mode: updating dashboard files and restarting service"
        if [[ -n "$TEAM_NUMBER" && "$TEAM_NUMBER" != "0" ]]; then
            configure_team
        fi
        install_files
        install_services
        # Restart services
        sudo systemctl daemon-reload
        sudo systemctl restart aprilvision-dashboard 2>/dev/null || true
        sleep 1
        if systemctl is-active --quiet aprilvision-dashboard 2>/dev/null; then
            log_success "Dashboard service restarted"
        else
            log_warn "Dashboard may still be starting. Check: journalctl -u aprilvision-dashboard -n 20"
        fi
        verify_installation
        exit 0
    fi

    check_root
    check_os_compatibility
    detect_architecture

    if $INSTALL_ONLY; then
        install_files
        install_services
        verify_installation
        print_completion
        exit 0
    fi

    # Full installation
    install_dependencies
    install_java
    download_photonvision

    if ! $DEV_MODE; then
        configure_team
        setup_user_and_permissions
        install_files
        install_services
    fi

    verify_installation
    print_completion
}

# Run main
main
