#!/bin/bash
#===============================================================================
# AprilVision 3.2 - Engine Updater
#
# Downloads and installs a new version of the detection engine.
# Automatically backs up the current JAR before updating.
#
# Usage: ./scripts/update_photonvision.sh [VERSION]
# Example: ./scripts/update_photonvision.sh v2026.3.0
#===============================================================================

set -e

CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

VERSION="${1:-}"
INSTALL_DIR="/opt/photonvision"

if [[ -z "$VERSION" ]]; then
    echo ""
    echo -e "${CYAN}AprilVision 3.2 - Engine Updater${NC}"
    echo ""
    echo "Usage: $0 <version>"
    echo "Example: $0 v2026.3.0"
    echo ""
    echo "Check latest version at:"
    echo "  https://github.com/PhotonVision/photonvision/releases"
    exit 1
fi

echo ""
echo -e "${CYAN}AprilVision 3.2 - Updating Detection Engine${NC}"
echo "============================================="

# Detect architecture
ARCH=$(uname -m)
case "$ARCH" in
    aarch64|arm64)
        JAR_NAME="photonvision-${VERSION}-linuxarm64.jar"
        ;;
    x86_64|amd64)
        JAR_NAME="photonvision-${VERSION}-linuxx64.jar"
        ;;
    *)
        echo -e "${RED}Unsupported architecture: $ARCH${NC}"
        exit 1
        ;;
esac

DOWNLOAD_URL="https://github.com/PhotonVision/photonvision/releases/download/${VERSION}/${JAR_NAME}"

echo -e "  Target version: ${GREEN}${VERSION}${NC}"
echo -e "  Architecture:   ${GREEN}${ARCH}${NC}"
echo -e "  Download:       ${JAR_NAME}"
echo ""

# Stop services
echo -e "${YELLOW}Stopping services...${NC}"
sudo systemctl stop aprilvision-dashboard 2>/dev/null || true
sudo systemctl stop photonvision 2>/dev/null || true

# Backup current JAR
if [[ -f "${INSTALL_DIR}/photonvision.jar" ]]; then
    echo "Backing up current JAR..."
    sudo cp "${INSTALL_DIR}/photonvision.jar" "${INSTALL_DIR}/photonvision.jar.bak"
fi

# Download new version with retry
echo "Downloading ${JAR_NAME}..."
RETRIES=0
MAX_RETRIES=4
WAIT=2

while [[ $RETRIES -lt $MAX_RETRIES ]]; do
    if sudo wget -q --show-progress -O "${INSTALL_DIR}/photonvision.jar" "$DOWNLOAD_URL"; then
        echo -e "${GREEN}Download successful!${NC}"
        break
    fi
    RETRIES=$((RETRIES + 1))
    if [[ $RETRIES -lt $MAX_RETRIES ]]; then
        echo -e "${YELLOW}Retrying in ${WAIT}s... (attempt $((RETRIES + 1))/${MAX_RETRIES})${NC}"
        sleep $WAIT
        WAIT=$((WAIT * 2))
    else
        echo -e "${RED}Download failed after $MAX_RETRIES attempts.${NC}"
        if [[ -f "${INSTALL_DIR}/photonvision.jar.bak" ]]; then
            echo "Restoring backup..."
            sudo mv "${INSTALL_DIR}/photonvision.jar.bak" "${INSTALL_DIR}/photonvision.jar"
        fi
        exit 1
    fi
done

# Clean backup
sudo rm -f "${INSTALL_DIR}/photonvision.jar.bak"

# Restart services
echo "Restarting services..."
sudo systemctl start photonvision
sleep 3
sudo systemctl start aprilvision-dashboard

echo ""
echo -e "${GREEN}Detection engine updated to ${VERSION}${NC}"
echo ""
echo "Verify with: sudo systemctl status photonvision"
echo "Health check: ./scripts/health_check.sh"
echo ""
