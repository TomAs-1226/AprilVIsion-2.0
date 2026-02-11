#!/bin/bash
# AprilVision 2.0 - Update PhotonVision to a new version
#
# Usage: ./update_photonvision.sh [VERSION]
# Example: ./update_photonvision.sh v2026.3.0

set -e

VERSION="${1:-}"
INSTALL_DIR="/opt/photonvision"

if [[ -z "$VERSION" ]]; then
    echo "Usage: $0 <version>"
    echo "Example: $0 v2026.3.0"
    echo ""
    echo "Check latest version at:"
    echo "  https://github.com/PhotonVision/photonvision/releases"
    exit 1
fi

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
        echo "Unsupported architecture: $ARCH"
        exit 1
        ;;
esac

DOWNLOAD_URL="https://github.com/PhotonVision/photonvision/releases/download/${VERSION}/${JAR_NAME}"

echo "Updating PhotonVision to ${VERSION}..."
echo "Download URL: ${DOWNLOAD_URL}"
echo ""

# Stop service
echo "Stopping PhotonVision service..."
sudo systemctl stop photonvision 2>/dev/null || true
sudo systemctl stop aprilvision-dashboard 2>/dev/null || true

# Backup current JAR
if [[ -f "${INSTALL_DIR}/photonvision.jar" ]]; then
    echo "Backing up current JAR..."
    sudo cp "${INSTALL_DIR}/photonvision.jar" "${INSTALL_DIR}/photonvision.jar.bak"
fi

# Download new version
echo "Downloading ${JAR_NAME}..."
if sudo wget -q --show-progress -O "${INSTALL_DIR}/photonvision.jar" "$DOWNLOAD_URL"; then
    echo "Download successful!"
else
    echo "Download failed. Restoring backup..."
    if [[ -f "${INSTALL_DIR}/photonvision.jar.bak" ]]; then
        sudo mv "${INSTALL_DIR}/photonvision.jar.bak" "${INSTALL_DIR}/photonvision.jar"
    fi
    exit 1
fi

# Clean backup
sudo rm -f "${INSTALL_DIR}/photonvision.jar.bak"

# Restart services
echo "Starting services..."
sudo systemctl start photonvision
sleep 3
sudo systemctl start aprilvision-dashboard

echo ""
echo "PhotonVision updated to ${VERSION}"
echo "Check status: sudo systemctl status photonvision"
