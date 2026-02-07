#!/bin/bash
# Download prebuilt WPILib ntcore libraries for ARM64
# Much faster than building from source!

set -e

WPILIB_VERSION="2026.2.1"
WPILIB_YEAR="2026"
ARCH="linuxarm64"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
LIBS_DIR="${PROJECT_DIR}/third_party/wpilib"

echo "Downloading WPILib ${WPILIB_VERSION} prebuilt libraries for ${ARCH}..."

mkdir -p "${LIBS_DIR}/lib"
mkdir -p "${LIBS_DIR}/include"

MAVEN_BASE="https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first"

# Function to download and extract
download_lib() {
    local name=$1
    local artifact=$2

    echo "Downloading ${name}..."

    # Download headers
    local headers_url="${MAVEN_BASE}/${name}/${artifact}/${WPILIB_VERSION}/${artifact}-${WPILIB_VERSION}-headers.zip"
    curl -sL "${headers_url}" -o "/tmp/${artifact}-headers.zip" || {
        echo "Warning: Could not download ${name} headers"
        return 1
    }
    unzip -q -o "/tmp/${artifact}-headers.zip" -d "${LIBS_DIR}/include/" 2>/dev/null || true
    rm -f "/tmp/${artifact}-headers.zip"

    # Download static library
    local lib_url="${MAVEN_BASE}/${name}/${artifact}/${WPILIB_VERSION}/${artifact}-${WPILIB_VERSION}-${ARCH}static.zip"
    curl -sL "${lib_url}" -o "/tmp/${artifact}-lib.zip" || {
        echo "Warning: Could not download ${name} library"
        return 1
    }
    unzip -q -o "/tmp/${artifact}-lib.zip" -d "${LIBS_DIR}/lib/" 2>/dev/null || true
    rm -f "/tmp/${artifact}-lib.zip"

    echo "  ${name} downloaded"
}

# Download required libraries
download_lib "ntcore" "ntcore-cpp"
download_lib "wpiutil" "wpiutil-cpp"
download_lib "wpinet" "wpinet-cpp"

# Flatten library directory structure
find "${LIBS_DIR}/lib" -name "*.a" -exec mv {} "${LIBS_DIR}/lib/" \; 2>/dev/null || true
find "${LIBS_DIR}/lib" -type d -empty -delete 2>/dev/null || true

echo ""
echo "WPILib libraries downloaded to: ${LIBS_DIR}"
echo ""
ls -la "${LIBS_DIR}/lib/"*.a 2>/dev/null || echo "No .a files found"
echo ""
echo "Headers in: ${LIBS_DIR}/include/"
