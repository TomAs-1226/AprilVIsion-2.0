#!/bin/bash
#
# FRC Vision Mac Simulator - Dependency Installation Script
#
# This script installs all dependencies needed to build and run
# the Mac simulator on macOS using Homebrew.
#
# Usage:
#   ./scripts/install_mac.sh
#
# Requirements:
#   - macOS 12 (Monterey) or later
#   - Xcode Command Line Tools
#   - Homebrew (will be installed if not present)
#

set -e

echo "========================================"
echo "FRC Vision Mac Simulator Installer"
echo "========================================"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Setup Homebrew environment
setup_homebrew_env() {
    if [[ -f "/opt/homebrew/bin/brew" ]]; then
        eval "$(/opt/homebrew/bin/brew shellenv)"
    elif [[ -f "/usr/local/bin/brew" ]]; then
        eval "$(/usr/local/bin/brew shellenv)"
    fi
}

print_status() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[!]${NC} $1"
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

# Check if running on macOS
if [[ "$(uname)" != "Darwin" ]]; then
    print_error "This script is for macOS only."
    exit 1
fi

# Check macOS version
MACOS_VERSION=$(sw_vers -productVersion)
MACOS_MAJOR=$(echo $MACOS_VERSION | cut -d. -f1)
if [[ $MACOS_MAJOR -lt 12 ]]; then
    print_warning "macOS 12 or later recommended. You have: $MACOS_VERSION"
fi

echo "Detected macOS $MACOS_VERSION"
echo ""

# ============================================================================
# Install Xcode Command Line Tools
# ============================================================================
echo "Checking Xcode Command Line Tools..."

if ! xcode-select -p &>/dev/null; then
    print_warning "Xcode Command Line Tools not installed."
    echo "Installing Xcode Command Line Tools..."
    xcode-select --install

    echo ""
    echo "Please complete the Xcode installation dialog that appeared."
    echo "Then re-run this script."
    exit 0
else
    print_status "Xcode Command Line Tools installed"
fi

# Check compiler
if ! command -v clang++ &>/dev/null; then
    print_error "clang++ not found. Please install Xcode Command Line Tools."
    exit 1
fi

CLANG_VERSION=$(clang++ --version | head -n1)
print_status "Compiler: $CLANG_VERSION"

# ============================================================================
# Install Homebrew
# ============================================================================
echo ""
echo "Checking Homebrew..."

# Try to setup Homebrew env first (in case it's installed but not in PATH)
setup_homebrew_env

if ! command -v brew &>/dev/null; then
    print_warning "Homebrew not installed."
    echo "Installing Homebrew..."
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

    # Add Homebrew to PATH for this session
    setup_homebrew_env
else
    print_status "Homebrew installed"
fi

# Update Homebrew (non-fatal if it fails)
echo "Updating Homebrew..."
brew update || print_warning "Homebrew update failed (network issue?), continuing anyway..."

# ============================================================================
# Install Dependencies
# ============================================================================
echo ""
echo "Installing dependencies..."

PACKAGES=(
    "cmake"
    "pkg-config"
    "opencv"
    "yaml-cpp"
)

for pkg in "${PACKAGES[@]}"; do
    if brew list "$pkg" &>/dev/null; then
        print_status "$pkg already installed"
    else
        echo "Installing $pkg..."
        brew install "$pkg"
        print_status "$pkg installed"
    fi
done

# ============================================================================
# Verify Installation
# ============================================================================
echo ""
echo "Verifying installation..."

# Check CMake
if command -v cmake &>/dev/null; then
    CMAKE_VERSION=$(cmake --version | head -n1)
    print_status "CMake: $CMAKE_VERSION"
else
    print_error "CMake not found"
    exit 1
fi

# Check OpenCV
if pkg-config --exists opencv4; then
    OPENCV_VERSION=$(pkg-config --modversion opencv4)
    print_status "OpenCV: $OPENCV_VERSION"
else
    print_error "OpenCV not found"
    exit 1
fi

# Check yaml-cpp
if pkg-config --exists yaml-cpp; then
    YAML_VERSION=$(pkg-config --modversion yaml-cpp)
    print_status "yaml-cpp: $YAML_VERSION"
else
    print_error "yaml-cpp not found"
    exit 1
fi

# ============================================================================
# Print Build Instructions
# ============================================================================
echo ""
echo "========================================"
echo "Installation Complete!"
echo "========================================"
echo ""
echo "To build the Mac simulator:"
echo ""
echo "  mkdir build && cd build"
echo "  cmake .. -DBUILD_MAC_SIM=ON"
echo "  make -j$(sysctl -n hw.ncpu)"
echo ""
echo "Or use the convenience script:"
echo ""
echo "  ./scripts/build_mac.sh"
echo ""
echo "To run the simulator:"
echo ""
echo "  ./build/frc_vision_sim"
echo ""
echo "Or:"
echo ""
echo "  ./scripts/run_sim.sh"
echo ""
echo "========================================"
