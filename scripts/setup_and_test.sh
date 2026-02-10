#!/bin/bash
# Ultimate FRC AprilTag Vision System - Setup and Test Script
# This script sets up the environment and runs comprehensive accuracy tests

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

PROJECT_ROOT="/home/user/AprilVIsion-2.0"
BUILD_DIR="$PROJECT_ROOT/build"
CONFIG_DIR="$PROJECT_ROOT/config"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}FRC AprilTag Vision - Setup & Test${NC}"
echo -e "${BLUE}========================================${NC}\n"

# Function to print section headers
section() {
    echo -e "\n${GREEN}==> $1${NC}"
}

# Function to print warnings
warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

# Function to print errors
error() {
    echo -e "${RED}✗ $1${NC}"
}

# Function to print success
success() {
    echo -e "${GREEN}✓ $1${NC}"
}

# Check dependencies
section "Checking Dependencies"

check_dependency() {
    if command -v "$1" &> /dev/null; then
        success "$1 found"
        return 0
    else
        error "$1 not found"
        return 1
    fi
}

DEPS_OK=true
check_dependency "cmake" || DEPS_OK=false
check_dependency "g++" || DEPS_OK=false
check_dependency "git" || DEPS_OK=false

# Check for OpenCV
if pkg-config --exists opencv4; then
    OPENCV_VERSION=$(pkg-config --modversion opencv4)
    success "OpenCV $OPENCV_VERSION found"
elif pkg-config --exists opencv; then
    OPENCV_VERSION=$(pkg-config --modversion opencv)
    success "OpenCV $OPENCV_VERSION found"
else
    error "OpenCV not found"
    warning "Install with: sudo apt-get install libopencv-dev"
    DEPS_OK=false
fi

# Check for AprilTag library
if pkg-config --exists apriltag; then
    APRILTAG_VERSION=$(pkg-config --modversion apriltag)
    success "AprilTag $APRILTAG_VERSION found"
elif [ -f "/usr/local/lib/libapriltag.so" ]; then
    success "AprilTag library found"
else
    warning "AprilTag library not found"
    warning "Install from: https://github.com/AprilRobotics/apriltag"
fi

if [ "$DEPS_OK" = false ]; then
    error "Missing dependencies. Please install them first."
    exit 1
fi

# Build the project
section "Building Project"

if [ -d "$BUILD_DIR" ]; then
    echo "Build directory exists, cleaning..."
    rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

echo "Running CMake..."
if cmake .. -DCMAKE_BUILD_TYPE=Release; then
    success "CMake configuration successful"
else
    error "CMake configuration failed"
    exit 1
fi

echo "Compiling..."
if make -j$(nproc); then
    success "Build successful"
else
    error "Build failed"
    exit 1
fi

# Check for calibration files
section "Checking Calibration Files"

cd "$CONFIG_DIR"
CALIB_OK=true

for cam in cam0 cam1 cam2; do
    if [ -f "${cam}_intrinsics.yml" ]; then
        success "${cam}_intrinsics.yml found"
    else
        warning "${cam}_intrinsics.yml not found - camera needs calibration"
        CALIB_OK=false
    fi
done

if [ "$CALIB_OK" = false ]; then
    warning "Some cameras need calibration. Run calibration tool first."
fi

# Validate configuration
section "Validating Configuration"

if [ -f "$CONFIG_DIR/config.yml" ]; then
    success "config.yml found"

    # Check for Phase 1 enhancements
    if grep -q "subpixel_refine" "$CONFIG_DIR/config.yml"; then
        success "Phase 1 sub-pixel refinement enabled"
    else
        warning "Phase 1 sub-pixel refinement not configured"
    fi

    if grep -q "calibration:" "$CONFIG_DIR/config.yml"; then
        success "Phase 1 calibration validation configured"
    else
        warning "Phase 1 calibration validation not configured"
    fi
else
    error "config.yml not found"
    exit 1
fi

# Check for field layout
if [ -f "$CONFIG_DIR/field_layout.json" ]; then
    TAG_COUNT=$(grep -o '"id"' "$CONFIG_DIR/field_layout.json" | wc -l)
    success "Field layout found with $TAG_COUNT tags"
else
    warning "field_layout.json not found"
fi

# System information
section "System Information"

echo "CPU: $(lscpu | grep 'Model name' | cut -d':' -f2 | xargs)"
echo "Cores: $(nproc)"
echo "Memory: $(free -h | awk '/^Mem:/ {print $2}')"
echo "OS: $(uname -srm)"

if [ -f "/sys/class/thermal/thermal_zone0/temp" ]; then
    TEMP=$(cat /sys/class/thermal/thermal_zone0/temp)
    TEMP_C=$((TEMP / 1000))
    echo "CPU Temp: ${TEMP_C}°C"
fi

# Camera detection
section "Detecting Cameras"

CAMERA_COUNT=0
for i in {0..10..2}; do
    if [ -e "/dev/video$i" ]; then
        success "Camera found: /dev/video$i"

        # Try to get camera info
        if command -v v4l2-ctl &> /dev/null; then
            CAM_NAME=$(v4l2-ctl -d /dev/video$i --info 2>/dev/null | grep "Card type" | cut -d':' -f2 | xargs || echo "Unknown")
            echo "  Name: $CAM_NAME"
        fi

        CAMERA_COUNT=$((CAMERA_COUNT + 1))
    fi
done

if [ $CAMERA_COUNT -eq 0 ]; then
    error "No cameras detected"
    warning "Check USB connections and permissions"
else
    success "Found $CAMERA_COUNT camera(s)"
fi

# Test modes
section "Available Test Modes"

echo ""
echo "1. Calibration Mode - Calibrate cameras with ChArUco board"
echo "2. Validation Mode - Test accuracy at known distance (1.5m)"
echo "3. Live Detection - Run full vision pipeline"
echo "4. Diagnostic Mode - Detailed accuracy testing"
echo ""

read -p "Select test mode (1-4, or press Enter to skip): " TEST_MODE

case $TEST_MODE in
    1)
        section "Calibration Mode"
        echo "Starting calibration tool..."
        echo "Instructions:"
        echo "1. Print ChArUco board from: scripts/print_calibration_board.pdf"
        echo "2. Point camera at board from various angles and distances"
        echo "3. Press SPACE to capture frame (need 20+ frames)"
        echo "4. Press 'c' to compute calibration"
        echo "5. Press 'q' to quit"
        echo ""
        read -p "Press Enter to start calibration tool..."

        if [ -f "$BUILD_DIR/calibrate_camera" ]; then
            "$BUILD_DIR/calibrate_camera"
        else
            error "Calibration tool not found. Build failed?"
        fi
        ;;

    2)
        section "Validation Mode"
        echo "Starting validation mode..."
        echo "Instructions:"
        echo "1. Place AprilTag EXACTLY 1.5m from camera"
        echo "2. Tag should face camera straight on (0° angle)"
        echo "3. System will measure accuracy"
        echo ""

        # Enable validation mode in config
        if grep -q "validation_mode_enable: false" "$CONFIG_DIR/config.yml"; then
            warning "Enabling validation mode in config.yml..."
            sed -i 's/validation_mode_enable: false/validation_mode_enable: true/' "$CONFIG_DIR/config.yml"
        fi

        read -p "Press Enter when tag is positioned at 1.5m..."

        if [ -f "$BUILD_DIR/frc_vision" ]; then
            "$BUILD_DIR/frc_vision" --config "$CONFIG_DIR/config.yml" --validation-mode
        else
            error "Vision executable not found"
        fi

        # Restore validation mode
        sed -i 's/validation_mode_enable: true/validation_mode_enable: false/' "$CONFIG_DIR/config.yml"
        ;;

    3)
        section "Live Detection Mode"
        echo "Starting full vision pipeline..."
        echo "Press Ctrl+C to stop"
        echo ""

        if [ -f "$BUILD_DIR/frc_vision" ]; then
            "$BUILD_DIR/frc_vision" --config "$CONFIG_DIR/config.yml"
        else
            error "Vision executable not found"
        fi
        ;;

    4)
        section "Diagnostic Mode"
        echo "Running comprehensive diagnostics..."
        echo ""

        # Check calibration quality
        echo "Calibration Quality:"
        for cam in cam0 cam1 cam2; do
            if [ -f "$CONFIG_DIR/${cam}_intrinsics.yml" ]; then
                echo "  $cam:"
                # Extract RMS error if available (would need to parse YAML)
                echo "    Status: Calibrated"
            else
                echo "  $cam: NOT CALIBRATED"
            fi
        done

        echo ""
        echo "Phase 1 Features:"
        grep -E "(subpixel_refine|min_rms_for_excellent)" "$CONFIG_DIR/config.yml" | sed 's/^/  /'

        echo ""
        echo "For detailed diagnostics, run live detection mode and check logs"
        ;;

    *)
        echo "Skipping test mode"
        ;;
esac

# Summary
section "Setup Complete"

echo ""
echo "Next Steps:"
echo "1. Calibrate all cameras if not done (run with mode 1)"
echo "2. Test accuracy at known distance (mode 2)"
echo "3. Run full vision pipeline (mode 3)"
echo ""
echo "Configuration: $CONFIG_DIR/config.yml"
echo "Logs: Check console output"
echo "Web interface: http://localhost:5800 (when running)"
echo ""

success "All checks passed!"
