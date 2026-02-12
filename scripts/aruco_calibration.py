#!/usr/bin/env python3
"""
AprilVision 3.2 - ArUco Calibration Helper

Assists with camera calibration using ChArUco boards.
- Generate a ChArUco board image for printing
- Check calibration status for cameras in the detection engine

Usage:
    python3 scripts/aruco_calibration.py --generate
    python3 scripts/aruco_calibration.py --generate --output board.png
    python3 scripts/aruco_calibration.py --check
    python3 scripts/aruco_calibration.py --instructions
"""

import argparse
import json
import os
import sys
import glob

# Colors for terminal output
CYAN = "\033[0;36m"
GREEN = "\033[0;32m"
YELLOW = "\033[1;33m"
RED = "\033[0;31m"
NC = "\033[0m"

CONFIG_DIR = "/opt/photonvision/photonvision_config"
CAMERAS_DIR = os.path.join(CONFIG_DIR, "cameras")


def print_banner():
    print()
    print(f"{CYAN}AprilVision 3.2 - ArUco Calibration Helper{NC}")
    print("=" * 44)
    print()


def generate_charuco_board(output_path="charuco_board.png", squares_x=9, squares_y=6,
                           square_length=0.030, marker_length=0.022):
    """Generate a ChArUco board image using OpenCV, or print instructions if unavailable."""
    try:
        import cv2
        import cv2.aruco as aruco

        # Create the ChArUco board
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        board = aruco.CharucoBoard(
            (squares_x, squares_y),
            square_length,
            marker_length,
            aruco_dict
        )

        # Generate the board image
        img_size = (int(squares_x * 100), int(squares_y * 100))
        board_image = board.generateImage(img_size, marginSize=20)

        cv2.imwrite(output_path, board_image)
        print(f"{GREEN}ChArUco board generated:{NC} {output_path}")
        print()
        print("Board parameters:")
        print(f"  Squares:        {squares_x} x {squares_y}")
        print(f"  Square size:    {square_length * 1000:.0f} mm")
        print(f"  Marker size:    {marker_length * 1000:.0f} mm")
        print(f"  Dictionary:     DICT_5X5_100")
        print()
        print(f"{YELLOW}Instructions:{NC}")
        print("  1. Print the board image at 100% scale (no scaling)")
        print("  2. Mount it on a flat, rigid surface (cardboard or foam board)")
        print("  3. Measure the actual printed square size with calipers")
        print("  4. Use the measured size when calibrating in the dashboard")
        print()
        return True

    except ImportError:
        print(f"{YELLOW}OpenCV (cv2) is not installed.{NC}")
        print()
        print("To install OpenCV for board generation:")
        print(f"  {CYAN}pip3 install opencv-contrib-python{NC}")
        print()
        print("Alternatively, you can download a pre-made ChArUco board:")
        print()
        print("  Option 1: Use the dashboard's built-in calibration board generator")
        print(f"            Open {CYAN}http://<coprocessor-ip>:5801{NC}")
        print("            Go to Cameras > Calibration")
        print()
        print("  Option 2: Generate one online at:")
        print(f"            {CYAN}https://calib.io/pages/camera-calibration-pattern-generator{NC}")
        print("            Select: ChArUco, 9x6, DICT_5X5_100")
        print()
        print("  Recommended board parameters:")
        print("    Squares:     9 x 6")
        print("    Square size: 30 mm")
        print("    Marker size: 22 mm")
        print("    Dictionary:  DICT_5X5_100")
        print()
        return False


def check_calibration_status():
    """Check if cameras have been calibrated in the detection engine."""
    print("Checking camera calibration status...")
    print()

    if not os.path.isdir(CONFIG_DIR):
        print(f"{YELLOW}Detection engine config directory not found:{NC} {CONFIG_DIR}")
        print("Make sure the detection engine has been installed (run ./setup.sh)")
        return

    if not os.path.isdir(CAMERAS_DIR):
        print(f"{YELLOW}No cameras directory found:{NC} {CAMERAS_DIR}")
        print("The detection engine may not have been started yet, or no cameras configured.")
        print()
        print("Start the engine and open the dashboard to configure cameras:")
        print(f"  sudo systemctl start photonvision")
        print(f"  Open {CYAN}http://<coprocessor-ip>:5801{NC}")
        return

    # Look for camera config directories
    camera_dirs = glob.glob(os.path.join(CAMERAS_DIR, "*"))

    if not camera_dirs:
        print(f"{YELLOW}No cameras configured yet.{NC}")
        print("Open the AprilVision dashboard to add cameras:")
        print(f"  {CYAN}http://<coprocessor-ip>:5801{NC}")
        return

    calibrated_count = 0
    total_count = 0

    for cam_dir in sorted(camera_dirs):
        if not os.path.isdir(cam_dir):
            continue

        cam_name = os.path.basename(cam_dir)
        total_count += 1

        # Look for calibration data files
        cal_files = glob.glob(os.path.join(cam_dir, "calibration*.json"))
        cal_files += glob.glob(os.path.join(cam_dir, "*calib*.json"))

        # Also check for camera config with calibration data
        config_file = os.path.join(cam_dir, "config.json")
        has_calibration = False

        if cal_files:
            has_calibration = True
        elif os.path.isfile(config_file):
            try:
                with open(config_file, "r") as f:
                    config = json.load(f)
                # Check if calibration data exists in the config
                if "calibrations" in config and len(config["calibrations"]) > 0:
                    has_calibration = True
                elif "cameraCalibrations" in config and len(config["cameraCalibrations"]) > 0:
                    has_calibration = True
            except (json.JSONDecodeError, KeyError, IOError):
                pass

        if has_calibration:
            calibrated_count += 1
            resolutions = []
            if os.path.isfile(config_file):
                try:
                    with open(config_file, "r") as f:
                        config = json.load(f)
                    cals = config.get("calibrations", config.get("cameraCalibrations", []))
                    for cal in cals:
                        w = cal.get("resolution", {}).get("width", cal.get("width", "?"))
                        h = cal.get("resolution", {}).get("height", cal.get("height", "?"))
                        resolutions.append(f"{w}x{h}")
                except (json.JSONDecodeError, KeyError, IOError):
                    pass

            res_str = f" ({', '.join(resolutions)})" if resolutions else ""
            print(f"  {GREEN}CALIBRATED{NC}    {cam_name}{res_str}")
        else:
            print(f"  {RED}NOT CALIBRATED{NC}  {cam_name}")

    print()
    print(f"  {calibrated_count}/{total_count} cameras calibrated")
    print()

    if calibrated_count < total_count:
        print(f"{YELLOW}Uncalibrated cameras will produce inaccurate pose estimates.{NC}")
        print()
        print("To calibrate a camera:")
        print(f"  1. Open the AprilVision dashboard: {CYAN}http://<coprocessor-ip>:5801{NC}")
        print("  2. Select the camera")
        print("  3. Go to the Calibration tab")
        print("  4. Print a calibration board (use --generate to create one)")
        print("  5. Follow the on-screen calibration instructions")
        print("  6. Collect at least 12 images from different angles")
        print("  7. Click 'Calibrate' and verify the reprojection error is < 1.0 px")
        print()
    else:
        print(f"{GREEN}All cameras are calibrated. Ready for accurate pose estimation.{NC}")
        print()


def print_instructions():
    """Print detailed calibration instructions."""
    print("Camera Calibration Guide")
    print("-" * 40)
    print()
    print("Accurate camera calibration is essential for reliable 3D pose estimation.")
    print("Without calibration, AprilTag distance and angle measurements will be wrong.")
    print()
    print(f"{CYAN}Step 1: Generate a Calibration Board{NC}")
    print("  python3 scripts/aruco_calibration.py --generate")
    print("  Print the board at 100% scale on letter/A4 paper.")
    print("  Mount on a flat rigid surface (cardboard works).")
    print()
    print(f"{CYAN}Step 2: Measure the Printed Board{NC}")
    print("  Use calipers to measure the actual printed square size.")
    print("  Printing often scales slightly - the actual measurement matters.")
    print()
    print(f"{CYAN}Step 3: Open the Dashboard{NC}")
    print("  Navigate to http://<coprocessor-ip>:5801")
    print("  Select the camera you want to calibrate.")
    print("  Go to the Calibration tab.")
    print()
    print(f"{CYAN}Step 4: Configure Calibration Settings{NC}")
    print("  - Set the board dimensions (9x6 for the default board)")
    print("  - Enter the measured square size in millimeters")
    print("  - Select the resolution you'll use for detection (640x480)")
    print()
    print(f"{CYAN}Step 5: Capture Calibration Images{NC}")
    print("  - Hold the board in front of the camera")
    print("  - Capture at least 12 images from different:")
    print("    * Distances (close, medium, far)")
    print("    * Angles (tilted left, right, up, down)")
    print("    * Positions (all four corners of the frame)")
    print("  - The board should fill 30-80% of the frame")
    print("  - Keep the board still during each capture")
    print()
    print(f"{CYAN}Step 6: Run Calibration{NC}")
    print("  - Click 'Calibrate' in the dashboard")
    print("  - Wait for processing (may take 30-60 seconds)")
    print("  - Check the reprojection error:")
    print(f"    * {GREEN}< 0.5 px{NC}  - Excellent")
    print(f"    * {GREEN}< 1.0 px{NC}  - Good (acceptable for FRC)")
    print(f"    * {YELLOW}1.0 - 2.0 px{NC} - Fair (consider recalibrating)")
    print(f"    * {RED}> 2.0 px{NC}  - Poor (recalibrate with better images)")
    print()
    print(f"{CYAN}Step 7: Verify{NC}")
    print("  python3 scripts/aruco_calibration.py --check")
    print()
    print("Tips:")
    print("  - Calibrate at the same resolution you'll use for detection")
    print("  - Recalibrate if you change the camera lens or focus")
    print("  - Good lighting helps - avoid glare on the board")
    print("  - Each camera needs its own calibration")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="AprilVision 3.2 - ArUco Calibration Helper",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 scripts/aruco_calibration.py --generate
  python3 scripts/aruco_calibration.py --generate --output my_board.png
  python3 scripts/aruco_calibration.py --check
  python3 scripts/aruco_calibration.py --instructions
        """,
    )

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--generate",
        action="store_true",
        help="Generate a ChArUco calibration board image",
    )
    group.add_argument(
        "--check",
        action="store_true",
        help="Check calibration status for all configured cameras",
    )
    group.add_argument(
        "--instructions",
        action="store_true",
        help="Print detailed calibration instructions",
    )

    parser.add_argument(
        "--output",
        default="charuco_board.png",
        help="Output path for generated board image (default: charuco_board.png)",
    )
    parser.add_argument(
        "--squares-x",
        type=int,
        default=9,
        help="Number of squares in X direction (default: 9)",
    )
    parser.add_argument(
        "--squares-y",
        type=int,
        default=6,
        help="Number of squares in Y direction (default: 6)",
    )

    args = parser.parse_args()

    print_banner()

    if args.generate:
        generate_charuco_board(
            output_path=args.output,
            squares_x=args.squares_x,
            squares_y=args.squares_y,
        )
    elif args.check:
        check_calibration_status()
    elif args.instructions:
        print_instructions()


if __name__ == "__main__":
    main()
