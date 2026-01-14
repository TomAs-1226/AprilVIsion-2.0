# FRC AprilTag Vision Coprocessor v2.0

Production-grade, ultra-low-latency AprilTag vision system for FRC 2026.
Designed for **Orange Pi 5 (RK3588S, 5GB)** running **Ubuntu-Rockchip** (Joshua Riek).

## Features

- **3 Camera Support** - Simultaneous capture at 60-120 FPS (640x480 or 800x600)
- **Multi-Tag Pose Estimation** - Accurate robot localization using PnP with RANSAC
- **Fast Motion Tracking** - Alpha-beta filter maintains pose during motion blur/dropouts
- **NetworkTables 4** - Full roboRIO integration with WPILib ntcore
- **Web Dashboard** - Live MJPEG streams with canvas overlays, real-time metrics
- **Hot Reload** - Change settings without restart via web API

## Quick Start

### 1. Install Dependencies

```bash
git clone <repo-url> frc-vision
cd frc-vision
sudo ./scripts/install_deps.sh
```

Log out and log back in for camera permissions to take effect.

### 2. Build

```bash
mkdir build && cd build
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
ninja
```

### 3. Run

```bash
./frc_vision
# Or from project root:
./scripts/run.sh
```

### 4. Access Dashboard

Open in browser: `http://<orange-pi-ip>:5800`

## NetworkTables Setup

### Configure roboRIO IP

Edit `config/config.yml`:

```yaml
outputs:
  nt_enable: true
  nt_server: "10.TE.AM.2"  # Replace TE.AM with your team number
```

### Verify Connection

1. Start the coprocessor
2. Check logs for "NT: Connected"
3. Use OutlineViewer or Shuffleboard to see `/FRCVision/*` tables

### NT4 Data Schema

| Topic | Type | Description |
|-------|------|-------------|
| `/FRCVision/status/uptime` | double | System uptime (seconds) |
| `/FRCVision/cam{i}/timestamp_capture` | double | Frame capture time (epoch) |
| `/FRCVision/cam{i}/tag_ids` | int[] | Detected tag IDs |
| `/FRCVision/cam{i}/corners_px` | double[] | [id, x1,y1..x4,y4, ...] |
| `/FRCVision/cam{i}/pose_robot` | double[] | [x, y, theta] |
| `/FRCVision/cam{i}/std_devs` | double[] | [σx, σy, σθ] |
| `/FRCVision/fused/pose` | double[] | Filtered [x, y, theta] |
| `/FRCVision/fused_raw/pose` | double[] | Unfiltered pose |
| `/FRCVision/fused/valid` | boolean | Pose validity |

## Camera Setup

### Device Paths

USB cameras typically appear as `/dev/video0`, `/dev/video2`, `/dev/video4`, etc.

List cameras:
```bash
v4l2-ctl --list-devices
```

### Permissions

If you see "permission denied" errors:

```bash
# Add user to video group
sudo usermod -a -G video $USER
# Log out and back in

# Or run manually:
sudo chmod 666 /dev/video*
```

### Camera Calibration

1. Print a checkerboard pattern (e.g., 9x6 inner corners)
2. Use OpenCV's calibration tool or capture images:
   ```bash
   # Use any camera capture tool to save 20+ images
   # from different angles
   ```
3. Run calibration script (or use MATLAB/OpenCV)
4. Save results to `config/cam{i}_intrinsics.yml`

## Configuration

### config/config.yml

```yaml
cameras:
  - name: front
    device: /dev/video0
    width: 640
    height: 480
    fps: 60
    intrinsics: cam0_intrinsics.yml
    extrinsics:
      x: 0.25      # Forward from robot center (m)
      y: 0.0       # Left from robot center (m)
      z: 0.50      # Up from floor (m)
      roll: 0      # Rotation around X (deg)
      pitch: 0     # Rotation around Y (deg)
      yaw: 0       # Rotation around Z (deg)

apriltag:
  family: tag36h11
  decimation: 2      # 1=full res, 2=half (faster), 4=quarter
  min_margin: 20     # Detection quality threshold
  max_tags_per_frame: 16

tracking:
  enable: true
  filter_alpha: 0.3  # Pose smoothing (lower=smoother)
```

### Performance Tuning

| Setting | Faster Detection | Better Accuracy |
|---------|------------------|-----------------|
| `decimation` | 3-4 | 1-2 |
| `refine_edges` | false | true |
| `min_margin` | 10 | 25+ |
| `resolution` | 640x480 | 800x600+ |
| `fps` | 120 | 60 |

For fast robot motion:
- Use `decimation: 2` for balance of speed and accuracy
- Enable tracking with `filter_alpha: 0.2-0.3`
- Set `dropout_ms: 150` to maintain pose during brief occlusions

## Field Layout

Edit `config/field_layout.json` with your field's AprilTag positions.

Default includes FRC 2024/2025-style tags. Update for 2026 when official layout is released.

## Service Installation

### Install as systemd service

```bash
# Build first
mkdir build && cd build && cmake .. -GNinja && ninja

# Full installation (recommended)
sudo ./scripts/install_service.sh
```

The install script:
1. Creates dedicated `frcvision` system user
2. Sets up udev rules for camera access (`/dev/video*`)
3. Deploys binary and config to `/opt/frc-vision`
4. Installs and enables the systemd service
5. Configures resource limits for real-time performance

### Service Commands

```bash
sudo systemctl start frc_vision   # Start
sudo systemctl stop frc_vision    # Stop
sudo systemctl restart frc_vision # Restart
sudo systemctl status frc_vision  # Status
sudo journalctl -u frc_vision -f  # Live logs
```

## Boot & Startup

### Fast Boot Design

The coprocessor is designed for **sub-65-second readiness** from power-on:

- **Immediate web server start** - Dashboard available within seconds
- **Async camera initialization** - Cameras connect in background with retry
- **Non-blocking NetworkTables** - Connects when roboRIO becomes available
- **Degraded mode support** - Continues operating with partial functionality

### Startup Sequence

1. Systemd starts service immediately after network target
2. Web server starts on port 5800
3. Configuration loaded from `/opt/frc-vision/config/config.yml`
4. Cameras initialize asynchronously (retries every 2s if unavailable)
5. NetworkTables connects in background (retries automatically)
6. System logs `[READY]` when at least 1 camera + NT initialized

### Check System Status

```bash
# Service status
systemctl status frc_vision

# Live logs
journalctl -u frc_vision -f

# Verify readiness via API
curl http://localhost:5800/api/status
```

### Readiness API

The `/api/status` endpoint returns JSON:

```json
{
  "ready": true,
  "state": "ready",
  "uptime_seconds": 12.5,
  "cameras_connected": 3,
  "cameras_expected": 3,
  "nt_connected": true,
  "web_ready": true
}
```

**Ready state**: `ready=true` when at least 1 camera is streaming AND web server is running.

### Troubleshooting Boot Issues

| Symptom | Check | Fix |
|---------|-------|-----|
| Service not starting | `systemctl status frc_vision` | Check logs for errors |
| No cameras | `ls -la /dev/video*` | Run `install_service.sh` for udev rules |
| NT not connecting | `curl localhost:5800/api/status` | Verify roboRIO IP in config |
| Slow startup | `journalctl -u frc_vision` | Ensure raw IP (not hostname) for NT |

### Direct Ethernet to roboRIO

For lowest latency, connect Orange Pi directly to roboRIO's 2nd Ethernet port:

1. Configure static IP on Orange Pi: `10.TE.AM.11` (e.g., `10.12.34.11` for team 1234)
2. Set roboRIO 2nd port: `10.TE.AM.2`
3. Update `config/config.yml`:
   ```yaml
   outputs:
     nt_server: "10.TE.AM.2"  # Use raw IP, never mDNS hostname
   ```

**Important**: Always use raw IP addresses, never `roborio-TEAM-frc.local` (mDNS causes boot delays)

## Web Dashboard

### Features

- Live MJPEG video from all cameras
- Tag detection overlays (corners, IDs, margins)
- Fused robot pose visualization on field
- Real-time latency and FPS metrics
- Configuration controls

### Endpoints

| URL | Description |
|-----|-------------|
| `/` | Dashboard |
| `/cam0.mjpeg` | Camera 0 stream |
| `/events` | Server-Sent Events (JSON data) |
| `/api/config` | GET/POST configuration |
| `/api/config/reload` | Reload from file |

## Architecture

```
┌─────────────┐  ┌─────────────┐  ┌─────────────┐
│  Camera 0   │  │  Camera 1   │  │  Camera 2   │
│  Capture    │  │  Capture    │  │  Capture    │
└──────┬──────┘  └──────┬──────┘  └──────┬──────┘
       │                │                │
       ▼                ▼                ▼
┌──────────────────────────────────────────────┐
│            Ring Buffers (SPSC)               │
│         "Latest Frame Wins" Policy           │
└──────────────────────────────────────────────┘
       │                │                │
       ▼                ▼                ▼
┌─────────────┐  ┌─────────────┐  ┌─────────────┐
│  Detector   │  │  Detector   │  │  Detector   │
│  + Tracker  │  │  + Tracker  │  │  + Tracker  │
│  + Pose     │  │  + Pose     │  │  + Pose     │
└──────┬──────┘  └──────┬──────┘  └──────┬──────┘
       │                │                │
       └────────────────┼────────────────┘
                        ▼
               ┌─────────────────┐
               │   Multi-Camera  │
               │     Fusion      │
               └────────┬────────┘
                        │
          ┌─────────────┼─────────────┐
          ▼             ▼             ▼
   ┌────────────┐ ┌────────────┐ ┌────────────┐
   │ NT4 Pub    │ │ Web Server │ │  Console   │
   │ (roboRIO)  │ │ (Dashboard)│ │   Status   │
   └────────────┘ └────────────┘ └────────────┘
```

## Troubleshooting

### No cameras detected

```bash
# List devices
ls -la /dev/video*

# Check if loaded
lsmod | grep uvcvideo

# Reload driver
sudo modprobe -r uvcvideo && sudo modprobe uvcvideo
```

### Low FPS

1. Reduce resolution: 640x480 instead of 1280x720
2. Increase decimation: 3-4 for faster detection
3. Check USB bandwidth: use USB 3.0 ports
4. Disable auto-exposure and set manual exposure

### NetworkTables not connecting

1. Verify roboRIO IP in config
2. Check network connectivity: `ping 10.TE.AM.2`
3. Ensure roboRIO is running and NT4 server is active
4. Check firewall settings

### High latency

1. Set `CAP_PROP_BUFFERSIZE=1` (done automatically)
2. Use MJPEG format instead of YUYV
3. Reduce JPEG quality for streaming
4. Increase detection threads

## Project Structure

```
/
├── CMakeLists.txt           # Build configuration (supports Orange Pi + Mac)
├── README.md                # This file
├── src/
│   ├── main.cpp            # Orange Pi entry point
│   ├── types.hpp           # Core data types
│   ├── ring_buffer.hpp     # Lock-free SPSC buffer
│   ├── config.hpp/cpp      # YAML configuration
│   ├── camera.hpp/cpp      # Multi-camera capture
│   ├── detector.hpp/cpp    # AprilTag detection
│   ├── tracker.hpp/cpp     # Motion tracking
│   ├── pose.hpp/cpp        # Pose estimation
│   ├── fusion.hpp/cpp      # Multi-camera fusion
│   ├── nt_publisher.hpp/cpp # NetworkTables 4
│   ├── web_server.hpp/cpp  # HTTP + SSE
│   ├── field_layout.hpp/cpp # Field tag positions
│   ├── platform/
│   │   └── frame_source.hpp # Platform abstraction interface
│   └── sim/                 # Mac Simulator
│       ├── sim_main.cpp     # Simulator entry point
│       ├── simulator.hpp/cpp # Main simulator orchestration
│       ├── sim_types.hpp    # Simulator data types
│       ├── robot_dynamics.hpp/cpp # WASD robot physics
│       ├── field_renderer.hpp/cpp # Synthetic tag rendering
│       └── auto_align.hpp/cpp # PID auto-align controller
├── assets/                  # Simulator assets
│   ├── 2024-crescendo.json  # WPILib field layout
│   ├── mac_cam_intrinsics.yml # Mac webcam calibration
│   └── mac_cam_extrinsics.yml # Camera mount transform
├── web/
│   ├── index.html          # Dashboard
│   ├── app.js              # Dashboard logic
│   └── style.css           # Dashboard styles
├── config/
│   ├── config.yml          # Orange Pi configuration
│   ├── sim_config.yml      # Mac Simulator configuration
│   ├── cam*_intrinsics.yml # Camera calibration
│   └── field_layout.json   # Tag positions
├── scripts/
│   ├── install_deps.sh     # Orange Pi dependency installer
│   ├── install_mac.sh      # Mac dependency installer
│   ├── build_mac.sh        # Mac build script
│   ├── run_sim.sh          # Run simulator
│   ├── fetch_layout.sh     # Download field layout
│   ├── run.sh              # Orange Pi run script
│   └── install_service.sh  # Production deploy
└── deploy/
    └── frc_vision.service  # systemd service
```

## License

MIT License - Free for FRC teams and educational use.

## Credits

- AprilTag library by AprilRobotics
- WPILib for NetworkTables
- cpp-httplib for web server
- Orange Pi community for hardware support

---

## Mac Simulator

The Mac simulator allows testing and validation of the vision system on a MacBook without requiring actual robot hardware. It uses the same core vision pipeline as the Orange Pi build.

### Features

- **2024 CRESCENDO Field Layout** - All 16 AprilTags in correct positions
- **Robot Dynamics** - WASD movement with realistic physics and odometry drift
- **Synthetic Tag Rendering** - Perspective-correct AprilTag rendering
- **Webcam Compositing** - Overlay synthetic tags on live webcam feed
- **Three Pose Views** - Ground truth, odometry (drifting), and vision-corrected
- **Auto-Align Demo** - PID controller to align with nearest tag
- **Full Pipeline** - Same detection, pose estimation, and fusion as production

### Quick Start (macOS)

#### 1. Install Dependencies

```bash
# Run the install script (installs Xcode tools, Homebrew, OpenCV, etc.)
./scripts/install_mac.sh
```

**Required tools installed:**
- Xcode Command Line Tools (Apple Clang compiler)
- Homebrew package manager
- CMake 3.16+
- OpenCV 4.x with highgui
- yaml-cpp
- pkg-config

#### 2. Build

```bash
# Using the convenience script:
./scripts/build_mac.sh

# Or manually:
mkdir build && cd build
cmake .. -DBUILD_MAC_SIM=ON
make -j$(sysctl -n hw.ncpu)
```

#### 3. Run

```bash
# Using the convenience script:
./scripts/run_sim.sh

# Or directly:
./build/frc_vision_sim

# Without webcam (synthetic only):
./build/frc_vision_sim --no-webcam
```

### Controls

| Key | Action |
|-----|--------|
| **W/A/S/D** | Move robot (forward/left/back/right) |
| **Q/E** | Rotate CCW/CW |
| **Shift** | Turbo mode (faster movement) |
| **V** | Toggle auto-align to nearest tag |
| **R** | Reset robot pose |
| **1** | Toggle true pose (green) |
| **2** | Toggle odometry pose (orange) |
| **3** | Toggle fused pose (magenta) |
| **4** | Toggle webcam composite |
| **5** | Toggle detection source (composite vs synthetic) |
| **ESC** | Quit |

### Windows

The simulator opens two windows:

1. **Camera View** - Shows the synthetic/composite camera feed with detected tag overlays
2. **Field View** - Top-down view of the field showing robot poses and all tags

### Configuration

Edit `config/sim_config.yml`:

```yaml
dynamics:
  max_speed: 4.0         # m/s
  max_turbo_speed: 5.5   # m/s with Shift
  odom_noise: 0.02       # Odometry noise factor

auto_align:
  standoff_distance: 1.0 # meters from tag when aligned

camera:
  width: 640
  height: 480
  add_noise: false       # Simulate sensor noise
  motion_blur: false     # Simulate motion blur

start_pose:
  x: 2.0                 # Starting X position (meters)
  y: 2.0                 # Starting Y position (meters)
  theta: 0.0             # Starting heading (radians)

webcam:
  enable: true           # Use MacBook webcam
  device: 0              # Camera device index
```

### Assets

The simulator uses these asset files:

| File | Description |
|------|-------------|
| `assets/2024-crescendo.json` | WPILib field layout with all 16 tags |
| `assets/mac_cam_intrinsics.yml` | Camera calibration (default for MacBook webcam) |
| `assets/mac_cam_extrinsics.yml` | Camera mount position on robot |

To download the official field layout:
```bash
./scripts/fetch_layout.sh
```

### Camera Calibration

The default intrinsics are approximate for a MacBook webcam. For best accuracy:

1. Print a checkerboard calibration pattern
2. Use OpenCV's `cv::calibrateCamera()` with 20+ images
3. Save results to `assets/mac_cam_intrinsics.yml`:

```yaml
%YAML:1.0
camera_matrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ fx, 0, cx, 0, fy, cy, 0, 0, 1 ]
distortion_coefficients: !!opencv-matrix
   rows: 5
   cols: 1
   dt: d
   data: [ k1, k2, p1, p2, k3 ]
```

### Architecture

The Mac simulator uses the same core modules as the Orange Pi production build:

```
Mac Simulator                       Orange Pi Build
--------------                      ---------------
sim_main.cpp                        main.cpp
    │                                   │
    ▼                                   ▼
┌──────────────┐                   ┌──────────────┐
│  Simulator   │                   │   Camera     │
│  (WASD/QE)   │                   │   Capture    │
└──────┬───────┘                   └──────┬───────┘
       │                                  │
       ▼                                  ▼
┌──────────────┐                   ┌──────────────┐
│  Field       │                   │              │
│  Renderer    │                   │              │
└──────┬───────┘                   │              │
       │                           │              │
       ▼                           │              │
┌──────────────────────────────────┴──────────────┐
│                 SHARED CORE                      │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐          │
│  │Detector │  │ Tracker │  │  Pose   │          │
│  └────┬────┘  └────┬────┘  └────┬────┘          │
│       └────────────┴────────────┘               │
│                    │                             │
│                    ▼                             │
│              ┌──────────┐                        │
│              │  Fusion  │                        │
│              └────┬─────┘                        │
│                   │                              │
│       ┌───────────┼───────────┐                 │
│       ▼           ▼           ▼                 │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐           │
│  │ NT Pub  │ │Web Serv │ │ Status  │           │
│  └─────────┘ └─────────┘ └─────────┘           │
└─────────────────────────────────────────────────┘
```

### Validation Use Cases

1. **Pose Correction** - Watch the fused pose (magenta) correct towards true pose (green) while odometry (orange) drifts
2. **Fast Motion Tracking** - Move quickly and observe dropout recovery
3. **Auto-Align** - Press V and watch the robot converge to face the nearest tag
4. **Multi-Tag Accuracy** - Position robot to see multiple tags for better pose estimates

### Troubleshooting

| Issue | Solution |
|-------|----------|
| No webcam | Run with `--no-webcam` or check camera permissions in System Preferences |
| Build fails | Run `./scripts/install_mac.sh` to install dependencies |
| Field layout missing | Run `./scripts/fetch_layout.sh` |
| Window not appearing | Ensure OpenCV highgui is installed (`brew reinstall opencv`) |
| Low FPS | Reduce camera resolution in config, or disable motion blur |

---

**FRC 2026 Season Ready!**
