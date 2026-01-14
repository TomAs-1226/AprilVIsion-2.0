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

# Deploy
sudo ./scripts/deploy.sh
```

### Service Commands

```bash
sudo systemctl start frc_vision   # Start
sudo systemctl stop frc_vision    # Stop
sudo systemctl restart frc_vision # Restart
sudo systemctl status frc_vision  # Status
sudo journalctl -u frc_vision -f  # Live logs
```

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
├── CMakeLists.txt           # Build configuration
├── README.md                # This file
├── src/
│   ├── main.cpp            # Entry point
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
│   └── field_layout.hpp/cpp # Field tag positions
├── web/
│   ├── index.html          # Dashboard
│   ├── app.js              # Dashboard logic
│   └── style.css           # Dashboard styles
├── config/
│   ├── config.yml          # Main configuration
│   ├── cam*_intrinsics.yml # Camera calibration
│   └── field_layout.json   # Tag positions
├── scripts/
│   ├── install_deps.sh     # Dependency installer
│   ├── run.sh              # Run script
│   └── deploy.sh           # Production deploy
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

**FRC 2026 Season Ready!**
