<p align="center">
  <img src="https://img.shields.io/badge/FRC-2026-blue?style=for-the-badge" alt="FRC 2026">
  <img src="https://img.shields.io/badge/AprilVision-v3.2.0-orange?style=for-the-badge" alt="AprilVision v3.2.0">
  <img src="https://img.shields.io/badge/Platform-ARM64%20%7C%20x86__64-green?style=for-the-badge" alt="Platform">
  <img src="https://img.shields.io/badge/License-MIT-purple?style=for-the-badge" alt="License">
</p>

# AprilVision 3.2

**Custom FRC vision system**

One command. Full AprilTag detection. Comp ready.

```bash
./setup.sh --team 1226
```

competition-ready vision system.

---

## Why AprilVision 3.2?

Most FRC teams spend hours wrestling with vision setup, debugging NetworkTables connections, and fighting service configurations. AprilVision 3.2 eliminates all of that.

| Problem | AprilVision 3.2 Solution |
|---------|--------------------------|
| Complex multi-step installs | Single `./setup.sh` command |
| Services don't survive reboot | Systemd auto-start + auto-recovery |
| No integrated dashboard | Custom branded dashboard at `:5801` |
| No robot code examples | Complete Java examples with multi-camera support |
| Manual network configuration | Auto-detects team number and roboRIO IP |
| No health monitoring | Built-in health check + match mode scripts |
| Debugging during competition | Camera snapshot diagnostics for post-match review |
| Camera calibration is confusing | ArUco calibration helper script |
| Network issues hard to diagnose | Built-in network diagnostics tool |

---

## Features

### Core Vision
- **AprilTag Detection** - tag36h11 fiducial detection with multi-tag PnP
- **Multi-Camera Support** - Run up to 3 cameras (front, left, right) simultaneously
- **Coprocessor Pose Estimation** - MULTI_TAG_PNP_ON_COPROCESSOR for best accuracy
- **NetworkTables 4.0** - Low-latency data streaming to roboRIO

### Dashboard and Monitoring
- **Custom Dashboard** at port 5801 with system health indicators
- **Pre-Match Health Check** - One command verifies every component
- **Match Mode** - Competition-optimized CPU, memory, and priority settings
- **Camera Snapshot** - Save diagnostic frames for post-match analysis

### Setup and Deployment
- **One-Script Install** - Java 17, engine, services, permissions in one command
- **Systemd Services** - Auto-start with crash recovery on reboot
- **Architecture Detection** - Automatically selects ARM64 or x86_64 builds

### New in 3.2
- **ArUco Calibration Helper** - Generate ChArUco boards and check calibration status
- **Network Diagnostics** - Test NetworkTables connectivity, latency, and port availability
- **WebSocket Tunneling** - Improved data transport for real-time camera feeds
- **3D Pose Pipeline** - Enhanced multi-tag 3D pose estimation with ambiguity filtering

---

## Quick Start

### 1. Deploy to Coprocessor

```bash
git clone https://github.com/TomAs-1226/AprilVIsion-2.0
cd AprilVIsion-2.0
./setup.sh --team YOUR_TEAM_NUMBER
```

**What this does (in ~2 minutes):**

| Step | Action |
|------|--------|
| 1 | Detects ARM64 or x86_64 architecture |
| 2 | Installs system packages (`curl`, `wget`, `v4l-utils`, `python3`) |
| 3 | Installs Java 17 JDK |
| 4 | Downloads detection engine JAR (~115 MB) with retry logic |
| 5 | Creates system user with camera permissions |
| 6 | Configures NetworkTables for your team's roboRIO |
| 7 | Installs the custom dashboard |
| 8 | Installs + enables two systemd services |
| 9 | Starts everything and verifies it's running |

### 2. Configure Cameras

Open the dashboard:
```
http://<coprocessor-ip>:5801
```

| Setting | Recommended | Why |
|---------|-------------|-----|
| Resolution | 640x480 | Best speed-to-accuracy ratio |
| FPS | 30 | Reliable detection without overloading USB |
| Pipeline | AprilTag | For fiducial detection |
| Tag Family | tag36h11 | FRC standard |
| Decimation | 2 | Half-res processing for faster cycles |

Then run **camera calibration** - this is critical for accurate pose estimation.

### 3. 3D Pose Setup

For accurate 3D pose estimation with multi-tag PnP:

1. **Calibrate each camera** through the dashboard calibration tool
2. **Measure camera mounting positions** precisely (in meters from robot center)
3. **Update Transform3d values** in `VisionSubsystem.java`:
   ```java
   // x = forward, y = left, z = up (meters from robot center)
   private static final Transform3d FRONT_ROBOT_TO_CAM = new Transform3d(
       new Translation3d(0.25, 0.0, 0.50),
       new Rotation3d(0, 0, 0)
   );
   ```
4. **Verify with ArUco calibration check**:
   ```bash
   python3 scripts/aruco_calibration.py --check
   ```

### 4. Add PhotonLib to Robot Code

In your robot project's `build.gradle`:

```gradle
repositories {
    maven { url "https://maven.photonvision.org/repository/internal" }
}

dependencies {
    implementation 'org.photonvision:photonlib-java:v2026.2.2'
}
```

### 5. Drop In the Robot Code

Copy from `robot-code-examples/` into your project:

```java
// VisionSubsystem.java - Manages 3 vision cameras with pose estimation
PhotonCamera frontCamera = new PhotonCamera("front");
PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
    fieldLayout,
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    FRONT_ROBOT_TO_CAM
);

// In periodic():
Optional<EstimatedRobotPose> pose = poseEstimator.update(frontCamera.getLatestResult());
```

### 6. Fuse Vision with Odometry

```java
// Feed vision into SwerveDrivePoseEstimator
Optional<EstimatedRobotPose> pose = vision.getFrontCameraPose();
if (pose.isPresent()) {
    var estimate = pose.get();
    poseEstimator.addVisionMeasurement(
        estimate.estimatedPose.toPose2d(),
        estimate.timestampSeconds,
        vision.getEstimationStdDevs(estimate)
    );
}
```

---

## Custom Tools

### Health Check

Run before every match to verify your vision system is ready:

```bash
./scripts/health_check.sh
```

```
AprilVision 3.2 - System Health Check
======================================
  Detection Engine ......... RUNNING
  Dashboard Proxy .......... RUNNING
  Java 17 ................. OK (17.0.x)
  Engine Binary ............ OK (114M)
  Camera Devices ........... 3 found
  Network (roboRIO) ....... REACHABLE
  Disk Space .............. OK (2.1G free)
  Memory .................. OK (412M / 4096M)
--------------------------------------
  RESULT: ALL SYSTEMS GO
```

### Match Mode

Activate competition-optimized settings before a match:

```bash
./scripts/match_mode.sh enable
```

This:
- Sets CPU governor to `performance` for lowest latency
- Increases Java heap to 768MB for smoother processing
- Disables non-essential system services to free resources
- Enables real-time thread priority for the detection engine
- Logs the change so you can review post-match

Disable after practice:
```bash
./scripts/match_mode.sh disable
```

### Camera Detection

List all connected cameras and their capabilities:
```bash
./scripts/detect_cameras.sh
```

### ArUco Calibration Helper

Generate a ChArUco board for camera calibration or check calibration status:
```bash
# Generate a calibration board image
python3 scripts/aruco_calibration.py --generate

# Check if cameras are calibrated
python3 scripts/aruco_calibration.py --check
```

### Network Diagnostics

Test connectivity to roboRIO and check all vision system ports:
```bash
./scripts/network_diagnostics.sh
```

### Update Detection Engine

Upgrade to a newer engine release:
```bash
./scripts/update_photonvision.sh v2026.3.0
```

---

## Architecture

```
+----------------------------------------------------------+
|              COPROCESSOR (Orange Pi / RPi / x86)          |
|                                                           |
|  +----------------------------------------------------+  |
|  |  Detection Engine (Port 5800, internal)             |  |
|  |  - AprilTag detection (tag36h11)                    |  |
|  |  - Multi-tag PnP on coprocessor                     |  |
|  |  - Camera calibration + MJPEG streams               |  |
|  |  - NetworkTables 4.0 publisher                      |  |
|  +----------------------------------------------------+  |
|                          |                                |
|  +----------------------------------------------------+  |
|  |  AprilVision Dashboard (Port 5801, team-facing)     |  |
|  |  - Custom branded UI overlay                        |  |
|  |  - System health indicators                         |  |
|  |  - Camera feed passthrough                          |  |
|  |  - Error pages with diagnostics                     |  |
|  +----------------------------------------------------+  |
+----------------------------------------------------------+
         |                         |
    NetworkTables              Port 5801
         |                         |
+------------------+       +------------------+
|    roboRIO       |       |  Driver Station  |
|  - PhotonLib     |       |  Browser @ :5801 |
|  - Pose Fusion   |       |  Health + Config |
|  - Auto-Align    |       |                  |
+------------------+       +------------------+
```

---

## Service Management

```bash
# Start / Stop / Restart
sudo systemctl start photonvision aprilvision-dashboard
sudo systemctl stop photonvision aprilvision-dashboard
sudo systemctl restart photonvision

# Status
sudo systemctl status photonvision
sudo systemctl status aprilvision-dashboard

# Logs (live)
journalctl -u photonvision -f
journalctl -u aprilvision-dashboard -f

# Full health check
./scripts/health_check.sh
```

---

## Configuration

### Team Number

Set during setup or edit manually:
```bash
# Re-run setup with new team number
./setup.sh --team 1226

# Or edit directly
nano config/config.yml
```

### Camera Mounting

Define camera positions in `VisionSubsystem.java`:

```java
// x = forward, y = left, z = up (meters from robot center)
private static final Transform3d FRONT_ROBOT_TO_CAM = new Transform3d(
    new Translation3d(0.25, 0.0, 0.50),
    new Rotation3d(0, 0, 0)
);
```

### Setup Options

| Flag | Description |
|------|-------------|
| `--team XXXX` | Set team number |
| `--install-only` | Reinstall services without downloading engine |
| `--dev` | Development mode (no service install) |
| `--clean` | Clean install (re-downloads everything) |

---

## Project Structure

```
AprilVIsion-2.0/
+-- setup.sh                             # One-command full setup
+-- README.md                            # You are here
+-- config/
|   +-- config.yml                       # Team + dashboard config
+-- web/
|   +-- index.html                       # Custom landing page
|   +-- app.js                           # Dashboard status logic
|   +-- style.css                        # Dark theme styles
+-- scripts/
|   +-- aprilvision-bridge.py            # Reverse proxy with branding
|   +-- detect_cameras.sh               # Camera detection utility
|   +-- update_photonvision.sh          # Engine version updater
|   +-- health_check.sh                 # Pre-match system diagnostics
|   +-- match_mode.sh                   # Competition optimization toggle
|   +-- aruco_calibration.py            # ArUco calibration helper
|   +-- network_diagnostics.sh          # Network connectivity tester
+-- deploy/
|   +-- photonvision.service            # Systemd: Detection engine
|   +-- aprilvision-dashboard.service   # Systemd: Dashboard service
+-- robot-code-examples/
|   +-- VisionSubsystem.java           # Multi-camera subsystem
|   +-- AlignToTagCommand.java         # Auto-align to AprilTag
|   +-- RobotContainerExample.java     # Full wiring example
+-- docs/
    +-- JAVA_INTEGRATION_GUIDE.md       # Step-by-step integration guide
```

---

## Robot Code Examples

### VisionSubsystem.java
Multi-camera subsystem managing 3 vision cameras with:
- `PhotonPoseEstimator` per camera using `MULTI_TAG_PNP_ON_COPROCESSOR`
- Distance-based standard deviation scaling (trusts closer tags more)
- Tag visibility queries across all cameras
- Camera connection status telemetry

### AlignToTagCommand.java
PID-based alignment command:
- Targets a specific AprilTag ID
- Uses `getBestCameraToTarget()` Transform3d for 3D positioning
- 3-axis PID control (forward, strafe, rotation)
- Factory methods: `alignClose()` (0.4m) and `alignMedium()` (1.0m)

### RobotContainerExample.java
Full integration showing:
- Vision + odometry fusion with `SwerveDrivePoseEstimator`
- Ambiguity filtering (rejects single-tag results with >0.2 ambiguity)
- Button bindings for auto-align commands
- Vision-assisted teleop (blends driver input with tag tracking)
- Autonomous routine with vision waypoints

---

## Troubleshooting

### Detection engine won't start
```bash
java -version                            # Must show 17.x
journalctl -u photonvision -n 50         # Check error logs
sudo java -jar /opt/photonvision/photonvision.jar  # Run manually to see errors
```

### Cameras not detected
```bash
./scripts/detect_cameras.sh              # List video devices
ls -la /dev/video*                       # Check permissions
./setup.sh --install-only                # Re-apply permissions
```

### Dashboard not loading
```bash
curl http://localhost:5800               # Is the detection engine running?
journalctl -u aprilvision-dashboard -f   # Check dashboard logs
sudo systemctl restart aprilvision-dashboard
```

### NetworkTables not connecting
1. Verify team number: `cat config/config.yml`
2. Ping roboRIO: `ping 10.XX.YY.2`
3. Run diagnostics: `./scripts/network_diagnostics.sh`
4. Check network settings at `:5801` > Settings > Network

### Run the full diagnostic
```bash
./scripts/health_check.sh               # Checks everything at once
```

---

## Credits

- **WPILib** - FRC framework and NetworkTables
- **PhotonLib** - Vision integration library for robot code
- **Team 1226** - AprilVision development and testing

---

## License

MIT License - Free for all FRC teams and educational use.
