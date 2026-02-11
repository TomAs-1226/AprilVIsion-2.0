# AprilVision 2.0 - Custom FRC Vision System

**Custom vision system built on PhotonVision libraries. One-script setup, zero hassle.**

AprilVision 2.0 replaces the original C++ detection pipeline with PhotonVision as the detection engine, wrapped in a custom-branded interface with one-script deployment for FRC teams.

---

## What is AprilVision 2.0?

AprilVision 2.0 is a custom FRC vision system that uses **PhotonVision** as its detection backbone. It provides:

- **PhotonVision detection engine** - AprilTag detection, multi-tag PnP, camera calibration
- **Custom branded web dashboard** - PhotonVision's full UI rebranded as AprilVision via reverse proxy
- **One-script setup** - `./setup.sh --team 1234` handles everything
- **PhotonLib robot code examples** - Ready-to-use Java code for WPILib integration
- **Systemd services** - Auto-start on boot, auto-recovery

---

## Quick Start

### Step 1: Install on Coprocessor

```bash
git clone https://github.com/TomAs-1226/AprilVIsion-2.0
cd AprilVIsion-2.0
./setup.sh --team YOUR_TEAM_NUMBER
```

This automatically:
1. Installs Java 17
2. Downloads PhotonVision v2026.2.2
3. Creates system user with camera permissions
4. Configures NetworkTables for your team's roboRIO
5. Installs and starts systemd services
6. Sets up the custom-branded reverse proxy

### Step 2: Configure Cameras

Open the dashboard in a browser:
```
http://<coprocessor-ip>:5801
```

This opens the PhotonVision interface (rebranded as AprilVision 2.0):
- Add your USB cameras
- Set resolution (640x480 recommended) and FPS (30)
- Select AprilTag pipeline with tag36h11 family
- Run camera calibration for best accuracy

### Step 3: Add PhotonLib to Robot Code

In your robot project's `build.gradle`:

```gradle
dependencies {
    implementation 'org.photonvision:photonlib-java:v2026.2.2'
}
```

### Step 4: Copy Robot Code Examples

Copy from `robot-code-examples/`:

```java
// VisionSubsystem.java - manages 3 PhotonVision cameras
public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera frontCamera = new PhotonCamera("front");
    private final PhotonPoseEstimator frontPoseEstimator;

    public VisionSubsystem() {
        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        frontPoseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            FRONT_ROBOT_TO_CAM
        );
    }

    public Optional<EstimatedRobotPose> getFrontCameraPose() {
        return frontPoseEstimator.update(frontCamera.getLatestResult());
    }
}
```

### Step 5: Fuse Vision with Odometry

```java
// In RobotContainer or DriveSubsystem periodic()
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

## Architecture

```
+--------------------------------------------------+
|              COPROCESSOR (Orange Pi / RPi)         |
|                                                    |
|  +---------------------------------------------+  |
|  |  PhotonVision (Port 5800, internal)          |  |
|  |  - AprilTag detection (tag36h11)             |  |
|  |  - Multi-tag PnP on coprocessor              |  |
|  |  - Camera calibration                        |  |
|  |  - MJPEG camera streams                      |  |
|  |  - NetworkTables publisher                   |  |
|  +---------------------------------------------+  |
|                       |                            |
|  +---------------------------------------------+  |
|  |  AprilVision Reverse Proxy (Port 5801)       |  |
|  |  - Proxies PhotonVision web UI               |  |
|  |  - Replaces PhotonVision branding            |  |
|  |  - Injects custom CSS/JS                     |  |
|  |  - Streams camera feeds through              |  |
|  +---------------------------------------------+  |
+--------------------------------------------------+
         |                    |
    NetworkTables         Port 5801
         |                    |
+------------------+   +------------------+
|    roboRIO       |   |  Driver Laptop   |
|  (Robot Code)    |   |  (Dashboard)     |
|  - PhotonLib     |   |  Browser @ :5801 |
|  - Pose Estimator|   |                  |
+------------------+   +------------------+
```

**Key point:** PhotonVision does all the heavy lifting. AprilVision 2.0 wraps it with custom branding and a streamlined setup experience.

---

## Service Management

```bash
# Start everything
sudo systemctl start photonvision aprilvision-dashboard

# Stop everything
sudo systemctl stop photonvision aprilvision-dashboard

# Check status
sudo systemctl status photonvision
sudo systemctl status aprilvision-dashboard

# View logs
journalctl -u photonvision -f
journalctl -u aprilvision-dashboard -f

# Restart after config changes
sudo systemctl restart photonvision
```

---

## Configuration

### Team Number

Set during `./setup.sh --team XXXX` or edit manually:

- `config/config.yml` - team number and roboRIO IP
- `/opt/photonvision/photonvision_config/network/networkSettings.json` - PhotonVision NT config

### Camera Configuration

All camera settings are managed through the PhotonVision web UI at `:5801`:

| Setting | Recommended | Notes |
|---------|-------------|-------|
| Resolution | 640x480 | Balance of speed and accuracy |
| FPS | 30 | Reliable detection rate |
| Pipeline | AprilTag | For fiducial detection |
| Tag Family | tag36h11 | FRC standard |
| Decimation | 2 | Half-res processing (faster) |

### Camera Mounting

Define camera positions in your robot code `VisionSubsystem.java`:

```java
private static final Transform3d FRONT_ROBOT_TO_CAM = new Transform3d(
    new Translation3d(0.25, 0.0, 0.50),    // x=forward, y=left, z=up (meters)
    new Rotation3d(0, 0, 0)                  // roll, pitch, yaw (radians)
);
```

---

## Project Structure

```
AprilVIsion-2.0/
+-- setup.sh                             # One-command setup
+-- README.md                            # This file
+-- config/
|   +-- config.yml                       # Team and dashboard config
+-- web/
|   +-- index.html                       # Landing page
|   +-- app.js                           # Landing page logic
|   +-- style.css                        # Landing page styles
+-- scripts/
|   +-- aprilvision-bridge.py            # Reverse proxy with branding
|   +-- detect_cameras.sh               # List camera devices
|   +-- update_photonvision.sh          # Update PhotonVision JAR
+-- deploy/
|   +-- photonvision.service            # systemd for PhotonVision
|   +-- aprilvision-dashboard.service   # systemd for reverse proxy
+-- robot-code-examples/
|   +-- VisionSubsystem.java           # PhotonLib multi-camera subsystem
|   +-- AlignToTagCommand.java         # Auto-align to AprilTag
|   +-- RobotContainerExample.java     # Full integration example
+-- docs/
    +-- JAVA_INTEGRATION_GUIDE.md       # Detailed PhotonLib guide
```

---

## Robot Code Examples

### VisionSubsystem.java
Multi-camera PhotonVision subsystem with:
- 3 PhotonCamera instances (front, left, right)
- PhotonPoseEstimator per camera with MULTI_TAG_PNP_ON_COPROCESSOR
- Distance-based standard deviation scaling
- Tag visibility queries

### AlignToTagCommand.java
PID-based alignment command that:
- Searches for a specific AprilTag ID
- Uses camera-to-target Transform3d for 3D positioning
- PID control on forward, strafe, and rotation axes
- Factory methods for common alignment distances

### RobotContainerExample.java
Full integration showing:
- Vision + odometry fusion with SwerveDrivePoseEstimator
- Ambiguity filtering for single-tag results
- Button bindings for auto-align commands
- Vision-assisted teleop driving

---

## PhotonLib Dependency

Add to your robot project's `build.gradle`:

```gradle
repositories {
    maven { url "https://maven.photonvision.org/repository/internal" }
}

dependencies {
    implementation 'org.photonvision:photonlib-java:v2026.2.2'
}
```

Required PhotonLib imports:
```java
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
```

---

## Updating PhotonVision

```bash
# Download new version
sudo systemctl stop photonvision
sudo wget -O /opt/photonvision/photonvision.jar \
    https://github.com/PhotonVision/photonvision/releases/download/vX.X.X/photonvision-vX.X.X-linuxarm64.jar
sudo systemctl start photonvision
```

Or use the provided script:
```bash
./scripts/update_photonvision.sh vX.X.X
```

---

## Troubleshooting

### PhotonVision not starting
```bash
# Check Java
java -version  # Should show 17.x

# Check logs
journalctl -u photonvision -n 50

# Try running manually
sudo java -jar /opt/photonvision/photonvision.jar
```

### Cameras not detected
```bash
# List video devices
v4l2-ctl --list-devices

# Check permissions
ls -la /dev/video*

# Re-run setup
./setup.sh --install-only
```

### Dashboard proxy not working
```bash
# Check if PhotonVision is running first
curl http://localhost:5800

# Check proxy logs
journalctl -u aprilvision-dashboard -f

# Restart proxy
sudo systemctl restart aprilvision-dashboard
```

### NetworkTables not connecting
1. Verify team number in config
2. Check roboRIO is on the network: `ping 10.XX.YY.2`
3. Verify in PhotonVision UI (Settings > Network)

---

## Credits

- **PhotonVision** - Core detection engine and web interface
- **WPILib** - FRC robotics framework and NetworkTables
- **PhotonLib** - Java library for PhotonVision integration

---

## License

MIT License - Free for FRC teams and educational use.
