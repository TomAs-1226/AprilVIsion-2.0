# FRC Vision Coprocessor API Documentation

Complete API reference for the FRC AprilTag Vision Coprocessor.

## Table of Contents

1. [NetworkTables API](#networktables-api)
2. [HTTP REST API](#http-rest-api)
3. [WebSocket/SSE API](#websocketsse-api)
4. [Auto-Align Integration](#auto-align-integration)
5. [Robot-Side Integration](#robot-side-integration)

---

## NetworkTables API

All data is published under the configurable root table (default: `/FRCVision`).

### Fused Pose Data

Primary pose data for robot localization and pose estimation.

| Topic | Type | Description |
|-------|------|-------------|
| `/FRCVision/fused/pose` | double[] | `[x, y, theta]` - Robot pose in field frame (meters, radians) |
| `/FRCVision/fused/std_devs` | double[] | `[x, y, theta]` - Standard deviations for WPILib pose estimator |
| `/FRCVision/fused/timestamp` | double | Capture timestamp (seconds since epoch) |
| `/FRCVision/fused/valid` | boolean | `true` if pose is trustworthy |
| `/FRCVision/fused/confidence` | double | 0.0-1.0 confidence score |
| `/FRCVision/fused/tag_count` | int | Number of tags used for pose |
| `/FRCVision/fused/cameras_contributing` | int | Number of cameras with valid pose |

### Raw Fused Pose (Unfiltered)

| Topic | Type | Description |
|-------|------|-------------|
| `/FRCVision/fused_raw/pose` | double[] | `[x, y, theta]` - Unfiltered pose |
| `/FRCVision/fused_raw/std_devs` | double[] | Standard deviations |
| `/FRCVision/fused_raw/timestamp` | double | Timestamp |
| `/FRCVision/fused_raw/valid` | boolean | Validity flag |

### Per-Camera Data

For each camera `N` (0, 1, 2, ...):

| Topic | Type | Description |
|-------|------|-------------|
| `/FRCVision/camN/timestamp_capture` | double | Frame capture time |
| `/FRCVision/camN/timestamp_publish` | double | Publish time |
| `/FRCVision/camN/latency_ms` | double | Total pipeline latency |
| `/FRCVision/camN/tag_ids` | int[] | Detected tag IDs |
| `/FRCVision/camN/tag_count` | int | Number of detected tags |
| `/FRCVision/camN/corners_px` | double[] | Tag corners `[id,x1,y1,x2,y2,x3,y3,x4,y4,...]` |
| `/FRCVision/camN/margins` | double[] | Detection margins (quality) |
| `/FRCVision/camN/pose_robot` | double[] | `[x, y, theta]` from this camera |
| `/FRCVision/camN/pose_cam` | double[] | `[x,y,z,qw,qx,qy,qz]` camera pose |
| `/FRCVision/camN/std_devs` | double[] | Standard deviations |
| `/FRCVision/camN/confidence` | double | Confidence score |
| `/FRCVision/camN/pose_valid` | boolean | Pose validity |

### System Status

| Topic | Type | Description |
|-------|------|-------------|
| `/FRCVision/status/uptime` | double | System uptime (seconds) |
| `/FRCVision/status/cpu_temp` | double | CPU temperature (°C) |
| `/FRCVision/status/cpu_usage` | double | CPU usage (%) |
| `/FRCVision/status/camN_fps` | double | Camera N frame rate |
| `/FRCVision/status/camN_dropped` | int | Camera N dropped frames |
| `/FRCVision/status/camN_connected` | boolean | Camera N connection status |

### Auto-Align Data

For automated alignment to AprilTags:

**Robot → Vision (Subscribed by coprocessor):**

| Topic | Type | Description |
|-------|------|-------------|
| `/FRCVision/auto_align/target_tag_id` | int | Target tag ID (-1 = none) |
| `/FRCVision/auto_align/target_offset` | double[] | `[distance_m, angle_rad]` offset from tag |

**Vision → Robot (Published by coprocessor):**

| Topic | Type | Description |
|-------|------|-------------|
| `/FRCVision/auto_align/target_visible` | boolean | Target tag currently visible |
| `/FRCVision/auto_align/target_pose` | double[] | `[x, y, theta]` where robot should go |
| `/FRCVision/auto_align/robot_pose` | double[] | `[x, y, theta]` current vision pose |
| `/FRCVision/auto_align/error` | double[] | `[x, y, theta]` alignment error |
| `/FRCVision/auto_align/distance_m` | double | Distance to target (meters) |
| `/FRCVision/auto_align/angle_error_rad` | double | Rotation error (radians) |
| `/FRCVision/auto_align/ready` | boolean | Within alignment tolerance |
| `/FRCVision/auto_align/has_target` | boolean | Target is set |
| `/FRCVision/auto_align/current_target_id` | int | Echo of current target |

---

## HTTP REST API

Base URL: `http://<coprocessor-ip>:5800`

### GET /api/status

System status and readiness check.

**Response:**
```json
{
  "ready": true,
  "state": "ready",
  "uptime_seconds": 125.5,
  "cameras_connected": 3,
  "cameras_expected": 3,
  "nt_connected": true,
  "web_ready": true
}
```

**States:**
- `warming_up` - System starting, not all cameras ready
- `ready` - At least 1 camera + web server running
- `degraded` - Running but with issues

### GET /api/config

Get current configuration.

**Response:**
```json
{
  "cameras": [...],
  "apriltag": {...},
  "tracking": {...},
  "field": {...},
  "outputs": {...}
}
```

### POST /api/config/reload

Trigger configuration hot-reload.

**Response:**
```json
{
  "success": true,
  "message": "Configuration reload triggered"
}
```

### GET /api/detections

Get latest detection data from all cameras.

**Response:**
```json
{
  "timestamp": 1234567890.123,
  "cameras": [
    {
      "id": 0,
      "name": "front",
      "tags": [
        {
          "id": 1,
          "corners": [[x1,y1], [x2,y2], [x3,y3], [x4,y4]],
          "margin": 85.5,
          "pose_valid": true
        }
      ],
      "robot_pose": {"x": 5.2, "y": 3.1, "theta": 1.57},
      "latency_ms": 12.5
    }
  ],
  "fused": {
    "valid": true,
    "pose": {"x": 5.2, "y": 3.1, "theta": 1.57},
    "std_devs": {"x": 0.05, "y": 0.05, "theta": 0.02},
    "confidence": 0.95,
    "tag_count": 3
  }
}
```

### GET /api/align

Get auto-alignment status.

**Response:**
```json
{
  "target_tag_id": 5,
  "target_visible": true,
  "target_pose": {"x": 5.0, "y": 3.0, "theta": 0.0},
  "robot_pose": {"x": 5.2, "y": 3.1, "theta": 0.05},
  "error": {"x": -0.2, "y": -0.1, "theta": -0.05},
  "distance_m": 0.224,
  "ready": false,
  "has_target": true
}
```

### POST /api/align/target

Set alignment target (alternative to NetworkTables).

**Request:**
```json
{
  "tag_id": 5,
  "distance_m": 0.5,
  "angle_rad": 0.0
}
```

**Response:**
```json
{
  "success": true,
  "target_tag_id": 5
}
```

### GET /stream/cam{N}

MJPEG video stream for camera N.

**Example:** `http://192.168.1.100:5800/stream/cam0`

Returns `multipart/x-mixed-replace` MJPEG stream with annotated video.

### GET /api/events

Server-Sent Events (SSE) stream for real-time updates.

**Events:**
- `detection` - New detection data
- `pose` - Updated fused pose
- `status` - System status change
- `align` - Alignment status update

**Example:**
```javascript
const events = new EventSource('http://192.168.1.100:5800/api/events');
events.addEventListener('pose', (e) => {
  const pose = JSON.parse(e.data);
  console.log('Robot at:', pose.x, pose.y);
});
```

---

## WebSocket/SSE API

### SSE Events

Connect to `/api/events` for Server-Sent Events:

```javascript
const sse = new EventSource('http://<ip>:5800/api/events');

sse.addEventListener('detection', (event) => {
  const data = JSON.parse(event.data);
  // { camera_id, tags: [...], latency_ms }
});

sse.addEventListener('pose', (event) => {
  const data = JSON.parse(event.data);
  // { x, y, theta, valid, confidence }
});

sse.addEventListener('align', (event) => {
  const data = JSON.parse(event.data);
  // { target_visible, error, distance_m, ready }
});
```

---

## Auto-Align Integration

### How Auto-Align Works

1. Robot sets `target_tag_id` via NetworkTables
2. Vision coprocessor calculates target pose based on tag position + offset
3. Vision publishes current pose, target pose, and error
4. Robot uses error values to drive to target
5. `ready` becomes `true` when error is within tolerance

### Alignment Geometry

```
        Tag (facing robot)
           ▲
           │
           │  distance_m
           │
    ───────┼───────  ← target_pose (robot should be here)
           │
           │  approach from this side
           │
         Robot
```

The `target_offset` parameter:
- `distance_m`: How far from the tag the robot should stop
- `angle_rad`: Rotation offset (0 = perpendicular to tag)

### Tolerance Values

Default tolerances for `ready` signal:
- Position: ±0.05 meters (5 cm)
- Rotation: ±0.05 radians (~3°)

---

## Robot-Side Integration

### WPILib Java Example

```java
import edu.wpi.first.networktables.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable visionTable;
    private final DoubleArraySubscriber poseSub;
    private final DoubleArraySubscriber stdDevsSub;
    private final BooleanSubscriber validSub;
    private final DoubleSubscriber timestampSub;

    // Auto-align
    private final IntegerPublisher targetTagPub;
    private final DoubleArrayPublisher targetOffsetPub;
    private final BooleanSubscriber alignReadySub;
    private final DoubleArraySubscriber alignErrorSub;

    private final SwerveDrivePoseEstimator poseEstimator;

    public VisionSubsystem(SwerveDrivePoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;

        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        visionTable = nt.getTable("FRCVision");

        // Subscribe to fused pose
        var fusedTable = visionTable.getSubTable("fused");
        poseSub = fusedTable.getDoubleArrayTopic("pose").subscribe(new double[]{});
        stdDevsSub = fusedTable.getDoubleArrayTopic("std_devs").subscribe(new double[]{});
        validSub = fusedTable.getBooleanTopic("valid").subscribe(false);
        timestampSub = fusedTable.getDoubleTopic("timestamp").subscribe(0.0);

        // Auto-align topics
        var alignTable = visionTable.getSubTable("auto_align");
        targetTagPub = alignTable.getIntegerTopic("target_tag_id").publish();
        targetOffsetPub = alignTable.getDoubleArrayTopic("target_offset").publish();
        alignReadySub = alignTable.getBooleanTopic("ready").subscribe(false);
        alignErrorSub = alignTable.getDoubleArrayTopic("error").subscribe(new double[]{});
    }

    @Override
    public void periodic() {
        updatePoseEstimator();
    }

    private void updatePoseEstimator() {
        if (!validSub.get()) return;

        double[] pose = poseSub.get();
        double[] stdDevs = stdDevsSub.get();
        double timestamp = timestampSub.get();

        if (pose.length < 3 || stdDevs.length < 3) return;

        Pose2d visionPose = new Pose2d(
            pose[0], pose[1], new Rotation2d(pose[2])
        );

        // Add vision measurement to pose estimator
        poseEstimator.addVisionMeasurement(
            visionPose,
            timestamp,
            VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[2])
        );
    }

    // ========== Auto-Align Methods ==========

    /**
     * Start alignment to a specific AprilTag
     * @param tagId AprilTag ID to align to
     * @param distanceMeters Distance from tag to stop
     * @param angleRadians Approach angle offset
     */
    public void startAlignment(int tagId, double distanceMeters, double angleRadians) {
        targetTagPub.set(tagId);
        targetOffsetPub.set(new double[]{distanceMeters, angleRadians});
    }

    /**
     * Stop alignment
     */
    public void stopAlignment() {
        targetTagPub.set(-1);
    }

    /**
     * Check if robot is aligned to target
     */
    public boolean isAligned() {
        return alignReadySub.get();
    }

    /**
     * Get alignment error for closed-loop control
     * @return [x_error, y_error, theta_error] or null
     */
    public double[] getAlignmentError() {
        double[] error = alignErrorSub.get();
        return error.length >= 3 ? error : null;
    }

    /**
     * Get current vision pose
     */
    public Pose2d getVisionPose() {
        double[] pose = poseSub.get();
        if (pose.length < 3) return null;
        return new Pose2d(pose[0], pose[1], new Rotation2d(pose[2]));
    }

    /**
     * Check if vision has valid pose
     */
    public boolean hasValidPose() {
        return validSub.get();
    }
}
```

### Auto-Align Command Example

```java
public class AlignToTagCommand extends Command {
    private final VisionSubsystem vision;
    private final DriveSubsystem drive;
    private final int tagId;
    private final double distanceM;

    private final PIDController xController = new PIDController(2.0, 0, 0.1);
    private final PIDController yController = new PIDController(2.0, 0, 0.1);
    private final PIDController thetaController = new PIDController(3.0, 0, 0.1);

    public AlignToTagCommand(VisionSubsystem vision, DriveSubsystem drive,
                            int tagId, double distanceM) {
        this.vision = vision;
        this.drive = drive;
        this.tagId = tagId;
        this.distanceM = distanceM;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        vision.startAlignment(tagId, distanceM, 0.0);

        xController.setTolerance(0.02);
        yController.setTolerance(0.02);
        thetaController.setTolerance(0.02);
    }

    @Override
    public void execute() {
        double[] error = vision.getAlignmentError();
        if (error == null) {
            drive.stop();
            return;
        }

        // PID control on error (error = target - current)
        double xSpeed = xController.calculate(error[0], 0);
        double ySpeed = yController.calculate(error[1], 0);
        double rotSpeed = thetaController.calculate(error[2], 0);

        // Clamp speeds
        xSpeed = MathUtil.clamp(xSpeed, -2.0, 2.0);
        ySpeed = MathUtil.clamp(ySpeed, -2.0, 2.0);
        rotSpeed = MathUtil.clamp(rotSpeed, -2.0, 2.0);

        drive.driveFieldRelative(xSpeed, ySpeed, rotSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        vision.stopAlignment();
    }

    @Override
    public boolean isFinished() {
        return vision.isAligned();
    }
}
```

### Usage in Robot Code

```java
// In RobotContainer.java

// Align to tag 5 at 0.5 meters
Command alignToSpeaker = new AlignToTagCommand(
    visionSubsystem, driveSubsystem, 5, 0.5
);

// Bind to button
driverController.a().whileTrue(alignToSpeaker);

// Or use inline
driverController.b().whileTrue(
    Commands.run(() -> {
        double[] error = visionSubsystem.getAlignmentError();
        if (error != null) {
            driveSubsystem.driveWithError(error[0], error[1], error[2]);
        }
    }, driveSubsystem)
    .beforeStarting(() -> visionSubsystem.startAlignment(7, 0.6, 0))
    .finallyDo(() -> visionSubsystem.stopAlignment())
);
```

---

## Standard Deviations for Pose Estimator

The vision system calculates standard deviations based on:
- Number of tags visible
- Average detection margin
- Average reprojection error
- Distance to tags

**Typical values:**
| Tags | Distance | Std Dev X/Y | Std Dev θ |
|------|----------|-------------|-----------|
| 1 | < 2m | 0.10 m | 0.10 rad |
| 1 | 2-4m | 0.20 m | 0.15 rad |
| 2+ | < 2m | 0.05 m | 0.05 rad |
| 2+ | 2-4m | 0.10 m | 0.08 rad |
| 3+ | any | 0.03 m | 0.03 rad |

Use these values with `SwerveDrivePoseEstimator.addVisionMeasurement()`.

---

## Latency Compensation

Vision timestamps are in seconds since epoch. For proper latency compensation:

```java
// Get vision timestamp
double visionTimestamp = timestampSub.get();

// Convert to FPGA time if needed (assuming clocks are synced)
// Most teams use system time directly with addVisionMeasurement

poseEstimator.addVisionMeasurement(
    visionPose,
    visionTimestamp,  // Use vision's timestamp
    stdDevs
);
```

**Note:** For best results, ensure Orange Pi and roboRIO clocks are synchronized via NTP or use the capture timestamp directly.
