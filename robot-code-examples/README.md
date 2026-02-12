# AprilVision 3.2 - Robot Code Examples

Java/WPILib examples for integrating AprilVision 3.2 vision data into your robot code.

These examples use **PhotonLib** (vision integration library) to receive
AprilTag detection data over NetworkTables from the coprocessor.

## Prerequisites

1. Run `./setup.sh --team YOUR_TEAM_NUMBER` on the coprocessor
2. Verify with `./scripts/health_check.sh`
3. Add PhotonLib to your robot project's `build.gradle`:

```gradle
repositories {
    maven { url "https://maven.photonvision.org/repository/internal" }
}

dependencies {
    implementation 'org.photonvision:photonlib-java:v2026.2.2'
}
```

## Files

### VisionSubsystem.java
Multi-camera subsystem managing 3 vision cameras (front, left, right).
- `PhotonPoseEstimator` per camera with `MULTI_TAG_PNP_ON_COPROCESSOR` strategy
- Distance-based standard deviation scaling (trusts closer tags more)
- Tag visibility queries across all cameras
- Camera connection status telemetry to SmartDashboard

### AlignToTagCommand.java
PID-based command that aligns the robot to a specific AprilTag ID.
- Uses `getBestCameraToTarget()` Transform3d for 3D positioning
- 3-axis PID control: forward, strafe, and rotation
- Factory methods: `alignClose()` (0.4m) and `alignMedium()` (1.0m)
- Configurable timeout protection

### RobotContainerExample.java
Complete integration example showing:
- Vision + odometry fusion with `SwerveDrivePoseEstimator`
- Ambiguity filtering (rejects single-tag results with >0.2 ambiguity)
- Auto-align button bindings for driver and operator
- Vision-assisted teleop (blends driver input with tag tracking)
- Autonomous routine with vision waypoints

## Camera Names

Camera names in Java code **must match** the names configured in the
AprilVision dashboard (accessed at `http://<coprocessor-ip>:5801`):

| Name | Position | Default Yaw |
|------|----------|-------------|
| `"front"` | Forward-facing | 0 deg |
| `"left"` | Left-side | 90 deg |
| `"right"` | Right-side | -90 deg |

## Camera Mounting

Edit the `Transform3d` constants in `VisionSubsystem.java` to match your
robot's camera mounting positions:

```java
// x = forward, y = left, z = up (meters from robot center)
private static final Transform3d FRONT_ROBOT_TO_CAM = new Transform3d(
    new Translation3d(0.25, 0.0, 0.50),
    new Rotation3d(0, 0, 0)
);
```

Measure carefully - inaccurate transforms cause inaccurate pose estimates.

## Full Integration Guide

See `docs/JAVA_INTEGRATION_GUIDE.md` for detailed step-by-step instructions.
