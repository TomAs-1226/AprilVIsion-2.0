# Robot Code Examples

Java/WPILib examples for integrating AprilVision 2.0 (PhotonVision-based) with your robot code.

## Prerequisites

Add PhotonLib to your `build.gradle`:

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
Multi-camera PhotonVision subsystem that manages 3 cameras (front, left, right).
Uses `PhotonPoseEstimator` with `MULTI_TAG_PNP_ON_COPROCESSOR` strategy for
maximum accuracy. Includes distance-based standard deviation scaling.

### AlignToTagCommand.java
PID-based command that aligns the robot to a specific AprilTag ID.
Uses PhotonVision's `getBestCameraToTarget()` Transform3d for 3D positioning.

### RobotContainerExample.java
Complete integration example showing:
- Vision pose fusion with `SwerveDrivePoseEstimator`
- Auto-align button bindings
- Vision-assisted teleop driving
- Autonomous routine with vision

## Camera Names

Camera names in the Java code **must match** the names configured in the
PhotonVision web UI (accessed at `http://<coprocessor-ip>:5801`):

- `"front"` - Forward-facing camera
- `"left"` - Left-side camera
- `"right"` - Right-side camera

## Camera Mounting

Edit the `Transform3d` constants in `VisionSubsystem.java` to match your
robot's camera mounting positions (meters and radians from robot center).
