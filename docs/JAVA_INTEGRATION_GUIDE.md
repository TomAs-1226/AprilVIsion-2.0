# AprilVision 3.2 - Java Integration Guide

> Step-by-step guide to integrating AprilVision 3.2 vision data into your FRC robot code using PhotonLib.

---

## Overview

AprilVision 3.2 uses an integrated detection engine. Your robot code uses **PhotonLib** (vision integration library) to receive AprilTag detection data over NetworkTables.

This guide walks through everything from initial setup to multi-camera pose estimation.

---

## 1. Prerequisites

### Coprocessor Setup
Run the one-script setup on your coprocessor:
```bash
./setup.sh --team YOUR_TEAM_NUMBER
```

Verify everything is running:
```bash
./scripts/health_check.sh
```

### Robot Project Dependencies
Add PhotonLib to your `build.gradle`:

```gradle
repositories {
    maven { url "https://maven.photonvision.org/repository/internal" }
}

dependencies {
    implementation 'org.photonvision:photonlib-java:v2026.2.2'
}
```

### Camera Configuration
Open `http://<coprocessor-ip>:5801` and configure:
- Camera names: `front`, `left`, `right` (must match robot code)
- Pipeline: AprilTag with tag36h11
- Resolution: 640x480 @ 30 FPS
- Run camera calibration for accurate poses

---

## 2. Basic Vision Subsystem

Start with a single-camera subsystem:

```java
package frc.robot.subsystems;

import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera frontCamera;
    private final PhotonPoseEstimator poseEstimator;

    // Camera mounting relative to robot center (meters, radians)
    private static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        new Translation3d(0.25, 0.0, 0.50), // 25cm forward, 50cm up
        new Rotation3d(0, 0, 0)               // facing forward
    );

    public VisionSubsystem() {
        frontCamera = new PhotonCamera("front");

        AprilTagFieldLayout fieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        poseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            ROBOT_TO_CAMERA
        );
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public Optional<EstimatedRobotPose> getEstimatedPose() {
        return poseEstimator.update(frontCamera.getLatestResult());
    }

    public boolean hasTargets() {
        return frontCamera.getLatestResult().hasTargets();
    }

    public boolean isConnected() {
        return frontCamera.isConnected();
    }
}
```

---

## 3. Pose Estimator Integration

Feed vision poses into WPILib's `SwerveDrivePoseEstimator`:

```java
// In your DriveSubsystem or RobotContainer periodic()
public void updateVisionPose() {
    Optional<EstimatedRobotPose> result = vision.getEstimatedPose();
    if (result.isEmpty()) return;

    EstimatedRobotPose estimate = result.get();

    // Filter out high-ambiguity single-tag results
    if (estimate.targetsUsed.size() == 1) {
        if (estimate.targetsUsed.get(0).getPoseAmbiguity() > 0.2) return;
    }

    // Calculate standard deviations
    var stdDevs = vision.getEstimationStdDevs(estimate);

    // Add to pose estimator with proper timestamp
    poseEstimator.addVisionMeasurement(
        estimate.estimatedPose.toPose2d(),
        estimate.timestampSeconds,
        stdDevs
    );
}
```

---

## 4. Standard Deviation Calculation

Scale trust based on distance to tags and tag count. Closer tags and multi-tag results get more trust:

```java
public Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose estimate) {
    int numTags = estimate.targetsUsed.size();
    double avgDist = 0;

    for (var target : estimate.targetsUsed) {
        var tagPose = poseEstimator.getFieldTags()
            .getTagPose(target.getFiducialId());
        if (tagPose.isPresent()) {
            avgDist += tagPose.get().toPose2d().getTranslation()
                .getDistance(estimate.estimatedPose.toPose2d().getTranslation());
        }
    }
    if (numTags == 0) return VecBuilder.fill(999, 999, 999);
    avgDist /= numTags;

    if (numTags > 1) {
        // Multi-tag: more trustworthy
        return VecBuilder.fill(
            0.2 * (1 + avgDist * avgDist / 30),
            0.2 * (1 + avgDist * avgDist / 30),
            0.5 * (1 + avgDist * avgDist / 20));
    } else {
        // Single tag: less trustworthy
        return VecBuilder.fill(
            0.5 * (1 + avgDist * avgDist / 15),
            0.5 * (1 + avgDist * avgDist / 15),
            0.9 * (1 + avgDist * avgDist / 10));
    }
}
```

---

## 5. Auto-Align Command

Align the robot to a specific AprilTag using PID control:

```java
public class AlignToTagCommand extends Command {
    private final VisionSubsystem vision;
    private final DriveSubsystem drive;
    private final int tagId;
    private final PIDController forwardPID = new PIDController(1.5, 0, 0.08);
    private final PIDController strafePID = new PIDController(1.5, 0, 0.08);
    private final PIDController rotPID = new PIDController(2.0, 0, 0.1);

    public AlignToTagCommand(VisionSubsystem vision, DriveSubsystem drive, int tagId) {
        this.vision = vision;
        this.drive = drive;
        this.tagId = tagId;
        rotPID.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive);
    }

    @Override
    public void execute() {
        var result = vision.getLatestFrontResult();
        if (!result.hasTargets()) { drive.stop(); return; }

        PhotonTrackedTarget target = null;
        for (var t : result.getTargets()) {
            if (t.getFiducialId() == tagId) { target = t; break; }
        }
        if (target == null) { drive.stop(); return; }

        Transform3d cam2target = target.getBestCameraToTarget();
        double fwd = cam2target.getX() - 0.5; // Stop 0.5m from tag
        double strafe = cam2target.getY();
        double rot = Math.atan2(strafe, cam2target.getX());

        drive.driveRobotRelative(
            MathUtil.clamp(forwardPID.calculate(fwd, 0), -2, 2),
            MathUtil.clamp(-strafePID.calculate(strafe, 0), -2, 2),
            MathUtil.clamp(-rotPID.calculate(rot, 0), -2, 2)
        );
    }

    @Override
    public void end(boolean interrupted) { drive.stop(); }

    @Override
    public boolean isFinished() {
        return forwardPID.atSetpoint() && strafePID.atSetpoint() && rotPID.atSetpoint();
    }
}
```

---

## 6. Multi-Camera Setup

For 3-camera configurations, each camera gets its own pose estimator:

```java
private final PhotonCamera frontCamera = new PhotonCamera("front");
private final PhotonCamera leftCamera = new PhotonCamera("left");
private final PhotonCamera rightCamera = new PhotonCamera("right");

// Each camera gets its own pose estimator with its mounting transform
private final PhotonPoseEstimator frontEstimator = new PhotonPoseEstimator(
    fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, FRONT_TRANSFORM);
private final PhotonPoseEstimator leftEstimator = new PhotonPoseEstimator(
    fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, LEFT_TRANSFORM);
private final PhotonPoseEstimator rightEstimator = new PhotonPoseEstimator(
    fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, RIGHT_TRANSFORM);

// In periodic(), update each and feed to the shared SwerveDrivePoseEstimator
```

See `robot-code-examples/VisionSubsystem.java` for the complete multi-camera implementation.

---

## 7. Competition Day Checklist

Before your match:
1. Run `./scripts/health_check.sh` on the coprocessor
2. Enable match mode: `./scripts/match_mode.sh enable`
3. Open `:5801` and verify cameras are detecting tags
4. Deploy robot code and check SmartDashboard for `Vision/FrontConnected: true`

---

## 8. Troubleshooting

### Camera not connecting
- Verify camera name in code matches the AprilVision dashboard
- Check `SmartDashboard.putBoolean("Vision/Connected", camera.isConnected())`

### No pose estimates
- Ensure AprilTag pipeline is selected in the dashboard
- Verify field layout matches current season
- Check for pose ambiguity filtering (>0.2 is rejected)

### High latency
- Use 640x480 resolution instead of higher
- Reduce FPS to 30 if at 60
- Check USB bandwidth (use USB 3.0 ports)
- Enable match mode for lowest latency: `./scripts/match_mode.sh enable`

### Inaccurate poses
- Run camera calibration in the dashboard
- Verify camera mounting Transform3d values match physical measurements
- Ensure multi-tag strategy is selected
