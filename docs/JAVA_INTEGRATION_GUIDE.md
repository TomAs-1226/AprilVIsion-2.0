# AprilVision 3.1 - Complete Java Integration Guide

## Overview

**AprilVision 3.1** is the C++ vision system. Your **robot code is 100% Java**.

This guide shows you how to integrate AprilVision 3.1 with WPILib Java robot code via NetworkTables.

---

## Architecture

```
┌─────────────────────────────────────────────────────┐
│  Orange Pi / Coprocessor (C++)                      │
│  ┌───────────────────────────────────────────────┐  │
│  │  AprilVision 3.1 (C++)                        │  │
│  │  • Detects AprilTags                          │  │
│  │  • Calculates robot pose                      │  │
│  │  • Auto-align trajectory planning             │  │
│  │  • Publishes to NetworkTables                 │  │
│  └─────────────────┬─────────────────────────────┘  │
└────────────────────┼────────────────────────────────┘
                     │ NetworkTables 4.0
                     │ (TCP/UDP)
┌────────────────────┼────────────────────────────────┐
│  roboRIO (Java)    ▼                                 │
│  ┌───────────────────────────────────────────────┐  │
│  │  Robot.java (Your Code)                       │  │
│  │  • Subscribes to vision data via NT           │  │
│  │  • Fuses with wheel odometry                  │  │
│  │  • Controls drive/shooter                     │  │
│  └───────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────┘
```

**Communication:** All done via **NetworkTables 4.0**
- Vision → Robot: Pose, confidence, alignment data
- Robot → Vision: Odometry, target requests, commands

---

## NetworkTables 3.1 Schema

### Vision → Robot (Published by AprilVision)

#### 1. Fused Robot Pose
```
/FRCVision/fused/
  pose              → double[3]    [x, y, theta] in meters/radians
  timestamp         → double       FPGA timestamp (seconds)
  std_devs          → double[3]    [σx, σy, σθ] standard deviations
  tag_count         → int          Number of tags used
  cameras_used      → int          Number of cameras contributing
  confidence        → double       0-1 confidence score
  valid             → boolean      True if pose is trustworthy
  latency_ms        → double       Processing latency
```

#### 2. Per-Camera Data (for debugging)
```
/FRCVision/cam{0,1,2}/
  timestamp         → double       Capture timestamp
  tag_ids           → int[]        Detected tag IDs
  tag_count         → int          Number of tags
  pose_robot        → double[3]    [x, y, theta] from this camera
  pose_valid        → boolean      Valid pose from this camera
  avg_reproj_error  → double       Average reprojection error (pixels)
  confidence        → double       0-1 confidence
```

#### 3. Auto-Align Guidance (Phase 3)
```
/FRCVision/auto_align/
  target_visible    → boolean      Is target tag visible?
  robot_pose        → double[3]    Current pose from vision
  target_pose       → double[3]    Where robot should be
  error             → double[3]    [Δx, Δy, Δθ] error
  distance_m        → double       Distance to target (meters)
  heading_error_deg → double       Heading error (degrees)
  ready             → boolean      True if aligned within tolerance
  current_stage     → string       "approach", "fine_align", "ready"
  has_target        → boolean      True if target is set
```

#### 4. Semicircle Shooter (Phase 3 Extension)
```
/FRCVision/semicircle/
  active            → boolean      Semicircle mode active?
  velocities        → double[3]    [vx, vy, omega] m/s, rad/s
  path_progress     → double       0-1 progress on path
  ready_to_shoot    → boolean      Aligned and ready to fire
  primary_tag       → int          Current primary tracking tag
  visible_tags      → int[]        All visible tag IDs
```

#### 5. System Status
```
/FRCVision/status/
  uptime            → double       System uptime (seconds)
  cpu_temp          → double       CPU temperature (°C)
  cpu_usage         → double       CPU usage (0-100%)

  nt_connected      → boolean      Connected to roboRIO?
  nt_connection_count → int        Number of NT connections
  nt_server_ip      → string       roboRIO IP address

  cam0_fps          → double       Camera 0 FPS
  cam0_connected    → boolean      Camera 0 connected?
  (repeat for cam1, cam2...)
```

### Robot → Vision (Subscribed by AprilVision)

#### 1. Auto-Align Target Request
```
/FRCVision/auto_align/
  target_tag_id     → int          Which tag to align to
  target_offset     → double[2]    [distance_m, angle_rad] offset
```

#### 2. Odometry (for pose fusion)
```
/RobotOdometry/
  pose              → double[3]    [x, y, theta] from encoders
  angular_velocity  → double       rad/s
  timestamp         → double       FPGA timestamp
```

#### 3. Game State
```
/FMSInfo/
  IsRedAlliance     → boolean      True if red alliance
  MatchTime         → double       Seconds remaining
```

---

## Java Integration Examples

### 1. Basic Pose Subscription

```java
import edu.wpi.first.networktables.*;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable visionTable;

    // Subscribers
    private final DoubleArraySubscriber poseSub;
    private final DoubleArraySubscriber stdDevsSub;
    private final BooleanSubscriber validSub;
    private final DoubleSubscriber confidenceSub;
    private final IntegerSubscriber tagCountSub;

    public VisionSubsystem() {
        var ntInst = NetworkTableInstance.getDefault();
        visionTable = ntInst.getTable("FRCVision").getSubTable("fused");

        // Subscribe to pose data
        poseSub = visionTable.getDoubleArrayTopic("pose")
            .subscribe(new double[]{0, 0, 0});

        stdDevsSub = visionTable.getDoubleArrayTopic("std_devs")
            .subscribe(new double[]{999, 999, 999});

        validSub = visionTable.getBooleanTopic("valid")
            .subscribe(false);

        confidenceSub = visionTable.getDoubleTopic("confidence")
            .subscribe(0.0);

        tagCountSub = visionTable.getIntegerTopic("tag_count")
            .subscribe(0);
    }

    /**
     * Get current vision pose estimate
     */
    public Optional<Pose2d> getVisionPose() {
        if (!validSub.get()) {
            return Optional.empty();
        }

        double[] pose = poseSub.get();
        return Optional.of(new Pose2d(
            pose[0],  // x (meters)
            pose[1],  // y (meters)
            new Rotation2d(pose[2])  // theta (radians)
        ));
    }

    /**
     * Get standard deviations for pose estimator
     */
    public Matrix<N3, N1> getStdDevs() {
        double[] stdDevs = stdDevsSub.get();
        return VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[2]);
    }

    /**
     * Get vision confidence (0-1)
     */
    public double getConfidence() {
        return confidenceSub.get();
    }

    /**
     * Get number of tags currently visible
     */
    public int getVisibleTagCount() {
        return (int) tagCountSub.get();
    }
}
```

### 2. Fusion with SwerveDrivePoseEstimator

```java
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

public class DriveSubsystem extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final VisionSubsystem vision;

    public DriveSubsystem(VisionSubsystem vision) {
        this.vision = vision;

        poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            getGyroAngle(),
            getModulePositions(),
            new Pose2d(),
            // Vision measurement std devs (updated dynamically)
            VecBuilder.fill(0.5, 0.5, 0.5),
            // Odometry std devs
            VecBuilder.fill(0.05, 0.05, 0.01)
        );
    }

    @Override
    public void periodic() {
        // Always update with odometry
        poseEstimator.update(getGyroAngle(), getModulePositions());

        // Add vision measurement if available and confident
        var visionPose = vision.getVisionPose();
        double confidence = vision.getConfidence();

        if (visionPose.isPresent() && confidence > 0.5) {
            // Dynamic standard deviations based on vision confidence
            var stdDevs = vision.getStdDevs();

            // Scale stddevs based on confidence
            // High confidence (0.9) → trust vision more → smaller stddevs
            // Low confidence (0.5) → trust vision less → larger stddevs
            double scaleFactor = 2.0 - confidence;  // 1.1 to 1.5
            stdDevs = stdDevs.times(scaleFactor);

            // Add vision measurement
            poseEstimator.addVisionMeasurement(
                visionPose.get(),
                Timer.getFPGATimestamp(),
                stdDevs
            );

            SmartDashboard.putString("Vision/Status", "FUSING");
            SmartDashboard.putNumber("Vision/Confidence", confidence);
        } else {
            SmartDashboard.putString("Vision/Status", "ODOMETRY ONLY");
        }

        // Publish fused pose
        var fusedPose = poseEstimator.getEstimatedPosition();
        SmartDashboard.putString("Robot/Pose",
            String.format("(%.2f, %.2f, %.1f°)",
                fusedPose.getX(),
                fusedPose.getY(),
                fusedPose.getRotation().getDegrees()));
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
}
```

### 3. Auto-Align to Target

```java
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlignCommand extends Command {
    private final DriveSubsystem drive;
    private final NetworkTable alignTable;

    private final DoubleArraySubscriber targetPoseSub;
    private final DoubleArraySubscriber robotPoseSub;
    private final BooleanSubscriber readySub;
    private final DoubleSubscriber headingErrorSub;

    private final IntegerPublisher targetTagPub;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    public AutoAlignCommand(DriveSubsystem drive, int targetTagId) {
        this.drive = drive;
        addRequirements(drive);

        var ntInst = NetworkTableInstance.getDefault();
        alignTable = ntInst.getTable("FRCVision").getSubTable("auto_align");

        // Subscribers
        targetPoseSub = alignTable.getDoubleArrayTopic("target_pose")
            .subscribe(new double[]{0, 0, 0});
        robotPoseSub = alignTable.getDoubleArrayTopic("robot_pose")
            .subscribe(new double[]{0, 0, 0});
        readySub = alignTable.getBooleanTopic("ready")
            .subscribe(false);
        headingErrorSub = alignTable.getDoubleTopic("heading_error_deg")
            .subscribe(999.0);

        // Publisher
        targetTagPub = alignTable.getIntegerTopic("target_tag_id")
            .publish();

        // PID controllers
        xController = new PIDController(2.0, 0, 0);
        yController = new PIDController(2.0, 0, 0);
        thetaController = new PIDController(3.0, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        // Tell vision system which tag to align to
        targetTagPub.set(7);  // Speaker tag (usually 7)

        System.out.println("[AutoAlign] Aligning to tag 7 (speaker)");
    }

    @Override
    public void execute() {
        double[] targetPose = targetPoseSub.get();
        double[] robotPose = robotPoseSub.get();

        // Calculate errors
        double xError = targetPose[0] - robotPose[0];
        double yError = targetPose[1] - robotPose[1];
        double thetaError = targetPose[2] - robotPose[2];

        // Normalize angle error
        while (thetaError > Math.PI) thetaError -= 2 * Math.PI;
        while (thetaError < -Math.PI) thetaError += 2 * Math.PI;

        // PID outputs
        double xVel = xController.calculate(0, -xError);  // Drive to reduce error
        double yVel = yController.calculate(0, -yError);
        double omegaVel = thetaController.calculate(0, -thetaError);

        // Apply to swerve drive (field-relative)
        var currentPose = drive.getPose();
        ChassisSpeeds fieldRelative = new ChassisSpeeds(xVel, yVel, omegaVel);
        ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelative, currentPose.getRotation());

        drive.drive(robotRelative);

        // Dashboard
        SmartDashboard.putNumber("AutoAlign/XError", xError);
        SmartDashboard.putNumber("AutoAlign/YError", yError);
        SmartDashboard.putNumber("AutoAlign/ThetaError", Math.toDegrees(thetaError));
        SmartDashboard.putBoolean("AutoAlign/Ready", readySub.get());
    }

    @Override
    public boolean isFinished() {
        // Done when vision says we're aligned
        return readySub.get();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        if (!interrupted) {
            System.out.println("[AutoAlign] ✓ Aligned and ready!");
        }
    }
}
```

### 4. Semicircle Shooter Auto

```java
public class SemicircleShooterAuto extends Command {
    private final DriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final NetworkTable semicircleTable;

    private final DoubleArraySubscriber velocitiesSub;
    private final BooleanSubscriber readySub;
    private final DoubleSubscriber progressSub;

    private boolean hasFired = false;

    public SemicircleShooterAuto(DriveSubsystem drive, ShooterSubsystem shooter) {
        this.drive = drive;
        this.shooter = shooter;
        addRequirements(drive);

        var ntInst = NetworkTableInstance.getDefault();
        semicircleTable = ntInst.getTable("FRCVision").getSubTable("semicircle");

        velocitiesSub = semicircleTable.getDoubleArrayTopic("velocities")
            .subscribe(new double[]{0, 0, 0});
        readySub = semicircleTable.getBooleanTopic("ready_to_shoot")
            .subscribe(false);
        progressSub = semicircleTable.getDoubleTopic("path_progress")
            .subscribe(0.0);
    }

    @Override
    public void initialize() {
        hasFired = false;
        shooter.setVelocity(4000);  // Spin up shooter
        System.out.println("[SemicircleAuto] Starting semicircle path around hub");
    }

    @Override
    public void execute() {
        // Get velocity commands from vision
        double[] velocities = velocitiesSub.get();  // [vx, vy, omega]

        ChassisSpeeds speeds = new ChassisSpeeds(
            velocities[0],  // vx (m/s)
            velocities[1],  // vy (m/s)
            velocities[2]   // omega (rad/s)
        );

        drive.drive(speeds);

        // Check if ready to shoot
        if (!hasFired && readySub.get() && shooter.atVelocity()) {
            System.out.println("[SemicircleAuto] 🎯 FIRING!");
            shooter.fire();
            hasFired = true;
        }

        // Dashboard
        SmartDashboard.putNumber("SemicircleAuto/Progress", progressSub.get());
        SmartDashboard.putBoolean("SemicircleAuto/ReadyToShoot", readySub.get());
        SmartDashboard.putBoolean("SemicircleAuto/HasFired", hasFired);
    }

    @Override
    public boolean isFinished() {
        // Done when path complete (progress = 1.0)
        return progressSub.get() >= 0.99;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        shooter.stop();
        System.out.println("[SemicircleAuto] Complete! Shot fired: " + hasFired);
    }
}
```

### 5. Vision-Based Odometry (WPILib 2025+)

```java
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonPoseEstimator poseEstimator;
    private final AprilTagFieldLayout fieldLayout;

    // AprilVision 3.1 subscribers
    private final DoubleArraySubscriber visionPoseSub;
    private final IntegerArraySubscriber tagIdsSub;

    public VisionSubsystem() {
        // Load field layout
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(
                AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            throw new RuntimeException("Failed to load field layout");
        }

        // Subscribe to AprilVision data
        var ntInst = NetworkTableInstance.getDefault();
        var visionTable = ntInst.getTable("FRCVision").getSubTable("fused");

        visionPoseSub = visionTable.getDoubleArrayTopic("pose")
            .subscribe(new double[]{0, 0, 0});
        tagIdsSub = visionTable.getIntegerArrayTopic("tag_ids")
            .subscribe(new long[]{});
    }

    /**
     * Get vision measurement for pose estimator
     *
     * AprilVision 3.1 already does all the heavy lifting (multi-tag fusion,
     * consistency checking, accuracy estimation). We just consume the result!
     */
    public Optional<VisionMeasurement> getVisionMeasurement() {
        double[] pose2d = visionPoseSub.get();

        if (pose2d.length < 3) {
            return Optional.empty();
        }

        // Convert 2D pose to 3D (assuming robot on ground)
        Pose2d pose = new Pose2d(pose2d[0], pose2d[1], new Rotation2d(pose2d[2]));

        // Get tag IDs for logging
        long[] tagIds = tagIdsSub.get();

        return Optional.of(new VisionMeasurement(
            pose,
            Timer.getFPGATimestamp(),
            tagIds
        ));
    }

    public record VisionMeasurement(Pose2d pose, double timestamp, long[] tagIds) {}
}
```

---

## Complete Robot Example

```java
public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // Log vision status to dashboard
        robotContainer.updateDashboard();
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }
}

public class RobotContainer {
    private final VisionSubsystem vision = new VisionSubsystem();
    private final DriveSubsystem drive = new DriveSubsystem(vision);
    private final ShooterSubsystem shooter = new ShooterSubsystem();

    private final CommandXboxController driverController =
        new CommandXboxController(0);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Default drive command
        drive.setDefaultCommand(
            drive.driveCommand(
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()
            )
        );

        // Auto-align to speaker on A button
        driverController.a()
            .whileTrue(new AutoAlignCommand(drive, 7));  // Tag 7 = speaker

        // Manual shoot on B button
        driverController.b()
            .onTrue(Commands.runOnce(() -> shooter.fire()));
    }

    public Command getAutonomousCommand() {
        // Semicircle shooter auto
        return new SemicircleShooterAuto(drive, shooter);
    }

    public void updateDashboard() {
        var visionPose = vision.getVisionPose();
        if (visionPose.isPresent()) {
            var pose = visionPose.get();
            SmartDashboard.putString("Vision/Pose",
                String.format("(%.2f, %.2f, %.1f°)",
                    pose.getX(), pose.getY(),
                    pose.getRotation().getDegrees()));
        } else {
            SmartDashboard.putString("Vision/Pose", "NO TAGS");
        }

        SmartDashboard.putNumber("Vision/TagCount", vision.getVisibleTagCount());
        SmartDashboard.putNumber("Vision/Confidence", vision.getConfidence());
    }
}
```

---

## Testing & Debugging

### 1. Check NetworkTables Connection

**On Driver Station:**
```
Outline Viewer → Connect to roboRIO → Navigate to /FRCVision
```

You should see:
- `/FRCVision/status/nt_connected` = true
- `/FRCVision/status/nt_server_ip` = "10.TE.AM.2"
- `/FRCVision/fused/valid` = true (when tags visible)

### 2. Verify Pose Data

**Java (robot code):**
```java
@Override
public void teleopPeriodic() {
    var pose = vision.getVisionPose();
    if (pose.isPresent()) {
        System.out.println("Vision pose: " + pose.get());
    } else {
        System.out.println("No vision pose available");
    }
}
```

### 3. Dashboard Monitoring

**Add to Robot.java:**
```java
SmartDashboard.putData("Vision", vision);
SmartDashboard.putData("Drive", drive);

// AprilVision status
SmartDashboard.putBoolean("Vision/Connected",
    NetworkTableInstance.getDefault()
        .getTable("FRCVision")
        .getSubTable("status")
        .getBooleanTopic("nt_connected")
        .subscribe(false)
        .get());
```

---

## Performance Tips

### 1. Use Polling Rate Limiting
```java
// Don't query NT faster than vision updates (50 Hz = 20ms)
private final Notifier visionUpdateNotifier;

public VisionSubsystem() {
    visionUpdateNotifier = new Notifier(this::updateVisionPose);
    visionUpdateNotifier.startPeriodic(0.02);  // 20ms = 50 Hz
}

private void updateVisionPose() {
    // Update pose estimate here
}
```

### 2. Cache Subscribers
```java
// ❌ BAD - Creates new subscriber every call
public double[] getPose() {
    return NetworkTableInstance.getDefault()
        .getTable("FRCVision")
        .getSubTable("fused")
        .getDoubleArrayTopic("pose")
        .subscribe(new double[]{0,0,0})
        .get();
}

// ✓ GOOD - Reuse subscriber
private final DoubleArraySubscriber poseSub;

public VisionSubsystem() {
    poseSub = NetworkTableInstance.getDefault()
        .getTable("FRCVision")
        .getSubTable("fused")
        .getDoubleArrayTopic("pose")
        .subscribe(new double[]{0,0,0});
}

public double[] getPose() {
    return poseSub.get();
}
```

### 3. Use Timestamped Values
```java
// Get both value and timestamp atomically
TimestampedDoubleArray stamped = poseSub.getAtomic();

double[] pose = stamped.value;
long timestampUs = stamped.timestamp;
double timestampSec = timestampUs / 1e6;

// Use for latency compensation
poseEstimator.addVisionMeasurement(pose, timestampSec, stdDevs);
```

---

## Troubleshooting

### Problem: "Vision subsystem not publishing data"

**Check:**
1. Is AprilVision service running?
   ```bash
   ssh admin@10.TE.AM.11
   sudo systemctl status frc_vision
   ```

2. Are cameras connected?
   ```bash
   ls /dev/video*
   ```

3. Check NT connection in dashboard
   - Should show "NT: Connected (10.TE.AM.2)"

### Problem: "Poses are wrong/jumping"

**Solutions:**
1. Verify field layout matches game
2. Check alliance color (red vs blue)
3. Enable pose consistency checking (should be on by default)
4. Check for reflections/false detections

### Problem: "Auto-align not working"

**Check:**
1. Is target tag visible?
   ```java
   boolean targetVisible = alignTable
       .getBooleanTopic("target_visible")
       .subscribe(false)
       .get();
   ```

2. Did you publish target tag ID?
   ```java
   targetTagPub.set(7);  // Must call this!
   ```

3. Check heading error magnitude

---

## Summary

✅ **All vision processing happens in C++ (AprilVision 3.1)**
✅ **All robot code is 100% Java (your code)**
✅ **Communication via NetworkTables 4.0**
✅ **Examples provided for all Phase 3 features**
✅ **WPILib integration with SwerveDrivePoseEstimator**
✅ **Auto-align, semicircle shooter, and more!**

**No C++ knowledge required for robot code! 🎉**
