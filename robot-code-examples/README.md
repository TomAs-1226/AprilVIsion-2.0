# Robot Code Examples

Java/WPILib examples for integrating the FRC Vision Coprocessor with your robot code.

## Files

- **VisionSubsystem.java** - Complete vision subsystem with pose estimation and auto-align
- **AlignToTagCommand.java** - Command for automatic alignment to AprilTags
- **RobotContainerExample.java** - Example showing button bindings and auto routines

## Quick Start

### 1. Copy Files

Copy these files to your robot project:
```
src/main/java/frc/robot/
├── subsystems/
│   └── VisionSubsystem.java
├── commands/
│   └── AlignToTagCommand.java
└── RobotContainer.java  (use as reference)
```

### 2. Initialize Vision Subsystem

In your `RobotContainer`:

```java
private final DriveSubsystem drive = new DriveSubsystem();
private final VisionSubsystem vision = new VisionSubsystem();

public RobotContainer() {
    // Connect vision to pose estimator
    vision.setPoseEstimator(drive.getPoseEstimator());
}
```

### 3. Use Pose Estimation

Vision automatically updates your pose estimator. No extra code needed!

```java
// Your drive subsystem already has the updated pose
Pose2d robotPose = drive.getPose();  // Includes vision corrections
```

### 4. Add Auto-Align Commands

```java
// Simple alignment
driver.a().whileTrue(new AlignToTagCommand(vision, drive, 5, 0.5));

// Custom distance and timeout
driver.b().whileTrue(new AlignToTagCommand(vision, drive, 7, 0.3, 0.0, 5.0));

// Static factory methods
driver.x().whileTrue(AlignToTagCommand.alignToSpeaker(vision, drive, 7));
```

## NetworkTables Topics

The vision coprocessor publishes to `/FRCVision/`:

### Pose Data
| Topic | Type | Description |
|-------|------|-------------|
| `fused/pose` | double[] | [x, y, theta] meters/radians |
| `fused/std_devs` | double[] | Standard deviations |
| `fused/valid` | boolean | Pose validity |
| `fused/confidence` | double | 0.0-1.0 confidence |

### Auto-Align (Robot → Vision)
| Topic | Type | Description |
|-------|------|-------------|
| `auto_align/target_tag_id` | int | Tag ID to align to (-1 = none) |
| `auto_align/target_offset` | double[] | [distance_m, angle_rad] |

### Auto-Align (Vision → Robot)
| Topic | Type | Description |
|-------|------|-------------|
| `auto_align/error` | double[] | [x, y, theta] alignment error |
| `auto_align/ready` | boolean | Within tolerance |
| `auto_align/target_visible` | boolean | Tag is visible |
| `auto_align/distance_m` | double | Distance to target |

## Tuning

### PID Gains

In `AlignToTagCommand.java`, tune these values for your robot:

```java
xController = new PIDController(2.0, 0.0, 0.1);   // X translation
yController = new PIDController(2.0, 0.0, 0.1);   // Y translation
thetaController = new PIDController(3.0, 0.0, 0.15);  // Rotation
```

Start with P-only control (I=0, D=0) and increase until the robot is responsive but not oscillating.

### Tolerances

Adjust in the command:
```java
private static final double POSITION_TOLERANCE = 0.03;  // 3 cm
private static final double ANGLE_TOLERANCE = 0.03;     // ~1.7 degrees
```

### Speed Limits

```java
private static final double MAX_SPEED = 2.0;      // m/s translation
private static final double MAX_ROT_SPEED = 2.0;  // rad/s rotation
```

## Troubleshooting

### Vision Not Updating Pose

1. Check vision is running: `curl http://orangepi:5800/api/status`
2. Verify NT connection in Shuffleboard/Glass
3. Check `vision.hasValidPose()` returns true
4. Verify cameras can see AprilTags

### Alignment Not Working

1. Check `vision.isTargetVisible()` returns true
2. Verify tag ID matches field layout
3. Check `vision.getAlignmentError()` returns valid values
4. Tune PID gains starting with P-only

### Robot Drives Wrong Direction

The error convention is `target - current`:
- Positive X error = robot needs to move forward (in field frame)
- Positive Y error = robot needs to move left (in field frame)
- Positive theta error = robot needs to rotate counter-clockwise

If directions are wrong, negate the corresponding speed in the command.

## Dependencies

Add to your `build.gradle`:
```gradle
dependencies {
    // WPILib (already included)
    implementation wpi.java.deps.wpilib()
    implementation wpi.java.vendor.java()
}
```

No additional dependencies required - uses standard WPILib NetworkTables.
