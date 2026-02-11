package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Command to automatically align the robot to an AprilTag using PhotonVision.
 *
 * AprilVision 2.0 - Custom Vision System built on PhotonVision libraries.
 *
 * This command uses PhotonVision's target data to drive the robot toward
 * a specific AprilTag. It uses PID control on the tag's relative position
 * from the camera.
 *
 * Usage:
 *   // Align to tag 5
 *   new AlignToTagCommand(vision, drive, 5)
 *
 *   // Bind to button
 *   driverController.a().whileTrue(new AlignToTagCommand(vision, drive, 7));
 */
public class AlignToTagCommand extends Command {

    private final VisionSubsystem vision;
    private final DriveSubsystem drive;
    private final int tagId;
    private final double targetDistanceMeters;

    // PID Controllers
    private final PIDController forwardController;  // Forward/backward
    private final PIDController strafeController;    // Left/right
    private final PIDController rotationController;  // Rotation

    // Configuration
    private static final double MAX_SPEED = 2.0;       // m/s
    private static final double MAX_ROT_SPEED = 2.0;   // rad/s
    private static final double POSITION_TOLERANCE = 0.05;  // meters
    private static final double ANGLE_TOLERANCE = 0.03;     // radians (~1.7 deg)

    // Timeout
    private final double timeoutSeconds;
    private double startTime;

    // State
    private boolean targetEverSeen = false;

    /**
     * Creates an AlignToTagCommand.
     *
     * @param vision Vision subsystem
     * @param drive Drive subsystem
     * @param tagId AprilTag ID to align to
     */
    public AlignToTagCommand(VisionSubsystem vision, DriveSubsystem drive, int tagId) {
        this(vision, drive, tagId, 0.5, 10.0);
    }

    /**
     * Creates an AlignToTagCommand with custom distance and timeout.
     *
     * @param vision Vision subsystem
     * @param drive Drive subsystem
     * @param tagId AprilTag ID to align to
     * @param targetDistanceMeters Desired distance from tag (meters)
     * @param timeoutSeconds Maximum alignment time
     */
    public AlignToTagCommand(VisionSubsystem vision, DriveSubsystem drive,
                            int tagId, double targetDistanceMeters,
                            double timeoutSeconds) {
        this.vision = vision;
        this.drive = drive;
        this.tagId = tagId;
        this.targetDistanceMeters = targetDistanceMeters;
        this.timeoutSeconds = timeoutSeconds;

        // PID controllers - tune these for your robot
        forwardController = new PIDController(1.5, 0.0, 0.08);
        strafeController = new PIDController(1.5, 0.0, 0.08);
        rotationController = new PIDController(2.0, 0.0, 0.1);

        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        forwardController.setTolerance(POSITION_TOLERANCE);
        strafeController.setTolerance(POSITION_TOLERANCE);
        rotationController.setTolerance(ANGLE_TOLERANCE);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        forwardController.reset();
        strafeController.reset();
        rotationController.reset();
        startTime = Timer.getFPGATimestamp();
        targetEverSeen = false;

        System.out.println("[AlignToTag] Starting alignment to tag " + tagId);
    }

    @Override
    public void execute() {
        // Look for the target tag in camera results
        Optional<PhotonTrackedTarget> target = findTargetTag();

        if (target.isEmpty()) {
            drive.stop();
            return;
        }

        targetEverSeen = true;
        PhotonTrackedTarget t = target.get();

        // PhotonVision gives us the target's position relative to the camera
        // as a Transform3d (best camera to target transform)
        Transform3d camToTarget = t.getBestCameraToTarget();

        // Extract target position relative to camera
        // X = forward distance to tag
        // Y = left/right offset (positive = left)
        // Z = up/down offset (positive = up)
        double forwardDistance = camToTarget.getX();
        double strafeOffset = camToTarget.getY();
        double yawToTarget = Math.atan2(strafeOffset, forwardDistance);

        // Calculate errors
        double forwardError = forwardDistance - targetDistanceMeters;
        double strafeError = strafeOffset;  // Want to center on tag (0 offset)
        double rotError = yawToTarget;       // Want to face tag (0 yaw)

        // PID outputs
        double forwardSpeed = forwardController.calculate(forwardError, 0);
        double strafeSpeed = strafeController.calculate(strafeError, 0);
        double rotSpeed = rotationController.calculate(rotError, 0);

        // Clamp speeds
        forwardSpeed = MathUtil.clamp(forwardSpeed, -MAX_SPEED, MAX_SPEED);
        strafeSpeed = MathUtil.clamp(strafeSpeed, -MAX_SPEED, MAX_SPEED);
        rotSpeed = MathUtil.clamp(rotSpeed, -MAX_ROT_SPEED, MAX_ROT_SPEED);

        // Deadband for small errors
        if (Math.abs(forwardError) < POSITION_TOLERANCE) forwardSpeed = 0;
        if (Math.abs(strafeError) < POSITION_TOLERANCE) strafeSpeed = 0;
        if (Math.abs(rotError) < ANGLE_TOLERANCE) rotSpeed = 0;

        // Drive the robot (robot-relative since we're using camera frame)
        drive.driveRobotRelative(forwardSpeed, -strafeSpeed, -rotSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        if (interrupted) {
            System.out.println("[AlignToTag] Alignment interrupted");
        } else {
            System.out.println("[AlignToTag] Alignment complete!");
        }
    }

    @Override
    public boolean isFinished() {
        // Timeout check
        if (Timer.getFPGATimestamp() - startTime > timeoutSeconds) {
            System.out.println("[AlignToTag] Timeout reached");
            return true;
        }

        // Check if all controllers are at setpoint
        return forwardController.atSetpoint() &&
               strafeController.atSetpoint() &&
               rotationController.atSetpoint();
    }

    /**
     * Searches all cameras for the target tag.
     */
    private Optional<PhotonTrackedTarget> findTargetTag() {
        // Check front camera first (most common for alignment)
        var frontResult = vision.getLatestFrontResult();
        if (frontResult.hasTargets()) {
            for (var target : frontResult.getTargets()) {
                if (target.getFiducialId() == tagId) {
                    return Optional.of(target);
                }
            }
        }
        return Optional.empty();
    }

    // ========================================================================
    // Static factory methods for common alignments
    // ========================================================================

    /**
     * Creates an alignment command for close-range scoring.
     */
    public static AlignToTagCommand alignClose(VisionSubsystem vision,
                                                DriveSubsystem drive,
                                                int tagId) {
        return new AlignToTagCommand(vision, drive, tagId, 0.4, 5.0);
    }

    /**
     * Creates an alignment command for medium-range approach.
     */
    public static AlignToTagCommand alignMedium(VisionSubsystem vision,
                                                 DriveSubsystem drive,
                                                 int tagId) {
        return new AlignToTagCommand(vision, drive, tagId, 1.0, 8.0);
    }
}
