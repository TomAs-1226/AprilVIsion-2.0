package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Command to automatically align the robot to an AprilTag.
 *
 * This command:
 * 1. Tells the vision system which tag to align to
 * 2. Uses PID control on the alignment error to drive the robot
 * 3. Finishes when the robot is within tolerance (vision reports "ready")
 *
 * Usage:
 *   // Align to tag 5 at 0.5 meters
 *   new AlignToTagCommand(vision, drive, 5, 0.5)
 *
 *   // Bind to button
 *   driverController.a().whileTrue(new AlignToTagCommand(vision, drive, 7, 0.6));
 */
public class AlignToTagCommand extends Command {

    private final VisionSubsystem vision;
    private final DriveSubsystem drive;
    private final int tagId;
    private final double distanceMeters;
    private final double approachAngle;

    // PID Controllers for each axis
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    // Configuration
    private static final double MAX_SPEED = 2.0;        // m/s
    private static final double MAX_ROT_SPEED = 2.0;    // rad/s
    private static final double POSITION_TOLERANCE = 0.03;  // meters
    private static final double ANGLE_TOLERANCE = 0.03;     // radians

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
     * @param distanceMeters Distance from tag to stop at
     */
    public AlignToTagCommand(VisionSubsystem vision, DriveSubsystem drive,
                            int tagId, double distanceMeters) {
        this(vision, drive, tagId, distanceMeters, 0.0, 10.0);
    }

    /**
     * Creates an AlignToTagCommand with custom angle and timeout.
     *
     * @param vision Vision subsystem
     * @param drive Drive subsystem
     * @param tagId AprilTag ID to align to
     * @param distanceMeters Distance from tag to stop at
     * @param approachAngle Approach angle offset from perpendicular (radians)
     * @param timeoutSeconds Maximum time to attempt alignment
     */
    public AlignToTagCommand(VisionSubsystem vision, DriveSubsystem drive,
                            int tagId, double distanceMeters, double approachAngle,
                            double timeoutSeconds) {
        this.vision = vision;
        this.drive = drive;
        this.tagId = tagId;
        this.distanceMeters = distanceMeters;
        this.approachAngle = approachAngle;
        this.timeoutSeconds = timeoutSeconds;

        // Configure PID controllers
        // Tune these values for your robot!
        xController = new PIDController(2.0, 0.0, 0.1);
        yController = new PIDController(2.0, 0.0, 0.1);
        thetaController = new PIDController(3.0, 0.0, 0.15);

        // Theta controller wraps around
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Set tolerances
        xController.setTolerance(POSITION_TOLERANCE);
        yController.setTolerance(POSITION_TOLERANCE);
        thetaController.setTolerance(ANGLE_TOLERANCE);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Tell vision to start tracking this tag
        vision.startAlignment(tagId, distanceMeters, approachAngle);

        // Reset controllers
        xController.reset();
        yController.reset();
        thetaController.reset();

        // Record start time
        startTime = Timer.getFPGATimestamp();
        targetEverSeen = false;

        System.out.println("[AlignToTag] Starting alignment to tag " + tagId);
    }

    @Override
    public void execute() {
        // Check if target is visible
        if (!vision.isTargetVisible()) {
            // Target not visible - stop and wait
            drive.stop();

            if (!targetEverSeen) {
                // Never seen the target - maybe rotate to find it?
                // Or just wait for it to appear
            }
            return;
        }

        targetEverSeen = true;

        // Get alignment error from vision
        double[] error = vision.getAlignmentError();
        if (error == null) {
            drive.stop();
            return;
        }

        // error[0] = x error (forward/back in field frame)
        // error[1] = y error (left/right in field frame)
        // error[2] = theta error (rotation)

        // Calculate control outputs
        // PID calculates output to drive error to zero
        double xSpeed = xController.calculate(error[0], 0);
        double ySpeed = yController.calculate(error[1], 0);
        double rotSpeed = thetaController.calculate(error[2], 0);

        // Clamp speeds
        xSpeed = MathUtil.clamp(xSpeed, -MAX_SPEED, MAX_SPEED);
        ySpeed = MathUtil.clamp(ySpeed, -MAX_SPEED, MAX_SPEED);
        rotSpeed = MathUtil.clamp(rotSpeed, -MAX_ROT_SPEED, MAX_ROT_SPEED);

        // Apply deadband for small errors
        if (Math.abs(error[0]) < POSITION_TOLERANCE) xSpeed = 0;
        if (Math.abs(error[1]) < POSITION_TOLERANCE) ySpeed = 0;
        if (Math.abs(error[2]) < ANGLE_TOLERANCE) rotSpeed = 0;

        // Drive the robot (field-relative)
        drive.driveFieldRelative(xSpeed, ySpeed, rotSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        drive.stop();

        // Stop alignment tracking
        vision.stopAlignment();

        if (interrupted) {
            System.out.println("[AlignToTag] Alignment interrupted");
        } else {
            System.out.println("[AlignToTag] Alignment complete!");
        }
    }

    @Override
    public boolean isFinished() {
        // Check for timeout
        if (Timer.getFPGATimestamp() - startTime > timeoutSeconds) {
            System.out.println("[AlignToTag] Timeout reached");
            return true;
        }

        // Check if vision reports aligned (uses its internal tolerances)
        if (vision.isAligned()) {
            return true;
        }

        // Also check our own PID tolerances
        return xController.atSetpoint() &&
               yController.atSetpoint() &&
               thetaController.atSetpoint();
    }

    // ========================================================================
    // Static factory methods for FRC 2026 REBUILT common alignments
    // ========================================================================

    /**
     * Creates an alignment command for hub scoring (close range).
     * Hub tags: Red 2-5,8-11 / Blue 18-21,24-27
     */
    public static AlignToTagCommand alignToHub(VisionSubsystem vision,
                                                DriveSubsystem drive,
                                                int hubTagId) {
        return new AlignToTagCommand(vision, drive, hubTagId, 0.5, 0.0, 5.0);
    }

    /**
     * Creates an alignment command for tower wall approach.
     * Tower wall tags: Red 15,16 / Blue 31,32
     */
    public static AlignToTagCommand alignToTowerWall(VisionSubsystem vision,
                                                      DriveSubsystem drive,
                                                      int towerTagId) {
        return new AlignToTagCommand(vision, drive, towerTagId, 0.3, 0.0, 5.0);
    }

    /**
     * Creates an alignment command for outpost approach.
     * Outpost tags: Red 13,14 / Blue 29,30
     */
    public static AlignToTagCommand alignToOutpost(VisionSubsystem vision,
                                                    DriveSubsystem drive,
                                                    int outpostTagId) {
        return new AlignToTagCommand(vision, drive, outpostTagId, 0.6, 0.0, 5.0);
    }
}
