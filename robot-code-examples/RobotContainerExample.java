package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlignToTagCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Example RobotContainer showing AprilVision 2.0 + PhotonLib integration.
 *
 * AprilVision 2.0 - Custom Vision System built on PhotonVision libraries.
 *
 * This demonstrates:
 * - Setting up VisionSubsystem with PhotonVision cameras
 * - Fusing vision poses into SwerveDrivePoseEstimator
 * - Binding auto-align commands to buttons
 * - Using vision data in autonomous routines
 *
 * Prerequisites:
 * - PhotonVision running on coprocessor (setup with ./setup.sh)
 * - PhotonLib dependency in build.gradle:
 *     implementation 'org.photonvision:photonlib-java:v2026.2.2'
 * - Cameras named "front", "left", "right" in PhotonVision UI
 */
public class RobotContainerExample {

    // Subsystems
    private final DriveSubsystem drive = new DriveSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem();

    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public RobotContainerExample() {
        configureBindings();
    }

    private void configureBindings() {
        // ====================================================================
        // Driver Controls - Auto-Alignment using PhotonVision
        // ====================================================================

        // A button - Align to tag 7 (close range, for scoring)
        driver.a().whileTrue(
            new AlignToTagCommand(vision, drive, 7)
        );

        // B button - Align to tag 3 (medium range, for approach)
        driver.b().whileTrue(
            AlignToTagCommand.alignMedium(vision, drive, 3)
        );

        // X button - Align to nearest visible tag using inline command
        driver.x().whileTrue(
            Commands.run(() -> {
                var target = vision.getBestFrontTarget();
                if (target.isPresent()) {
                    var t = target.get();
                    var camToTarget = t.getBestCameraToTarget();
                    double yaw = Math.atan2(camToTarget.getY(), camToTarget.getX());
                    // Simple proportional control to face the tag
                    drive.driveRobotRelative(0, 0, -yaw * 2.0);
                } else {
                    drive.stop();
                }
            }, drive)
        );

        // ====================================================================
        // Operator Controls
        // ====================================================================

        // D-pad alignments to specific tags
        operator.povUp().whileTrue(AlignToTagCommand.alignClose(vision, drive, 1));
        operator.povDown().whileTrue(AlignToTagCommand.alignClose(vision, drive, 2));
        operator.povLeft().whileTrue(AlignToTagCommand.alignClose(vision, drive, 3));
        operator.povRight().whileTrue(AlignToTagCommand.alignClose(vision, drive, 4));
    }

    /**
     * Call this from Robot.java's robotPeriodic() to fuse vision with odometry.
     *
     * This is the core integration point - it feeds PhotonVision pose estimates
     * into WPILib's SwerveDrivePoseEstimator with appropriate trust levels.
     */
    public void updateVisionPoseEstimator() {
        SwerveDrivePoseEstimator poseEstimator = drive.getPoseEstimator();

        // Process each camera's pose estimate
        processCamera(poseEstimator, vision.getFrontCameraPose());
        processCamera(poseEstimator, vision.getLeftCameraPose());
        processCamera(poseEstimator, vision.getRightCameraPose());
    }

    /**
     * Processes a single camera's pose estimate and feeds it to the pose estimator.
     */
    private void processCamera(SwerveDrivePoseEstimator poseEstimator,
                               Optional<EstimatedRobotPose> cameraPose) {
        if (cameraPose.isEmpty()) return;

        EstimatedRobotPose estimate = cameraPose.get();

        // Skip high-ambiguity single-tag results
        if (estimate.targetsUsed.size() == 1) {
            double ambiguity = estimate.targetsUsed.get(0).getPoseAmbiguity();
            if (ambiguity > 0.2) return;  // Too ambiguous to trust
        }

        // Calculate standard deviations based on distance and tag count
        var stdDevs = vision.getEstimationStdDevs(estimate);

        // Feed the pose estimate to the SwerveDrivePoseEstimator
        // with the timestamp from when the image was captured
        poseEstimator.addVisionMeasurement(
            estimate.estimatedPose.toPose2d(),
            estimate.timestampSeconds,
            stdDevs
        );
    }

    /**
     * Gets the autonomous command.
     */
    public Command getAutonomousCommand() {
        return Commands.sequence(
            Commands.print("[Auto] Starting autonomous..."),

            // Wait for vision system to see tags
            Commands.waitUntil(() -> vision.hasTargets())
                .withTimeout(2.0),

            // Drive forward
            drive.driveDistanceCommand(2.0, 0.0),

            // Align to scoring tag
            new AlignToTagCommand(vision, drive, 7, 0.5, 3.0),

            // Confirm alignment
            Commands.either(
                Commands.print("[Auto] Aligned to tag!"),
                Commands.print("[Auto] Alignment timeout, continuing"),
                () -> vision.isTagVisible(7)
            ),

            Commands.print("[Auto] Autonomous complete!")
        );
    }

    /**
     * Gets a vision-assisted teleop drive command.
     * When the driver holds right trigger, rotation is assisted by vision
     * to keep facing the nearest visible tag.
     */
    public Command getVisionAssistedDriveCommand() {
        return Commands.run(() -> {
            double xInput = -driver.getLeftY();
            double yInput = -driver.getLeftX();
            double rotInput = -driver.getRightX();

            // Vision-assisted rotation when right trigger held
            if (driver.getRightTriggerAxis() > 0.5) {
                var target = vision.getBestFrontTarget();
                if (target.isPresent()) {
                    var camToTarget = target.get().getBestCameraToTarget();
                    double yawError = Math.atan2(camToTarget.getY(), camToTarget.getX());
                    // Blend driver input with vision correction
                    rotInput = rotInput * 0.3 + (-yawError * 2.0) * 0.7;
                }
            }

            drive.driveFieldRelative(xInput * 4.0, yInput * 4.0, rotInput * 3.0);
        }, drive);
    }
}
