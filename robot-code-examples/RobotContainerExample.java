package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlignToTagCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Example RobotContainer showing vision integration.
 *
 * This demonstrates:
 * - Setting up VisionSubsystem with pose estimator
 * - Binding auto-align commands to buttons
 * - Using vision data in autonomous
 */
public class RobotContainerExample {

    // Subsystems
    private final DriveSubsystem drive = new DriveSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem();

    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // Tag IDs for FRC 2026 REBUILT
    // Hub tags (scoring): Red 2-5,8-11 / Blue 18-21,24-27
    // Tower wall tags: Red 15,16 / Blue 31,32
    // Outpost tags: Red 13,14 / Blue 29,30
    // Trench tags: Red 1,6,7,12 / Blue 17,22,23,28
    private static final int BLUE_HUB_TAG = 20;    // Blue hub face D (near upper)
    private static final int RED_HUB_TAG = 10;     // Red hub face D (near upper)
    private static final int BLUE_TOWER_TAG = 31;   // Blue tower wall (bottom)
    private static final int RED_TOWER_TAG = 15;    // Red tower wall (top)

    public RobotContainerExample() {
        // Connect vision to drive's pose estimator
        vision.setPoseEstimator(drive.getPoseEstimator());

        configureBindings();
    }

    private void configureBindings() {
        // ====================================================================
        // Driver Controls - Auto-Alignment
        // ====================================================================

        // A button - Align to hub (uses alliance color)
        driver.a().whileTrue(
            Commands.either(
                // Blue alliance
                new AlignToTagCommand(vision, drive, BLUE_HUB_TAG, 0.5),
                // Red alliance
                new AlignToTagCommand(vision, drive, RED_HUB_TAG, 0.5),
                () -> isBlueAlliance()
            )
        );

        // B button - Align to tower wall
        driver.b().whileTrue(
            Commands.either(
                new AlignToTagCommand(vision, drive, BLUE_TOWER_TAG, 0.3),
                new AlignToTagCommand(vision, drive, RED_TOWER_TAG, 0.3),
                () -> isBlueAlliance()
            )
        );

        // X button - Quick align using inline command
        driver.x().whileTrue(
            Commands.run(() -> {
                double[] error = vision.getAlignmentError();
                if (error != null && vision.isTargetVisible()) {
                    // Simple proportional control
                    double xSpeed = -error[0] * 2.0;
                    double ySpeed = -error[1] * 2.0;
                    double rotSpeed = -error[2] * 3.0;
                    drive.driveFieldRelative(xSpeed, ySpeed, rotSpeed);
                } else {
                    drive.stop();
                }
            }, drive)
            .beforeStarting(() -> vision.startAlignment(7, 0.5, 0))
            .finallyDo(() -> vision.stopAlignment())
        );

        // Y button - Lock onto nearest visible tag (dynamic targeting)
        driver.y().whileTrue(
            Commands.sequence(
                // First, check what tags are visible and pick one
                Commands.runOnce(() -> {
                    // You could implement tag selection logic here
                    // For now, just use tag 1
                    vision.startAlignment(1, 0.6, 0);
                }),
                // Then run alignment
                Commands.run(() -> {
                    double[] error = vision.getAlignmentError();
                    if (error != null) {
                        drive.driveFieldRelative(
                            -error[0] * 2.0,
                            -error[1] * 2.0,
                            -error[2] * 3.0
                        );
                    }
                }, drive)
            ).finallyDo(() -> vision.stopAlignment())
        );

        // ====================================================================
        // Operator Controls - Manual Alignment
        // ====================================================================

        // D-pad up - Align to tag 1
        operator.povUp().whileTrue(new AlignToTagCommand(vision, drive, 1, 0.5));

        // D-pad down - Align to tag 2
        operator.povDown().whileTrue(new AlignToTagCommand(vision, drive, 2, 0.5));

        // D-pad left - Align to tag 3
        operator.povLeft().whileTrue(new AlignToTagCommand(vision, drive, 3, 0.5));

        // D-pad right - Align to tag 4
        operator.povRight().whileTrue(new AlignToTagCommand(vision, drive, 4, 0.5));
    }

    /**
     * Gets the autonomous command.
     */
    public Command getAutonomousCommand() {
        return Commands.sequence(
            // Example auto: Drive forward, align to tag, score
            Commands.print("Starting auto..."),

            // Wait for vision to initialize
            Commands.waitUntil(() -> vision.hasValidPose())
                .withTimeout(2.0),

            // Drive towards scoring position
            drive.driveDistanceCommand(2.0, 0.0),

            // Align to hub tag
            new AlignToTagCommand(vision, drive, BLUE_HUB_TAG, 0.5)
                .withTimeout(3.0),

            // Check alignment
            Commands.either(
                Commands.print("Aligned! Ready to score"),
                Commands.print("Alignment failed, proceeding anyway"),
                () -> vision.isAligned()
            ),

            // Score game piece
            // shooter.shootCommand(),

            Commands.print("Auto complete!")
        );
    }

    /**
     * Example: Use vision pose in teleop for driver assistance.
     */
    public Command getVisionAssistedDriveCommand() {
        return Commands.run(() -> {
            // Get driver input
            double xInput = -driver.getLeftY();
            double yInput = -driver.getLeftX();
            double rotInput = -driver.getRightX();

            // If driver is holding right trigger, use vision-assisted rotation
            if (driver.getRightTriggerAxis() > 0.5 && vision.hasAlignmentTarget()) {
                double rotError = vision.getAlignmentErrorTheta();
                // Blend driver input with vision correction
                rotInput = rotInput * 0.3 + (-rotError * 3.0) * 0.7;
            }

            drive.driveFieldRelative(xInput * 4.0, yInput * 4.0, rotInput * 3.0);
        }, drive);
    }

    private boolean isBlueAlliance() {
        // Replace with actual alliance check
        // return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        return true;
    }
}
