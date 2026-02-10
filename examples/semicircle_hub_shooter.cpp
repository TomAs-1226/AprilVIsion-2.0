/**
 * @file semicircle_hub_shooter.cpp
 * @brief Example: Semicircle path around hub with dynamic heading and tag handoff
 *
 * FRC 2025/2026 Use Case:
 * - Robot travels on semicircle path around central hub
 * - Continuously aims shooter at hub center while moving
 * - Smoothly transitions between visible AprilTags
 * - Shoots when aligned and at optimal position
 *
 * This demonstrates:
 * - Curved path following (semicircle)
 * - Dynamic heading control (face hub while moving)
 * - Smart tag handoff (switches tags smoothly)
 * - Continuous pose estimation during motion
 */

#include "phase3_curved_paths.hpp"
#include "pose.hpp"
#include <iostream>
#include <chrono>

using namespace frc_vision;
using namespace frc_vision::phase3;

int main() {
    std::cout << "==============================================\n";
    std::cout << " Semicircle Hub Shooter - AprilVision 2.1\n";
    std::cout << "==============================================\n\n";

    // =========================================================================
    // SETUP
    // =========================================================================

    // Hub location (field coordinates)
    // For FRC 2025/2026, adjust based on actual field layout
    double HUB_CENTER_X = 8.27;  // Center of field (meters)
    double HUB_CENTER_Y = 4.105; // Center of field (meters)

    // Shooting parameters
    double SHOOTING_RADIUS = 2.5;  // Stay 2.5m from hub center
    double SHOOTING_SPEED = 1500;  // RPM (example)

    // Initialize semicircle shooter system
    SemicircleShooter shooter(HUB_CENTER_X, HUB_CENTER_Y);

    // Example: Robot starts at this position
    Pose2D current_pose;
    current_pose.x = 6.0;   // 6m from origin
    current_pose.y = 4.0;   // 4m from origin
    current_pose.theta = 0.0;  // Facing +X

    std::cout << "Starting position: (" << current_pose.x << ", "
              << current_pose.y << "), heading: "
              << (current_pose.theta * 180 / M_PI) << "°\n\n";

    // =========================================================================
    // PLAN SEMICIRCLE PATH
    // =========================================================================

    std::cout << "Planning semicircle path around hub...\n";
    auto path = shooter.plan_semicircle_approach(current_pose, SHOOTING_RADIUS);

    std::cout << "Path planned:\n";
    std::cout << "  Radius: " << path.radius << "m\n";
    std::cout << "  Length: " << path.length << "m\n";
    std::cout << "  Max velocity: " << path.max_velocity << " m/s\n";
    std::cout << "  Direction: " << (path.clockwise ? "Clockwise" : "Counter-clockwise") << "\n\n";

    // =========================================================================
    // MAIN CONTROL LOOP
    // =========================================================================

    std::cout << "Starting semicircle motion with dynamic heading...\n";
    std::cout << "Robot will:\n";
    std::cout << "  1. Follow semicircle path around hub\n";
    std::cout << "  2. Continuously aim at hub center\n";
    std::cout << "  3. Switch between visible AprilTags\n";
    std::cout << "  4. Shoot when aligned\n\n";

    // Simulate control loop (50 Hz)
    double dt = 0.02;  // 20ms per cycle
    double elapsed_time = 0.0;
    bool has_shot = false;

    // Simulate some AprilTag detections
    // In real code, these come from vision system
    std::vector<TagDetection> simulated_tags = {
        // Tag 1: Close and high quality
        {.id = 1, .pose_valid = true, .distance_m = 2.3,
         .accuracy_estimate = {.confidence_level = "high", .estimated_error_m = 0.02}},

        // Tag 2: Medium distance
        {.id = 2, .pose_valid = true, .distance_m = 3.1,
         .accuracy_estimate = {.confidence_level = "medium", .estimated_error_m = 0.05}},

        // Tag 3: Far away
        {.id = 3, .pose_valid = true, .distance_m = 4.5,
         .accuracy_estimate = {.confidence_level = "low", .estimated_error_m = 0.10}},
    };

    // =========================================================================
    // CONTROL LOOP
    // =========================================================================

    while (elapsed_time < 10.0) {  // Run for 10 seconds max

        // Get guidance commands from semicircle shooter
        auto guidance = shooter.execute(current_pose, path, simulated_tags, dt);

        // Send commands to swerve drive
        // In real code: send guidance.drive_velocity_mps, strafe_velocity_mps, angular_velocity_rps
        // to your drive subsystem

        std::cout << "[t=" << std::fixed << std::setprecision(2) << elapsed_time << "s] ";
        std::cout << "Stage: " << guidance.current_stage;
        std::cout << ", Dist: " << guidance.distance_to_target_m << "m";
        std::cout << ", Heading err: " << guidance.heading_error_deg << "°";
        std::cout << ", Vel: (" << guidance.drive_velocity_mps << ", "
                  << guidance.strafe_velocity_mps << ", "
                  << guidance.angular_velocity_rps << ")\n";

        // Check if ready to shoot
        if (!has_shot &&
            std::abs(guidance.heading_error_deg) < 3.0 &&  // Heading within 3°
            guidance.distance_to_target_m < 5.0 &&          // Not too far
            guidance.is_aligned) {                          // On path

            std::cout << "\n🎯 SHOOTING! Heading aligned within tolerance.\n\n";
            has_shot = true;

            // In real code:
            // shooter_subsystem.spin_up(SHOOTING_SPEED);
            // shooter_subsystem.fire();
        }

        // Simulate robot motion (in real code, this comes from odometry + vision fusion)
        // Very simplified simulation: move along path
        double motion_x = guidance.drive_velocity_mps * std::cos(current_pose.theta) * dt;
        double motion_y = guidance.drive_velocity_mps * std::sin(current_pose.theta) * dt;
        motion_x += guidance.strafe_velocity_mps * std::cos(current_pose.theta + M_PI/2) * dt;
        motion_y += guidance.strafe_velocity_mps * std::sin(current_pose.theta + M_PI/2) * dt;

        current_pose.x += motion_x;
        current_pose.y += motion_y;
        current_pose.theta += guidance.angular_velocity_rps * dt;

        // Normalize angle
        while (current_pose.theta > M_PI) current_pose.theta -= 2*M_PI;
        while (current_pose.theta < -M_PI) current_pose.theta += 2*M_PI;

        // Check if path complete
        if (guidance.current_stage == "path_complete") {
            std::cout << "\n✓ Semicircle path complete!\n";
            break;
        }

        elapsed_time += dt;

        // In real code, this would be one periodic() call
        // Sleep to simulate real-time (not needed in actual robot code)
        // std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // =========================================================================
    // SUMMARY
    // =========================================================================

    std::cout << "\n==============================================\n";
    std::cout << "Final robot position:\n";
    std::cout << "  X: " << current_pose.x << "m\n";
    std::cout << "  Y: " << current_pose.y << "m\n";
    std::cout << "  Heading: " << (current_pose.theta * 180 / M_PI) << "°\n";
    std::cout << "\nTotal time: " << elapsed_time << "s\n";
    std::cout << "Shot taken: " << (has_shot ? "YES ✓" : "NO") << "\n";
    std::cout << "==============================================\n";

    return 0;
}

/**
 * INTEGRATION WITH YOUR ROBOT CODE (WPILib):
 *
 * ```java
 * public class SemicircleShooterAuto extends SequentialCommandGroup {
 *
 *     public SemicircleShooterAuto(DriveSubsystem drive, ShooterSubsystem shooter, VisionSubsystem vision) {
 *         addCommands(
 *             // 1. Initialize semicircle path
 *             new InstantCommand(() -> {
 *                 vision.setSemicircleMode(true, HUB_CENTER, SHOOTING_RADIUS);
 *             }),
 *
 *             // 2. Follow path with dynamic heading
 *             new RunCommand(() -> {
 *                 // Get vision guidance
 *                 var guidance = vision.getSemicircleGuidance();
 *
 *                 // Send to swerve drive
 *                 ChassisSpeeds speeds = new ChassisSpeeds(
 *                     guidance.drive_velocity_mps,
 *                     guidance.strafe_velocity_mps,
 *                     guidance.angular_velocity_rps
 *                 );
 *
 *                 drive.drive(speeds);
 *
 *                 // Spin up shooter
 *                 shooter.setVelocity(SHOOTING_RPM);
 *
 *             }, drive).until(() -> vision.isReadyToShoot()),
 *
 *             // 3. Fire!
 *             new InstantCommand(() -> shooter.fire()),
 *
 *             // 4. Continue path or stop
 *             new WaitCommand(0.5),
 *             new InstantCommand(() -> drive.stop())
 *         );
 *     }
 * }
 * ```
 */
