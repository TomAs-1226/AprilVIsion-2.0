/**
 * @file phase3_autoalign.hpp
 * @brief Phase 3: Auto-align from far away - Trajectory planning and guidance
 *
 * Features:
 * - Calculate optimal shooting positions based on field geometry
 * - Provide trajectory guidance for robot navigation
 * - Multi-point alignment (when multiple targets visible)
 * - Safety checks and collision avoidance
 * - Real-time path updates as robot moves
 */

#pragma once

#include "types.hpp"
#include <vector>
#include <optional>
#include <cmath>
#include <algorithm>

namespace frc_vision {
namespace phase3 {

/**
 * @brief Target shooting position on the field
 */
struct ShootingPosition {
    Pose2D pose;                    // Target position (x, y, theta)
    double distance_to_target_m;    // Distance to speaker/target
    double expected_accuracy;       // 0-1 confidence in shot success
    std::string position_type;      // "optimal", "acceptable", "fallback"
    int visible_tags;               // Number of tags visible from this position
};

/**
 * @brief Auto-align trajectory waypoint
 */
struct AlignmentWaypoint {
    Pose2D pose;                    // Waypoint pose
    double speed_mps;               // Recommended speed
    double curvature;               // Path curvature (1/radius)
    bool requires_precision;        // High-precision navigation needed
};

/**
 * @brief Auto-align guidance command
 */
struct AlignmentGuidance {
    // Current state
    Pose2D current_pose;
    Pose2D target_pose;
    double distance_to_target_m;
    double heading_error_deg;

    // Navigation commands
    double drive_velocity_mps;      // Forward velocity (-5 to +5 m/s)
    double angular_velocity_rps;    // Rotation rate (-3 to +3 rad/s)
    double strafe_velocity_mps;     // Sideways velocity for swerve (-5 to +5 m/s)

    // Status
    bool is_aligned;                // Within tolerance
    bool ready_to_shoot;            // Aligned and stable
    double alignment_quality;       // 0-1 confidence

    // Multi-stage guidance
    std::string current_stage;      // "approach", "fine_align", "ready"
    std::vector<AlignmentWaypoint> waypoints;  // Path to target

    // Safety
    bool path_clear;                // No collisions detected
    std::vector<std::string> warnings;
};

/**
 * @brief Field zone definitions for FRC 2024/2025
 */
struct FieldZones {
    // Speaker positions (blue alliance coords)
    static constexpr double BLUE_SPEAKER_X = 0.0;
    static constexpr double BLUE_SPEAKER_Y = 5.55;  // Center of field
    static constexpr double BLUE_SPEAKER_Z = 2.05;  // Speaker height

    static constexpr double RED_SPEAKER_X = 16.54;
    static constexpr double RED_SPEAKER_Y = 5.55;
    static constexpr double RED_SPEAKER_Z = 2.05;

    // Optimal shooting zones (distance from speaker)
    static constexpr double OPTIMAL_SHOT_DISTANCE_MIN = 1.5;  // meters
    static constexpr double OPTIMAL_SHOT_DISTANCE_MAX = 3.5;
    static constexpr double ACCEPTABLE_SHOT_DISTANCE_MAX = 5.0;

    // Field boundaries
    static constexpr double FIELD_LENGTH = 16.54;  // meters
    static constexpr double FIELD_WIDTH = 8.21;
};

/**
 * @brief Auto-align trajectory planner (Phase 3)
 */
class AutoAlignPlanner {
public:
    AutoAlignPlanner(bool is_red_alliance = false)
        : is_red_alliance_(is_red_alliance)
        , alignment_tolerance_m_(0.05)      // 5cm position tolerance
        , alignment_tolerance_deg_(2.0)     // 2° angle tolerance
        , stable_time_threshold_s_(0.5)     // Must be stable for 0.5s
    {}

    /**
     * @brief Calculate optimal shooting positions for current robot pose
     * @param current_pose Current robot position
     * @param field_tags Detected AprilTags on field
     * @return Sorted list of shooting positions (best first)
     */
    std::vector<ShootingPosition> calculate_shooting_positions(
        const Pose2D& current_pose,
        const std::vector<TagDetection>& field_tags)
    {
        std::vector<ShootingPosition> positions;

        // Get speaker position based on alliance
        double speaker_x = is_red_alliance_ ?
            FieldZones::RED_SPEAKER_X : FieldZones::BLUE_SPEAKER_X;
        double speaker_y = is_red_alliance_ ?
            FieldZones::RED_SPEAKER_Y : FieldZones::BLUE_SPEAKER_Y;

        // Generate candidate positions in arc around speaker
        // Try distances from 1.5m to 5m, angles from -30° to +30°
        for (double distance = FieldZones::OPTIMAL_SHOT_DISTANCE_MIN;
             distance <= FieldZones::ACCEPTABLE_SHOT_DISTANCE_MAX;
             distance += 0.5) {

            for (double angle_deg = -30.0; angle_deg <= 30.0; angle_deg += 10.0) {
                double angle_rad = angle_deg * M_PI / 180.0;

                // Calculate position on arc
                double x = speaker_x + distance * std::cos(angle_rad + (is_red_alliance_ ? M_PI : 0.0));
                double y = speaker_y + distance * std::sin(angle_rad);

                // Check if position is on field
                if (x < 0.0 || x > FieldZones::FIELD_LENGTH ||
                    y < 0.0 || y > FieldZones::FIELD_WIDTH) {
                    continue;
                }

                // Calculate heading to face speaker
                double heading = std::atan2(speaker_y - y, speaker_x - x);

                ShootingPosition pos;
                pos.pose.x = x;
                pos.pose.y = y;
                pos.pose.theta = heading;
                pos.distance_to_target_m = distance;

                // Estimate accuracy based on distance and visible tags
                pos.visible_tags = count_visible_tags(pos.pose, field_tags);
                pos.expected_accuracy = calculate_shot_accuracy(distance, pos.visible_tags);

                // Classify position
                if (distance <= FieldZones::OPTIMAL_SHOT_DISTANCE_MAX) {
                    pos.position_type = pos.visible_tags >= 2 ? "optimal" : "acceptable";
                } else {
                    pos.position_type = "fallback";
                }

                positions.push_back(pos);
            }
        }

        // Sort by expected accuracy (best first)
        std::sort(positions.begin(), positions.end(),
                 [](const ShootingPosition& a, const ShootingPosition& b) {
                     return a.expected_accuracy > b.expected_accuracy;
                 });

        return positions;
    }

    /**
     * @brief Generate alignment guidance from current pose to target
     * @param current_pose Current robot position
     * @param target_pose Desired shooting position
     * @param current_velocity Current robot velocity (for smoothing)
     * @return Guidance commands for robot
     */
    AlignmentGuidance generate_guidance(
        const Pose2D& current_pose,
        const Pose2D& target_pose,
        double current_velocity_mps = 0.0)
    {
        AlignmentGuidance guidance;
        guidance.current_pose = current_pose;
        guidance.target_pose = target_pose;

        // Calculate errors
        double dx = target_pose.x - current_pose.x;
        double dy = target_pose.y - current_pose.y;
        guidance.distance_to_target_m = std::sqrt(dx * dx + dy * dy);

        double angle_to_target = std::atan2(dy, dx);
        double heading_error = normalize_angle(target_pose.theta - current_pose.theta);
        guidance.heading_error_deg = heading_error * 180.0 / M_PI;

        // Check if aligned
        guidance.is_aligned = (guidance.distance_to_target_m < alignment_tolerance_m_) &&
                             (std::abs(heading_error) < alignment_tolerance_deg_ * M_PI / 180.0);

        // Multi-stage approach
        if (guidance.distance_to_target_m > 2.0) {
            guidance.current_stage = "approach";
            guidance.alignment_quality = 0.3;
        } else if (guidance.distance_to_target_m > 0.5) {
            guidance.current_stage = "fine_align";
            guidance.alignment_quality = 0.6;
        } else if (!guidance.is_aligned) {
            guidance.current_stage = "fine_align";
            guidance.alignment_quality = 0.8;
        } else {
            guidance.current_stage = "ready";
            guidance.alignment_quality = 1.0;
            guidance.ready_to_shoot = true;
        }

        // Generate velocity commands based on stage
        if (guidance.current_stage == "approach") {
            // Fast approach: point toward target and drive
            double approach_angle_error = normalize_angle(angle_to_target - current_pose.theta);

            guidance.drive_velocity_mps = std::min(3.0, guidance.distance_to_target_m * 1.5);
            guidance.angular_velocity_rps = std::clamp(approach_angle_error * 2.0, -2.0, 2.0);
            guidance.strafe_velocity_mps = 0.0;  // No strafing during approach

        } else if (guidance.current_stage == "fine_align") {
            // Precise alignment: use holonomic drive for position + rotation
            // Transform error to robot frame
            double error_robot_x = dx * std::cos(-current_pose.theta) - dy * std::sin(-current_pose.theta);
            double error_robot_y = dx * std::sin(-current_pose.theta) + dy * std::cos(-current_pose.theta);

            // P-controller for velocity (slower for precision)
            guidance.drive_velocity_mps = std::clamp(error_robot_x * 2.0, -1.5, 1.5);
            guidance.strafe_velocity_mps = std::clamp(error_robot_y * 2.0, -1.5, 1.5);
            guidance.angular_velocity_rps = std::clamp(heading_error * 3.0, -1.5, 1.5);

        } else {
            // Ready: hold position
            guidance.drive_velocity_mps = 0.0;
            guidance.strafe_velocity_mps = 0.0;
            guidance.angular_velocity_rps = 0.0;
        }

        // Safety checks
        guidance.path_clear = true;  // TODO: Add collision detection

        if (guidance.distance_to_target_m > 8.0) {
            guidance.warnings.push_back("Target very far away (>8m)");
            guidance.alignment_quality *= 0.5;
        }

        // Generate waypoints for visualization
        guidance.waypoints = generate_waypoints(current_pose, target_pose, 5);

        return guidance;
    }

    /**
     * @brief Check if robot has been stably aligned
     * @param guidance Current guidance
     * @param dt Time since last check
     * @return True if aligned and stable
     */
    bool is_stable_aligned(const AlignmentGuidance& guidance, double dt) {
        if (guidance.is_aligned) {
            stable_time_ += dt;
        } else {
            stable_time_ = 0.0;
        }

        return stable_time_ >= stable_time_threshold_s_;
    }

    /**
     * @brief Reset stable time counter
     */
    void reset_stability() {
        stable_time_ = 0.0;
    }

    /**
     * @brief Set alliance color
     */
    void set_alliance(bool is_red) {
        is_red_alliance_ = is_red;
    }

private:
    /**
     * @brief Count visible tags from a position
     */
    int count_visible_tags(const Pose2D& pose, const std::vector<TagDetection>& tags) const {
        // Estimate based on distance and field coverage
        // TODO: More sophisticated visibility check
        int count = 0;
        for (const auto& tag : tags) {
            if (tag.pose_valid && tag.distance_m < 5.0) {
                count++;
            }
        }
        return count;
    }

    /**
     * @brief Calculate expected shot accuracy
     */
    double calculate_shot_accuracy(double distance, int visible_tags) const {
        // Accuracy decreases with distance, increases with more tags
        double distance_factor = 1.0 - std::clamp((distance - 1.5) / 3.5, 0.0, 1.0);
        double tag_factor = std::min(1.0, visible_tags / 3.0);

        return distance_factor * 0.7 + tag_factor * 0.3;
    }

    /**
     * @brief Generate waypoints along path
     */
    std::vector<AlignmentWaypoint> generate_waypoints(
        const Pose2D& start, const Pose2D& end, int num_points) const
    {
        std::vector<AlignmentWaypoint> waypoints;

        for (int i = 1; i <= num_points; i++) {
            double t = static_cast<double>(i) / num_points;

            AlignmentWaypoint wp;
            wp.pose.x = start.x + t * (end.x - start.x);
            wp.pose.y = start.y + t * (end.y - start.y);
            wp.pose.theta = start.theta + t * normalize_angle(end.theta - start.theta);

            wp.speed_mps = t < 0.7 ? 3.0 : 1.0;  // Slow down near end
            wp.curvature = 0.0;
            wp.requires_precision = (t > 0.7);

            waypoints.push_back(wp);
        }

        return waypoints;
    }

    /**
     * @brief Normalize angle to [-π, π]
     */
    double normalize_angle(double angle) const {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    bool is_red_alliance_;
    double alignment_tolerance_m_;
    double alignment_tolerance_deg_;
    double stable_time_threshold_s_;
    double stable_time_ = 0.0;
};

} // namespace phase3
} // namespace frc_vision
