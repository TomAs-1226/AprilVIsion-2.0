/**
 * @file phase3_curved_paths.hpp
 * @brief Phase 3 Extension: Curved path following with dynamic heading control
 *
 * For FRC 2025/2026 scenarios like:
 * - Scoring in hub while moving on semicircle path
 * - Dynamic heading adjustment to face goal while translating
 * - Smooth tag handoffs as robot moves around field
 */

#pragma once

#include "types.hpp"
#include "phase3_autoalign.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace frc_vision {
namespace phase3 {

/**
 * @brief Curved path segment (arc or circle)
 */
struct CurvedPathSegment {
    Pose2D start_pose;          // Starting pose
    Pose2D end_pose;            // Ending pose
    double radius;              // Arc radius (meters)
    double center_x, center_y;  // Arc center point
    double start_angle;         // Starting angle on arc
    double end_angle;           // Ending angle on arc
    bool clockwise;             // Direction of travel

    double length;              // Total arc length
    double max_velocity;        // Max velocity on this segment
};

/**
 * @brief Dynamic heading target (for aiming while moving)
 */
struct DynamicHeadingTarget {
    double target_x, target_y;  // Point to aim at (e.g., hub center)
    double offset_angle = 0.0;  // Additional angle offset (radians)
    bool face_target = true;    // True = face target, false = tangent to path
};

/**
 * @brief Tag handoff manager - smoothly transitions between tags
 */
class TagHandoffManager {
public:
    TagHandoffManager()
        : hysteresis_distance_(0.5)  // Switch tags if new one is 0.5m closer
        , min_switch_interval_s_(0.5)  // Don't switch more than every 0.5s
    {}

    /**
     * @brief Select best tag(s) for current position
     *
     * Uses hysteresis to prevent rapid switching:
     * - Current primary tag gets "bonus" to stay selected
     * - Only switches if new tag is significantly better
     *
     * @param current_pose Current robot pose
     * @param detections All detected tags with accuracy estimates
     * @return Tag IDs in priority order (best first)
     */
    std::vector<int> select_tags(
        const Pose2D& current_pose,
        const std::vector<TagDetection>& detections,
        double dt)
    {
        time_since_last_switch_ += dt;

        if (detections.empty()) {
            return {};
        }

        // Score each tag based on:
        // - Distance (closer = better)
        // - Accuracy estimate (more confident = better)
        // - Viewing angle (more frontal = better)
        // - Hysteresis bonus if currently primary

        struct TagScore {
            int tag_id;
            double distance;
            double accuracy_score;
            double total_score;
        };

        std::vector<TagScore> scores;

        for (const auto& det : detections) {
            if (!det.pose_valid) continue;

            TagScore score;
            score.tag_id = det.id;
            score.distance = det.distance_m;

            // Accuracy score from Phase 2 per-tag accuracy
            score.accuracy_score = 1.0;
            if (det.accuracy_estimate.confidence_level == "high") {
                score.accuracy_score = 1.0;
            } else if (det.accuracy_estimate.confidence_level == "medium") {
                score.accuracy_score = 0.7;
            } else {
                score.accuracy_score = 0.4;
            }

            // Distance score (inverse quadratic)
            double distance_score = 1.0 / (1.0 + score.distance * score.distance * 0.1);

            // Combine scores
            score.total_score = distance_score * 0.6 + score.accuracy_score * 0.4;

            // HYSTERESIS: Bonus for current primary tag
            if (det.id == current_primary_tag_ && time_since_last_switch_ < 2.0) {
                score.total_score += 0.2;  // 20% bonus to stay with current tag
            }

            scores.push_back(score);
        }

        // Sort by score (best first)
        std::sort(scores.begin(), scores.end(),
                 [](const TagScore& a, const TagScore& b) {
                     return a.total_score > b.total_score;
                 });

        // Check if we should switch primary tag
        if (!scores.empty()) {
            int best_tag = scores[0].tag_id;

            if (best_tag != current_primary_tag_ &&
                time_since_last_switch_ >= min_switch_interval_s_) {

                // Only switch if significantly better
                if (scores.size() == 1 || scores[0].total_score > scores[1].total_score * 1.1) {
                    std::cout << "[TagHandoff] Switching from tag " << current_primary_tag_
                             << " to tag " << best_tag << std::endl;
                    current_primary_tag_ = best_tag;
                    time_since_last_switch_ = 0.0;
                }
            }
        }

        // Return sorted tag IDs
        std::vector<int> result;
        for (const auto& score : scores) {
            result.push_back(score.tag_id);
        }

        return result;
    }

    /**
     * @brief Get current primary tracking tag
     */
    int get_primary_tag() const {
        return current_primary_tag_;
    }

    /**
     * @brief Force switch to specific tag
     */
    void set_primary_tag(int tag_id) {
        current_primary_tag_ = tag_id;
        time_since_last_switch_ = 0.0;
    }

    /**
     * @brief Reset handoff manager
     */
    void reset() {
        current_primary_tag_ = -1;
        time_since_last_switch_ = 0.0;
    }

private:
    int current_primary_tag_ = -1;
    double time_since_last_switch_ = 0.0;
    double hysteresis_distance_;
    double min_switch_interval_s_;
};

/**
 * @brief Curved path follower with dynamic heading control
 *
 * Allows robot to follow curved paths (arcs, circles) while
 * independently controlling heading (e.g., always facing hub)
 */
class CurvedPathFollower {
public:
    CurvedPathFollower()
        : lookahead_distance_(0.3)  // Look 30cm ahead on path
        , max_lateral_error_(0.1)    // Max 10cm off path
    {}

    /**
     * @brief Create semicircle path from start to end
     *
     * @param start Starting pose
     * @param end Ending pose
     * @param radius Semicircle radius (meters)
     * @param clockwise True for clockwise motion
     * @return Curved path segment
     */
    static CurvedPathSegment create_semicircle(
        const Pose2D& start,
        const Pose2D& end,
        double radius,
        bool clockwise = true)
    {
        CurvedPathSegment segment;
        segment.start_pose = start;
        segment.end_pose = end;
        segment.radius = radius;
        segment.clockwise = clockwise;

        // Calculate arc center
        // Center is perpendicular to line connecting start and end
        double dx = end.x - start.x;
        double dy = end.y - start.y;
        double chord_length = std::sqrt(dx * dx + dy * dy);

        // For semicircle: center is at midpoint of chord
        segment.center_x = (start.x + end.x) / 2.0;
        segment.center_y = (start.y + end.y) / 2.0;

        // Calculate start and end angles
        segment.start_angle = std::atan2(start.y - segment.center_y,
                                        start.x - segment.center_x);
        segment.end_angle = std::atan2(end.y - segment.center_y,
                                      end.x - segment.center_x);

        // Arc length = radius * angle
        double angle_diff = segment.end_angle - segment.start_angle;
        if (clockwise && angle_diff > 0) angle_diff -= 2 * M_PI;
        if (!clockwise && angle_diff < 0) angle_diff += 2 * M_PI;
        segment.length = radius * std::abs(angle_diff);

        // Velocity limit based on radius (centripetal acceleration)
        // a_c = v^2 / r, limit to 3 m/s^2
        segment.max_velocity = std::sqrt(3.0 * radius);
        segment.max_velocity = std::clamp(segment.max_velocity, 0.5, 3.0);

        return segment;
    }

    /**
     * @brief Calculate velocity commands to follow curved path with dynamic heading
     *
     * @param current_pose Current robot pose
     * @param segment Path segment to follow
     * @param heading_target Optional dynamic heading target
     * @return Velocity commands (vx, vy, omega)
     */
    AlignmentGuidance follow_path(
        const Pose2D& current_pose,
        const CurvedPathSegment& segment,
        const std::optional<DynamicHeadingTarget>& heading_target = std::nullopt)
    {
        AlignmentGuidance guidance;
        guidance.current_pose = current_pose;

        // 1. Find closest point on arc
        double dx = current_pose.x - segment.center_x;
        double dy = current_pose.y - segment.center_y;
        double current_angle = std::atan2(dy, dx);

        // 2. Calculate lookahead point on arc
        double lookahead_angle_offset = lookahead_distance_ / segment.radius;
        if (!segment.clockwise) lookahead_angle_offset = -lookahead_angle_offset;
        double lookahead_angle = current_angle + lookahead_angle_offset;

        double lookahead_x = segment.center_x + segment.radius * std::cos(lookahead_angle);
        double lookahead_y = segment.center_y + segment.radius * std::sin(lookahead_angle);

        // 3. Calculate translation velocity (toward lookahead point)
        double to_lookahead_x = lookahead_x - current_pose.x;
        double to_lookahead_y = lookahead_y - current_pose.y;

        // Transform to robot frame
        double cos_theta = std::cos(-current_pose.theta);
        double sin_theta = std::sin(-current_pose.theta);

        double vx_robot = to_lookahead_x * cos_theta - to_lookahead_y * sin_theta;
        double vy_robot = to_lookahead_x * sin_theta + to_lookahead_y * cos_theta;

        // Scale by velocity limit
        double vel_magnitude = std::sqrt(vx_robot * vx_robot + vy_robot * vy_robot);
        if (vel_magnitude > segment.max_velocity) {
            vx_robot = vx_robot / vel_magnitude * segment.max_velocity;
            vy_robot = vy_robot / vel_magnitude * segment.max_velocity;
        }

        guidance.drive_velocity_mps = vx_robot;
        guidance.strafe_velocity_mps = vy_robot;

        // 4. Calculate heading control (independent of translation)
        double target_heading;

        if (heading_target.has_value() && heading_target->face_target) {
            // Face the target point (e.g., hub center)
            double to_target_x = heading_target->target_x - current_pose.x;
            double to_target_y = heading_target->target_y - current_pose.y;
            target_heading = std::atan2(to_target_y, to_target_x) + heading_target->offset_angle;
        } else {
            // Face tangent to path
            target_heading = lookahead_angle + (segment.clockwise ? -M_PI/2 : M_PI/2);
        }

        double heading_error = normalize_angle(target_heading - current_pose.theta);
        guidance.angular_velocity_rps = std::clamp(heading_error * 3.0, -2.0, 2.0);

        // 5. Check alignment status
        double lateral_error = std::abs(std::sqrt(dx*dx + dy*dy) - segment.radius);
        guidance.is_aligned = (lateral_error < max_lateral_error_);

        // Distance along path
        double angle_progress = normalize_angle(current_angle - segment.start_angle);
        if (segment.clockwise && angle_progress > 0) angle_progress -= 2*M_PI;
        if (!segment.clockwise && angle_progress < 0) angle_progress += 2*M_PI;
        double distance_on_path = segment.radius * std::abs(angle_progress);

        guidance.distance_to_target_m = segment.length - distance_on_path;
        guidance.heading_error_deg = heading_error * 180.0 / M_PI;

        // Stage determination
        if (guidance.distance_to_target_m > 1.0) {
            guidance.current_stage = "following";
            guidance.alignment_quality = 0.5;
        } else if (guidance.distance_to_target_m > 0.2) {
            guidance.current_stage = "approaching_end";
            guidance.alignment_quality = 0.7;
        } else {
            guidance.current_stage = "path_complete";
            guidance.alignment_quality = 0.9;
        }

        return guidance;
    }

private:
    double normalize_angle(double angle) const {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    double lookahead_distance_;
    double max_lateral_error_;
};

/**
 * @brief Complete system for semicircle + dynamic heading + tag handoff
 *
 * Example use case:
 * - Robot travels on semicircle path around hub
 * - Continuously aims at hub center while moving
 * - Smoothly switches between visible AprilTags
 */
class SemicircleShooter {
public:
    SemicircleShooter(double hub_x, double hub_y)
        : hub_center_x_(hub_x)
        , hub_center_y_(hub_y)
    {}

    /**
     * @brief Plan semicircle path around hub
     */
    CurvedPathSegment plan_semicircle_approach(
        const Pose2D& current_pose,
        double desired_shooting_radius = 2.5)
    {
        // Calculate start and end points on semicircle around hub
        double current_to_hub_angle = std::atan2(
            hub_center_y_ - current_pose.y,
            hub_center_x_ - current_pose.x);

        // Start point: current position projected to circle
        Pose2D start;
        start.x = hub_center_x_ + desired_shooting_radius * std::cos(current_to_hub_angle);
        start.y = hub_center_y_ + desired_shooting_radius * std::sin(current_to_hub_angle);
        start.theta = current_to_hub_angle;

        // End point: 180° around hub
        Pose2D end;
        end.x = hub_center_x_ + desired_shooting_radius * std::cos(current_to_hub_angle + M_PI);
        end.y = hub_center_y_ + desired_shooting_radius * std::sin(current_to_hub_angle + M_PI);
        end.theta = current_to_hub_angle + M_PI;

        return CurvedPathFollower::create_semicircle(start, end, desired_shooting_radius, true);
    }

    /**
     * @brief Execute semicircle with dynamic heading and tag handoff
     */
    AlignmentGuidance execute(
        const Pose2D& current_pose,
        const CurvedPathSegment& path,
        const std::vector<TagDetection>& detections,
        double dt)
    {
        // Select best tags for current position (with handoff)
        auto priority_tags = handoff_manager_.select_tags(current_pose, detections, dt);

        // Set up dynamic heading to face hub
        DynamicHeadingTarget heading_target;
        heading_target.target_x = hub_center_x_;
        heading_target.target_y = hub_center_y_;
        heading_target.face_target = true;

        // Follow curved path
        auto guidance = path_follower_.follow_path(current_pose, path, heading_target);

        // Add tag info to guidance
        if (!priority_tags.empty()) {
            std::cout << "[SemicircleShooter] Using tags: ";
            for (int tag : priority_tags) {
                std::cout << tag << " ";
            }
            std::cout << std::endl;
        }

        return guidance;
    }

private:
    double hub_center_x_;
    double hub_center_y_;
    CurvedPathFollower path_follower_;
    TagHandoffManager handoff_manager_;
};

} // namespace phase3
} // namespace frc_vision
