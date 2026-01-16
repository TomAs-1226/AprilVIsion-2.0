/**
 * @file auto_align.cpp
 * @brief Auto-align controller implementation
 */

#include "auto_align.hpp"
#include <cmath>
#include <limits>

namespace frc_vision {
namespace sim {

// ============================================================================
// PIDController Implementation
// ============================================================================

PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd) {}

void PIDController::set_gains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::set_limits(double min_output, double max_output) {
    min_output_ = min_output;
    max_output_ = max_output;
}

void PIDController::set_integral_limits(double min_integral, double max_integral) {
    min_integral_ = min_integral;
    max_integral_ = max_integral;
}

double PIDController::calculate(double setpoint, double measurement, double dt) {
    double error = setpoint - measurement;

    // Proportional
    double p_term = kp_ * error;

    // Integral with anti-windup
    integral_ += error * dt;
    integral_ = std::clamp(integral_, min_integral_, max_integral_);
    double i_term = ki_ * integral_;

    // Derivative
    double d_term = 0;
    if (!first_run_ && dt > 0) {
        double derivative = (error - prev_error_) / dt;
        d_term = kd_ * derivative;
    }

    prev_error_ = error;
    first_run_ = false;

    // Sum and limit output
    double output = p_term + i_term + d_term;
    return std::clamp(output, min_output_, max_output_);
}

void PIDController::reset() {
    integral_ = 0;
    prev_error_ = 0;
    first_run_ = true;
}

// ============================================================================
// AutoAlignController Implementation
// ============================================================================

AutoAlignController::AutoAlignController() = default;

void AutoAlignController::initialize(const AutoAlignParams& params, const FieldLayout& field) {
    params_ = params;
    field_ = field;

    // Configure PID controllers with aggressive gains for fast response
    // Increased gains: 2x position, 1.5x heading for snappy response
    double pos_kp_fast = params.pos_kp * 2.5;
    double pos_kd_fast = params.pos_kd * 2.0;
    double heading_kp_fast = params.heading_kp * 2.0;
    double heading_kd_fast = params.heading_kd * 1.5;

    pos_x_pid_.set_gains(pos_kp_fast, params.pos_ki, pos_kd_fast);
    pos_x_pid_.set_limits(-params.max_speed * 1.2, params.max_speed * 1.2);
    pos_x_pid_.set_integral_limits(-0.5, 0.5);

    pos_y_pid_.set_gains(pos_kp_fast, params.pos_ki, pos_kd_fast);
    pos_y_pid_.set_limits(-params.max_speed * 1.2, params.max_speed * 1.2);
    pos_y_pid_.set_integral_limits(-0.5, 0.5);

    heading_pid_.set_gains(heading_kp_fast, params.heading_ki, heading_kd_fast);
    heading_pid_.set_limits(-params.max_rotation * 1.5, params.max_rotation * 1.5);
    heading_pid_.set_integral_limits(-0.5, 0.5);
}

void AutoAlignController::set_enabled(bool enabled) {
    if (enabled && !enabled_) {
        // Just enabled - reset controllers
        reset();
    }
    enabled_ = enabled;
}

void AutoAlignController::reset() {
    pos_x_pid_.reset();
    pos_y_pid_.reset();
    heading_pid_.reset();
    at_target_ = false;
    target_tag_id_ = -1;
}

bool AutoAlignController::update(const Pose2D& current_pose,
                                 const std::vector<int>& visible_tag_ids,
                                 double dt,
                                 double& vx_out, double& vy_out, double& omega_out) {
    vx_out = vy_out = omega_out = 0;

    if (!enabled_) {
        return false;
    }

    // Find target tag
    if (target_tag_id_ < 0 || !field_.has_tag(target_tag_id_)) {
        target_tag_id_ = find_nearest_tag(current_pose, visible_tag_ids);
    }

    if (target_tag_id_ < 0 || !field_.has_tag(target_tag_id_)) {
        // No valid target
        at_target_ = false;
        return false;
    }

    // Get tag position
    const auto& tag = field_.get_tag(target_tag_id_);
    double tag_x = tag.center_field.x;
    double tag_y = tag.center_field.y;

    // Compute direction from robot to tag
    double dx_to_tag = tag_x - current_pose.x;
    double dy_to_tag = tag_y - current_pose.y;
    double dist_to_tag = std::sqrt(dx_to_tag * dx_to_tag + dy_to_tag * dy_to_tag);

    // Target heading: face the tag
    double target_heading = std::atan2(dy_to_tag, dx_to_tag);

    // Target position: standoff distance from tag, towards the robot's current direction
    double target_x, target_y;
    if (dist_to_tag > 0.01) {
        // Move towards a position that is standoff_distance from tag, along the line robot->tag
        double unit_x = dx_to_tag / dist_to_tag;
        double unit_y = dy_to_tag / dist_to_tag;
        target_x = tag_x - unit_x * params_.standoff_distance;
        target_y = tag_y - unit_y * params_.standoff_distance;
    } else {
        target_x = current_pose.x;
        target_y = current_pose.y;
    }

    // Position error in field frame
    double error_x = target_x - current_pose.x;
    double error_y = target_y - current_pose.y;

    // Transform error to robot frame for control
    double cos_theta = std::cos(current_pose.theta);
    double sin_theta = std::sin(current_pose.theta);
    double error_forward = error_x * cos_theta + error_y * sin_theta;
    double error_left = -error_x * sin_theta + error_y * cos_theta;

    // Heading error (normalized to [-pi, pi])
    double error_heading = normalize_angle(target_heading - current_pose.theta);

    // Check if at target
    double pos_error = std::sqrt(error_x * error_x + error_y * error_y);
    at_target_ = (pos_error < params_.pos_tolerance &&
                  std::abs(error_heading) < params_.heading_tolerance);

    if (at_target_) {
        // At target - stop
        return false;
    }

    // Improved approach: Simultaneous translation and rotation with priority weighting
    // Uses feedforward terms for faster response

    // Calculate PID outputs
    double pid_vx = pos_x_pid_.calculate(0, -error_forward, dt);
    double pid_vy = pos_y_pid_.calculate(0, -error_left, dt);
    double pid_omega = heading_pid_.calculate(0, -error_heading, dt);

    // Add feedforward: proportional to error for faster initial response
    double ff_gain = 1.5;  // Feedforward multiplier
    double ff_vx = ff_gain * error_forward;
    double ff_vy = ff_gain * error_left;
    double ff_omega = ff_gain * 2.0 * error_heading;  // More aggressive rotation FF

    // Clamp feedforward
    double max_ff_vel = params_.max_speed * 0.8;
    double max_ff_omega = params_.max_rotation * 0.8;
    ff_vx = std::clamp(ff_vx, -max_ff_vel, max_ff_vel);
    ff_vy = std::clamp(ff_vy, -max_ff_vel, max_ff_vel);
    ff_omega = std::clamp(ff_omega, -max_ff_omega, max_ff_omega);

    // Blended approach based on heading error
    // When far off, prioritize rotation; when close, allow full translation
    double heading_priority = std::min(1.0, std::abs(error_heading) / 0.8);  // 0-1 based on heading error
    double translation_scale = 1.0 - heading_priority * 0.7;  // Reduce translation when far off heading

    // Combine PID + feedforward with priority scaling
    vx_out = (pid_vx + ff_vx) * translation_scale;
    vy_out = (pid_vy + ff_vy) * translation_scale;
    omega_out = pid_omega + ff_omega;

    // Apply final limits
    double max_speed = params_.max_speed * 1.2;
    double max_rot = params_.max_rotation * 1.5;
    vx_out = std::clamp(vx_out, -max_speed, max_speed);
    vy_out = std::clamp(vy_out, -max_speed, max_speed);
    omega_out = std::clamp(omega_out, -max_rot, max_rot);

    return true;
}

int AutoAlignController::find_nearest_tag(const Pose2D& pose, const std::vector<int>& visible_ids) const {
    int nearest_id = -1;
    double min_dist = std::numeric_limits<double>::max();

    for (int id : visible_ids) {
        if (!field_.has_tag(id)) continue;

        const auto& tag = field_.get_tag(id);
        double dx = tag.center_field.x - pose.x;
        double dy = tag.center_field.y - pose.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < min_dist) {
            min_dist = dist;
            nearest_id = id;
        }
    }

    // If no visible tags, find nearest from all field tags
    if (nearest_id < 0) {
        for (const auto& [id, tag] : field_.tags) {
            double dx = tag.center_field.x - pose.x;
            double dy = tag.center_field.y - pose.y;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < min_dist) {
                min_dist = dist;
                nearest_id = id;
            }
        }
    }

    return nearest_id;
}

void AutoAlignController::compute_target(const Pose2D& pose, int tag_id,
                                         double& target_x, double& target_y,
                                         double& target_heading) const {
    const auto& tag = field_.get_tag(tag_id);

    // Get tag facing direction (opposite of tag normal)
    cv::Mat R_tag;
    cv::Rodrigues(tag.pose_field.rvec(), R_tag);

    // Tag faces in +Z direction of tag frame
    // We want to stand at standoff distance in that direction
    double tag_facing_x = R_tag.at<double>(0, 2);
    double tag_facing_y = R_tag.at<double>(1, 2);

    // Normalize
    double len = std::sqrt(tag_facing_x * tag_facing_x + tag_facing_y * tag_facing_y);
    if (len > 0.01) {
        tag_facing_x /= len;
        tag_facing_y /= len;
    } else {
        // Tag facing up/down - estimate from pose to tag
        double dx = tag.center_field.x - pose.x;
        double dy = tag.center_field.y - pose.y;
        len = std::sqrt(dx * dx + dy * dy);
        if (len > 0.01) {
            tag_facing_x = dx / len;
            tag_facing_y = dy / len;
        } else {
            tag_facing_x = 1.0;
            tag_facing_y = 0.0;
        }
    }

    // Target position: standoff distance from tag in facing direction
    target_x = tag.center_field.x + tag_facing_x * params_.standoff_distance;
    target_y = tag.center_field.y + tag_facing_y * params_.standoff_distance;

    // Target heading: face the tag
    target_heading = std::atan2(tag.center_field.y - target_y,
                                tag.center_field.x - target_x);
}

double AutoAlignController::normalize_angle(double angle) const {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

} // namespace sim
} // namespace frc_vision
