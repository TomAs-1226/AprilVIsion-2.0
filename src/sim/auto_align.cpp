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

    // Configure PID controllers
    pos_x_pid_.set_gains(params.pos_kp, params.pos_ki, params.pos_kd);
    pos_x_pid_.set_limits(-params.max_speed, params.max_speed);
    pos_x_pid_.set_integral_limits(-1.0, 1.0);

    pos_y_pid_.set_gains(params.pos_kp, params.pos_ki, params.pos_kd);
    pos_y_pid_.set_limits(-params.max_speed, params.max_speed);
    pos_y_pid_.set_integral_limits(-1.0, 1.0);

    heading_pid_.set_gains(params.heading_kp, params.heading_ki, params.heading_kd);
    heading_pid_.set_limits(-params.max_rotation, params.max_rotation);
    heading_pid_.set_integral_limits(-1.0, 1.0);
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

    // Compute target position and heading
    double target_x, target_y, target_heading;
    compute_target(current_pose, target_tag_id_, target_x, target_y, target_heading);

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

    // Compute control outputs (in robot frame)
    vx_out = pos_x_pid_.calculate(0, -error_forward, dt);
    vy_out = pos_y_pid_.calculate(0, -error_left, dt);
    omega_out = heading_pid_.calculate(0, -error_heading, dt);

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
