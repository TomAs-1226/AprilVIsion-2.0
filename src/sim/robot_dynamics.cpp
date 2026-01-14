/**
 * @file robot_dynamics.cpp
 * @brief Robot dynamics implementation
 */

#include "robot_dynamics.hpp"
#include <cmath>
#include <chrono>

namespace frc_vision {
namespace sim {

RobotDynamics::RobotDynamics() {
    // Seed RNG with current time
    auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    rng_.seed(static_cast<unsigned int>(seed));
}

void RobotDynamics::initialize(const DynamicsParams& params, const Pose2D& start_pose) {
    params_ = params;
    reset(start_pose);
}

void RobotDynamics::reset(const Pose2D& pose) {
    state_.true_pose = pose;
    state_.odom_pose = pose;
    state_.fused_pose = pose;
    state_.vx = 0;
    state_.vy = 0;
    state_.omega = 0;
    use_external_cmd_ = false;
}

void RobotDynamics::update(const InputState& input, double dt) {
    // Determine target velocities from input
    double target_vx = 0, target_vy = 0, target_omega = 0;

    if (use_external_cmd_) {
        // Use external command (auto-align)
        target_vx = cmd_vx_;
        target_vy = cmd_vy_;
        target_omega = cmd_omega_;
    } else {
        // Use keyboard input
        double max_speed = input.turbo ? params_.max_turbo_speed : params_.max_speed;

        if (input.forward) target_vx += max_speed;
        if (input.backward) target_vx -= max_speed;
        if (input.left) target_vy += max_speed;
        if (input.right) target_vy -= max_speed;
        if (input.rotate_ccw) target_omega += params_.max_rotation;
        if (input.rotate_cw) target_omega -= params_.max_rotation;
    }

    // Apply acceleration limits
    auto accel_towards = [this, dt](double current, double target, double accel, double decel) {
        double error = target - current;
        double max_change = (std::abs(target) > std::abs(current) ? accel : decel) * dt;

        if (std::abs(error) <= max_change) {
            return target;
        }
        return current + (error > 0 ? max_change : -max_change);
    };

    state_.vx = accel_towards(state_.vx, target_vx, params_.accel, params_.decel);
    state_.vy = accel_towards(state_.vy, target_vy, params_.accel, params_.decel);
    state_.omega = accel_towards(state_.omega, target_omega, params_.rot_accel, params_.rot_accel);

    // Compute motion in robot frame
    double dx_robot = state_.vx * dt;
    double dy_robot = state_.vy * dt;
    double dtheta = state_.omega * dt;

    // Transform to field frame
    double cos_theta = std::cos(state_.true_pose.theta);
    double sin_theta = std::sin(state_.true_pose.theta);
    double dx_field = dx_robot * cos_theta - dy_robot * sin_theta;
    double dy_field = dx_robot * sin_theta + dy_robot * cos_theta;

    // Update true pose (ground truth - no noise)
    state_.true_pose.x += dx_field;
    state_.true_pose.y += dy_field;
    state_.true_pose.theta = normalize_angle(state_.true_pose.theta + dtheta);

    // Update odometry with noise
    add_odometry_noise(dx_field, dy_field, dtheta);

    // Update fused pose (for now, just copy true pose - will be corrected by vision)
    // In real use, this gets updated by the vision fusion
}

void RobotDynamics::add_odometry_noise(double dx, double dy, double dtheta) {
    double dist = std::sqrt(dx * dx + dy * dy);

    // Translation noise proportional to distance traveled
    double trans_noise = params_.odom_trans_noise * dist;
    double dx_noise = dx + trans_noise * noise_dist_(rng_);
    double dy_noise = dy + trans_noise * noise_dist_(rng_);

    // Rotation noise proportional to rotation + some from translation
    double rot_noise = params_.odom_rot_noise * (std::abs(dtheta) + dist * 0.1);
    double dtheta_noise = dtheta + rot_noise * noise_dist_(rng_);

    // Add constant drift
    dx_noise += params_.odom_drift_rate * noise_dist_(rng_) * 0.016; // ~60fps
    dy_noise += params_.odom_drift_rate * noise_dist_(rng_) * 0.016;

    // Update odometry pose
    state_.odom_pose.x += dx_noise;
    state_.odom_pose.y += dy_noise;
    state_.odom_pose.theta = normalize_angle(state_.odom_pose.theta + dtheta_noise);
}

void RobotDynamics::apply_velocity_command(double vx, double vy, double omega) {
    cmd_vx_ = vx;
    cmd_vy_ = vy;
    cmd_omega_ = omega;
}

void RobotDynamics::set_external_command_mode(bool enabled) {
    use_external_cmd_ = enabled;
    if (!enabled) {
        cmd_vx_ = cmd_vy_ = cmd_omega_ = 0;
    }
}

void RobotDynamics::get_velocities(double& vx, double& vy, double& omega) const {
    vx = state_.vx;
    vy = state_.vy;
    omega = state_.omega;
}

double RobotDynamics::clamp(double value, double min_val, double max_val) const {
    return std::max(min_val, std::min(max_val, value));
}

double RobotDynamics::normalize_angle(double angle) const {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

} // namespace sim
} // namespace frc_vision
