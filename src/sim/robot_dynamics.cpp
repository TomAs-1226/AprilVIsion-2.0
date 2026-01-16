/**
 * @file robot_dynamics.cpp
 * @brief Robot dynamics implementation with realistic swerve kinematics
 */

#include "robot_dynamics.hpp"
#include <cmath>
#include <chrono>
#include <algorithm>

namespace frc_vision {
namespace sim {

RobotDynamics::RobotDynamics() {
    // Seed RNG with current time
    auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    rng_.seed(static_cast<unsigned int>(seed));
}

void RobotDynamics::initialize(const DynamicsParams& params, const Pose2D& start_pose) {
    params_ = params;

    // Compute module positions from track width and wheel base
    double half_track = params_.swerve.track_width / 2.0;
    double half_base = params_.swerve.wheel_base / 2.0;

    // FL, FR, BL, BR
    module_positions_[0] = {half_base, half_track};   // Front-left
    module_positions_[1] = {half_base, -half_track};  // Front-right
    module_positions_[2] = {-half_base, half_track};  // Back-left
    module_positions_[3] = {-half_base, -half_track}; // Back-right

    reset(start_pose);
}

void RobotDynamics::reset(const Pose2D& pose) {
    state_.true_pose = pose;
    state_.odom_pose = pose;
    state_.fused_pose = pose;
    state_.vx = 0;
    state_.vy = 0;
    state_.omega = 0;
    state_.ax = 0;
    state_.ay = 0;
    state_.alpha = 0;
    state_.battery_voltage = params_.swerve.battery_voltage;
    state_.total_current_draw = 0;
    state_.is_slipping = false;
    state_.slip_ratio = 0;
    use_external_cmd_ = false;

    // Reset all modules
    for (auto& module : state_.modules) {
        module.wheel_speed = 0;
        module.steer_angle = 0;
        module.target_speed = 0;
        module.target_angle = 0;
        module.drive_current = 0;
        module.steer_current = 0;
    }
}

void RobotDynamics::update(const InputState& input, double dt) {
    if (params_.use_accurate_swerve) {
        update_swerve(input, dt);
    } else {
        update_simple(input, dt);
    }
}

void RobotDynamics::update_simple(const InputState& input, double dt) {
    // Original simple dynamics (fallback)
    double target_vx = 0, target_vy = 0, target_omega = 0;

    if (use_external_cmd_) {
        target_vx = cmd_vx_;
        target_vy = cmd_vy_;
        target_omega = cmd_omega_;
    } else {
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

    // Update true pose
    state_.true_pose.x += dx_field;
    state_.true_pose.y += dy_field;
    state_.true_pose.theta = normalize_angle(state_.true_pose.theta + dtheta);

    // Update odometry with noise
    add_odometry_noise(dx_field, dy_field, dtheta);
}

void RobotDynamics::update_swerve(const InputState& input, double dt) {
    // Get target velocities
    double target_vx = 0, target_vy = 0, target_omega = 0;

    if (use_external_cmd_) {
        target_vx = cmd_vx_;
        target_vy = cmd_vy_;
        target_omega = cmd_omega_;
    } else {
        double max_speed = input.turbo ? params_.max_turbo_speed : params_.max_speed;

        if (input.forward) target_vx += max_speed;
        if (input.backward) target_vx -= max_speed;
        if (input.left) target_vy += max_speed;
        if (input.right) target_vy -= max_speed;
        if (input.rotate_ccw) target_omega += params_.max_rotation;
        if (input.rotate_cw) target_omega -= params_.max_rotation;
    }

    // Compute module targets from chassis velocity
    std::array<double, 4> target_speeds;
    std::array<double, 4> target_angles;
    compute_module_targets(target_vx, target_vy, target_omega, target_speeds, target_angles);

    // Set module targets
    for (int i = 0; i < 4; i++) {
        state_.modules[i].target_speed = target_speeds[i];
        state_.modules[i].target_angle = target_angles[i];
    }

    // Update battery simulation
    update_battery(dt);

    // Update module states with motor dynamics
    update_module_states(dt);

    // Apply traction limits (wheel slip)
    apply_traction_limits(dt);

    // Compute chassis motion from actual module states
    double dx_robot, dy_robot, dtheta;
    compute_chassis_motion(dt, dx_robot, dy_robot, dtheta);

    // Store accelerations for reference
    state_.ax = (state_.vx - (dx_robot / dt)) / dt;
    state_.ay = (state_.vy - (dy_robot / dt)) / dt;
    state_.alpha = (state_.omega - (dtheta / dt)) / dt;

    // Update velocities from actual motion
    if (dt > 0.0001) {
        state_.vx = dx_robot / dt;
        state_.vy = dy_robot / dt;
        state_.omega = dtheta / dt;
    }

    // Transform to field frame
    double cos_theta = std::cos(state_.true_pose.theta);
    double sin_theta = std::sin(state_.true_pose.theta);
    double dx_field = dx_robot * cos_theta - dy_robot * sin_theta;
    double dy_field = dx_robot * sin_theta + dy_robot * cos_theta;

    // Update true pose
    state_.true_pose.x += dx_field;
    state_.true_pose.y += dy_field;
    state_.true_pose.theta = normalize_angle(state_.true_pose.theta + dtheta);

    // Update odometry from module encoders
    update_odometry_from_modules(dt);
}

void RobotDynamics::compute_module_targets(double vx, double vy, double omega,
                                           std::array<double, 4>& speeds,
                                           std::array<double, 4>& angles) {
    // Compute individual module velocities using inverse kinematics
    double max_speed = 0;

    for (int i = 0; i < 4; i++) {
        double mx = module_positions_[i].first;
        double my = module_positions_[i].second;

        // Module velocity = chassis velocity + rotation contribution
        double module_vx = vx - omega * my;
        double module_vy = vy + omega * mx;

        double speed = std::sqrt(module_vx * module_vx + module_vy * module_vy);
        double angle = std::atan2(module_vy, module_vx);

        speeds[i] = speed;
        angles[i] = angle;
        max_speed = std::max(max_speed, speed);
    }

    // Desaturate if any module exceeds max speed
    if (max_speed > params_.max_turbo_speed) {
        double scale = params_.max_turbo_speed / max_speed;
        for (int i = 0; i < 4; i++) {
            speeds[i] *= scale;
        }
    }

    // Optimize module angles (take shorter path)
    for (int i = 0; i < 4; i++) {
        double current_angle = state_.modules[i].steer_angle;
        double target_angle = angles[i];

        // Normalize difference
        double diff = normalize_angle(target_angle - current_angle);

        // If angle is > 90°, flip direction and negate speed
        if (std::abs(diff) > M_PI / 2.0) {
            angles[i] = normalize_angle(target_angle + M_PI);
            speeds[i] = -speeds[i];
        }
    }
}

void RobotDynamics::update_module_states(double dt) {
    double total_current = 0;
    const auto& swerve = params_.swerve;

    for (int i = 0; i < 4; i++) {
        auto& module = state_.modules[i];

        // Simulate steering motor response (first-order lag)
        module.steer_angle = simulate_motor_response(
            module.steer_angle, module.target_angle,
            swerve.steer_response_tau, dt);

        // Normalize steering angle
        module.steer_angle = normalize_angle(module.steer_angle);

        // Simulate drive motor response
        // Account for battery voltage sag
        double voltage_ratio = state_.battery_voltage / swerve.battery_voltage;
        double effective_target = module.target_speed * voltage_ratio;

        module.wheel_speed = simulate_motor_response(
            module.wheel_speed, effective_target,
            swerve.drive_response_tau, dt);

        // Estimate motor current (simplified)
        double speed_error = std::abs(module.target_speed - module.wheel_speed);
        module.drive_current = speed_error * 20.0;  // Rough estimate: 20A per m/s error
        module.drive_current = std::min(module.drive_current, 40.0);  // Cap at 40A

        total_current += module.drive_current;
    }

    state_.total_current_draw = total_current;
}

void RobotDynamics::compute_chassis_motion(double dt, double& dx, double& dy, double& dtheta) {
    // Forward kinematics: average module contributions
    double sum_vx = 0, sum_vy = 0, sum_omega = 0;
    double weight_sum = 0;

    for (int i = 0; i < 4; i++) {
        const auto& module = state_.modules[i];
        double mx = module_positions_[i].first;
        double my = module_positions_[i].second;

        // Module velocity in robot frame
        double module_vx = module.wheel_speed * std::cos(module.steer_angle);
        double module_vy = module.wheel_speed * std::sin(module.steer_angle);

        // Extract chassis velocity (this is approximate for overdetermined system)
        sum_vx += module_vx;
        sum_vy += module_vy;

        // Calculate omega contribution from this module
        // omega = (module_vy - vy) / mx  OR  omega = (vx - module_vx) / my
        double r = std::sqrt(mx * mx + my * my);
        if (r > 0.01) {
            // Cross product gives omega * r
            double omega_contrib = (-module_vx * my + module_vy * mx) / (r * r);
            sum_omega += omega_contrib;
        }
        weight_sum += 1.0;
    }

    if (weight_sum > 0) {
        dx = (sum_vx / weight_sum) * dt;
        dy = (sum_vy / weight_sum) * dt;
        dtheta = (sum_omega / weight_sum) * dt;
    } else {
        dx = dy = dtheta = 0;
    }
}

double RobotDynamics::simulate_motor_response(double current, double target, double tau, double dt) {
    // First-order lag: dx/dt = (target - current) / tau
    double alpha = 1.0 - std::exp(-dt / tau);
    return current + alpha * (target - current);
}

double RobotDynamics::compute_motor_torque(double voltage, double speed, double kt, double kv, double resistance) {
    // Motor torque = Kt * current
    // Back-EMF = speed / Kv
    // Current = (voltage - back_emf) / resistance
    double back_emf = speed / kv;
    double current = (voltage - back_emf) / resistance;
    return kt * current;
}

double RobotDynamics::compute_wheel_force(double torque, double wheel_radius, double gear_ratio) {
    double wheel_torque = torque * gear_ratio;
    return wheel_torque / wheel_radius;
}

double RobotDynamics::compute_slip_ratio(double wheel_speed, double ground_speed) {
    if (std::abs(ground_speed) < 0.01 && std::abs(wheel_speed) < 0.01) {
        return 0;
    }
    double ref_speed = std::max(std::abs(wheel_speed), std::abs(ground_speed));
    return (wheel_speed - ground_speed) / ref_speed;
}

double RobotDynamics::compute_traction_limit(double normal_force, double cof) {
    return normal_force * cof;
}

void RobotDynamics::apply_traction_limits(double dt) {
    const auto& swerve = params_.swerve;

    // Weight per wheel (assuming even distribution)
    double weight_per_wheel = swerve.robot_mass * 9.81 / 4.0;
    double max_traction = compute_traction_limit(weight_per_wheel, swerve.wheel_cof);
    double max_accel_per_wheel = max_traction / (swerve.robot_mass / 4.0);

    double max_slip = 0;
    bool any_slipping = false;

    for (int i = 0; i < 4; i++) {
        auto& module = state_.modules[i];

        // Estimate ground speed at this wheel
        double mx = module_positions_[i].first;
        double my = module_positions_[i].second;

        // Current chassis velocity at module position
        double ground_vx = state_.vx - state_.omega * my;
        double ground_vy = state_.vy + state_.omega * mx;

        // Project onto wheel direction
        double wheel_dir_x = std::cos(module.steer_angle);
        double wheel_dir_y = std::sin(module.steer_angle);
        double ground_speed = ground_vx * wheel_dir_x + ground_vy * wheel_dir_y;

        // Compute slip
        double slip = compute_slip_ratio(module.wheel_speed, ground_speed);
        max_slip = std::max(max_slip, std::abs(slip));

        // If slip is too high, limit wheel speed
        if (std::abs(slip) > swerve.slip_ratio_threshold) {
            any_slipping = true;

            // Reduce wheel speed towards ground speed
            double reduction = (std::abs(slip) - swerve.slip_ratio_threshold) * 0.5;
            reduction = std::min(reduction, 1.0);

            module.wheel_speed = module.wheel_speed * (1.0 - reduction) + ground_speed * reduction;
        }
    }

    state_.is_slipping = any_slipping;
    state_.slip_ratio = max_slip;
}

void RobotDynamics::update_battery(double dt) {
    const auto& swerve = params_.swerve;

    // Voltage sag due to current draw
    double voltage_drop = state_.total_current_draw * swerve.battery_internal_resistance;

    state_.battery_voltage = swerve.battery_voltage - voltage_drop;
    state_.battery_voltage = std::max(state_.battery_voltage, swerve.brownout_voltage);

    // Slow recovery when current drops
    if (voltage_drop < 0.5) {
        state_.battery_voltage += 0.1 * dt;  // Slow recovery
        state_.battery_voltage = std::min(state_.battery_voltage, swerve.battery_voltage);
    }
}

void RobotDynamics::update_odometry_from_modules(double dt) {
    // Compute odometry from wheel encoder readings (with noise)
    double sum_dx = 0, sum_dy = 0, sum_dtheta = 0;

    for (int i = 0; i < 4; i++) {
        const auto& module = state_.modules[i];
        double mx = module_positions_[i].first;
        double my = module_positions_[i].second;

        // Distance traveled by this wheel (with encoder noise)
        double wheel_dist = module.wheel_speed * dt;
        wheel_dist *= (1.0 + params_.odom_trans_noise * noise_dist_(rng_));

        // Steer angle with noise
        double steer = module.steer_angle;
        steer += params_.odom_rot_noise * noise_dist_(rng_) * 0.01;

        // Module displacement
        double mod_dx = wheel_dist * std::cos(steer);
        double mod_dy = wheel_dist * std::sin(steer);

        sum_dx += mod_dx;
        sum_dy += mod_dy;

        // Omega contribution
        double r = std::sqrt(mx * mx + my * my);
        if (r > 0.01) {
            double omega_contrib = (-mod_dx * my + mod_dy * mx) / (r * r);
            sum_dtheta += omega_contrib;
        }
    }

    // Average
    double dx = sum_dx / 4.0;
    double dy = sum_dy / 4.0;
    double dtheta = sum_dtheta / 4.0;

    // Add drift
    dx += params_.odom_drift_rate * noise_dist_(rng_) * dt;
    dy += params_.odom_drift_rate * noise_dist_(rng_) * dt;

    // Transform to field frame using odometry pose
    double cos_th = std::cos(state_.odom_pose.theta);
    double sin_th = std::sin(state_.odom_pose.theta);
    double dx_field = dx * cos_th - dy * sin_th;
    double dy_field = dx * sin_th + dy * cos_th;

    // Update odometry pose
    state_.odom_pose.x += dx_field;
    state_.odom_pose.y += dy_field;
    state_.odom_pose.theta = normalize_angle(state_.odom_pose.theta + dtheta);
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
    dx_noise += params_.odom_drift_rate * noise_dist_(rng_) * 0.016;
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
