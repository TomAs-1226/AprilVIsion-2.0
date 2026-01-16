#pragma once
/**
 * @file robot_dynamics.hpp
 * @brief Simulated robot dynamics with realistic swerve kinematics
 *
 * Features:
 * - 4-wheel swerve drive kinematics
 * - Motor response curves (first-order lag)
 * - Wheel slip simulation
 * - Battery voltage sag
 * - Configurable odometry noise
 */

#include "sim_types.hpp"
#include <random>
#include <array>

namespace frc_vision {
namespace sim {

/**
 * @brief Module position enum for clarity
 */
enum class ModulePosition {
    FRONT_LEFT = 0,
    FRONT_RIGHT = 1,
    BACK_LEFT = 2,
    BACK_RIGHT = 3
};

/**
 * @brief Simulates robot motion with realistic swerve dynamics and odometry noise
 */
class RobotDynamics {
public:
    RobotDynamics();

    /**
     * @brief Initialize with parameters
     */
    void initialize(const DynamicsParams& params, const Pose2D& start_pose);

    /**
     * @brief Update robot state based on input
     * @param input Current input state
     * @param dt Time step in seconds
     */
    void update(const InputState& input, double dt);

    /**
     * @brief Get current robot state
     */
    const RobotState& state() const { return state_; }

    /**
     * @brief Get mutable robot state (for vision correction)
     */
    RobotState& state() { return state_; }

    /**
     * @brief Reset to starting pose
     */
    void reset(const Pose2D& pose);

    /**
     * @brief Apply external velocity command (for auto-align / path following)
     */
    void apply_velocity_command(double vx, double vy, double omega);

    /**
     * @brief Check if using external velocity command
     */
    bool using_external_command() const { return use_external_cmd_; }

    /**
     * @brief Set external command mode
     */
    void set_external_command_mode(bool enabled);

    /**
     * @brief Get current velocities
     */
    void get_velocities(double& vx, double& vy, double& omega) const;

    /**
     * @brief Get swerve module states (for visualization)
     */
    const std::array<SwerveModuleState, 4>& get_module_states() const { return state_.modules; }

    /**
     * @brief Get current battery voltage (accounts for sag)
     */
    double get_battery_voltage() const { return state_.battery_voltage; }

    /**
     * @brief Check if robot is experiencing wheel slip
     */
    bool is_slipping() const { return state_.is_slipping; }

private:
    // Basic helpers
    double clamp(double value, double min_val, double max_val) const;
    double normalize_angle(double angle) const;

    // Simple dynamics (fallback mode)
    void update_simple(const InputState& input, double dt);

    // Accurate swerve simulation
    void update_swerve(const InputState& input, double dt);

    // Swerve kinematics helpers
    void compute_module_targets(double vx, double vy, double omega,
                                std::array<double, 4>& speeds,
                                std::array<double, 4>& angles);
    void update_module_states(double dt);
    void compute_chassis_motion(double dt, double& dx, double& dy, double& dtheta);

    // Motor simulation
    double simulate_motor_response(double current, double target, double tau, double dt);
    double compute_motor_torque(double voltage, double speed, double kt, double kv, double resistance);
    double compute_wheel_force(double torque, double wheel_radius, double gear_ratio);

    // Wheel slip
    double compute_slip_ratio(double wheel_speed, double ground_speed);
    double compute_traction_limit(double normal_force, double cof);
    void apply_traction_limits(double dt);

    // Battery simulation
    void update_battery(double dt);

    // Odometry
    void add_odometry_noise(double dx, double dy, double dtheta);
    void update_odometry_from_modules(double dt);

    // Parameters
    DynamicsParams params_;
    RobotState state_;

    // Module positions relative to robot center (computed from track_width/wheel_base)
    std::array<std::pair<double, double>, 4> module_positions_;

    // External command (for auto-align / path following)
    bool use_external_cmd_ = false;
    double cmd_vx_ = 0, cmd_vy_ = 0, cmd_omega_ = 0;

    // Random generator for noise
    std::mt19937 rng_;
    std::normal_distribution<double> noise_dist_{0.0, 1.0};
};

} // namespace sim
} // namespace frc_vision
