#pragma once
/**
 * @file robot_dynamics.hpp
 * @brief Simulated robot dynamics with odometry drift
 */

#include "sim_types.hpp"
#include <random>

namespace frc_vision {
namespace sim {

/**
 * @brief Simulates robot motion with realistic dynamics and odometry noise
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
     * @brief Apply external velocity command (for auto-align)
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

private:
    double clamp(double value, double min_val, double max_val) const;
    double normalize_angle(double angle) const;
    void add_odometry_noise(double dx, double dy, double dtheta);

    DynamicsParams params_;
    RobotState state_;

    // External command (for auto-align)
    bool use_external_cmd_ = false;
    double cmd_vx_ = 0, cmd_vy_ = 0, cmd_omega_ = 0;

    // Random generator for noise
    std::mt19937 rng_;
    std::normal_distribution<double> noise_dist_{0.0, 1.0};
};

} // namespace sim
} // namespace frc_vision
