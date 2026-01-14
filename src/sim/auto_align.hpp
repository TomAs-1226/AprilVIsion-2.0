#pragma once
/**
 * @file auto_align.hpp
 * @brief Auto-align controller using vision-corrected pose
 *
 * Implements PID control to drive robot to face nearest tag
 * at a specified standoff distance.
 */

#include "sim_types.hpp"
#include "../types.hpp"

namespace frc_vision {
namespace sim {

/**
 * @brief PID controller for single axis
 */
class PIDController {
public:
    PIDController(double kp = 0, double ki = 0, double kd = 0);

    void set_gains(double kp, double ki, double kd);
    void set_limits(double min_output, double max_output);
    void set_integral_limits(double min_integral, double max_integral);

    double calculate(double setpoint, double measurement, double dt);
    void reset();

private:
    double kp_, ki_, kd_;
    double min_output_ = -1e9, max_output_ = 1e9;
    double min_integral_ = -1e9, max_integral_ = 1e9;

    double integral_ = 0;
    double prev_error_ = 0;
    bool first_run_ = true;
};

/**
 * @brief Auto-align controller
 *
 * Drives robot to face nearest visible tag at standoff distance.
 */
class AutoAlignController {
public:
    AutoAlignController();

    /**
     * @brief Initialize with parameters
     */
    void initialize(const AutoAlignParams& params, const FieldLayout& field);

    /**
     * @brief Update controller with current state
     * @param current_pose Current fused robot pose
     * @param visible_tag_ids IDs of currently visible tags
     * @param dt Time step in seconds
     * @param vx_out Output velocity X (forward)
     * @param vy_out Output velocity Y (left)
     * @param omega_out Output angular velocity (CCW)
     * @return true if actively aligning, false if at target
     */
    bool update(const Pose2D& current_pose,
               const std::vector<int>& visible_tag_ids,
               double dt,
               double& vx_out, double& vy_out, double& omega_out);

    /**
     * @brief Enable/disable auto-align
     */
    void set_enabled(bool enabled);
    bool is_enabled() const { return enabled_; }

    /**
     * @brief Check if at target position
     */
    bool is_at_target() const { return at_target_; }

    /**
     * @brief Get current target tag ID
     */
    int target_tag_id() const { return target_tag_id_; }

    /**
     * @brief Set specific target tag (or -1 for nearest)
     */
    void set_target_tag(int tag_id) { target_tag_id_ = tag_id; }

    /**
     * @brief Reset controller state
     */
    void reset();

private:
    double normalize_angle(double angle) const;
    int find_nearest_tag(const Pose2D& pose, const std::vector<int>& visible_ids) const;
    void compute_target(const Pose2D& pose, int tag_id, double& target_x, double& target_y, double& target_heading) const;

    AutoAlignParams params_;
    FieldLayout field_;

    bool enabled_ = false;
    bool at_target_ = false;
    int target_tag_id_ = -1;

    // PID controllers
    PIDController pos_x_pid_;
    PIDController pos_y_pid_;
    PIDController heading_pid_;
};

} // namespace sim
} // namespace frc_vision
