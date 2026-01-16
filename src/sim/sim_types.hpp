#pragma once
/**
 * @file sim_types.hpp
 * @brief Type definitions for Mac simulator
 */

#include "../types.hpp"
#include <opencv2/core.hpp>

namespace frc_vision {
namespace sim {

/**
 * @brief Simulated robot state with ground truth and odometry
 */
struct RobotState {
    // Ground truth (actual position)
    Pose2D true_pose;

    // Odometry (with drift/noise)
    Pose2D odom_pose;

    // Vision-corrected fused pose
    Pose2D fused_pose;

    // Velocities
    double vx = 0;      // m/s forward
    double vy = 0;      // m/s left
    double omega = 0;   // rad/s CCW

    // Robot dimensions
    static constexpr double LENGTH = 0.75;  // meters (front to back)
    static constexpr double WIDTH = 0.65;   // meters (side to side)
};

/**
 * @brief User input state
 */
struct InputState {
    bool forward = false;   // W
    bool backward = false;  // S
    bool left = false;      // A
    bool right = false;     // D
    bool rotate_ccw = false; // Q
    bool rotate_cw = false;  // E
    bool turbo = false;     // Shift

    bool auto_align = false; // V toggle
    bool reset_pose = false; // R
    bool quit = false;       // ESC

    // Visualization toggles
    bool show_true_pose = true;   // 1
    bool show_odom_pose = true;   // 2
    bool show_fused_pose = true;  // 3
    bool show_webcam = true;      // 4
    bool detect_on_composite = true; // 5 (vs synthetic only)
};

/**
 * @brief Robot dynamics parameters
 */
struct DynamicsParams {
    // Max velocities
    double max_speed = 4.0;         // m/s
    double max_turbo_speed = 5.5;   // m/s
    double max_rotation = 6.0;      // rad/s

    // Acceleration
    double accel = 8.0;             // m/s^2
    double decel = 12.0;            // m/s^2 (brake faster than accel)
    double rot_accel = 15.0;        // rad/s^2

    // Odometry noise
    double odom_trans_noise = 0.02; // fraction of motion
    double odom_rot_noise = 0.03;   // fraction of rotation
    double odom_drift_rate = 0.005; // m/s drift
};

/**
 * @brief Auto-align controller parameters
 *
 * Tuned for fast, responsive alignment with vision feedback.
 * These base gains are multiplied by the controller for aggressive response.
 */
struct AutoAlignParams {
    // PID gains for position (base values - controller applies multipliers)
    double pos_kp = 3.0;            // Increased from 2.0
    double pos_ki = 0.05;           // Reduced for less windup
    double pos_kd = 0.5;            // Increased for damping

    // PID gains for heading
    double heading_kp = 6.0;        // Increased from 4.0 for snappy rotation
    double heading_ki = 0.05;       // Reduced for less windup
    double heading_kd = 0.4;        // Increased for damping

    // Limits
    double max_speed = 3.0;         // m/s during align (increased from 2.0)
    double max_rotation = 5.0;      // rad/s during align (increased from 3.0)

    // Tolerances
    double pos_tolerance = 0.03;    // meters (tightened from 0.05)
    double heading_tolerance = 0.015; // radians (tightened from 0.02)

    // Target standoff distance from tag
    double standoff_distance = 0.8; // meters (closer for precision)
};

/**
 * @brief Synthetic camera parameters
 */
struct SyntheticCameraParams {
    // Resolution
    int width = 640;
    int height = 480;

    // Intrinsics (will be loaded from file or defaults)
    double fx = 600.0;
    double fy = 600.0;
    double cx = 320.0;
    double cy = 240.0;

    // Noise/blur simulation
    bool add_noise = false;
    double noise_stddev = 5.0;
    bool motion_blur = false;
    int blur_kernel_size = 5;

    // FPS limiting
    double target_fps = 60.0;
};

/**
 * @brief Visualization parameters
 */
struct VisualizationParams {
    // Field view
    double field_view_scale = 60.0;  // pixels per meter
    int field_margin = 50;           // pixels

    // Colors (BGR)
    cv::Scalar true_pose_color = cv::Scalar(0, 255, 0);    // Green
    cv::Scalar odom_pose_color = cv::Scalar(0, 165, 255);  // Orange
    cv::Scalar fused_pose_color = cv::Scalar(255, 0, 255); // Magenta
    cv::Scalar tag_color = cv::Scalar(255, 255, 0);        // Cyan
    cv::Scalar robot_outline_color = cv::Scalar(200, 200, 200);

    // Robot visualization
    bool show_fov_cone = true;
    double fov_cone_length = 3.0;  // meters
};

/**
 * @brief Complete simulator configuration
 */
struct SimConfig {
    DynamicsParams dynamics;
    AutoAlignParams auto_align;
    SyntheticCameraParams camera;
    VisualizationParams visualization;

    // Paths
    std::string field_layout_path = "assets/2024-crescendo.json";
    std::string intrinsics_path = "assets/mac_cam_intrinsics.yml";
    std::string extrinsics_path = "assets/mac_cam_extrinsics.yml";

    // Starting pose
    Pose2D start_pose = {2.0, 2.0, 0.0};

    // Webcam device
    int webcam_device = 0;
    bool use_webcam = true;
};

} // namespace sim
} // namespace frc_vision
