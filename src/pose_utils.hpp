/**
 * @file pose_utils.hpp
 * @brief Advanced pose calculation utilities with robust coordinate transforms
 *
 * Fixes for angle and distance accuracy issues:
 * - Proper coordinate system transformations (OpenCV camera -> FRC field)
 * - Robust Euler angle extraction with gimbal lock handling
 * - Enhanced distance calculation with multi-method validation
 * - Diagnostic outputs for debugging
 */

#pragma once

#include "types.hpp"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <iostream>

namespace frc_vision {
namespace pose_utils {

/**
 * @brief Convert rotation matrix to Euler angles (roll, pitch, yaw) with gimbal lock detection
 *
 * Uses ZYX convention (yaw-pitch-roll) which is standard for FRC
 * Returns angles in radians
 */
struct EulerAngles {
    double roll = 0.0;   // Rotation around X
    double pitch = 0.0;  // Rotation around Y
    double yaw = 0.0;    // Rotation around Z (heading)
    bool gimbal_lock = false;  // True if near gimbal lock (pitch ~= ±90°)
};

inline EulerAngles rotation_matrix_to_euler_zyx(const cv::Mat& R) {
    EulerAngles angles;

    // Check for gimbal lock (pitch = ±90°)
    double sy = std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +
                           R.at<double>(1, 0) * R.at<double>(1, 0));

    if (sy < 1e-6) {
        // Gimbal lock case
        angles.gimbal_lock = true;
        angles.roll = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        angles.pitch = std::atan2(-R.at<double>(2, 0), sy);
        angles.yaw = 0.0;  // Yaw is arbitrary in gimbal lock
    } else {
        // Normal case
        angles.roll = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        angles.pitch = std::atan2(-R.at<double>(2, 0), sy);
        angles.yaw = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    }

    return angles;
}

/**
 * @brief Transform pose from OpenCV camera coordinates to FRC field coordinates
 *
 * OpenCV camera: X-right, Y-down, Z-forward
 * FRC field: X-forward (downfield), Y-left, Z-up
 *
 * This transformation matrix rotates:
 * - OpenCV +Z (forward) -> FRC +X (downfield)
 * - OpenCV +X (right) -> FRC -Y (right becomes field-left)
 * - OpenCV +Y (down) -> FRC -Z (down becomes field-down)
 */
inline Pose3D opencv_camera_to_frc_field(const Pose3D& opencv_pose) {
    // Transformation matrix from OpenCV camera coords to FRC field coords
    // This is a 90° rotation to align coordinate systems
    cv::Mat T_opencv_to_frc = (cv::Mat_<double>(3, 3) <<
        0,  0,  1,   // X_frc = Z_opencv (forward)
        -1, 0,  0,   // Y_frc = -X_opencv (left)
        0, -1,  0);  // Z_frc = -Y_opencv (up)

    // Transform position
    cv::Vec3d pos_opencv(opencv_pose.position.x,
                         opencv_pose.position.y,
                         opencv_pose.position.z);
    cv::Mat pos_frc_mat = T_opencv_to_frc * cv::Mat(pos_opencv);
    cv::Vec3d pos_frc(pos_frc_mat.at<double>(0),
                      pos_frc_mat.at<double>(1),
                      pos_frc_mat.at<double>(2));

    // Transform rotation
    cv::Mat R_opencv;
    cv::Rodrigues(opencv_pose.rvec(), R_opencv);
    cv::Mat R_frc = T_opencv_to_frc * R_opencv * T_opencv_to_frc.t();

    cv::Vec3d rvec_frc;
    cv::Rodrigues(R_frc, rvec_frc);

    return Pose3D::from_rvec_tvec(rvec_frc, pos_frc);
}

/**
 * @brief Extract robot 2D pose with proper FRC coordinate system
 *
 * Handles coordinate system transformations and returns (x, y, theta)
 * where theta is heading in radians (CCW from +X axis)
 */
inline Pose2D extract_robot_pose_2d(const Pose3D& robot_to_field_frc) {
    Pose2D pose;

    // Position is straightforward (already in FRC coords)
    pose.x = robot_to_field_frc.position.x;
    pose.y = robot_to_field_frc.position.y;

    // Extract yaw angle (heading) from rotation matrix
    cv::Mat R;
    cv::Rodrigues(robot_to_field_frc.rvec(), R);

    EulerAngles angles = rotation_matrix_to_euler_zyx(R);

    if (angles.gimbal_lock) {
        std::cerr << "[Pose] Warning: Gimbal lock detected, yaw may be inaccurate" << std::endl;
    }

    // Yaw is rotation around Z-axis (up)
    pose.theta = angles.yaw;

    // Normalize angle to [-π, π]
    while (pose.theta > M_PI) pose.theta -= 2.0 * M_PI;
    while (pose.theta < -M_PI) pose.theta += 2.0 * M_PI;

    return pose;
}

/**
 * @brief Compute distance with enhanced accuracy and diagnostics
 *
 * Returns DistanceEstimate with multiple methods and consistency checking
 */
inline DistanceEstimate compute_robust_distance(
    const cv::Vec3d& tvec,
    const std::vector<cv::Point2d>& corners_2d,
    double tag_size_m,
    double focal_length,
    bool verbose = false)
{
    DistanceEstimate est;

    // Method 1: Direct from translation vector (most accurate for good pose)
    est.distance_pnp = cv::norm(tvec);

    // Method 2: Pinhole model from average edge length
    if (corners_2d.size() == 4) {
        double edge_sum = 0.0;
        for (size_t i = 0; i < 4; i++) {
            size_t j = (i + 1) % 4;
            edge_sum += cv::norm(corners_2d[j] - corners_2d[i]);
        }
        double avg_edge_pixels = edge_sum / 4.0;

        if (avg_edge_pixels > 1.0 && focal_length > 0.0) {
            est.distance_pinhole = (tag_size_m * focal_length) / avg_edge_pixels;
        }

        // Method 3 & 4: Vertical and horizontal edges
        double vert1 = cv::norm(corners_2d[2] - corners_2d[1]); // right edge
        double vert2 = cv::norm(corners_2d[0] - corners_2d[3]); // left edge
        double horiz1 = cv::norm(corners_2d[1] - corners_2d[0]); // bottom edge
        double horiz2 = cv::norm(corners_2d[3] - corners_2d[2]); // top edge

        double vert_avg = (vert1 + vert2) / 2.0;
        double horiz_avg = (horiz1 + horiz2) / 2.0;

        if (vert_avg > 1.0) {
            est.distance_vertical_edges = (tag_size_m * focal_length) / vert_avg;
        }
        if (horiz_avg > 1.0) {
            est.distance_horizontal_edges = (tag_size_m * focal_length) / horiz_avg;
        }

        // Consistency checks
        if (est.distance_vertical_edges > 0.0 && est.distance_horizontal_edges > 0.0) {
            double diff = std::abs(est.distance_vertical_edges - est.distance_horizontal_edges);
            est.edge_consistency = std::exp(-diff / 0.3);
        }
    }

    if (est.distance_pnp > 0.0 && est.distance_pinhole > 0.0) {
        double diff = std::abs(est.distance_pnp - est.distance_pinhole);
        est.pnp_pinhole_agreement = std::exp(-diff / 0.5);
    }

    // Weighted fusion
    double total_weight = 0.0;
    double weighted_sum = 0.0;

    // Trust PnP most when it's available (70% weight)
    if (est.distance_pnp > 0.01 && est.distance_pnp < 20.0) {
        weighted_sum += est.distance_pnp * 0.7;
        total_weight += 0.7;
    }

    // Pinhole is robust backup (30% weight)
    if (est.distance_pinhole > 0.01 && est.distance_pinhole < 20.0) {
        weighted_sum += est.distance_pinhole * 0.3;
        total_weight += 0.3;
    }

    if (total_weight > 0.0) {
        est.distance_fused = weighted_sum / total_weight;
    } else {
        est.distance_fused = est.distance_pnp > 0.0 ? est.distance_pnp : 10.0;
    }

    est.confidence = est.pnp_pinhole_agreement * 0.6 + est.edge_consistency * 0.4;
    est.is_consistent = (est.confidence > 0.7);

    if (verbose) {
        std::cout << "[Distance] PnP: " << est.distance_pnp << "m, "
                  << "Pinhole: " << est.distance_pinhole << "m, "
                  << "Fused: " << est.distance_fused << "m, "
                  << "Confidence: " << est.confidence << std::endl;
    }

    return est;
}

/**
 * @brief Normalize angle to [-π, π] range
 */
inline double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

/**
 * @brief Compute angle difference (shortest path)
 */
inline double angle_diff(double a, double b) {
    double diff = a - b;
    return normalize_angle(diff);
}

/**
 * @brief Convert degrees to radians
 */
inline double deg_to_rad(double degrees) {
    return degrees * M_PI / 180.0;
}

/**
 * @brief Convert radians to degrees
 */
inline double rad_to_deg(double radians) {
    return radians * 180.0 / M_PI;
}

} // namespace pose_utils
} // namespace frc_vision
