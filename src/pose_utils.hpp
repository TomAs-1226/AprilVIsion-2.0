/**
 * @file pose_utils.hpp
 * @brief Proven coordinate transform utilities for FRC vision
 *
 * Uses the same approach as PhotonVision: work with 4x4 homogeneous transforms
 * throughout the pipeline, and only extract (x, y, theta) at the very end.
 *
 * Coordinate systems:
 *   OpenCV camera: X-right, Y-down, Z-forward
 *   FRC field:     X-forward (downfield), Y-left, Z-up
 *   Tag local:     Defined by AprilTag library (Z out of tag face)
 *
 * Key insight: solvePnP gives [R|t] mapping object frame → OpenCV camera frame.
 * When object points are in FRC field coords, PnP gives field→camera(OpenCV).
 * Inverting gives camera(OpenCV)→field(FRC). The camera origin in field coords
 * is simply -R^T * t, which is ALREADY in FRC coords (no conversion needed!).
 */

#pragma once

#include "types.hpp"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <iostream>

namespace frc_vision {
namespace pose_utils {

// ============================================================================
// 4x4 Homogeneous Transform Helpers (proven, no ambiguity)
// ============================================================================

/**
 * @brief Build 4x4 homogeneous transform from rvec + tvec
 */
inline cv::Mat make_transform(const cv::Vec3d& rvec, const cv::Vec3d& tvec) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    T.at<double>(0, 3) = tvec[0];
    T.at<double>(1, 3) = tvec[1];
    T.at<double>(2, 3) = tvec[2];
    return T;
}

/**
 * @brief Invert a 4x4 homogeneous transform (exact, no numerical error)
 */
inline cv::Mat invert_transform(const cv::Mat& T) {
    cv::Mat R = T(cv::Rect(0, 0, 3, 3));
    cv::Mat t = T(cv::Rect(3, 0, 1, 3));
    cv::Mat Rt = R.t();
    cv::Mat T_inv = cv::Mat::eye(4, 4, CV_64F);
    Rt.copyTo(T_inv(cv::Rect(0, 0, 3, 3)));
    cv::Mat t_inv = -Rt * t;
    t_inv.copyTo(T_inv(cv::Rect(3, 0, 1, 3)));
    return T_inv;
}

/**
 * @brief Extract position and rvec from a 4x4 transform
 */
inline void decompose_transform(const cv::Mat& T, cv::Vec3d& rvec, cv::Vec3d& tvec) {
    cv::Mat R = T(cv::Rect(0, 0, 3, 3));
    cv::Rodrigues(R, rvec);
    tvec = cv::Vec3d(T.at<double>(0, 3), T.at<double>(1, 3), T.at<double>(2, 3));
}

// ============================================================================
// Euler Angle Extraction (for robot heading)
// ============================================================================

struct EulerAngles {
    double roll = 0.0;   // Rotation around X
    double pitch = 0.0;  // Rotation around Y
    double yaw = 0.0;    // Rotation around Z (heading)
    bool gimbal_lock = false;
};

/**
 * @brief Extract Euler angles from rotation matrix (ZYX convention)
 */
inline EulerAngles rotation_matrix_to_euler_zyx(const cv::Mat& R) {
    EulerAngles angles;
    double sy = std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +
                           R.at<double>(1, 0) * R.at<double>(1, 0));
    if (sy < 1e-6) {
        angles.gimbal_lock = true;
        angles.roll = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        angles.pitch = std::atan2(-R.at<double>(2, 0), sy);
        angles.yaw = 0.0;
    } else {
        angles.roll = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        angles.pitch = std::atan2(-R.at<double>(2, 0), sy);
        angles.yaw = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    }
    return angles;
}

// ============================================================================
// The proven PhotonVision approach: extract robot pose from PnP result
// ============================================================================

/**
 * @brief Extract camera (x, y, heading) on field from PnP result.
 *
 * This is the PROVEN approach used by PhotonVision and Limelight.
 *
 * When PnP object points are in FRC field coords:
 *   - PnP gives [R|t] mapping field(FRC) → camera(OpenCV)
 *   - Camera position in field = -R^T * t (already in FRC coords!)
 *   - Camera heading = direction camera Z-axis points on field XY plane
 *
 * @param rvec Rotation vector from solvePnP
 * @param tvec Translation vector from solvePnP
 * @return (x, y, theta) of the camera in field frame
 */
inline Pose2D extract_camera_field_pose(const cv::Vec3d& rvec, const cv::Vec3d& tvec) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // Camera position in field = -R^T * t
    // This is exact: the PnP object points are in FRC coords,
    // so the inverse transform position is automatically in FRC coords.
    cv::Mat camera_pos = -R.t() * cv::Mat(tvec);

    Pose2D pose;
    pose.x = camera_pos.at<double>(0);
    pose.y = camera_pos.at<double>(1);

    // Camera heading: camera's Z-axis (forward in OpenCV) projected onto field XY
    // R^T maps from camera(OpenCV) → field(FRC)
    // Camera Z = [0,0,1] in camera frame
    // Camera Z in field = R^T * [0,0,1] = third column of R^T = third row of R
    double cam_z_in_field_x = R.at<double>(2, 0);
    double cam_z_in_field_y = R.at<double>(2, 1);
    pose.theta = std::atan2(cam_z_in_field_y, cam_z_in_field_x);

    return pose;
}

/**
 * @brief Convert camera field pose to robot field pose using extrinsics.
 *
 * Camera extrinsics from config are in FRC robot body coords:
 *   x = forward of robot center (meters)
 *   y = left of robot center (meters)
 *   yaw = rotation of camera relative to robot forward (radians)
 *
 * @param camera_pose Camera (x, y, theta) in field frame
 * @param cam_x_robot Camera X offset in robot frame (forward, meters)
 * @param cam_y_robot Camera Y offset in robot frame (left, meters)
 * @param cam_yaw_robot Camera yaw offset from robot forward (radians)
 * @return Robot (x, y, theta) in field frame
 */
inline Pose2D camera_to_robot_field_pose(
    const Pose2D& camera_pose,
    double cam_x_robot,
    double cam_y_robot,
    double cam_yaw_robot)
{
    // Robot heading = camera heading minus camera yaw offset
    double robot_heading = camera_pose.theta - cam_yaw_robot;

    // Robot position = camera position minus rotated camera offset
    // The camera offset is in robot body frame, so we rotate by robot heading
    double cos_h = std::cos(robot_heading);
    double sin_h = std::sin(robot_heading);

    Pose2D robot_pose;
    robot_pose.x = camera_pose.x - (cam_x_robot * cos_h - cam_y_robot * sin_h);
    robot_pose.y = camera_pose.y - (cam_x_robot * sin_h + cam_y_robot * cos_h);
    robot_pose.theta = robot_heading;

    // Normalize to [-pi, pi]
    while (robot_pose.theta > M_PI) robot_pose.theta -= 2.0 * M_PI;
    while (robot_pose.theta < -M_PI) robot_pose.theta += 2.0 * M_PI;

    return robot_pose;
}

/**
 * @brief For single-tag: compose tag_to_camera with tag_to_field
 * to get camera(OpenCV)→field(FRC) as a 4x4 transform.
 *
 * tag_to_camera: from solvePnP (maps tag local → camera OpenCV)
 * tag_to_field: from field layout (maps tag local → field FRC)
 *
 * camera_to_field = tag_to_field * camera_to_tag
 *                 = tag_to_field * inv(tag_to_camera)
 *
 * The intermediate frame (tag local) cancels out, making this valid
 * even though the two transforms use different output coord systems.
 *
 * Returns (x, y, theta) of camera in field frame.
 */
inline Pose2D single_tag_camera_field_pose(
    const cv::Vec3d& tag_to_camera_rvec,
    const cv::Vec3d& tag_to_camera_tvec,
    const Pose3D& tag_pose_in_field)
{
    // Build 4x4 transforms
    cv::Mat T_tag_to_camera = make_transform(tag_to_camera_rvec, tag_to_camera_tvec);
    cv::Mat T_tag_to_field = make_transform(tag_pose_in_field.rvec(), tag_pose_in_field.tvec());

    // camera_to_field = tag_to_field * inv(tag_to_camera)
    cv::Mat T_camera_to_tag = invert_transform(T_tag_to_camera);
    cv::Mat T_camera_to_field = T_tag_to_field * T_camera_to_tag;

    // Extract camera position in field (from the 4x4 transform)
    Pose2D pose;
    pose.x = T_camera_to_field.at<double>(0, 3);
    pose.y = T_camera_to_field.at<double>(1, 3);

    // Camera heading in field: camera Z-axis direction on field XY plane
    // The camera's Z-axis in field coords is the third column of the
    // rotation part of T_camera_to_field (it maps camera axes to field axes)
    double cam_z_field_x = T_camera_to_field.at<double>(0, 2);
    double cam_z_field_y = T_camera_to_field.at<double>(1, 2);
    pose.theta = std::atan2(cam_z_field_y, cam_z_field_x);

    return pose;
}

// ============================================================================
// Utility functions
// ============================================================================

inline double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

inline double angle_diff(double a, double b) {
    return normalize_angle(a - b);
}

inline double deg_to_rad(double degrees) {
    return degrees * M_PI / 180.0;
}

inline double rad_to_deg(double radians) {
    return radians * 180.0 / M_PI;
}

// Keep this for backward compatibility with code that references it,
// but the new pipeline doesn't use it - we use the 4x4 approach instead.
inline Pose3D opencv_camera_to_frc_field(const Pose3D& opencv_pose) {
    cv::Mat T_opencv_to_frc = (cv::Mat_<double>(3, 3) <<
        0,  0,  1,
        -1, 0,  0,
        0, -1,  0);

    cv::Vec3d pos_opencv(opencv_pose.position.x,
                         opencv_pose.position.y,
                         opencv_pose.position.z);
    cv::Mat pos_frc_mat = T_opencv_to_frc * cv::Mat(pos_opencv);
    cv::Vec3d pos_frc(pos_frc_mat.at<double>(0),
                      pos_frc_mat.at<double>(1),
                      pos_frc_mat.at<double>(2));

    cv::Mat R_opencv;
    cv::Rodrigues(opencv_pose.rvec(), R_opencv);
    cv::Mat R_frc = T_opencv_to_frc * R_opencv * T_opencv_to_frc.t();

    cv::Vec3d rvec_frc;
    cv::Rodrigues(R_frc, rvec_frc);

    return Pose3D::from_rvec_tvec(rvec_frc, pos_frc);
}

inline Pose2D extract_robot_pose_2d(const Pose3D& robot_to_field_frc) {
    Pose2D pose;
    pose.x = robot_to_field_frc.position.x;
    pose.y = robot_to_field_frc.position.y;

    cv::Mat R;
    cv::Rodrigues(robot_to_field_frc.rvec(), R);
    EulerAngles angles = rotation_matrix_to_euler_zyx(R);
    pose.theta = angles.yaw;

    while (pose.theta > M_PI) pose.theta -= 2.0 * M_PI;
    while (pose.theta < -M_PI) pose.theta += 2.0 * M_PI;

    return pose;
}

} // namespace pose_utils
} // namespace frc_vision
