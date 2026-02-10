#pragma once
/**
 * @file pose.hpp
 * @brief Single-tag and multi-tag pose estimation
 *
 * Implements:
 * - Single tag pose via PnP
 * - Multi-tag pose via combined corner PnP with RANSAC
 * - Reprojection error calculation for quality metrics
 */

#include "types.hpp"
#include "field_layout.hpp"

namespace frc_vision {

/**
 * @brief Pose estimator for single and multi-tag localization
 */
class PoseEstimator {
public:
    PoseEstimator();

    /**
     * @brief Initialize with field layout and tag size
     * @param layout Field layout with tag positions
     */
    void initialize(const FieldLayout& layout);

    /**
     * @brief Estimate pose from single tag detection
     * @param detection Tag detection with corners
     * @param intrinsics Camera intrinsic parameters
     * @return Updated detection with pose filled in
     */
    TagDetection estimate_single_tag(
        const TagDetection& detection,
        const CameraIntrinsics& intrinsics);

    /**
     * @brief Estimate camera pose from multiple tags (recommended when >= 2 tags)
     * @param detections Vector of tag detections
     * @param intrinsics Camera intrinsic parameters
     * @return Camera-to-field pose and robot pose
     */
    struct MultiTagResult {
        bool valid = false;
        Pose3D camera_to_field;
        Pose2D robot_pose_field;
        double reprojection_error = 0;
        int tags_used = 0;
        std::vector<int> inlier_tag_ids;
    };

    MultiTagResult estimate_multi_tag(
        const std::vector<TagDetection>& detections,
        const CameraIntrinsics& intrinsics,
        const Pose3D& camera_to_robot);

    /**
     * @brief Calculate reprojection error for a pose estimate
     * @param corners_2d Observed image corners
     * @param corners_3d World coordinates of corners
     * @param rvec Rotation vector of estimated pose
     * @param tvec Translation vector of estimated pose
     * @param intrinsics Camera intrinsics
     * @return RMS reprojection error in pixels
     */
    static double calculate_reprojection_error(
        const std::vector<cv::Point2d>& corners_2d,
        const std::vector<cv::Point3d>& corners_3d,
        const cv::Vec3d& rvec,
        const cv::Vec3d& tvec,
        const CameraIntrinsics& intrinsics);

    /**
     * @brief Convert camera pose to 2D robot pose
     * @param camera_to_field Camera pose in field frame
     * @param camera_to_robot Camera-to-robot transform
     * @return Robot pose (x, y, theta) in field frame
     */
    static Pose2D camera_to_robot_pose(
        const Pose3D& camera_to_field,
        const Pose3D& camera_to_robot);

    /**
     * @brief Compute distance to a tag using pinhole camera model.
     * distance = (real_tag_size * focal_length) / pixel_tag_size
     * @param detection Tag detection with corners
     * @param intrinsics Camera intrinsic parameters
     * @return Distance in meters
     */
    double compute_tag_distance(
        const TagDetection& detection,
        const CameraIntrinsics& intrinsics);

    /**
     * @brief Compute multi-method distance estimate with geometric consistency (Phase 1).
     * Calculates distance using multiple methods and validates consistency.
     * @param detection Tag detection with corners and pose
     * @param intrinsics Camera intrinsic parameters
     * @return Distance estimate with confidence based on method agreement
     */
    DistanceEstimate compute_distance_estimate(
        const TagDetection& detection,
        const CameraIntrinsics& intrinsics);

    /**
     * @brief Estimate per-tag accuracy based on viewing conditions (Phase 2).
     * @param detection Tag detection with pose and quality metrics
     * @param distance Measured distance to tag
     * @return Accuracy estimate with predicted errors
     */
    static TagAccuracyEstimate estimate_tag_accuracy(
        const TagDetection& detection,
        double distance);

    /**
     * @brief Get quality metrics for pose estimate.
     * Includes standard deviations for WPILib SwerveDrivePoseEstimator fusion.
     * @param detections Detected tags
     * @param reproj_error Reprojection error in pixels
     * @param intrinsics Camera intrinsics for distance calculation
     */
    PoseQuality compute_quality(
        const std::vector<TagDetection>& detections,
        double reproj_error,
        const CameraIntrinsics& intrinsics);

    /**
     * @brief Estimate standard deviations based on detection quality.
     * These values are consumed by the robot's pose estimator Kalman filter
     * to weight vision measurements against wheel odometry.
     * @param tag_count Number of tags used
     * @param avg_distance Average distance to tags (meters)
     * @param reproj_error Reprojection error (pixels)
     */
    static PoseQuality estimate_std_devs(
        int tag_count,
        double avg_distance,
        double reproj_error);

    /**
     * @brief Get the field layout
     */
    const FieldLayout& field_layout() const { return field_; }

private:
    FieldLayout field_;

    // Local tag corners for single-tag PnP
    std::vector<cv::Point3d> tag_corners_local_;

    // PhotonVision-style ambiguity threshold for single-tag poses.
    // Reject if best_reproj_err / alt_reproj_err > this value.
    // 0.2 = strict (PhotonVision default), higher = more permissive.
    static constexpr double MAX_SINGLE_TAG_AMBIGUITY = 0.2;
};

} // namespace frc_vision
