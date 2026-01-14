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
     * @brief Get quality metrics for pose estimate
     */
    static PoseQuality compute_quality(
        const std::vector<TagDetection>& detections,
        double reproj_error);

    /**
     * @brief Estimate standard deviations based on detection quality
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
};

} // namespace frc_vision
