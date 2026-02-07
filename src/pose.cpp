/**
 * @file pose.cpp
 * @brief Pose estimation implementation
 */

#include "pose.hpp"
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <cmath>
#include <set>

namespace frc_vision {

PoseEstimator::PoseEstimator() = default;

void PoseEstimator::initialize(const FieldLayout& layout) {
    field_ = layout;

    // Pre-compute local tag corners (centered at origin)
    auto corners = get_tag_corners_local(field_.tag_size_m);
    tag_corners_local_.clear();
    for (const auto& c : corners) {
        tag_corners_local_.push_back(c.to_cv());
    }

    std::cout << "[PoseEstimator] Initialized with " << field_.tags.size()
              << " field tags, size=" << field_.tag_size_m << "m" << std::endl;
}

TagDetection PoseEstimator::estimate_single_tag(
    const TagDetection& detection,
    const CameraIntrinsics& intrinsics)
{
    TagDetection result = detection;

    if (!intrinsics.valid() || tag_corners_local_.empty()) {
        result.pose_valid = false;
        return result;
    }

    // Build image points
    std::vector<cv::Point2d> image_points;
    for (const auto& corner : detection.corners.corners) {
        image_points.push_back(corner.to_cv());
    }

    // Solve PnP (tag-local frame to camera frame)
    cv::Vec3d rvec, tvec;
    bool success = cv::solvePnP(
        tag_corners_local_,
        image_points,
        intrinsics.camera_matrix,
        intrinsics.dist_coeffs,
        rvec, tvec,
        false,
        cv::SOLVEPNP_IPPE_SQUARE  // Best for square markers
    );

    if (!success) {
        result.pose_valid = false;
        return result;
    }

    // Calculate reprojection error
    result.reprojection_error = calculate_reprojection_error(
        image_points, tag_corners_local_, rvec, tvec, intrinsics);

    // Store pose (tag frame to camera frame)
    result.tag_to_camera = Pose3D::from_rvec_tvec(rvec, tvec);
    result.pose_valid = true;

    return result;
}

PoseEstimator::MultiTagResult PoseEstimator::estimate_multi_tag(
    const std::vector<TagDetection>& detections,
    const CameraIntrinsics& intrinsics,
    const Pose3D& camera_to_robot)
{
    MultiTagResult result;

    if (!intrinsics.valid()) {
        return result;
    }

    // Collect all known tag corners in field frame
    std::vector<cv::Point3d> object_points;
    std::vector<cv::Point2d> image_points;
    std::vector<int> tag_ids;

    for (const auto& det : detections) {
        if (!field_.has_tag(det.id)) {
            continue;
        }

        const auto& field_tag = field_.get_tag(det.id);
        auto corners_3d = field_tag.get_corners_cv();

        for (int i = 0; i < 4; i++) {
            object_points.push_back(corners_3d[i]);
            image_points.push_back(det.corners.corners[i].to_cv());
        }

        tag_ids.push_back(det.id);
    }

    if (tag_ids.size() < 1) {
        // No known tags
        return result;
    }

    if (tag_ids.size() == 1) {
        // Single tag - use single tag estimation with field transform
        auto det_with_pose = estimate_single_tag(detections[0], intrinsics);
        if (!det_with_pose.pose_valid || !field_.has_tag(det_with_pose.id)) {
            return result;
        }

        // Transform: camera = tag_to_camera * tag^-1 * field
        // We want: camera_to_field = field_to_tag * tag_to_camera^-1
        const auto& field_tag = field_.get_tag(det_with_pose.id);
        Pose3D camera_to_tag = det_with_pose.tag_to_camera.inverse();
        result.camera_to_field = field_tag.pose_field.compose(camera_to_tag);

        result.reprojection_error = det_with_pose.reprojection_error;
        result.tags_used = 1;
        result.inlier_tag_ids.push_back(det_with_pose.id);
        result.valid = true;

    } else {
        // Multiple tags - solve combined PnP with RANSAC
        cv::Vec3d rvec, tvec;
        cv::Mat inliers;

        bool success = cv::solvePnPRansac(
            object_points,
            image_points,
            intrinsics.camera_matrix,
            intrinsics.dist_coeffs,
            rvec, tvec,
            false,
            100,    // iterations
            8.0,    // reprojection threshold
            0.99,   // confidence
            inliers,
            cv::SOLVEPNP_SQPNP  // More robust for multiple points
        );

        if (!success) {
            // Fallback to regular PnP
            success = cv::solvePnP(
                object_points,
                image_points,
                intrinsics.camera_matrix,
                intrinsics.dist_coeffs,
                rvec, tvec,
                false,
                cv::SOLVEPNP_SQPNP
            );
        }

        if (!success) {
            return result;
        }

        // This gives us field_to_camera (world points in field frame)
        // We want camera_to_field
        Pose3D field_to_camera = Pose3D::from_rvec_tvec(rvec, tvec);
        result.camera_to_field = field_to_camera.inverse();

        // Calculate reprojection error
        result.reprojection_error = calculate_reprojection_error(
            image_points, object_points, rvec, tvec, intrinsics);

        // Count inliers
        if (!inliers.empty()) {
            std::set<int> inlier_tags;
            for (int i = 0; i < inliers.rows; i++) {
                int corner_idx = inliers.at<int>(i);
                int tag_idx = corner_idx / 4;
                if (tag_idx < static_cast<int>(tag_ids.size())) {
                    inlier_tags.insert(tag_ids[tag_idx]);
                }
            }
            result.inlier_tag_ids.assign(inlier_tags.begin(), inlier_tags.end());
            result.tags_used = static_cast<int>(inlier_tags.size());
        } else {
            result.inlier_tag_ids = tag_ids;
            result.tags_used = static_cast<int>(tag_ids.size());
        }

        result.valid = true;
    }

    // Convert to robot pose
    if (result.valid) {
        result.robot_pose_field = camera_to_robot_pose(
            result.camera_to_field, camera_to_robot);
    }

    return result;
}

double PoseEstimator::calculate_reprojection_error(
    const std::vector<cv::Point2d>& corners_2d,
    const std::vector<cv::Point3d>& corners_3d,
    const cv::Vec3d& rvec,
    const cv::Vec3d& tvec,
    const CameraIntrinsics& intrinsics)
{
    if (corners_2d.empty() || corners_3d.empty()) {
        return std::numeric_limits<double>::max();
    }

    std::vector<cv::Point2d> projected;
    cv::projectPoints(corners_3d, rvec, tvec,
                     intrinsics.camera_matrix, intrinsics.dist_coeffs,
                     projected);

    double sum_sq = 0;
    for (size_t i = 0; i < corners_2d.size(); i++) {
        double dx = corners_2d[i].x - projected[i].x;
        double dy = corners_2d[i].y - projected[i].y;
        sum_sq += dx * dx + dy * dy;
    }

    return std::sqrt(sum_sq / corners_2d.size());
}

Pose2D PoseEstimator::camera_to_robot_pose(
    const Pose3D& camera_to_field,
    const Pose3D& camera_to_robot)
{
    // robot_to_field = camera_to_field * robot_to_camera
    // where robot_to_camera = camera_to_robot^-1
    Pose3D robot_to_camera = camera_to_robot.inverse();
    Pose3D robot_to_field = camera_to_field.compose(robot_to_camera);

    // Extract 2D pose (x, y, yaw)
    Pose2D result;
    result.x = robot_to_field.position.x;
    result.y = robot_to_field.position.y;

    // Extract yaw from rotation
    // For a rotation matrix R, yaw = atan2(R21, R11) for ZYX convention
    cv::Mat R;
    cv::Rodrigues(robot_to_field.rvec(), R);

    // Assuming robot Z is up, we want rotation around Z
    result.theta = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));

    return result;
}

PoseQuality PoseEstimator::compute_quality(
    const std::vector<TagDetection>& detections,
    double reproj_error)
{
    PoseQuality quality;
    quality.tag_count = static_cast<int>(detections.size());
    quality.avg_reproj_error = reproj_error;

    if (detections.empty()) {
        return quality;
    }

    double margin_sum = 0;
    double area_sum = 0;
    for (const auto& det : detections) {
        margin_sum += det.decision_margin;
        area_sum += det.corners.area();
    }

    quality.avg_margin = margin_sum / detections.size();

    // Estimate distance from tag pixel area
    // Larger area = closer = more accurate
    double avg_area = area_sum / detections.size();
    double estimated_distance = 1000.0 / std::sqrt(avg_area);  // Rough approximation

    // Compute confidence (0-1)
    double tag_factor = std::min(1.0, quality.tag_count / 4.0);
    double margin_factor = std::min(1.0, quality.avg_margin / 100.0);
    double error_factor = std::max(0.0, 1.0 - reproj_error / 10.0);
    double distance_factor = std::max(0.0, 1.0 - estimated_distance / 5.0);

    quality.confidence = tag_factor * 0.3 + margin_factor * 0.2 +
                        error_factor * 0.3 + distance_factor * 0.2;

    // Estimate standard deviations
    auto std_devs = estimate_std_devs(quality.tag_count, estimated_distance, reproj_error);
    quality.std_dev_x = std_devs.std_dev_x;
    quality.std_dev_y = std_devs.std_dev_y;
    quality.std_dev_theta = std_devs.std_dev_theta;

    return quality;
}

PoseQuality PoseEstimator::estimate_std_devs(
    int tag_count,
    double avg_distance,
    double reproj_error)
{
    PoseQuality quality;
    quality.tag_count = tag_count;
    quality.avg_reproj_error = reproj_error;

    // Base standard deviations (at 1m with 1 tag, 1px error)
    constexpr double BASE_XY_STD = 0.05;   // 5cm
    constexpr double BASE_THETA_STD = 0.03; // ~2 degrees

    // Scale by number of tags (more tags = lower std dev)
    double tag_scale = 1.0 / std::sqrt(std::max(1, tag_count));

    // Scale by distance (further = higher std dev)
    double distance_scale = std::max(0.5, avg_distance);

    // Scale by reprojection error
    double error_scale = 1.0 + reproj_error * 0.1;

    quality.std_dev_x = BASE_XY_STD * tag_scale * distance_scale * error_scale;
    quality.std_dev_y = BASE_XY_STD * tag_scale * distance_scale * error_scale;
    quality.std_dev_theta = BASE_THETA_STD * tag_scale * error_scale;

    // Clamp to reasonable ranges
    quality.std_dev_x = std::clamp(quality.std_dev_x, 0.01, 1.0);
    quality.std_dev_y = std::clamp(quality.std_dev_y, 0.01, 1.0);
    quality.std_dev_theta = std::clamp(quality.std_dev_theta, 0.005, 0.5);

    return quality;
}

} // namespace frc_vision
