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

    // Use solvePnPGeneric with IPPE_SQUARE to get BOTH solutions.
    // IPPE always returns exactly 2 solutions for a planar target.
    // The ambiguity ratio (best_err / alt_err) tells us how confident
    // we are in the pose - PhotonVision rejects ambiguity > 0.2.
    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat reprojErrors;
    int num_solutions = cv::solvePnPGeneric(
        tag_corners_local_,
        image_points,
        intrinsics.camera_matrix,
        intrinsics.dist_coeffs,
        rvecs, tvecs,
        false,
        cv::SOLVEPNP_IPPE_SQUARE,
        cv::noArray(), cv::noArray(),
        reprojErrors
    );

    if (num_solutions == 0) {
        result.pose_valid = false;
        return result;
    }

    // Pick best solution (lowest reprojection error)
    // solvePnPGeneric returns each rvec/tvec as a 3x1 CV_64F Mat
    auto mat_to_vec3d = [](const cv::Mat& m) -> cv::Vec3d {
        return cv::Vec3d(m.at<double>(0), m.at<double>(1), m.at<double>(2));
    };
    cv::Vec3d best_rvec = mat_to_vec3d(rvecs[0]);
    cv::Vec3d best_tvec = mat_to_vec3d(tvecs[0]);
    double best_err = calculate_reprojection_error(
        image_points, tag_corners_local_, best_rvec, best_tvec, intrinsics);

    // Compute ambiguity from both solutions
    double alt_err = std::numeric_limits<double>::max();
    if (num_solutions >= 2) {
        cv::Vec3d alt_rvec = mat_to_vec3d(rvecs[1]);
        cv::Vec3d alt_tvec = mat_to_vec3d(tvecs[1]);
        alt_err = calculate_reprojection_error(
            image_points, tag_corners_local_, alt_rvec, alt_tvec, intrinsics);

        // If alternate solution is actually better, swap
        if (alt_err < best_err) {
            std::swap(best_rvec, alt_rvec);
            std::swap(best_tvec, alt_tvec);
            std::swap(best_err, alt_err);
        }

        // Ambiguity = best / alt. Close to 1.0 = very ambiguous (both solutions equally good).
        // Close to 0.0 = unambiguous (one solution clearly better).
        result.ambiguity = (alt_err > 1e-6) ? (best_err / alt_err) : 0.0;
    } else {
        result.ambiguity = 0.0;  // Only one solution = unambiguous
    }

    // Refine the best IPPE solution with iterative PnP (like PhotonVision does).
    // This converges to a more precise local minimum.
    cv::solvePnP(
        tag_corners_local_,
        image_points,
        intrinsics.camera_matrix,
        intrinsics.dist_coeffs,
        best_rvec, best_tvec,
        true,  // useExtrinsicGuess = true (refine from IPPE result)
        cv::SOLVEPNP_ITERATIVE
    );

    // Recalculate reprojection error after refinement
    result.reprojection_error = calculate_reprojection_error(
        image_points, tag_corners_local_, best_rvec, best_tvec, intrinsics);

    // Store pose (tag frame to camera frame)
    result.tag_to_camera = Pose3D::from_rvec_tvec(best_rvec, best_tvec);
    result.pose_valid = true;

    // Distance from PnP solution (more accurate than pinhole approximation)
    result.distance_m = cv::norm(best_tvec);

    // Sanity check: reject unreasonable distances (> 10m for FRC field)
    // and unreasonable reprojection errors
    if (result.distance_m > 10.0 || result.distance_m < 0.05 ||
        result.reprojection_error > 10.0) {
        result.pose_valid = false;
    }

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
        // Single tag - use single tag estimation with field transform.
        // Find the detection that matched a known field tag.
        const TagDetection* known_det = nullptr;
        for (const auto& det : detections) {
            if (field_.has_tag(det.id)) {
                known_det = &det;
                break;
            }
        }
        if (!known_det) return result;

        auto det_with_pose = estimate_single_tag(*known_det, intrinsics);
        if (!det_with_pose.pose_valid) {
            return result;
        }

        // Ambiguity filter (PhotonVision approach):
        // Reject single-tag poses where the two IPPE solutions are too similar.
        // High ambiguity = both solutions are equally valid = pose is unreliable.
        // Threshold of 0.2 from PhotonVision (lower = stricter).
        if (det_with_pose.ambiguity > MAX_SINGLE_TAG_AMBIGUITY) {
            return result;
        }

        // Transform: camera_to_field = tag_to_field * camera_to_tag
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
            200,    // iterations (more for better convergence)
            3.0,    // reprojection threshold (tighter for competition accuracy)
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

        // Refine with iterative PnP using the RANSAC result as initial guess
        cv::solvePnP(
            object_points,
            image_points,
            intrinsics.camera_matrix,
            intrinsics.dist_coeffs,
            rvec, tvec,
            true,  // use extrinsic guess from RANSAC
            cv::SOLVEPNP_ITERATIVE
        );

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

double PoseEstimator::compute_tag_distance(
    const TagDetection& detection,
    const CameraIntrinsics& intrinsics)
{
    // Pure vision distance using pinhole camera model:
    // distance = (real_tag_size * focal_length) / pixel_tag_size
    //
    // pixel_tag_size = average of the 4 edge lengths of the detected tag
    // corners in the image. This is more robust than using area because
    // it handles perspective foreshortening better.

    if (!intrinsics.valid()) {
        // Fallback: rough estimate from pixel area
        double area = detection.corners.area();
        if (area < 1.0) return 10.0;
        return (field_.tag_size_m * 500.0) / std::sqrt(area);
    }

    // Average focal length (fx and fy should be close for square pixels)
    double focal = (intrinsics.fx + intrinsics.fy) / 2.0;

    // Compute average edge length in pixels
    const auto& c = detection.corners.corners;
    double edge_sum = 0;
    for (int i = 0; i < 4; i++) {
        int j = (i + 1) % 4;
        double dx = c[j].x - c[i].x;
        double dy = c[j].y - c[i].y;
        edge_sum += std::sqrt(dx * dx + dy * dy);
    }
    double avg_pixel_size = edge_sum / 4.0;

    if (avg_pixel_size < 1.0) return 10.0;

    return (field_.tag_size_m * focal) / avg_pixel_size;
}

PoseQuality PoseEstimator::compute_quality(
    const std::vector<TagDetection>& detections,
    double reproj_error,
    const CameraIntrinsics& intrinsics)
{
    PoseQuality quality;
    quality.tag_count = static_cast<int>(detections.size());
    quality.avg_reproj_error = reproj_error;

    if (detections.empty()) {
        return quality;
    }

    double margin_sum = 0;
    double distance_sum = 0;
    double max_ambiguity = 0;
    for (const auto& det : detections) {
        margin_sum += det.decision_margin;
        // Use PnP distance if available, fallback to pinhole
        if (det.pose_valid && det.distance_m > 0.01) {
            distance_sum += det.distance_m;
        } else {
            distance_sum += compute_tag_distance(det, intrinsics);
        }
        max_ambiguity = std::max(max_ambiguity, det.ambiguity);
    }

    quality.avg_margin = margin_sum / detections.size();
    double avg_distance = distance_sum / detections.size();

    // Compute confidence (0-1)
    // More tags, better margins, lower reproj error, closer distance, low ambiguity = higher confidence
    double tag_factor = std::min(1.0, quality.tag_count / 4.0);
    double margin_factor = std::min(1.0, quality.avg_margin / 100.0);
    double error_factor = std::max(0.0, 1.0 - reproj_error / 5.0);
    double distance_factor = std::clamp(1.0 - avg_distance / 8.0, 0.0, 1.0);
    // Ambiguity factor: 1.0 if unambiguous, drops toward 0 if highly ambiguous
    double ambiguity_factor = std::clamp(1.0 - max_ambiguity, 0.0, 1.0);

    quality.confidence = tag_factor * 0.25 + margin_factor * 0.15 +
                        error_factor * 0.25 + distance_factor * 0.15 +
                        ambiguity_factor * 0.20;

    // Estimate standard deviations for the RoboRIO pose estimator.
    // These directly control how much the robot trusts vision vs wheel odometry.
    auto std_devs = estimate_std_devs(quality.tag_count, avg_distance, reproj_error);

    // Inflate std devs for ambiguous single-tag detections
    double ambiguity_inflation = 1.0 + max_ambiguity * 2.0;
    quality.std_dev_x = std_devs.std_dev_x * ambiguity_inflation;
    quality.std_dev_y = std_devs.std_dev_y * ambiguity_inflation;
    quality.std_dev_theta = std_devs.std_dev_theta * ambiguity_inflation;

    // Clamp to useful range
    quality.std_dev_x = std::clamp(quality.std_dev_x, 0.01, 3.0);
    quality.std_dev_y = std::clamp(quality.std_dev_y, 0.01, 3.0);
    quality.std_dev_theta = std::clamp(quality.std_dev_theta, 0.01, 1.5);

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

    // Standard deviations for WPILib SwerveDrivePoseEstimator.
    // These values tell the Kalman filter how much to trust vision vs odometry.
    //
    // WPILib default odometry std devs: (0.1, 0.1, 0.1) [m, m, rad]
    // We want vision to beat odometry when close with multiple tags,
    // and be worse than odometry when far away with poor detections.
    //
    // Empirical model calibrated for 640x480 cameras at typical FRC distances:
    //   std_xy  = 0.02 * distance^2 / sqrt(tag_count) * (1 + reproj_err * 0.2)
    //   std_theta = 0.04 / sqrt(tag_count) * (1 + reproj_err * 0.2)

    double dist_sq = avg_distance * avg_distance;
    double tag_scale = 1.0 / std::sqrt(std::max(1, tag_count));
    double error_scale = 1.0 + reproj_error * 0.2;

    // XY standard deviation scales with distance squared (pinhole geometry)
    quality.std_dev_x = 0.02 * dist_sq * tag_scale * error_scale;
    quality.std_dev_y = 0.02 * dist_sq * tag_scale * error_scale;

    // Theta standard deviation - less dependent on distance
    quality.std_dev_theta = 0.04 * tag_scale * error_scale;

    // Clamp to useful range for the pose estimator
    // At minimum (close, multi-tag): ~0.01m = very trusted
    // At maximum (far, single tag): ~2.0m = mostly ignored vs odometry
    quality.std_dev_x = std::clamp(quality.std_dev_x, 0.01, 2.0);
    quality.std_dev_y = std::clamp(quality.std_dev_y, 0.01, 2.0);
    quality.std_dev_theta = std::clamp(quality.std_dev_theta, 0.01, 1.0);

    return quality;
}

} // namespace frc_vision
