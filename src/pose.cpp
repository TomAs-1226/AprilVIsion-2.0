/**
 * @file pose.cpp
 * @brief Pose estimation — rewritten with proven PhotonVision approach
 *
 * Key changes from previous version:
 * 1. Distance = PnP translation norm ONLY (no multi-method fusion noise)
 * 2. Coordinate transforms use clean 4x4 homogeneous matrices
 * 3. camera_to_robot uses simple 2D offset (config extrinsics are FRC coords)
 * 4. No broken double-conversion of coordinate systems
 * 5. Multi-tag uses field-frame object points → PnP → direct field pose
 */

#include "pose.hpp"
#include "pose_utils.hpp"
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <cmath>
#include <set>

namespace frc_vision {

PoseEstimator::PoseEstimator() = default;

void PoseEstimator::initialize(const FieldLayout& layout) {
    field_ = layout;

    auto corners = get_tag_corners_local(field_.tag_size_m);
    tag_corners_local_.clear();
    for (const auto& c : corners) {
        tag_corners_local_.push_back(c.to_cv());
    }

    std::cout << "[PoseEstimator] Initialized with " << field_.tags.size()
              << " field tags, size=" << field_.tag_size_m << "m" << std::endl;
}

// =============================================================================
// Single-tag pose estimation (PhotonVision IPPE approach)
// =============================================================================

TagDetection PoseEstimator::estimate_single_tag(
    const TagDetection& detection,
    const CameraIntrinsics& intrinsics)
{
    TagDetection result = detection;

    if (!intrinsics.valid() || tag_corners_local_.empty()) {
        result.pose_valid = false;
        return result;
    }

    // Build image points from detected corners
    std::vector<cv::Point2d> image_points;
    for (const auto& corner : detection.corners.corners) {
        image_points.push_back(corner.to_cv());
    }

    // IPPE_SQUARE: proven method for planar square targets (PhotonVision uses this)
    // Returns exactly 2 solutions - the ambiguity ratio tells us confidence
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

    auto mat_to_vec3d = [](const cv::Mat& m) -> cv::Vec3d {
        return cv::Vec3d(m.at<double>(0), m.at<double>(1), m.at<double>(2));
    };

    cv::Vec3d best_rvec = mat_to_vec3d(rvecs[0]);
    cv::Vec3d best_tvec = mat_to_vec3d(tvecs[0]);
    double best_err = calculate_reprojection_error(
        image_points, tag_corners_local_, best_rvec, best_tvec, intrinsics);

    // Compute ambiguity from both IPPE solutions
    double alt_err = std::numeric_limits<double>::max();
    if (num_solutions >= 2) {
        cv::Vec3d alt_rvec = mat_to_vec3d(rvecs[1]);
        cv::Vec3d alt_tvec = mat_to_vec3d(tvecs[1]);
        alt_err = calculate_reprojection_error(
            image_points, tag_corners_local_, alt_rvec, alt_tvec, intrinsics);

        if (alt_err < best_err) {
            std::swap(best_rvec, alt_rvec);
            std::swap(best_tvec, alt_tvec);
            std::swap(best_err, alt_err);
        }

        // Ambiguity = best / alt (PhotonVision style)
        // Close to 0 = unambiguous, close to 1 = very ambiguous
        result.ambiguity = (alt_err > 1e-6) ? (best_err / alt_err) : 0.0;
    } else {
        result.ambiguity = 0.0;
    }

    // Refine best solution with iterative PnP (proven: PhotonVision does this too)
    cv::solvePnP(
        tag_corners_local_,
        image_points,
        intrinsics.camera_matrix,
        intrinsics.dist_coeffs,
        best_rvec, best_tvec,
        true,  // useExtrinsicGuess
        cv::SOLVEPNP_ITERATIVE
    );

    // Final reprojection error after refinement
    result.reprojection_error = calculate_reprojection_error(
        image_points, tag_corners_local_, best_rvec, best_tvec, intrinsics);

    // Store the tag_to_camera transform
    result.tag_to_camera = Pose3D::from_rvec_tvec(best_rvec, best_tvec);
    result.pose_valid = true;

    // DISTANCE: PnP translation vector norm — the ONLY proven method
    // This is what PhotonVision, Limelight, and all accurate FRC vision systems use.
    // The translation vector directly encodes 3D camera-to-tag distance.
    result.distance_m = cv::norm(best_tvec);

    // Diagnostic distance estimate (for display only, not used in pipeline)
    result.distance_estimate = compute_distance_estimate(result, intrinsics);

    // Per-tag accuracy estimate
    result.accuracy_estimate = estimate_tag_accuracy(result, result.distance_m);

    // Sanity checks: reject unreasonable results
    if (result.distance_m > 10.0 || result.distance_m < 0.05 ||
        result.reprojection_error > 10.0) {
        result.pose_valid = false;
    }

    return result;
}

// =============================================================================
// Multi-tag pose estimation (PhotonVision MegaTag approach)
// =============================================================================

PoseEstimator::MultiTagResult PoseEstimator::estimate_multi_tag(
    const std::vector<TagDetection>& detections,
    const CameraIntrinsics& intrinsics,
    const Pose3D& camera_to_robot)
{
    MultiTagResult result;

    if (!intrinsics.valid()) {
        return result;
    }

    // Collect ALL corners from ALL known tags with their field positions
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

    if (tag_ids.empty()) {
        return result;
    }

    // Extract camera extrinsic offsets from the Pose3D config (FRC robot body coords)
    // Config specifies x=forward, y=left, z=up in the ROBOT body frame.
    // These are NOT in OpenCV coords — do NOT convert them.
    double cam_x_robot = camera_to_robot.position.x;
    double cam_y_robot = camera_to_robot.position.y;
    // Yaw offset: extract from the rotation part of camera_to_robot
    double cam_yaw_robot = 0.0;
    {
        cv::Vec3d cam_rvec = camera_to_robot.rvec();
        double rvec_norm = cv::norm(cam_rvec);
        if (rvec_norm > 1e-6) {
            cv::Mat R_cam;
            cv::Rodrigues(cam_rvec, R_cam);
            cam_yaw_robot = std::atan2(R_cam.at<double>(1, 0), R_cam.at<double>(0, 0));
        }
    }

    if (tag_ids.size() == 1) {
        // ===== SINGLE TAG: use IPPE + field layout composition =====
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

        // PhotonVision ambiguity filter: reject if both IPPE solutions are equally good
        if (det_with_pose.ambiguity > MAX_SINGLE_TAG_AMBIGUITY) {
            return result;
        }

        // PROVEN 4x4 HOMOGENEOUS TRANSFORM APPROACH:
        // tag_to_camera from PnP × tag_to_field from layout = camera_to_field
        // This correctly handles mixed coordinate systems (OpenCV camera + FRC field)
        // because the intermediate tag-local frame cancels out.
        const auto& field_tag = field_.get_tag(det_with_pose.id);

        Pose2D camera_field_pose = pose_utils::single_tag_camera_field_pose(
            det_with_pose.tag_to_camera.rvec(),
            det_with_pose.tag_to_camera.tvec(),
            field_tag.pose_field);

        // Convert camera pose to robot pose using config extrinsics (simple 2D offset)
        result.robot_pose_field = pose_utils::camera_to_robot_field_pose(
            camera_field_pose, cam_x_robot, cam_y_robot, cam_yaw_robot);

        result.reprojection_error = det_with_pose.reprojection_error;
        result.tags_used = 1;
        result.inlier_tag_ids.push_back(det_with_pose.id);
        result.valid = true;

    } else {
        // ===== MULTI-TAG: all corners as rigid constellation =====
        // Object points are in FRC field coords, so PnP gives field→camera(OpenCV)
        cv::Vec3d rvec, tvec;
        cv::Mat inliers;

        // RANSAC with SQPNP (proven robust for 8+ points)
        bool success = cv::solvePnPRansac(
            object_points,
            image_points,
            intrinsics.camera_matrix,
            intrinsics.dist_coeffs,
            rvec, tvec,
            false,
            500,    // iterations
            2.0,    // reprojection threshold (pixels)
            0.99,   // confidence
            inliers,
            cv::SOLVEPNP_SQPNP
        );

        if (!success) {
            // Fallback to regular PnP without RANSAC
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

        // Refine with inliers only (proven: reduces outlier influence)
        if (!inliers.empty() && inliers.rows > 4) {
            std::vector<cv::Point3d> inlier_obj;
            std::vector<cv::Point2d> inlier_img;
            for (int i = 0; i < inliers.rows; i++) {
                int idx = inliers.at<int>(i);
                if (idx >= 0 && idx < static_cast<int>(object_points.size())) {
                    inlier_obj.push_back(object_points[idx]);
                    inlier_img.push_back(image_points[idx]);
                }
            }
            if (inlier_obj.size() >= 4) {
                cv::solvePnP(
                    inlier_obj, inlier_img,
                    intrinsics.camera_matrix, intrinsics.dist_coeffs,
                    rvec, tvec,
                    true,
                    cv::SOLVEPNP_ITERATIVE
                );
            }
        } else {
            // Refine with all points
            cv::solvePnP(
                object_points, image_points,
                intrinsics.camera_matrix, intrinsics.dist_coeffs,
                rvec, tvec,
                true,
                cv::SOLVEPNP_ITERATIVE
            );
        }

        // PROVEN APPROACH: PnP with field-frame object points gives
        // field→camera(OpenCV). Extract camera field pose directly
        // using -R^T*t for position and R's third row for heading.
        Pose2D camera_field_pose = pose_utils::extract_camera_field_pose(rvec, tvec);

        // Convert camera pose to robot pose
        result.robot_pose_field = pose_utils::camera_to_robot_field_pose(
            camera_field_pose, cam_x_robot, cam_y_robot, cam_yaw_robot);

        // Reprojection error
        result.reprojection_error = calculate_reprojection_error(
            image_points, object_points, rvec, tvec, intrinsics);

        // Count inlier tags
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

    // Store camera_to_field for backward compatibility with NT/web
    if (result.valid) {
        result.camera_to_field = Pose3D::from_rvec_tvec(
            cv::Vec3d(0, 0, result.robot_pose_field.theta),
            cv::Vec3d(result.robot_pose_field.x,
                      result.robot_pose_field.y,
                      0));
    }

    return result;
}

// =============================================================================
// camera_to_robot_pose: proven 2D approach (no broken 3D conversion)
// =============================================================================

Pose2D PoseEstimator::camera_to_robot_pose(
    const Pose3D& camera_to_field,
    const Pose3D& camera_to_robot)
{
    // FIXED: The old code called opencv_camera_to_frc_field() on the
    // camera_to_robot config transform, which is WRONG because config
    // extrinsics are already in FRC robot body coords, not OpenCV coords.
    // That double-conversion was the root cause of inaccurate angles/distances.
    //
    // Now uses simple 2D offset which is what PhotonVision does.
    double cam_x = camera_to_robot.position.x;
    double cam_y = camera_to_robot.position.y;

    double cam_yaw = 0.0;
    cv::Vec3d rvec = camera_to_robot.rvec();
    if (cv::norm(rvec) > 1e-6) {
        cv::Mat R_cam;
        cv::Rodrigues(rvec, R_cam);
        cam_yaw = std::atan2(R_cam.at<double>(1, 0), R_cam.at<double>(0, 0));
    }

    Pose2D camera_field_pose;
    camera_field_pose.x = camera_to_field.position.x;
    camera_field_pose.y = camera_to_field.position.y;

    cv::Vec3d field_rvec = camera_to_field.rvec();
    if (cv::norm(field_rvec) > 1e-6) {
        cv::Mat R_field;
        cv::Rodrigues(field_rvec, R_field);
        camera_field_pose.theta = std::atan2(R_field.at<double>(1, 0), R_field.at<double>(0, 0));
    }

    return pose_utils::camera_to_robot_field_pose(camera_field_pose, cam_x, cam_y, cam_yaw);
}

// =============================================================================
// Reprojection error calculation (standard, unchanged)
// =============================================================================

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

// =============================================================================
// Distance estimation: PnP-only (proven) + diagnostics
// =============================================================================

double PoseEstimator::compute_tag_distance(
    const TagDetection& detection,
    const CameraIntrinsics& intrinsics)
{
    // If PnP pose is available, use it (proven accurate)
    if (detection.pose_valid && detection.distance_m > 0.05) {
        return detection.distance_m;
    }

    // Fallback: pinhole model for when PnP isn't available
    if (!intrinsics.valid()) {
        double area = detection.corners.area();
        if (area < 1.0) return 10.0;
        return (field_.tag_size_m * 500.0) / std::sqrt(area);
    }

    double focal = (intrinsics.fx + intrinsics.fy) / 2.0;
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

DistanceEstimate PoseEstimator::compute_distance_estimate(
    const TagDetection& detection,
    const CameraIntrinsics& intrinsics)
{
    DistanceEstimate est;

    if (!intrinsics.valid()) {
        double area = detection.corners.area();
        est.distance_fused = (area > 1.0) ?
            (field_.tag_size_m * 500.0) / std::sqrt(area) : 10.0;
        est.confidence = 0.3;
        return est;
    }

    const auto& c = detection.corners.corners;
    double focal = (intrinsics.fx + intrinsics.fy) / 2.0;

    // Method 1: PnP distance (authoritative)
    if (detection.pose_valid) {
        est.distance_pnp = std::sqrt(
            detection.tag_to_camera.position.x * detection.tag_to_camera.position.x +
            detection.tag_to_camera.position.y * detection.tag_to_camera.position.y +
            detection.tag_to_camera.position.z * detection.tag_to_camera.position.z);
    }

    // Method 2: Pinhole (diagnostic only)
    double edge_lengths[4];
    edge_lengths[0] = std::sqrt((c[1].x-c[0].x)*(c[1].x-c[0].x) + (c[1].y-c[0].y)*(c[1].y-c[0].y));
    edge_lengths[1] = std::sqrt((c[2].x-c[1].x)*(c[2].x-c[1].x) + (c[2].y-c[1].y)*(c[2].y-c[1].y));
    edge_lengths[2] = std::sqrt((c[3].x-c[2].x)*(c[3].x-c[2].x) + (c[3].y-c[2].y)*(c[3].y-c[2].y));
    edge_lengths[3] = std::sqrt((c[0].x-c[3].x)*(c[0].x-c[3].x) + (c[0].y-c[3].y)*(c[0].y-c[3].y));

    double avg_edge = (edge_lengths[0]+edge_lengths[1]+edge_lengths[2]+edge_lengths[3]) / 4.0;
    est.distance_pinhole = (avg_edge > 1.0) ? (field_.tag_size_m * focal) / avg_edge : 10.0;

    double vert_avg = (edge_lengths[1] + edge_lengths[3]) / 2.0;
    double horiz_avg = (edge_lengths[0] + edge_lengths[2]) / 2.0;
    est.distance_vertical_edges = (vert_avg > 1.0) ? (field_.tag_size_m * focal) / vert_avg : 10.0;
    est.distance_horizontal_edges = (horiz_avg > 1.0) ? (field_.tag_size_m * focal) / horiz_avg : 10.0;

    // Edge consistency (diagnostic)
    double vert_horiz_diff = std::abs(est.distance_vertical_edges - est.distance_horizontal_edges);
    est.edge_consistency = std::exp(-vert_horiz_diff / 0.3);

    // PnP vs pinhole agreement (diagnostic)
    if (detection.pose_valid && est.distance_pnp > 0.01) {
        double diff = std::abs(est.distance_pnp - est.distance_pinhole);
        est.pnp_pinhole_agreement = std::exp(-diff / 0.5);
    } else {
        est.pnp_pinhole_agreement = 0.5;
    }

    // PROVEN: PnP translation norm IS the distance. Period.
    est.distance_fused = (detection.pose_valid && est.distance_pnp > 0.05) ?
        est.distance_pnp : est.distance_pinhole;

    est.confidence = est.pnp_pinhole_agreement * 0.5 + est.edge_consistency * 0.3;
    if (est.pnp_pinhole_agreement > 0.8 && est.edge_consistency > 0.8) {
        est.confidence = std::min(1.0, est.confidence + 0.2);
    }
    est.is_consistent = (detection.pose_valid && est.confidence > 0.4);

    return est;
}

// =============================================================================
// Per-tag accuracy estimation
// =============================================================================

TagAccuracyEstimate PoseEstimator::estimate_tag_accuracy(
    const TagDetection& detection,
    double distance)
{
    TagAccuracyEstimate est;

    if (distance < 0.01 || distance > 20.0) {
        est.confidence_level = "low";
        est.estimated_error_m = 1.0;
        est.estimated_angle_error_deg = 45.0;
        return est;
    }

    // Quadratic distance error model (proven empirical model)
    est.base_error = 0.01 * distance * distance;
    est.reprojection_contribution = detection.reprojection_error * 0.005 * distance;

    // Viewing angle from edge ratio
    double viewing_angle_factor = 0.0;
    if (detection.corners.corners.size() >= 4) {
        const auto& c = detection.corners.corners;
        double e0 = std::sqrt((c[1].x-c[0].x)*(c[1].x-c[0].x) + (c[1].y-c[0].y)*(c[1].y-c[0].y));
        double e1 = std::sqrt((c[2].x-c[1].x)*(c[2].x-c[1].x) + (c[2].y-c[1].y)*(c[2].y-c[1].y));
        double e2 = std::sqrt((c[3].x-c[2].x)*(c[3].x-c[2].x) + (c[3].y-c[2].y)*(c[3].y-c[2].y));
        double e3 = std::sqrt((c[0].x-c[3].x)*(c[0].x-c[3].x) + (c[0].y-c[3].y)*(c[0].y-c[3].y));
        double va = (e1 + e3) / 2.0;
        double ha = (e0 + e2) / 2.0;
        if (va > 0.0 && ha > 0.0) {
            double aspect = std::max(va, ha) / std::min(va, ha);
            viewing_angle_factor = (aspect - 1.0) * 0.5;
        }
    }
    viewing_angle_factor = std::clamp(viewing_angle_factor, 0.0, 2.0);
    est.viewing_angle_contribution = viewing_angle_factor * 0.01 * distance;
    est.ambiguity_contribution = detection.ambiguity * 0.02 * distance;

    est.estimated_error_m = est.base_error + est.reprojection_contribution +
                            est.viewing_angle_contribution + est.ambiguity_contribution;
    est.estimated_error_m = std::clamp(est.estimated_error_m, 0.005, 2.0);

    est.estimated_angle_error_deg = (2.0 / distance) * (1.0 + detection.reprojection_error);
    est.estimated_angle_error_deg = std::clamp(est.estimated_angle_error_deg, 0.5, 45.0);

    if (est.estimated_error_m < 0.05 && detection.ambiguity < 0.1) {
        est.confidence_level = "high";
    } else if (est.estimated_error_m < 0.15 && detection.ambiguity < 0.3) {
        est.confidence_level = "medium";
    } else {
        est.confidence_level = "low";
    }

    return est;
}

// =============================================================================
// Quality metrics for WPILib pose estimator
// =============================================================================

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
        distance_sum += (det.pose_valid && det.distance_m > 0.01) ?
            det.distance_m : compute_tag_distance(det, intrinsics);
        max_ambiguity = std::max(max_ambiguity, det.ambiguity);
    }

    quality.avg_margin = margin_sum / detections.size();
    double avg_distance = distance_sum / detections.size();

    double tag_factor = std::min(1.0, quality.tag_count / 4.0);
    double margin_factor = std::min(1.0, quality.avg_margin / 100.0);
    double error_factor = std::max(0.0, 1.0 - reproj_error / 5.0);
    double distance_factor = std::clamp(1.0 - avg_distance / 8.0, 0.0, 1.0);
    double ambiguity_factor = std::clamp(1.0 - max_ambiguity, 0.0, 1.0);

    quality.confidence = tag_factor * 0.25 + margin_factor * 0.15 +
                        error_factor * 0.25 + distance_factor * 0.15 +
                        ambiguity_factor * 0.20;

    auto std_devs = estimate_std_devs(quality.tag_count, avg_distance, reproj_error);
    double ambiguity_inflation = 1.0 + max_ambiguity * 2.0;
    quality.std_dev_x = std_devs.std_dev_x * ambiguity_inflation;
    quality.std_dev_y = std_devs.std_dev_y * ambiguity_inflation;
    quality.std_dev_theta = std_devs.std_dev_theta * ambiguity_inflation;

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

    // PhotonVision-calibrated std dev model for WPILib SwerveDrivePoseEstimator
    double dist_sq = avg_distance * avg_distance;
    double tag_scale = 1.0 / std::sqrt(std::max(1, tag_count));
    double error_scale = 1.0 + reproj_error * 0.2;

    quality.std_dev_x = 0.02 * dist_sq * tag_scale * error_scale;
    quality.std_dev_y = 0.02 * dist_sq * tag_scale * error_scale;
    quality.std_dev_theta = 0.04 * tag_scale * error_scale;

    quality.std_dev_x = std::clamp(quality.std_dev_x, 0.01, 2.0);
    quality.std_dev_y = std::clamp(quality.std_dev_y, 0.01, 2.0);
    quality.std_dev_theta = std::clamp(quality.std_dev_theta, 0.01, 1.0);

    return quality;
}

} // namespace frc_vision
