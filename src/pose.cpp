/**
 * @file pose.cpp
 * @brief Pose estimation implementation
 */

#include "pose.hpp"
#include "pose_utils.hpp"  // Phase 1/2: Enhanced coordinate transforms
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <cmath>
#include <set>
#include <deque>

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

    // Distance from PnP translation vector norm - this is the proven approach
    // used by PhotonVision, Limelight, and all accurate FRC vision systems.
    // The translation vector directly encodes camera-to-tag distance in 3D.
    // Do NOT override with pinhole/edge fusion which adds noise.
    result.distance_m = cv::norm(best_tvec);

    // Compute multi-method estimate for diagnostics display only (not used for distance_m)
    result.distance_estimate = compute_distance_estimate(result, intrinsics);

    // Phase 2: Estimate per-tag accuracy based on viewing conditions
    result.accuracy_estimate = estimate_tag_accuracy(result, result.distance_m);

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

    // Phase 1 MegaTag: Collect ALL corners from ALL tags as single constellation
    // with corner quality weighting based on detection margin
    std::vector<cv::Point3d> object_points;
    std::vector<cv::Point2d> image_points;
    std::vector<double> corner_weights;  // Phase 1: Quality-based weighting
    std::vector<int> tag_ids;

    for (const auto& det : detections) {
        if (!field_.has_tag(det.id)) {
            continue;
        }

        const auto& field_tag = field_.get_tag(det.id);
        auto corners_3d = field_tag.get_corners_cv();

        // Phase 1: Weight corners by detection quality (√(margin/100))
        // Higher decision margin = more confident detection = higher weight
        double corner_weight = std::sqrt(std::max(0.0, det.decision_margin) / 100.0);
        corner_weight = std::clamp(corner_weight, 0.1, 1.0);

        for (int i = 0; i < 4; i++) {
            object_points.push_back(corners_3d[i]);
            image_points.push_back(det.corners.corners[i].to_cv());
            corner_weights.push_back(corner_weight);
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

        // CRITICAL FIX: Convert camera_to_tag from OpenCV to FRC BEFORE composing!
        // field_tag.pose_field is in FRC coordinates, but camera_to_tag is in OpenCV coordinates
        // We MUST convert camera_to_tag to FRC first, otherwise angles are completely wrong
        const auto& field_tag = field_.get_tag(det_with_pose.id);
        Pose3D camera_to_tag_opencv = det_with_pose.tag_to_camera.inverse();

        // Convert camera_to_tag from OpenCV coords to FRC coords
        Pose3D camera_to_tag_frc = pose_utils::opencv_camera_to_frc_field(camera_to_tag_opencv);

        // NOW we can safely compose (both in FRC coords)
        result.camera_to_field = field_tag.pose_field.compose(camera_to_tag_frc);

        result.reprojection_error = det_with_pose.reprojection_error;
        result.tags_used = 1;
        result.inlier_tag_ids.push_back(det_with_pose.id);
        result.valid = true;

    } else {
        // Phase 1 MegaTag: Aggressive RANSAC treating all tags as single rigid constellation
        cv::Vec3d rvec, tvec;
        cv::Mat inliers;

        // Phase 1: More aggressive RANSAC for better outlier rejection
        bool success = cv::solvePnPRansac(
            object_points,
            image_points,
            intrinsics.camera_matrix,
            intrinsics.dist_coeffs,
            rvec, tvec,
            false,
            500,    // Phase 1: 500 iterations (up from 200) for robustness
            2.0,    // Phase 1: 2.0px threshold (down from 3.0) for competition accuracy
            0.99,   // 99% confidence
            inliers,
            cv::SOLVEPNP_SQPNP  // Robust for multiple points
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

        // Phase 1: Refine with INLIERS ONLY for maximum accuracy
        // Extract inlier corners based on RANSAC mask
        std::vector<cv::Point3d> inlier_object_points;
        std::vector<cv::Point2d> inlier_image_points;

        if (!inliers.empty() && inliers.rows > 4) {
            // Use only RANSAC inliers for refinement
            for (int i = 0; i < inliers.rows; i++) {
                int idx = inliers.at<int>(i);
                if (idx >= 0 && idx < static_cast<int>(object_points.size())) {
                    inlier_object_points.push_back(object_points[idx]);
                    inlier_image_points.push_back(image_points[idx]);
                }
            }

            // Refine with iterative PnP on inliers only
            if (inlier_object_points.size() >= 4) {
                cv::solvePnP(
                    inlier_object_points,
                    inlier_image_points,
                    intrinsics.camera_matrix,
                    intrinsics.dist_coeffs,
                    rvec, tvec,
                    true,  // use extrinsic guess from RANSAC
                    cv::SOLVEPNP_ITERATIVE
                );
            }
        } else {
            // If no inliers or too few, refine with all points
            cv::solvePnP(
                object_points,
                image_points,
                intrinsics.camera_matrix,
                intrinsics.dist_coeffs,
                rvec, tvec,
                true,
                cv::SOLVEPNP_ITERATIVE
            );
        }

        // CRITICAL FIX: PnP gives field_to_camera where camera is in OpenCV coords
        // The object_points are in FRC field coords, but PnP works with OpenCV camera
        // So field_to_camera represents: FRC_field -> OpenCV_camera
        Pose3D field_to_camera_opencv = Pose3D::from_rvec_tvec(rvec, tvec);
        Pose3D camera_to_field_opencv = field_to_camera_opencv.inverse();

        // Convert from OpenCV camera coords to FRC coords
        result.camera_to_field = pose_utils::opencv_camera_to_frc_field(camera_to_field_opencv);

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
    // CRITICAL FIX: camera_to_field is now in FRC coords (after earlier fix)
    // But camera_to_robot is in OpenCV coords (from camera extrinsics config)
    // We MUST convert camera_to_robot to FRC before composing!

    // Convert camera_to_robot from OpenCV to FRC coords
    Pose3D camera_to_robot_frc = pose_utils::opencv_camera_to_frc_field(camera_to_robot);

    // Now compute robot_to_camera in FRC coords
    Pose3D robot_to_camera_frc = camera_to_robot_frc.inverse();

    // Compose: robot_to_field = camera_to_field * robot_to_camera
    // Both are now in FRC coords, so composition is valid
    Pose3D robot_to_field_frc = camera_to_field.compose(robot_to_camera_frc);

    // Extract 2D pose (already in FRC coords)
    Pose2D result = pose_utils::extract_robot_pose_2d(robot_to_field_frc);

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

DistanceEstimate PoseEstimator::compute_distance_estimate(
    const TagDetection& detection,
    const CameraIntrinsics& intrinsics)
{
    DistanceEstimate est;

    if (!intrinsics.valid()) {
        // Fallback to simple area-based estimate
        double area = detection.corners.area();
        if (area > 1.0) {
            est.distance_fused = (field_.tag_size_m * 500.0) / std::sqrt(area);
        } else {
            est.distance_fused = 10.0;
        }
        est.confidence = 0.3;  // Low confidence for uncalibrated
        return est;
    }

    const auto& c = detection.corners.corners;
    double focal = (intrinsics.fx + intrinsics.fy) / 2.0;

    // Method 1: Distance from PnP (if pose is valid)
    if (detection.pose_valid) {
        est.distance_pnp = std::sqrt(
            detection.tag_to_camera.position.x * detection.tag_to_camera.position.x +
            detection.tag_to_camera.position.y * detection.tag_to_camera.position.y +
            detection.tag_to_camera.position.z * detection.tag_to_camera.position.z
        );
    }

    // Method 2: Pinhole model from average edge length
    double edge_lengths[4];
    edge_lengths[0] = std::sqrt((c[1].x - c[0].x) * (c[1].x - c[0].x) +
                                 (c[1].y - c[0].y) * (c[1].y - c[0].y)); // bottom
    edge_lengths[1] = std::sqrt((c[2].x - c[1].x) * (c[2].x - c[1].x) +
                                 (c[2].y - c[1].y) * (c[2].y - c[1].y)); // right
    edge_lengths[2] = std::sqrt((c[3].x - c[2].x) * (c[3].x - c[2].x) +
                                 (c[3].y - c[2].y) * (c[3].y - c[2].y)); // top
    edge_lengths[3] = std::sqrt((c[0].x - c[3].x) * (c[0].x - c[3].x) +
                                 (c[0].y - c[3].y) * (c[0].y - c[3].y)); // left

    double avg_edge = (edge_lengths[0] + edge_lengths[1] +
                       edge_lengths[2] + edge_lengths[3]) / 4.0;

    if (avg_edge > 1.0) {
        est.distance_pinhole = (field_.tag_size_m * focal) / avg_edge;
    } else {
        est.distance_pinhole = 10.0;
    }

    // Method 3: Vertical edges only (right + left)
    double vertical_avg = (edge_lengths[1] + edge_lengths[3]) / 2.0;
    if (vertical_avg > 1.0) {
        est.distance_vertical_edges = (field_.tag_size_m * focal) / vertical_avg;
    } else {
        est.distance_vertical_edges = 10.0;
    }

    // Method 4: Horizontal edges only (bottom + top)
    double horizontal_avg = (edge_lengths[0] + edge_lengths[2]) / 2.0;
    if (horizontal_avg > 1.0) {
        est.distance_horizontal_edges = (field_.tag_size_m * focal) / horizontal_avg;
    } else {
        est.distance_horizontal_edges = 10.0;
    }

    // Geometric consistency: vertical vs horizontal edges should agree
    // (unless viewing angle is extreme)
    double vert_horiz_diff = std::abs(est.distance_vertical_edges - est.distance_horizontal_edges);
    est.edge_consistency = std::exp(-vert_horiz_diff / 0.3);

    // Agreement between PnP and pinhole methods
    if (detection.pose_valid && est.distance_pnp > 0.01) {
        double pnp_pinhole_diff = std::abs(est.distance_pnp - est.distance_pinhole);
        est.pnp_pinhole_agreement = std::exp(-pnp_pinhole_diff / 0.5);
    } else {
        est.pnp_pinhole_agreement = 0.5;  // Neutral if no PnP
    }

    // PhotonVision approach: PnP translation norm IS the distance.
    // The fused value is just PnP (or pinhole fallback for diagnostics).
    // Do NOT blend with edge methods - it degrades accuracy.
    est.distance_fused = (detection.pose_valid && est.distance_pnp > 0.05) ?
        est.distance_pnp : (est.distance_pinhole > 0.05 ? est.distance_pinhole : 10.0);

    // Confidence: how well do the cross-validation methods agree with PnP
    est.confidence = est.pnp_pinhole_agreement * 0.5 + est.edge_consistency * 0.3;
    if (est.pnp_pinhole_agreement > 0.8 && est.edge_consistency > 0.8) {
        est.confidence = std::min(1.0, est.confidence + 0.2);
    }

    est.is_consistent = (detection.pose_valid && est.confidence > 0.4);

    return est;
}

TagAccuracyEstimate PoseEstimator::estimate_tag_accuracy(
    const TagDetection& detection,
    double distance)
{
    // Phase 2: Per-tag accuracy estimation based on viewing conditions
    TagAccuracyEstimate est;

    if (distance < 0.01 || distance > 20.0) {
        est.confidence_level = "low";
        est.estimated_error_m = 1.0;  // Very uncertain
        est.estimated_angle_error_deg = 45.0;
        return est;
    }

    // Base error model: error increases quadratically with distance
    // At 1m: ~1cm, at 2m: ~4cm, at 3m: ~9cm (without other factors)
    est.base_error = 0.01 * distance * distance;

    // Reprojection error contribution
    // Higher reprojection = less confident pose
    est.reprojection_contribution = detection.reprojection_error * 0.005 * distance;

    // Viewing angle contribution (estimate from edge ratio)
    double viewing_angle_factor = 0.0;
    if (detection.corners.corners.size() >= 4) {
        const auto& c = detection.corners.corners;

        // Compute edge lengths
        double edge0 = std::sqrt((c[1].x - c[0].x) * (c[1].x - c[0].x) +
                                 (c[1].y - c[0].y) * (c[1].y - c[0].y));
        double edge1 = std::sqrt((c[2].x - c[1].x) * (c[2].x - c[1].x) +
                                 (c[2].y - c[1].y) * (c[2].y - c[1].y));
        double edge2 = std::sqrt((c[3].x - c[2].x) * (c[3].x - c[2].x) +
                                 (c[3].y - c[2].y) * (c[3].y - c[2].y));
        double edge3 = std::sqrt((c[0].x - c[3].x) * (c[0].x - c[3].x) +
                                 (c[0].y - c[3].y) * (c[0].y - c[3].y));

        // Aspect ratio deviation from 1.0 indicates viewing angle
        double vert_avg = (edge1 + edge3) / 2.0;
        double horiz_avg = (edge0 + edge2) / 2.0;

        if (vert_avg > 0.0 && horiz_avg > 0.0) {
            double aspect_ratio = std::max(vert_avg, horiz_avg) / std::min(vert_avg, horiz_avg);
            // aspect_ratio of 1.0 = frontal view (good)
            // aspect_ratio of 2.0+ = oblique view (poor)
            viewing_angle_factor = (aspect_ratio - 1.0) * 0.5;  // Clamp later
        }
    }
    viewing_angle_factor = std::clamp(viewing_angle_factor, 0.0, 2.0);
    est.viewing_angle_contribution = viewing_angle_factor * 0.01 * distance;

    // Ambiguity contribution (from IPPE)
    est.ambiguity_contribution = detection.ambiguity * 0.02 * distance;

    // Total estimated error (meters)
    est.estimated_error_m = est.base_error +
                            est.reprojection_contribution +
                            est.viewing_angle_contribution +
                            est.ambiguity_contribution;

    // Clamp to reasonable range
    est.estimated_error_m = std::clamp(est.estimated_error_m, 0.005, 2.0);

    // Angle error estimation (increases with distance, reprojection error)
    // At close range (<1m) with good reprojection: ~1°
    // At far range (>3m) with poor reprojection: ~10°
    est.estimated_angle_error_deg = (2.0 / distance) * (1.0 + detection.reprojection_error);
    est.estimated_angle_error_deg = std::clamp(est.estimated_angle_error_deg, 0.5, 45.0);

    // Confidence level classification
    if (est.estimated_error_m < 0.05 && detection.ambiguity < 0.1) {
        est.confidence_level = "high";
    } else if (est.estimated_error_m < 0.15 && detection.ambiguity < 0.3) {
        est.confidence_level = "medium";
    } else {
        est.confidence_level = "low";
    }

    return est;
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
