/**
 * @file fusion.cpp
 * @brief Multi-camera pose fusion implementation
 */

#include "fusion.hpp"
#include "detector.hpp"
#include "pose.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>

namespace frc_vision {

// =============================================================================
// PoseFusion
// =============================================================================

PoseFusion::PoseFusion() = default;

void PoseFusion::initialize(int num_cameras, double filter_alpha) {
    std::lock_guard<std::mutex> lock(mutex_);
    num_cameras_ = num_cameras;
    estimates_.resize(num_cameras);
    smoother_ = PoseSmoother(filter_alpha);
    fused_ = FusedPose();
}

void PoseFusion::update(const CameraPoseEstimate& estimate) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (estimate.camera_id < 0 || estimate.camera_id >= num_cameras_) {
        return;
    }

    // Innovation gating: if odometry says robot is far from where
    // vision thinks, reject the vision measurement. This prevents
    // bad detections from corrupting the pose estimate.
    if (estimate.valid && !passes_innovation_gate(estimate)) {
        // Mark as invalid - don't use this measurement
        CameraPoseEstimate rejected = estimate;
        rejected.valid = false;
        estimates_[estimate.camera_id] = rejected;
        return;
    }

    estimates_[estimate.camera_id] = estimate;
}

void PoseFusion::update_odometry(const OdometryData& odom) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_ = odom;
}

bool PoseFusion::passes_innovation_gate(const CameraPoseEstimate& estimate) const {
    // If no odometry available, accept all vision measurements
    if (!latest_odom_.valid) {
        return true;
    }

    // Reject vision during fast rotation - cameras produce blurry/bad data
    if (std::abs(latest_odom_.angular_velocity) > ANGULAR_VEL_REJECT) {
        return false;
    }

    // Check distance between vision pose and odometry pose.
    // If they disagree by more than INNOVATION_GATE_M, reject vision.
    // This catches false positive tag detections.
    double dx = estimate.pose.x - latest_odom_.pose.x;
    double dy = estimate.pose.y - latest_odom_.pose.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance > INNOVATION_GATE_M) {
        std::cerr << "[Fusion] Innovation gate rejected cam" << estimate.camera_id
                  << " vision: (" << estimate.pose.x << ", " << estimate.pose.y
                  << ") odom: (" << latest_odom_.pose.x << ", " << latest_odom_.pose.y
                  << ") dist: " << distance << "m" << std::endl;
        return false;
    }

    return true;
}

FusedPose PoseFusion::get_fused_pose() {
    std::lock_guard<std::mutex> lock(mutex_);
    compute_fusion();
    return fused_;
}

void PoseFusion::compute_fusion() {
    // Collect valid estimates
    std::vector<CameraPoseEstimate> valid_estimates;
    for (const auto& est : estimates_) {
        if (est.valid) {
            valid_estimates.push_back(est);
        }
    }

    if (valid_estimates.empty()) {
        // No valid poses - use prediction from smoother
        fused_.valid = false;
        fused_.pose_filtered = smoother_.update(fused_.pose_raw, false);
        return;
    }

    // Compute weights and weighted poses
    std::vector<std::pair<Pose2D, double>> weighted_poses;
    double total_weight = 0;
    int total_tags = 0;
    SteadyTimePoint latest_capture;

    for (const auto& est : valid_estimates) {
        double weight = compute_weight(est);
        weighted_poses.emplace_back(est.pose, weight);
        total_weight += weight;
        total_tags += est.tag_count;

        if (est.timestamps.capture > latest_capture) {
            latest_capture = est.timestamps.capture;
        }
    }

    // Compute weighted average pose
    fused_.pose_raw = weighted_average(weighted_poses);

    // Apply smoothing filter
    fused_.pose_filtered = smoother_.update(fused_.pose_raw, true);

    // Fuse quality metrics
    fused_.quality = fuse_quality(valid_estimates);
    fused_.quality.tag_count = total_tags;

    // Update metadata
    fused_.valid = true;
    fused_.cameras_contributing = static_cast<int>(valid_estimates.size());
    fused_.total_tags = total_tags;
    fused_.timestamps.capture = latest_capture;
    fused_.timestamps.publish = SteadyClock::now();
}

double PoseFusion::compute_weight(const CameraPoseEstimate& estimate) const {
    if (!estimate.valid || estimate.tag_count == 0) {
        return 0.0;
    }

    // Base weight from tag count
    double weight = 1.0 + (estimate.tag_count - 1) * tag_weight_;

    // Add margin contribution
    weight += estimate.quality.avg_margin * margin_weight_;

    // Penalize high reprojection error
    weight *= std::exp(-estimate.quality.avg_reproj_error * error_penalty_);

    // Factor in confidence
    weight *= estimate.quality.confidence;

    return std::max(0.01, weight);
}

Pose2D PoseFusion::weighted_average(
    const std::vector<std::pair<Pose2D, double>>& poses) const
{
    if (poses.empty()) {
        return Pose2D();
    }

    if (poses.size() == 1) {
        return poses[0].first;
    }

    double total_weight = 0;
    double sum_x = 0, sum_y = 0;
    double sum_cos = 0, sum_sin = 0;

    for (const auto& [pose, weight] : poses) {
        sum_x += pose.x * weight;
        sum_y += pose.y * weight;
        // Circular mean for angle
        sum_cos += std::cos(pose.theta) * weight;
        sum_sin += std::sin(pose.theta) * weight;
        total_weight += weight;
    }

    Pose2D result;
    if (total_weight > 0) {
        result.x = sum_x / total_weight;
        result.y = sum_y / total_weight;
        result.theta = std::atan2(sum_sin, sum_cos);
    }

    return result;
}

PoseQuality PoseFusion::fuse_quality(
    const std::vector<CameraPoseEstimate>& estimates) const
{
    PoseQuality fused;

    if (estimates.empty()) {
        return fused;
    }

    double sum_margin = 0;
    double sum_error = 0;
    double sum_confidence = 0;
    double min_std_x = std::numeric_limits<double>::max();
    double min_std_y = std::numeric_limits<double>::max();
    double min_std_theta = std::numeric_limits<double>::max();

    for (const auto& est : estimates) {
        sum_margin += est.quality.avg_margin;
        sum_error += est.quality.avg_reproj_error;
        sum_confidence += est.quality.confidence;
        fused.tag_count += est.tag_count;

        // Fused std dev is minimum (best camera wins for each axis)
        min_std_x = std::min(min_std_x, est.quality.std_dev_x);
        min_std_y = std::min(min_std_y, est.quality.std_dev_y);
        min_std_theta = std::min(min_std_theta, est.quality.std_dev_theta);
    }

    int n = static_cast<int>(estimates.size());
    fused.avg_margin = sum_margin / n;
    fused.avg_reproj_error = sum_error / n;
    fused.confidence = sum_confidence / n;

    // Multi-camera fusion improves accuracy
    double fusion_factor = 1.0 / std::sqrt(static_cast<double>(n));
    fused.std_dev_x = min_std_x * fusion_factor;
    fused.std_dev_y = min_std_y * fusion_factor;
    fused.std_dev_theta = min_std_theta * fusion_factor;

    return fused;
}

std::optional<CameraPoseEstimate> PoseFusion::get_camera_estimate(int camera_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (camera_id >= 0 && camera_id < static_cast<int>(estimates_.size())) {
        return estimates_[camera_id];
    }
    return std::nullopt;
}

std::vector<CameraPoseEstimate> PoseFusion::get_all_estimates() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return estimates_;
}

void PoseFusion::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& est : estimates_) {
        est = CameraPoseEstimate();
    }
    fused_ = FusedPose();
    smoother_.reset();
}

void PoseFusion::set_filter_alpha(double alpha) {
    std::lock_guard<std::mutex> lock(mutex_);
    smoother_.set_alpha(alpha);
}

// =============================================================================
// VisionPipeline
// =============================================================================

VisionPipeline::VisionPipeline() = default;

void VisionPipeline::initialize(
    int num_cameras,
    const FieldLayout& field,
    const DetectorConfig& detector_config,
    const TrackerConfig& tracker_config,
    double filter_alpha)
{
    num_cameras_ = num_cameras;

    // Create detector and tracker per camera
    detectors_.clear();
    trackers_.clear();

    for (int i = 0; i < num_cameras; i++) {
        auto detector = std::make_unique<Detector>();
        detector->initialize(detector_config);
        detectors_.push_back(std::move(detector));

        auto tracker = std::make_unique<TagTracker>(i);
        tracker->initialize(tracker_config);
        trackers_.push_back(std::move(tracker));
    }

    // Create pose estimator
    pose_estimator_ = std::make_unique<PoseEstimator>();
    pose_estimator_->initialize(field);

    // Initialize fusion
    fusion_.initialize(num_cameras, filter_alpha);

    std::cout << "[VisionPipeline] Initialized with " << num_cameras
              << " cameras" << std::endl;
}

ProcessedFrame VisionPipeline::process_frame(
    const Frame& frame,
    const CameraIntrinsics& intrinsics,
    const Pose3D& camera_to_robot)
{
    ProcessedFrame result;
    result.camera_id = frame.camera_id;
    result.frame_number = frame.frame_number;
    result.timestamps.capture = frame.capture_time;
    result.timestamps.capture_wall = frame.capture_wall_time;

    if (frame.camera_id < 0 || frame.camera_id >= num_cameras_) {
        return result;
    }

    // Detection
    result.timestamps.detect_start = SteadyClock::now();
    auto detections = detectors_[frame.camera_id]->detect(frame.image);
    result.timestamps.detect_end = SteadyClock::now();

    // Tracking
    detections = trackers_[frame.camera_id]->update(detections, frame.capture_time);

    // Single-tag pose estimation for each detection
    for (auto& det : detections) {
        det = pose_estimator_->estimate_single_tag(det, intrinsics);
    }

    // Multi-tag pose estimation
    auto multi_result = pose_estimator_->estimate_multi_tag(
        detections, intrinsics, camera_to_robot);

    result.timestamps.pose_end = SteadyClock::now();

    // Fill in frame detections
    result.detections.camera_id = frame.camera_id;
    result.detections.frame_number = frame.frame_number;
    result.detections.timestamps = result.timestamps;
    result.detections.detections = detections;
    result.detections.multi_tag_pose_valid = multi_result.valid;
    result.detections.camera_to_field = multi_result.camera_to_field;
    result.detections.robot_pose_field = multi_result.robot_pose_field;
    result.detections.multi_tag_reproj_error = multi_result.reprojection_error;
    result.detections.tags_used_for_pose = multi_result.tags_used;

    // Update fusion
    CameraPoseEstimate estimate;
    estimate.camera_id = frame.camera_id;
    estimate.valid = multi_result.valid;
    estimate.pose = multi_result.robot_pose_field;
    estimate.tag_count = multi_result.tags_used;
    estimate.timestamps = result.timestamps;
    estimate.quality = pose_estimator_->compute_quality(detections, multi_result.reprojection_error, intrinsics);
    result.detections.quality = estimate.quality;
    fusion_.update(estimate);

    // Annotate frame for web streaming
    result.annotated_image = frame.image.clone();
    annotate_frame(result.annotated_image, result.detections);

    result.timestamps.publish = SteadyClock::now();

    return result;
}

void VisionPipeline::annotate_frame(cv::Mat& image, const FrameDetections& detections) {
    // Colors
    const cv::Scalar COLOR_CORNER = cv::Scalar(0, 255, 0);    // Green
    const cv::Scalar COLOR_EDGE = cv::Scalar(255, 255, 0);    // Cyan
    const cv::Scalar COLOR_CENTER = cv::Scalar(0, 0, 255);    // Red
    const cv::Scalar COLOR_TEXT = cv::Scalar(255, 255, 255);  // White
    const cv::Scalar COLOR_POSE = cv::Scalar(255, 0, 255);    // Magenta
    const cv::Scalar COLOR_NO_DETECT = cv::Scalar(0, 165, 255);  // Orange

    // Always show camera ID and detection count (helps verify stream is working)
    std::string cam_str = "CAM" + std::to_string(detections.camera_id);
    cv::putText(image, cam_str, cv::Point(10, image.rows - 15),
               cv::FONT_HERSHEY_SIMPLEX, 0.7, COLOR_TEXT, 2);

    // Show detection count
    int num_dets = static_cast<int>(detections.detections.size());
    std::string det_str = "Tags: " + std::to_string(num_dets);
    cv::Scalar det_color = num_dets > 0 ? COLOR_CORNER : COLOR_NO_DETECT;
    cv::putText(image, det_str, cv::Point(image.cols - 100, image.rows - 15),
               cv::FONT_HERSHEY_SIMPLEX, 0.6, det_color, 2);

    // If no detections, show helpful message
    if (num_dets == 0) {
        cv::putText(image, "No AprilTags detected", cv::Point(10, 25),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, COLOR_NO_DETECT, 2);
    }

    for (const auto& det : detections.detections) {
        // Draw corners
        for (int i = 0; i < 4; i++) {
            cv::Point2d p1(det.corners.corners[i].x, det.corners.corners[i].y);
            cv::Point2d p2(det.corners.corners[(i + 1) % 4].x,
                          det.corners.corners[(i + 1) % 4].y);

            cv::circle(image, p1, 4, COLOR_CORNER, -1);
            cv::line(image, p1, p2, COLOR_EDGE, 2);
        }

        // Draw center
        auto center = det.corners.center();
        cv::circle(image, cv::Point2d(center.x, center.y), 3, COLOR_CENTER, -1);

        // Draw ID and margin
        std::string label = std::to_string(det.id);
        cv::putText(image, label,
                   cv::Point(center.x + 10, center.y - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, COLOR_TEXT, 2);

        // Draw margin and distance
        std::string margin_str = "M:" + std::to_string(static_cast<int>(det.decision_margin));
        cv::putText(image, margin_str,
                   cv::Point(center.x + 10, center.y + 15),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, COLOR_TEXT, 1);

        // Distance on screen
        if (det.distance_m > 0.01) {
            std::string dist_str = std::to_string(det.distance_m).substr(0, 4) + "m";
            cv::putText(image, dist_str,
                       cv::Point(center.x + 10, center.y + 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, COLOR_CORNER, 1);
        }
    }

    // Draw robot pose if valid
    if (detections.multi_tag_pose_valid) {
        // Draw background rectangle for better visibility
        cv::rectangle(image, cv::Point(5, 5), cv::Point(350, 55), cv::Scalar(0, 0, 0), -1);

        std::string pose_str = "Robot: (" +
            std::to_string(detections.robot_pose_field.x).substr(0, 5) + ", " +
            std::to_string(detections.robot_pose_field.y).substr(0, 5) + ", " +
            std::to_string(detections.robot_pose_field.theta * 180.0 / M_PI).substr(0, 5) + " deg)";
        cv::putText(image, pose_str, cv::Point(10, 25),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, COLOR_POSE, 2);

        std::string tags_str = "Using " + std::to_string(detections.tags_used_for_pose) +
            " tags | Err: " + std::to_string(detections.multi_tag_reproj_error).substr(0, 4) + "px";
        cv::putText(image, tags_str, cv::Point(10, 48),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_CORNER, 1);
    }

    // Draw latency and FPS info in top right
    double latency = detections.timestamps.total_pipeline_ms();
    std::string latency_str = std::to_string(static_cast<int>(latency)) + "ms";
    cv::putText(image, latency_str, cv::Point(image.cols - 60, 25),
               cv::FONT_HERSHEY_SIMPLEX, 0.6, COLOR_TEXT, 2);
}

FusedPose VisionPipeline::get_fused_pose() {
    return fusion_.get_fused_pose();
}

TagTracker* VisionPipeline::get_tracker(int camera_id) {
    if (camera_id >= 0 && camera_id < static_cast<int>(trackers_.size())) {
        return trackers_[camera_id].get();
    }
    return nullptr;
}

void VisionPipeline::update_config(
    const DetectorConfig& detector_config,
    const TrackerConfig& tracker_config)
{
    for (auto& detector : detectors_) {
        detector->update_config(detector_config);
    }
    for (auto& tracker : trackers_) {
        tracker->update_config(tracker_config);
    }
}

} // namespace frc_vision
