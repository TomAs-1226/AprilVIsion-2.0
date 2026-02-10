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

    // Convert to grayscale ONCE (detector would do this anyway).
    // Passing gray directly avoids a redundant BGR→gray conversion inside detect().
    cv::Mat gray;
    if (frame.image.channels() > 1) {
        cv::cvtColor(frame.image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = frame.image;
    }

    // Detection on grayscale
    result.timestamps.detect_start = SteadyClock::now();
    auto detections = detectors_[frame.camera_id]->detect(gray);
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

    // NOTE: Do NOT clone or annotate here. The caller (main.cpp) handles
    // streaming at reduced resolution/rate to avoid the massive CPU cost
    // of cloning + annotating + encoding every full-res frame.
    // result.annotated_image is left empty.

    result.timestamps.publish = SteadyClock::now();

    return result;
}

void VisionPipeline::annotate_frame(cv::Mat& image, const FrameDetections& detections) {
    // Compact annotations designed for 320x240 stream frames.
    // Uses thin lines and small fonts to keep drawing fast.
    const cv::Scalar GREEN(0, 255, 0);
    const cv::Scalar CYAN(255, 255, 0);
    const cv::Scalar RED(0, 0, 255);
    const cv::Scalar WHITE(255, 255, 255);
    const cv::Scalar MAGENTA(255, 0, 255);
    const cv::Scalar ORANGE(0, 165, 255);

    int num_dets = static_cast<int>(detections.detections.size());

    // Camera ID + tag count (bottom of frame)
    std::string cam_str = "C" + std::to_string(detections.camera_id) + " " +
                          std::to_string(num_dets) + "t";
    cv::putText(image, cam_str, cv::Point(3, image.rows - 4),
               cv::FONT_HERSHEY_SIMPLEX, 0.35, WHITE, 1);

    for (const auto& det : detections.detections) {
        // Draw tag outline (thin lines only - skip corner circles for speed)
        for (int i = 0; i < 4; i++) {
            cv::Point p1(static_cast<int>(det.corners.corners[i].x),
                         static_cast<int>(det.corners.corners[i].y));
            cv::Point p2(static_cast<int>(det.corners.corners[(i + 1) % 4].x),
                         static_cast<int>(det.corners.corners[(i + 1) % 4].y));
            cv::line(image, p1, p2, GREEN, 1);
        }

        // ID + distance + ambiguity label
        auto center = det.corners.center();
        int cx = static_cast<int>(center.x);
        int cy = static_cast<int>(center.y);

        char label_buf[32];
        if (det.distance_m > 0.01) {
            std::snprintf(label_buf, sizeof(label_buf), "%d %.1fm a%.2f",
                         det.id, det.distance_m, det.ambiguity);
        } else {
            std::snprintf(label_buf, sizeof(label_buf), "%d", det.id);
        }
        // Color based on ambiguity: green=good, orange=ok, red=bad
        cv::Scalar label_color = (det.ambiguity < 0.2) ? GREEN :
                                 (det.ambiguity < 0.5) ? ORANGE : RED;
        cv::putText(image, label_buf, cv::Point(cx + 5, cy - 3),
                   cv::FONT_HERSHEY_SIMPLEX, 0.3, label_color, 1);
    }

    // Robot pose (top-left, compact)
    if (detections.multi_tag_pose_valid) {
        cv::rectangle(image, cv::Point(0, 0), cv::Point(180, 24), cv::Scalar(0, 0, 0), -1);

        char buf[64];
        std::snprintf(buf, sizeof(buf), "(%.2f,%.2f,%.0f) %dt",
            detections.robot_pose_field.x, detections.robot_pose_field.y,
            detections.robot_pose_field.theta * 180.0 / M_PI,
            detections.tags_used_for_pose);
        cv::putText(image, buf, cv::Point(2, 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.3, MAGENTA, 1);

        char buf2[32];
        std::snprintf(buf2, sizeof(buf2), "err:%.1fpx", detections.multi_tag_reproj_error);
        cv::putText(image, buf2, cv::Point(2, 20),
                   cv::FONT_HERSHEY_SIMPLEX, 0.28, GREEN, 1);
    }

    // Latency (top-right)
    double latency = detections.timestamps.total_pipeline_ms();
    char lat_buf[16];
    std::snprintf(lat_buf, sizeof(lat_buf), "%dms", static_cast<int>(latency));
    cv::putText(image, lat_buf, cv::Point(image.cols - 35, 10),
               cv::FONT_HERSHEY_SIMPLEX, 0.3, WHITE, 1);
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
