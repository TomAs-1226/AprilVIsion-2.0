#pragma once
/**
 * @file fusion.hpp
 * @brief Multi-camera pose fusion
 *
 * Fuses pose estimates from multiple cameras using weighted averaging
 * based on detection quality, tag count, and reprojection error.
 */

#include "types.hpp"
#include "tracker.hpp"
#include "detector.hpp"
#include "pose.hpp"
#include <vector>
#include <mutex>
#include <memory>

namespace frc_vision {

/**
 * @brief Per-camera pose estimate for fusion
 */
struct CameraPoseEstimate {
    int camera_id = -1;
    bool valid = false;
    Pose2D pose;
    PoseQuality quality;
    Timestamps timestamps;
    int tag_count = 0;
};

/**
 * @brief Multi-camera pose fusion
 */
class PoseFusion {
public:
    PoseFusion();

    /**
     * @brief Initialize fusion with number of cameras
     * @param num_cameras Number of cameras to fuse
     * @param filter_alpha Smoothing factor for output filter
     */
    void initialize(int num_cameras, double filter_alpha = 0.3);

    /**
     * @brief Update with new pose estimate from a camera
     * @param estimate Camera pose estimate
     */
    void update(const CameraPoseEstimate& estimate);

    /**
     * @brief Get fused pose (call after updating all cameras for a frame)
     * @return Fused pose with quality metrics
     */
    FusedPose get_fused_pose();

    /**
     * @brief Force recomputation of fused pose
     */
    void compute_fusion();

    /**
     * @brief Get latest estimate from a specific camera
     */
    std::optional<CameraPoseEstimate> get_camera_estimate(int camera_id) const;

    /**
     * @brief Get all camera estimates
     */
    std::vector<CameraPoseEstimate> get_all_estimates() const;

    /**
     * @brief Clear all estimates
     */
    void clear();

    /**
     * @brief Set filter alpha for pose smoothing
     */
    void set_filter_alpha(double alpha);

private:
    double compute_weight(const CameraPoseEstimate& estimate) const;
    Pose2D weighted_average(const std::vector<std::pair<Pose2D, double>>& poses) const;
    PoseQuality fuse_quality(const std::vector<CameraPoseEstimate>& estimates) const;

    int num_cameras_ = 0;
    std::vector<CameraPoseEstimate> estimates_;
    FusedPose fused_;
    PoseSmoother smoother_;
    mutable std::mutex mutex_;

    // Fusion parameters
    double tag_weight_ = 2.0;     // Weight multiplier per tag
    double margin_weight_ = 0.01; // Weight per margin unit
    double error_penalty_ = 0.1;  // Penalty per pixel reproj error
};

/**
 * @brief Pipeline processor that combines detection, tracking, pose, and fusion
 */
class VisionPipeline {
public:
    VisionPipeline();

    /**
     * @brief Initialize pipeline
     * @param num_cameras Number of cameras
     * @param field Field layout for pose estimation
     * @param detector_config Detector configuration
     * @param tracker_config Tracker configuration
     * @param filter_alpha Pose smoothing factor
     */
    void initialize(
        int num_cameras,
        const FieldLayout& field,
        const DetectorConfig& detector_config,
        const TrackerConfig& tracker_config,
        double filter_alpha = 0.3);

    /**
     * @brief Process frame from a camera
     * @param frame Input frame
     * @param intrinsics Camera intrinsics
     * @param camera_to_robot Camera-to-robot transform
     * @return Processed frame with detections and pose
     */
    ProcessedFrame process_frame(
        const Frame& frame,
        const CameraIntrinsics& intrinsics,
        const Pose3D& camera_to_robot);

    /**
     * @brief Get current fused pose
     */
    FusedPose get_fused_pose();

    /**
     * @brief Get tracker for a camera
     */
    TagTracker* get_tracker(int camera_id);

    /**
     * @brief Get fusion module
     */
    PoseFusion& fusion() { return fusion_; }

    /**
     * @brief Update configuration
     */
    void update_config(
        const DetectorConfig& detector_config,
        const TrackerConfig& tracker_config);

private:
    std::vector<std::unique_ptr<Detector>> detectors_;
    std::vector<std::unique_ptr<TagTracker>> trackers_;
    std::unique_ptr<PoseEstimator> pose_estimator_;
    PoseFusion fusion_;
    int num_cameras_ = 0;

    // For annotation
    void annotate_frame(cv::Mat& image, const FrameDetections& detections);
};

} // namespace frc_vision
