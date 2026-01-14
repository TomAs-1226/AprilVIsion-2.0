#pragma once
/**
 * @file tracker.hpp
 * @brief Tag tracking for fast motion with prediction and ROI guidance
 *
 * Uses constant-velocity alpha-beta filter for corner prediction.
 * Maintains tracks through brief detection dropouts.
 */

#include "types.hpp"
#include <unordered_map>
#include <optional>

namespace frc_vision {

/**
 * @brief Alpha-beta filter for smooth tracking with velocity estimation
 */
class AlphaBetaFilter {
public:
    AlphaBetaFilter(double alpha = 0.3, double beta = 0.1);

    /**
     * @brief Update filter with new measurement
     * @param measurement New observed value
     * @param dt Time since last update (seconds)
     * @return Filtered value
     */
    double update(double measurement, double dt);

    /**
     * @brief Predict value at future time
     * @param dt Time into future (seconds)
     * @return Predicted value
     */
    double predict(double dt) const;

    /**
     * @brief Get current filtered value
     */
    double value() const { return x_; }

    /**
     * @brief Get current velocity estimate
     */
    double velocity() const { return v_; }

    /**
     * @brief Reset filter state
     */
    void reset(double initial_value = 0.0);

    /**
     * @brief Set filter parameters
     */
    void set_params(double alpha, double beta) { alpha_ = alpha; beta_ = beta; }

private:
    double alpha_;  // Position smoothing (0-1, higher = more responsive)
    double beta_;   // Velocity smoothing (0-1)
    double x_ = 0;  // Filtered position
    double v_ = 0;  // Estimated velocity
    bool initialized_ = false;
};

/**
 * @brief Tracked tag with motion prediction
 */
struct TagTracklet {
    int tag_id = -1;
    int track_id = -1;          // Unique track ID

    // Current state
    TagCorners corners;         // Last observed corners
    TagCorners predicted_corners; // Predicted corners for next frame

    // Per-corner filters (x, y for each of 4 corners)
    std::array<AlphaBetaFilter, 8> corner_filters;

    // Track quality
    double confidence = 1.0;     // 0-1, decays during dropouts
    int frames_tracked = 0;
    int frames_since_detection = 0;
    SteadyTimePoint last_seen;
    SteadyTimePoint first_seen;

    // Detection quality from last observation
    double last_margin = 0;
    int last_hamming = 0;
    double last_reproj_error = 0;

    // Pose tracking (optional)
    bool pose_valid = false;
    Pose3D pose;
    Pose3D pose_filtered;
    std::array<AlphaBetaFilter, 6> pose_filters;  // x,y,z,rx,ry,rz

    /**
     * @brief Get bounding box for predicted corners (for ROI)
     */
    cv::Rect get_predicted_roi(int margin = 50) const;

    /**
     * @brief Check if track is still alive
     */
    bool is_alive(int max_frames_missed, double min_confidence) const;
};

/**
 * @brief Multi-tag tracker for a single camera
 */
class TagTracker {
public:
    TagTracker(int camera_id = 0);

    /**
     * @brief Initialize tracker with configuration
     */
    void initialize(const TrackerConfig& config);

    /**
     * @brief Update tracks with new detections
     * @param detections New tag detections from current frame
     * @param timestamp Current frame timestamp
     * @return Updated detections with tracking info filled in
     */
    std::vector<TagDetection> update(
        const std::vector<TagDetection>& detections,
        SteadyTimePoint timestamp);

    /**
     * @brief Get predicted ROIs for next frame detection
     * @return Vector of ROIs to search for tags
     */
    std::vector<cv::Rect> get_predicted_rois() const;

    /**
     * @brief Get all active tracks
     */
    const std::unordered_map<int, TagTracklet>& tracks() const { return tracks_; }

    /**
     * @brief Get track by tag ID
     */
    std::optional<TagTracklet> get_track(int tag_id) const;

    /**
     * @brief Predict corners for a tag at future time
     * @param tag_id Tag ID to predict
     * @param dt Time into future (seconds)
     * @return Predicted corners if track exists
     */
    std::optional<TagCorners> predict_corners(int tag_id, double dt) const;

    /**
     * @brief Get number of active tracks
     */
    size_t active_track_count() const { return tracks_.size(); }

    /**
     * @brief Clear all tracks
     */
    void clear();

    /**
     * @brief Update configuration
     */
    void update_config(const TrackerConfig& config);

private:
    int camera_id_;
    TrackerConfig config_;

    std::unordered_map<int, TagTracklet> tracks_;  // tag_id -> tracklet
    int next_track_id_ = 0;

    SteadyTimePoint last_update_;
    bool first_update_ = true;

    // Helper methods
    void update_track(TagTracklet& track, const TagDetection& det,
                     double dt, SteadyTimePoint timestamp);
    void predict_track(TagTracklet& track, double dt);
    void prune_tracks(SteadyTimePoint timestamp);
    void decay_confidence(TagTracklet& track, double dt);
};

/**
 * @brief Pose smoother for reducing jitter without adding latency
 */
class PoseSmoother {
public:
    PoseSmoother(double alpha = 0.3);

    /**
     * @brief Update with new pose measurement
     * @param pose Raw pose measurement
     * @param valid Whether measurement is valid
     * @return Smoothed pose
     */
    Pose2D update(const Pose2D& pose, bool valid);

    /**
     * @brief Get current smoothed pose
     */
    const Pose2D& smoothed() const { return smoothed_; }

    /**
     * @brief Get current raw pose
     */
    const Pose2D& raw() const { return raw_; }

    /**
     * @brief Reset smoother
     */
    void reset();

    /**
     * @brief Set smoothing factor
     */
    void set_alpha(double alpha) { alpha_ = alpha; }

private:
    double alpha_;
    Pose2D smoothed_;
    Pose2D raw_;
    bool initialized_ = false;

    // Velocity estimation for prediction
    double vx_ = 0, vy_ = 0, vtheta_ = 0;
    SteadyTimePoint last_time_;
};

} // namespace frc_vision
