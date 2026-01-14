/**
 * @file tracker.cpp
 * @brief Tag tracking implementation
 */

#include "tracker.hpp"
#include <algorithm>
#include <cmath>

namespace frc_vision {

// =============================================================================
// AlphaBetaFilter
// =============================================================================

AlphaBetaFilter::AlphaBetaFilter(double alpha, double beta)
    : alpha_(alpha), beta_(beta) {}

double AlphaBetaFilter::update(double measurement, double dt) {
    if (!initialized_) {
        x_ = measurement;
        v_ = 0;
        initialized_ = true;
        return x_;
    }

    // Predict
    double x_pred = x_ + v_ * dt;

    // Update
    double residual = measurement - x_pred;
    x_ = x_pred + alpha_ * residual;
    v_ = v_ + (beta_ / dt) * residual;

    return x_;
}

double AlphaBetaFilter::predict(double dt) const {
    return x_ + v_ * dt;
}

void AlphaBetaFilter::reset(double initial_value) {
    x_ = initial_value;
    v_ = 0;
    initialized_ = false;
}

// =============================================================================
// TagTracklet
// =============================================================================

cv::Rect TagTracklet::get_predicted_roi(int margin) const {
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto& corner : predicted_corners.corners) {
        min_x = std::min(min_x, corner.x);
        min_y = std::min(min_y, corner.y);
        max_x = std::max(max_x, corner.x);
        max_y = std::max(max_y, corner.y);
    }

    return cv::Rect(
        static_cast<int>(min_x - margin),
        static_cast<int>(min_y - margin),
        static_cast<int>(max_x - min_x + 2 * margin),
        static_cast<int>(max_y - min_y + 2 * margin)
    );
}

bool TagTracklet::is_alive(int max_frames_missed, double min_confidence) const {
    return frames_since_detection < max_frames_missed && confidence > min_confidence;
}

// =============================================================================
// TagTracker
// =============================================================================

TagTracker::TagTracker(int camera_id)
    : camera_id_(camera_id) {}

void TagTracker::initialize(const TrackerConfig& config) {
    config_ = config;
    tracks_.clear();
    next_track_id_ = 0;
    first_update_ = true;
}

std::vector<TagDetection> TagTracker::update(
    const std::vector<TagDetection>& detections,
    SteadyTimePoint timestamp)
{
    if (!config_.enable) {
        // Tracking disabled, just return detections as-is
        return detections;
    }

    // Calculate dt
    double dt = 0.016;  // Default ~60 FPS
    if (!first_update_) {
        dt = std::chrono::duration<double>(timestamp - last_update_).count();
        dt = std::clamp(dt, 0.001, 0.5);  // Clamp to reasonable range
    }
    first_update_ = false;
    last_update_ = timestamp;

    // Build set of detected tag IDs
    std::unordered_set<int> detected_ids;
    for (const auto& det : detections) {
        detected_ids.insert(det.id);
    }

    // Update existing tracks with matched detections
    for (const auto& det : detections) {
        auto it = tracks_.find(det.id);

        if (it != tracks_.end()) {
            // Update existing track
            update_track(it->second, det, dt, timestamp);
        } else {
            // Create new track
            TagTracklet track;
            track.tag_id = det.id;
            track.track_id = next_track_id_++;
            track.first_seen = timestamp;

            // Initialize filters with current corners
            for (int i = 0; i < 4; i++) {
                track.corner_filters[i * 2].reset(det.corners.corners[i].x);
                track.corner_filters[i * 2 + 1].reset(det.corners.corners[i].y);
            }

            update_track(track, det, dt, timestamp);
            tracks_[det.id] = std::move(track);
        }
    }

    // Update tracks without detections (prediction only)
    for (auto& [tag_id, track] : tracks_) {
        if (detected_ids.find(tag_id) == detected_ids.end()) {
            // No detection this frame - predict and decay
            track.frames_since_detection++;
            decay_confidence(track, dt);
            predict_track(track, dt);
        }
    }

    // Prune dead tracks
    prune_tracks(timestamp);

    // Build output with tracking info
    std::vector<TagDetection> result;
    result.reserve(detections.size());

    for (auto det : detections) {
        auto it = tracks_.find(det.id);
        if (it != tracks_.end()) {
            det.track_id = it->second.track_id;
            det.frames_tracked = it->second.frames_tracked;
            det.confidence = it->second.confidence;
        }
        result.push_back(std::move(det));
    }

    return result;
}

void TagTracker::update_track(TagTracklet& track, const TagDetection& det,
                              double dt, SteadyTimePoint timestamp)
{
    // Update corner filters
    for (int i = 0; i < 4; i++) {
        track.corner_filters[i * 2].update(det.corners.corners[i].x, dt);
        track.corner_filters[i * 2 + 1].update(det.corners.corners[i].y, dt);
    }

    // Update current corners from filters
    for (int i = 0; i < 4; i++) {
        track.corners.corners[i].x = track.corner_filters[i * 2].value();
        track.corners.corners[i].y = track.corner_filters[i * 2 + 1].value();
    }

    // Predict next frame corners
    predict_track(track, dt);

    // Update quality metrics
    track.last_margin = det.decision_margin;
    track.last_hamming = det.hamming;
    track.last_seen = timestamp;
    track.frames_since_detection = 0;
    track.frames_tracked++;

    // Restore confidence
    track.confidence = std::min(1.0, track.confidence + 0.2);
}

void TagTracker::predict_track(TagTracklet& track, double dt) {
    for (int i = 0; i < 4; i++) {
        track.predicted_corners.corners[i].x = track.corner_filters[i * 2].predict(dt);
        track.predicted_corners.corners[i].y = track.corner_filters[i * 2 + 1].predict(dt);
    }
}

void TagTracker::decay_confidence(TagTracklet& track, double dt) {
    // Exponential decay
    double decay_rate = -std::log(config_.velocity_decay);
    track.confidence *= std::exp(-decay_rate * dt * 60.0);  // Normalize to ~60 FPS
}

void TagTracker::prune_tracks(SteadyTimePoint timestamp) {
    int max_frames = static_cast<int>(config_.dropout_ms / 16.67);  // Assume ~60 FPS
    double min_conf = 0.1;

    for (auto it = tracks_.begin(); it != tracks_.end(); ) {
        if (!it->second.is_alive(max_frames, min_conf)) {
            it = tracks_.erase(it);
        } else {
            ++it;
        }
    }
}

std::vector<cv::Rect> TagTracker::get_predicted_rois() const {
    if (!config_.roi_enable) {
        return {};
    }

    std::vector<cv::Rect> rois;
    rois.reserve(tracks_.size());

    for (const auto& [tag_id, track] : tracks_) {
        if (track.confidence > 0.3) {
            rois.push_back(track.get_predicted_roi(50));
        }
    }

    return rois;
}

std::optional<TagTracklet> TagTracker::get_track(int tag_id) const {
    auto it = tracks_.find(tag_id);
    if (it != tracks_.end()) {
        return it->second;
    }
    return std::nullopt;
}

std::optional<TagCorners> TagTracker::predict_corners(int tag_id, double dt) const {
    auto it = tracks_.find(tag_id);
    if (it == tracks_.end()) {
        return std::nullopt;
    }

    TagCorners predicted;
    for (int i = 0; i < 4; i++) {
        predicted.corners[i].x = it->second.corner_filters[i * 2].predict(dt);
        predicted.corners[i].y = it->second.corner_filters[i * 2 + 1].predict(dt);
    }
    return predicted;
}

void TagTracker::clear() {
    tracks_.clear();
    next_track_id_ = 0;
    first_update_ = true;
}

void TagTracker::update_config(const TrackerConfig& config) {
    double new_alpha = config.filter_alpha;

    // Update filter parameters on existing tracks
    if (new_alpha != config_.filter_alpha) {
        for (auto& [tag_id, track] : tracks_) {
            for (auto& filter : track.corner_filters) {
                filter.set_params(new_alpha, new_alpha * 0.5);
            }
        }
    }

    config_ = config;
}

// =============================================================================
// PoseSmoother
// =============================================================================

PoseSmoother::PoseSmoother(double alpha)
    : alpha_(alpha) {}

Pose2D PoseSmoother::update(const Pose2D& pose, bool valid) {
    raw_ = pose;

    if (!valid) {
        // Use velocity prediction if no valid measurement
        if (initialized_) {
            auto now = SteadyClock::now();
            double dt = std::chrono::duration<double>(now - last_time_).count();
            dt = std::clamp(dt, 0.001, 0.1);

            smoothed_.x += vx_ * dt;
            smoothed_.y += vy_ * dt;
            smoothed_.theta += vtheta_ * dt;

            // Decay velocity during dropout
            vx_ *= 0.9;
            vy_ *= 0.9;
            vtheta_ *= 0.9;

            last_time_ = now;
        }
        return smoothed_;
    }

    if (!initialized_) {
        smoothed_ = pose;
        initialized_ = true;
        last_time_ = SteadyClock::now();
        return smoothed_;
    }

    auto now = SteadyClock::now();
    double dt = std::chrono::duration<double>(now - last_time_).count();
    dt = std::clamp(dt, 0.001, 0.1);
    last_time_ = now;

    // EMA update
    Pose2D prev = smoothed_;

    smoothed_.x = alpha_ * pose.x + (1.0 - alpha_) * smoothed_.x;
    smoothed_.y = alpha_ * pose.y + (1.0 - alpha_) * smoothed_.y;

    // Handle angle wrapping for theta
    double dtheta = pose.theta - smoothed_.theta;
    while (dtheta > M_PI) dtheta -= 2 * M_PI;
    while (dtheta < -M_PI) dtheta += 2 * M_PI;
    smoothed_.theta = smoothed_.theta + alpha_ * dtheta;

    // Normalize theta
    while (smoothed_.theta > M_PI) smoothed_.theta -= 2 * M_PI;
    while (smoothed_.theta < -M_PI) smoothed_.theta += 2 * M_PI;

    // Update velocity estimates
    vx_ = (smoothed_.x - prev.x) / dt;
    vy_ = (smoothed_.y - prev.y) / dt;
    vtheta_ = (smoothed_.theta - prev.theta) / dt;

    return smoothed_;
}

void PoseSmoother::reset() {
    initialized_ = false;
    vx_ = vy_ = vtheta_ = 0;
}

} // namespace frc_vision
