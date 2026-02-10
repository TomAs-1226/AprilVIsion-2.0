/**
 * @file phase2_monitoring.hpp
 * @brief Phase 2: Runtime calibration health and pose consistency monitoring
 *
 * Advanced monitoring features:
 * - Real-time calibration drift detection
 * - Pose consistency checking (temporal, spatial, odometry validation)
 * - Outlier detection and filtering
 */

#pragma once

#include "types.hpp"
#include <deque>
#include <vector>
#include <chrono>
#include <cmath>
#include <iostream>

namespace frc_vision {
namespace phase2 {

/**
 * @brief Runtime calibration health metrics
 */
struct RuntimeCalibrationHealth {
    double avg_reproj_error = 0.0;          // Rolling average (1-minute window)
    double max_reproj_error = 0.0;          // Maximum in window
    int suspicious_detection_count = 0;      // Count with reproj > 5px
    int total_detection_count = 0;           // Total detections in window

    bool calibration_suspect = false;        // True if drift detected
    std::string status_message = "OK";       // Human-readable status

    double calibration_confidence = 1.0;     // 0-1 confidence in calibration quality
};

/**
 * @brief Runtime calibration health monitor
 *
 * Tracks reprojection errors over time to detect calibration drift
 */
class CalibrationHealthMonitor {
public:
    CalibrationHealthMonitor(double baseline_rms = 0.5)
        : baseline_rms_(baseline_rms)
        , window_duration_seconds_(60.0)  // 1-minute window
    {}

    /**
     * @brief Update with new detection
     */
    void update(double reprojection_error, SteadyTimePoint timestamp) {
        // Add to rolling window
        reproj_errors_.push_back({reprojection_error, timestamp});

        // Remove old entries (older than window duration)
        auto cutoff_time = timestamp - std::chrono::duration<double>(window_duration_seconds_);

        while (!reproj_errors_.empty() &&
               reproj_errors_.front().timestamp < cutoff_time) {
            reproj_errors_.pop_front();
        }

        // Update health every 100 detections or every 10 seconds
        detection_count_++;
        if (detection_count_ % 100 == 0 ||
            std::chrono::duration<double>(timestamp - last_check_time_).count() > 10.0) {
            compute_health();
            last_check_time_ = timestamp;
        }
    }

    /**
     * @brief Get current calibration health
     */
    const RuntimeCalibrationHealth& get_health() const {
        return health_;
    }

    /**
     * @brief Reset monitoring (after recalibration)
     */
    void reset(double new_baseline_rms = 0.5) {
        reproj_errors_.clear();
        baseline_rms_ = new_baseline_rms;
        detection_count_ = 0;
        health_ = RuntimeCalibrationHealth();
    }

private:
    struct ReproError {
        double error;
        SteadyTimePoint timestamp;
    };

    void compute_health() {
        if (reproj_errors_.empty()) {
            health_.status_message = "No data";
            return;
        }

        // Compute statistics
        double sum = 0.0;
        health_.max_reproj_error = 0.0;
        health_.suspicious_detection_count = 0;

        for (const auto& entry : reproj_errors_) {
            sum += entry.error;
            health_.max_reproj_error = std::max(health_.max_reproj_error, entry.error);
            if (entry.error > 5.0) {
                health_.suspicious_detection_count++;
            }
        }

        health_.avg_reproj_error = sum / reproj_errors_.size();
        health_.total_detection_count = static_cast<int>(reproj_errors_.size());

        // Calibration drift detection
        double drift_factor = health_.avg_reproj_error / baseline_rms_;
        double suspicious_ratio = static_cast<double>(health_.suspicious_detection_count) /
                                 health_.total_detection_count;

        // Flag calibration as suspect if:
        // 1. Average error is > 2x baseline RMS
        // 2. More than 10% detections have >5px error
        health_.calibration_suspect = (drift_factor > 2.0) || (suspicious_ratio > 0.1);

        // Compute confidence (1.0 = perfect, 0.0 = total failure)
        health_.calibration_confidence = 1.0 / (1.0 + drift_factor - 1.0);
        health_.calibration_confidence = std::clamp(health_.calibration_confidence, 0.0, 1.0);

        // Status message
        if (health_.calibration_suspect) {
            health_.status_message = "CALIBRATION DRIFT DETECTED - Consider recalibrating!";
            std::cerr << "[CalibrationHealth] WARNING: " << health_.status_message << std::endl;
            std::cerr << "  Avg reproj error: " << health_.avg_reproj_error
                     << "px (baseline: " << baseline_rms_ << "px)" << std::endl;
        } else if (drift_factor > 1.5) {
            health_.status_message = "Warning: Calibration quality degrading";
        } else {
            health_.status_message = "OK";
        }
    }

    std::deque<ReproError> reproj_errors_;
    double baseline_rms_;
    double window_duration_seconds_;
    int detection_count_ = 0;
    SteadyTimePoint last_check_time_ = SteadyClock::now();
    RuntimeCalibrationHealth health_;
};

/**
 * @brief Pose consistency metrics
 */
struct PoseConsistencyMetrics {
    bool passes_temporal_check = true;      // No impossible jumps in position
    bool passes_spatial_check = true;       // Cameras agree on position
    bool passes_odometry_check = true;      // Agrees with wheel odometry
    bool passes_sanity_checks = true;       // All checks pass

    double temporal_confidence = 1.0;       // 0-1 confidence from temporal check
    double spatial_confidence = 1.0;        // 0-1 confidence from spatial check
    double odometry_confidence = 1.0;       // 0-1 confidence from odometry check

    double overall_confidence = 1.0;        // Combined confidence
    std::vector<std::string> warnings;
};

/**
 * @brief Pose consistency checker (Phase 2)
 *
 * Validates pose estimates for physical plausibility and consistency
 */
class PoseConsistencyChecker {
public:
    PoseConsistencyChecker()
        : max_velocity_mps_(5.0)      // FRC robots max ~5 m/s
        , max_angular_velocity_rps_(3.0)  // Max ~3 rad/s spinning
        , spatial_variance_threshold_(0.25)  // 0.25m² variance threshold
        , odometry_distance_threshold_(2.0)  // 2m max disagreement
    {}

    /**
     * @brief Check temporal consistency (no impossible velocity jumps)
     */
    bool check_temporal(const Pose2D& current_pose, const Pose2D& previous_pose,
                       double dt_seconds, PoseConsistencyMetrics& metrics) {
        if (dt_seconds < 0.001) {
            // Too close in time, can't check
            return true;
        }

        // Compute position change
        double dx = current_pose.x - previous_pose.x;
        double dy = current_pose.y - previous_pose.y;
        double distance_moved = std::sqrt(dx * dx + dy * dy);

        double velocity = distance_moved / dt_seconds;

        // Compute angular velocity
        double dtheta = current_pose.theta - previous_pose.theta;
        // Normalize to [-π, π]
        while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
        while (dtheta < -M_PI) dtheta += 2.0 * M_PI;

        double angular_velocity = std::abs(dtheta) / dt_seconds;

        // Check for impossible speeds
        if (velocity > max_velocity_mps_) {
            metrics.passes_temporal_check = false;
            metrics.warnings.push_back("Impossible velocity: " +
                                      std::to_string(velocity) + " m/s");
            metrics.temporal_confidence = max_velocity_mps_ / velocity;
            return false;
        }

        if (angular_velocity > max_angular_velocity_rps_) {
            metrics.passes_temporal_check = false;
            metrics.warnings.push_back("Impossible angular velocity: " +
                                      std::to_string(angular_velocity) + " rad/s");
            metrics.temporal_confidence = max_angular_velocity_rps_ / angular_velocity;
            return false;
        }

        // Confidence based on how close to limits
        double vel_ratio = velocity / max_velocity_mps_;
        double ang_vel_ratio = angular_velocity / max_angular_velocity_rps_;
        metrics.temporal_confidence = 1.0 - std::max(vel_ratio, ang_vel_ratio);
        metrics.temporal_confidence = std::clamp(metrics.temporal_confidence, 0.0, 1.0);

        return true;
    }

    /**
     * @brief Check spatial consistency (multiple cameras agree)
     */
    bool check_spatial(const std::vector<Pose2D>& camera_poses,
                      PoseConsistencyMetrics& metrics) {
        if (camera_poses.size() < 2) {
            // Need at least 2 cameras for spatial check
            return true;
        }

        // Compute variance of poses
        double mean_x = 0.0, mean_y = 0.0;
        for (const auto& pose : camera_poses) {
            mean_x += pose.x;
            mean_y += pose.y;
        }
        mean_x /= camera_poses.size();
        mean_y /= camera_poses.size();

        double var_x = 0.0, var_y = 0.0;
        for (const auto& pose : camera_poses) {
            var_x += (pose.x - mean_x) * (pose.x - mean_x);
            var_y += (pose.y - mean_y) * (pose.y - mean_y);
        }
        var_x /= camera_poses.size();
        var_y /= camera_poses.size();

        double total_variance = var_x + var_y;

        if (total_variance > spatial_variance_threshold_) {
            metrics.passes_spatial_check = false;
            metrics.warnings.push_back("High spatial variance: " +
                                      std::to_string(total_variance) + " m²");
            metrics.spatial_confidence = spatial_variance_threshold_ / total_variance;
            return false;
        }

        metrics.spatial_confidence = 1.0 - (total_variance / spatial_variance_threshold_);
        metrics.spatial_confidence = std::clamp(metrics.spatial_confidence, 0.0, 1.0);

        return true;
    }

    /**
     * @brief Check odometry consistency (vision agrees with wheel odometry)
     */
    bool check_odometry(const Pose2D& vision_pose, const Pose2D& odometry_pose,
                       PoseConsistencyMetrics& metrics) {
        // Compute distance between vision and odometry estimates
        double dx = vision_pose.x - odometry_pose.x;
        double dy = vision_pose.y - odometry_pose.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance > odometry_distance_threshold_) {
            metrics.passes_odometry_check = false;
            metrics.warnings.push_back("Vision-odometry disagreement: " +
                                      std::to_string(distance) + " m");
            metrics.odometry_confidence = odometry_distance_threshold_ / distance;
            return false;
        }

        metrics.odometry_confidence = 1.0 - (distance / odometry_distance_threshold_);
        metrics.odometry_confidence = std::clamp(metrics.odometry_confidence, 0.0, 1.0);

        return true;
    }

    /**
     * @brief Full consistency check
     */
    PoseConsistencyMetrics check_all(
        const Pose2D& current_pose,
        const Pose2D& previous_pose,
        double dt_seconds,
        const std::vector<Pose2D>& camera_poses,
        const std::optional<Pose2D>& odometry_pose = std::nullopt)
    {
        PoseConsistencyMetrics metrics;

        // Temporal check
        check_temporal(current_pose, previous_pose, dt_seconds, metrics);

        // Spatial check
        check_spatial(camera_poses, metrics);

        // Odometry check (if available)
        if (odometry_pose.has_value()) {
            check_odometry(current_pose, odometry_pose.value(), metrics);
        }

        // Overall sanity
        metrics.passes_sanity_checks = metrics.passes_temporal_check &&
                                       metrics.passes_spatial_check &&
                                       metrics.passes_odometry_check;

        // Overall confidence (geometric mean of all confidences)
        metrics.overall_confidence = std::pow(
            metrics.temporal_confidence *
            metrics.spatial_confidence *
            metrics.odometry_confidence,
            1.0 / 3.0
        );

        return metrics;
    }

private:
    double max_velocity_mps_;
    double max_angular_velocity_rps_;
    double spatial_variance_threshold_;
    double odometry_distance_threshold_;
};

} // namespace phase2
} // namespace frc_vision
