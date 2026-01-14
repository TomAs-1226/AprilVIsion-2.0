#pragma once
/**
 * @file detector.hpp
 * @brief AprilTag detection with optimized settings for FRC
 */

#include "types.hpp"
#include <memory>

// Forward declare apriltag types
struct apriltag_detector;
struct apriltag_family;

namespace frc_vision {

/**
 * @brief AprilTag detector wrapper with FRC optimizations
 */
class Detector {
public:
    Detector();
    ~Detector();

    // Non-copyable
    Detector(const Detector&) = delete;
    Detector& operator=(const Detector&) = delete;

    /**
     * @brief Initialize detector with configuration
     * @param config Detector configuration
     * @return true on success
     */
    bool initialize(const DetectorConfig& config);

    /**
     * @brief Detect AprilTags in image
     * @param image Input image (BGR or grayscale)
     * @return Vector of detections
     */
    std::vector<TagDetection> detect(const cv::Mat& image);

    /**
     * @brief Detect AprilTags in ROI only (for tracking)
     * @param image Full input image
     * @param roi Region of interest to search
     * @return Vector of detections (coordinates in full image frame)
     */
    std::vector<TagDetection> detect_roi(const cv::Mat& image, const cv::Rect& roi);

    /**
     * @brief Update configuration (hot reload)
     */
    void update_config(const DetectorConfig& config);

    /**
     * @brief Get current configuration
     */
    const DetectorConfig& config() const { return config_; }

    /**
     * @brief Get detection statistics
     */
    struct Stats {
        double avg_detect_time_ms = 0;
        uint64_t total_detections = 0;
        uint64_t frames_processed = 0;
    };
    Stats get_stats() const { return stats_; }

private:
    void cleanup();
    std::vector<TagDetection> process_detections(void* detections_ptr);

    DetectorConfig config_;

    apriltag_detector* detector_ = nullptr;
    apriltag_family* family_ = nullptr;

    Stats stats_;
    mutable std::mutex stats_mutex_;
};

} // namespace frc_vision
