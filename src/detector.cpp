/**
 * @file detector.cpp
 * @brief AprilTag detection implementation
 */

#include "detector.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <chrono>
#include <algorithm>

// AprilTag library headers
extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagStandard41h12.h"
#include "tagCircle21h7.h"
}

namespace frc_vision {

Detector::Detector() = default;

Detector::~Detector() {
    cleanup();
}

void Detector::cleanup() {
    if (detector_) {
        apriltag_detector_destroy(detector_);
        detector_ = nullptr;
    }

    if (family_) {
        // Destroy based on family type
        if (config_.family == "tag36h11") {
            tag36h11_destroy(family_);
        } else if (config_.family == "tag25h9") {
            tag25h9_destroy(family_);
        } else if (config_.family == "tag16h5") {
            tag16h5_destroy(family_);
        } else if (config_.family == "tagStandard41h12") {
            tagStandard41h12_destroy(family_);
        } else if (config_.family == "tagCircle21h7") {
            tagCircle21h7_destroy(family_);
        }
        family_ = nullptr;
    }
}

bool Detector::initialize(const DetectorConfig& config) {
    cleanup();
    config_ = config;

    // Create tag family
    if (config_.family == "tag36h11") {
        family_ = tag36h11_create();
    } else if (config_.family == "tag25h9") {
        family_ = tag25h9_create();
    } else if (config_.family == "tag16h5") {
        family_ = tag16h5_create();
    } else if (config_.family == "tagStandard41h12") {
        family_ = tagStandard41h12_create();
    } else if (config_.family == "tagCircle21h7") {
        family_ = tagCircle21h7_create();
    } else {
        std::cerr << "[Detector] Unknown tag family: " << config_.family
                  << ", defaulting to tag36h11" << std::endl;
        family_ = tag36h11_create();
    }

    if (!family_) {
        std::cerr << "[Detector] Failed to create tag family" << std::endl;
        return false;
    }

    // Create detector
    detector_ = apriltag_detector_create();
    if (!detector_) {
        std::cerr << "[Detector] Failed to create detector" << std::endl;
        return false;
    }

    // Add family to detector
    apriltag_detector_add_family(detector_, family_);

    // Configure detector for speed vs accuracy tradeoff
    detector_->quad_decimate = static_cast<float>(config_.decimation);
    detector_->quad_sigma = static_cast<float>(config_.sigma);
    detector_->nthreads = config_.nthreads;
    detector_->refine_edges = config_.refine_edges ? 1 : 0;
    detector_->decode_sharpening = 0.25;  // Helps with motion blur

    std::cout << "[Detector] Initialized with family=" << config_.family
              << ", decimate=" << config_.decimation
              << ", threads=" << config_.nthreads << std::endl;

    return true;
}

std::vector<TagDetection> Detector::detect(const cv::Mat& image) {
    if (!detector_ || image.empty()) {
        return {};
    }

    auto start = SteadyClock::now();

    // Convert to grayscale if needed
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else if (image.channels() == 4) {
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
    } else {
        gray = image;
    }

    // Create apriltag image wrapper
    image_u8_t img = {
        .width = gray.cols,
        .height = gray.rows,
        .stride = static_cast<int32_t>(gray.step),
        .buf = gray.data
    };

    // Run detection
    zarray_t* detections = apriltag_detector_detect(detector_, &img);

    // Process detections (pass grayscale for sub-pixel refinement)
    auto result = process_detections(detections, gray);

    // Free detections
    apriltag_detections_destroy(detections);

    // Update statistics
    auto end = SteadyClock::now();
    double detect_time = std::chrono::duration<double, std::milli>(end - start).count();

    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.frames_processed++;
        stats_.total_detections += result.size();
        // Exponential moving average
        stats_.avg_detect_time_ms = stats_.avg_detect_time_ms * 0.9 + detect_time * 0.1;
    }

    return result;
}

std::vector<TagDetection> Detector::detect_roi(const cv::Mat& image, const cv::Rect& roi) {
    if (!detector_ || image.empty()) {
        return {};
    }

    // Validate ROI
    cv::Rect safe_roi = roi & cv::Rect(0, 0, image.cols, image.rows);
    if (safe_roi.width < 20 || safe_roi.height < 20) {
        return {};
    }

    // Extract ROI
    cv::Mat roi_image = image(safe_roi);

    // Detect in ROI
    auto detections = detect(roi_image);

    // Offset detection coordinates to full image frame
    for (auto& det : detections) {
        for (auto& corner : det.corners.corners) {
            corner.x += safe_roi.x;
            corner.y += safe_roi.y;
        }
    }

    return detections;
}

std::vector<TagDetection> Detector::process_detections(void* detections_ptr, const cv::Mat& gray_image) {
    zarray_t* detections = static_cast<zarray_t*>(detections_ptr);
    std::vector<TagDetection> result;

    if (!detections) {
        return result;
    }

    int num_detections = zarray_size(detections);

    // Sort by decision margin (quality) if we need to limit
    std::vector<std::pair<double, int>> sorted_indices;
    for (int i = 0; i < num_detections; i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);
        sorted_indices.emplace_back(det->decision_margin, i);
    }

    // Sort descending by margin
    std::sort(sorted_indices.begin(), sorted_indices.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });

    // Process up to max_tags_per_frame
    int max_tags = std::min(num_detections, config_.max_tags_per_frame);

    for (int i = 0; i < max_tags; i++) {
        int idx = sorted_indices[i].second;
        apriltag_detection_t* det;
        zarray_get(detections, idx, &det);

        // Filter by quality
        if (det->hamming > config_.max_hamming) {
            continue;
        }
        if (det->decision_margin < config_.min_margin) {
            continue;
        }

        TagDetection tag;
        tag.id = det->id;
        tag.decision_margin = det->decision_margin;
        tag.hamming = det->hamming;

        // Extract corners (counter-clockwise from bottom-left)
        // AprilTag detection corners are: bottom-left, bottom-right, top-right, top-left
        for (int j = 0; j < 4; j++) {
            tag.corners.corners[j].x = det->p[j][0];
            tag.corners.corners[j].y = det->p[j][1];
        }

        // Phase 1: Additional sub-pixel corner refinement with OpenCV
        if (config_.subpixel_refine && !gray_image.empty()) {
            std::vector<cv::Point2f> corners_cv;
            corners_cv.reserve(4);
            for (int j = 0; j < 4; j++) {
                corners_cv.emplace_back(tag.corners.corners[j].x, tag.corners.corners[j].y);
            }

            // Sub-pixel refinement parameters (optimized for AprilTag corners)
            cv::Size win_size(5, 5);           // 5x5 window
            cv::Size zero_zone(-1, -1);        // No dead zone
            cv::TermCriteria criteria(
                cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                40,      // Max 40 iterations
                0.001    // Epsilon for convergence
            );

            try {
                cv::cornerSubPix(gray_image, corners_cv, win_size, zero_zone, criteria);

                // Update corners with refined positions
                for (int j = 0; j < 4; j++) {
                    tag.corners.corners[j].x = corners_cv[j].x;
                    tag.corners.corners[j].y = corners_cv[j].y;
                }
            } catch (const cv::Exception& e) {
                // If sub-pixel refinement fails, keep original corners
                // This can happen at image edges
            }
        }

        result.push_back(std::move(tag));
    }

    return result;
}

void Detector::update_config(const DetectorConfig& config) {
    // Some settings can be updated without reinitializing
    if (detector_) {
        if (config.decimation != config_.decimation) {
            detector_->quad_decimate = static_cast<float>(config.decimation);
        }
        if (config.sigma != config_.sigma) {
            detector_->quad_sigma = static_cast<float>(config.sigma);
        }
        if (config.nthreads != config_.nthreads) {
            detector_->nthreads = config.nthreads;
        }
        if (config.refine_edges != config_.refine_edges) {
            detector_->refine_edges = config.refine_edges ? 1 : 0;
        }
    }

    // Update stored config
    config_.decimation = config.decimation;
    config_.sigma = config.sigma;
    config_.nthreads = config.nthreads;
    config_.refine_edges = config.refine_edges;
    config_.max_hamming = config.max_hamming;
    config_.min_margin = config.min_margin;
    config_.max_tags_per_frame = config.max_tags_per_frame;

    // If family changed, need full reinit
    if (config.family != config_.family) {
        initialize(config);
    }
}

} // namespace frc_vision
