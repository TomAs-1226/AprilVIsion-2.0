/**
 * @file calibration.cpp
 * @brief Camera calibration implementation using ChArUco boards
 */

#include "calibration.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>

namespace frc_vision {

CameraCalibrator::CameraCalibrator() = default;
CameraCalibrator::~CameraCalibrator() = default;

void CameraCalibrator::initialize(const CalibrationBoardConfig& config) {
    config_ = config;

    // Create ArUco dictionary
    dictionary_ = cv::aruco::getPredefinedDictionary(config_.dictionary_id);

    // Create ChArUco board
    board_ = cv::aruco::CharucoBoard::create(
        config_.squares_x,
        config_.squares_y,
        config_.square_length,
        config_.marker_length,
        dictionary_);

    // Configure detector for accuracy
    detector_params_ = cv::aruco::DetectorParameters::create();
    detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    detector_params_->cornerRefinementWinSize = 5;
    detector_params_->cornerRefinementMaxIterations = 50;
    detector_params_->cornerRefinementMinAccuracy = 0.01;
    detector_params_->adaptiveThreshWinSizeMin = 3;
    detector_params_->adaptiveThreshWinSizeMax = 23;
    detector_params_->adaptiveThreshWinSizeStep = 10;

    clear();

    std::cout << "[Calibrator] Initialized with " << config_.squares_x << "x"
              << config_.squares_y << " ChArUco board" << std::endl;
}

CalibrationFrame CameraCalibrator::process_frame(const cv::Mat& image) {
    CalibrationFrame result;
    result.valid = false;
    result.corners_detected = 0;
    result.markers_detected = 0;

    if (image.empty()) {
        return result;
    }

    // Convert to grayscale if needed
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    // Detect ArUco markers
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<int> marker_ids;
    cv::aruco::detectMarkers(gray, dictionary_, marker_corners, marker_ids, detector_params_);

    result.markers_detected = static_cast<int>(marker_ids.size());

    if (marker_ids.empty()) {
        return result;
    }

    // Interpolate ChArUco corners
    std::vector<cv::Point2f> charuco_corners;
    std::vector<int> charuco_ids;
    int corners = cv::aruco::interpolateCornersCharuco(
        marker_corners, marker_ids, gray, board_,
        charuco_corners, charuco_ids);

    result.corners_detected = corners;

    // Store for visualization
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_corners_ = charuco_corners;
        last_ids_ = charuco_ids;
        last_marker_corners_ = marker_corners;
        last_marker_ids_ = marker_ids;
    }

    // Need minimum corners for a valid frame
    if (corners >= 6) {
        result.valid = true;
        result.image = image.clone();
    }

    return result;
}

bool CameraCalibrator::capture_frame(const cv::Mat& image) {
    if (image.empty()) {
        return false;
    }

    // Convert to grayscale
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    // Detect ArUco markers
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<int> marker_ids;
    cv::aruco::detectMarkers(gray, dictionary_, marker_corners, marker_ids, detector_params_);

    if (marker_ids.size() < 4) {
        std::cerr << "[Calibrator] Not enough markers detected: " << marker_ids.size() << std::endl;
        return false;
    }

    // Interpolate ChArUco corners
    std::vector<cv::Point2f> charuco_corners;
    std::vector<int> charuco_ids;
    int corners = cv::aruco::interpolateCornersCharuco(
        marker_corners, marker_ids, gray, board_,
        charuco_corners, charuco_ids);

    if (corners < 6) {
        std::cerr << "[Calibrator] Not enough corners detected: " << corners << std::endl;
        return false;
    }

    // Add to calibration set
    {
        std::lock_guard<std::mutex> lock(mutex_);
        all_charuco_corners_.push_back(charuco_corners);
        all_charuco_ids_.push_back(charuco_ids);
        captured_frames_++;
    }

    std::cout << "[Calibrator] Frame " << captured_frames_.load()
              << " captured with " << corners << " corners" << std::endl;

    return true;
}

CalibrationResult CameraCalibrator::compute_calibration(cv::Size image_size) {
    CalibrationResult result;

    std::lock_guard<std::mutex> lock(mutex_);

    if (all_charuco_corners_.size() < static_cast<size_t>(min_frames())) {
        result.error_message = "Not enough frames. Need at least " +
                              std::to_string(min_frames()) + " frames, have " +
                              std::to_string(all_charuco_corners_.size());
        return result;
    }

    // Run calibration
    cv::Mat camera_matrix, dist_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;

    try {
        result.rms_error = cv::aruco::calibrateCameraCharuco(
            all_charuco_corners_,
            all_charuco_ids_,
            board_,
            image_size,
            camera_matrix,
            dist_coeffs,
            rvecs, tvecs,
            cv::CALIB_RATIONAL_MODEL  // Use rational distortion model for wide-angle
        );
    } catch (const cv::Exception& e) {
        result.error_message = "Calibration failed: " + std::string(e.what());
        return result;
    }

    if (result.rms_error > 2.0) {
        result.error_message = "Calibration RMS error too high: " +
                              std::to_string(result.rms_error);
        // Continue anyway, but mark as potentially low quality
    }

    // Compute per-frame reprojection errors
    for (size_t i = 0; i < all_charuco_corners_.size(); i++) {
        std::vector<cv::Point2f> projected;
        std::vector<cv::Point3f> obj_points;

        // Get object points for this frame's detected corners
        for (int id : all_charuco_ids_[i]) {
            cv::Point3f pt = board_->chessboardCorners[id];
            obj_points.push_back(pt);
        }

        cv::projectPoints(obj_points, rvecs[i], tvecs[i],
                         camera_matrix, dist_coeffs, projected);

        double err = 0;
        for (size_t j = 0; j < projected.size(); j++) {
            double dx = projected[j].x - all_charuco_corners_[i][j].x;
            double dy = projected[j].y - all_charuco_corners_[i][j].y;
            err += std::sqrt(dx * dx + dy * dy);
        }
        err /= projected.size();
        result.per_frame_errors.push_back(err);
    }

    // Store results
    result.intrinsics.camera_matrix = camera_matrix;
    result.intrinsics.dist_coeffs = dist_coeffs;
    result.intrinsics.width = image_size.width;
    result.intrinsics.height = image_size.height;
    result.intrinsics.fx = camera_matrix.at<double>(0, 0);
    result.intrinsics.fy = camera_matrix.at<double>(1, 1);
    result.intrinsics.cx = camera_matrix.at<double>(0, 2);
    result.intrinsics.cy = camera_matrix.at<double>(1, 2);
    result.frames_used = static_cast<int>(all_charuco_corners_.size());
    result.success = true;

    // Phase 1: Compute comprehensive quality metrics
    result.quality_metrics = calibration_utils::compute_quality_metrics(
        result, all_charuco_corners_, image_size);

    std::cout << "[Calibrator] Calibration complete!" << std::endl;
    std::cout << "  RMS Error: " << std::fixed << std::setprecision(4) << result.rms_error << " pixels" << std::endl;
    std::cout << "  Frames used: " << result.frames_used << std::endl;
    std::cout << "  fx=" << result.intrinsics.fx << " fy=" << result.intrinsics.fy << std::endl;
    std::cout << "  cx=" << result.intrinsics.cx << " cy=" << result.intrinsics.cy << std::endl;

    return result;
}

bool CameraCalibrator::save_calibration(const std::string& path, const CalibrationResult& result) {
    if (!result.success) {
        return false;
    }

    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "[Calibrator] Failed to open " << path << " for writing" << std::endl;
        return false;
    }

    fs << "image_width" << result.intrinsics.width;
    fs << "image_height" << result.intrinsics.height;
    fs << "camera_matrix" << result.intrinsics.camera_matrix;
    fs << "distortion_coefficients" << result.intrinsics.dist_coeffs;
    fs << "rms_error" << result.rms_error;
    fs << "calibration_frames" << result.frames_used;

    // Also save individual parameters for easy reading
    fs << "fx" << result.intrinsics.fx;
    fs << "fy" << result.intrinsics.fy;
    fs << "cx" << result.intrinsics.cx;
    fs << "cy" << result.intrinsics.cy;

    fs.release();

    std::cout << "[Calibrator] Saved calibration to " << path << std::endl;
    return true;
}

void CameraCalibrator::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    all_charuco_corners_.clear();
    all_charuco_ids_.clear();
    captured_frames_ = 0;
    last_corners_.clear();
    last_ids_.clear();
    last_marker_corners_.clear();
    last_marker_ids_.clear();
}

void CameraCalibrator::draw_board(cv::Mat& image) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Draw detected ArUco markers
    if (!last_marker_corners_.empty()) {
        cv::aruco::drawDetectedMarkers(image, last_marker_corners_, last_marker_ids_);
    }

    // Draw ChArUco corners
    if (!last_corners_.empty()) {
        cv::aruco::drawDetectedCornersCharuco(image, last_corners_, last_ids_, cv::Scalar(0, 255, 0));
    }

    // Draw status
    std::string status = "Frames: " + std::to_string(captured_frames_.load()) + "/" +
                        std::to_string(recommended_frames()) +
                        " | Corners: " + std::to_string(last_corners_.size());

    cv::putText(image, status, cv::Point(10, 30),
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

    // Draw instructions
    if (captured_frames_ < min_frames()) {
        cv::putText(image, "Move board to different positions/angles",
                   cv::Point(10, image.rows - 40),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        cv::putText(image, "Press 'c' to capture, 'q' when done",
                   cv::Point(10, image.rows - 15),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
    } else {
        cv::putText(image, "Ready to calibrate! Press 'q' to compute",
                   cv::Point(10, image.rows - 15),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }
}

cv::Mat CameraCalibrator::generate_board_image(const CalibrationBoardConfig& config,
                                                int pixels_per_square) {
    auto dictionary = cv::aruco::getPredefinedDictionary(config.dictionary_id);
    auto board = cv::aruco::CharucoBoard::create(
        config.squares_x,
        config.squares_y,
        config.square_length,
        config.marker_length,
        dictionary);

    cv::Mat board_image;
    board->draw(cv::Size(config.squares_x * pixels_per_square,
                         config.squares_y * pixels_per_square),
                board_image, 20, 1);

    return board_image;
}

// =============================================================================
// Calibration Utilities
// =============================================================================

namespace calibration_utils {

cv::Mat undistort_image(const cv::Mat& image, const CameraIntrinsics& intrinsics) {
    if (!intrinsics.valid() || image.empty()) {
        return image.clone();
    }

    cv::Mat undistorted;
    cv::undistort(image, undistorted,
                  intrinsics.camera_matrix, intrinsics.dist_coeffs);
    return undistorted;
}

std::vector<cv::Point2f> undistort_points(
    const std::vector<cv::Point2f>& points,
    const CameraIntrinsics& intrinsics)
{
    if (!intrinsics.valid() || points.empty()) {
        return points;
    }

    std::vector<cv::Point2f> undistorted;
    cv::undistortPoints(points, undistorted,
                       intrinsics.camera_matrix, intrinsics.dist_coeffs,
                       cv::noArray(), intrinsics.camera_matrix);
    return undistorted;
}

cv::Mat get_optimal_camera_matrix(
    const CameraIntrinsics& intrinsics,
    cv::Size image_size,
    double alpha)
{
    if (!intrinsics.valid()) {
        return cv::Mat();
    }

    return cv::getOptimalNewCameraMatrix(
        intrinsics.camera_matrix, intrinsics.dist_coeffs,
        image_size, alpha);
}

CalibrationQualityMetrics compute_quality_metrics(
    const CalibrationResult& result,
    const std::vector<std::vector<cv::Point2f>>& all_corners,
    cv::Size image_size) {

    CalibrationQualityMetrics metrics;

    if (!result.success || result.per_frame_errors.empty()) {
        metrics.quality_level = "poor";
        metrics.confidence = 0.0;
        metrics.warnings.push_back("Calibration failed or no per-frame errors available");
        return metrics;
    }

    const auto& intr = result.intrinsics;

    // Basic error metrics
    metrics.rms_error = result.rms_error;
    metrics.per_frame_errors = result.per_frame_errors;

    // Find max per-frame error
    metrics.max_per_frame_error = 0.0;
    for (double err : result.per_frame_errors) {
        if (err > metrics.max_per_frame_error) {
            metrics.max_per_frame_error = err;
        }
    }

    // Focal length asymmetry
    double avg_focal = (intr.fx + intr.fy) / 2.0;
    metrics.focal_length_asymmetry = std::abs(intr.fx - intr.fy) / avg_focal;

    // Principal point offset from center
    double cx_center = image_size.width / 2.0;
    double cy_center = image_size.height / 2.0;
    double dx = intr.cx - cx_center;
    double dy = intr.cy - cy_center;
    metrics.principal_point_offset = std::sqrt(dx * dx + dy * dy);

    // Spatial coverage analysis (3x3 grid)
    metrics.corner_coverage_grid.fill(0);

    int grid_width = image_size.width / 3;
    int grid_height = image_size.height / 3;

    for (const auto& frame_corners : all_corners) {
        for (const auto& corner : frame_corners) {
            int grid_x = static_cast<int>(corner.x / grid_width);
            int grid_y = static_cast<int>(corner.y / grid_height);

            // Clamp to valid grid indices
            grid_x = std::max(0, std::min(2, grid_x));
            grid_y = std::max(0, std::min(2, grid_y));

            int grid_idx = grid_y * 3 + grid_x;
            metrics.corner_coverage_grid[grid_idx]++;
        }
    }

    // Find min/max coverage
    metrics.min_corners_per_region = metrics.corner_coverage_grid[0];
    metrics.max_corners_per_region = metrics.corner_coverage_grid[0];

    for (int count : metrics.corner_coverage_grid) {
        if (count < metrics.min_corners_per_region) {
            metrics.min_corners_per_region = count;
        }
        if (count > metrics.max_corners_per_region) {
            metrics.max_corners_per_region = count;
        }
    }

    // Check if all regions have at least 3 samples
    metrics.has_good_coverage = true;
    for (int count : metrics.corner_coverage_grid) {
        if (count < 3) {
            metrics.has_good_coverage = false;
            break;
        }
    }

    // Quality level assessment based on RMS error
    if (metrics.rms_error < 0.3) {
        metrics.quality_level = "excellent";
        metrics.confidence = 0.95;
    } else if (metrics.rms_error < 0.5) {
        metrics.quality_level = "good";
        metrics.confidence = 0.85;
    } else if (metrics.rms_error < 1.0) {
        metrics.quality_level = "acceptable";
        metrics.confidence = 0.70;
    } else {
        metrics.quality_level = "poor";
        metrics.confidence = 0.50;
    }

    // Adjust confidence based on spatial coverage
    if (!metrics.has_good_coverage) {
        metrics.confidence *= 0.8;
        metrics.warnings.push_back("Insufficient spatial coverage in some image regions");
    }

    // Check for focal length issues
    double fx_expected = image_size.width * 0.8;
    if (intr.fx < fx_expected * 0.5 || intr.fx > fx_expected * 2.0) {
        metrics.warnings.push_back("Unusual focal length: " + std::to_string(intr.fx));
        metrics.confidence *= 0.9;
    }

    // Check for asymmetry
    if (metrics.focal_length_asymmetry > 0.05) {
        metrics.warnings.push_back("High focal length asymmetry: " +
                                   std::to_string(metrics.focal_length_asymmetry * 100) + "%");
        metrics.confidence *= 0.95;
    }

    // Check principal point offset
    double max_offset = image_size.width * 0.1;
    if (metrics.principal_point_offset > max_offset) {
        metrics.warnings.push_back("Principal point far from center: " +
                                   std::to_string(metrics.principal_point_offset) + " pixels");
        metrics.confidence *= 0.95;
    }

    // Recommendations
    if (metrics.rms_error > 0.5) {
        metrics.recommendations.push_back("Capture more frames with better lighting");
        metrics.recommendations.push_back("Ensure calibration board is flat and in focus");
    }

    if (!metrics.has_good_coverage) {
        metrics.recommendations.push_back("Capture frames with board in all regions of image");
        metrics.recommendations.push_back("Include tilted views and different distances");
    }

    if (metrics.max_per_frame_error > 2.0) {
        metrics.recommendations.push_back("Review frames with high error (>" +
                                         std::to_string(metrics.max_per_frame_error) + "px)");
    }

    return metrics;
}

bool validate_calibration(const CalibrationResult& result) {
    if (!result.success) {
        std::cerr << "[Calibration] Validation FAILED: Calibration was not successful" << std::endl;
        return false;
    }

    const auto& metrics = result.quality_metrics;
    const auto& intr = result.intrinsics;

    // Print comprehensive quality report
    std::cout << "\n========== Calibration Quality Report ==========" << std::endl;
    std::cout << "Quality Level: " << metrics.quality_level << std::endl;
    std::cout << "Confidence: " << std::fixed << std::setprecision(1)
              << (metrics.confidence * 100.0) << "%" << std::endl;
    std::cout << "\nError Metrics:" << std::endl;
    std::cout << "  RMS Error: " << std::setprecision(3) << metrics.rms_error << " pixels" << std::endl;
    std::cout << "  Max Frame Error: " << metrics.max_per_frame_error << " pixels" << std::endl;
    std::cout << "  Frames Used: " << result.frames_used << std::endl;

    std::cout << "\nIntrinsic Parameters:" << std::endl;
    std::cout << "  Focal Length: fx=" << std::setprecision(2) << intr.fx
              << ", fy=" << intr.fy << std::endl;
    std::cout << "  Asymmetry: " << std::setprecision(1)
              << (metrics.focal_length_asymmetry * 100.0) << "%" << std::endl;
    std::cout << "  Principal Point: (" << std::setprecision(1)
              << intr.cx << ", " << intr.cy << ")" << std::endl;
    std::cout << "  Offset from Center: " << std::setprecision(1)
              << metrics.principal_point_offset << " pixels" << std::endl;

    std::cout << "\nSpatial Coverage (3x3 grid):" << std::endl;
    for (int row = 0; row < 3; row++) {
        std::cout << "  ";
        for (int col = 0; col < 3; col++) {
            int count = metrics.corner_coverage_grid[row * 3 + col];
            std::cout << std::setw(4) << count << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "  Min/Max per region: " << metrics.min_corners_per_region
              << " / " << metrics.max_corners_per_region << std::endl;
    std::cout << "  Good Coverage: " << (metrics.has_good_coverage ? "YES" : "NO") << std::endl;

    // Print warnings
    if (!metrics.warnings.empty()) {
        std::cout << "\nWarnings:" << std::endl;
        for (const auto& warning : metrics.warnings) {
            std::cout << "  ⚠ " << warning << std::endl;
        }
    }

    // Print recommendations
    if (!metrics.recommendations.empty()) {
        std::cout << "\nRecommendations:" << std::endl;
        for (const auto& rec : metrics.recommendations) {
            std::cout << "  → " << rec << std::endl;
        }
    }

    std::cout << "================================================\n" << std::endl;

    // Determine pass/fail
    bool pass = true;

    if (metrics.rms_error > 1.5) {
        std::cerr << "[Calibration] FAIL: RMS error too high (>" << metrics.rms_error << ")" << std::endl;
        pass = false;
    }

    if (!metrics.has_good_coverage) {
        std::cerr << "[Calibration] FAIL: Insufficient spatial coverage" << std::endl;
        pass = false;
    }

    if (metrics.confidence < 0.6) {
        std::cerr << "[Calibration] FAIL: Overall confidence too low ("
                  << (metrics.confidence * 100) << "%)" << std::endl;
        pass = false;
    }

    if (pass) {
        std::cout << "[Calibration] ✓ Validation PASSED" << std::endl;
    } else {
        std::cout << "[Calibration] ✗ Validation FAILED" << std::endl;
    }

    return pass;
}

bool validate_at_distance(
    const CameraIntrinsics& intrinsics,
    double tag_size_m,
    double expected_distance_m,
    const std::vector<cv::Point2f>& detected_corners,
    double max_error_cm) {

    if (detected_corners.size() != 4) {
        std::cerr << "[Calibration] Validation failed: Need exactly 4 corners, got "
                  << detected_corners.size() << std::endl;
        return false;
    }

    if (!intrinsics.valid()) {
        std::cerr << "[Calibration] Validation failed: Invalid intrinsics" << std::endl;
        return false;
    }

    // Compute distance using pinhole model (average of edge lengths)
    double edge_lengths[4];
    edge_lengths[0] = cv::norm(detected_corners[1] - detected_corners[0]); // bottom edge
    edge_lengths[1] = cv::norm(detected_corners[2] - detected_corners[1]); // right edge
    edge_lengths[2] = cv::norm(detected_corners[3] - detected_corners[2]); // top edge
    edge_lengths[3] = cv::norm(detected_corners[0] - detected_corners[3]); // left edge

    double avg_edge_pixels = (edge_lengths[0] + edge_lengths[1] +
                               edge_lengths[2] + edge_lengths[3]) / 4.0;

    double focal_avg = (intrinsics.fx + intrinsics.fy) / 2.0;
    double estimated_distance = (tag_size_m * focal_avg) / avg_edge_pixels;

    // Compute distance error
    double distance_error_m = std::abs(estimated_distance - expected_distance_m);
    double distance_error_cm = distance_error_m * 100.0;

    // Compute angle consistency (edges should be roughly equal length)
    double vertical_avg = (edge_lengths[1] + edge_lengths[3]) / 2.0;
    double horizontal_avg = (edge_lengths[0] + edge_lengths[2]) / 2.0;
    double edge_ratio = std::max(vertical_avg, horizontal_avg) /
                        std::min(vertical_avg, horizontal_avg);
    double angle_error_deg = (edge_ratio - 1.0) * 45.0;  // Rough approximation

    // Print validation results
    std::cout << "\n========== Distance Validation Report ==========" << std::endl;
    std::cout << "Expected Distance: " << std::fixed << std::setprecision(2)
              << expected_distance_m << " m" << std::endl;
    std::cout << "Measured Distance: " << estimated_distance << " m" << std::endl;
    std::cout << "Distance Error: " << std::setprecision(1)
              << distance_error_cm << " cm" << std::endl;
    std::cout << "Angle Consistency: " << std::setprecision(1)
              << angle_error_deg << "°" << std::endl;
    std::cout << "Edge Lengths: [" << std::setprecision(1)
              << edge_lengths[0] << ", " << edge_lengths[1] << ", "
              << edge_lengths[2] << ", " << edge_lengths[3] << "] pixels" << std::endl;

    bool pass = (distance_error_cm <= max_error_cm);

    if (pass) {
        std::cout << "[Calibration] ✓ Distance validation PASSED" << std::endl;
    } else {
        std::cout << "[Calibration] ✗ Distance validation FAILED" << std::endl;
        std::cout << "  Error " << distance_error_cm << " cm exceeds maximum "
                  << max_error_cm << " cm" << std::endl;
    }

    std::cout << "================================================\n" << std::endl;

    return pass;
}

}  // namespace calibration_utils

}  // namespace frc_vision
