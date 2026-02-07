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

bool validate_calibration(const CalibrationResult& result) {
    if (!result.success) {
        return false;
    }

    // Check RMS error
    if (result.rms_error > 1.5) {
        std::cerr << "[Calibration] Warning: High RMS error: " << result.rms_error << std::endl;
    }

    // Check focal length is reasonable
    const auto& intr = result.intrinsics;
    double aspect = static_cast<double>(intr.width) / intr.height;
    double fx_expected = intr.width * 0.8;  // Rough estimate for typical cameras

    if (intr.fx < fx_expected * 0.5 || intr.fx > fx_expected * 2.0) {
        std::cerr << "[Calibration] Warning: Unusual focal length: " << intr.fx << std::endl;
    }

    // Check principal point is near center
    double cx_expected = intr.width / 2.0;
    double cy_expected = intr.height / 2.0;

    if (std::abs(intr.cx - cx_expected) > intr.width * 0.1 ||
        std::abs(intr.cy - cy_expected) > intr.height * 0.1) {
        std::cerr << "[Calibration] Warning: Principal point far from center" << std::endl;
    }

    return true;
}

}  // namespace calibration_utils

}  // namespace frc_vision
