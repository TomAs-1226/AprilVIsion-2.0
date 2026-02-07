/**
 * @file calibration.hpp
 * @brief Camera calibration using ChArUco boards
 *
 * ChArUco boards combine the robustness of ArUco markers with
 * the accuracy of chessboard corners for camera calibration.
 */

#pragma once

#include "types.hpp"
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <vector>
#include <string>
#include <atomic>
#include <mutex>

namespace frc_vision {

/**
 * @brief Calibration board configuration
 */
struct CalibrationBoardConfig {
    int squares_x = 5;           // Chessboard squares in X
    int squares_y = 7;           // Chessboard squares in Y
    float square_length = 0.04f; // Square side length in meters (40mm)
    float marker_length = 0.02f; // ArUco marker side length in meters (20mm)
    int dictionary_id = cv::aruco::DICT_4X4_50;  // ArUco dictionary
};

/**
 * @brief Result of a single calibration frame
 */
struct CalibrationFrame {
    int frame_number;
    int corners_detected;
    int markers_detected;
    double reproj_error;
    bool valid;
    cv::Mat image;  // Original image for review
};

/**
 * @brief Camera calibration result
 */
struct CalibrationResult {
    bool success = false;
    CameraIntrinsics intrinsics;
    double rms_error = 0.0;
    int frames_used = 0;
    std::string error_message;

    // Per-frame errors for analysis
    std::vector<double> per_frame_errors;
};

/**
 * @brief Camera calibrator using ChArUco boards
 */
class CameraCalibrator {
public:
    CameraCalibrator();
    ~CameraCalibrator();

    /**
     * @brief Initialize with board configuration
     */
    void initialize(const CalibrationBoardConfig& config);

    /**
     * @brief Process a calibration frame
     * @param image Input image
     * @return Frame result with detection info
     */
    CalibrationFrame process_frame(const cv::Mat& image);

    /**
     * @brief Add a frame to the calibration set
     * @param image Input image
     * @return true if frame was added successfully
     */
    bool capture_frame(const cv::Mat& image);

    /**
     * @brief Get number of captured frames
     */
    int captured_frame_count() const { return captured_frames_; }

    /**
     * @brief Compute calibration from captured frames
     * @param image_size Image dimensions
     * @return Calibration result
     */
    CalibrationResult compute_calibration(cv::Size image_size);

    /**
     * @brief Save calibration to YAML file
     */
    bool save_calibration(const std::string& path, const CalibrationResult& result);

    /**
     * @brief Clear all captured frames
     */
    void clear();

    /**
     * @brief Draw detected board on image (for visualization)
     */
    void draw_board(cv::Mat& image);

    /**
     * @brief Generate a printable calibration board image
     */
    static cv::Mat generate_board_image(const CalibrationBoardConfig& config,
                                         int pixels_per_square = 100);

    /**
     * @brief Get minimum recommended frames
     */
    static int min_frames() { return 10; }

    /**
     * @brief Get recommended frames for good calibration
     */
    static int recommended_frames() { return 20; }

private:
    CalibrationBoardConfig config_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::CharucoBoard> board_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

    // Captured calibration data
    std::vector<std::vector<cv::Point2f>> all_charuco_corners_;
    std::vector<std::vector<int>> all_charuco_ids_;
    std::atomic<int> captured_frames_{0};

    // Last detection (for visualization)
    std::vector<cv::Point2f> last_corners_;
    std::vector<int> last_ids_;
    std::vector<std::vector<cv::Point2f>> last_marker_corners_;
    std::vector<int> last_marker_ids_;

    mutable std::mutex mutex_;
};

/**
 * @brief Utility functions for calibration
 */
namespace calibration_utils {

/**
 * @brief Undistort image using calibration
 */
cv::Mat undistort_image(const cv::Mat& image, const CameraIntrinsics& intrinsics);

/**
 * @brief Undistort points using calibration
 */
std::vector<cv::Point2f> undistort_points(
    const std::vector<cv::Point2f>& points,
    const CameraIntrinsics& intrinsics);

/**
 * @brief Compute optimal new camera matrix for undistortion
 */
cv::Mat get_optimal_camera_matrix(
    const CameraIntrinsics& intrinsics,
    cv::Size image_size,
    double alpha = 0.0);  // 0 = crop to valid, 1 = keep all

/**
 * @brief Validate calibration quality
 */
bool validate_calibration(const CalibrationResult& result);

}  // namespace calibration_utils

}  // namespace frc_vision
