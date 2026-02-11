#pragma once
/**
 * @file types.hpp
 * @brief Core data types for FRC Vision Coprocessor
 *
 * All timestamps use std::chrono::steady_clock for latency measurements
 * and std::chrono::system_clock for absolute time synchronization.
 */

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

namespace frc_vision {

// =============================================================================
// Time Types
// =============================================================================

using SteadyClock = std::chrono::steady_clock;
using SystemClock = std::chrono::system_clock;
using SteadyTimePoint = SteadyClock::time_point;
using SystemTimePoint = SystemClock::time_point;
using Duration = std::chrono::duration<double>;

/**
 * @brief Timestamp bundle for latency tracking
 */
struct Timestamps {
    SteadyTimePoint capture;      // When frame was captured (monotonic)
    SteadyTimePoint detect_start; // When detection started
    SteadyTimePoint detect_end;   // When detection completed
    SteadyTimePoint pose_end;     // When pose estimation completed
    SteadyTimePoint publish;      // When published to NT/web
    SystemTimePoint capture_wall; // Wall clock at capture for roboRIO sync

    // Latency breakdown in milliseconds
    double capture_to_detect_ms() const {
        return std::chrono::duration<double, std::milli>(detect_start - capture).count();
    }
    double detect_ms() const {
        return std::chrono::duration<double, std::milli>(detect_end - detect_start).count();
    }
    double pose_ms() const {
        return std::chrono::duration<double, std::milli>(pose_end - detect_end).count();
    }
    double total_pipeline_ms() const {
        return std::chrono::duration<double, std::milli>(publish - capture).count();
    }
};

// =============================================================================
// Detection Types
// =============================================================================

/**
 * @brief 2D point in image coordinates
 */
struct Point2D {
    double x = 0.0;
    double y = 0.0;

    Point2D() = default;
    Point2D(double x_, double y_) : x(x_), y(y_) {}

    cv::Point2d to_cv() const { return cv::Point2d(x, y); }
    static Point2D from_cv(const cv::Point2d& p) { return {p.x, p.y}; }
};

/**
 * @brief 3D point in world/robot coordinates
 */
struct Point3D {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Point3D() = default;
    Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    cv::Point3d to_cv() const { return cv::Point3d(x, y, z); }
    static Point3D from_cv(const cv::Point3d& p) { return {p.x, p.y, p.z}; }
};

/**
 * @brief Quaternion for rotation representation
 */
struct Quaternion {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Quaternion() = default;
    Quaternion(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}

    // Convert from rotation vector (Rodrigues)
    static Quaternion from_rvec(const cv::Vec3d& rvec);
    cv::Vec3d to_rvec() const;
};

/**
 * @brief 6DoF pose (position + orientation)
 */
struct Pose3D {
    Point3D position;
    Quaternion orientation;

    // Alternative representations
    cv::Vec3d rvec() const { return orientation.to_rvec(); }
    cv::Vec3d tvec() const { return cv::Vec3d(position.x, position.y, position.z); }

    static Pose3D from_rvec_tvec(const cv::Vec3d& rvec, const cv::Vec3d& tvec);

    // Compose transformations
    Pose3D inverse() const;
    Pose3D compose(const Pose3D& other) const;
};

/**
 * @brief 2D robot pose on field (x, y, theta)
 */
struct Pose2D {
    double x = 0.0;     // meters
    double y = 0.0;     // meters
    double theta = 0.0; // radians

    Pose2D() = default;
    Pose2D(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
};

/**
 * @brief Corner points of a detected AprilTag (counter-clockwise from bottom-left)
 */
struct TagCorners {
    std::array<Point2D, 4> corners;

    Point2D center() const {
        return {(corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0,
                (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4.0};
    }

    double area() const {
        // Shoelace formula
        double a = 0.0;
        for (int i = 0; i < 4; i++) {
            int j = (i + 1) % 4;
            a += corners[i].x * corners[j].y;
            a -= corners[j].x * corners[i].y;
        }
        return std::abs(a) / 2.0;
    }
};

/**
 * @brief Multi-method distance estimation results (Phase 1)
 */
struct DistanceEstimate {
    double distance_pnp = 0.0;        // Distance from PnP solve
    double distance_pinhole = 0.0;    // Distance from pinhole model
    double distance_edge = 0.0;       // Distance from tag edge length
    double distance_fused = 0.0;      // Weighted fusion of methods
    bool is_consistent = false;       // All methods agree within threshold
    double confidence = 0.0;          // Overall confidence (0-1)
    double stddev = 0.0;              // Standard deviation of estimates
};

/**
 * @brief Per-tag accuracy prediction (Phase 2)
 */
struct TagAccuracyEstimate {
    double estimated_error_m = 0.0;           // Predicted position error (meters)
    double confidence = 1.0;                   // Confidence in this tag (0-1)
    double reprojection_contribution = 0.0;    // Error from reprojection
    double viewing_angle_contribution = 0.0;   // Error from viewing angle
    double ambiguity_contribution = 0.0;       // Error from pose ambiguity
    double distance_contribution = 0.0;        // Error from distance
};

/**
 * @brief Single AprilTag detection
 */
struct TagDetection {
    int id = -1;                 // Tag ID
    TagCorners corners;          // Image coordinates of corners
    double decision_margin = 0;  // Detection quality (higher = better)
    int hamming = 0;             // Bit errors corrected (0 = perfect)

    // Pose estimation results (filled after pose solve)
    bool pose_valid = false;
    Pose3D tag_to_camera;        // Transform from tag frame to camera frame
    double reprojection_error = std::numeric_limits<double>::max();
    double distance_m = 0.0;     // Distance to tag in meters (from PnP or pinhole)
    double ambiguity = 1.0;      // Pose ambiguity ratio (0=unambiguous, 1=highly ambiguous)

    // Phase 1 enhancements
    DistanceEstimate distance_estimate;      // Multi-method distance validation
    TagAccuracyEstimate accuracy_estimate;   // Predicted accuracy

    // For tracking
    int track_id = -1;           // Persistent track ID
    int frames_tracked = 0;
    double confidence = 0.0;
};

/**
 * @brief Quality metrics for pose estimation
 */
struct PoseQuality {
    int tag_count = 0;
    double avg_margin = 0;
    double avg_reproj_error = 0;
    double confidence = 0;        // 0-1 confidence score

    // Standard deviations for WPILib SwerveDrivePoseEstimator.
    // These tell the robot's Kalman filter how much to trust vision vs odometry.
    double std_dev_x = 0.1;       // meters
    double std_dev_y = 0.1;       // meters
    double std_dev_theta = 0.05;  // radians
};

/**
 * @brief Detection result for a single frame
 */
struct FrameDetections {
    int camera_id = -1;
    uint64_t frame_number = 0;
    Timestamps timestamps;

    std::vector<TagDetection> detections;

    // Multi-tag pose (if >= 2 tags visible)
    bool multi_tag_pose_valid = false;
    Pose3D camera_to_field;      // Camera pose in field frame
    Pose2D robot_pose_field;     // Robot pose on field (2D)
    double multi_tag_reproj_error = 0;
    int tags_used_for_pose = 0;

    // Pre-computed quality (set by pipeline, consumed by NT publisher)
    PoseQuality quality;

    // Quality metrics
    double avg_margin() const {
        if (detections.empty())
            return 0;
        double sum = 0;
        for (const auto& d : detections)
            sum += d.decision_margin;
        return sum / detections.size();
    }

    double avg_reproj_error() const {
        if (detections.empty())
            return 0;
        double sum = 0;
        int count = 0;
        for (const auto& d : detections) {
            if (d.pose_valid) {
                sum += d.reprojection_error;
                count++;
            }
        }
        return count > 0 ? sum / count : 0;
    }
};

// =============================================================================
// Frame Types
// =============================================================================

/**
 * @brief Raw camera frame with metadata
 */
struct Frame {
    cv::Mat image;               // BGR or grayscale
    int camera_id = -1;
    uint64_t frame_number = 0;
    SteadyTimePoint capture_time;
    SystemTimePoint capture_wall_time;

    bool empty() const { return image.empty(); }
};

/**
 * @brief Processed frame with detections and JPEG for streaming
 */
struct ProcessedFrame {
    int camera_id = -1;
    uint64_t frame_number = 0;
    Timestamps timestamps;

    cv::Mat annotated_image;     // Image with overlays drawn
    std::vector<uint8_t> jpeg;   // JPEG-encoded for web streaming
    FrameDetections detections;
};

// =============================================================================
// Fused Output Types
// =============================================================================

/**
 * @brief Fused robot pose from all cameras
 */
struct FusedPose {
    bool valid = false;
    Pose2D pose_raw;             // Raw fused pose
    Pose2D pose_filtered;        // Filtered/smoothed pose
    PoseQuality quality;
    Timestamps timestamps;

    int cameras_contributing = 0;
    int total_tags = 0;
};

// =============================================================================
// Configuration Types
// =============================================================================

/**
 * @brief Camera configuration
 */
struct CameraConfig {
    std::string name = "cam0";
    std::string device = "/dev/video0";
    int width = 640;
    int height = 480;
    int fps = 30;                // 30fps balances detection rate vs CPU
    int exposure = -1;           // -1 = auto
    int gain = -1;               // -1 = auto
    std::string format = "MJPG"; // MJPG or YUYV
    std::string intrinsics_file;
    Pose3D camera_to_robot;      // Extrinsic transform
};

/**
 * @brief Camera intrinsic parameters
 */
struct CameraIntrinsics {
    cv::Mat camera_matrix;       // 3x3
    cv::Mat dist_coeffs;         // 5x1 or 8x1
    int width = 0;
    int height = 0;

    // Individual parameters for convenience
    double fx = 0;  // Focal length X
    double fy = 0;  // Focal length Y
    double cx = 0;  // Principal point X
    double cy = 0;  // Principal point Y

    bool valid() const {
        return !camera_matrix.empty() && camera_matrix.rows == 3 && fx > 0;
    }

    // Extract fx, fy, cx, cy from camera matrix
    void update_from_matrix() {
        if (!camera_matrix.empty() && camera_matrix.rows == 3) {
            fx = camera_matrix.at<double>(0, 0);
            fy = camera_matrix.at<double>(1, 1);
            cx = camera_matrix.at<double>(0, 2);
            cy = camera_matrix.at<double>(1, 2);
        }
    }

    /**
     * @brief Create default intrinsics for uncalibrated cameras.
     * Uses typical USB webcam parameters (~65 degree HFOV).
     * Good enough for approximate pose estimation until proper calibration.
     */
    static CameraIntrinsics create_default(int w, int h) {
        CameraIntrinsics intr;
        intr.width = w;
        intr.height = h;

        // Typical USB webcam: ~65° horizontal FOV
        // f = width / (2 * tan(hfov/2)) = width / (2 * tan(32.5°))
        double f = w / (2.0 * std::tan(32.5 * M_PI / 180.0));

        intr.camera_matrix = (cv::Mat_<double>(3, 3) <<
            f, 0, w / 2.0,
            0, f, h / 2.0,
            0, 0, 1.0);

        intr.dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

        intr.fx = f;
        intr.fy = f;
        intr.cx = w / 2.0;
        intr.cy = h / 2.0;

        return intr;
    }
};

// =============================================================================
// Calibration Quality & Distance Estimation (Phase 1 Enhancements)
// =============================================================================

/**
 * @brief Comprehensive calibration quality metrics
 */
struct CalibrationQualityMetrics {
    double rms_error = 0.0;                    // Overall RMS reprojection error (pixels)
    double max_per_frame_error = 0.0;          // Maximum error in any single frame
    std::vector<double> per_frame_errors;      // Error distribution

    double focal_length_asymmetry = 0.0;       // |fx - fy| / avg(fx, fy)
    double principal_point_offset = 0.0;       // Distance from image center (pixels)

    // Spatial coverage analysis (3x3 grid)
    std::array<int, 9> corner_coverage_grid;   // Corner count per region
    int min_corners_per_region = 0;
    int max_corners_per_region = 0;
    bool has_good_coverage = false;            // All regions have >= 3 samples

    // Quality assessment
    std::string quality_level = "unknown";     // "excellent", "good", "acceptable", "poor"
    double confidence = 0.0;                   // 0-1 overall confidence score

    // Recommendations
    std::vector<std::string> warnings;
    std::vector<std::string> recommendations;
};

/**
 * @brief Multi-method distance estimation with geometric consistency checking
 */
struct DistanceEstimate {
    double distance_pnp = 0.0;                 // From PnP translation vector norm
    double distance_pinhole = 0.0;             // From pinhole model (avg edge length)
    double distance_vertical_edges = 0.0;      // From vertical edges only
    double distance_horizontal_edges = 0.0;    // From horizontal edges only

    double distance_fused = 0.0;               // Final fused estimate
    double confidence = 0.0;                   // 0-1 based on agreement between methods

    // Consistency metrics
    double pnp_pinhole_agreement = 0.0;        // exp(-|pnp - pinhole| / 0.5)
    double edge_consistency = 0.0;             // exp(-|vert - horiz| / 0.3)

    bool is_consistent = false;                // True if all methods agree within tolerance
};

/**
 * @brief Per-tag accuracy estimation based on viewing conditions
 */
struct TagAccuracyEstimate {
    double estimated_error_m = 0.0;            // Predicted absolute distance error (meters)
    double estimated_angle_error_deg = 0.0;    // Predicted angle error (degrees)

    std::string confidence_level = "unknown";  // "high", "medium", "low"

    // Contributing factors
    double base_error = 0.0;                   // 0.01 * distance^2
    double reprojection_contribution = 0.0;    // reproj_err * 0.005 * distance
    double viewing_angle_contribution = 0.0;   // viewing_angle_factor * 0.01 * distance
    double ambiguity_contribution = 0.0;       // ambiguity * 0.02 * distance
};

/**
 * @brief AprilTag detector configuration
 */
struct DetectorConfig {
    std::string family = "tag36h11";
    int decimation = 2;          // Downsampling factor (1-4)
    double sigma = 0.0;          // Gaussian blur sigma (0 = off)
    int nthreads = 4;            // Detection threads
    bool refine_edges = true;    // Subpixel edge refinement (AprilTag library)
    bool subpixel_refine = true; // Additional OpenCV cornerSubPix refinement (Phase 1)
    int max_hamming = 1;         // Max bit errors to accept
    double min_margin = 20.0;    // Minimum decision margin
    int max_tags_per_frame = 16; // Limit detections per frame
};

/**
 * @brief Tracking configuration
 */
struct TrackerConfig {
    bool enable = true;
    int dropout_ms = 150;        // Max time to maintain track without detection
    double filter_alpha = 0.3;   // EMA filter coefficient (0-1)
    bool roi_enable = false;     // Use predicted ROI for next frame
    double velocity_decay = 0.9; // Velocity decay per frame
};

/**
 * @brief Calibration quality thresholds and validation mode
 */
struct CalibrationConfig {
    // Quality thresholds (RMS reprojection error in pixels)
    double min_rms_for_excellent = 0.3;
    double min_rms_for_good = 0.5;
    double min_rms_for_acceptable = 1.0;

    // Spatial coverage requirements
    bool require_spatial_coverage = true;
    int min_samples_per_region = 3;  // For 3x3 grid

    // Validation mode from set distance
    bool validation_mode_enable = false;
    double validation_distance_m = 1.5;       // Distance to place tag for validation
    int validation_frames_required = 30;      // Frames to collect
    double max_distance_error_cm = 2.0;       // Maximum acceptable error in cm
    double max_angle_error_deg = 2.0;         // Maximum acceptable angle error in degrees
};

/**
 * @brief Field tag layout (single tag)
 */
struct FieldTag {
    int id = -1;
    std::array<Point3D, 4> corners_field;  // Corners in field frame
    Point3D center_field;
    Pose3D pose_field;           // Tag pose in field frame

    // Get tag corner positions for PnP
    std::vector<cv::Point3d> get_corners_cv() const {
        std::vector<cv::Point3d> pts;
        for (const auto& c : corners_field)
            pts.push_back(c.to_cv());
        return pts;
    }
};

/**
 * @brief Full field layout
 */
struct FieldLayout {
    std::string name = "FRC 2026";
    double tag_size_m = 0.1651;  // 6.5 inches
    std::unordered_map<int, FieldTag> tags;

    bool has_tag(int id) const { return tags.count(id) > 0; }
    const FieldTag& get_tag(int id) const { return tags.at(id); }
};

/**
 * @brief Output configuration
 */
struct OutputConfig {
    bool nt_enable = true;
    std::string nt_server = "10.0.0.2"; // roboRIO IP
    std::string nt_table_root = "/FRCVision";
    int web_port = 5800;
    int publish_rate_hz = 50;
};

/**
 * @brief Performance tuning
 */
struct PerformanceConfig {
    int jpeg_quality = 70;
    bool enable_annotations = true;
    int annotation_line_width = 2;
};

/**
 * @brief Master configuration
 */
struct Config {
    std::vector<CameraConfig> cameras;
    DetectorConfig detector;
    TrackerConfig tracker;
    CalibrationConfig calibration;
    FieldLayout field;
    OutputConfig output;
    PerformanceConfig performance;

    // Reload flag
    std::atomic<bool> reload_requested{false};
};

// =============================================================================
// System Status
// =============================================================================

/**
 * @brief Per-camera status
 */
struct CameraStatus {
    int camera_id = -1;
    bool connected = false;
    double fps = 0;
    uint64_t frames_captured = 0;
    uint64_t frames_dropped = 0;
    double avg_detect_ms = 0;
    double avg_pose_ms = 0;
    double avg_total_ms = 0;
    int tags_detected = 0;
};

/**
 * @brief System-wide status
 */
struct SystemStatus {
    SteadyTimePoint start_time;
    double uptime_seconds = 0;
    double cpu_temp = 0;         // Celsius (if available)
    double cpu_usage = 0;        // Percent

    std::vector<CameraStatus> cameras;

    // Fused pose stats
    double fused_fps = 0;
    bool fused_pose_valid = false;
};

// =============================================================================
// Utility Functions (inline implementations)
// =============================================================================

inline Quaternion Quaternion::from_rvec(const cv::Vec3d& rvec) {
    double angle = cv::norm(rvec);
    if (angle < 1e-10) {
        return Quaternion(1, 0, 0, 0);
    }
    double half_angle = angle / 2.0;
    double s = std::sin(half_angle) / angle;
    return Quaternion(std::cos(half_angle), rvec[0] * s, rvec[1] * s, rvec[2] * s);
}

inline cv::Vec3d Quaternion::to_rvec() const {
    double angle = 2.0 * std::acos(std::clamp(w, -1.0, 1.0));
    double s = std::sqrt(1.0 - w * w);
    if (s < 1e-10) {
        return cv::Vec3d(0, 0, 0);
    }
    return cv::Vec3d(x / s * angle, y / s * angle, z / s * angle);
}

inline Pose3D Pose3D::from_rvec_tvec(const cv::Vec3d& rvec, const cv::Vec3d& tvec) {
    Pose3D p;
    p.position = {tvec[0], tvec[1], tvec[2]};
    p.orientation = Quaternion::from_rvec(rvec);
    return p;
}

inline Pose3D Pose3D::inverse() const {
    // For a transform T that takes points from frame A to frame B,
    // T.inverse() takes points from frame B to frame A
    cv::Mat R;
    cv::Rodrigues(rvec(), R);
    cv::Mat R_inv = R.t();
    cv::Vec3d rvec_inv;
    cv::Rodrigues(R_inv, rvec_inv);

    cv::Mat t(tvec());
    cv::Mat t_inv = -R_inv * t;

    return from_rvec_tvec(rvec_inv, cv::Vec3d(t_inv.at<double>(0), t_inv.at<double>(1), t_inv.at<double>(2)));
}

inline Pose3D Pose3D::compose(const Pose3D& other) const {
    cv::Mat R1, R2;
    cv::Rodrigues(rvec(), R1);
    cv::Rodrigues(other.rvec(), R2);

    cv::Mat R_composed = R1 * R2;
    cv::Vec3d rvec_composed;
    cv::Rodrigues(R_composed, rvec_composed);

    cv::Mat t1(tvec());
    cv::Mat t2(other.tvec());
    cv::Mat t_composed = R1 * t2 + t1;

    return from_rvec_tvec(rvec_composed,
                         cv::Vec3d(t_composed.at<double>(0), t_composed.at<double>(1), t_composed.at<double>(2)));
}

} // namespace frc_vision
