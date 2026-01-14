#pragma once
/**
 * @file field_renderer.hpp
 * @brief Synthetic AprilTag field rendering for simulator
 *
 * Renders AprilTags as they would appear from the simulated robot's camera,
 * with proper perspective projection based on camera intrinsics/extrinsics.
 */

#include "sim_types.hpp"
#include "../types.hpp"
#include <opencv2/core.hpp>
#include <unordered_map>

namespace frc_vision {
namespace sim {

/**
 * @brief Renders synthetic camera views of AprilTags on the field
 */
class FieldRenderer {
public:
    FieldRenderer();

    /**
     * @brief Initialize with field layout and camera parameters
     */
    void initialize(
        const FieldLayout& field,
        const CameraIntrinsics& intrinsics,
        const Pose3D& camera_to_robot,
        const SyntheticCameraParams& params);

    /**
     * @brief Render synthetic view from robot pose
     * @param robot_pose Robot pose in field frame
     * @return Rendered image with AprilTags
     */
    cv::Mat render(const Pose2D& robot_pose);

    /**
     * @brief Composite synthetic tags onto webcam frame
     * @param webcam_frame Base webcam frame
     * @param robot_pose Robot pose in field frame
     * @param mode 0 = overlay, 1 = picture-in-picture
     * @return Composited frame
     */
    cv::Mat composite(const cv::Mat& webcam_frame, const Pose2D& robot_pose, int mode = 0);

    /**
     * @brief Get visible tag IDs from current pose
     */
    std::vector<int> get_visible_tags(const Pose2D& robot_pose) const;

    /**
     * @brief Enable/disable motion blur simulation
     */
    void set_motion_blur(bool enabled, int kernel_size = 5);

    /**
     * @brief Enable/disable noise simulation
     */
    void set_noise(bool enabled, double stddev = 5.0);

private:
    /**
     * @brief Generate AprilTag pattern for given ID
     */
    cv::Mat generate_tag_pattern(int tag_id);

    /**
     * @brief Project 3D point to image coordinates
     */
    cv::Point2d project_point(const cv::Point3d& world_pt, const cv::Mat& R, const cv::Mat& t) const;

    /**
     * @brief Check if tag is visible from camera
     */
    bool is_tag_visible(const FieldTag& tag, const cv::Mat& R, const cv::Mat& t) const;

    /**
     * @brief Render single tag onto image
     */
    void render_tag(cv::Mat& image, const FieldTag& tag, const cv::Mat& R, const cv::Mat& t);

    /**
     * @brief Get camera pose from robot pose
     */
    void get_camera_pose(const Pose2D& robot_pose, cv::Mat& R, cv::Mat& t) const;

    FieldLayout field_;
    CameraIntrinsics intrinsics_;
    Pose3D camera_to_robot_;
    SyntheticCameraParams params_;

    // Pre-computed camera matrix
    cv::Mat K_;
    cv::Mat dist_coeffs_;

    // Cached tag patterns (36h11 encoding)
    std::unordered_map<int, cv::Mat> tag_patterns_;

    // Motion blur / noise settings
    bool motion_blur_enabled_ = false;
    int motion_blur_size_ = 5;
    bool noise_enabled_ = false;
    double noise_stddev_ = 5.0;
};

/**
 * @brief Renders top-down field visualization
 */
class TopDownRenderer {
public:
    TopDownRenderer();

    /**
     * @brief Initialize with field and visualization params
     */
    void initialize(const FieldLayout& field, const VisualizationParams& params);

    /**
     * @brief Render top-down view
     * @param state Current robot state
     * @param fov_angle Camera field of view (radians)
     */
    cv::Mat render(const RobotState& state, double fov_angle = 1.2);

    /**
     * @brief Draw overlay text with stats
     */
    void draw_stats(cv::Mat& image,
                   double fps, double latency_ms,
                   int tag_count, double reproj_error,
                   double confidence, int target_id,
                   bool auto_align_active);

    /**
     * @brief Set visualization flags
     */
    void set_show_true_pose(bool show) { show_true_pose_ = show; }
    void set_show_odom_pose(bool show) { show_odom_pose_ = show; }
    void set_show_fused_pose(bool show) { show_fused_pose_ = show; }

private:
    cv::Point2i field_to_screen(double x, double y) const;
    void draw_robot(cv::Mat& image, const Pose2D& pose, const cv::Scalar& color, bool fill = false);
    void draw_fov_cone(cv::Mat& image, const Pose2D& pose, double fov, double length, const cv::Scalar& color);
    void draw_tag_marker(cv::Mat& image, const FieldTag& tag);

    FieldLayout field_;
    VisualizationParams params_;

    // Field image dimensions
    int image_width_ = 0;
    int image_height_ = 0;
    double offset_x_ = 0;
    double offset_y_ = 0;

    // Visualization flags
    bool show_true_pose_ = true;
    bool show_odom_pose_ = true;
    bool show_fused_pose_ = true;
};

} // namespace sim
} // namespace frc_vision
