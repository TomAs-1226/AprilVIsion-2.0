/**
 * @file field_renderer.cpp
 * @brief Synthetic field rendering implementation
 */

#include "field_renderer.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <random>

namespace frc_vision {
namespace sim {

// ============================================================================
// Tag36h11 bit pattern lookup (simplified - just corner IDs for visualization)
// For a complete implementation, this would include full 36h11 encoding
// ============================================================================

// Generate tag pattern for 36h11 family
// Tag pattern is 10x10 grid with 8x8 data region
static const int TAG_GRID_SIZE = 10;
static const int TAG_DATA_SIZE = 8;

// ============================================================================
// FieldRenderer Implementation
// ============================================================================

FieldRenderer::FieldRenderer() = default;

void FieldRenderer::initialize(
    const FieldLayout& field,
    const CameraIntrinsics& intrinsics,
    const Pose3D& camera_to_robot,
    const SyntheticCameraParams& params) {

    field_ = field;
    intrinsics_ = intrinsics;
    camera_to_robot_ = camera_to_robot;
    params_ = params;

    // Build camera matrix
    K_ = cv::Mat::eye(3, 3, CV_64F);
    K_.at<double>(0, 0) = params.fx;
    K_.at<double>(1, 1) = params.fy;
    K_.at<double>(0, 2) = params.cx;
    K_.at<double>(1, 2) = params.cy;

    if (intrinsics.valid()) {
        K_ = intrinsics.camera_matrix.clone();
        dist_coeffs_ = intrinsics.dist_coeffs.clone();
    } else {
        dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
    }

    // Pre-generate tag patterns for visible tags
    for (const auto& [id, tag] : field_.tags) {
        tag_patterns_[id] = generate_tag_pattern(id);
    }
}

cv::Mat FieldRenderer::generate_tag_pattern(int tag_id) {
    // Create 10x10 pattern for tag36h11
    // Black border (1 cell), white border (1 cell), 6x6 data
    cv::Mat pattern(TAG_GRID_SIZE, TAG_GRID_SIZE, CV_8UC1, cv::Scalar(255));

    // Outer black border
    cv::rectangle(pattern, cv::Point(0, 0), cv::Point(9, 9), cv::Scalar(0), -1);

    // Inner white border
    cv::rectangle(pattern, cv::Point(1, 1), cv::Point(8, 8), cv::Scalar(255), -1);

    // Data region with encoded bits (simplified visualization)
    // Real tag36h11 would have proper bit encoding
    std::mt19937 rng(tag_id);  // Deterministic pattern per tag ID
    for (int y = 2; y < 8; y++) {
        for (int x = 2; x < 8; x++) {
            // Simple pattern based on tag ID
            int bit_idx = (y - 2) * 6 + (x - 2);
            bool is_black = ((tag_id >> (bit_idx % 10)) ^ (bit_idx / 10)) & 1;
            pattern.at<uint8_t>(y, x) = is_black ? 0 : 255;
        }
    }

    return pattern;
}

void FieldRenderer::get_camera_pose(const Pose2D& robot_pose, cv::Mat& R, cv::Mat& t) const {
    // Robot pose in field frame
    double rx = robot_pose.x;
    double ry = robot_pose.y;
    double rtheta = robot_pose.theta;

    // Camera-to-robot transform
    const auto& c2r = camera_to_robot_;

    // Build robot rotation matrix (around Z axis)
    cv::Mat R_robot = cv::Mat::eye(3, 3, CV_64F);
    R_robot.at<double>(0, 0) = std::cos(rtheta);
    R_robot.at<double>(0, 1) = -std::sin(rtheta);
    R_robot.at<double>(1, 0) = std::sin(rtheta);
    R_robot.at<double>(1, 1) = std::cos(rtheta);

    // Camera position relative to robot in field frame
    cv::Mat cam_offset_robot(3, 1, CV_64F);
    cam_offset_robot.at<double>(0) = c2r.position.x;
    cam_offset_robot.at<double>(1) = c2r.position.y;
    cam_offset_robot.at<double>(2) = c2r.position.z;

    cv::Mat cam_offset_field = R_robot * cam_offset_robot;

    // Camera position in field
    t = cv::Mat(3, 1, CV_64F);
    t.at<double>(0) = rx + cam_offset_field.at<double>(0);
    t.at<double>(1) = ry + cam_offset_field.at<double>(1);
    t.at<double>(2) = cam_offset_field.at<double>(2);

    // Camera rotation: robot rotation + camera-to-robot rotation
    cv::Mat R_cam_to_robot;
    cv::Rodrigues(c2r.rvec(), R_cam_to_robot);

    R = R_robot * R_cam_to_robot;

    // For camera extrinsics, we need world-to-camera transform
    R = R.t();  // Transpose for world-to-camera
    t = -R * t;
}

cv::Point2d FieldRenderer::project_point(const cv::Point3d& world_pt, const cv::Mat& R, const cv::Mat& t) const {
    // Transform to camera frame
    cv::Mat pw(3, 1, CV_64F);
    pw.at<double>(0) = world_pt.x;
    pw.at<double>(1) = world_pt.y;
    pw.at<double>(2) = world_pt.z;

    cv::Mat pc = R * pw + t;

    double x = pc.at<double>(0);
    double y = pc.at<double>(1);
    double z = pc.at<double>(2);

    if (z <= 0.1) {
        return cv::Point2d(-1, -1);  // Behind camera
    }

    // Project to image
    double u = K_.at<double>(0, 0) * x / z + K_.at<double>(0, 2);
    double v = K_.at<double>(1, 1) * y / z + K_.at<double>(1, 2);

    return cv::Point2d(u, v);
}

bool FieldRenderer::is_tag_visible(const FieldTag& tag, const cv::Mat& R, const cv::Mat& t) const {
    // Check if tag center is in front of camera
    cv::Point2d center_proj = project_point(tag.center_field.to_cv(), R, t);

    // Allow tags that are partially off-screen (more generous bounds)
    if (center_proj.x < -300 || center_proj.x > params_.width + 300 ||
        center_proj.y < -300 || center_proj.y > params_.height + 300) {
        return false;
    }

    // Check distance to tag - don't render if too far (> 15 meters)
    cv::Mat t_cam = -R.t() * t;  // Camera position in world
    double dx = tag.center_field.x - t_cam.at<double>(0);
    double dy = tag.center_field.y - t_cam.at<double>(1);
    double dz = tag.center_field.z - t_cam.at<double>(2);
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (distance > 15.0) {
        return false;
    }

    // Check if tag is facing camera (dot product of normal with view direction)
    // Tag normal is the Z axis of the tag frame
    cv::Mat tag_rvec(3, 1, CV_64F);
    tag_rvec.at<double>(0) = tag.pose_field.rvec()[0];
    tag_rvec.at<double>(1) = tag.pose_field.rvec()[1];
    tag_rvec.at<double>(2) = tag.pose_field.rvec()[2];

    cv::Mat R_tag;
    cv::Rodrigues(tag_rvec, R_tag);

    // Tag normal in world frame (Z axis of tag)
    cv::Mat normal_world = R_tag.col(2);

    // View direction (from camera to tag center in world frame)
    cv::Mat view_dir(3, 1, CV_64F);
    view_dir.at<double>(0) = dx;
    view_dir.at<double>(1) = dy;
    view_dir.at<double>(2) = dz;
    view_dir /= distance;

    double dot = normal_world.dot(view_dir);

    // Tag is visible if normal points towards camera (dot < 0 means facing camera)
    // Allow viewing from up to ~75 degrees from perpendicular
    return dot < 0.7;
}

void FieldRenderer::render_tag(cv::Mat& image, const FieldTag& tag, const cv::Mat& R, const cv::Mat& t) {
    // Get tag corners in image
    std::vector<cv::Point2f> img_corners;
    int behind_camera = 0;
    for (const auto& corner : tag.corners_field) {
        cv::Point2d proj = project_point(corner.to_cv(), R, t);
        if (proj.x < 0) {
            behind_camera++;
            // Approximate corner position for partially visible tags
            proj.x = params_.width / 2;
            proj.y = params_.height / 2;
        }
        img_corners.push_back(cv::Point2f(static_cast<float>(proj.x), static_cast<float>(proj.y)));
    }

    // If more than 2 corners are behind camera, skip rendering
    if (behind_camera > 2) return;

    // Get source pattern
    auto it = tag_patterns_.find(tag.id);
    if (it == tag_patterns_.end()) return;

    const cv::Mat& pattern = it->second;

    // Create larger pattern for better quality warping
    cv::Mat pattern_large;
    cv::resize(pattern, pattern_large, cv::Size(100, 100), 0, 0, cv::INTER_NEAREST);

    // Source corners (pattern corners)
    std::vector<cv::Point2f> src_corners = {
        cv::Point2f(0, 99),    // Bottom-left
        cv::Point2f(99, 99),   // Bottom-right
        cv::Point2f(99, 0),    // Top-right
        cv::Point2f(0, 0)      // Top-left
    };

    // Compute homography and warp
    cv::Mat H = cv::getPerspectiveTransform(src_corners, img_corners);

    // Create mask for the tag region
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
    std::vector<cv::Point> poly;
    for (const auto& pt : img_corners) {
        poly.push_back(cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)));
    }
    cv::fillConvexPoly(mask, poly, cv::Scalar(255));

    // Warp pattern onto image
    cv::Mat warped;
    cv::warpPerspective(pattern_large, warped, H, image.size());

    // Convert grayscale pattern to BGR
    cv::Mat warped_bgr;
    cv::cvtColor(warped, warped_bgr, cv::COLOR_GRAY2BGR);

    // Blend into image
    warped_bgr.copyTo(image, mask);

    // Draw tag ID text
    cv::Point2d center = project_point(tag.center_field.to_cv(), R, t);
    if (center.x >= 0) {
        std::string id_text = std::to_string(tag.id);
        cv::putText(image, id_text,
                   cv::Point(static_cast<int>(center.x) - 10, static_cast<int>(center.y) + 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
    }
}

cv::Mat FieldRenderer::render(const Pose2D& robot_pose) {
    // Create blank image
    cv::Mat image(params_.height, params_.width, CV_8UC3, cv::Scalar(40, 40, 40));

    // Get camera pose
    cv::Mat R, t;
    get_camera_pose(robot_pose, R, t);

    // Render each visible tag
    for (const auto& [id, tag] : field_.tags) {
        if (is_tag_visible(tag, R, t)) {
            render_tag(image, tag, R, t);
        }
    }

    // Add noise if enabled
    if (noise_enabled_) {
        cv::Mat noise(image.size(), CV_8SC3);
        cv::randn(noise, cv::Scalar(0, 0, 0), cv::Scalar(noise_stddev_, noise_stddev_, noise_stddev_));
        image += noise;
    }

    // Add motion blur if enabled
    if (motion_blur_enabled_) {
        cv::GaussianBlur(image, image, cv::Size(motion_blur_size_, motion_blur_size_), 0);
    }

    return image;
}

cv::Mat FieldRenderer::composite(const cv::Mat& webcam_frame, const Pose2D& robot_pose, int mode) {
    cv::Mat result;

    // Render synthetic view
    cv::Mat synthetic = render(robot_pose);

    if (mode == 0) {
        // Overlay mode: blend synthetic on top of webcam where tags are
        result = webcam_frame.clone();

        // Create mask for non-background pixels in synthetic
        cv::Mat gray;
        cv::cvtColor(synthetic, gray, cv::COLOR_BGR2GRAY);
        cv::Mat mask;
        cv::threshold(gray, mask, 45, 255, cv::THRESH_BINARY);

        // Dilate mask slightly for better blending
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        // Copy synthetic onto result where mask is set
        synthetic.copyTo(result, mask);

    } else {
        // Picture-in-picture mode
        result = webcam_frame.clone();

        // Resize synthetic to corner
        int pip_width = params_.width / 3;
        int pip_height = params_.height / 3;
        cv::Mat pip_view;
        cv::resize(synthetic, pip_view, cv::Size(pip_width, pip_height));

        // Place in top-right corner with border
        int x_offset = result.cols - pip_width - 10;
        int y_offset = 10;

        cv::rectangle(result,
                     cv::Point(x_offset - 2, y_offset - 2),
                     cv::Point(x_offset + pip_width + 2, y_offset + pip_height + 2),
                     cv::Scalar(255, 255, 255), 2);

        pip_view.copyTo(result(cv::Rect(x_offset, y_offset, pip_width, pip_height)));
    }

    return result;
}

std::vector<int> FieldRenderer::get_visible_tags(const Pose2D& robot_pose) const {
    cv::Mat R, t;
    const_cast<FieldRenderer*>(this)->get_camera_pose(robot_pose, R, t);

    std::vector<int> visible;
    for (const auto& [id, tag] : field_.tags) {
        if (is_tag_visible(tag, R, t)) {
            visible.push_back(id);
        }
    }
    return visible;
}

void FieldRenderer::set_motion_blur(bool enabled, int kernel_size) {
    motion_blur_enabled_ = enabled;
    motion_blur_size_ = kernel_size | 1;  // Ensure odd
}

void FieldRenderer::set_noise(bool enabled, double stddev) {
    noise_enabled_ = enabled;
    noise_stddev_ = stddev;
}

// ============================================================================
// TopDownRenderer Implementation
// ============================================================================

TopDownRenderer::TopDownRenderer() = default;

void TopDownRenderer::initialize(const FieldLayout& field, const VisualizationParams& params) {
    field_ = field;
    params_ = params;

    // FRC field dimensions (approximate for Crescendo 2024)
    // Field is about 16.5m x 8.2m
    double field_width = 16.54;   // meters
    double field_height = 8.21;   // meters

    offset_x_ = params_.field_margin;
    offset_y_ = params_.field_margin;

    image_width_ = static_cast<int>(field_width * params_.field_view_scale) + 2 * params_.field_margin;
    image_height_ = static_cast<int>(field_height * params_.field_view_scale) + 2 * params_.field_margin;
}

cv::Point2i TopDownRenderer::field_to_screen(double x, double y) const {
    int sx = static_cast<int>(x * params_.field_view_scale + offset_x_);
    int sy = static_cast<int>((8.21 - y) * params_.field_view_scale + offset_y_);  // Flip Y
    return cv::Point2i(sx, sy);
}

void TopDownRenderer::draw_robot(cv::Mat& image, const Pose2D& pose, const cv::Scalar& color, bool fill) {
    // Robot corners in robot frame
    double half_l = RobotState::LENGTH / 2;
    double half_w = RobotState::WIDTH / 2;

    std::vector<cv::Point2d> corners_robot = {
        {half_l, half_w},    // Front-left
        {half_l, -half_w},   // Front-right
        {-half_l, -half_w},  // Back-right
        {-half_l, half_w}    // Back-left
    };

    // Transform to field frame
    double cos_t = std::cos(pose.theta);
    double sin_t = std::sin(pose.theta);

    std::vector<cv::Point> corners_screen;
    for (const auto& c : corners_robot) {
        double fx = pose.x + c.x * cos_t - c.y * sin_t;
        double fy = pose.y + c.x * sin_t + c.y * cos_t;
        corners_screen.push_back(field_to_screen(fx, fy));
    }

    // Draw robot polygon
    if (fill) {
        cv::fillConvexPoly(image, corners_screen, color);
    } else {
        cv::polylines(image, corners_screen, true, color, 2);
    }

    // Draw front indicator (triangle)
    cv::Point front_center = field_to_screen(
        pose.x + half_l * 0.7 * cos_t,
        pose.y + half_l * 0.7 * sin_t
    );
    cv::circle(image, front_center, 5, color, -1);
}

void TopDownRenderer::draw_fov_cone(cv::Mat& image, const Pose2D& pose, double fov, double length, const cv::Scalar& color) {
    double half_fov = fov / 2;

    // FOV cone edges
    double left_angle = pose.theta + half_fov;
    double right_angle = pose.theta - half_fov;

    cv::Point robot_pos = field_to_screen(pose.x, pose.y);
    cv::Point left_edge = field_to_screen(
        pose.x + length * std::cos(left_angle),
        pose.y + length * std::sin(left_angle)
    );
    cv::Point right_edge = field_to_screen(
        pose.x + length * std::cos(right_angle),
        pose.y + length * std::sin(right_angle)
    );

    // Draw cone
    std::vector<cv::Point> cone = {robot_pos, left_edge, right_edge};
    cv::fillConvexPoly(image, cone, cv::Scalar(color[0] * 0.3, color[1] * 0.3, color[2] * 0.3));
    cv::line(image, robot_pos, left_edge, color, 1);
    cv::line(image, robot_pos, right_edge, color, 1);
}

void TopDownRenderer::draw_tag_marker(cv::Mat& image, const FieldTag& tag) {
    cv::Point center = field_to_screen(tag.center_field.x, tag.center_field.y);

    // Draw tag as small rectangle
    int size = 8;
    cv::rectangle(image,
                 cv::Point(center.x - size, center.y - size),
                 cv::Point(center.x + size, center.y + size),
                 params_.tag_color, -1);

    // Draw tag ID
    cv::putText(image, std::to_string(tag.id),
               cv::Point(center.x - 5, center.y + 4),
               cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 0, 0), 1);
}

cv::Mat TopDownRenderer::render(const RobotState& state, double fov_angle) {
    // Create dark background
    cv::Mat image(image_height_, image_width_, CV_8UC3, cv::Scalar(30, 30, 30));

    // Draw field boundary
    cv::rectangle(image,
                 field_to_screen(0, 8.21),
                 field_to_screen(16.54, 0),
                 cv::Scalar(60, 60, 60), 2);

    // Draw center line
    cv::line(image,
            field_to_screen(8.27, 0),
            field_to_screen(8.27, 8.21),
            cv::Scalar(50, 50, 50), 1);

    // Draw all tags
    for (const auto& [id, tag] : field_.tags) {
        draw_tag_marker(image, tag);
    }

    // Draw FOV cones first (behind robots)
    if (params_.show_fov_cone) {
        if (show_true_pose_) {
            draw_fov_cone(image, state.true_pose, fov_angle, params_.fov_cone_length, params_.true_pose_color);
        }
    }

    // Draw robots
    if (show_odom_pose_) {
        draw_robot(image, state.odom_pose, params_.odom_pose_color, false);
    }
    if (show_fused_pose_) {
        draw_robot(image, state.fused_pose, params_.fused_pose_color, false);
    }
    if (show_true_pose_) {
        draw_robot(image, state.true_pose, params_.true_pose_color, true);
    }

    // Draw legend
    int legend_y = image_height_ - 80;
    cv::putText(image, "True Pose", cv::Point(10, legend_y), cv::FONT_HERSHEY_SIMPLEX, 0.4, params_.true_pose_color, 1);
    cv::putText(image, "Odom Pose", cv::Point(10, legend_y + 15), cv::FONT_HERSHEY_SIMPLEX, 0.4, params_.odom_pose_color, 1);
    cv::putText(image, "Fused Pose", cv::Point(10, legend_y + 30), cv::FONT_HERSHEY_SIMPLEX, 0.4, params_.fused_pose_color, 1);

    return image;
}

void TopDownRenderer::draw_stats(cv::Mat& image,
                                 double fps, double latency_ms,
                                 int tag_count, double reproj_error,
                                 double confidence, int target_id,
                                 bool auto_align_active) {
    int x = 10;
    int y = 20;
    int line_height = 18;

    auto put_stat = [&](const std::string& text, const cv::Scalar& color = cv::Scalar(255, 255, 255)) {
        cv::putText(image, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1);
        y += line_height;
    };

    put_stat("FPS: " + std::to_string(static_cast<int>(fps)));
    put_stat("Latency: " + std::to_string(static_cast<int>(latency_ms)) + " ms");
    put_stat("Tags: " + std::to_string(tag_count));
    put_stat("Reproj: " + std::to_string(reproj_error).substr(0, 5) + " px");
    put_stat("Confidence: " + std::to_string(static_cast<int>(confidence * 100)) + "%");

    if (target_id >= 0) {
        put_stat("Target: Tag " + std::to_string(target_id), cv::Scalar(0, 255, 255));
    }

    if (auto_align_active) {
        put_stat("AUTO-ALIGN ACTIVE", cv::Scalar(0, 255, 0));
    }
}

} // namespace sim
} // namespace frc_vision
