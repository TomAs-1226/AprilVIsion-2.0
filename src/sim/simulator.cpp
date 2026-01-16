/**
 * @file simulator.cpp
 * @brief Simulator implementation with PathPlanner and field selection
 */

#include "simulator.hpp"
#include "../field_layout.hpp"
#include "../detector.hpp"
#include "../pose.hpp"
#include "../tracker.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <filesystem>

namespace fs = std::filesystem;

namespace frc_vision {
namespace sim {

Simulator::Simulator()
    : last_update_time_(Clock::now())
    , last_render_time_(Clock::now())
    , key_hold_time_(Clock::now())
    , fps_start_time_(Clock::now()) {

    // Initialize path
    current_path_.name = "UserPath";
    current_path_.max_velocity = 4.0;
    current_path_.max_acceleration = 3.0;
}

Simulator::~Simulator() {
    shutdown_requested_.store(true);

    if (webcam_thread_.joinable()) {
        webcam_running_.store(false);
        webcam_thread_.join();
    }

    if (webcam_.isOpened()) {
        webcam_.release();
    }

    cv::destroyAllWindows();
}

bool Simulator::initialize(const std::string& config_path) {
    std::cout << "========================================" << std::endl;
    std::cout << "FRC Vision Mac Simulator v2.1" << std::endl;
    std::cout << "With PathPlanner & Multi-Game Support" << std::endl;
    std::cout << "========================================" << std::endl;

    // Load configuration
    if (!load_config(config_path)) {
        return false;
    }

    // Load field layout
    if (!load_field_layout()) {
        return false;
    }

    // Load camera calibration
    if (!load_camera_calibration()) {
        std::cerr << "[Sim] Warning: Using default camera calibration" << std::endl;
    }

    // Initialize webcam (optional)
    if (sim_config_.use_webcam) {
        if (!initialize_webcam()) {
            std::cerr << "[Sim] Warning: Webcam not available, using synthetic only" << std::endl;
            sim_config_.use_webcam = false;
        }
    }

    // Initialize robot dynamics
    dynamics_.initialize(sim_config_.dynamics, sim_config_.start_pose);

    // Initialize renderers
    field_renderer_.initialize(field_, intrinsics_, camera_to_robot_, sim_config_.camera);
    topdown_renderer_.initialize(field_, sim_config_.visualization);

    // Initialize auto-align
    auto_align_.initialize(sim_config_.auto_align, field_);

    // Initialize scoring system
    scoring_.initialize(current_game_year_, Alliance::BLUE);

    // Initialize path planner
    path_planner_.set_field_layout(field_);
    SwerveConfig swerve_cfg;
    path_planner_.set_swerve_config(swerve_cfg);

    // Initialize vision pipeline
    if (!initialize_vision_pipeline()) {
        return false;
    }

    // Create windows
    cv::namedWindow(CAMERA_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(FIELD_WINDOW, cv::WINDOW_AUTOSIZE);

    // Set mouse callback for field window
    cv::setMouseCallback(FIELD_WINDOW, mouse_callback, this);

    // Position windows
    cv::moveWindow(CAMERA_WINDOW, 50, 50);
    cv::moveWindow(FIELD_WINDOW, 750, 50);

    print_keybinds();

    return true;
}

bool Simulator::load_config(const std::string& path) {
    config_dir_ = fs::path(path).parent_path().string();
    if (config_dir_.empty()) config_dir_ = ".";

    if (!fs::exists(path)) {
        std::cerr << "[Sim] Config file not found: " << path << std::endl;
        std::cerr << "[Sim] Using default configuration" << std::endl;
        sim_config_ = SimConfig();
        return true;
    }

    try {
        YAML::Node config = YAML::LoadFile(path);

        // Game year selection
        if (config["game_year"]) {
            std::string year_str = config["game_year"].as<std::string>();
            current_game_year_ = game_year_from_string(year_str);
        }

        // Dynamics
        if (config["dynamics"]) {
            auto d = config["dynamics"];
            if (d["max_speed"]) sim_config_.dynamics.max_speed = d["max_speed"].as<double>();
            if (d["max_turbo_speed"]) sim_config_.dynamics.max_turbo_speed = d["max_turbo_speed"].as<double>();
            if (d["max_rotation"]) sim_config_.dynamics.max_rotation = d["max_rotation"].as<double>();
            if (d["odom_noise"]) sim_config_.dynamics.odom_trans_noise = d["odom_noise"].as<double>();
        }

        // Auto-align
        if (config["auto_align"]) {
            auto a = config["auto_align"];
            if (a["pos_kp"]) sim_config_.auto_align.pos_kp = a["pos_kp"].as<double>();
            if (a["heading_kp"]) sim_config_.auto_align.heading_kp = a["heading_kp"].as<double>();
            if (a["standoff_distance"]) sim_config_.auto_align.standoff_distance = a["standoff_distance"].as<double>();
        }

        // Camera
        if (config["camera"]) {
            auto c = config["camera"];
            if (c["width"]) sim_config_.camera.width = c["width"].as<int>();
            if (c["height"]) sim_config_.camera.height = c["height"].as<int>();
            if (c["add_noise"]) sim_config_.camera.add_noise = c["add_noise"].as<bool>();
            if (c["motion_blur"]) sim_config_.camera.motion_blur = c["motion_blur"].as<bool>();
        }

        // Start pose
        if (config["start_pose"]) {
            auto p = config["start_pose"];
            if (p["x"]) sim_config_.start_pose.x = p["x"].as<double>();
            if (p["y"]) sim_config_.start_pose.y = p["y"].as<double>();
            if (p["theta"]) sim_config_.start_pose.theta = p["theta"].as<double>();
        }

        // Paths
        if (config["field_layout"]) sim_config_.field_layout_path = config["field_layout"].as<std::string>();
        if (config["intrinsics"]) sim_config_.intrinsics_path = config["intrinsics"].as<std::string>();
        if (config["extrinsics"]) sim_config_.extrinsics_path = config["extrinsics"].as<std::string>();

        // Webcam
        if (config["webcam"]) {
            auto w = config["webcam"];
            if (w["device"]) sim_config_.webcam_device = w["device"].as<int>();
            if (w["enable"]) sim_config_.use_webcam = w["enable"].as<bool>();
        }

        std::cout << "[Sim] Loaded config from: " << path << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[Sim] Error loading config: " << e.what() << std::endl;
        return false;
    }
}

bool Simulator::load_field_layout() {
    return load_field_for_year(current_game_year_);
}

bool Simulator::load_field_for_year(GameYear year) {
    std::string filename;
    switch (year) {
        case GameYear::CRESCENDO_2024:
            filename = "2024-crescendo.json";
            field_length_ = 16.541;
            field_width_ = 8.211;
            break;
        case GameYear::REEFSCAPE_2025:
            filename = "2025-reefscape.json";
            field_length_ = 17.548;
            field_width_ = 8.052;
            break;
        case GameYear::REBUILT_2026:
        default:
            filename = "2026-rebuilt.json";
            field_length_ = 16.541;
            field_width_ = 8.069;
            break;
    }

    // Try to find the file
    fs::path layout_path = fs::path(config_dir_) / ".." / "assets" / filename;
    if (!fs::exists(layout_path)) {
        layout_path = "assets/" + filename;
    }
    if (!fs::exists(layout_path)) {
        layout_path = fs::path(config_dir_) / sim_config_.field_layout_path;
    }

    if (!fs::exists(layout_path)) {
        std::cerr << "[Sim] Field layout not found: " << filename << std::endl;
        return false;
    }

    try {
        std::ifstream file(layout_path);
        nlohmann::json json;
        file >> json;

        field_.tags.clear();
        field_.name = json.value("name", "FRC Field");
        field_.tag_size_m = 0.1651;  // 6.5 inches

        double half_size = field_.tag_size_m / 2.0;

        for (const auto& tag_json : json["tags"]) {
            int id = tag_json.contains("ID") ? tag_json["ID"].get<int>() : tag_json["id"].get<int>();
            FieldTag tag;
            tag.id = id;

            // Parse translation (support both formats)
            double tx, ty, tz;
            if (tag_json["pose"].contains("translation")) {
                tx = tag_json["pose"]["translation"]["x"];
                ty = tag_json["pose"]["translation"]["y"];
                tz = tag_json["pose"]["translation"]["z"];
            } else {
                tx = tag_json["pose"]["x"];
                ty = tag_json["pose"]["y"];
                tz = tag_json["pose"]["z"];
            }

            tag.center_field = {tx, ty, tz};

            // Parse quaternion rotation
            double qw, qx, qy, qz;
            if (tag_json["pose"].contains("rotation") && tag_json["pose"]["rotation"].contains("quaternion")) {
                qw = tag_json["pose"]["rotation"]["quaternion"]["W"];
                qx = tag_json["pose"]["rotation"]["quaternion"]["X"];
                qy = tag_json["pose"]["rotation"]["quaternion"]["Y"];
                qz = tag_json["pose"]["rotation"]["quaternion"]["Z"];
            } else {
                qw = 1.0; qx = 0; qy = 0; qz = 0;
            }

            // Convert quaternion to rotation matrix
            cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
            R.at<double>(0, 0) = 1 - 2 * (qy * qy + qz * qz);
            R.at<double>(0, 1) = 2 * (qx * qy - qz * qw);
            R.at<double>(0, 2) = 2 * (qx * qz + qy * qw);
            R.at<double>(1, 0) = 2 * (qx * qy + qz * qw);
            R.at<double>(1, 1) = 1 - 2 * (qx * qx + qz * qz);
            R.at<double>(1, 2) = 2 * (qy * qz - qx * qw);
            R.at<double>(2, 0) = 2 * (qx * qz - qy * qw);
            R.at<double>(2, 1) = 2 * (qy * qz + qx * qw);
            R.at<double>(2, 2) = 1 - 2 * (qx * qx + qy * qy);

            cv::Vec3d rvec;
            cv::Rodrigues(R, rvec);
            tag.pose_field = Pose3D::from_rvec_tvec(rvec, cv::Vec3d(tx, ty, tz));

            // Compute corners in field frame
            std::array<cv::Point3d, 4> corners_local = {
                cv::Point3d(-half_size, -half_size, 0),
                cv::Point3d(half_size, -half_size, 0),
                cv::Point3d(half_size, half_size, 0),
                cv::Point3d(-half_size, half_size, 0)
            };

            for (int i = 0; i < 4; i++) {
                cv::Mat pt = (cv::Mat_<double>(3, 1) <<
                    corners_local[i].x, corners_local[i].y, corners_local[i].z);
                cv::Mat pt_field = R * pt;
                tag.corners_field[i] = {
                    pt_field.at<double>(0) + tx,
                    pt_field.at<double>(1) + ty,
                    pt_field.at<double>(2) + tz
                };
            }

            field_.tags[id] = tag;
        }

        std::cout << "[Sim] Loaded " << field_.tags.size() << " AprilTags for "
                  << game_year_to_string(year) << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[Sim] Error loading field layout: " << e.what() << std::endl;
        return false;
    }
}

void Simulator::cycle_field() {
    switch (current_game_year_) {
        case GameYear::CRESCENDO_2024:
            set_game_year(GameYear::REEFSCAPE_2025);
            break;
        case GameYear::REEFSCAPE_2025:
            set_game_year(GameYear::REBUILT_2026);
            break;
        case GameYear::REBUILT_2026:
        default:
            set_game_year(GameYear::CRESCENDO_2024);
            break;
    }
}

void Simulator::set_game_year(GameYear year) {
    if (load_field_for_year(year)) {
        current_game_year_ = year;

        // Reinitialize components with new field
        field_renderer_.initialize(field_, intrinsics_, camera_to_robot_, sim_config_.camera);
        topdown_renderer_.initialize(field_, sim_config_.visualization);
        auto_align_.initialize(sim_config_.auto_align, field_);
        scoring_.initialize(year, Alliance::BLUE);
        path_planner_.set_field_layout(field_);

        if (pipeline_) {
            DetectorConfig det_cfg;
            det_cfg.family = "tag36h11";
            det_cfg.decimation = 2;
            TrackerConfig track_cfg;
            pipeline_->initialize(1, field_, det_cfg, track_cfg, 0.3);
        }

        // Clear path when changing fields
        clear_path();

        std::cout << "[Sim] Switched to " << game_year_to_string(year) << std::endl;
    }
}

bool Simulator::load_camera_calibration() {
    fs::path intr_path = fs::path(config_dir_) / sim_config_.intrinsics_path;
    if (!fs::exists(intr_path)) {
        intr_path = fs::path(config_dir_) / ".." / "assets" / "mac_cam_intrinsics.yml";
    }
    if (!fs::exists(intr_path)) {
        intr_path = "assets/mac_cam_intrinsics.yml";
    }

    if (fs::exists(intr_path)) {
        cv::FileStorage fs_intr(intr_path.string(), cv::FileStorage::READ);
        if (fs_intr.isOpened()) {
            fs_intr["camera_matrix"] >> intrinsics_.camera_matrix;
            fs_intr["distortion_coefficients"] >> intrinsics_.dist_coeffs;
            fs_intr["image_width"] >> intrinsics_.width;
            fs_intr["image_height"] >> intrinsics_.height;
        }
    }

    if (!intrinsics_.valid()) {
        intrinsics_.camera_matrix = (cv::Mat_<double>(3, 3) <<
            sim_config_.camera.fx, 0, sim_config_.camera.cx,
            0, sim_config_.camera.fy, sim_config_.camera.cy,
            0, 0, 1);
        intrinsics_.dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
        intrinsics_.width = sim_config_.camera.width;
        intrinsics_.height = sim_config_.camera.height;
    }

    // Load extrinsics
    fs::path extr_path = fs::path(config_dir_) / sim_config_.extrinsics_path;
    if (!fs::exists(extr_path)) {
        extr_path = fs::path(config_dir_) / ".." / "assets" / "mac_cam_extrinsics.yml";
    }

    if (fs::exists(extr_path)) {
        try {
            YAML::Node extr = YAML::LoadFile(extr_path.string());
            double x = extr["translation"]["x"].as<double>();
            double y = extr["translation"]["y"].as<double>();
            double z = extr["translation"]["z"].as<double>();
            double roll = extr["rotation"]["roll"].as<double>() * M_PI / 180.0;
            double pitch = extr["rotation"]["pitch"].as<double>() * M_PI / 180.0;
            double yaw = extr["rotation"]["yaw"].as<double>() * M_PI / 180.0;

            cv::Mat Rx = (cv::Mat_<double>(3, 3) << 1,0,0, 0,cos(roll),-sin(roll), 0,sin(roll),cos(roll));
            cv::Mat Ry = (cv::Mat_<double>(3, 3) << cos(pitch),0,sin(pitch), 0,1,0, -sin(pitch),0,cos(pitch));
            cv::Mat Rz = (cv::Mat_<double>(3, 3) << cos(yaw),-sin(yaw),0, sin(yaw),cos(yaw),0, 0,0,1);
            cv::Mat R = Rz * Ry * Rx;
            cv::Vec3d rvec;
            cv::Rodrigues(R, rvec);
            camera_to_robot_ = Pose3D::from_rvec_tvec(rvec, cv::Vec3d(x, y, z));
        } catch (...) {}
    }

    if (camera_to_robot_.position.x == 0 && camera_to_robot_.position.z == 0) {
        camera_to_robot_.position = {0.30, 0.0, 0.45};
        cv::Mat R = (cv::Mat_<double>(3, 3) << 1,0,0, 0,cos(0.26),-sin(0.26), 0,sin(0.26),cos(0.26));
        cv::Vec3d rvec;
        cv::Rodrigues(R, rvec);
        camera_to_robot_.orientation = Quaternion::from_rvec(rvec);
    }

    return true;
}

bool Simulator::initialize_webcam() {
    webcam_.open(sim_config_.webcam_device);
    if (!webcam_.isOpened()) return false;

    webcam_.set(cv::CAP_PROP_FRAME_WIDTH, sim_config_.camera.width);
    webcam_.set(cv::CAP_PROP_FRAME_HEIGHT, sim_config_.camera.height);
    webcam_.set(cv::CAP_PROP_FPS, sim_config_.camera.target_fps);

    webcam_running_.store(true);
    webcam_thread_ = std::thread([this]() {
        cv::Mat frame;
        while (webcam_running_.load()) {
            if (webcam_.read(frame)) {
                std::lock_guard<std::mutex> lock(webcam_mutex_);
                webcam_frame_ = frame.clone();
            }
        }
    });

    return true;
}

bool Simulator::initialize_vision_pipeline() {
    pipeline_ = std::make_unique<VisionPipeline>();

    DetectorConfig det_cfg;
    det_cfg.family = "tag36h11";
    det_cfg.decimation = 2;
    det_cfg.sigma = 0.0;
    det_cfg.nthreads = 4;
    det_cfg.refine_edges = true;
    det_cfg.min_margin = 20.0;

    TrackerConfig track_cfg;
    track_cfg.enable = true;
    track_cfg.dropout_ms = 150;
    track_cfg.filter_alpha = 0.3;

    pipeline_->initialize(1, field_, det_cfg, track_cfg, 0.3);
    return true;
}

void Simulator::print_keybinds() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "CONTROLS:" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  WASD     - Move robot" << std::endl;
    std::cout << "  Q/E      - Rotate CCW/CW" << std::endl;
    std::cout << "  SHIFT    - Turbo mode" << std::endl;
    std::cout << "  V        - Toggle auto-align" << std::endl;
    std::cout << "  R        - Reset robot pose" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "PATH PLANNER:" << std::endl;
    std::cout << "  P        - Toggle path edit mode (click field to add waypoints)" << std::endl;
    std::cout << "  X        - Execute path" << std::endl;
    std::cout << "  C        - Clear path" << std::endl;
    std::cout << "  Z        - Remove last waypoint" << std::endl;
    std::cout << "  G        - Generate code (prints to console)" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "FIELD:" << std::endl;
    std::cout << "  F        - Cycle field (2024->2025->2026)" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "DISPLAY:" << std::endl;
    std::cout << "  1-5      - Toggle overlays" << std::endl;
    std::cout << "  ESC      - Quit" << std::endl;
    std::cout << "========================================\n" << std::endl;
}

// Mouse callback for waypoint placement
void Simulator::mouse_callback(int event, int x, int y, int flags, void* userdata) {
    Simulator* sim = static_cast<Simulator*>(userdata);
    sim->handle_mouse(event, x, y, flags);
}

void Simulator::handle_mouse(int event, int x, int y, int flags) {
    (void)flags;

    if (mode_ != SimMode::PATH_EDIT) return;

    if (event == cv::EVENT_LBUTTONDOWN) {
        // Convert screen coords to field coords
        auto [fx, fy] = screen_to_field(x, y);

        // Validate bounds
        if (fx >= 0 && fx <= field_length_ && fy >= 0 && fy <= field_width_) {
            add_waypoint_at_click(fx, fy);
        }
    }
}

cv::Point2d Simulator::field_to_screen(double fx, double fy) const {
    double scale = sim_config_.visualization.field_view_scale;
    int margin = sim_config_.visualization.field_margin;

    double sx = margin + fx * scale;
    double sy = margin + (field_width_ - fy) * scale;  // Flip Y

    return cv::Point2d(sx, sy);
}

std::pair<double, double> Simulator::screen_to_field(int sx, int sy) const {
    double scale = sim_config_.visualization.field_view_scale;
    int margin = sim_config_.visualization.field_margin;

    double fx = (sx - margin) / scale;
    double fy = field_width_ - (sy - margin) / scale;  // Flip Y

    return {fx, fy};
}

void Simulator::add_waypoint_at_click(double field_x, double field_y) {
    PathWaypoint wp;
    wp.pose.x = field_x;
    wp.pose.y = field_y;

    // Calculate heading towards the next waypoint or use current robot heading
    if (current_path_.waypoints.empty()) {
        wp.pose.theta = dynamics_.state().true_pose.theta;
    } else {
        // Point towards the new waypoint from the previous one
        const auto& prev = current_path_.waypoints.back();
        wp.pose.theta = std::atan2(field_y - prev.pose.y, field_x - prev.pose.x);
    }

    wp.name = "WP" + std::to_string(current_path_.waypoints.size());

    current_path_.waypoints.push_back(wp);

    std::cout << "[Path] Added waypoint " << wp.name << " at ("
              << field_x << ", " << field_y << ")" << std::endl;
}

void Simulator::remove_last_waypoint() {
    if (!current_path_.waypoints.empty()) {
        std::cout << "[Path] Removed " << current_path_.waypoints.back().name << std::endl;
        current_path_.waypoints.pop_back();
    }
}

void Simulator::clear_path() {
    current_path_.waypoints.clear();
    current_path_.trajectory.clear();
    path_executing_ = false;
    path_time_ = 0;
    std::cout << "[Path] Cleared all waypoints" << std::endl;
}

void Simulator::execute_path() {
    if (current_path_.waypoints.size() < 2) {
        std::cout << "[Path] Need at least 2 waypoints to execute" << std::endl;
        return;
    }

    // Add start position as first waypoint if not already there
    auto& first_wp = current_path_.waypoints.front();
    auto robot_pose = dynamics_.state().true_pose;

    double dist = std::sqrt(std::pow(first_wp.pose.x - robot_pose.x, 2) +
                           std::pow(first_wp.pose.y - robot_pose.y, 2));
    if (dist > 0.5) {
        // Insert current position as starting point
        PathWaypoint start;
        start.pose = robot_pose;
        start.name = "Start";
        current_path_.waypoints.insert(current_path_.waypoints.begin(), start);
    }

    // Generate trajectory
    path_planner_.generate_trajectory(current_path_);

    path_time_ = 0;
    path_executing_ = true;
    mode_ = SimMode::PATH_EXECUTE;

    std::cout << "[Path] Executing path with " << current_path_.waypoints.size()
              << " waypoints, total time: " << current_path_.total_time << "s" << std::endl;
}

void Simulator::stop_path_execution() {
    path_executing_ = false;
    mode_ = SimMode::DRIVE;
    dynamics_.set_external_command_mode(false);
    std::cout << "[Path] Execution stopped" << std::endl;
}

void Simulator::generate_path_code() {
    if (current_path_.waypoints.empty()) {
        std::cout << "[Path] No waypoints to generate code for" << std::endl;
        return;
    }

    std::cout << "\n========== GENERATED JAVA CODE ==========\n" << std::endl;
    std::cout << path_planner_.generate_code(current_path_, CodeFormat::WPILIB_JAVA) << std::endl;
    std::cout << "=========================================\n" << std::endl;
}

void Simulator::handle_path_execution(double dt) {
    if (!path_executing_ || current_path_.trajectory.empty()) {
        return;
    }

    path_time_ += dt;

    if (path_time_ >= current_path_.total_time) {
        // Path complete
        stop_path_execution();
        std::cout << "[Path] Execution complete!" << std::endl;
        return;
    }

    // Sample trajectory
    Pose2D target = path_planner_.sample_path(current_path_, path_time_);
    double target_vel = path_planner_.sample_velocity(current_path_, path_time_);

    // Simple path following: drive towards target
    Pose2D robot = dynamics_.state().true_pose;
    double dx = target.x - robot.x;
    double dy = target.y - robot.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    // Calculate velocities in robot frame
    double cos_th = std::cos(robot.theta);
    double sin_th = std::sin(robot.theta);
    double vx_field = (dist > 0.01) ? (dx / dist) * target_vel : 0;
    double vy_field = (dist > 0.01) ? (dy / dist) * target_vel : 0;

    double vx_robot = vx_field * cos_th + vy_field * sin_th;
    double vy_robot = -vx_field * sin_th + vy_field * cos_th;

    // Heading control
    double heading_error = target.theta - robot.theta;
    while (heading_error > M_PI) heading_error -= 2 * M_PI;
    while (heading_error < -M_PI) heading_error += 2 * M_PI;
    double omega = heading_error * 3.0;  // P controller

    dynamics_.set_external_command_mode(true);
    dynamics_.apply_velocity_command(vx_robot, vy_robot, omega);
}

int Simulator::run() {
    std::cout << "[Sim] Running " << game_year_to_string(current_game_year_)
              << "... Press ESC to quit.\n" << std::endl;

    while (!shutdown_requested_.load()) {
        auto now = Clock::now();
        double dt = std::chrono::duration<double>(now - last_update_time_).count();
        last_update_time_ = now;
        dt = std::min(dt, 0.1);

        process_input();
        if (shutdown_requested_.load()) break;

        // Handle path execution
        if (mode_ == SimMode::PATH_EXECUTE) {
            handle_path_execution(dt);
        } else {
            update_dynamics(dt);
            handle_auto_align(dt);
        }

        capture_and_render();
        run_detection();
        update_fusion();
        update_visualization();

        // FPS calculation
        frame_count_++;
        double elapsed = std::chrono::duration<double>(now - fps_start_time_).count();
        if (elapsed >= 1.0) {
            stats_.fps = frame_count_ / elapsed;
            frame_count_ = 0;
            fps_start_time_ = now;
        }
    }

    std::cout << "[Sim] Shutting down..." << std::endl;
    return 0;
}

void Simulator::process_input() {
    int key = cv::waitKey(1);
    if (key >= 0) {
        handle_key(key);
    }
}

void Simulator::handle_key(int key) {
    if (key >= 'A' && key <= 'Z') {
        key = key - 'A' + 'a';
    }

    switch (key) {
        // Movement
        case 'w': input_.forward = true; key_hold_time_ = Clock::now(); break;
        case 's': input_.backward = true; key_hold_time_ = Clock::now(); break;
        case 'a': input_.left = true; key_hold_time_ = Clock::now(); break;
        case 'd': input_.right = true; key_hold_time_ = Clock::now(); break;
        case 'q': input_.rotate_ccw = true; key_hold_time_ = Clock::now(); break;
        case 'e': input_.rotate_cw = true; key_hold_time_ = Clock::now(); break;

        case 0x10: case 225: case 229:
            input_.turbo = true;
            key_hold_time_ = Clock::now();
            break;

        // Auto-align
        case 'v':
            if (mode_ == SimMode::PATH_EXECUTE) stop_path_execution();
            input_.auto_align = !input_.auto_align;
            auto_align_.set_enabled(input_.auto_align);
            mode_ = input_.auto_align ? SimMode::AUTO_ALIGN : SimMode::DRIVE;
            std::cout << "[Sim] Auto-align: " << (input_.auto_align ? "ENABLED" : "disabled") << std::endl;
            break;

        // Reset
        case 'r':
            dynamics_.reset(sim_config_.start_pose);
            auto_align_.reset();
            input_.auto_align = false;
            auto_align_.set_enabled(false);
            stop_path_execution();
            mode_ = SimMode::DRIVE;
            std::cout << "[Sim] Robot pose reset" << std::endl;
            break;

        // Path planning
        case 'p':
            if (mode_ == SimMode::PATH_EXECUTE) stop_path_execution();
            mode_ = (mode_ == SimMode::PATH_EDIT) ? SimMode::DRIVE : SimMode::PATH_EDIT;
            input_.auto_align = false;
            auto_align_.set_enabled(false);
            std::cout << "[Path] Edit mode: " << (mode_ == SimMode::PATH_EDIT ? "ON - click field to add waypoints" : "OFF") << std::endl;
            break;

        case 'x':
            execute_path();
            break;

        case 'c':
            clear_path();
            break;

        case 'z':
            remove_last_waypoint();
            break;

        case 'g':
            generate_path_code();
            break;

        // Field selection
        case 'f':
            cycle_field();
            break;

        // Display toggles
        case '1':
            input_.show_true_pose = !input_.show_true_pose;
            topdown_renderer_.set_show_true_pose(input_.show_true_pose);
            break;
        case '2':
            input_.show_odom_pose = !input_.show_odom_pose;
            topdown_renderer_.set_show_odom_pose(input_.show_odom_pose);
            break;
        case '3':
            input_.show_fused_pose = !input_.show_fused_pose;
            topdown_renderer_.set_show_fused_pose(input_.show_fused_pose);
            break;
        case '4':
            input_.show_webcam = !input_.show_webcam;
            break;
        case '5':
            input_.detect_on_composite = !input_.detect_on_composite;
            break;

        case 27: // ESC
            shutdown_requested_.store(true);
            break;
    }
}

void Simulator::update_dynamics(double dt) {
    if (!auto_align_.is_enabled() && mode_ != SimMode::PATH_EXECUTE) {
        dynamics_.set_external_command_mode(false);
    }

    auto now = Clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(now - key_hold_time_).count();
    if (elapsed_ms > 150.0) {
        input_.forward = input_.backward = false;
        input_.left = input_.right = false;
        input_.rotate_ccw = input_.rotate_cw = false;
        input_.turbo = false;
    }

    dynamics_.update(input_, dt);
}

void Simulator::handle_auto_align(double dt) {
    if (!auto_align_.is_enabled()) return;

    auto visible = field_renderer_.get_visible_tags(dynamics_.state().fused_pose);
    double vx, vy, omega;
    bool active = auto_align_.update(dynamics_.state().fused_pose, visible, dt, vx, vy, omega);

    if (active) {
        dynamics_.set_external_command_mode(true);
        dynamics_.apply_velocity_command(vx, vy, omega);
    } else if (auto_align_.is_at_target()) {
        dynamics_.set_external_command_mode(true);
        dynamics_.apply_velocity_command(0, 0, 0);
    }
}

void Simulator::capture_and_render() {
    const auto& true_pose = dynamics_.state().true_pose;
    cv::Mat synthetic = field_renderer_.render(true_pose);

    cv::Mat webcam;
    if (sim_config_.use_webcam && input_.show_webcam) {
        std::lock_guard<std::mutex> lock(webcam_mutex_);
        if (!webcam_frame_.empty()) {
            webcam = webcam_frame_.clone();
        }
    }

    if (!webcam.empty()) {
        if (webcam.size() != synthetic.size()) {
            cv::resize(webcam, webcam, synthetic.size());
        }
        current_frame_ = field_renderer_.composite(webcam, true_pose, 0);
    } else {
        current_frame_ = synthetic;
    }

    display_frame_ = current_frame_.clone();
}

void Simulator::run_detection() {
    cv::Mat detect_frame = input_.detect_on_composite ? current_frame_ :
        field_renderer_.render(dynamics_.state().true_pose);

    Frame frame;
    frame.camera_id = 0;
    frame.frame_number = frame_count_;
    frame.capture_time = SteadyClock::now();
    frame.capture_wall_time = SystemClock::now();
    frame.image = detect_frame;

    auto processed = pipeline_->process_frame(frame, intrinsics_, camera_to_robot_);
    last_detections_ = processed.detections;

    stats_.tag_count = static_cast<int>(last_detections_.detections.size());
    stats_.detect_ms = last_detections_.timestamps.detect_ms();
    stats_.pose_ms = last_detections_.timestamps.pose_ms();
    stats_.reproj_error = last_detections_.avg_reproj_error();
    stats_.confidence = last_detections_.multi_tag_pose_valid ? 1.0 :
        (last_detections_.detections.empty() ? 0.0 : 0.5);

    // Draw detections
    for (const auto& det : last_detections_.detections) {
        for (int i = 0; i < 4; i++) {
            cv::Point2d p1(det.corners.corners[i].x, det.corners.corners[i].y);
            cv::Point2d p2(det.corners.corners[(i+1)%4].x, det.corners.corners[(i+1)%4].y);
            cv::line(display_frame_, p1, p2, cv::Scalar(0, 255, 0), 2);
        }
        auto center = det.corners.center();
        cv::putText(display_frame_, std::to_string(det.id),
                   cv::Point(static_cast<int>(center.x) - 10, static_cast<int>(center.y)),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
    }
}

void Simulator::update_fusion() {
    auto fused = pipeline_->get_fused_pose();
    if (fused.valid) {
        dynamics_.state().fused_pose = fused.pose_filtered;
    } else {
        dynamics_.state().fused_pose = dynamics_.state().true_pose;
    }
}

void Simulator::update_visualization() {
    // Camera view stats
    cv::putText(display_frame_, "FPS: " + std::to_string(static_cast<int>(stats_.fps)),
               cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    cv::putText(display_frame_, "Tags: " + std::to_string(stats_.tag_count),
               cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    cv::putText(display_frame_, game_year_to_string(current_game_year_),
               cv::Point(10, 75), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);

    // Mode indicator
    std::string mode_str;
    cv::Scalar mode_color(255, 255, 255);
    switch (mode_) {
        case SimMode::PATH_EDIT:
            mode_str = "MODE: PATH EDIT (click field)";
            mode_color = cv::Scalar(255, 165, 0);
            break;
        case SimMode::PATH_EXECUTE:
            mode_str = "MODE: EXECUTING PATH";
            mode_color = cv::Scalar(0, 255, 0);
            break;
        case SimMode::AUTO_ALIGN:
            mode_str = "MODE: AUTO-ALIGN Tag " + std::to_string(auto_align_.target_tag_id());
            mode_color = cv::Scalar(0, 255, 0);
            break;
        default:
            mode_str = "MODE: DRIVE";
            break;
    }
    cv::putText(display_frame_, mode_str, cv::Point(10, 100),
               cv::FONT_HERSHEY_SIMPLEX, 0.6, mode_color, 2);

    cv::imshow(CAMERA_WINDOW, display_frame_);

    // Field view
    cv::Mat field_view = topdown_renderer_.render(dynamics_.state());

    // Draw waypoints and path
    if (!current_path_.waypoints.empty()) {
        // Draw waypoints
        for (size_t i = 0; i < current_path_.waypoints.size(); i++) {
            const auto& wp = current_path_.waypoints[i];
            auto pt = field_to_screen(wp.pose.x, wp.pose.y);

            // Draw waypoint circle
            cv::Scalar color = (i == 0) ? cv::Scalar(0, 255, 0) :
                              (i == current_path_.waypoints.size() - 1) ? cv::Scalar(0, 0, 255) :
                              cv::Scalar(255, 165, 0);
            cv::circle(field_view, cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)),
                      8, color, -1);
            cv::putText(field_view, wp.name,
                       cv::Point(static_cast<int>(pt.x) + 10, static_cast<int>(pt.y)),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

            // Draw line to previous waypoint
            if (i > 0) {
                const auto& prev = current_path_.waypoints[i - 1];
                auto prev_pt = field_to_screen(prev.pose.x, prev.pose.y);
                cv::line(field_view,
                        cv::Point(static_cast<int>(prev_pt.x), static_cast<int>(prev_pt.y)),
                        cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)),
                        cv::Scalar(255, 165, 0), 2);
            }
        }

        // Draw trajectory if generated
        if (!current_path_.trajectory.empty()) {
            for (size_t i = 1; i < current_path_.trajectory.size(); i++) {
                auto p1 = field_to_screen(current_path_.trajectory[i-1].pose.x,
                                         current_path_.trajectory[i-1].pose.y);
                auto p2 = field_to_screen(current_path_.trajectory[i].pose.x,
                                         current_path_.trajectory[i].pose.y);
                cv::line(field_view,
                        cv::Point(static_cast<int>(p1.x), static_cast<int>(p1.y)),
                        cv::Point(static_cast<int>(p2.x), static_cast<int>(p2.y)),
                        cv::Scalar(0, 255, 255), 1);
            }
        }
    }

    // Draw path info
    if (mode_ == SimMode::PATH_EDIT) {
        cv::putText(field_view, "CLICK TO ADD WAYPOINTS",
                   cv::Point(10, field_view.rows - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 165, 0), 2);
    }
    if (!current_path_.waypoints.empty()) {
        cv::putText(field_view, "Waypoints: " + std::to_string(current_path_.waypoints.size()),
                   cv::Point(10, field_view.rows - 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }

    topdown_renderer_.draw_stats(field_view, stats_.fps, stats_.detect_ms + stats_.pose_ms,
                                 stats_.tag_count, stats_.reproj_error, stats_.confidence,
                                 auto_align_.target_tag_id(), auto_align_.is_enabled());

    cv::imshow(FIELD_WINDOW, field_view);
}

} // namespace sim
} // namespace frc_vision
