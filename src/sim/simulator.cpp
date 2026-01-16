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

    // Register default subsystems (shooter, intake, elevator)
    path_planner_.register_default_subsystems();

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
    std::cout << "FRC VISION AUTO CODER - CONTROLS" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  WASD     - Move robot" << std::endl;
    std::cout << "  Q/E      - Rotate CCW/CW" << std::endl;
    std::cout << "  SHIFT    - Turbo mode" << std::endl;
    std::cout << "  V        - Toggle auto-align" << std::endl;
    std::cout << "  R        - Reset robot pose" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "PATH PLANNER:" << std::endl;
    std::cout << "  P        - Toggle path edit mode" << std::endl;
    std::cout << "  Click    - Add waypoint (in edit mode)" << std::endl;
    std::cout << "  X        - Execute path" << std::endl;
    std::cout << "  C        - Clear path" << std::endl;
    std::cout << "  Z        - Remove last waypoint" << std::endl;
    std::cout << "  G        - Generate WPILib 2026 code" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "WAYPOINT EDITING:" << std::endl;
    std::cout << "  Click WP - Select waypoint" << std::endl;
    std::cout << "  Drag WP  - Adjust heading (click twice)" << std::endl;
    std::cout << "  [ / ]    - Rotate selected heading CCW/CW" << std::endl;
    std::cout << "  DEL/BS   - Delete selected waypoint" << std::endl;
    std::cout << "  Right-click WP - Add actions to waypoint" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "ACTION BLOCKS (Visual Block Coding):" << std::endl;
    std::cout << "  B        - Add action block at waypoint" << std::endl;
    std::cout << "  T/L      - List all action blocks" << std::endl;
    std::cout << "  Click Block - Toggle enabled/disabled" << std::endl;
    std::cout << "  0-9      - Toggle block by index" << std::endl;
    std::cout << "  Right-click tag - Add tag-triggered action" << std::endl;
    std::cout << "  Visual blocks combine conditions:" << std::endl;
    std::cout << "    [At WP 2] AND [Tag 5 Visible] -> [Shoot]" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "SUBSYSTEMS:" << std::endl;
    std::cout << "  Registered: Shooter, Intake, Elevator" << std::endl;
    std::cout << "  Actions: Fire, SpinUp, IntakeIn/Out, SetHeight" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "FIELD:" << std::endl;
    std::cout << "  F        - Cycle field (2024->2025->2026)" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "DISPLAY:" << std::endl;
    std::cout << "  1-5      - Toggle overlays" << std::endl;
    std::cout << "  ESC      - Deselect / Quit" << std::endl;
    std::cout << "========================================\n" << std::endl;
}

// Mouse callback for waypoint placement
void Simulator::mouse_callback(int event, int x, int y, int flags, void* userdata) {
    Simulator* sim = static_cast<Simulator*>(userdata);
    sim->handle_mouse(event, x, y, flags);
}

void Simulator::handle_mouse(int event, int x, int y, int flags) {
    (void)flags;

    // Handle action menu interactions first
    if (action_menu_.visible) {
        if (event == cv::EVENT_MOUSEMOVE) {
            handle_menu_hover(x, y);
            return;
        } else if (event == cv::EVENT_LBUTTONDOWN) {
            // Check if click is inside menu
            int menu_height = static_cast<int>(action_menu_.items.size()) * ActionMenuState::ITEM_HEIGHT + 2 * ActionMenuState::PADDING;
            if (x >= action_menu_.x && x <= action_menu_.x + ActionMenuState::MENU_WIDTH &&
                y >= action_menu_.y && y <= action_menu_.y + menu_height) {
                handle_menu_click(x, y);
                return;
            } else {
                // Click outside menu - close it
                hide_action_menu();
            }
        }
    }

    // Handle mouse movement for hover effects and heading drag
    if (event == cv::EVENT_MOUSEMOVE) {
        // Update hover states
        waypoint_edit_.hover_waypoint = get_waypoint_at_screen_pos(x, y);
        waypoint_edit_.hover_action_block = get_action_block_at_screen_pos(x, y);

        // Handle heading drag
        if (waypoint_edit_.dragging_heading && waypoint_edit_.selected_waypoint >= 0) {
            handle_waypoint_drag(x, y);
        }
        return;
    }

    // Handle mouse button release - stop heading drag
    if (event == cv::EVENT_LBUTTONUP) {
        waypoint_edit_.dragging_heading = false;
        return;
    }

    // Handle right-click for tag/waypoint action menus
    if (event == cv::EVENT_RBUTTONDOWN) {
        // Check waypoint first
        int wp_idx = get_waypoint_at_screen_pos(x, y);
        if (wp_idx >= 0) {
            show_waypoint_action_menu(wp_idx, x, y);
            return;
        }

        // Then check tag
        int tag_id = get_tag_at_screen_pos(x, y);
        if (tag_id >= 0) {
            show_tag_action_menu(tag_id, x, y);
            return;
        }
    }

    // Handle left-click
    if (event == cv::EVENT_LBUTTONDOWN) {
        // Check if clicking on an action block (to toggle it)
        int block_idx = get_action_block_at_screen_pos(x, y);
        if (block_idx >= 0) {
            toggle_action_block(block_idx);
            return;
        }

        // Check if clicking on a waypoint (to select it or start heading drag)
        int wp_idx = get_waypoint_at_screen_pos(x, y);
        if (wp_idx >= 0) {
            if (waypoint_edit_.selected_waypoint == wp_idx) {
                // Already selected - start heading drag
                waypoint_edit_.dragging_heading = true;
                std::cout << "[Path] Drag to change heading for waypoint " << wp_idx << std::endl;
            } else {
                // Select this waypoint
                waypoint_edit_.selected_waypoint = wp_idx;
                std::cout << "[Path] Selected waypoint " << wp_idx << std::endl;
            }
            return;
        }

        // In path edit mode, click to add waypoint
        if (mode_ == SimMode::PATH_EDIT) {
            // Check if clicking on a tag first
            int tag_id = get_tag_at_screen_pos(x, y);
            if (tag_id >= 0) {
                // Show menu for tag instead of adding waypoint
                show_tag_action_menu(tag_id, x, y);
                return;
            }

            // Deselect waypoint when clicking elsewhere
            waypoint_edit_.selected_waypoint = -1;

            // Convert screen coords to field coords
            auto [fx, fy] = screen_to_field(x, y);

            // Validate bounds
            if (fx >= 0 && fx <= field_length_ && fy >= 0 && fy <= field_width_) {
                add_waypoint_at_click(fx, fy);
            }
        } else {
            // Not in path edit mode - just deselect
            waypoint_edit_.selected_waypoint = -1;
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

    // Reset execution state
    path_time_ = 0;
    current_waypoint_index_ = 0;
    path_executing_ = true;
    mode_ = SimMode::PATH_EXECUTE;

    // Reset action blocks for fresh execution
    PathPlanner::reset_action_blocks(current_path_);

    std::cout << "[Path] Executing path with " << current_path_.waypoints.size()
              << " waypoints, total time: " << current_path_.total_time << "s" << std::endl;
    std::cout << "[Path] Action blocks: " << current_path_.action_blocks.size() << std::endl;
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

    // Generate all code with scaffolds
    std::cout << "\n========== GENERATING WPILib 2026 CODE ==========\n" << std::endl;

    auto result = path_planner_.generate_all_code(current_path_, CodeFormat::WPILIB_JAVA, true);

    // Print each generated file
    for (const auto& file : result.files) {
        std::cout << "\n// ========== " << file.path << " ==========\n";
        std::cout << "// " << file.description << "\n";
        std::cout << file.content << std::endl;
    }

    // Print integration notes
    std::cout << "\n========== INTEGRATION NOTES ==========\n" << std::endl;
    for (const auto& note : result.integration_notes) {
        std::cout << note << std::endl;
    }

    std::cout << "\n===================================================\n" << std::endl;
}

// ============================================================================
// Tag Click and Action Menu System
// ============================================================================

int Simulator::get_tag_at_screen_pos(int x, int y) const {
    // Convert screen position to field coordinates
    auto [fx, fy] = screen_to_field(x, y);

    // Check each tag in the field layout
    for (const auto& [tag_id, tag] : field_.tags) {
        double tag_x = tag.center_field.x;
        double tag_y = tag.center_field.y;

        // Check if click is within 0.3 meters of tag center
        double dist = std::sqrt(std::pow(fx - tag_x, 2) + std::pow(fy - tag_y, 2));
        if (dist < 0.3) {
            return tag_id;
        }
    }

    return -1;  // No tag at this position
}

void Simulator::show_tag_action_menu(int tag_id, int screen_x, int screen_y) {
    action_menu_.visible = true;
    action_menu_.x = screen_x;
    action_menu_.y = screen_y;
    action_menu_.selected_tag_id = tag_id;
    action_menu_.hover_index = -1;
    action_menu_.selected_index = -1;

    // Get available actions for this tag
    auto actions = path_planner_.get_actions_for_tag(tag_id);

    // Build menu items
    action_menu_.items.clear();

    // Header item (not clickable)
    MenuItem header;
    header.label = "Tag " + std::to_string(tag_id) + " Actions:";
    header.enabled = false;
    action_menu_.items.push_back(header);

    // Add action items
    for (const auto& action : actions) {
        MenuItem item;
        item.label = action_to_string(action);
        item.action = action;
        item.enabled = true;
        item.callback = [this, tag_id, action]() {
            execute_tag_action(tag_id, action);
        };
        action_menu_.items.push_back(item);
    }

    // Add separator and special actions
    MenuItem sep;
    sep.label = "---";
    sep.enabled = false;
    action_menu_.items.push_back(sep);

    MenuItem add_marker;
    add_marker.label = "Add as Event Marker";
    add_marker.action = RobotAction::CUSTOM_COMMAND;
    add_marker.enabled = true;
    add_marker.callback = [this, tag_id]() {
        // Show sub-menu or default action
        add_event_marker_for_action(tag_id, RobotAction::AUTO_ALIGN_TO_TAG);
    };
    action_menu_.items.push_back(add_marker);

    MenuItem waypoint;
    waypoint.label = "Add Waypoint Here";
    waypoint.action = RobotAction::CUSTOM_COMMAND;
    waypoint.enabled = true;
    waypoint.callback = [this, tag_id]() {
        if (field_.has_tag(tag_id)) {
            const auto& tag = field_.get_tag(tag_id);
            add_waypoint_at_click(tag.center_field.x, tag.center_field.y);
            hide_action_menu();
        }
    };
    action_menu_.items.push_back(waypoint);

    mode_ = SimMode::ACTION_MENU;

    std::cout << "[Menu] Showing actions for Tag " << tag_id << std::endl;
}

void Simulator::hide_action_menu() {
    action_menu_.visible = false;
    action_menu_.selected_tag_id = -1;
    action_menu_.items.clear();
    if (mode_ == SimMode::ACTION_MENU) {
        mode_ = SimMode::DRIVE;
    }
}

void Simulator::handle_menu_click(int x, int y) {
    int relative_y = y - action_menu_.y - ActionMenuState::PADDING;
    int index = relative_y / ActionMenuState::ITEM_HEIGHT;

    if (index >= 0 && index < static_cast<int>(action_menu_.items.size())) {
        const auto& item = action_menu_.items[index];
        if (item.enabled && item.callback) {
            item.callback();
            hide_action_menu();
        }
    }
    (void)x;
}

void Simulator::handle_menu_hover(int x, int y) {
    int relative_y = y - action_menu_.y - ActionMenuState::PADDING;
    int index = relative_y / ActionMenuState::ITEM_HEIGHT;

    if (x >= action_menu_.x && x <= action_menu_.x + ActionMenuState::MENU_WIDTH &&
        index >= 0 && index < static_cast<int>(action_menu_.items.size())) {
        action_menu_.hover_index = index;
    } else {
        action_menu_.hover_index = -1;
    }
}

void Simulator::execute_tag_action(int tag_id, RobotAction action) {
    std::cout << "[Action] Executing " << action_to_string(action) << " for Tag " << tag_id << std::endl;

    switch (action) {
        case RobotAction::AUTO_ALIGN_TO_TAG:
        case RobotAction::FACE_TAG:
            // Start auto-align to this tag
            auto_align_.set_target_tag(tag_id);
            auto_align_.set_enabled(true);
            input_.auto_align = true;
            mode_ = SimMode::AUTO_ALIGN;
            break;

        case RobotAction::SHOOT:
        case RobotAction::INTAKE_IN:
        case RobotAction::SET_ELEVATOR_HEIGHT:
            // Add as event marker to current path
            add_event_marker_for_action(tag_id, action);
            break;

        case RobotAction::STOW:
            // Just print - would need actual subsystem
            std::cout << "[Action] Stow command - add to RobotContainer" << std::endl;
            break;

        default:
            std::cout << "[Action] Action not implemented in simulator" << std::endl;
            break;
    }
}

void Simulator::add_event_marker_for_action(int tag_id, RobotAction action) {
    // Determine the waypoint index to bind this action to
    int waypoint_idx = current_path_.waypoints.empty() ? 0 :
                       static_cast<int>(current_path_.waypoints.size()) - 1;

    // Create an ActionBlock with combined conditions:
    // - At or after waypoint X
    // - AND tag is visible AND within 2m
    ActionBlock block;
    block.name = action_to_command_name(action) + "_Tag" + std::to_string(tag_id);
    block.description = "At WP" + std::to_string(waypoint_idx) +
                       " + Tag " + std::to_string(tag_id) + " visible";
    block.logic = ConditionLogic::AND;

    // Condition 1: At waypoint (if we have waypoints)
    if (!current_path_.waypoints.empty()) {
        Condition wp_cond;
        wp_cond.type = TriggerCondition::AT_WAYPOINT;
        wp_cond.waypoint_index = waypoint_idx;
        wp_cond.trigger_on_edge = false;  // Don't need edge detection for waypoint
        block.conditions.push_back(wp_cond);
    }

    // Condition 2: Tag visible
    Condition vis_cond;
    vis_cond.type = TriggerCondition::TAG_VISIBLE;
    vis_cond.tag_id = tag_id;
    vis_cond.trigger_on_edge = true;  // Trigger once when tag becomes visible
    block.conditions.push_back(vis_cond);

    // Condition 3: Within 2m of tag
    Condition dist_cond;
    dist_cond.type = TriggerCondition::DISTANCE_TO_TAG;
    dist_cond.tag_id = tag_id;
    dist_cond.distance = 2.0;  // meters
    dist_cond.compare = Condition::Compare::LESS_THAN;
    dist_cond.trigger_on_edge = false;
    block.conditions.push_back(dist_cond);

    // Action to execute
    ActionBlock::ActionStep step;
    step.action = action;
    step.parameters["tag_id"] = static_cast<double>(tag_id);
    block.actions.push_back(step);

    // Add to path
    current_path_.action_blocks.push_back(block);

    std::cout << "\n[ActionBlock] Created combined trigger block:" << std::endl;
    std::cout << "  Name: " << block.name << std::endl;
    std::cout << "  Conditions:" << std::endl;
    for (const auto& cond : block.conditions) {
        std::cout << "    - " << cond.describe() << std::endl;
    }
    std::cout << "  Action: " << action_to_string(action) << std::endl;
    std::cout << "  Total blocks: " << current_path_.action_blocks.size() << std::endl;
}

void Simulator::draw_action_menu(cv::Mat& frame) {
    if (!action_menu_.visible || action_menu_.items.empty()) return;

    int menu_height = static_cast<int>(action_menu_.items.size()) * ActionMenuState::ITEM_HEIGHT + 2 * ActionMenuState::PADDING;

    // Draw menu background with semi-transparent overlay
    cv::Mat overlay = frame.clone();
    cv::rectangle(overlay,
                  cv::Point(action_menu_.x, action_menu_.y),
                  cv::Point(action_menu_.x + ActionMenuState::MENU_WIDTH, action_menu_.y + menu_height),
                  cv::Scalar(40, 40, 40), cv::FILLED);
    cv::addWeighted(overlay, 0.9, frame, 0.1, 0, frame);

    // Draw border
    cv::rectangle(frame,
                  cv::Point(action_menu_.x, action_menu_.y),
                  cv::Point(action_menu_.x + ActionMenuState::MENU_WIDTH, action_menu_.y + menu_height),
                  cv::Scalar(100, 100, 100), 1);

    // Draw items
    for (size_t i = 0; i < action_menu_.items.size(); i++) {
        const auto& item = action_menu_.items[i];
        int item_y = action_menu_.y + ActionMenuState::PADDING + static_cast<int>(i) * ActionMenuState::ITEM_HEIGHT;

        // Highlight hovered item
        if (static_cast<int>(i) == action_menu_.hover_index && item.enabled) {
            cv::rectangle(frame,
                          cv::Point(action_menu_.x + 2, item_y),
                          cv::Point(action_menu_.x + ActionMenuState::MENU_WIDTH - 2, item_y + ActionMenuState::ITEM_HEIGHT - 2),
                          cv::Scalar(80, 80, 80), cv::FILLED);
        }

        // Draw separator
        if (item.label == "---") {
            cv::line(frame,
                     cv::Point(action_menu_.x + 10, item_y + ActionMenuState::ITEM_HEIGHT / 2),
                     cv::Point(action_menu_.x + ActionMenuState::MENU_WIDTH - 10, item_y + ActionMenuState::ITEM_HEIGHT / 2),
                     cv::Scalar(100, 100, 100), 1);
            continue;
        }

        // Draw text
        cv::Scalar text_color = item.enabled ? cv::Scalar(255, 255, 255) : cv::Scalar(150, 150, 150);
        cv::putText(frame, item.label,
                    cv::Point(action_menu_.x + 10, item_y + 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1);
    }
}

// ============================================================================
// Waypoint Interaction System
// ============================================================================

int Simulator::get_waypoint_at_screen_pos(int x, int y) const {
    if (current_path_.waypoints.empty()) return -1;

    auto [fx, fy] = screen_to_field(x, y);

    for (size_t i = 0; i < current_path_.waypoints.size(); i++) {
        const auto& wp = current_path_.waypoints[i];
        double dist = std::sqrt(std::pow(fx - wp.pose.x, 2) + std::pow(fy - wp.pose.y, 2));
        if (dist < 0.3) {  // 0.3m click radius
            return static_cast<int>(i);
        }
    }
    return -1;
}

void Simulator::show_waypoint_action_menu(int waypoint_idx, int screen_x, int screen_y) {
    if (waypoint_idx < 0 || waypoint_idx >= static_cast<int>(current_path_.waypoints.size())) return;

    action_menu_.visible = true;
    action_menu_.x = screen_x;
    action_menu_.y = screen_y;
    action_menu_.selected_waypoint_id = waypoint_idx;
    action_menu_.selected_tag_id = -1;
    action_menu_.menu_type = ActionMenuState::MenuType::WAYPOINT;
    action_menu_.hover_index = -1;

    const auto& wp = current_path_.waypoints[waypoint_idx];
    action_menu_.items.clear();

    // Header
    MenuItem header;
    header.label = "Waypoint " + std::to_string(waypoint_idx) + " (" + wp.name + ")";
    header.enabled = false;
    action_menu_.items.push_back(header);

    // Heading info
    MenuItem heading_info;
    heading_info.label = "Heading: " + std::to_string(static_cast<int>(wp.pose.theta * 180 / M_PI)) + " deg";
    heading_info.enabled = false;
    action_menu_.items.push_back(heading_info);

    // Separator
    MenuItem sep;
    sep.label = "---";
    sep.enabled = false;
    action_menu_.items.push_back(sep);

    // Action items
    std::vector<std::pair<std::string, RobotAction>> actions = {
        {"Add: Spin Up Shooter", RobotAction::SPIN_UP_SHOOTER},
        {"Add: Shoot", RobotAction::SHOOT},
        {"Add: Intake In", RobotAction::INTAKE_IN},
        {"Add: Intake Out", RobotAction::INTAKE_OUT},
        {"Add: Set Elevator High", RobotAction::SET_ELEVATOR_HEIGHT},
        {"Add: Stow", RobotAction::STOW},
        {"Add: Wait 0.5s", RobotAction::WAIT}
    };

    for (const auto& [label, action] : actions) {
        MenuItem item;
        item.label = label;
        item.action = action;
        item.enabled = true;
        item.callback = [this, waypoint_idx, action]() {
            add_action_to_waypoint(waypoint_idx, action);
        };
        action_menu_.items.push_back(item);
    }

    // Separator
    action_menu_.items.push_back(sep);

    // Heading controls - get current heading at callback time
    MenuItem rotate_left;
    rotate_left.label = "Rotate Heading -45 deg";
    rotate_left.enabled = true;
    rotate_left.callback = [this, waypoint_idx]() {
        if (waypoint_idx >= 0 && waypoint_idx < static_cast<int>(current_path_.waypoints.size())) {
            double current = current_path_.waypoints[waypoint_idx].pose.theta;
            update_waypoint_heading(waypoint_idx, current - M_PI / 4);
        }
    };
    action_menu_.items.push_back(rotate_left);

    MenuItem rotate_right;
    rotate_right.label = "Rotate Heading +45 deg";
    rotate_right.enabled = true;
    rotate_right.callback = [this, waypoint_idx]() {
        if (waypoint_idx >= 0 && waypoint_idx < static_cast<int>(current_path_.waypoints.size())) {
            double current = current_path_.waypoints[waypoint_idx].pose.theta;
            update_waypoint_heading(waypoint_idx, current + M_PI / 4);
        }
    };
    action_menu_.items.push_back(rotate_right);

    // Delete waypoint
    MenuItem del;
    del.label = "Delete Waypoint";
    del.enabled = true;
    del.callback = [this, waypoint_idx]() {
        if (waypoint_idx >= 0 && waypoint_idx < static_cast<int>(current_path_.waypoints.size())) {
            current_path_.waypoints.erase(current_path_.waypoints.begin() + waypoint_idx);
            std::cout << "[Path] Deleted waypoint " << waypoint_idx << std::endl;
        }
    };
    action_menu_.items.push_back(del);

    mode_ = SimMode::ACTION_MENU;
    std::cout << "[Menu] Showing actions for Waypoint " << waypoint_idx << std::endl;
}

void Simulator::execute_waypoint_action(int waypoint_idx, RobotAction action) {
    std::cout << "[Action] Adding " << action_to_string(action) << " at Waypoint " << waypoint_idx << std::endl;
    add_action_to_waypoint(waypoint_idx, action);
}

void Simulator::add_action_to_waypoint(int waypoint_idx, RobotAction action) {
    // Create an action block that triggers at this waypoint
    ActionBlock block;
    block.name = action_to_command_name(action) + "_WP" + std::to_string(waypoint_idx);
    block.description = action_to_string(action) + " at waypoint " + std::to_string(waypoint_idx);
    block.logic = ConditionLogic::AND;

    // Add waypoint condition
    Condition wp_cond;
    wp_cond.type = TriggerCondition::AT_WAYPOINT;
    wp_cond.waypoint_index = waypoint_idx;
    wp_cond.trigger_on_edge = true;
    block.conditions.push_back(wp_cond);

    // Add action
    ActionBlock::ActionStep step;
    step.action = action;

    // Set default parameters based on action type
    if (action == RobotAction::SET_ELEVATOR_HEIGHT) {
        step.parameters["height"] = 1.0;  // 1 meter high
    } else if (action == RobotAction::WAIT) {
        step.parameters["time"] = 0.5;  // 0.5 seconds
    }

    block.actions.push_back(step);

    current_path_.action_blocks.push_back(block);

    std::cout << "[ActionBlock] Created: " << block.name << std::endl;
    std::cout << "  Conditions: " << block.conditions[0].describe() << std::endl;
    std::cout << "  Action: " << action_to_string(action) << std::endl;
    std::cout << "  Total blocks: " << current_path_.action_blocks.size() << std::endl;
}

void Simulator::update_waypoint_heading(int waypoint_idx, double heading) {
    if (waypoint_idx < 0 || waypoint_idx >= static_cast<int>(current_path_.waypoints.size())) return;

    // Normalize heading to [-PI, PI]
    while (heading > M_PI) heading -= 2 * M_PI;
    while (heading < -M_PI) heading += 2 * M_PI;

    current_path_.waypoints[waypoint_idx].pose.theta = heading;
    std::cout << "[Path] Waypoint " << waypoint_idx << " heading set to "
              << (heading * 180 / M_PI) << " deg" << std::endl;
}

void Simulator::handle_waypoint_drag(int x, int y) {
    if (waypoint_edit_.selected_waypoint < 0 ||
        waypoint_edit_.selected_waypoint >= static_cast<int>(current_path_.waypoints.size())) {
        return;
    }

    auto [fx, fy] = screen_to_field(x, y);
    auto& wp = current_path_.waypoints[waypoint_edit_.selected_waypoint];

    // Calculate new heading based on drag position relative to waypoint
    double dx = fx - wp.pose.x;
    double dy = fy - wp.pose.y;
    double new_heading = std::atan2(dy, dx);

    wp.pose.theta = new_heading;
}

// ============================================================================
// Action Block Visualization
// ============================================================================

int Simulator::get_action_block_at_screen_pos(int x, int y) const {
    // Action blocks are drawn near their associated waypoints
    // Check each block's visual position
    for (size_t i = 0; i < current_path_.action_blocks.size(); i++) {
        const auto& block = current_path_.action_blocks[i];

        // Find the associated waypoint
        int wp_idx = -1;
        for (const auto& cond : block.conditions) {
            if (cond.type == TriggerCondition::AT_WAYPOINT) {
                wp_idx = cond.waypoint_index;
                break;
            }
        }

        if (wp_idx >= 0 && wp_idx < static_cast<int>(current_path_.waypoints.size())) {
            const auto& wp = current_path_.waypoints[wp_idx];
            auto pt = field_to_screen(wp.pose.x, wp.pose.y);

            // Block is drawn offset from waypoint
            double block_x = pt.x + 20 + (i % 3) * 25;
            double block_y = pt.y - 30 - (i / 3) * 25;

            // Check if click is within block icon (20x20 pixels)
            if (x >= block_x - 10 && x <= block_x + 10 &&
                y >= block_y - 10 && y <= block_y + 10) {
                return static_cast<int>(i);
            }
        }
    }
    return -1;
}

void Simulator::toggle_action_block(int block_idx) {
    if (block_idx < 0 || block_idx >= static_cast<int>(current_path_.action_blocks.size())) return;

    auto& block = current_path_.action_blocks[block_idx];
    block.enabled = !block.enabled;

    std::cout << "[ActionBlock] " << block.name << " is now "
              << (block.enabled ? "ENABLED" : "DISABLED") << std::endl;
}

void Simulator::draw_action_blocks(cv::Mat& frame) {
    if (current_path_.action_blocks.empty()) return;

    // Group blocks by their associated waypoint
    std::map<int, std::vector<size_t>> blocks_by_waypoint;

    for (size_t i = 0; i < current_path_.action_blocks.size(); i++) {
        const auto& block = current_path_.action_blocks[i];
        int wp_idx = -1;
        int tag_id = -1;

        for (const auto& cond : block.conditions) {
            if (cond.type == TriggerCondition::AT_WAYPOINT) {
                wp_idx = cond.waypoint_index;
            } else if (cond.type == TriggerCondition::TAG_VISIBLE ||
                       cond.type == TriggerCondition::DISTANCE_TO_TAG) {
                tag_id = cond.tag_id;
            }
        }

        if (wp_idx >= 0) {
            blocks_by_waypoint[wp_idx].push_back(i);
        } else if (tag_id >= 0 && field_.has_tag(tag_id)) {
            // Draw near tag instead
            const auto& tag = field_.get_tag(tag_id);
            auto pt = field_to_screen(tag.center_field.x, tag.center_field.y);

            // Draw action block indicator
            cv::Scalar color = block.enabled ?
                (block.has_triggered ? cv::Scalar(0, 200, 0) : cv::Scalar(255, 200, 0)) :
                cv::Scalar(100, 100, 100);

            int offset_x = static_cast<int>(10 + (i % 3) * 22);
            int offset_y = static_cast<int>(-25 - (i / 3) * 22);

            cv::Point block_pos(static_cast<int>(pt.x) + offset_x, static_cast<int>(pt.y) + offset_y);
            cv::rectangle(frame, block_pos - cv::Point(10, 10), block_pos + cv::Point(10, 10), color, -1);
            cv::rectangle(frame, block_pos - cv::Point(10, 10), block_pos + cv::Point(10, 10),
                         cv::Scalar(255, 255, 255), 1);

            // Draw action icon/letter
            std::string icon = "A";  // Generic action
            if (!block.actions.empty()) {
                switch (block.actions[0].action) {
                    case RobotAction::SHOOT: icon = "S"; break;
                    case RobotAction::SPIN_UP_SHOOTER: icon = "^"; break;
                    case RobotAction::INTAKE_IN: icon = "I"; break;
                    case RobotAction::INTAKE_OUT: icon = "O"; break;
                    case RobotAction::SET_ELEVATOR_HEIGHT: icon = "E"; break;
                    case RobotAction::STOW: icon = "H"; break;
                    case RobotAction::AUTO_ALIGN_TO_TAG: icon = "@"; break;
                    default: icon = "?"; break;
                }
            }
            cv::putText(frame, icon, block_pos - cv::Point(5, -5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
        }
    }

    // Draw blocks grouped by waypoint
    for (const auto& [wp_idx, block_indices] : blocks_by_waypoint) {
        if (wp_idx < 0 || wp_idx >= static_cast<int>(current_path_.waypoints.size())) continue;

        const auto& wp = current_path_.waypoints[wp_idx];
        auto pt = field_to_screen(wp.pose.x, wp.pose.y);

        for (size_t j = 0; j < block_indices.size(); j++) {
            size_t i = block_indices[j];
            const auto& block = current_path_.action_blocks[i];

            // Color based on state
            cv::Scalar color;
            if (!block.enabled) {
                color = cv::Scalar(80, 80, 80);  // Gray - disabled
            } else if (block.has_triggered) {
                color = cv::Scalar(0, 200, 0);   // Green - triggered
            } else {
                color = cv::Scalar(255, 180, 0); // Cyan/yellow - active
            }

            int offset_x = static_cast<int>(20 + (j % 3) * 22);
            int offset_y = static_cast<int>(-15 - (j / 3) * 22);

            cv::Point block_pos(static_cast<int>(pt.x) + offset_x, static_cast<int>(pt.y) + offset_y);

            // Draw block box
            cv::rectangle(frame, block_pos - cv::Point(10, 10), block_pos + cv::Point(10, 10), color, -1);
            cv::rectangle(frame, block_pos - cv::Point(10, 10), block_pos + cv::Point(10, 10),
                         cv::Scalar(255, 255, 255), 1);

            // Draw hover effect
            if (waypoint_edit_.hover_action_block == static_cast<int>(i)) {
                cv::rectangle(frame, block_pos - cv::Point(12, 12), block_pos + cv::Point(12, 12),
                             cv::Scalar(0, 255, 255), 2);
            }

            // Draw action icon
            std::string icon = "?";
            if (!block.actions.empty()) {
                switch (block.actions[0].action) {
                    case RobotAction::SHOOT: icon = "S"; break;
                    case RobotAction::SPIN_UP_SHOOTER: icon = "^"; break;
                    case RobotAction::INTAKE_IN: icon = "I"; break;
                    case RobotAction::INTAKE_OUT: icon = "O"; break;
                    case RobotAction::SET_ELEVATOR_HEIGHT: icon = "E"; break;
                    case RobotAction::STOW: icon = "H"; break;
                    case RobotAction::AUTO_ALIGN_TO_TAG: icon = "@"; break;
                    case RobotAction::WAIT: icon = "W"; break;
                    default: icon = "A"; break;
                }
            }
            cv::putText(frame, icon, block_pos - cv::Point(5, -5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);

            // Draw connector line to waypoint
            cv::line(frame, block_pos, cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)),
                    cv::Scalar(150, 150, 150), 1);
        }
    }

    // Draw action block legend/info at bottom
    int y_pos = frame.rows - 60;
    cv::putText(frame, "Action Blocks: " + std::to_string(current_path_.action_blocks.size()) +
                " (Click to toggle, T to list)",
               cv::Point(10, y_pos), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);

    // Draw legend
    cv::rectangle(frame, cv::Point(10, y_pos + 10), cv::Point(25, y_pos + 25),
                 cv::Scalar(255, 180, 0), -1);
    cv::putText(frame, "Enabled", cv::Point(30, y_pos + 22),
               cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(200, 200, 200), 1);

    cv::rectangle(frame, cv::Point(100, y_pos + 10), cv::Point(115, y_pos + 25),
                 cv::Scalar(0, 200, 0), -1);
    cv::putText(frame, "Triggered", cv::Point(120, y_pos + 22),
               cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(200, 200, 200), 1);

    cv::rectangle(frame, cv::Point(200, y_pos + 10), cv::Point(215, y_pos + 25),
                 cv::Scalar(80, 80, 80), -1);
    cv::putText(frame, "Disabled", cv::Point(220, y_pos + 22),
               cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(200, 200, 200), 1);
}

void Simulator::draw_waypoint_headings(cv::Mat& frame) {
    if (current_path_.waypoints.empty()) return;

    double scale = sim_config_.visualization.field_view_scale;
    double arrow_length = 0.6 * scale;  // 0.6 meters in screen pixels

    for (size_t i = 0; i < current_path_.waypoints.size(); i++) {
        const auto& wp = current_path_.waypoints[i];
        auto pt = field_to_screen(wp.pose.x, wp.pose.y);

        // Calculate arrow endpoint
        double cos_th = std::cos(wp.pose.theta);
        double sin_th = std::sin(wp.pose.theta);
        cv::Point arrow_end(
            static_cast<int>(pt.x + arrow_length * cos_th),
            static_cast<int>(pt.y - arrow_length * sin_th)  // Flip Y for screen coords
        );

        // Color based on selection state
        cv::Scalar arrow_color;
        if (waypoint_edit_.selected_waypoint == static_cast<int>(i)) {
            arrow_color = cv::Scalar(0, 255, 255);  // Yellow - selected
        } else if (waypoint_edit_.hover_waypoint == static_cast<int>(i)) {
            arrow_color = cv::Scalar(0, 200, 255);  // Orange - hover
        } else {
            arrow_color = cv::Scalar(200, 200, 255);  // Light red - normal
        }

        // Draw heading arrow
        cv::arrowedLine(frame,
            cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)),
            arrow_end,
            arrow_color, 2, cv::LINE_AA, 0, 0.3);

        // Draw heading angle text for selected waypoint
        if (waypoint_edit_.selected_waypoint == static_cast<int>(i)) {
            std::string angle_str = std::to_string(static_cast<int>(wp.pose.theta * 180 / M_PI)) + " deg";
            cv::putText(frame, angle_str,
                       cv::Point(static_cast<int>(pt.x) + 15, static_cast<int>(pt.y) - 20),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1);
        }
    }
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

    // Update current waypoint index based on trajectory progress
    if (!current_path_.waypoints.empty()) {
        // Estimate waypoint based on time fraction
        double progress = path_time_ / current_path_.total_time;
        int new_waypoint = static_cast<int>(progress * (current_path_.waypoints.size() - 1));
        if (new_waypoint > current_waypoint_index_) {
            current_waypoint_index_ = new_waypoint;
            std::cout << "[Path] Reached waypoint " << current_waypoint_index_ << std::endl;
        }
    }

    // Check and execute action blocks
    check_and_execute_action_blocks();

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

// ============================================================================
// Action Block Execution
// ============================================================================

std::set<int> Simulator::get_visible_tags() const {
    std::set<int> visible;
    for (const auto& det : last_detections_.detections) {
        visible.insert(det.id);
    }
    return visible;
}

std::map<int, double> Simulator::get_tag_distances() const {
    std::map<int, double> distances;
    Pose2D robot = dynamics_.state().true_pose;

    for (const auto& det : last_detections_.detections) {
        // Calculate distance from robot to tag
        if (field_.has_tag(det.id)) {
            const auto& tag = field_.get_tag(det.id);
            double dx = tag.center_field.x - robot.x;
            double dy = tag.center_field.y - robot.y;
            distances[det.id] = std::sqrt(dx * dx + dy * dy);
        }
    }
    return distances;
}

void Simulator::check_and_execute_action_blocks() {
    if (current_path_.action_blocks.empty()) return;

    // Get current state
    auto visible_tags = get_visible_tags();
    auto tag_distances = get_tag_distances();
    double robot_velocity = std::sqrt(
        dynamics_.state().vx * dynamics_.state().vx +
        dynamics_.state().vy * dynamics_.state().vy);

    // Check for triggered blocks
    auto triggered = PathPlanner::get_triggered_blocks(
        current_path_,
        current_waypoint_index_,
        path_time_,
        visible_tags,
        tag_distances,
        robot_velocity);

    // Execute triggered blocks
    for (const auto* block : triggered) {
        std::cout << "[ActionBlock] Triggered: " << block->name << std::endl;
        for (const auto& step : block->actions) {
            execute_action_step(step);
        }
    }
}

void Simulator::execute_action_step(const ActionBlock::ActionStep& step) {
    std::cout << "[ActionBlock] Executing: " << action_to_string(step.action) << std::endl;

    switch (step.action) {
        case RobotAction::AUTO_ALIGN_TO_TAG:
            // Enable auto-align mode
            if (!step.parameters.empty()) {
                auto it = step.parameters.find("tag_id");
                if (it != step.parameters.end()) {
                    int tag_id = static_cast<int>(it->second);
                    auto_align_.set_target_tag(tag_id);
                }
            }
            auto_align_.set_enabled(true);
            break;

        case RobotAction::FACE_TAG:
            // Rotate to face a specific tag
            if (!step.parameters.empty()) {
                auto it = step.parameters.find("tag_id");
                if (it != step.parameters.end()) {
                    int tag_id = static_cast<int>(it->second);
                    if (field_.has_tag(tag_id)) {
                        const auto& tag = field_.get_tag(tag_id);
                        Pose2D robot = dynamics_.state().true_pose;
                        double target_theta = std::atan2(
                            tag.center_field.y - robot.y,
                            tag.center_field.x - robot.x);
                        // Apply rotation (simplified)
                        std::cout << "[ActionBlock] Facing tag " << tag_id
                                  << " at angle " << (target_theta * 180 / M_PI) << " deg" << std::endl;
                    }
                }
            }
            break;

        case RobotAction::INTAKE_IN:
        case RobotAction::INTAKE_OUT:
        case RobotAction::INTAKE_STOP:
            std::cout << "[ActionBlock] Intake action (simulated)" << std::endl;
            break;

        case RobotAction::SPIN_UP_SHOOTER:
        case RobotAction::SHOOT:
        case RobotAction::SHOOTER_STOP:
            std::cout << "[ActionBlock] Shooter action (simulated)" << std::endl;
            break;

        case RobotAction::SET_ELEVATOR_HEIGHT:
            if (!step.parameters.empty()) {
                auto it = step.parameters.find("height");
                if (it != step.parameters.end()) {
                    std::cout << "[ActionBlock] Set elevator to " << it->second << "m" << std::endl;
                }
            }
            break;

        case RobotAction::STOW:
            std::cout << "[ActionBlock] Stowing mechanisms" << std::endl;
            break;

        case RobotAction::WAIT:
            if (!step.parameters.empty()) {
                auto it = step.parameters.find("time");
                if (it != step.parameters.end()) {
                    std::cout << "[ActionBlock] Wait " << it->second << "s (simulated)" << std::endl;
                }
            }
            break;

        default:
            std::cout << "[ActionBlock] Unknown action" << std::endl;
            break;
    }
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

        // Handle path execution or manual control
        if (mode_ == SimMode::PATH_EXECUTE) {
            handle_path_execution(dt);
        } else {
            handle_auto_align(dt);
        }

        // Always update dynamics (path execution sets velocity commands)
        update_dynamics(dt);

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

        // Action blocks
        case 'b': {
            // Add action block at selected waypoint (or last waypoint)
            int wp_idx = waypoint_edit_.selected_waypoint;
            if (wp_idx < 0 && !current_path_.waypoints.empty()) {
                wp_idx = static_cast<int>(current_path_.waypoints.size()) - 1;
            }

            if (wp_idx >= 0 && wp_idx < static_cast<int>(current_path_.waypoints.size())) {
                // Show action selection menu for this waypoint
                auto pt = field_to_screen(current_path_.waypoints[wp_idx].pose.x,
                                         current_path_.waypoints[wp_idx].pose.y);
                show_waypoint_action_menu(wp_idx, static_cast<int>(pt.x), static_cast<int>(pt.y));
            } else {
                std::cout << "[ActionBlock] Add waypoints first (P to enter path edit mode)" << std::endl;
            }
            break;
        }

        case 'l':
        case 't': {
            // List/toggle action blocks
            std::cout << "\n========== ACTION BLOCKS ==========\n" << std::endl;
            if (current_path_.action_blocks.empty()) {
                std::cout << "No action blocks defined." << std::endl;
                std::cout << "Right-click waypoints or tags to add actions." << std::endl;
                std::cout << "Or press B to add block at selected waypoint." << std::endl;
            } else {
                for (size_t i = 0; i < current_path_.action_blocks.size(); i++) {
                    const auto& block = current_path_.action_blocks[i];
                    std::cout << "[" << i << "] " << block.name;
                    std::cout << " - " << (block.enabled ? "ENABLED" : "disabled");
                    if (block.has_triggered) std::cout << " (Triggered)";
                    std::cout << std::endl;
                    std::cout << "    " << block.describe() << std::endl << std::endl;
                }
                std::cout << "Click on block icons to toggle, or press 0-9 to toggle by index." << std::endl;
            }
            std::cout << "===================================\n" << std::endl;
            break;
        }

        // Toggle action blocks by index (0-9)
        case '0': case '6': case '7': case '8': case '9': {
            // Only handle if not a display toggle (1-5 are for display)
            int block_idx = key - '0';
            if (block_idx >= 0 && block_idx < static_cast<int>(current_path_.action_blocks.size())) {
                toggle_action_block(block_idx);
            }
            break;
        }

        // Waypoint heading controls
        case '[': {
            // Rotate selected waypoint heading CCW by 15 degrees
            if (waypoint_edit_.selected_waypoint >= 0 &&
                waypoint_edit_.selected_waypoint < static_cast<int>(current_path_.waypoints.size())) {
                double current = current_path_.waypoints[waypoint_edit_.selected_waypoint].pose.theta;
                update_waypoint_heading(waypoint_edit_.selected_waypoint, current + M_PI / 12);
            } else {
                std::cout << "[Path] Select a waypoint first (click on it)" << std::endl;
            }
            break;
        }
        case ']': {
            // Rotate selected waypoint heading CW by 15 degrees
            if (waypoint_edit_.selected_waypoint >= 0 &&
                waypoint_edit_.selected_waypoint < static_cast<int>(current_path_.waypoints.size())) {
                double current = current_path_.waypoints[waypoint_edit_.selected_waypoint].pose.theta;
                update_waypoint_heading(waypoint_edit_.selected_waypoint, current - M_PI / 12);
            } else {
                std::cout << "[Path] Select a waypoint first (click on it)" << std::endl;
            }
            break;
        }

        // Delete selected waypoint
        case 127:  // Backspace
        case 8: {  // Delete
            if (waypoint_edit_.selected_waypoint >= 0 &&
                waypoint_edit_.selected_waypoint < static_cast<int>(current_path_.waypoints.size())) {
                int idx = waypoint_edit_.selected_waypoint;
                current_path_.waypoints.erase(current_path_.waypoints.begin() + idx);
                std::cout << "[Path] Deleted waypoint " << idx << std::endl;
                waypoint_edit_.selected_waypoint = -1;

                // Re-number waypoints
                for (size_t i = 0; i < current_path_.waypoints.size(); i++) {
                    current_path_.waypoints[i].name = "WP" + std::to_string(i);
                }
            }
            break;
        }

        case 27: // ESC
            // First deselect waypoint, then quit
            if (waypoint_edit_.selected_waypoint >= 0) {
                waypoint_edit_.selected_waypoint = -1;
                std::cout << "[Path] Waypoint deselected" << std::endl;
            } else {
                shutdown_requested_.store(true);
            }
            break;
    }
}

void Simulator::update_dynamics(double dt) {
    // Only disable external command mode if not in auto-align or path execution
    if (!auto_align_.is_enabled() && mode_ != SimMode::PATH_EXECUTE) {
        dynamics_.set_external_command_mode(false);
    }

    // Clear stale input if no recent key presses (for manual driving only)
    if (mode_ == SimMode::DRIVE) {
        auto now = Clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(now - key_hold_time_).count();
        if (elapsed_ms > 150.0) {
            input_.forward = input_.backward = false;
            input_.left = input_.right = false;
            input_.rotate_ccw = input_.rotate_cw = false;
            input_.turbo = false;
        }
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
            mode_str = "MODE: PATH EDIT (click/right-click field)";
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
        case SimMode::ACTION_MENU:
            mode_str = "MODE: ACTION MENU (Tag " + std::to_string(action_menu_.selected_tag_id) + ")";
            mode_color = cv::Scalar(255, 0, 255);
            break;
        default:
            mode_str = "MODE: DRIVE (right-click tags for actions)";
            break;
    }
    cv::putText(display_frame_, mode_str, cv::Point(10, 100),
               cv::FONT_HERSHEY_SIMPLEX, 0.6, mode_color, 2);

    cv::imshow(CAMERA_WINDOW, display_frame_);

    // Field view
    cv::Mat field_view = topdown_renderer_.render(dynamics_.state());

    // Draw waypoints and path
    if (!current_path_.waypoints.empty()) {
        // Draw trajectory first (underneath waypoints)
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

        // Draw lines between waypoints
        for (size_t i = 1; i < current_path_.waypoints.size(); i++) {
            const auto& wp = current_path_.waypoints[i];
            const auto& prev = current_path_.waypoints[i - 1];
            auto pt = field_to_screen(wp.pose.x, wp.pose.y);
            auto prev_pt = field_to_screen(prev.pose.x, prev.pose.y);
            cv::line(field_view,
                    cv::Point(static_cast<int>(prev_pt.x), static_cast<int>(prev_pt.y)),
                    cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)),
                    cv::Scalar(255, 165, 0), 2);
        }

        // Draw waypoint headings (arrows showing robot facing direction)
        draw_waypoint_headings(field_view);

        // Draw waypoint circles on top
        for (size_t i = 0; i < current_path_.waypoints.size(); i++) {
            const auto& wp = current_path_.waypoints[i];
            auto pt = field_to_screen(wp.pose.x, wp.pose.y);

            // Color based on selection/state
            cv::Scalar color;
            if (waypoint_edit_.selected_waypoint == static_cast<int>(i)) {
                color = cv::Scalar(0, 255, 255);  // Yellow - selected
            } else if (waypoint_edit_.hover_waypoint == static_cast<int>(i)) {
                color = cv::Scalar(0, 200, 255);  // Orange - hover
            } else if (i == 0) {
                color = cv::Scalar(0, 255, 0);    // Green - start
            } else if (i == current_path_.waypoints.size() - 1) {
                color = cv::Scalar(0, 0, 255);    // Red - end
            } else {
                color = cv::Scalar(255, 165, 0);  // Orange - intermediate
            }

            cv::circle(field_view, cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)),
                      10, color, -1);
            cv::circle(field_view, cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)),
                      10, cv::Scalar(255, 255, 255), 1);

            // Waypoint number/name
            cv::putText(field_view, std::to_string(i),
                       cv::Point(static_cast<int>(pt.x) - 4, static_cast<int>(pt.y) + 4),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
        }
    }

    // Draw action blocks (visual block coding display)
    draw_action_blocks(field_view);

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

    // Draw action menu if visible
    draw_action_menu(field_view);

    // Add hint for right-click
    if (mode_ != SimMode::ACTION_MENU) {
        cv::putText(field_view, "Right-click tags for actions",
                   cv::Point(field_view.cols - 200, 20),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(180, 180, 180), 1);
    }

    cv::imshow(FIELD_WINDOW, field_view);
}

} // namespace sim
} // namespace frc_vision
