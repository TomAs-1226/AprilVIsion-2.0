/**
 * @file simulator.cpp
 * @brief Simulator implementation
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
    std::cout << "FRC Vision Mac Simulator v2.0" << std::endl;
    std::cout << "2024 CRESCENDO Field Layout" << std::endl;
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

    // Initialize vision pipeline
    if (!initialize_vision_pipeline()) {
        return false;
    }

    // Create windows
    cv::namedWindow(CAMERA_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(FIELD_WINDOW, cv::WINDOW_AUTOSIZE);

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

        // Use defaults
        sim_config_ = SimConfig();
        return true;
    }

    try {
        YAML::Node config = YAML::LoadFile(path);

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
    // Try config-relative path first
    fs::path layout_path = fs::path(config_dir_) / sim_config_.field_layout_path;

    // Try assets directory
    if (!fs::exists(layout_path)) {
        layout_path = fs::path(config_dir_) / ".." / "assets" / "2024-crescendo.json";
    }

    // Try current directory
    if (!fs::exists(layout_path)) {
        layout_path = "assets/2024-crescendo.json";
    }

    if (!fs::exists(layout_path)) {
        std::cerr << "\n[ERROR] Field layout not found!" << std::endl;
        std::cerr << "Please ensure assets/2024-crescendo.json exists." << std::endl;
        std::cerr << "\nTo fix:" << std::endl;
        std::cerr << "  ./scripts/fetch_layout.sh" << std::endl;
        std::cerr << "\nOr download from:" << std::endl;
        std::cerr << "  https://github.com/wpilibsuite/allwpilib/raw/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2024-crescendo.json" << std::endl;
        return false;
    }

    try {
        // Load using our existing loader, but we need to parse WPILib format
        // The field_layout.cpp expects a different format, so we'll parse directly here

        std::ifstream file(layout_path);
        nlohmann::json json;
        file >> json;

        field_.name = "FRC 2024 CRESCENDO";
        field_.tag_size_m = 0.1651;  // 6.5 inches

        double half_size = field_.tag_size_m / 2.0;

        for (const auto& tag_json : json["tags"]) {
            int id = tag_json["ID"];
            FieldTag tag;
            tag.id = id;

            // Parse translation
            double tx = tag_json["pose"]["translation"]["x"];
            double ty = tag_json["pose"]["translation"]["y"];
            double tz = tag_json["pose"]["translation"]["z"];

            tag.center_field = {tx, ty, tz};

            // Parse quaternion rotation
            double qw = tag_json["pose"]["rotation"]["quaternion"]["W"];
            double qx = tag_json["pose"]["rotation"]["quaternion"]["X"];
            double qy = tag_json["pose"]["rotation"]["quaternion"]["Y"];
            double qz = tag_json["pose"]["rotation"]["quaternion"]["Z"];

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

            // Convert to rvec
            cv::Vec3d rvec;
            cv::Rodrigues(R, rvec);
            tag.pose_field = Pose3D::from_rvec_tvec(rvec, cv::Vec3d(tx, ty, tz));

            // Compute corners in field frame
            // Tag corners in tag-local frame (counter-clockwise from bottom-left)
            std::array<cv::Point3d, 4> corners_local = {
                cv::Point3d(-half_size, -half_size, 0),  // Bottom-left
                cv::Point3d(half_size, -half_size, 0),   // Bottom-right
                cv::Point3d(half_size, half_size, 0),    // Top-right
                cv::Point3d(-half_size, half_size, 0)    // Top-left
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

        std::cout << "[Sim] Loaded " << field_.tags.size() << " AprilTags from "
                  << layout_path << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[Sim] Error loading field layout: " << e.what() << std::endl;
        return false;
    }
}

bool Simulator::load_camera_calibration() {
    // Load intrinsics
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
            std::cout << "[Sim] Loaded intrinsics from: " << intr_path << std::endl;
        }
    }

    // Set defaults if not loaded
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
    if (!fs::exists(extr_path)) {
        extr_path = "assets/mac_cam_extrinsics.yml";
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

            // Compute rotation matrix from Euler angles (ZYX convention)
            cv::Mat Rx = (cv::Mat_<double>(3, 3) <<
                1, 0, 0,
                0, cos(roll), -sin(roll),
                0, sin(roll), cos(roll));
            cv::Mat Ry = (cv::Mat_<double>(3, 3) <<
                cos(pitch), 0, sin(pitch),
                0, 1, 0,
                -sin(pitch), 0, cos(pitch));
            cv::Mat Rz = (cv::Mat_<double>(3, 3) <<
                cos(yaw), -sin(yaw), 0,
                sin(yaw), cos(yaw), 0,
                0, 0, 1);

            cv::Mat R = Rz * Ry * Rx;
            cv::Vec3d rvec;
            cv::Rodrigues(R, rvec);

            camera_to_robot_ = Pose3D::from_rvec_tvec(rvec, cv::Vec3d(x, y, z));

            std::cout << "[Sim] Loaded extrinsics from: " << extr_path << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[Sim] Error loading extrinsics: " << e.what() << std::endl;
        }
    }

    // Default extrinsics if not loaded
    if (camera_to_robot_.position.x == 0 && camera_to_robot_.position.z == 0) {
        camera_to_robot_.position = {0.30, 0.0, 0.45};
        // Small downward pitch
        cv::Mat R = (cv::Mat_<double>(3, 3) <<
            1, 0, 0,
            0, cos(0.26), -sin(0.26),
            0, sin(0.26), cos(0.26));
        cv::Vec3d rvec;
        cv::Rodrigues(R, rvec);
        camera_to_robot_.orientation = Quaternion::from_rvec(rvec);
    }

    return true;
}

bool Simulator::initialize_webcam() {
    std::cout << "[Sim] Opening webcam device " << sim_config_.webcam_device << "..." << std::endl;

    webcam_.open(sim_config_.webcam_device);

    if (!webcam_.isOpened()) {
        return false;
    }

    // Configure webcam
    webcam_.set(cv::CAP_PROP_FRAME_WIDTH, sim_config_.camera.width);
    webcam_.set(cv::CAP_PROP_FRAME_HEIGHT, sim_config_.camera.height);
    webcam_.set(cv::CAP_PROP_FPS, sim_config_.camera.target_fps);

    // Start webcam capture thread
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

    std::cout << "[Sim] Webcam initialized: "
              << webcam_.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
              << webcam_.get(cv::CAP_PROP_FRAME_HEIGHT) << " @ "
              << webcam_.get(cv::CAP_PROP_FPS) << " fps" << std::endl;

    return true;
}

bool Simulator::initialize_vision_pipeline() {
    // Create vision pipeline
    pipeline_ = std::make_unique<VisionPipeline>();

    // Use default detector and tracker config
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

    std::cout << "[Sim] Vision pipeline initialized" << std::endl;
    return true;
}

void Simulator::print_keybinds() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "CONTROLS:" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  WASD     - Move robot (forward/back/strafe)" << std::endl;
    std::cout << "  Q/E      - Rotate CCW/CW" << std::endl;
    std::cout << "  SHIFT    - Turbo mode (faster movement)" << std::endl;
    std::cout << "  V        - Toggle auto-align to nearest tag" << std::endl;
    std::cout << "  R        - Reset robot pose" << std::endl;
    std::cout << "  1        - Toggle true pose (green)" << std::endl;
    std::cout << "  2        - Toggle odometry pose (orange)" << std::endl;
    std::cout << "  3        - Toggle fused pose (magenta)" << std::endl;
    std::cout << "  4        - Toggle webcam composite" << std::endl;
    std::cout << "  5        - Toggle detect on composite vs synthetic" << std::endl;
    std::cout << "  ESC      - Quit" << std::endl;
    std::cout << "========================================\n" << std::endl;
}

int Simulator::run() {
    std::cout << "[Sim] Running... Press ESC to quit.\n" << std::endl;

    while (!shutdown_requested_.load()) {
        auto now = Clock::now();
        double dt = std::chrono::duration<double>(now - last_update_time_).count();
        last_update_time_ = now;

        // Clamp dt to avoid physics explosions
        dt = std::min(dt, 0.1);

        // Process input
        process_input();

        if (shutdown_requested_.load()) break;

        // Update dynamics
        update_dynamics(dt);

        // Handle auto-align
        handle_auto_align(dt);

        // Capture and render
        capture_and_render();

        // Run detection
        run_detection();

        // Update fusion
        update_fusion();

        // Update visualization
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
    // Note: Don't reset movement keys here - they stay active until handle_key releases them
}

void Simulator::handle_key(int key) {
    // Handle lowercase
    if (key >= 'A' && key <= 'Z') {
        key = key - 'A' + 'a';
    }

    // Reset movement key hold timers - movement keys need repeated key events to stay active
    // When a WASD/QE key is pressed, it sets the flag and resets the timer
    // The timer in update_dynamics will clear the flag if no key event in ~100ms

    switch (key) {
        // Movement - these stay active while key is held (key repeat)
        case 'w': input_.forward = true; key_hold_time_ = Clock::now(); break;
        case 's': input_.backward = true; key_hold_time_ = Clock::now(); break;
        case 'a': input_.left = true; key_hold_time_ = Clock::now(); break;
        case 'd': input_.right = true; key_hold_time_ = Clock::now(); break;
        case 'q': input_.rotate_ccw = true; key_hold_time_ = Clock::now(); break;
        case 'e': input_.rotate_cw = true; key_hold_time_ = Clock::now(); break;

        // Shift for turbo (varies by platform)
        case 0x10:
        case 225:  // macOS left shift
        case 229:  // macOS right shift
            input_.turbo = true;
            key_hold_time_ = Clock::now();
            break;

        // Toggles - one-shot, don't need hold
        case 'v':
            input_.auto_align = !input_.auto_align;
            auto_align_.set_enabled(input_.auto_align);
            std::cout << "[Sim] Auto-align: " << (input_.auto_align ? "ENABLED" : "disabled") << std::endl;
            break;

        case 'r':
            dynamics_.reset(sim_config_.start_pose);
            auto_align_.reset();
            input_.auto_align = false;
            auto_align_.set_enabled(false);
            std::cout << "[Sim] Robot pose reset" << std::endl;
            break;

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
            std::cout << "[Sim] Webcam: " << (input_.show_webcam ? "ON" : "OFF") << std::endl;
            break;

        case '5':
            input_.detect_on_composite = !input_.detect_on_composite;
            std::cout << "[Sim] Detection on: " << (input_.detect_on_composite ? "composite" : "synthetic") << std::endl;
            break;

        case 27: // ESC
            shutdown_requested_.store(true);
            break;

        default:
            // Unknown key - don't clear movement, it might be a modifier
            break;
    }
}

void Simulator::update_dynamics(double dt) {
    if (!auto_align_.is_enabled()) {
        dynamics_.set_external_command_mode(false);
    }

    // Check if we should clear movement keys (no key event for 150ms)
    auto now = Clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(now - key_hold_time_).count();
    if (elapsed_ms > 150.0) {
        // No recent key event - clear all movement
        input_.forward = input_.backward = false;
        input_.left = input_.right = false;
        input_.rotate_ccw = input_.rotate_cw = false;
        input_.turbo = false;
    }

    dynamics_.update(input_, dt);
}

void Simulator::handle_auto_align(double dt) {
    if (!auto_align_.is_enabled()) return;

    // Get visible tags
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

    // Render synthetic view
    cv::Mat synthetic = field_renderer_.render(true_pose);

    // Get webcam frame if available
    cv::Mat webcam;
    if (sim_config_.use_webcam && input_.show_webcam) {
        std::lock_guard<std::mutex> lock(webcam_mutex_);
        if (!webcam_frame_.empty()) {
            webcam = webcam_frame_.clone();
        }
    }

    // Create composite or use synthetic
    if (!webcam.empty()) {
        // Resize webcam to match synthetic if needed
        if (webcam.size() != synthetic.size()) {
            cv::resize(webcam, webcam, synthetic.size());
        }

        // Composite (mode 0 = overlay tags on webcam)
        current_frame_ = field_renderer_.composite(webcam, true_pose, 0);
    } else {
        current_frame_ = synthetic;
    }

    // Display frame
    display_frame_ = current_frame_.clone();
}

void Simulator::run_detection() {
    // Choose frame for detection
    cv::Mat detect_frame;
    if (input_.detect_on_composite) {
        detect_frame = current_frame_;
    } else {
        detect_frame = field_renderer_.render(dynamics_.state().true_pose);
    }

    // Create Frame object
    Frame frame;
    frame.camera_id = 0;
    frame.frame_number = frame_count_;
    frame.capture_time = SteadyClock::now();
    frame.capture_wall_time = SystemClock::now();
    frame.image = detect_frame;

    // Process through pipeline
    auto processed = pipeline_->process_frame(frame, intrinsics_, camera_to_robot_);
    last_detections_ = processed.detections;

    // Update stats
    stats_.tag_count = static_cast<int>(last_detections_.detections.size());
    stats_.detect_ms = last_detections_.timestamps.detect_ms();
    stats_.pose_ms = last_detections_.timestamps.pose_ms();
    stats_.reproj_error = last_detections_.avg_reproj_error();

    if (last_detections_.multi_tag_pose_valid) {
        stats_.confidence = 1.0;  // Could compute from quality
    } else if (!last_detections_.detections.empty()) {
        stats_.confidence = 0.5;
    } else {
        stats_.confidence = 0.0;
    }

    // Draw detections on display frame
    for (const auto& det : last_detections_.detections) {
        // Draw corners
        for (int i = 0; i < 4; i++) {
            cv::Point2d p1(det.corners.corners[i].x, det.corners.corners[i].y);
            cv::Point2d p2(det.corners.corners[(i+1)%4].x, det.corners.corners[(i+1)%4].y);
            cv::line(display_frame_, p1, p2, cv::Scalar(0, 255, 0), 2);
        }

        // Draw ID
        auto center = det.corners.center();
        cv::putText(display_frame_, std::to_string(det.id),
                   cv::Point(static_cast<int>(center.x) - 10, static_cast<int>(center.y)),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
    }
}

void Simulator::update_fusion() {
    // Get fused pose from pipeline
    auto fused = pipeline_->get_fused_pose();

    if (fused.valid) {
        // Update robot state with vision-corrected pose
        dynamics_.state().fused_pose = fused.pose_filtered;
    } else {
        // Use true pose as fallback (for now)
        dynamics_.state().fused_pose = dynamics_.state().true_pose;
    }
}

void Simulator::update_visualization() {
    // Draw stats on camera view
    std::string fps_text = "FPS: " + std::to_string(static_cast<int>(stats_.fps));
    std::string tags_text = "Tags: " + std::to_string(stats_.tag_count);
    std::string latency_text = "Latency: " + std::to_string(static_cast<int>(stats_.detect_ms + stats_.pose_ms)) + "ms";

    cv::putText(display_frame_, fps_text, cv::Point(10, 25),
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    cv::putText(display_frame_, tags_text, cv::Point(10, 50),
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    cv::putText(display_frame_, latency_text, cv::Point(10, 75),
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

    if (auto_align_.is_enabled()) {
        std::string align_text = "AUTO-ALIGN: Tag " + std::to_string(auto_align_.target_tag_id());
        if (auto_align_.is_at_target()) {
            align_text += " [AT TARGET]";
        }
        cv::putText(display_frame_, align_text, cv::Point(10, 100),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }

    cv::imshow(CAMERA_WINDOW, display_frame_);

    // Render top-down view
    cv::Mat field_view = topdown_renderer_.render(dynamics_.state());
    topdown_renderer_.draw_stats(field_view, stats_.fps, stats_.detect_ms + stats_.pose_ms,
                                 stats_.tag_count, stats_.reproj_error, stats_.confidence,
                                 auto_align_.target_tag_id(), auto_align_.is_enabled());

    cv::imshow(FIELD_WINDOW, field_view);
}

} // namespace sim
} // namespace frc_vision
