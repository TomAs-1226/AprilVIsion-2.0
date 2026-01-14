/**
 * @file config.cpp
 * @brief Configuration loading implementation
 */

#include "config.hpp"
#include <yaml-cpp/yaml.h>
#include <opencv2/core.hpp>
#include <fstream>
#include <iostream>
#include <filesystem>

namespace frc_vision {

ConfigManager::ConfigManager() = default;
ConfigManager::~ConfigManager() = default;

bool ConfigManager::load(const std::string& path) {
    config_path_ = path;
    return reload();
}

bool ConfigManager::reload() {
    try {
        if (!std::filesystem::exists(config_path_)) {
            std::cerr << "[Config] File not found: " << config_path_ << std::endl;
            return false;
        }

        YAML::Node root = YAML::LoadFile(config_path_);

        // Parse cameras
        if (root["cameras"]) {
            YAML::Node cameras_node = root["cameras"];
            parse_cameras(&cameras_node);
        }

        // Parse detector settings
        if (root["apriltag"]) {
            YAML::Node apriltag_node = root["apriltag"];
            parse_detector(&apriltag_node);
        }

        // Parse tracking settings
        if (root["tracking"]) {
            YAML::Node tracking_node = root["tracking"];
            parse_tracker(&tracking_node);
        }

        // Parse field layout
        if (root["field"]) {
            YAML::Node field_node = root["field"];
            parse_field(&field_node);
        }

        // Parse output settings
        if (root["outputs"]) {
            YAML::Node outputs_node = root["outputs"];
            parse_output(&outputs_node);
        }

        // Parse performance settings
        if (root["performance"]) {
            YAML::Node performance_node = root["performance"];
            parse_performance(&performance_node);
        }

        std::cout << "[Config] Loaded configuration from " << config_path_ << std::endl;
        std::cout << "[Config] " << config_.cameras.size() << " cameras configured" << std::endl;
        std::cout << "[Config] " << config_.field.tags.size() << " field tags loaded" << std::endl;

        config_.reload_requested.store(false);
        return true;

    } catch (const YAML::Exception& e) {
        std::cerr << "[Config] YAML parse error: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "[Config] Error loading config: " << e.what() << std::endl;
        return false;
    }
}

bool ConfigManager::parse_cameras(void* node_ptr) {
    YAML::Node& node = *static_cast<YAML::Node*>(node_ptr);
    config_.cameras.clear();

    for (const auto& cam_node : node) {
        CameraConfig cam;

        cam.name = cam_node["name"].as<std::string>("cam");
        cam.device = cam_node["device"].as<std::string>("/dev/video0");
        cam.width = cam_node["width"].as<int>(640);
        cam.height = cam_node["height"].as<int>(480);
        cam.fps = cam_node["fps"].as<int>(60);
        cam.exposure = cam_node["exposure"].as<int>(-1);
        cam.gain = cam_node["gain"].as<int>(-1);
        cam.format = cam_node["format"].as<std::string>("MJPG");

        if (cam_node["intrinsics"]) {
            cam.intrinsics_file = cam_node["intrinsics"].as<std::string>();
        }

        if (cam_node["extrinsics"]) {
            YAML::Node extrinsics_node = cam_node["extrinsics"];
            cam.camera_to_robot = parse_transform(&extrinsics_node);
        }

        config_.cameras.push_back(std::move(cam));
    }

    return true;
}

bool ConfigManager::parse_detector(void* node_ptr) {
    YAML::Node& node = *static_cast<YAML::Node*>(node_ptr);

    config_.detector.family = node["family"].as<std::string>("tag36h11");
    config_.detector.decimation = node["decimation"].as<int>(2);
    config_.detector.sigma = node["sigma"].as<double>(0.0);
    config_.detector.nthreads = node["nthreads"].as<int>(4);
    config_.detector.refine_edges = node["refine_edges"].as<bool>(true);
    config_.detector.max_hamming = node["max_hamming"].as<int>(1);
    config_.detector.min_margin = node["min_margin"].as<double>(20.0);
    config_.detector.max_tags_per_frame = node["max_tags_per_frame"].as<int>(16);

    return true;
}

bool ConfigManager::parse_tracker(void* node_ptr) {
    YAML::Node& node = *static_cast<YAML::Node*>(node_ptr);

    config_.tracker.enable = node["enable"].as<bool>(true);
    config_.tracker.dropout_ms = node["dropout_ms"].as<int>(150);
    config_.tracker.filter_alpha = node["filter_alpha"].as<double>(0.3);
    config_.tracker.roi_enable = node["roi_enable"].as<bool>(false);
    config_.tracker.velocity_decay = node["velocity_decay"].as<double>(0.9);

    return true;
}

bool ConfigManager::parse_field(void* node_ptr) {
    YAML::Node& node = *static_cast<YAML::Node*>(node_ptr);

    config_.field.tag_size_m = node["tag_size_m"].as<double>(0.1651);
    config_.field.name = node["name"].as<std::string>("FRC 2026");

    // Load field layout from JSON file if specified
    if (node["layout_file"]) {
        std::string layout_file = node["layout_file"].as<std::string>();

        // Resolve relative path
        std::filesystem::path config_dir = std::filesystem::path(config_path_).parent_path();
        std::filesystem::path layout_path = config_dir / layout_file;

        // Actual loading happens in field_layout.cpp
        // Store the path for later
        // The field_layout module will populate the tags
    }

    return true;
}

bool ConfigManager::parse_output(void* node_ptr) {
    YAML::Node& node = *static_cast<YAML::Node*>(node_ptr);

    config_.output.nt_enable = node["nt_enable"].as<bool>(true);
    config_.output.nt_server = node["nt_server"].as<std::string>("10.0.0.2");
    config_.output.nt_table_root = node["nt_table_root"].as<std::string>("/FRCVision");
    config_.output.web_port = node["web_port"].as<int>(5800);
    config_.output.publish_rate_hz = node["publish_rate_hz"].as<int>(50);

    return true;
}

bool ConfigManager::parse_performance(void* node_ptr) {
    YAML::Node& node = *static_cast<YAML::Node*>(node_ptr);

    config_.performance.jpeg_quality = node["jpeg_quality"].as<int>(70);
    config_.performance.enable_annotations = node["enable_annotations"].as<bool>(true);
    config_.performance.annotation_line_width = node["annotation_line_width"].as<int>(2);

    return true;
}

Pose3D ConfigManager::parse_transform(void* node_ptr) {
    YAML::Node& node = *static_cast<YAML::Node*>(node_ptr);
    Pose3D pose;

    // Position
    pose.position.x = node["x"].as<double>(0.0);
    pose.position.y = node["y"].as<double>(0.0);
    pose.position.z = node["z"].as<double>(0.0);

    // Rotation (roll, pitch, yaw in degrees -> convert to quaternion)
    double roll = node["roll"].as<double>(0.0) * M_PI / 180.0;
    double pitch = node["pitch"].as<double>(0.0) * M_PI / 180.0;
    double yaw = node["yaw"].as<double>(0.0) * M_PI / 180.0;

    // Convert Euler angles to quaternion (ZYX convention)
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    pose.orientation.w = cr * cp * cy + sr * sp * sy;
    pose.orientation.x = sr * cp * cy - cr * sp * sy;
    pose.orientation.y = cr * sp * cy + sr * cp * sy;
    pose.orientation.z = cr * cp * sy - sr * sp * cy;

    return pose;
}

std::optional<CameraIntrinsics> ConfigManager::load_intrinsics(const std::string& path) {
    try {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cerr << "[Config] Cannot open intrinsics file: " << path << std::endl;
            return std::nullopt;
        }

        CameraIntrinsics intrinsics;

        fs["camera_matrix"] >> intrinsics.camera_matrix;
        fs["distortion_coefficients"] >> intrinsics.dist_coeffs;

        // Also try alternate names
        if (intrinsics.camera_matrix.empty()) {
            fs["K"] >> intrinsics.camera_matrix;
        }
        if (intrinsics.dist_coeffs.empty()) {
            fs["D"] >> intrinsics.dist_coeffs;
        }

        fs["image_width"] >> intrinsics.width;
        fs["image_height"] >> intrinsics.height;

        if (intrinsics.camera_matrix.empty()) {
            std::cerr << "[Config] No camera_matrix found in: " << path << std::endl;
            return std::nullopt;
        }

        // Convert to double if needed
        if (intrinsics.camera_matrix.type() != CV_64F) {
            intrinsics.camera_matrix.convertTo(intrinsics.camera_matrix, CV_64F);
        }
        if (!intrinsics.dist_coeffs.empty() && intrinsics.dist_coeffs.type() != CV_64F) {
            intrinsics.dist_coeffs.convertTo(intrinsics.dist_coeffs, CV_64F);
        }

        std::cout << "[Config] Loaded intrinsics from " << path << std::endl;
        return intrinsics;

    } catch (const cv::Exception& e) {
        std::cerr << "[Config] OpenCV error loading intrinsics: " << e.what() << std::endl;
        return std::nullopt;
    }
}

} // namespace frc_vision
