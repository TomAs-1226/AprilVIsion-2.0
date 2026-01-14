/**
 * @file field_layout.cpp
 * @brief Field tag layout loading implementation
 */

#include "field_layout.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <cmath>

namespace frc_vision {

using json = nlohmann::json;

std::array<Point3D, 4> get_tag_corners_local(double tag_size_m) {
    // Tag corners in tag-local frame (Z pointing out of tag)
    // Counter-clockwise from bottom-left when viewing tag face-on
    double half = tag_size_m / 2.0;
    return {{
        {-half, -half, 0.0},  // bottom-left
        { half, -half, 0.0},  // bottom-right
        { half,  half, 0.0},  // top-right
        {-half,  half, 0.0}   // top-left
    }};
}

FieldLayout load_field_layout(const std::string& path, double tag_size_m) {
    FieldLayout layout;
    layout.tag_size_m = tag_size_m;

    try {
        std::ifstream file(path);
        if (!file.is_open()) {
            std::cerr << "[FieldLayout] Cannot open file: " << path << std::endl;
            return create_default_field_layout(tag_size_m);
        }

        json root = json::parse(file);

        if (root.contains("name")) {
            layout.name = root["name"].get<std::string>();
        }

        if (root.contains("tag_size_m")) {
            layout.tag_size_m = root["tag_size_m"].get<double>();
        }

        auto corners_local = get_tag_corners_local(layout.tag_size_m);

        if (root.contains("tags")) {
            for (const auto& tag_json : root["tags"]) {
                FieldTag tag;
                tag.id = tag_json["id"].get<int>();

                // Parse tag pose in field frame
                if (tag_json.contains("pose")) {
                    const auto& pose = tag_json["pose"];
                    tag.center_field.x = pose["x"].get<double>();
                    tag.center_field.y = pose["y"].get<double>();
                    tag.center_field.z = pose["z"].get<double>();

                    // Rotation (quaternion or euler)
                    if (pose.contains("qw")) {
                        tag.pose_field.orientation.w = pose["qw"].get<double>();
                        tag.pose_field.orientation.x = pose["qx"].get<double>();
                        tag.pose_field.orientation.y = pose["qy"].get<double>();
                        tag.pose_field.orientation.z = pose["qz"].get<double>();
                    } else {
                        // Euler angles (degrees)
                        double roll = pose.value("roll", 0.0) * M_PI / 180.0;
                        double pitch = pose.value("pitch", 0.0) * M_PI / 180.0;
                        double yaw = pose.value("yaw", 0.0) * M_PI / 180.0;

                        double cy = std::cos(yaw * 0.5);
                        double sy = std::sin(yaw * 0.5);
                        double cp = std::cos(pitch * 0.5);
                        double sp = std::sin(pitch * 0.5);
                        double cr = std::cos(roll * 0.5);
                        double sr = std::sin(roll * 0.5);

                        tag.pose_field.orientation.w = cr * cp * cy + sr * sp * sy;
                        tag.pose_field.orientation.x = sr * cp * cy - cr * sp * sy;
                        tag.pose_field.orientation.y = cr * sp * cy + sr * cp * sy;
                        tag.pose_field.orientation.z = cr * cp * sy - sr * sp * cy;
                    }

                    tag.pose_field.position = tag.center_field;
                }

                // Transform local corners to field frame
                cv::Mat R;
                cv::Rodrigues(tag.pose_field.rvec(), R);

                for (int i = 0; i < 4; i++) {
                    cv::Mat p_local = (cv::Mat_<double>(3, 1) <<
                        corners_local[i].x, corners_local[i].y, corners_local[i].z);
                    cv::Mat p_field = R * p_local;

                    tag.corners_field[i].x = p_field.at<double>(0) + tag.center_field.x;
                    tag.corners_field[i].y = p_field.at<double>(1) + tag.center_field.y;
                    tag.corners_field[i].z = p_field.at<double>(2) + tag.center_field.z;
                }

                layout.tags[tag.id] = tag;
            }
        }

        std::cout << "[FieldLayout] Loaded " << layout.tags.size()
                  << " tags from " << path << std::endl;

    } catch (const json::exception& e) {
        std::cerr << "[FieldLayout] JSON parse error: " << e.what() << std::endl;
        return create_default_field_layout(tag_size_m);
    } catch (const std::exception& e) {
        std::cerr << "[FieldLayout] Error: " << e.what() << std::endl;
        return create_default_field_layout(tag_size_m);
    }

    return layout;
}

FieldLayout create_default_field_layout(double tag_size_m) {
    FieldLayout layout;
    layout.name = "FRC 2026 Default";
    layout.tag_size_m = tag_size_m;

    auto corners_local = get_tag_corners_local(tag_size_m);

    // FRC 2026 field dimensions (approximate - update with actual values)
    // Field is 16.54m x 8.21m (54'3.25" x 26'11.25")
    constexpr double FIELD_LENGTH = 16.54;
    constexpr double FIELD_WIDTH = 8.21;

    // Speaker tags (Red Alliance - IDs 1-4)
    // These are on the scoring structure
    std::vector<std::tuple<int, double, double, double, double>> tag_positions = {
        // ID, X, Y, Z, Yaw (degrees, 0 = facing +X)
        // Red Alliance (right side when standing at red driver station)
        {1,  15.08, 0.98, 1.45, 180},  // Red source right
        {2,  15.08, 2.66, 1.45, 180},  // Red source left
        {3,  16.54, 4.10, 1.45, 0},    // Red speaker center
        {4,  16.54, 5.55, 1.45, 0},    // Red speaker side

        // Blue Alliance (left side when standing at blue driver station)
        {5,  1.46, 0.98, 1.45, 0},     // Blue source right
        {6,  1.46, 2.66, 1.45, 0},     // Blue source left
        {7,  0.0, 4.10, 1.45, 180},    // Blue speaker center
        {8,  0.0, 5.55, 1.45, 180},    // Blue speaker side

        // Stage tags (center field)
        {9,  4.64, 4.50, 1.32, 120},   // Blue stage left
        {10, 4.64, 3.71, 1.32, 60},    // Blue stage right
        {11, 5.32, 4.10, 1.32, 0},     // Blue stage center

        {12, 11.90, 4.50, 1.32, 60},   // Red stage left
        {13, 11.90, 3.71, 1.32, 120},  // Red stage right
        {14, 11.22, 4.10, 1.32, 180},  // Red stage center

        // Amp tags
        {15, 0.0, 7.62, 1.30, 90},     // Blue amp
        {16, 16.54, 7.62, 1.30, 270},  // Red amp
    };

    for (const auto& [id, x, y, z, yaw_deg] : tag_positions) {
        FieldTag tag;
        tag.id = id;
        tag.center_field = {x, y, z};

        // Convert yaw to quaternion (rotation around Z axis)
        double yaw = yaw_deg * M_PI / 180.0;
        tag.pose_field.position = tag.center_field;
        tag.pose_field.orientation.w = std::cos(yaw / 2.0);
        tag.pose_field.orientation.x = 0;
        tag.pose_field.orientation.y = 0;
        tag.pose_field.orientation.z = std::sin(yaw / 2.0);

        // Transform corners to field frame
        cv::Mat R;
        cv::Rodrigues(tag.pose_field.rvec(), R);

        for (int i = 0; i < 4; i++) {
            cv::Mat p_local = (cv::Mat_<double>(3, 1) <<
                corners_local[i].x, corners_local[i].y, corners_local[i].z);
            cv::Mat p_field = R * p_local;

            tag.corners_field[i].x = p_field.at<double>(0) + tag.center_field.x;
            tag.corners_field[i].y = p_field.at<double>(1) + tag.center_field.y;
            tag.corners_field[i].z = p_field.at<double>(2) + tag.center_field.z;
        }

        layout.tags[tag.id] = tag;
    }

    std::cout << "[FieldLayout] Created default layout with "
              << layout.tags.size() << " tags" << std::endl;

    return layout;
}

} // namespace frc_vision
