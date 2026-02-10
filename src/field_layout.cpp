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
    layout.name = "FRC 2026 REBUILT (Welded)";
    layout.tag_size_m = tag_size_m;

    auto corners_local = get_tag_corners_local(tag_size_m);

    // FRC 2026 REBUILT field - 32 tags total
    // Welded field variant: 16.541m x 8.069m
    // Quaternion format: (W, X, Y, Z) for tag facing direction
    struct TagDef {
        int id;
        double x, y, z;
        double qw, qx, qy, qz;
    };

    std::vector<TagDef> tag_defs = {
        // Red trench (8 trench tags total, 4 per alliance)
        { 1, 11.8780, 7.4248, 0.889,  0.0, 0.0, 0.0, 1.0},
        { 6, 11.8780, 0.6445, 0.889,  0.0, 0.0, 0.0, 1.0},
        { 7, 11.9529, 0.6445, 0.889,  1.0, 0.0, 0.0, 0.0},
        {12, 11.9529, 7.4248, 0.889,  1.0, 0.0, 0.0, 0.0},

        // Red hub (16 hub tags total, 8 per alliance, 2 per face)
        { 2, 11.9154, 4.6380, 1.124,  0.7071068, 0.0, 0.0, 0.7071068},
        { 3, 11.3119, 4.3902, 1.124,  0.0, 0.0, 0.0, 1.0},
        { 4, 11.3119, 4.0346, 1.124,  0.0, 0.0, 0.0, 1.0},
        { 5, 11.9154, 3.4312, 1.124, -0.7071068, 0.0, 0.0, 0.7071068},
        { 8, 12.2710, 3.4312, 1.124, -0.7071068, 0.0, 0.0, 0.7071068},
        { 9, 12.5192, 3.6790, 1.124,  1.0, 0.0, 0.0, 0.0},
        {10, 12.5192, 4.0346, 1.124,  1.0, 0.0, 0.0, 0.0},
        {11, 12.2710, 4.6380, 1.124,  0.7071068, 0.0, 0.0, 0.7071068},

        // Red outpost & tower wall (8 wall tags total, 4 per alliance)
        {13, 16.5333, 7.4033, 0.552,  0.0, 0.0, 0.0, 1.0},
        {14, 16.5333, 6.9715, 0.552,  0.0, 0.0, 0.0, 1.0},
        {15, 16.5330, 4.3236, 0.552,  0.0, 0.0, 0.0, 1.0},
        {16, 16.5330, 3.8918, 0.552,  0.0, 0.0, 0.0, 1.0},

        // Blue trench
        {17,  4.6631, 0.6445, 0.889,  1.0, 0.0, 0.0, 0.0},
        {22,  4.6631, 7.4248, 0.889,  1.0, 0.0, 0.0, 0.0},
        {23,  4.5882, 7.4248, 0.889,  0.0, 0.0, 0.0, 1.0},
        {28,  4.5882, 0.6445, 0.889,  0.0, 0.0, 0.0, 1.0},

        // Blue hub
        {18,  4.6256, 3.4312, 1.124, -0.7071068, 0.0, 0.0, 0.7071068},
        {19,  5.2292, 3.6790, 1.124,  1.0, 0.0, 0.0, 0.0},
        {20,  5.2292, 4.0346, 1.124,  1.0, 0.0, 0.0, 0.0},
        {21,  4.6256, 4.6380, 1.124,  0.7071068, 0.0, 0.0, 0.7071068},
        {24,  4.2700, 4.6380, 1.124,  0.7071068, 0.0, 0.0, 0.7071068},
        {25,  4.0219, 4.3902, 1.124,  0.0, 0.0, 0.0, 1.0},
        {26,  4.0219, 4.0346, 1.124,  0.0, 0.0, 0.0, 1.0},
        {27,  4.2700, 3.4312, 1.124, -0.7071068, 0.0, 0.0, 0.7071068},

        // Blue outpost & tower wall
        {29,  0.0077, 0.6660, 0.552,  1.0, 0.0, 0.0, 0.0},
        {30,  0.0077, 1.0978, 0.552,  1.0, 0.0, 0.0, 0.0},
        {31,  0.0081, 3.7457, 0.552,  1.0, 0.0, 0.0, 0.0},
        {32,  0.0081, 4.1775, 0.552,  1.0, 0.0, 0.0, 0.0},
    };

    for (const auto& def : tag_defs) {
        FieldTag tag;
        tag.id = def.id;
        tag.center_field = {def.x, def.y, def.z};

        tag.pose_field.position = tag.center_field;
        tag.pose_field.orientation.w = def.qw;
        tag.pose_field.orientation.x = def.qx;
        tag.pose_field.orientation.y = def.qy;
        tag.pose_field.orientation.z = def.qz;

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

    std::cout << "[FieldLayout] Created FRC 2026 REBUILT default layout with "
              << layout.tags.size() << " tags" << std::endl;

    return layout;
}

} // namespace frc_vision
