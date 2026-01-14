#pragma once
/**
 * @file field_layout.hpp
 * @brief Field tag layout loading and management
 */

#include "types.hpp"
#include <string>

namespace frc_vision {

/**
 * @brief Load field layout from JSON file
 * @param path Path to field_layout.json
 * @param tag_size_m Size of tags in meters
 * @return FieldLayout on success
 */
FieldLayout load_field_layout(const std::string& path, double tag_size_m);

/**
 * @brief Create a default FRC 2026 field layout
 *
 * Contains approximate positions for common AprilTag placements.
 * @param tag_size_m Size of tags in meters (default 6.5" = 0.1651m)
 */
FieldLayout create_default_field_layout(double tag_size_m = 0.1651);

/**
 * @brief Get tag corner positions in tag-local frame
 *
 * Returns corners in counter-clockwise order starting from bottom-left,
 * centered at the tag origin.
 * @param tag_size_m Size of tag in meters
 */
std::array<Point3D, 4> get_tag_corners_local(double tag_size_m);

} // namespace frc_vision
