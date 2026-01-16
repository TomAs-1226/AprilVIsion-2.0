#pragma once
/**
 * @file path_planner.hpp
 * @brief PathPlanner-style autonomous path planning and code generation
 *
 * This module provides:
 * - Path definition with waypoints
 * - Bezier curve trajectory generation
 * - Action triggers at waypoints and tags
 * - Code generation for WPILib swerve drive systems
 *
 * Compatible with standard FRC swerve drive implementations.
 */

#include "sim_types.hpp"
#include "game_scoring.hpp"
#include "../types.hpp"
#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <map>

namespace frc_vision {
namespace sim {

/**
 * @brief Waypoint rotation mode
 */
enum class RotationMode {
    CONSTANT,       // Keep constant rotation throughout segment
    LINEAR,         // Linear interpolation to next waypoint rotation
    HOLONOMIC       // Rotate independently of translation
};

/**
 * @brief Path constraint type
 */
enum class ConstraintType {
    MAX_VELOCITY,
    MAX_ACCELERATION,
    MAX_ANGULAR_VELOCITY,
    ZONE  // Constrain within a zone
};

/**
 * @brief Action trigger condition
 */
enum class TriggerCondition {
    AT_WAYPOINT,        // When reaching a waypoint
    TAG_VISIBLE,        // When specific tag becomes visible
    DISTANCE_TO_TAG,    // When within distance of tag
    TIME_ELAPSED,       // After time from path start
    CUSTOM              // Custom condition function
};

/**
 * @brief Single waypoint in a path
 */
struct PathWaypoint {
    Pose2D pose;                    // Position and heading
    double velocity = 0;            // Desired velocity at waypoint (m/s), 0 = stop
    RotationMode rotation_mode = RotationMode::HOLONOMIC;

    // Bezier control point offsets (relative to waypoint)
    double control_in_x = 0;        // Entry control point
    double control_in_y = 0;
    double control_out_x = 0;       // Exit control point
    double control_out_y = 0;

    // Constraints
    double max_velocity = -1;       // -1 = use path default
    double max_accel = -1;
    double max_angular_vel = -1;

    std::string name = "";          // Optional name for this waypoint
};

/**
 * @brief Action to execute during path following
 */
struct PathAction {
    std::string name;               // Human-readable name
    std::string command_name;       // Code-friendly command name
    TriggerCondition trigger;

    // Trigger parameters
    int waypoint_index = -1;        // For AT_WAYPOINT
    int tag_id = -1;                // For TAG_VISIBLE, DISTANCE_TO_TAG
    double distance = 0;            // For DISTANCE_TO_TAG
    double time = 0;                // For TIME_ELAPSED

    // Action parameters
    std::map<std::string, double> parameters;
    bool wait_for_completion = false;
    double timeout = 5.0;           // Max time to wait

    // Code generation
    std::string wpilib_command = "";  // Generated WPILib command
};

/**
 * @brief Path constraint zone
 */
struct ConstraintZone {
    std::string name;
    double min_x, max_x;
    double min_y, max_y;
    ConstraintType constraint_type;
    double value;
};

/**
 * @brief Complete autonomous path
 */
struct AutoPath {
    std::string name;
    std::string description;

    // Waypoints
    std::vector<PathWaypoint> waypoints;

    // Actions
    std::vector<PathAction> actions;

    // Global constraints
    double max_velocity = 4.0;          // m/s
    double max_acceleration = 3.0;      // m/s^2
    double max_angular_velocity = 4.0;  // rad/s
    double max_angular_accel = 6.0;     // rad/s^2

    // Constraint zones
    std::vector<ConstraintZone> constraint_zones;

    // Path following settings
    bool reverse = false;
    bool reset_odometry = true;

    // Generated trajectory (computed by PathPlanner)
    struct TrajectoryPoint {
        double time;
        Pose2D pose;
        double velocity;
        double acceleration;
        double angular_velocity;
        double curvature;
    };
    std::vector<TrajectoryPoint> trajectory;

    double total_time = 0;
    double total_distance = 0;
};

/**
 * @brief Swerve drive configuration for code generation
 */
struct SwerveConfig {
    // Physical parameters
    double track_width = 0.5969;        // meters (23.5 inches)
    double wheel_base = 0.5969;         // meters
    double wheel_diameter = 0.1016;     // meters (4 inches)
    double drive_gear_ratio = 6.75;     // L2 ratio
    double steer_gear_ratio = 12.8;

    // Motor types
    std::string drive_motor = "NEO";    // NEO, Falcon500, Kraken
    std::string steer_motor = "NEO550";

    // CAN IDs (front-left, front-right, back-left, back-right)
    std::array<int, 4> drive_ids = {1, 2, 3, 4};
    std::array<int, 4> steer_ids = {5, 6, 7, 8};
    std::array<int, 4> encoder_ids = {9, 10, 11, 12};

    // Encoder offsets (radians)
    std::array<double, 4> encoder_offsets = {0, 0, 0, 0};

    // PID gains
    double drive_kP = 0.1;
    double drive_kI = 0.0;
    double drive_kD = 0.0;
    double drive_kS = 0.0;
    double drive_kV = 2.0;
    double drive_kA = 0.0;

    double steer_kP = 5.0;
    double steer_kI = 0.0;
    double steer_kD = 0.0;

    // Path following PID
    double translation_kP = 5.0;
    double translation_kI = 0.0;
    double translation_kD = 0.0;

    double rotation_kP = 5.0;
    double rotation_kI = 0.0;
    double rotation_kD = 0.0;
};

/**
 * @brief Code generation output format
 */
enum class CodeFormat {
    WPILIB_JAVA,
    WPILIB_CPP,
    WPILIB_PYTHON,
    PATHPLANNER_JSON
};

/**
 * @brief PathPlanner-style autonomous path planner and code generator
 */
class PathPlanner {
public:
    PathPlanner();

    /**
     * @brief Set swerve drive configuration
     */
    void set_swerve_config(const SwerveConfig& config);

    /**
     * @brief Set field layout for tag-based actions
     */
    void set_field_layout(const FieldLayout& field);

    /**
     * @brief Create a new path
     */
    AutoPath& create_path(const std::string& name);

    /**
     * @brief Get path by name
     */
    AutoPath* get_path(const std::string& name);

    /**
     * @brief Add waypoint to path
     */
    void add_waypoint(AutoPath& path, const PathWaypoint& waypoint);

    /**
     * @brief Add action to path
     */
    void add_action(AutoPath& path, const PathAction& action);

    /**
     * @brief Generate trajectory for path (fills trajectory vector)
     * @param path Path to generate trajectory for
     * @param dt Time step for trajectory points (default 0.02s = 50Hz)
     */
    void generate_trajectory(AutoPath& path, double dt = 0.02);

    /**
     * @brief Generate code for path
     * @param path Path to generate code for
     * @param format Output code format
     * @return Generated code as string
     */
    std::string generate_code(const AutoPath& path, CodeFormat format) const;

    /**
     * @brief Generate complete autonomous command group
     * @param paths List of path names to include
     * @param format Output code format
     * @return Generated code as string
     */
    std::string generate_auto_routine(const std::vector<std::string>& paths, CodeFormat format) const;

    /**
     * @brief Save path to JSON file
     */
    bool save_path(const AutoPath& path, const std::string& filename) const;

    /**
     * @brief Load path from JSON file
     */
    bool load_path(const std::string& filename, AutoPath& path);

    /**
     * @brief Get all paths
     */
    const std::map<std::string, AutoPath>& get_paths() const { return paths_; }

    /**
     * @brief Sample point on path at given time
     */
    Pose2D sample_path(const AutoPath& path, double t) const;

    /**
     * @brief Get heading at time t on path
     */
    double sample_heading(const AutoPath& path, double t) const;

    /**
     * @brief Get velocity at time t on path
     */
    double sample_velocity(const AutoPath& path, double t) const;

private:
    // Trajectory generation helpers
    void compute_bezier_points(AutoPath& path);
    double calculate_path_length(const AutoPath& path) const;
    void apply_constraints(AutoPath& path);
    void generate_time_profile(AutoPath& path);

    // Code generation helpers
    std::string generate_java_path(const AutoPath& path) const;
    std::string generate_cpp_path(const AutoPath& path) const;
    std::string generate_python_path(const AutoPath& path) const;
    std::string generate_pathplanner_json(const AutoPath& path) const;

    std::string generate_java_action(const PathAction& action) const;
    std::string generate_cpp_action(const PathAction& action) const;
    std::string generate_python_action(const PathAction& action) const;

    // Bezier curve math
    Pose2D bezier_point(const PathWaypoint& p0, const PathWaypoint& p1, double t) const;
    double bezier_curvature(const PathWaypoint& p0, const PathWaypoint& p1, double t) const;

    std::map<std::string, AutoPath> paths_;
    SwerveConfig swerve_config_;
    FieldLayout field_;
};

// ============================================================================
// Preset path generators
// ============================================================================

/**
 * @brief Generate a path to align with and score at a specific tag
 */
AutoPath generate_score_path(
    const Pose2D& start,
    int target_tag_id,
    const std::string& score_action,
    const FieldLayout& field,
    double standoff_distance = 1.0);

/**
 * @brief Generate a path for autonomous game piece pickup and scoring
 */
AutoPath generate_pickup_score_path(
    const Pose2D& start,
    const Pose2D& pickup_location,
    int score_tag_id,
    const std::string& score_action,
    const FieldLayout& field);

/**
 * @brief Generate multi-piece autonomous routine
 */
std::vector<AutoPath> generate_multi_piece_auto(
    GameYear game,
    Alliance alliance,
    int num_pieces,
    const FieldLayout& field);

} // namespace sim
} // namespace frc_vision
