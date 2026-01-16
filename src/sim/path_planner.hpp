#pragma once
/**
 * @file path_planner.hpp
 * @brief Production-grade Auto Coder for FRC 2026
 *
 * Features:
 * - Path definition with waypoints and event markers
 * - Bezier curve trajectory generation
 * - Tag-triggered actions with GUI menus
 * - WPILib 2026 Java/C++/Python code generation
 * - Subsystem and command scaffolding generation
 * - Alliance mirroring support
 * - Heading profiles (interpolate, hold, face target)
 *
 * Code generation is compliant with WPILib 2026 command-based structure.
 */

#include "sim_types.hpp"
#include "game_scoring.hpp"
#include "../types.hpp"
#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <map>
#include <set>

namespace frc_vision {
namespace sim {

/**
 * @brief Waypoint rotation mode / heading profile
 */
enum class RotationMode {
    CONSTANT,       // Keep constant rotation throughout segment
    LINEAR,         // Linear interpolation start->end heading over time
    HOLONOMIC,      // Rotate independently of translation
    HOLD_HEADING,   // Hold heading at start value
    FACE_TARGET     // Face a specific target point
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
    AT_WAYPOINT,        // When reaching a waypoint (by index)
    AT_TIME,            // At specific time in trajectory
    TAG_VISIBLE,        // When specific tag becomes visible
    DISTANCE_TO_TAG,    // When within distance of tag
    TAG_ALIGNED,        // When aligned with tag (facing it)
    GAME_PIECE_DETECTED,// When intake detects game piece
    CUSTOM              // Custom condition function
};

/**
 * @brief Available robot actions for auto builder
 */
enum class RobotAction {
    // Intake actions
    INTAKE_IN,
    INTAKE_OUT,
    INTAKE_STOP,

    // Shooter actions
    SPIN_UP_SHOOTER,
    SHOOT,
    SHOOTER_STOP,

    // Elevator/arm actions
    SET_ELEVATOR_HEIGHT,
    STOW,
    EXTEND_ARM,
    RETRACT_ARM,

    // Vision/alignment actions
    AUTO_ALIGN_TO_TAG,
    FACE_TAG,
    TRACK_TAG,

    // Climbing
    DEPLOY_CLIMBER,
    CLIMB,

    // Generic
    WAIT,
    CUSTOM_COMMAND
};

/**
 * @brief Get human-readable name for action
 */
inline std::string action_to_string(RobotAction action) {
    switch (action) {
        case RobotAction::INTAKE_IN: return "Intake In";
        case RobotAction::INTAKE_OUT: return "Intake Out";
        case RobotAction::INTAKE_STOP: return "Intake Stop";
        case RobotAction::SPIN_UP_SHOOTER: return "Spin Up Shooter";
        case RobotAction::SHOOT: return "Shoot";
        case RobotAction::SHOOTER_STOP: return "Shooter Stop";
        case RobotAction::SET_ELEVATOR_HEIGHT: return "Set Elevator Height";
        case RobotAction::STOW: return "Stow";
        case RobotAction::EXTEND_ARM: return "Extend Arm";
        case RobotAction::RETRACT_ARM: return "Retract Arm";
        case RobotAction::AUTO_ALIGN_TO_TAG: return "Auto-Align to Tag";
        case RobotAction::FACE_TAG: return "Face Tag";
        case RobotAction::TRACK_TAG: return "Track Tag";
        case RobotAction::DEPLOY_CLIMBER: return "Deploy Climber";
        case RobotAction::CLIMB: return "Climb";
        case RobotAction::WAIT: return "Wait";
        case RobotAction::CUSTOM_COMMAND: return "Custom Command";
        default: return "Unknown";
    }
}

/**
 * @brief Get WPILib command class name for action
 */
inline std::string action_to_command_name(RobotAction action) {
    switch (action) {
        case RobotAction::INTAKE_IN: return "IntakeIn";
        case RobotAction::INTAKE_OUT: return "IntakeOut";
        case RobotAction::INTAKE_STOP: return "IntakeStop";
        case RobotAction::SPIN_UP_SHOOTER: return "SpinUpShooter";
        case RobotAction::SHOOT: return "Shoot";
        case RobotAction::SHOOTER_STOP: return "ShooterStop";
        case RobotAction::SET_ELEVATOR_HEIGHT: return "SetElevatorHeight";
        case RobotAction::STOW: return "Stow";
        case RobotAction::EXTEND_ARM: return "ExtendArm";
        case RobotAction::RETRACT_ARM: return "RetractArm";
        case RobotAction::AUTO_ALIGN_TO_TAG: return "AutoAlignToTarget";
        case RobotAction::FACE_TAG: return "FaceTag";
        case RobotAction::TRACK_TAG: return "TrackTag";
        case RobotAction::DEPLOY_CLIMBER: return "DeployClimber";
        case RobotAction::CLIMB: return "Climb";
        case RobotAction::WAIT: return "WaitCommand";
        case RobotAction::CUSTOM_COMMAND: return "CustomCommand";
        default: return "UnknownCommand";
    }
}

/**
 * @brief Get required subsystem for action
 */
inline std::string action_to_subsystem(RobotAction action) {
    switch (action) {
        case RobotAction::INTAKE_IN:
        case RobotAction::INTAKE_OUT:
        case RobotAction::INTAKE_STOP:
            return "IntakeSubsystem";
        case RobotAction::SPIN_UP_SHOOTER:
        case RobotAction::SHOOT:
        case RobotAction::SHOOTER_STOP:
            return "ShooterSubsystem";
        case RobotAction::SET_ELEVATOR_HEIGHT:
        case RobotAction::STOW:
        case RobotAction::EXTEND_ARM:
        case RobotAction::RETRACT_ARM:
            return "ElevatorSubsystem";
        case RobotAction::AUTO_ALIGN_TO_TAG:
        case RobotAction::FACE_TAG:
        case RobotAction::TRACK_TAG:
            return "VisionSubsystem";
        case RobotAction::DEPLOY_CLIMBER:
        case RobotAction::CLIMB:
            return "ClimberSubsystem";
        default:
            return "";
    }
}

/**
 * @brief Tag action binding (what to do when seeing/clicking a tag)
 */
struct TagActionBinding {
    int tag_id;
    RobotAction action;
    std::map<std::string, double> parameters;
    double trigger_distance = 2.0;  // meters
    bool auto_trigger = false;      // Trigger when within distance during auto
    std::string custom_command;     // For CUSTOM_COMMAND action
};

/**
 * @brief Event marker for path following
 */
struct EventMarker {
    std::string name;
    TriggerCondition trigger;

    // Trigger parameters (use appropriate one based on trigger type)
    int waypoint_index = -1;    // For AT_WAYPOINT
    double time = 0;            // For AT_TIME
    int tag_id = -1;            // For tag-based triggers
    double distance = 0;        // For DISTANCE_TO_TAG

    // Action to execute
    RobotAction action;
    std::map<std::string, double> parameters;
    std::string custom_command;

    // Execution options
    bool wait_for_completion = false;
    double timeout = 5.0;
    bool parallel = false;      // Run parallel with path following
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
 * @brief Complete autonomous path/routine
 */
struct AutoPath {
    std::string name;
    std::string description;

    // Waypoints
    std::vector<PathWaypoint> waypoints;

    // Event markers (actions during path following)
    std::vector<EventMarker> event_markers;

    // Tag action bindings (actions when seeing specific tags)
    std::vector<TagActionBinding> tag_bindings;

    // Legacy actions (for backward compatibility)
    std::vector<PathAction> actions;

    // Global constraints
    double max_velocity = 4.0;          // m/s
    double max_acceleration = 3.0;      // m/s^2
    double max_angular_velocity = 4.0;  // rad/s
    double max_angular_accel = 6.0;     // rad/s^2

    // Heading profile
    RotationMode heading_mode = RotationMode::LINEAR;
    Pose2D face_target;                 // For FACE_TARGET mode

    // Constraint zones
    std::vector<ConstraintZone> constraint_zones;

    // Path following settings
    bool reverse = false;
    bool reset_odometry = true;
    bool stop_at_end = true;            // Stop drivetrain at end
    bool mirror_for_red = true;         // Auto-mirror for red alliance

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

    // Track which subsystems/commands are needed
    std::set<std::string> required_subsystems;
    std::set<std::string> required_commands;
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
 * @brief Generated file info for code output
 */
struct GeneratedFile {
    std::string path;           // e.g., "frc/robot/auto/UserPath.java"
    std::string content;        // File contents
    std::string description;    // What this file is for
};

/**
 * @brief Complete code generation result
 */
struct CodeGenerationResult {
    std::vector<GeneratedFile> files;
    std::vector<std::string> warnings;
    std::vector<std::string> integration_notes;
    bool success = true;
};

/**
 * @brief Production-grade Auto Coder for FRC 2026
 *
 * Generates:
 * - Path trajectory classes
 * - Auto routine command groups
 * - Event map for markers
 * - Subsystem scaffolds (if needed)
 * - Command scaffolds (if needed)
 * - Field constants with mirroring
 * - Drivetrain interface (if needed)
 */
class PathPlanner {
public:
    PathPlanner();

    // ========================================================================
    // Configuration
    // ========================================================================

    /**
     * @brief Set swerve drive configuration
     */
    void set_swerve_config(const SwerveConfig& config);

    /**
     * @brief Set field layout for tag-based actions
     */
    void set_field_layout(const FieldLayout& field);

    /**
     * @brief Set alliance for mirroring
     */
    void set_alliance(Alliance alliance) { alliance_ = alliance; }

    // ========================================================================
    // Path Creation and Editing
    // ========================================================================

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
     * @brief Add event marker to path
     */
    void add_event_marker(AutoPath& path, const EventMarker& marker);

    /**
     * @brief Add tag action binding to path
     */
    void add_tag_binding(AutoPath& path, const TagActionBinding& binding);

    /**
     * @brief Add action to path (legacy support)
     */
    void add_action(AutoPath& path, const PathAction& action);

    // ========================================================================
    // Trajectory Generation
    // ========================================================================

    /**
     * @brief Generate trajectory for path (fills trajectory vector)
     * @param path Path to generate trajectory for
     * @param dt Time step for trajectory points (default 0.02s = 50Hz)
     */
    void generate_trajectory(AutoPath& path, double dt = 0.02);

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

    /**
     * @brief Mirror pose for red alliance
     */
    Pose2D mirror_pose(const Pose2D& pose) const;

    // ========================================================================
    // Code Generation
    // ========================================================================

    /**
     * @brief Generate all code for a path (main entry point)
     * @param path Path to generate code for
     * @param format Output code format
     * @param generate_scaffolds If true, generate missing subsystems/commands
     * @return Complete generation result with all files
     */
    CodeGenerationResult generate_all_code(
        const AutoPath& path,
        CodeFormat format,
        bool generate_scaffolds = true) const;

    /**
     * @brief Generate code for path (simple string output - legacy)
     */
    std::string generate_code(const AutoPath& path, CodeFormat format) const;

    /**
     * @brief Generate complete autonomous command group
     */
    std::string generate_auto_routine(const std::vector<std::string>& paths, CodeFormat format) const;

    // ========================================================================
    // Subsystem and Command Scaffolding
    // ========================================================================

    /**
     * @brief Generate subsystem scaffold
     */
    GeneratedFile generate_subsystem_scaffold(const std::string& name, CodeFormat format) const;

    /**
     * @brief Generate command scaffold
     */
    GeneratedFile generate_command_scaffold(
        const std::string& name,
        const std::string& subsystem,
        CodeFormat format) const;

    /**
     * @brief Generate field constants with mirroring helpers
     */
    GeneratedFile generate_field_constants(CodeFormat format) const;

    /**
     * @brief Generate drivetrain interface (if real drivetrain unknown)
     */
    GeneratedFile generate_drivetrain_interface(CodeFormat format) const;

    /**
     * @brief Generate event map for markers
     */
    GeneratedFile generate_event_map(const AutoPath& path, CodeFormat format) const;

    // ========================================================================
    // Path I/O
    // ========================================================================

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

    // ========================================================================
    // Available Actions for GUI
    // ========================================================================

    /**
     * @brief Get list of available actions for GUI menu
     */
    static std::vector<RobotAction> get_available_actions();

    /**
     * @brief Get actions relevant for a specific tag
     */
    std::vector<RobotAction> get_actions_for_tag(int tag_id) const;

private:
    // Trajectory generation helpers
    void compute_bezier_points(AutoPath& path);
    double calculate_path_length(const AutoPath& path) const;
    void apply_constraints(AutoPath& path);
    void generate_time_profile(AutoPath& path);
    void analyze_required_code(AutoPath& path) const;

    // Code generation helpers - Java
    std::string generate_java_path(const AutoPath& path) const;
    std::string generate_java_trajectory_class(const AutoPath& path) const;
    std::string generate_java_auto_routine(const AutoPath& path) const;
    std::string generate_java_event_marker(const EventMarker& marker) const;
    std::string generate_java_action(const PathAction& action) const;

    // Code generation helpers - C++
    std::string generate_cpp_path(const AutoPath& path) const;
    std::string generate_cpp_action(const PathAction& action) const;

    // Code generation helpers - Python
    std::string generate_python_path(const AutoPath& path) const;
    std::string generate_python_action(const PathAction& action) const;

    // Code generation helpers - PathPlanner JSON
    std::string generate_pathplanner_json(const AutoPath& path) const;

    // Bezier curve math
    Pose2D bezier_point(const PathWaypoint& p0, const PathWaypoint& p1, double t) const;
    double bezier_curvature(const PathWaypoint& p0, const PathWaypoint& p1, double t) const;

    std::map<std::string, AutoPath> paths_;
    SwerveConfig swerve_config_;
    FieldLayout field_;
    Alliance alliance_ = Alliance::BLUE;

    // Field dimensions for mirroring
    static constexpr double FIELD_LENGTH = 16.54;  // meters
    static constexpr double FIELD_WIDTH = 8.21;    // meters
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
