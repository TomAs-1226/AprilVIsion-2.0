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
 * @brief Action trigger condition type
 */
enum class TriggerCondition {
    AT_WAYPOINT,        // When reaching a waypoint (by index)
    AT_TIME,            // At specific time in trajectory
    TAG_VISIBLE,        // When specific tag becomes visible
    DISTANCE_TO_TAG,    // When within distance of tag
    TAG_ALIGNED,        // When aligned with tag (facing it)
    GAME_PIECE_DETECTED,// When intake detects game piece
    VELOCITY_BELOW,     // When robot velocity is below threshold
    CUSTOM              // Custom condition function
};

/**
 * @brief Logic operator for combining conditions
 */
enum class ConditionLogic {
    AND,    // All conditions must be true
    OR      // Any condition must be true
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
 * @brief Single condition in a block (for visual block coding style)
 *
 * Represents one condition that can be combined with others.
 * Example conditions:
 * - "At waypoint 2"
 * - "See tag 5"
 * - "Within 1.5m of tag 7"
 */
struct Condition {
    TriggerCondition type;

    // Parameters (use based on type)
    int waypoint_index = -1;    // For AT_WAYPOINT
    double time = 0;            // For AT_TIME
    int tag_id = -1;            // For tag-based triggers
    double distance = 0;        // For DISTANCE_TO_TAG
    double threshold = 0;       // For VELOCITY_BELOW
    std::string custom_check;   // For CUSTOM

    // Comparison operator for DISTANCE_TO_TAG
    enum class Compare { LESS_THAN, GREATER_THAN, EQUAL };
    Compare compare = Compare::LESS_THAN;

    // State tracking (for edge detection - trigger once)
    mutable bool was_true = false;
    bool trigger_on_edge = true;  // Only trigger on false->true transition

    /**
     * @brief Get human-readable description of this condition
     */
    std::string describe() const {
        switch (type) {
            case TriggerCondition::AT_WAYPOINT:
                return "At Waypoint " + std::to_string(waypoint_index);
            case TriggerCondition::AT_TIME:
                return "At Time " + std::to_string(time) + "s";
            case TriggerCondition::TAG_VISIBLE:
                return "Tag " + std::to_string(tag_id) + " Visible";
            case TriggerCondition::DISTANCE_TO_TAG:
                return "Within " + std::to_string(distance) + "m of Tag " + std::to_string(tag_id);
            case TriggerCondition::TAG_ALIGNED:
                return "Aligned with Tag " + std::to_string(tag_id);
            case TriggerCondition::VELOCITY_BELOW:
                return "Speed < " + std::to_string(threshold) + " m/s";
            default:
                return "Custom Condition";
        }
    }
};

/**
 * @brief Action block with combined conditions (visual block coding style)
 *
 * This represents a complete "block" in the visual auto builder:
 * - Multiple conditions combined with AND/OR logic
 * - One or more actions to execute when conditions are met
 * - Execution options (parallel, wait, timeout)
 *
 * Example block:
 *   WHEN: [At Waypoint 2] AND [Tag 5 Visible] AND [Distance < 1.5m]
 *   DO:   [Spin Up Shooter] then [Shoot]
 */
struct ActionBlock {
    std::string name;
    std::string description;

    // Conditions (combined with logic operator)
    std::vector<Condition> conditions;
    ConditionLogic logic = ConditionLogic::AND;

    // Actions to execute (in sequence)
    struct ActionStep {
        RobotAction action;
        std::map<std::string, double> parameters;
        std::string custom_command;
        double delay_before = 0;    // Wait before executing
        bool wait_for_completion = false;
    };
    std::vector<ActionStep> actions;

    // Execution options
    bool enabled = true;
    bool one_shot = true;           // Only trigger once per path execution
    bool parallel_with_path = true; // Run alongside path following
    double timeout = 10.0;          // Max time to wait for conditions

    // Runtime state
    mutable bool has_triggered = false;
    mutable double trigger_time = -1;

    /**
     * @brief Reset runtime state (call at start of path execution)
     */
    void reset() const {
        has_triggered = false;
        trigger_time = -1;
        for (auto& cond : conditions) {
            cond.was_true = false;
        }
    }

    /**
     * @brief Get block description for GUI
     */
    std::string describe() const {
        std::string desc = "WHEN: ";
        for (size_t i = 0; i < conditions.size(); i++) {
            if (i > 0) desc += (logic == ConditionLogic::AND ? " AND " : " OR ");
            desc += "[" + conditions[i].describe() + "]";
        }
        desc += "\nDO: ";
        for (size_t i = 0; i < actions.size(); i++) {
            if (i > 0) desc += " → ";
            desc += "[" + action_to_string(actions[i].action) + "]";
        }
        return desc;
    }
};

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

    // Action blocks (visual block coding style - combines conditions + actions)
    std::vector<ActionBlock> action_blocks;

    // Event markers (actions during path following) - legacy, use action_blocks
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

// ============================================================================
// Subsystem Configuration System
// ============================================================================

/**
 * @brief Type of robot subsystem
 */
enum class SubsystemType {
    SHOOTER,        // Fixed shooter (can pre-spin, fire single/multiple)
    INTAKE,         // Intake mechanism (in/out/stop, detect game piece)
    ELEVATOR,       // Elevator/lift (set height, stow)
    ARM,            // Arm mechanism (extend/retract, set angle)
    CLIMBER,        // Climbing mechanism (deploy, climb)
    TURRET,         // Rotating turret (track target, set angle) - if applicable
    CUSTOM          // User-defined subsystem
};

/**
 * @brief Single action for a subsystem
 */
struct SubsystemAction {
    std::string name;           // e.g., "fire", "spinUp", "intake"
    std::string description;    // Human-readable description
    RobotAction action_type;    // Maps to RobotAction enum

    // Parameters this action accepts
    struct Parameter {
        std::string name;
        std::string type;       // "double", "int", "bool"
        double default_value = 0;
        double min_value = 0;
        double max_value = 100;
        std::string unit;       // e.g., "m", "deg", "rpm", "ms"
    };
    std::vector<Parameter> parameters;

    // Timing
    double typical_duration = 0.5;  // seconds
    bool is_instant = false;        // Completes immediately
    bool can_be_interrupted = true;

    // Requirements
    std::vector<std::string> required_sensors;
    std::string required_state;     // e.g., "elevated" for shooter
};

/**
 * @brief Configuration for a robot subsystem
 */
struct SubsystemConfig {
    std::string name;               // e.g., "ShooterSubsystem"
    SubsystemType type;
    std::string description;

    // Available actions for this subsystem
    std::vector<SubsystemAction> actions;

    // Hardware configuration
    struct Motor {
        std::string name;
        int can_id = 0;
        std::string type = "NEO";   // NEO, NEO550, Falcon, etc.
        bool inverted = false;
        double gear_ratio = 1.0;
    };
    std::vector<Motor> motors;

    // Sensors
    struct Sensor {
        std::string name;
        std::string type;           // "encoder", "limit_switch", "beam_break", "color"
        int channel = 0;
    };
    std::vector<Sensor> sensors;

    // Physical properties
    double max_velocity = 0;        // units depend on type (rpm, m/s, deg/s)
    double max_acceleration = 0;
    double position_tolerance = 0.1;

    // State machine (simplified)
    std::vector<std::string> states; // e.g., "idle", "spinning", "ready", "firing"
    std::string default_state = "idle";
};

/**
 * @brief Runtime state of a simulated subsystem
 */
struct SimulatedSubsystemState {
    std::string current_state = "idle";
    double current_value = 0;       // Position, velocity, etc.
    double target_value = 0;
    bool is_busy = false;
    double action_start_time = 0;
    std::string current_action;
    bool has_game_piece = false;    // For intake
};

/**
 * @brief Preset subsystem configurations for common FRC mechanisms
 */
namespace SubsystemPresets {

    /**
     * @brief Create a fixed shooter subsystem configuration
     */
    inline SubsystemConfig create_shooter(const std::string& name = "ShooterSubsystem") {
        SubsystemConfig config;
        config.name = name;
        config.type = SubsystemType::SHOOTER;
        config.description = "Fixed-position shooter (no turret)";
        config.max_velocity = 6000;  // RPM
        config.states = {"idle", "spinning_up", "ready", "firing"};

        // Spin up action
        SubsystemAction spin_up;
        spin_up.name = "spinUp";
        spin_up.description = "Spin up shooter wheels to target RPM";
        spin_up.action_type = RobotAction::SPIN_UP_SHOOTER;
        spin_up.typical_duration = 1.5;
        spin_up.parameters.push_back({"rpm", "double", 5000, 0, 6000, "rpm"});
        config.actions.push_back(spin_up);

        // Fire action
        SubsystemAction fire;
        fire.name = "fire";
        fire.description = "Fire a game piece";
        fire.action_type = RobotAction::SHOOT;
        fire.typical_duration = 0.3;
        fire.required_state = "ready";
        config.actions.push_back(fire);

        // Fire multiple
        SubsystemAction fire_multi;
        fire_multi.name = "fireMultiple";
        fire_multi.description = "Fire multiple game pieces";
        fire_multi.action_type = RobotAction::SHOOT;
        fire_multi.typical_duration = 1.5;
        fire_multi.parameters.push_back({"count", "int", 2, 1, 5, ""});
        fire_multi.parameters.push_back({"delay_between", "double", 0.3, 0.1, 1.0, "s"});
        config.actions.push_back(fire_multi);

        // Stop
        SubsystemAction stop;
        stop.name = "stop";
        stop.description = "Stop shooter wheels";
        stop.action_type = RobotAction::SHOOTER_STOP;
        stop.is_instant = true;
        config.actions.push_back(stop);

        return config;
    }

    /**
     * @brief Create an intake subsystem configuration
     */
    inline SubsystemConfig create_intake(const std::string& name = "IntakeSubsystem") {
        SubsystemConfig config;
        config.name = name;
        config.type = SubsystemType::INTAKE;
        config.description = "Game piece intake mechanism";
        config.states = {"idle", "intaking", "holding", "ejecting"};

        // Intake in
        SubsystemAction intake_in;
        intake_in.name = "intakeIn";
        intake_in.description = "Run intake to collect game piece";
        intake_in.action_type = RobotAction::INTAKE_IN;
        intake_in.typical_duration = 2.0;
        intake_in.parameters.push_back({"speed", "double", 1.0, 0.1, 1.0, ""});
        config.actions.push_back(intake_in);

        // Intake out (eject)
        SubsystemAction intake_out;
        intake_out.name = "eject";
        intake_out.description = "Eject game piece";
        intake_out.action_type = RobotAction::INTAKE_OUT;
        intake_out.typical_duration = 0.5;
        config.actions.push_back(intake_out);

        // Stop
        SubsystemAction stop;
        stop.name = "stop";
        stop.description = "Stop intake";
        stop.action_type = RobotAction::INTAKE_STOP;
        stop.is_instant = true;
        config.actions.push_back(stop);

        return config;
    }

    /**
     * @brief Create an elevator subsystem configuration
     */
    inline SubsystemConfig create_elevator(const std::string& name = "ElevatorSubsystem") {
        SubsystemConfig config;
        config.name = name;
        config.type = SubsystemType::ELEVATOR;
        config.description = "Elevator/lift mechanism";
        config.max_velocity = 1.5;  // m/s
        config.max_acceleration = 3.0;
        config.position_tolerance = 0.02;  // meters
        config.states = {"idle", "moving", "at_position"};

        // Set height
        SubsystemAction set_height;
        set_height.name = "setHeight";
        set_height.description = "Move elevator to specific height";
        set_height.action_type = RobotAction::SET_ELEVATOR_HEIGHT;
        set_height.typical_duration = 1.0;
        set_height.parameters.push_back({"height", "double", 0.5, 0, 1.5, "m"});
        config.actions.push_back(set_height);

        // Stow
        SubsystemAction stow;
        stow.name = "stow";
        stow.description = "Return elevator to stowed position";
        stow.action_type = RobotAction::STOW;
        stow.typical_duration = 0.8;
        config.actions.push_back(stow);

        // Preset positions
        SubsystemAction preset_low;
        preset_low.name = "goToLow";
        preset_low.description = "Move to low scoring position";
        preset_low.action_type = RobotAction::SET_ELEVATOR_HEIGHT;
        preset_low.typical_duration = 0.6;
        config.actions.push_back(preset_low);

        SubsystemAction preset_high;
        preset_high.name = "goToHigh";
        preset_high.description = "Move to high scoring position";
        preset_high.action_type = RobotAction::SET_ELEVATOR_HEIGHT;
        preset_high.typical_duration = 1.2;
        config.actions.push_back(preset_high);

        return config;
    }

} // namespace SubsystemPresets

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
    // Action Block System (Visual Block Coding)
    // ========================================================================

    /**
     * @brief Add action block to path
     */
    void add_action_block(AutoPath& path, const ActionBlock& block);

    /**
     * @brief Create a simple action block (single condition + single action)
     */
    static ActionBlock create_simple_block(
        const std::string& name,
        TriggerCondition trigger_type,
        RobotAction action,
        int waypoint_index = -1,
        int tag_id = -1,
        double distance = 0);

    /**
     * @brief Create a combined action block (waypoint + tag condition)
     */
    static ActionBlock create_waypoint_tag_block(
        const std::string& name,
        int waypoint_index,
        int tag_id,
        double max_distance,
        RobotAction action);

    /**
     * @brief Reset all action blocks for path execution
     */
    static void reset_action_blocks(AutoPath& path);

    /**
     * @brief Check action block conditions during execution
     * @param block Action block to check
     * @param current_waypoint Current waypoint index
     * @param current_time Current time in trajectory
     * @param visible_tags Set of currently visible tag IDs
     * @param tag_distances Map of tag ID to distance
     * @param robot_velocity Current robot velocity
     * @return true if all/any conditions met (based on logic)
     */
    static bool check_block_conditions(
        const ActionBlock& block,
        int current_waypoint,
        double current_time,
        const std::set<int>& visible_tags,
        const std::map<int, double>& tag_distances,
        double robot_velocity);

    /**
     * @brief Get triggered action blocks for current state
     */
    static std::vector<const ActionBlock*> get_triggered_blocks(
        const AutoPath& path,
        int current_waypoint,
        double current_time,
        const std::set<int>& visible_tags,
        const std::map<int, double>& tag_distances,
        double robot_velocity);

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

    // ========================================================================
    // Subsystem Management
    // ========================================================================

    /**
     * @brief Register a subsystem configuration
     */
    void register_subsystem(const SubsystemConfig& config);

    /**
     * @brief Get registered subsystem by name
     */
    const SubsystemConfig* get_subsystem(const std::string& name) const;

    /**
     * @brief Get all registered subsystems
     */
    const std::map<std::string, SubsystemConfig>& get_subsystems() const { return subsystems_; }

    /**
     * @brief Get all available actions from registered subsystems
     */
    std::vector<std::pair<std::string, SubsystemAction>> get_all_subsystem_actions() const;

    /**
     * @brief Register default subsystems (shooter, intake, elevator)
     */
    void register_default_subsystems();

    /**
     * @brief Simulate a subsystem action
     * @return Duration of the action in seconds
     */
    static double simulate_subsystem_action(
        const SubsystemConfig& subsystem,
        const SubsystemAction& action,
        SimulatedSubsystemState& state,
        const std::map<std::string, double>& parameters);

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

    // Registered subsystems
    std::map<std::string, SubsystemConfig> subsystems_;

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
