/**
 * @file path_planner.cpp
 * @brief Production-grade Auto Coder for FRC 2026
 *
 * Generates complete, compilable WPILib Java/C++/Python code for:
 * - Swerve path following with proper heading profiles
 * - Event markers and tag-triggered actions
 * - Subsystem and command scaffolds
 * - Alliance mirroring support
 */

#include "path_planner.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>

namespace frc_vision {
namespace sim {

using json = nlohmann::json;

PathPlanner::PathPlanner() = default;

void PathPlanner::set_swerve_config(const SwerveConfig& config) {
    swerve_config_ = config;
}

void PathPlanner::set_field_layout(const FieldLayout& field) {
    field_ = field;
}

AutoPath& PathPlanner::create_path(const std::string& name) {
    paths_[name] = AutoPath();
    paths_[name].name = name;
    return paths_[name];
}

AutoPath* PathPlanner::get_path(const std::string& name) {
    auto it = paths_.find(name);
    return (it != paths_.end()) ? &it->second : nullptr;
}

void PathPlanner::add_waypoint(AutoPath& path, const PathWaypoint& waypoint) {
    path.waypoints.push_back(waypoint);
}

void PathPlanner::add_event_marker(AutoPath& path, const EventMarker& marker) {
    path.event_markers.push_back(marker);
    // Track required subsystem
    std::string subsystem = action_to_subsystem(marker.action);
    if (!subsystem.empty()) {
        path.required_subsystems.insert(subsystem);
    }
    path.required_commands.insert(action_to_command_name(marker.action));
}

void PathPlanner::add_tag_binding(AutoPath& path, const TagActionBinding& binding) {
    path.tag_bindings.push_back(binding);
    std::string subsystem = action_to_subsystem(binding.action);
    if (!subsystem.empty()) {
        path.required_subsystems.insert(subsystem);
    }
    path.required_commands.insert(action_to_command_name(binding.action));
}

void PathPlanner::add_action(AutoPath& path, const PathAction& action) {
    path.actions.push_back(action);
}

// ============================================================================
// Action Block System Implementation
// ============================================================================

void PathPlanner::add_action_block(AutoPath& path, const ActionBlock& block) {
    path.action_blocks.push_back(block);
}

ActionBlock PathPlanner::create_simple_block(
    const std::string& name,
    TriggerCondition trigger_type,
    RobotAction action,
    int waypoint_index,
    int tag_id,
    double distance)
{
    ActionBlock block;
    block.name = name;

    // Create the condition
    Condition cond;
    cond.type = trigger_type;
    cond.waypoint_index = waypoint_index;
    cond.tag_id = tag_id;
    cond.distance = distance;
    block.conditions.push_back(cond);

    // Create the action
    ActionBlock::ActionStep step;
    step.action = action;
    block.actions.push_back(step);

    return block;
}

ActionBlock PathPlanner::create_waypoint_tag_block(
    const std::string& name,
    int waypoint_index,
    int tag_id,
    double max_distance,
    RobotAction action)
{
    ActionBlock block;
    block.name = name;
    block.description = "At waypoint " + std::to_string(waypoint_index) +
                       " when tag " + std::to_string(tag_id) +
                       " is within " + std::to_string(max_distance) + "m";
    block.logic = ConditionLogic::AND;

    // Condition 1: At waypoint
    Condition wp_cond;
    wp_cond.type = TriggerCondition::AT_WAYPOINT;
    wp_cond.waypoint_index = waypoint_index;
    block.conditions.push_back(wp_cond);

    // Condition 2: Tag within distance
    Condition tag_cond;
    tag_cond.type = TriggerCondition::DISTANCE_TO_TAG;
    tag_cond.tag_id = tag_id;
    tag_cond.distance = max_distance;
    tag_cond.compare = Condition::Compare::LESS_THAN;
    block.conditions.push_back(tag_cond);

    // Action
    ActionBlock::ActionStep step;
    step.action = action;
    block.actions.push_back(step);

    return block;
}

void PathPlanner::reset_action_blocks(AutoPath& path) {
    for (auto& block : path.action_blocks) {
        block.reset();
    }
}

bool PathPlanner::check_block_conditions(
    const ActionBlock& block,
    int current_waypoint,
    double current_time,
    const std::set<int>& visible_tags,
    const std::map<int, double>& tag_distances,
    double robot_velocity)
{
    if (!block.enabled) return false;
    if (block.one_shot && block.has_triggered) return false;

    bool result = (block.logic == ConditionLogic::AND);  // AND starts true, OR starts false

    for (const auto& cond : block.conditions) {
        bool cond_met = false;

        switch (cond.type) {
            case TriggerCondition::AT_WAYPOINT:
                cond_met = (current_waypoint >= cond.waypoint_index);
                break;

            case TriggerCondition::AT_TIME:
                cond_met = (current_time >= cond.time);
                break;

            case TriggerCondition::TAG_VISIBLE:
                cond_met = (visible_tags.count(cond.tag_id) > 0);
                break;

            case TriggerCondition::DISTANCE_TO_TAG: {
                auto it = tag_distances.find(cond.tag_id);
                if (it != tag_distances.end()) {
                    double dist = it->second;
                    switch (cond.compare) {
                        case Condition::Compare::LESS_THAN:
                            cond_met = (dist < cond.distance);
                            break;
                        case Condition::Compare::GREATER_THAN:
                            cond_met = (dist > cond.distance);
                            break;
                        case Condition::Compare::EQUAL:
                            cond_met = (std::abs(dist - cond.distance) < 0.1);
                            break;
                    }
                }
                break;
            }

            case TriggerCondition::TAG_ALIGNED:
                // Simplified: tag is visible and close
                cond_met = (visible_tags.count(cond.tag_id) > 0);
                if (cond_met) {
                    auto it = tag_distances.find(cond.tag_id);
                    cond_met = (it != tag_distances.end() && it->second < 2.0);
                }
                break;

            case TriggerCondition::VELOCITY_BELOW:
                cond_met = (robot_velocity < cond.threshold);
                break;

            case TriggerCondition::GAME_PIECE_DETECTED:
                // Would need sensor input - assume false for now
                cond_met = false;
                break;

            case TriggerCondition::CUSTOM:
                // Custom conditions not evaluated here
                cond_met = false;
                break;
        }

        // Handle edge detection (trigger on false->true transition)
        if (cond.trigger_on_edge) {
            bool should_trigger = cond_met && !cond.was_true;
            cond.was_true = cond_met;
            cond_met = should_trigger;
        }

        // Apply logic
        if (block.logic == ConditionLogic::AND) {
            result = result && cond_met;
        } else {
            result = result || cond_met;
        }
    }

    return result;
}

std::vector<const ActionBlock*> PathPlanner::get_triggered_blocks(
    const AutoPath& path,
    int current_waypoint,
    double current_time,
    const std::set<int>& visible_tags,
    const std::map<int, double>& tag_distances,
    double robot_velocity)
{
    std::vector<const ActionBlock*> triggered;

    for (const auto& block : path.action_blocks) {
        if (check_block_conditions(block, current_waypoint, current_time,
                                   visible_tags, tag_distances, robot_velocity)) {
            triggered.push_back(&block);
            block.has_triggered = true;
            block.trigger_time = current_time;
        }
    }

    return triggered;
}

std::vector<RobotAction> PathPlanner::get_available_actions() {
    return {
        RobotAction::INTAKE_IN,
        RobotAction::INTAKE_OUT,
        RobotAction::INTAKE_STOP,
        RobotAction::SPIN_UP_SHOOTER,
        RobotAction::SHOOT,
        RobotAction::SHOOTER_STOP,
        RobotAction::SET_ELEVATOR_HEIGHT,
        RobotAction::STOW,
        RobotAction::EXTEND_ARM,
        RobotAction::RETRACT_ARM,
        RobotAction::AUTO_ALIGN_TO_TAG,
        RobotAction::FACE_TAG,
        RobotAction::TRACK_TAG,
        RobotAction::DEPLOY_CLIMBER,
        RobotAction::CLIMB,
        RobotAction::WAIT,
        RobotAction::CUSTOM_COMMAND
    };
}

std::vector<RobotAction> PathPlanner::get_actions_for_tag(int tag_id) const {
    // Return actions relevant for tags (scoring, alignment, etc.)
    (void)tag_id;  // In future, could customize based on tag type
    return {
        RobotAction::AUTO_ALIGN_TO_TAG,
        RobotAction::FACE_TAG,
        RobotAction::SHOOT,
        RobotAction::INTAKE_IN,
        RobotAction::SET_ELEVATOR_HEIGHT,
        RobotAction::STOW
    };
}

// ============================================================================
// Subsystem Management Implementation
// ============================================================================

void PathPlanner::register_subsystem(const SubsystemConfig& config) {
    subsystems_[config.name] = config;
    std::cout << "[PathPlanner] Registered subsystem: " << config.name
              << " with " << config.actions.size() << " actions" << std::endl;
}

const SubsystemConfig* PathPlanner::get_subsystem(const std::string& name) const {
    auto it = subsystems_.find(name);
    return (it != subsystems_.end()) ? &it->second : nullptr;
}

std::vector<std::pair<std::string, SubsystemAction>> PathPlanner::get_all_subsystem_actions() const {
    std::vector<std::pair<std::string, SubsystemAction>> all_actions;
    for (const auto& [name, subsystem] : subsystems_) {
        for (const auto& action : subsystem.actions) {
            all_actions.push_back({name, action});
        }
    }
    return all_actions;
}

void PathPlanner::register_default_subsystems() {
    // Register standard FRC subsystems
    register_subsystem(SubsystemPresets::create_shooter());
    register_subsystem(SubsystemPresets::create_intake());
    register_subsystem(SubsystemPresets::create_elevator());
    std::cout << "[PathPlanner] Registered default subsystems" << std::endl;
}

double PathPlanner::simulate_subsystem_action(
    const SubsystemConfig& subsystem,
    const SubsystemAction& action,
    SimulatedSubsystemState& state,
    const std::map<std::string, double>& parameters)
{
    // Update state based on action
    state.current_action = action.name;
    state.is_busy = !action.is_instant;

    // Handle different subsystem types
    switch (subsystem.type) {
        case SubsystemType::SHOOTER:
            if (action.action_type == RobotAction::SPIN_UP_SHOOTER) {
                state.current_state = "spinning_up";
                auto it = parameters.find("rpm");
                state.target_value = (it != parameters.end()) ? it->second : 5000;
            } else if (action.action_type == RobotAction::SHOOT) {
                state.current_state = "firing";
                state.has_game_piece = false;
            } else if (action.action_type == RobotAction::SHOOTER_STOP) {
                state.current_state = "idle";
                state.target_value = 0;
            }
            break;

        case SubsystemType::INTAKE:
            if (action.action_type == RobotAction::INTAKE_IN) {
                state.current_state = "intaking";
            } else if (action.action_type == RobotAction::INTAKE_OUT) {
                state.current_state = "ejecting";
                state.has_game_piece = false;
            } else if (action.action_type == RobotAction::INTAKE_STOP) {
                state.current_state = "idle";
            }
            break;

        case SubsystemType::ELEVATOR:
            if (action.action_type == RobotAction::SET_ELEVATOR_HEIGHT) {
                state.current_state = "moving";
                auto it = parameters.find("height");
                state.target_value = (it != parameters.end()) ? it->second : 0.5;
            } else if (action.action_type == RobotAction::STOW) {
                state.current_state = "moving";
                state.target_value = 0;
            }
            break;

        default:
            break;
    }

    return action.typical_duration;
}

Pose2D PathPlanner::mirror_pose(const Pose2D& pose) const {
    Pose2D mirrored;
    mirrored.x = FIELD_LENGTH - pose.x;
    mirrored.y = pose.y;
    mirrored.theta = M_PI - pose.theta;
    while (mirrored.theta > M_PI) mirrored.theta -= 2 * M_PI;
    while (mirrored.theta < -M_PI) mirrored.theta += 2 * M_PI;
    return mirrored;
}

// ============================================================================
// Trajectory Generation
// ============================================================================

Pose2D PathPlanner::bezier_point(const PathWaypoint& p0, const PathWaypoint& p1, double t) const {
    double t2 = t * t;
    double t3 = t2 * t;
    double mt = 1.0 - t;
    double mt2 = mt * mt;
    double mt3 = mt2 * mt;

    double c0x = p0.pose.x + p0.control_out_x;
    double c0y = p0.pose.y + p0.control_out_y;
    double c1x = p1.pose.x + p1.control_in_x;
    double c1y = p1.pose.y + p1.control_in_y;

    Pose2D result;
    result.x = mt3 * p0.pose.x + 3 * mt2 * t * c0x + 3 * mt * t2 * c1x + t3 * p1.pose.x;
    result.y = mt3 * p0.pose.y + 3 * mt2 * t * c0y + 3 * mt * t2 * c1y + t3 * p1.pose.y;

    // Handle heading based on rotation mode
    if (p0.rotation_mode == RotationMode::LINEAR || p0.rotation_mode == RotationMode::HOLONOMIC) {
        double diff = p1.pose.theta - p0.pose.theta;
        while (diff > M_PI) diff -= 2 * M_PI;
        while (diff < -M_PI) diff += 2 * M_PI;
        result.theta = p0.pose.theta + t * diff;
    } else if (p0.rotation_mode == RotationMode::HOLD_HEADING) {
        result.theta = p0.pose.theta;
    } else {
        result.theta = p0.pose.theta;
    }

    return result;
}

double PathPlanner::bezier_curvature(const PathWaypoint& p0, const PathWaypoint& p1, double t) const {
    double mt = 1.0 - t;

    double c0x = p0.pose.x + p0.control_out_x;
    double c0y = p0.pose.y + p0.control_out_y;
    double c1x = p1.pose.x + p1.control_in_x;
    double c1y = p1.pose.y + p1.control_in_y;

    double dx = 3 * mt * mt * (c0x - p0.pose.x) + 6 * mt * t * (c1x - c0x) + 3 * t * t * (p1.pose.x - c1x);
    double dy = 3 * mt * mt * (c0y - p0.pose.y) + 6 * mt * t * (c1y - c0y) + 3 * t * t * (p1.pose.y - c1y);

    double ddx = 6 * mt * (c1x - 2 * c0x + p0.pose.x) + 6 * t * (p1.pose.x - 2 * c1x + c0x);
    double ddy = 6 * mt * (c1y - 2 * c0y + p0.pose.y) + 6 * t * (p1.pose.y - 2 * c1y + c0y);

    double denom = std::pow(dx * dx + dy * dy, 1.5);
    if (denom < 1e-6) return 0;

    return std::abs(dx * ddy - dy * ddx) / denom;
}

double PathPlanner::calculate_path_length(const AutoPath& path) const {
    if (path.waypoints.size() < 2) return 0;

    double length = 0;
    for (size_t i = 0; i < path.waypoints.size() - 1; i++) {
        const int steps = 50;
        Pose2D prev = bezier_point(path.waypoints[i], path.waypoints[i + 1], 0);
        for (int j = 1; j <= steps; j++) {
            double t = static_cast<double>(j) / steps;
            Pose2D curr = bezier_point(path.waypoints[i], path.waypoints[i + 1], t);
            double dx = curr.x - prev.x;
            double dy = curr.y - prev.y;
            length += std::sqrt(dx * dx + dy * dy);
            prev = curr;
        }
    }

    return length;
}

void PathPlanner::generate_trajectory(AutoPath& path, double dt) {
    if (path.waypoints.size() < 2) return;

    path.trajectory.clear();
    path.total_distance = calculate_path_length(path);

    double max_vel = path.max_velocity;
    double max_accel = path.max_acceleration;

    std::vector<double> segment_lengths;
    for (size_t i = 0; i < path.waypoints.size() - 1; i++) {
        double len = 0;
        Pose2D prev = bezier_point(path.waypoints[i], path.waypoints[i + 1], 0);
        for (int j = 1; j <= 50; j++) {
            double t = static_cast<double>(j) / 50;
            Pose2D curr = bezier_point(path.waypoints[i], path.waypoints[i + 1], t);
            len += std::sqrt(std::pow(curr.x - prev.x, 2) + std::pow(curr.y - prev.y, 2));
            prev = curr;
        }
        segment_lengths.push_back(len);
    }

    double accel_dist = (max_vel * max_vel) / (2 * max_accel);
    double cruise_dist = std::max(0.0, path.total_distance - 2 * accel_dist);

    double accel_time = max_vel / max_accel;
    double cruise_time = cruise_dist / max_vel;
    path.total_time = 2 * accel_time + cruise_time;

    double time = 0;
    size_t current_segment = 0;
    double segment_progress = 0;

    while (time <= path.total_time + dt && current_segment < path.waypoints.size() - 1) {
        AutoPath::TrajectoryPoint pt;
        pt.time = time;

        double velocity, acceleration;
        if (time < accel_time) {
            velocity = max_accel * time;
            acceleration = max_accel;
        } else if (time < accel_time + cruise_time) {
            velocity = max_vel;
            acceleration = 0;
        } else {
            double decel_time = time - accel_time - cruise_time;
            velocity = max_vel - max_accel * decel_time;
            acceleration = -max_accel;
        }
        velocity = std::max(0.0, velocity);

        pt.velocity = velocity;
        pt.acceleration = acceleration;

        double dist_at_time;
        if (time < accel_time) {
            dist_at_time = 0.5 * max_accel * time * time;
        } else if (time < accel_time + cruise_time) {
            dist_at_time = accel_dist + max_vel * (time - accel_time);
        } else {
            double decel_time = time - accel_time - cruise_time;
            dist_at_time = accel_dist + cruise_dist + max_vel * decel_time - 0.5 * max_accel * decel_time * decel_time;
        }

        double remaining_dist = dist_at_time;
        current_segment = 0;
        for (size_t i = 0; i < segment_lengths.size(); i++) {
            if (remaining_dist <= segment_lengths[i]) {
                current_segment = i;
                segment_progress = remaining_dist / segment_lengths[i];
                break;
            }
            remaining_dist -= segment_lengths[i];
            current_segment = i + 1;
        }

        if (current_segment >= path.waypoints.size() - 1) {
            current_segment = path.waypoints.size() - 2;
            segment_progress = 1.0;
        }

        pt.pose = bezier_point(path.waypoints[current_segment],
                               path.waypoints[current_segment + 1],
                               segment_progress);
        pt.curvature = bezier_curvature(path.waypoints[current_segment],
                                        path.waypoints[current_segment + 1],
                                        segment_progress);

        if (!path.trajectory.empty()) {
            double dtheta = pt.pose.theta - path.trajectory.back().pose.theta;
            while (dtheta > M_PI) dtheta -= 2 * M_PI;
            while (dtheta < -M_PI) dtheta += 2 * M_PI;
            pt.angular_velocity = dtheta / dt;
        } else {
            pt.angular_velocity = 0;
        }

        path.trajectory.push_back(pt);
        time += dt;
    }
}

Pose2D PathPlanner::sample_path(const AutoPath& path, double t) const {
    if (path.trajectory.empty()) {
        return path.waypoints.empty() ? Pose2D() : path.waypoints[0].pose;
    }

    for (size_t i = 0; i < path.trajectory.size() - 1; i++) {
        if (t >= path.trajectory[i].time && t <= path.trajectory[i + 1].time) {
            double alpha = (t - path.trajectory[i].time) /
                          (path.trajectory[i + 1].time - path.trajectory[i].time);

            Pose2D result;
            result.x = path.trajectory[i].pose.x +
                      alpha * (path.trajectory[i + 1].pose.x - path.trajectory[i].pose.x);
            result.y = path.trajectory[i].pose.y +
                      alpha * (path.trajectory[i + 1].pose.y - path.trajectory[i].pose.y);

            double dtheta = path.trajectory[i + 1].pose.theta - path.trajectory[i].pose.theta;
            while (dtheta > M_PI) dtheta -= 2 * M_PI;
            while (dtheta < -M_PI) dtheta += 2 * M_PI;
            result.theta = path.trajectory[i].pose.theta + alpha * dtheta;

            return result;
        }
    }

    return path.trajectory.back().pose;
}

double PathPlanner::sample_heading(const AutoPath& path, double t) const {
    return sample_path(path, t).theta;
}

double PathPlanner::sample_velocity(const AutoPath& path, double t) const {
    if (path.trajectory.empty()) return 0;

    for (size_t i = 0; i < path.trajectory.size() - 1; i++) {
        if (t >= path.trajectory[i].time && t <= path.trajectory[i + 1].time) {
            double alpha = (t - path.trajectory[i].time) /
                          (path.trajectory[i + 1].time - path.trajectory[i].time);
            return path.trajectory[i].velocity +
                   alpha * (path.trajectory[i + 1].velocity - path.trajectory[i].velocity);
        }
    }

    return 0;
}

// ============================================================================
// Code Generation - Main Entry Point
// ============================================================================

CodeGenerationResult PathPlanner::generate_all_code(
    const AutoPath& path,
    CodeFormat format,
    bool generate_scaffolds) const {

    CodeGenerationResult result;

    if (format != CodeFormat::WPILIB_JAVA) {
        result.warnings.push_back("Only Java code generation is fully supported for WPILib 2026");
    }

    // Generate field constants
    result.files.push_back(generate_field_constants(format));

    // Generate drivetrain interface
    result.files.push_back(generate_drivetrain_interface(format));

    // Generate trajectory/path class
    GeneratedFile path_file;
    path_file.path = "frc/robot/auto/" + path.name + "Path.java";
    path_file.content = generate_java_trajectory_class(path);
    path_file.description = "Trajectory and path-following command for " + path.name;
    result.files.push_back(path_file);

    // Generate event map
    if (!path.event_markers.empty()) {
        result.files.push_back(generate_event_map(path, format));
    }

    // Generate auto routine
    GeneratedFile routine_file;
    routine_file.path = "frc/robot/auto/AutoRoutine_" + path.name + ".java";
    routine_file.content = generate_java_auto_routine(path);
    routine_file.description = "Complete autonomous routine for " + path.name;
    result.files.push_back(routine_file);

    // Generate scaffolds if requested
    if (generate_scaffolds) {
        for (const auto& subsystem : path.required_subsystems) {
            result.files.push_back(generate_subsystem_scaffold(subsystem, format));
        }

        for (const auto& cmd : path.required_commands) {
            // Find which subsystem this command needs
            std::string subsystem;
            for (const auto& marker : path.event_markers) {
                if (action_to_command_name(marker.action) == cmd) {
                    subsystem = action_to_subsystem(marker.action);
                    break;
                }
            }
            result.files.push_back(generate_command_scaffold(cmd, subsystem, format));
        }
    }

    // Add integration notes
    result.integration_notes.push_back("=== INTEGRATION INSTRUCTIONS ===");
    result.integration_notes.push_back("");
    result.integration_notes.push_back("1. Copy generated files to your robot project:");
    for (const auto& file : result.files) {
        result.integration_notes.push_back("   - " + file.path);
    }
    result.integration_notes.push_back("");
    result.integration_notes.push_back("2. In RobotContainer.java, register the auto:");
    result.integration_notes.push_back("   autoChooser.addOption(\"" + path.name + "\", ");
    result.integration_notes.push_back("       AutoRoutine_" + path.name + ".getCommand(drivetrain, subsystems...));");
    result.integration_notes.push_back("");
    result.integration_notes.push_back("3. Tune PID gains in " + path.name + "Path.java");
    result.integration_notes.push_back("   - Translation: kP=" + std::to_string(swerve_config_.translation_kP));
    result.integration_notes.push_back("   - Rotation: kP=" + std::to_string(swerve_config_.rotation_kP));
    result.integration_notes.push_back("");
    result.integration_notes.push_back("4. Fill in TODO sections in subsystem/command scaffolds");

    return result;
}

std::string PathPlanner::generate_code(const AutoPath& path, CodeFormat format) const {
    switch (format) {
        case CodeFormat::WPILIB_JAVA:
            return generate_java_trajectory_class(path);
        case CodeFormat::WPILIB_CPP:
            return generate_cpp_path(path);
        case CodeFormat::WPILIB_PYTHON:
            return generate_python_path(path);
        case CodeFormat::PATHPLANNER_JSON:
            return generate_pathplanner_json(path);
        default:
            return "";
    }
}

// ============================================================================
// Java Code Generation
// ============================================================================

std::string PathPlanner::generate_java_trajectory_class(const AutoPath& path) const {
    std::ostringstream ss;

    ss << "// ============================================================================\n";
    ss << "// Auto-generated by FRC Vision Auto Coder - " << path.name << "\n";
    ss << "// WPILib 2026 Compliant - Compiles without modification\n";
    ss << "// ============================================================================\n\n";

    ss << "package frc.robot.auto;\n\n";

    // All required imports
    ss << "import edu.wpi.first.math.controller.HolonomicDriveController;\n";
    ss << "import edu.wpi.first.math.controller.PIDController;\n";
    ss << "import edu.wpi.first.math.controller.ProfiledPIDController;\n";
    ss << "import edu.wpi.first.math.geometry.Pose2d;\n";
    ss << "import edu.wpi.first.math.geometry.Rotation2d;\n";
    ss << "import edu.wpi.first.math.geometry.Translation2d;\n";
    ss << "import edu.wpi.first.math.kinematics.ChassisSpeeds;\n";
    ss << "import edu.wpi.first.math.kinematics.SwerveDriveKinematics;\n";
    ss << "import edu.wpi.first.math.trajectory.Trajectory;\n";
    ss << "import edu.wpi.first.math.trajectory.TrajectoryConfig;\n";
    ss << "import edu.wpi.first.math.trajectory.TrajectoryGenerator;\n";
    ss << "import edu.wpi.first.math.trajectory.TrapezoidProfile;\n";
    ss << "import edu.wpi.first.wpilibj.DriverStation;\n";
    ss << "import edu.wpi.first.wpilibj.Timer;\n";
    ss << "import edu.wpi.first.wpilibj2.command.Command;\n";
    ss << "import edu.wpi.first.wpilibj2.command.Commands;\n";
    ss << "import edu.wpi.first.wpilibj2.command.FunctionalCommand;\n";
    ss << "import frc.robot.subsystems.IDrivetrain;\n";
    ss << "import java.util.List;\n";
    ss << "import java.util.Optional;\n";
    ss << "import java.util.function.Supplier;\n\n";

    ss << "/**\n";
    ss << " * Path: " << path.name << "\n";
    ss << " * " << path.description << "\n";
    ss << " *\n";
    ss << " * Waypoints: " << path.waypoints.size() << "\n";
    ss << " * Estimated time: " << std::fixed << std::setprecision(2) << path.total_time << "s\n";
    ss << " * Max velocity: " << path.max_velocity << " m/s\n";
    ss << " * Max acceleration: " << path.max_acceleration << " m/s²\n";
    ss << " */\n";
    ss << "public class " << path.name << "Path {\n\n";

    // Constants
    ss << "    // ========== Trajectory Constraints ==========\n";
    ss << "    public static final double MAX_VELOCITY = " << path.max_velocity << "; // m/s\n";
    ss << "    public static final double MAX_ACCELERATION = " << path.max_acceleration << "; // m/s²\n";
    ss << "    public static final double MAX_ANGULAR_VELOCITY = " << path.max_angular_velocity << "; // rad/s\n";
    ss << "    public static final double MAX_ANGULAR_ACCEL = " << path.max_angular_accel << "; // rad/s²\n\n";

    // PID Constants
    ss << "    // ========== Path Following PID (TUNE THESE) ==========\n";
    ss << "    public static final double TRANSLATION_KP = " << swerve_config_.translation_kP << ";\n";
    ss << "    public static final double TRANSLATION_KI = " << swerve_config_.translation_kI << ";\n";
    ss << "    public static final double TRANSLATION_KD = " << swerve_config_.translation_kD << ";\n\n";
    ss << "    public static final double ROTATION_KP = " << swerve_config_.rotation_kP << ";\n";
    ss << "    public static final double ROTATION_KI = " << swerve_config_.rotation_kI << ";\n";
    ss << "    public static final double ROTATION_KD = " << swerve_config_.rotation_kD << ";\n\n";

    // Waypoints
    if (!path.waypoints.empty()) {
        ss << "    // ========== Waypoints ==========\n";
        ss << "    public static final Pose2d START_POSE = new Pose2d(\n";
        ss << "        " << path.waypoints.front().pose.x << ",\n";
        ss << "        " << path.waypoints.front().pose.y << ",\n";
        ss << "        Rotation2d.fromRadians(" << path.waypoints.front().pose.theta << ")\n";
        ss << "    );\n\n";

        ss << "    public static final Pose2d END_POSE = new Pose2d(\n";
        ss << "        " << path.waypoints.back().pose.x << ",\n";
        ss << "        " << path.waypoints.back().pose.y << ",\n";
        ss << "        Rotation2d.fromRadians(" << path.waypoints.back().pose.theta << ")\n";
        ss << "    );\n\n";

        ss << "    public static final List<Translation2d> INTERIOR_WAYPOINTS = List.of(\n";
        for (size_t i = 1; i < path.waypoints.size() - 1; i++) {
            ss << "        new Translation2d(" << path.waypoints[i].pose.x << ", "
               << path.waypoints[i].pose.y << ")";
            if (i < path.waypoints.size() - 2) ss << ",";
            ss << "\n";
        }
        ss << "    );\n\n";
    }

    // getTrajectoryConfig method
    ss << "    // ========== Trajectory Generation ==========\n";
    ss << "    public static TrajectoryConfig getTrajectoryConfig(SwerveDriveKinematics kinematics) {\n";
    ss << "        return new TrajectoryConfig(MAX_VELOCITY, MAX_ACCELERATION)\n";
    ss << "            .setKinematics(kinematics)\n";
    ss << "            .setReversed(" << (path.reverse ? "true" : "false") << ");\n";
    ss << "    }\n\n";

    // getTrajectory method with mirroring
    ss << "    public static Trajectory getTrajectory(SwerveDriveKinematics kinematics, boolean mirrorForRed) {\n";
    ss << "        Pose2d start = mirrorForRed ? FieldConstants.mirror(START_POSE) : START_POSE;\n";
    ss << "        Pose2d end = mirrorForRed ? FieldConstants.mirror(END_POSE) : END_POSE;\n";
    ss << "        List<Translation2d> interior = mirrorForRed\n";
    ss << "            ? INTERIOR_WAYPOINTS.stream().map(FieldConstants::mirror).toList()\n";
    ss << "            : INTERIOR_WAYPOINTS;\n\n";
    ss << "        return TrajectoryGenerator.generateTrajectory(\n";
    ss << "            start, interior, end, getTrajectoryConfig(kinematics)\n";
    ss << "        );\n";
    ss << "    }\n\n";

    // Alliance-aware trajectory getter
    ss << "    public static Trajectory getTrajectoryForAlliance(SwerveDriveKinematics kinematics) {\n";
    ss << "        boolean isRed = DriverStation.getAlliance()\n";
    ss << "            .map(alliance -> alliance == DriverStation.Alliance.Red)\n";
    ss << "            .orElse(false);\n";
    ss << "        return getTrajectory(kinematics, isRed && " << (path.mirror_for_red ? "true" : "false") << ");\n";
    ss << "    }\n\n";

    // Heading supplier based on mode
    ss << "    // ========== Heading Profile ==========\n";
    ss << "    /**\n";
    ss << "     * Get heading supplier for path following.\n";
    ss << "     * Mode: " << static_cast<int>(path.heading_mode) << "\n";
    ss << "     */\n";
    ss << "    public static Supplier<Rotation2d> getHeadingSupplier(\n";
    ss << "            Trajectory trajectory, Timer timer, boolean mirrorForRed) {\n";

    switch (path.heading_mode) {
        case RotationMode::LINEAR:
        case RotationMode::HOLONOMIC:
            ss << "        // Linear interpolation from start to end heading\n";
            ss << "        Rotation2d startHeading = mirrorForRed\n";
            ss << "            ? FieldConstants.mirror(START_POSE).getRotation()\n";
            ss << "            : START_POSE.getRotation();\n";
            ss << "        Rotation2d endHeading = mirrorForRed\n";
            ss << "            ? FieldConstants.mirror(END_POSE).getRotation()\n";
            ss << "            : END_POSE.getRotation();\n";
            ss << "        double totalTime = trajectory.getTotalTimeSeconds();\n\n";
            ss << "        return () -> {\n";
            ss << "            double t = Math.min(timer.get() / totalTime, 1.0);\n";
            ss << "            return startHeading.interpolate(endHeading, t);\n";
            ss << "        };\n";
            break;
        case RotationMode::HOLD_HEADING:
            ss << "        // Hold heading at start value\n";
            ss << "        Rotation2d heading = mirrorForRed\n";
            ss << "            ? FieldConstants.mirror(START_POSE).getRotation()\n";
            ss << "            : START_POSE.getRotation();\n";
            ss << "        return () -> heading;\n";
            break;
        case RotationMode::FACE_TARGET:
            ss << "        // Face target point\n";
            ss << "        Translation2d target = mirrorForRed\n";
            ss << "            ? FieldConstants.mirror(new Translation2d(" << path.face_target.x << ", " << path.face_target.y << "))\n";
            ss << "            : new Translation2d(" << path.face_target.x << ", " << path.face_target.y << ");\n";
            ss << "        return () -> {\n";
            ss << "            Pose2d currentPose = trajectory.sample(timer.get()).poseMeters;\n";
            ss << "            Translation2d diff = target.minus(currentPose.getTranslation());\n";
            ss << "            return new Rotation2d(diff.getX(), diff.getY());\n";
            ss << "        };\n";
            break;
        default:
            ss << "        // Default: use trajectory tangent\n";
            ss << "        return () -> trajectory.sample(timer.get()).poseMeters.getRotation();\n";
    }
    ss << "    }\n\n";

    // Main command generation
    ss << "    // ========== Path Following Command ==========\n";
    ss << "    /**\n";
    ss << "     * Creates the path following command.\n";
    ss << "     *\n";
    ss << "     * @param drive The drivetrain subsystem (must implement IDrivetrain)\n";
    ss << "     * @return Command that follows this path\n";
    ss << "     */\n";
    ss << "    public static Command getCommand(IDrivetrain drive) {\n";
    ss << "        boolean isRed = DriverStation.getAlliance()\n";
    ss << "            .map(alliance -> alliance == DriverStation.Alliance.Red)\n";
    ss << "            .orElse(false);\n";
    ss << "        boolean mirror = isRed && " << (path.mirror_for_red ? "true" : "false") << ";\n\n";

    ss << "        Trajectory trajectory = getTrajectory(drive.getKinematics(), mirror);\n";
    ss << "        Timer timer = new Timer();\n\n";

    ss << "        // Create controllers\n";
    ss << "        PIDController xController = new PIDController(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD);\n";
    ss << "        PIDController yController = new PIDController(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD);\n";
    ss << "        ProfiledPIDController thetaController = new ProfiledPIDController(\n";
    ss << "            ROTATION_KP, ROTATION_KI, ROTATION_KD,\n";
    ss << "            new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCEL)\n";
    ss << "        );\n";
    ss << "        thetaController.enableContinuousInput(-Math.PI, Math.PI);\n\n";

    ss << "        HolonomicDriveController controller = new HolonomicDriveController(\n";
    ss << "            xController, yController, thetaController\n";
    ss << "        );\n\n";

    ss << "        Supplier<Rotation2d> headingSupplier = getHeadingSupplier(trajectory, timer, mirror);\n\n";

    // Create the command
    ss << "        return new FunctionalCommand(\n";
    ss << "            // Init\n";
    ss << "            () -> {\n";
    ss << "                timer.restart();\n";
    if (path.reset_odometry) {
        ss << "                Pose2d startPose = mirror ? FieldConstants.mirror(START_POSE) : START_POSE;\n";
        ss << "                drive.resetOdometry(startPose);\n";
    }
    ss << "            },\n";
    ss << "            // Execute\n";
    ss << "            () -> {\n";
    ss << "                double currentTime = timer.get();\n";
    ss << "                Trajectory.State desiredState = trajectory.sample(currentTime);\n";
    ss << "                Rotation2d targetHeading = headingSupplier.get();\n\n";
    ss << "                ChassisSpeeds targetSpeeds = controller.calculate(\n";
    ss << "                    drive.getPose(),\n";
    ss << "                    desiredState,\n";
    ss << "                    targetHeading\n";
    ss << "                );\n\n";
    ss << "                drive.drive(targetSpeeds);\n";
    ss << "            },\n";
    ss << "            // End\n";
    ss << "            interrupted -> {\n";
    ss << "                timer.stop();\n";
    if (path.stop_at_end) {
        ss << "                drive.stop();\n";
    }
    ss << "            },\n";
    ss << "            // IsFinished\n";
    ss << "            () -> timer.hasElapsed(trajectory.getTotalTimeSeconds()),\n";
    ss << "            drive\n";
    ss << "        ).withName(\"" << path.name << "Path\");\n";
    ss << "    }\n";
    ss << "}\n";

    return ss.str();
}

std::string PathPlanner::generate_java_auto_routine(const AutoPath& path) const {
    std::ostringstream ss;

    ss << "// ============================================================================\n";
    ss << "// Auto-generated Autonomous Routine: " << path.name << "\n";
    ss << "// WPILib 2026 Compliant\n";
    ss << "// ============================================================================\n\n";

    ss << "package frc.robot.auto;\n\n";

    ss << "import edu.wpi.first.wpilibj2.command.Command;\n";
    ss << "import edu.wpi.first.wpilibj2.command.Commands;\n";
    ss << "import frc.robot.subsystems.IDrivetrain;\n";

    // Import required subsystems
    for (const auto& subsystem : path.required_subsystems) {
        ss << "import frc.robot.subsystems." << subsystem << ";\n";
    }
    // Import required commands
    for (const auto& cmd : path.required_commands) {
        ss << "import frc.robot.commands." << cmd << ";\n";
    }
    ss << "\n";

    ss << "/**\n";
    ss << " * Autonomous routine: " << path.name << "\n";
    ss << " *\n";
    ss << " * Sequences:\n";
    ss << " * 1. Reset odometry to start pose\n";
    ss << " * 2. Follow path with event markers\n";
    ss << " * 3. Stop drivetrain at end\n";
    ss << " */\n";
    ss << "public class AutoRoutine_" << path.name << " {\n\n";

    // Generate the command method
    ss << "    /**\n";
    ss << "     * Creates the complete autonomous command.\n";
    ss << "     *\n";
    ss << "     * @param drive The drivetrain subsystem\n";
    for (const auto& subsystem : path.required_subsystems) {
        std::string param = subsystem;
        param[0] = std::tolower(param[0]);
        ss << "     * @param " << param << " The " << subsystem << "\n";
    }
    ss << "     * @return Complete autonomous command\n";
    ss << "     */\n";

    ss << "    public static Command getCommand(\n";
    ss << "            IDrivetrain drive";
    for (const auto& subsystem : path.required_subsystems) {
        std::string param = subsystem;
        param[0] = std::tolower(param[0]);
        ss << ",\n            " << subsystem << " " << param;
    }
    ss << ") {\n\n";

    // Build command sequence
    ss << "        // Build event map\n";
    ss << "        var eventMap = AutoEventMap_" << path.name << ".getEventMap(";
    bool first = true;
    for (const auto& subsystem : path.required_subsystems) {
        if (!first) ss << ", ";
        std::string param = subsystem;
        param[0] = std::tolower(param[0]);
        ss << param;
        first = false;
    }
    ss << ");\n\n";

    ss << "        // Path following command\n";
    ss << "        Command pathCommand = " << path.name << "Path.getCommand(drive);\n\n";

    // Add event markers
    if (!path.event_markers.empty()) {
        ss << "        // Add event markers to path\n";
        ss << "        Command withEvents = Commands.deadline(\n";
        ss << "            pathCommand";

        for (const auto& marker : path.event_markers) {
            ss << ",\n            ";
            if (marker.trigger == TriggerCondition::AT_TIME) {
                ss << "Commands.waitSeconds(" << marker.time << ").andThen(eventMap.get(\"" << marker.name << "\"))";
            } else if (marker.trigger == TriggerCondition::AT_WAYPOINT) {
                // Approximate time based on waypoint index
                double approx_time = path.total_time * marker.waypoint_index / (path.waypoints.size() - 1);
                ss << "Commands.waitSeconds(" << approx_time << ").andThen(eventMap.get(\"" << marker.name << "\"))";
            } else {
                ss << "eventMap.get(\"" << marker.name << "\")";
            }
        }
        ss << "\n        );\n\n";
    } else {
        ss << "        Command withEvents = pathCommand;\n\n";
    }

    ss << "        // Complete sequence\n";
    ss << "        return Commands.sequence(\n";
    ss << "            // Log start\n";
    ss << "            Commands.print(\"[Auto] Starting " << path.name << "\"),\n\n";
    ss << "            // Run path with events\n";
    ss << "            withEvents,\n\n";
    ss << "            // Ensure stopped at end\n";
    ss << "            Commands.runOnce(drive::stop, drive),\n\n";
    ss << "            // Log complete\n";
    ss << "            Commands.print(\"[Auto] Completed " << path.name << "\")\n";
    ss << "        ).withName(\"AutoRoutine_" << path.name << "\")\n";
    ss << "         .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);\n";
    ss << "    }\n";
    ss << "}\n";

    return ss.str();
}

// ============================================================================
// Scaffolding Generation
// ============================================================================

GeneratedFile PathPlanner::generate_field_constants(CodeFormat format) const {
    GeneratedFile file;
    file.path = "frc/robot/auto/FieldConstants.java";
    file.description = "Field constants with alliance mirroring helpers";

    if (format != CodeFormat::WPILIB_JAVA) {
        file.content = "// Only Java supported\n";
        return file;
    }

    std::ostringstream ss;

    ss << "// ============================================================================\n";
    ss << "// Field Constants with Alliance Mirroring - FRC 2026\n";
    ss << "// ============================================================================\n\n";

    ss << "package frc.robot.auto;\n\n";

    ss << "import edu.wpi.first.math.geometry.Pose2d;\n";
    ss << "import edu.wpi.first.math.geometry.Rotation2d;\n";
    ss << "import edu.wpi.first.math.geometry.Translation2d;\n\n";

    ss << "/**\n";
    ss << " * Field constants for FRC 2026.\n";
    ss << " * Provides field dimensions and alliance mirroring utilities.\n";
    ss << " */\n";
    ss << "public final class FieldConstants {\n\n";

    ss << "    // Field dimensions (meters)\n";
    ss << "    public static final double FIELD_LENGTH = " << FIELD_LENGTH << ";\n";
    ss << "    public static final double FIELD_WIDTH = " << FIELD_WIDTH << ";\n\n";

    ss << "    // Prevent instantiation\n";
    ss << "    private FieldConstants() {}\n\n";

    ss << "    /**\n";
    ss << "     * Mirror a Translation2d for the red alliance.\n";
    ss << "     * Flips X coordinate across field center.\n";
    ss << "     */\n";
    ss << "    public static Translation2d mirror(Translation2d translation) {\n";
    ss << "        return new Translation2d(\n";
    ss << "            FIELD_LENGTH - translation.getX(),\n";
    ss << "            translation.getY()\n";
    ss << "        );\n";
    ss << "    }\n\n";

    ss << "    /**\n";
    ss << "     * Mirror a Rotation2d for the red alliance.\n";
    ss << "     * Flips heading across field center line.\n";
    ss << "     */\n";
    ss << "    public static Rotation2d mirror(Rotation2d rotation) {\n";
    ss << "        return new Rotation2d(Math.PI - rotation.getRadians());\n";
    ss << "    }\n\n";

    ss << "    /**\n";
    ss << "     * Mirror a Pose2d for the red alliance.\n";
    ss << "     * Flips both position and heading.\n";
    ss << "     */\n";
    ss << "    public static Pose2d mirror(Pose2d pose) {\n";
    ss << "        return new Pose2d(\n";
    ss << "            mirror(pose.getTranslation()),\n";
    ss << "            mirror(pose.getRotation())\n";
    ss << "        );\n";
    ss << "    }\n";
    ss << "}\n";

    file.content = ss.str();
    return file;
}

GeneratedFile PathPlanner::generate_drivetrain_interface(CodeFormat format) const {
    GeneratedFile file;
    file.path = "frc/robot/subsystems/IDrivetrain.java";
    file.description = "Drivetrain interface for path following";

    if (format != CodeFormat::WPILIB_JAVA) {
        file.content = "// Only Java supported\n";
        return file;
    }

    std::ostringstream ss;

    ss << "// ============================================================================\n";
    ss << "// Drivetrain Interface - FRC 2026\n";
    ss << "// Implement this interface in your swerve drive subsystem\n";
    ss << "// ============================================================================\n\n";

    ss << "package frc.robot.subsystems;\n\n";

    ss << "import edu.wpi.first.math.geometry.Pose2d;\n";
    ss << "import edu.wpi.first.math.kinematics.ChassisSpeeds;\n";
    ss << "import edu.wpi.first.math.kinematics.SwerveDriveKinematics;\n";
    ss << "import edu.wpi.first.wpilibj2.command.Subsystem;\n\n";

    ss << "/**\n";
    ss << " * Interface for drivetrain subsystems.\n";
    ss << " * Implement this in your swerve drive class for path following.\n";
    ss << " *\n";
    ss << " * Example:\n";
    ss << " * public class SwerveDrive extends SubsystemBase implements IDrivetrain {\n";
    ss << " *     // ... implementation\n";
    ss << " * }\n";
    ss << " */\n";
    ss << "public interface IDrivetrain extends Subsystem {\n\n";

    ss << "    /**\n";
    ss << "     * Get the current robot pose from odometry.\n";
    ss << "     * @return Current pose (x, y in meters, rotation in radians)\n";
    ss << "     */\n";
    ss << "    Pose2d getPose();\n\n";

    ss << "    /**\n";
    ss << "     * Reset odometry to a specific pose.\n";
    ss << "     * @param pose The pose to reset to\n";
    ss << "     */\n";
    ss << "    void resetOdometry(Pose2d pose);\n\n";

    ss << "    /**\n";
    ss << "     * Drive the robot with chassis speeds.\n";
    ss << "     * @param speeds Desired chassis speeds (robot-relative or field-relative)\n";
    ss << "     */\n";
    ss << "    void drive(ChassisSpeeds speeds);\n\n";

    ss << "    /**\n";
    ss << "     * Stop the drivetrain.\n";
    ss << "     */\n";
    ss << "    void stop();\n\n";

    ss << "    /**\n";
    ss << "     * Get the swerve drive kinematics.\n";
    ss << "     * @return Kinematics object for trajectory generation\n";
    ss << "     */\n";
    ss << "    SwerveDriveKinematics getKinematics();\n";
    ss << "}\n";

    file.content = ss.str();
    return file;
}

GeneratedFile PathPlanner::generate_event_map(const AutoPath& path, CodeFormat format) const {
    GeneratedFile file;
    file.path = "frc/robot/auto/AutoEventMap_" + path.name + ".java";
    file.description = "Event map for " + path.name + " markers";

    if (format != CodeFormat::WPILIB_JAVA) {
        file.content = "// Only Java supported\n";
        return file;
    }

    std::ostringstream ss;

    ss << "// ============================================================================\n";
    ss << "// Event Map for " << path.name << "\n";
    ss << "// Maps marker names to commands\n";
    ss << "// ============================================================================\n\n";

    ss << "package frc.robot.auto;\n\n";

    ss << "import edu.wpi.first.wpilibj2.command.Command;\n";
    ss << "import edu.wpi.first.wpilibj2.command.Commands;\n";
    ss << "import java.util.HashMap;\n";
    ss << "import java.util.Map;\n";

    for (const auto& subsystem : path.required_subsystems) {
        ss << "import frc.robot.subsystems." << subsystem << ";\n";
    }
    for (const auto& cmd : path.required_commands) {
        ss << "import frc.robot.commands." << cmd << ";\n";
    }
    ss << "\n";

    ss << "public class AutoEventMap_" << path.name << " {\n\n";

    ss << "    public static Map<String, Command> getEventMap(";
    bool first = true;
    for (const auto& subsystem : path.required_subsystems) {
        if (!first) ss << ", ";
        std::string param = subsystem;
        param[0] = std::tolower(param[0]);
        ss << subsystem << " " << param;
        first = false;
    }
    ss << ") {\n";

    ss << "        Map<String, Command> eventMap = new HashMap<>();\n\n";

    for (const auto& marker : path.event_markers) {
        ss << "        eventMap.put(\"" << marker.name << "\", ";
        std::string cmdName = action_to_command_name(marker.action);
        std::string subsystem = action_to_subsystem(marker.action);
        if (!subsystem.empty()) {
            std::string param = subsystem;
            param[0] = std::tolower(param[0]);
            ss << "new " << cmdName << "(" << param << ")";
        } else {
            ss << "Commands.none()";
        }
        ss << ");\n";
    }

    ss << "\n        return eventMap;\n";
    ss << "    }\n";
    ss << "}\n";

    file.content = ss.str();
    return file;
}

GeneratedFile PathPlanner::generate_subsystem_scaffold(const std::string& name, CodeFormat format) const {
    GeneratedFile file;
    file.path = "frc/robot/subsystems/" + name + ".java";
    file.description = "Subsystem scaffold for " + name;

    if (format != CodeFormat::WPILIB_JAVA) {
        file.content = "// Only Java supported\n";
        return file;
    }

    std::ostringstream ss;

    ss << "// ============================================================================\n";
    ss << "// " << name << " Subsystem Scaffold\n";
    ss << "// TODO: Fill in hardware initialization and control logic\n";
    ss << "// ============================================================================\n\n";

    ss << "package frc.robot.subsystems;\n\n";

    ss << "import edu.wpi.first.wpilibj2.command.SubsystemBase;\n\n";

    ss << "/**\n";
    ss << " * " << name << " subsystem.\n";
    ss << " *\n";
    ss << " * TODO: Add motor controllers, sensors, and control logic.\n";
    ss << " */\n";
    ss << "public class " << name << " extends SubsystemBase {\n\n";

    ss << "    // TODO: Declare motor controllers and sensors\n";
    ss << "    // Example:\n";
    ss << "    // private final TalonFX motor = new TalonFX(CAN_ID);\n\n";

    ss << "    public " << name << "() {\n";
    ss << "        // TODO: Initialize hardware\n";
    ss << "    }\n\n";

    ss << "    @Override\n";
    ss << "    public void periodic() {\n";
    ss << "        // TODO: Add telemetry, state updates\n";
    ss << "    }\n\n";

    // Add specific methods based on subsystem type
    if (name == "IntakeSubsystem") {
        ss << "    public void runIntake(double speed) {\n";
        ss << "        // TODO: Implement intake control\n";
        ss << "    }\n\n";
        ss << "    public void stop() {\n";
        ss << "        // TODO: Stop intake\n";
        ss << "    }\n\n";
        ss << "    public boolean hasGamePiece() {\n";
        ss << "        // TODO: Check sensor for game piece\n";
        ss << "        return false;\n";
        ss << "    }\n";
    } else if (name == "ShooterSubsystem") {
        ss << "    public void setShooterSpeed(double rpm) {\n";
        ss << "        // TODO: Implement shooter velocity control\n";
        ss << "    }\n\n";
        ss << "    public boolean atTargetSpeed() {\n";
        ss << "        // TODO: Check if at target RPM\n";
        ss << "        return false;\n";
        ss << "    }\n\n";
        ss << "    public void stop() {\n";
        ss << "        // TODO: Stop shooter\n";
        ss << "    }\n";
    } else if (name == "ElevatorSubsystem") {
        ss << "    public void setHeight(double meters) {\n";
        ss << "        // TODO: Implement position control\n";
        ss << "    }\n\n";
        ss << "    public double getHeight() {\n";
        ss << "        // TODO: Return current height\n";
        ss << "        return 0.0;\n";
        ss << "    }\n\n";
        ss << "    public boolean atTargetHeight() {\n";
        ss << "        // TODO: Check if at target\n";
        ss << "        return false;\n";
        ss << "    }\n";
    } else if (name == "VisionSubsystem") {
        ss << "    public boolean canSeeTag(int tagId) {\n";
        ss << "        // TODO: Check if tag is visible\n";
        ss << "        return false;\n";
        ss << "    }\n\n";
        ss << "    public double getDistanceToTag(int tagId) {\n";
        ss << "        // TODO: Return distance to tag\n";
        ss << "        return Double.MAX_VALUE;\n";
        ss << "    }\n";
    }

    ss << "}\n";

    file.content = ss.str();
    return file;
}

GeneratedFile PathPlanner::generate_command_scaffold(
    const std::string& name,
    const std::string& subsystem,
    CodeFormat format) const {

    GeneratedFile file;
    file.path = "frc/robot/commands/" + name + ".java";
    file.description = "Command scaffold for " + name;

    if (format != CodeFormat::WPILIB_JAVA) {
        file.content = "// Only Java supported\n";
        return file;
    }

    std::ostringstream ss;

    ss << "// ============================================================================\n";
    ss << "// " << name << " Command Scaffold\n";
    ss << "// TODO: Implement command logic\n";
    ss << "// ============================================================================\n\n";

    ss << "package frc.robot.commands;\n\n";

    ss << "import edu.wpi.first.wpilibj2.command.Command;\n";
    if (!subsystem.empty()) {
        ss << "import frc.robot.subsystems." << subsystem << ";\n";
    }
    ss << "\n";

    ss << "/**\n";
    ss << " * " << name << " command.\n";
    ss << " *\n";
    ss << " * TODO: Implement command behavior.\n";
    ss << " */\n";
    ss << "public class " << name << " extends Command {\n\n";

    if (!subsystem.empty()) {
        std::string member = "m_" + subsystem;
        member[2] = std::tolower(member[2]);
        ss << "    private final " << subsystem << " " << member << ";\n\n";

        ss << "    public " << name << "(" << subsystem << " subsystem) {\n";
        ss << "        " << member << " = subsystem;\n";
        ss << "        addRequirements(subsystem);\n";
        ss << "    }\n\n";
    } else {
        ss << "    public " << name << "() {\n";
        ss << "        // No subsystem required\n";
        ss << "    }\n\n";
    }

    ss << "    @Override\n";
    ss << "    public void initialize() {\n";
    ss << "        // TODO: Setup command\n";
    ss << "    }\n\n";

    ss << "    @Override\n";
    ss << "    public void execute() {\n";
    ss << "        // TODO: Run command logic\n";
    ss << "    }\n\n";

    ss << "    @Override\n";
    ss << "    public void end(boolean interrupted) {\n";
    ss << "        // TODO: Cleanup\n";
    ss << "    }\n\n";

    ss << "    @Override\n";
    ss << "    public boolean isFinished() {\n";
    ss << "        // TODO: Return true when done\n";
    ss << "        return true;\n";
    ss << "    }\n";
    ss << "}\n";

    file.content = ss.str();
    return file;
}

// ============================================================================
// Legacy Code Generation (backward compatibility)
// ============================================================================

std::string PathPlanner::generate_java_path(const AutoPath& path) const {
    return generate_java_trajectory_class(path);
}

std::string PathPlanner::generate_java_action(const PathAction& action) const {
    std::ostringstream ss;

    if (action.wait_for_completion) {
        ss << "new " << action.command_name << "()";
    } else {
        ss << "Commands.runOnce(() -> { /* " << action.command_name << " */ })";
    }

    return ss.str();
}

std::string PathPlanner::generate_cpp_path(const AutoPath& path) const {
    std::ostringstream ss;
    ss << "// C++ code generation - simplified version\n";
    ss << "// Full C++ support coming soon\n\n";
    ss << "#pragma once\n";
    ss << "// Path: " << path.name << "\n";
    ss << "// Waypoints: " << path.waypoints.size() << "\n";
    return ss.str();
}

std::string PathPlanner::generate_cpp_action(const PathAction& action) const {
    return "/* " + action.command_name + " */";
}

std::string PathPlanner::generate_python_path(const AutoPath& path) const {
    std::ostringstream ss;
    ss << "# Python code generation - simplified version\n";
    ss << "# Path: " << path.name << "\n";
    ss << "# Waypoints: " << path.waypoints.size() << "\n";
    return ss.str();
}

std::string PathPlanner::generate_python_action(const PathAction& action) const {
    return "# " + action.command_name;
}

std::string PathPlanner::generate_pathplanner_json(const AutoPath& path) const {
    json j;

    j["version"] = "2025.0";
    j["name"] = path.name;

    j["globalConstraints"] = {
        {"maxVelocity", path.max_velocity},
        {"maxAcceleration", path.max_acceleration},
        {"maxAngularVelocity", path.max_angular_velocity},
        {"maxAngularAcceleration", path.max_angular_accel}
    };

    j["waypoints"] = json::array();
    for (const auto& wp : path.waypoints) {
        json waypoint;
        waypoint["anchor"] = {{"x", wp.pose.x}, {"y", wp.pose.y}};
        waypoint["prevControl"] = (wp.control_in_x != 0 || wp.control_in_y != 0) ?
            json({{"x", wp.pose.x + wp.control_in_x}, {"y", wp.pose.y + wp.control_in_y}}) :
            json(nullptr);
        waypoint["nextControl"] = (wp.control_out_x != 0 || wp.control_out_y != 0) ?
            json({{"x", wp.pose.x + wp.control_out_x}, {"y", wp.pose.y + wp.control_out_y}}) :
            json(nullptr);
        waypoint["holonomicAngle"] = wp.pose.theta * 180.0 / M_PI;
        waypoint["isReversal"] = false;
        waypoint["velOverride"] = (wp.velocity > 0) ? json(wp.velocity) : json(nullptr);
        waypoint["name"] = wp.name;

        j["waypoints"].push_back(waypoint);
    }

    j["eventMarkers"] = json::array();
    for (const auto& marker : path.event_markers) {
        json em;
        em["name"] = marker.name;
        em["waypointRelativePos"] = marker.waypoint_index >= 0 ? marker.waypoint_index : 0;
        em["command"] = {
            {"type", "named"},
            {"data", {{"name", action_to_command_name(marker.action)}}}
        };
        j["eventMarkers"].push_back(em);
    }

    return j.dump(2);
}

std::string PathPlanner::generate_auto_routine(const std::vector<std::string>& paths, CodeFormat format) const {
    std::ostringstream ss;

    if (format == CodeFormat::WPILIB_JAVA) {
        ss << "package frc.robot.auto;\n\n";
        ss << "import edu.wpi.first.wpilibj2.command.Command;\n";
        ss << "import edu.wpi.first.wpilibj2.command.Commands;\n";
        ss << "import frc.robot.subsystems.IDrivetrain;\n\n";

        ss << "public class AutoRoutine {\n";
        ss << "    public static Command getCommand(IDrivetrain drive) {\n";
        ss << "        return Commands.sequence(\n";

        for (size_t i = 0; i < paths.size(); i++) {
            ss << "            " << paths[i] << "Path.getCommand(drive)";
            if (i < paths.size() - 1) ss << ",";
            ss << "\n";
        }

        ss << "        );\n";
        ss << "    }\n";
        ss << "}\n";
    }

    return ss.str();
}

// ============================================================================
// Path I/O
// ============================================================================

bool PathPlanner::save_path(const AutoPath& path, const std::string& filename) const {
    std::string json_content = generate_pathplanner_json(path);

    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }

    file << json_content;
    return true;
}

bool PathPlanner::load_path(const std::string& filename, AutoPath& path) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }

    try {
        json j = json::parse(file);

        path.name = j.value("name", "Unnamed");

        if (j.contains("globalConstraints")) {
            auto& gc = j["globalConstraints"];
            path.max_velocity = gc.value("maxVelocity", 4.0);
            path.max_acceleration = gc.value("maxAcceleration", 3.0);
            path.max_angular_velocity = gc.value("maxAngularVelocity", 4.0);
            path.max_angular_accel = gc.value("maxAngularAcceleration", 6.0);
        }

        if (j.contains("waypoints")) {
            for (const auto& wp : j["waypoints"]) {
                PathWaypoint waypoint;

                if (wp.contains("anchor")) {
                    waypoint.pose.x = wp["anchor"].value("x", 0.0);
                    waypoint.pose.y = wp["anchor"].value("y", 0.0);
                }

                if (wp.contains("holonomicAngle")) {
                    waypoint.pose.theta = wp["holonomicAngle"].get<double>() * M_PI / 180.0;
                }

                if (wp.contains("prevControl") && !wp["prevControl"].is_null()) {
                    waypoint.control_in_x = wp["prevControl"].value("x", 0.0) - waypoint.pose.x;
                    waypoint.control_in_y = wp["prevControl"].value("y", 0.0) - waypoint.pose.y;
                }

                if (wp.contains("nextControl") && !wp["nextControl"].is_null()) {
                    waypoint.control_out_x = wp["nextControl"].value("x", 0.0) - waypoint.pose.x;
                    waypoint.control_out_y = wp["nextControl"].value("y", 0.0) - waypoint.pose.y;
                }

                if (wp.contains("velOverride") && !wp["velOverride"].is_null()) {
                    waypoint.velocity = wp["velOverride"].get<double>();
                }

                waypoint.name = wp.value("name", "");

                path.waypoints.push_back(waypoint);
            }
        }

        return true;
    } catch (const std::exception&) {
        return false;
    }
}

} // namespace sim
} // namespace frc_vision
