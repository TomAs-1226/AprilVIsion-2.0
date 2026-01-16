/**
 * @file path_planner.cpp
 * @brief PathPlanner-style code generator implementation
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

void PathPlanner::add_action(AutoPath& path, const PathAction& action) {
    path.actions.push_back(action);
}

// ============================================================================
// Trajectory Generation
// ============================================================================

Pose2D PathPlanner::bezier_point(const PathWaypoint& p0, const PathWaypoint& p1, double t) const {
    // Cubic Bezier curve: B(t) = (1-t)^3*P0 + 3*(1-t)^2*t*C0 + 3*(1-t)*t^2*C1 + t^3*P1
    double t2 = t * t;
    double t3 = t2 * t;
    double mt = 1.0 - t;
    double mt2 = mt * mt;
    double mt3 = mt2 * mt;

    // Control points
    double c0x = p0.pose.x + p0.control_out_x;
    double c0y = p0.pose.y + p0.control_out_y;
    double c1x = p1.pose.x + p1.control_in_x;
    double c1y = p1.pose.y + p1.control_in_y;

    Pose2D result;
    result.x = mt3 * p0.pose.x + 3 * mt2 * t * c0x + 3 * mt * t2 * c1x + t3 * p1.pose.x;
    result.y = mt3 * p0.pose.y + 3 * mt2 * t * c0y + 3 * mt * t2 * c1y + t3 * p1.pose.y;

    // Interpolate rotation based on mode
    if (p0.rotation_mode == RotationMode::LINEAR || p0.rotation_mode == RotationMode::HOLONOMIC) {
        // Shortest path interpolation
        double diff = p1.pose.theta - p0.pose.theta;
        while (diff > M_PI) diff -= 2 * M_PI;
        while (diff < -M_PI) diff += 2 * M_PI;
        result.theta = p0.pose.theta + t * diff;
    } else {
        result.theta = p0.pose.theta;
    }

    return result;
}

double PathPlanner::bezier_curvature(const PathWaypoint& p0, const PathWaypoint& p1, double t) const {
    // First derivative
    double mt = 1.0 - t;

    double c0x = p0.pose.x + p0.control_out_x;
    double c0y = p0.pose.y + p0.control_out_y;
    double c1x = p1.pose.x + p1.control_in_x;
    double c1y = p1.pose.y + p1.control_in_y;

    double dx = 3 * mt * mt * (c0x - p0.pose.x) + 6 * mt * t * (c1x - c0x) + 3 * t * t * (p1.pose.x - c1x);
    double dy = 3 * mt * mt * (c0y - p0.pose.y) + 6 * mt * t * (c1y - c0y) + 3 * t * t * (p1.pose.y - c1y);

    // Second derivative
    double ddx = 6 * mt * (c1x - 2 * c0x + p0.pose.x) + 6 * t * (p1.pose.x - 2 * c1x + c0x);
    double ddy = 6 * mt * (c1y - 2 * c0y + p0.pose.y) + 6 * t * (p1.pose.y - 2 * c1y + c0y);

    // Curvature = |dx * ddy - dy * ddx| / (dx^2 + dy^2)^(3/2)
    double denom = std::pow(dx * dx + dy * dy, 1.5);
    if (denom < 1e-6) return 0;

    return std::abs(dx * ddy - dy * ddx) / denom;
}

double PathPlanner::calculate_path_length(const AutoPath& path) const {
    if (path.waypoints.size() < 2) return 0;

    double length = 0;
    for (size_t i = 0; i < path.waypoints.size() - 1; i++) {
        // Approximate arc length with numerical integration
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

    // Generate velocity profile using trapezoidal motion
    double max_vel = path.max_velocity;
    double max_accel = path.max_acceleration;

    // Calculate times for each segment
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

    // Simple trapezoidal profile for total path
    double accel_dist = (max_vel * max_vel) / (2 * max_accel);
    double cruise_dist = std::max(0.0, path.total_distance - 2 * accel_dist);

    double accel_time = max_vel / max_accel;
    double cruise_time = cruise_dist / max_vel;
    path.total_time = 2 * accel_time + cruise_time;

    // Generate trajectory points
    double time = 0;
    double accumulated_dist = 0;
    size_t current_segment = 0;
    double segment_progress = 0;

    while (time <= path.total_time + dt && current_segment < path.waypoints.size() - 1) {
        AutoPath::TrajectoryPoint pt;
        pt.time = time;

        // Calculate velocity and acceleration at this time
        double velocity, acceleration;
        if (time < accel_time) {
            // Acceleration phase
            velocity = max_accel * time;
            acceleration = max_accel;
        } else if (time < accel_time + cruise_time) {
            // Cruise phase
            velocity = max_vel;
            acceleration = 0;
        } else {
            // Deceleration phase
            double decel_time = time - accel_time - cruise_time;
            velocity = max_vel - max_accel * decel_time;
            acceleration = -max_accel;
        }
        velocity = std::max(0.0, velocity);

        pt.velocity = velocity;
        pt.acceleration = acceleration;

        // Find position on path
        double dist_at_time;
        if (time < accel_time) {
            dist_at_time = 0.5 * max_accel * time * time;
        } else if (time < accel_time + cruise_time) {
            dist_at_time = accel_dist + max_vel * (time - accel_time);
        } else {
            double decel_time = time - accel_time - cruise_time;
            dist_at_time = accel_dist + cruise_dist + max_vel * decel_time - 0.5 * max_accel * decel_time * decel_time;
        }

        // Map distance to path parameter
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

        // Angular velocity from heading change
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

    // Find trajectory points surrounding t
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
// Code Generation
// ============================================================================

std::string PathPlanner::generate_code(const AutoPath& path, CodeFormat format) const {
    switch (format) {
        case CodeFormat::WPILIB_JAVA:
            return generate_java_path(path);
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

std::string PathPlanner::generate_java_path(const AutoPath& path) const {
    std::ostringstream ss;

    ss << "// Auto-generated path: " << path.name << "\n";
    ss << "// Generated by FRC Vision PathPlanner\n\n";

    ss << "package frc.robot.auto;\n\n";

    ss << "import edu.wpi.first.math.geometry.Pose2d;\n";
    ss << "import edu.wpi.first.math.geometry.Rotation2d;\n";
    ss << "import edu.wpi.first.math.geometry.Translation2d;\n";
    ss << "import edu.wpi.first.math.trajectory.Trajectory;\n";
    ss << "import edu.wpi.first.math.trajectory.TrajectoryConfig;\n";
    ss << "import edu.wpi.first.math.trajectory.TrajectoryGenerator;\n";
    ss << "import edu.wpi.first.wpilibj2.command.Command;\n";
    ss << "import edu.wpi.first.wpilibj2.command.Commands;\n";
    ss << "import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;\n";
    ss << "import java.util.List;\n\n";

    ss << "public class " << path.name << "Path {\n\n";

    // Trajectory config
    ss << "    private static final TrajectoryConfig config = new TrajectoryConfig(\n";
    ss << "        " << std::fixed << std::setprecision(2) << path.max_velocity << ", // max velocity m/s\n";
    ss << "        " << path.max_acceleration << "  // max acceleration m/s^2\n";
    ss << "    );\n\n";

    // Waypoints
    ss << "    public static final List<Translation2d> interiorWaypoints = List.of(\n";
    for (size_t i = 1; i < path.waypoints.size() - 1; i++) {
        ss << "        new Translation2d(" << path.waypoints[i].pose.x << ", "
           << path.waypoints[i].pose.y << ")";
        if (i < path.waypoints.size() - 2) ss << ",";
        ss << "\n";
    }
    ss << "    );\n\n";

    // Start and end poses
    if (!path.waypoints.empty()) {
        ss << "    public static final Pose2d startPose = new Pose2d(\n";
        ss << "        " << path.waypoints.front().pose.x << ",\n";
        ss << "        " << path.waypoints.front().pose.y << ",\n";
        ss << "        Rotation2d.fromRadians(" << path.waypoints.front().pose.theta << ")\n";
        ss << "    );\n\n";

        ss << "    public static final Pose2d endPose = new Pose2d(\n";
        ss << "        " << path.waypoints.back().pose.x << ",\n";
        ss << "        " << path.waypoints.back().pose.y << ",\n";
        ss << "        Rotation2d.fromRadians(" << path.waypoints.back().pose.theta << ")\n";
        ss << "    );\n\n";
    }

    // Generate trajectory
    ss << "    public static Trajectory getTrajectory() {\n";
    ss << "        return TrajectoryGenerator.generateTrajectory(\n";
    ss << "            startPose,\n";
    ss << "            interiorWaypoints,\n";
    ss << "            endPose,\n";
    ss << "            config\n";
    ss << "        );\n";
    ss << "    }\n\n";

    // Generate command
    ss << "    public static Command getCommand(SwerveDrive drive) {\n";
    ss << "        var trajectory = getTrajectory();\n\n";

    ss << "        var thetaController = new ProfiledPIDController(\n";
    ss << "            " << swerve_config_.rotation_kP << ",\n";
    ss << "            " << swerve_config_.rotation_kI << ",\n";
    ss << "            " << swerve_config_.rotation_kD << ",\n";
    ss << "            new TrapezoidProfile.Constraints(" << path.max_angular_velocity << ", "
       << path.max_angular_accel << ")\n";
    ss << "        );\n";
    ss << "        thetaController.enableContinuousInput(-Math.PI, Math.PI);\n\n";

    // Actions
    ss << "        return Commands.sequence(\n";
    if (path.reset_odometry) {
        ss << "            Commands.runOnce(() -> drive.resetOdometry(startPose)),\n";
    }

    ss << "            new SwerveControllerCommand(\n";
    ss << "                trajectory,\n";
    ss << "                drive::getPose,\n";
    ss << "                drive.getKinematics(),\n";
    ss << "                new PIDController(" << swerve_config_.translation_kP << ", "
       << swerve_config_.translation_kI << ", " << swerve_config_.translation_kD << "),\n";
    ss << "                new PIDController(" << swerve_config_.translation_kP << ", "
       << swerve_config_.translation_kI << ", " << swerve_config_.translation_kD << "),\n";
    ss << "                thetaController,\n";
    ss << "                drive::setModuleStates,\n";
    ss << "                drive\n";
    ss << "            )";

    // Add actions
    for (const auto& action : path.actions) {
        ss << ",\n            " << generate_java_action(action);
    }

    ss << "\n        );\n";
    ss << "    }\n";
    ss << "}\n";

    return ss.str();
}

std::string PathPlanner::generate_java_action(const PathAction& action) const {
    std::ostringstream ss;

    if (action.wait_for_completion) {
        ss << "Commands.runOnce(() -> " << action.command_name << "())";
    } else {
        ss << "Commands.parallel(\n";
        ss << "                Commands.runOnce(() -> " << action.command_name << "())";

        if (action.trigger == TriggerCondition::TAG_VISIBLE) {
            ss << ",\n                Commands.waitUntil(() -> vision.canSeeTag(" << action.tag_id << "))";
        } else if (action.trigger == TriggerCondition::DISTANCE_TO_TAG) {
            ss << ",\n                Commands.waitUntil(() -> vision.getDistanceToTag("
               << action.tag_id << ") < " << action.distance << ")";
        }

        ss << "\n            )";
    }

    return ss.str();
}

std::string PathPlanner::generate_cpp_path(const AutoPath& path) const {
    std::ostringstream ss;

    ss << "// Auto-generated path: " << path.name << "\n";
    ss << "// Generated by FRC Vision PathPlanner\n\n";

    ss << "#pragma once\n\n";
    ss << "#include <frc/geometry/Pose2d.h>\n";
    ss << "#include <frc/geometry/Translation2d.h>\n";
    ss << "#include <frc/trajectory/Trajectory.h>\n";
    ss << "#include <frc/trajectory/TrajectoryConfig.h>\n";
    ss << "#include <frc/trajectory/TrajectoryGenerator.h>\n";
    ss << "#include <frc2/command/Commands.h>\n";
    ss << "#include <frc2/command/SwerveControllerCommand.h>\n";
    ss << "#include <units/velocity.h>\n";
    ss << "#include <units/acceleration.h>\n";
    ss << "#include <vector>\n\n";

    ss << "namespace auto_paths {\n\n";

    ss << "class " << path.name << "Path {\n";
    ss << "public:\n";

    // Config
    ss << "    static frc::TrajectoryConfig GetConfig() {\n";
    ss << "        return frc::TrajectoryConfig(\n";
    ss << "            " << path.max_velocity << "_mps,\n";
    ss << "            " << path.max_acceleration << "_mps_sq\n";
    ss << "        );\n";
    ss << "    }\n\n";

    // Start pose
    if (!path.waypoints.empty()) {
        ss << "    static frc::Pose2d GetStartPose() {\n";
        ss << "        return frc::Pose2d{\n";
        ss << "            " << path.waypoints.front().pose.x << "_m,\n";
        ss << "            " << path.waypoints.front().pose.y << "_m,\n";
        ss << "            frc::Rotation2d{units::radian_t{" << path.waypoints.front().pose.theta << "}}\n";
        ss << "        };\n";
        ss << "    }\n\n";

        ss << "    static frc::Pose2d GetEndPose() {\n";
        ss << "        return frc::Pose2d{\n";
        ss << "            " << path.waypoints.back().pose.x << "_m,\n";
        ss << "            " << path.waypoints.back().pose.y << "_m,\n";
        ss << "            frc::Rotation2d{units::radian_t{" << path.waypoints.back().pose.theta << "}}\n";
        ss << "        };\n";
        ss << "    }\n\n";
    }

    // Interior waypoints
    ss << "    static std::vector<frc::Translation2d> GetInteriorWaypoints() {\n";
    ss << "        return {\n";
    for (size_t i = 1; i < path.waypoints.size() - 1; i++) {
        ss << "            frc::Translation2d{" << path.waypoints[i].pose.x << "_m, "
           << path.waypoints[i].pose.y << "_m}";
        if (i < path.waypoints.size() - 2) ss << ",";
        ss << "\n";
    }
    ss << "        };\n";
    ss << "    }\n\n";

    // Generate trajectory
    ss << "    static frc::Trajectory GetTrajectory() {\n";
    ss << "        return frc::TrajectoryGenerator::GenerateTrajectory(\n";
    ss << "            GetStartPose(),\n";
    ss << "            GetInteriorWaypoints(),\n";
    ss << "            GetEndPose(),\n";
    ss << "            GetConfig()\n";
    ss << "        );\n";
    ss << "    }\n\n";

    // Command generation
    ss << "    template<typename DriveSubsystem>\n";
    ss << "    static frc2::CommandPtr GetCommand(DriveSubsystem* drive) {\n";
    ss << "        auto trajectory = GetTrajectory();\n\n";

    ss << "        frc::ProfiledPIDController<units::radians> thetaController{\n";
    ss << "            " << swerve_config_.rotation_kP << ",\n";
    ss << "            " << swerve_config_.rotation_kI << ",\n";
    ss << "            " << swerve_config_.rotation_kD << ",\n";
    ss << "            frc::TrapezoidProfile<units::radians>::Constraints{\n";
    ss << "                " << path.max_angular_velocity << "_rad_per_s,\n";
    ss << "                " << path.max_angular_accel << "_rad_per_s_sq\n";
    ss << "            }\n";
    ss << "        };\n";
    ss << "        thetaController.EnableContinuousInput(-std::numbers::pi * 1_rad, std::numbers::pi * 1_rad);\n\n";

    ss << "        return frc2::cmd::Sequence(\n";
    if (path.reset_odometry) {
        ss << "            frc2::cmd::RunOnce([drive] { drive->ResetOdometry(GetStartPose()); }),\n";
    }
    ss << "            frc2::SwerveControllerCommand<4>(\n";
    ss << "                trajectory,\n";
    ss << "                [drive] { return drive->GetPose(); },\n";
    ss << "                drive->GetKinematics(),\n";
    ss << "                frc::PIDController{" << swerve_config_.translation_kP << ", "
       << swerve_config_.translation_kI << ", " << swerve_config_.translation_kD << "},\n";
    ss << "                frc::PIDController{" << swerve_config_.translation_kP << ", "
       << swerve_config_.translation_kI << ", " << swerve_config_.translation_kD << "},\n";
    ss << "                thetaController,\n";
    ss << "                [drive](auto states) { drive->SetModuleStates(states); },\n";
    ss << "                {drive}\n";
    ss << "            ).ToPtr()\n";
    ss << "        );\n";
    ss << "    }\n";
    ss << "};\n\n";

    ss << "} // namespace auto_paths\n";

    return ss.str();
}

std::string PathPlanner::generate_python_path(const AutoPath& path) const {
    std::ostringstream ss;

    ss << "# Auto-generated path: " << path.name << "\n";
    ss << "# Generated by FRC Vision PathPlanner\n\n";

    ss << "from wpimath.geometry import Pose2d, Rotation2d, Translation2d\n";
    ss << "from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator\n";
    ss << "from commands2 import Commands, SwerveControllerCommand\n";
    ss << "from wpimath.controller import PIDController, ProfiledPIDController\n";
    ss << "from wpimath.trajectory import TrapezoidProfile\n";
    ss << "import math\n\n";

    ss << "class " << path.name << "Path:\n";

    // Config
    ss << "    MAX_VELOCITY = " << path.max_velocity << "  # m/s\n";
    ss << "    MAX_ACCELERATION = " << path.max_acceleration << "  # m/s^2\n";
    ss << "    MAX_ANGULAR_VELOCITY = " << path.max_angular_velocity << "  # rad/s\n";
    ss << "    MAX_ANGULAR_ACCEL = " << path.max_angular_accel << "  # rad/s^2\n\n";

    // Start pose
    if (!path.waypoints.empty()) {
        ss << "    START_POSE = Pose2d(\n";
        ss << "        " << path.waypoints.front().pose.x << ",\n";
        ss << "        " << path.waypoints.front().pose.y << ",\n";
        ss << "        Rotation2d(" << path.waypoints.front().pose.theta << ")\n";
        ss << "    )\n\n";

        ss << "    END_POSE = Pose2d(\n";
        ss << "        " << path.waypoints.back().pose.x << ",\n";
        ss << "        " << path.waypoints.back().pose.y << ",\n";
        ss << "        Rotation2d(" << path.waypoints.back().pose.theta << ")\n";
        ss << "    )\n\n";
    }

    // Interior waypoints
    ss << "    INTERIOR_WAYPOINTS = [\n";
    for (size_t i = 1; i < path.waypoints.size() - 1; i++) {
        ss << "        Translation2d(" << path.waypoints[i].pose.x << ", "
           << path.waypoints[i].pose.y << "),\n";
    }
    ss << "    ]\n\n";

    // Get trajectory
    ss << "    @classmethod\n";
    ss << "    def get_trajectory(cls):\n";
    ss << "        config = TrajectoryConfig(cls.MAX_VELOCITY, cls.MAX_ACCELERATION)\n";
    ss << "        return TrajectoryGenerator.generateTrajectory(\n";
    ss << "            cls.START_POSE,\n";
    ss << "            cls.INTERIOR_WAYPOINTS,\n";
    ss << "            cls.END_POSE,\n";
    ss << "            config\n";
    ss << "        )\n\n";

    // Get command
    ss << "    @classmethod\n";
    ss << "    def get_command(cls, drive):\n";
    ss << "        trajectory = cls.get_trajectory()\n\n";

    ss << "        theta_controller = ProfiledPIDController(\n";
    ss << "            " << swerve_config_.rotation_kP << ",\n";
    ss << "            " << swerve_config_.rotation_kI << ",\n";
    ss << "            " << swerve_config_.rotation_kD << ",\n";
    ss << "            TrapezoidProfile.Constraints(cls.MAX_ANGULAR_VELOCITY, cls.MAX_ANGULAR_ACCEL)\n";
    ss << "        )\n";
    ss << "        theta_controller.enableContinuousInput(-math.pi, math.pi)\n\n";

    ss << "        return Commands.sequence(\n";
    if (path.reset_odometry) {
        ss << "            Commands.runOnce(lambda: drive.reset_odometry(cls.START_POSE)),\n";
    }
    ss << "            SwerveControllerCommand(\n";
    ss << "                trajectory,\n";
    ss << "                drive.get_pose,\n";
    ss << "                drive.kinematics,\n";
    ss << "                PIDController(" << swerve_config_.translation_kP << ", "
       << swerve_config_.translation_kI << ", " << swerve_config_.translation_kD << "),\n";
    ss << "                PIDController(" << swerve_config_.translation_kP << ", "
       << swerve_config_.translation_kI << ", " << swerve_config_.translation_kD << "),\n";
    ss << "                theta_controller,\n";
    ss << "                drive.set_module_states,\n";
    ss << "                [drive]\n";
    ss << "            )\n";
    ss << "        )\n";

    return ss.str();
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
    for (const auto& action : path.actions) {
        json marker;
        marker["name"] = action.name;
        marker["command"] = action.command_name;
        marker["waypointIndex"] = action.waypoint_index;
        j["eventMarkers"].push_back(marker);
    }

    return j.dump(2);
}

std::string PathPlanner::generate_auto_routine(const std::vector<std::string>& paths, CodeFormat format) const {
    std::ostringstream ss;

    if (format == CodeFormat::WPILIB_JAVA) {
        ss << "// Auto-generated autonomous routine\n";
        ss << "// Generated by FRC Vision PathPlanner\n\n";

        ss << "package frc.robot.auto;\n\n";
        ss << "import edu.wpi.first.wpilibj2.command.Command;\n";
        ss << "import edu.wpi.first.wpilibj2.command.Commands;\n\n";

        ss << "public class AutoRoutine {\n";
        ss << "    public static Command getCommand(SwerveDrive drive) {\n";
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

        if (j.contains("eventMarkers")) {
            for (const auto& em : j["eventMarkers"]) {
                PathAction action;
                action.name = em.value("name", "");
                action.command_name = em.value("command", "");
                action.waypoint_index = em.value("waypointIndex", 0);
                action.trigger = TriggerCondition::AT_WAYPOINT;
                path.actions.push_back(action);
            }
        }

        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

// ============================================================================
// Preset Path Generators
// ============================================================================

AutoPath generate_score_path(
    const Pose2D& start,
    int target_tag_id,
    const std::string& score_action,
    const FieldLayout& field,
    double standoff_distance) {

    AutoPath path;
    path.name = "ScoreAtTag" + std::to_string(target_tag_id);

    // Start waypoint
    PathWaypoint start_wp;
    start_wp.pose = start;
    start_wp.name = "Start";
    path.waypoints.push_back(start_wp);

    // Get tag position
    if (field.has_tag(target_tag_id)) {
        const auto& tag = field.get_tag(target_tag_id);

        // Calculate approach position
        double tag_x = tag.center_field.x;
        double tag_y = tag.center_field.y;

        // Direction from start to tag
        double dx = tag_x - start.x;
        double dy = tag_y - start.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist > 0.01) {
            // Target position is standoff distance from tag
            double target_x = tag_x - (dx / dist) * standoff_distance;
            double target_y = tag_y - (dy / dist) * standoff_distance;
            double target_theta = std::atan2(dy, dx);

            PathWaypoint score_wp;
            score_wp.pose = {target_x, target_y, target_theta};
            score_wp.velocity = 0; // Stop at scoring position
            score_wp.name = "Score";

            // Add control points for smooth approach
            double ctrl_dist = dist * 0.3;
            score_wp.control_in_x = -std::cos(target_theta) * ctrl_dist;
            score_wp.control_in_y = -std::sin(target_theta) * ctrl_dist;

            path.waypoints.push_back(score_wp);
        }
    }

    // Add scoring action
    PathAction action;
    action.name = score_action;
    action.command_name = score_action;
    action.trigger = TriggerCondition::AT_WAYPOINT;
    action.waypoint_index = 1;
    path.actions.push_back(action);

    return path;
}

AutoPath generate_pickup_score_path(
    const Pose2D& start,
    const Pose2D& pickup_location,
    int score_tag_id,
    const std::string& score_action,
    const FieldLayout& field) {

    AutoPath path;
    path.name = "PickupAndScore";

    // Start
    PathWaypoint start_wp;
    start_wp.pose = start;
    path.waypoints.push_back(start_wp);

    // Pickup location
    PathWaypoint pickup_wp;
    pickup_wp.pose = pickup_location;
    pickup_wp.velocity = 0.5; // Slow for pickup
    pickup_wp.name = "Pickup";
    path.waypoints.push_back(pickup_wp);

    // Pickup action
    PathAction pickup_action;
    pickup_action.name = "pickup";
    pickup_action.command_name = "intakeGamePiece";
    pickup_action.trigger = TriggerCondition::AT_WAYPOINT;
    pickup_action.waypoint_index = 1;
    pickup_action.wait_for_completion = true;
    path.actions.push_back(pickup_action);

    // Score location
    if (field.has_tag(score_tag_id)) {
        const auto& tag = field.get_tag(score_tag_id);

        double dx = tag.center_field.x - pickup_location.x;
        double dy = tag.center_field.y - pickup_location.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist > 0.01) {
            double target_x = tag.center_field.x - (dx / dist) * 1.0;
            double target_y = tag.center_field.y - (dy / dist) * 1.0;
            double target_theta = std::atan2(dy, dx);

            PathWaypoint score_wp;
            score_wp.pose = {target_x, target_y, target_theta};
            score_wp.velocity = 0;
            score_wp.name = "Score";
            path.waypoints.push_back(score_wp);
        }
    }

    // Score action
    PathAction score_act;
    score_act.name = score_action;
    score_act.command_name = score_action;
    score_act.trigger = TriggerCondition::AT_WAYPOINT;
    score_act.waypoint_index = 2;
    path.actions.push_back(score_act);

    return path;
}

std::vector<AutoPath> generate_multi_piece_auto(
    GameYear game,
    Alliance alliance,
    int num_pieces,
    const FieldLayout& field) {

    std::vector<AutoPath> paths;

    // Game-specific multi-piece auto generation would go here
    // This is a placeholder that generates basic paths

    Pose2D start = (alliance == Alliance::BLUE) ?
        Pose2D{1.5, 4.0, 0.0} : Pose2D{15.0, 4.0, M_PI};

    for (int i = 0; i < num_pieces; i++) {
        AutoPath piece_path;
        piece_path.name = "Piece" + std::to_string(i + 1);

        PathWaypoint wp1;
        wp1.pose = start;
        piece_path.waypoints.push_back(wp1);

        // Add more waypoints based on game...
        paths.push_back(piece_path);
    }

    return paths;
}

} // namespace sim
} // namespace frc_vision
