# FRC Vision PathPlanner API Documentation

## Overview

The FRC Vision PathPlanner is a code generator that creates autonomous path-following commands for WPILib-based swerve drive robots. It supports:

- Bezier curve trajectory generation
- Tag-triggered actions
- Code generation for Java, C++, and Python
- PathPlanner JSON export/import compatibility

## Quick Start

### Creating a Path

```cpp
#include "sim/path_planner.hpp"

using namespace frc_vision::sim;

PathPlanner planner;

// Configure swerve drive
SwerveConfig config;
config.track_width = 0.5969;      // 23.5 inches
config.wheel_base = 0.5969;
config.drive_motor = "NEO";
config.steer_motor = "NEO550";
planner.set_swerve_config(config);

// Create a path
AutoPath& path = planner.create_path("ScoreSpeaker");

// Add waypoints
PathWaypoint start;
start.pose = {1.5, 4.0, 0.0};  // x, y, theta (radians)
start.name = "Start";
planner.add_waypoint(path, start);

PathWaypoint score;
score.pose = {3.0, 5.5, M_PI/4};
score.velocity = 0;  // Stop at this point
score.name = "ScorePosition";
planner.add_waypoint(path, score);

// Generate trajectory
planner.generate_trajectory(path);

// Generate code
std::string java_code = planner.generate_code(path, CodeFormat::WPILIB_JAVA);
```

## API Reference

### PathPlanner Class

#### Constructor
```cpp
PathPlanner();
```

#### Configuration Methods

```cpp
void set_swerve_config(const SwerveConfig& config);
```
Configure the swerve drive parameters for accurate code generation.

```cpp
void set_field_layout(const FieldLayout& field);
```
Set the field layout for tag-based action triggers.

#### Path Creation

```cpp
AutoPath& create_path(const std::string& name);
```
Create a new path with the given name. Returns a reference to the path.

```cpp
AutoPath* get_path(const std::string& name);
```
Get an existing path by name. Returns nullptr if not found.

```cpp
void add_waypoint(AutoPath& path, const PathWaypoint& waypoint);
```
Add a waypoint to a path.

```cpp
void add_action(AutoPath& path, const PathAction& action);
```
Add an action trigger to a path.

#### Trajectory Generation

```cpp
void generate_trajectory(AutoPath& path, double dt = 0.02);
```
Generate the trajectory for a path. The `dt` parameter specifies the time step (default 50Hz).

#### Code Generation

```cpp
std::string generate_code(const AutoPath& path, CodeFormat format) const;
```
Generate code for a single path.

Supported formats:
- `CodeFormat::WPILIB_JAVA` - Java code for WPILib
- `CodeFormat::WPILIB_CPP` - C++ code for WPILib
- `CodeFormat::WPILIB_PYTHON` - Python code for RobotPy
- `CodeFormat::PATHPLANNER_JSON` - PathPlanner-compatible JSON

```cpp
std::string generate_auto_routine(const std::vector<std::string>& paths, CodeFormat format) const;
```
Generate a complete autonomous routine that chains multiple paths.

#### File I/O

```cpp
bool save_path(const AutoPath& path, const std::string& filename) const;
```
Save a path to JSON file (PathPlanner format).

```cpp
bool load_path(const std::string& filename, AutoPath& path);
```
Load a path from JSON file.

#### Path Sampling

```cpp
Pose2D sample_path(const AutoPath& path, double t) const;
```
Get the pose at time `t` on the path.

```cpp
double sample_velocity(const AutoPath& path, double t) const;
```
Get the velocity at time `t` on the path.

### Data Structures

#### PathWaypoint

```cpp
struct PathWaypoint {
    Pose2D pose;                    // Position and heading
    double velocity = 0;            // Target velocity (m/s), 0 = stop
    RotationMode rotation_mode = RotationMode::HOLONOMIC;

    // Bezier control points (relative to waypoint)
    double control_in_x = 0;
    double control_in_y = 0;
    double control_out_x = 0;
    double control_out_y = 0;

    // Local constraints (-1 = use path default)
    double max_velocity = -1;
    double max_accel = -1;
    double max_angular_vel = -1;

    std::string name = "";
};
```

#### PathAction

```cpp
struct PathAction {
    std::string name;               // Human-readable name
    std::string command_name;       // Command to execute
    TriggerCondition trigger;       // When to trigger

    // Trigger parameters
    int waypoint_index = -1;        // For AT_WAYPOINT
    int tag_id = -1;                // For TAG_VISIBLE
    double distance = 0;            // For DISTANCE_TO_TAG
    double time = 0;                // For TIME_ELAPSED

    bool wait_for_completion = false;
    double timeout = 5.0;
};
```

#### TriggerCondition

```cpp
enum class TriggerCondition {
    AT_WAYPOINT,        // Trigger at specific waypoint
    TAG_VISIBLE,        // Trigger when tag becomes visible
    DISTANCE_TO_TAG,    // Trigger within distance of tag
    TIME_ELAPSED,       // Trigger after time from path start
    CUSTOM              // Custom condition
};
```

#### AutoPath

```cpp
struct AutoPath {
    std::string name;
    std::string description;

    std::vector<PathWaypoint> waypoints;
    std::vector<PathAction> actions;

    // Constraints
    double max_velocity = 4.0;          // m/s
    double max_acceleration = 3.0;      // m/s^2
    double max_angular_velocity = 4.0;  // rad/s
    double max_angular_accel = 6.0;     // rad/s^2

    // Options
    bool reverse = false;
    bool reset_odometry = true;

    // Generated data
    double total_time = 0;
    double total_distance = 0;
};
```

#### SwerveConfig

```cpp
struct SwerveConfig {
    // Physical dimensions
    double track_width = 0.5969;        // meters
    double wheel_base = 0.5969;         // meters
    double wheel_diameter = 0.1016;     // meters
    double drive_gear_ratio = 6.75;     // L2 ratio
    double steer_gear_ratio = 12.8;

    // Motor types: "NEO", "Falcon500", "Kraken"
    std::string drive_motor = "NEO";
    std::string steer_motor = "NEO550";

    // CAN IDs [FL, FR, BL, BR]
    std::array<int, 4> drive_ids = {1, 2, 3, 4};
    std::array<int, 4> steer_ids = {5, 6, 7, 8};
    std::array<int, 4> encoder_ids = {9, 10, 11, 12};

    // PID Gains
    double drive_kP = 0.1, drive_kI = 0.0, drive_kD = 0.0;
    double drive_kS = 0.0, drive_kV = 2.0, drive_kA = 0.0;
    double steer_kP = 5.0, steer_kI = 0.0, steer_kD = 0.0;

    // Path following PID
    double translation_kP = 5.0;
    double rotation_kP = 5.0;
};
```

## Preset Path Generators

### generate_score_path

```cpp
AutoPath generate_score_path(
    const Pose2D& start,
    int target_tag_id,
    const std::string& score_action,
    const FieldLayout& field,
    double standoff_distance = 1.0);
```

Generates a path from `start` to a scoring position facing `target_tag_id`.

**Example:**
```cpp
auto path = generate_score_path(
    {1.5, 4.0, 0.0},  // Start position
    7,                 // Target tag (Blue Speaker)
    "score_speaker",   // Action to execute
    field,
    1.0               // 1 meter standoff
);
```

### generate_pickup_score_path

```cpp
AutoPath generate_pickup_score_path(
    const Pose2D& start,
    const Pose2D& pickup_location,
    int score_tag_id,
    const std::string& score_action,
    const FieldLayout& field);
```

Generates a path that picks up a game piece and scores it.

### generate_multi_piece_auto

```cpp
std::vector<AutoPath> generate_multi_piece_auto(
    GameYear game,
    Alliance alliance,
    int num_pieces,
    const FieldLayout& field);
```

Generates a multi-piece autonomous routine for a specific game and alliance.

## Generated Code Examples

### Java Output

```java
// Auto-generated path: ScoreSpeaker
package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
// ... imports

public class ScoreSpeakerPath {

    public static final Pose2d startPose = new Pose2d(
        1.5, 4.0, Rotation2d.fromRadians(0.0)
    );

    public static final Pose2d endPose = new Pose2d(
        3.0, 5.5, Rotation2d.fromRadians(0.785)
    );

    public static Trajectory getTrajectory() {
        return TrajectoryGenerator.generateTrajectory(
            startPose,
            interiorWaypoints,
            endPose,
            config
        );
    }

    public static Command getCommand(SwerveDrive drive) {
        var trajectory = getTrajectory();
        // ... command generation
    }
}
```

### C++ Output

```cpp
// Auto-generated path: ScoreSpeaker
#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/trajectory/Trajectory.h>

namespace auto_paths {

class ScoreSpeakerPath {
public:
    static frc::Pose2d GetStartPose() {
        return frc::Pose2d{1.5_m, 4.0_m, frc::Rotation2d{0.0_rad}};
    }

    static frc::Trajectory GetTrajectory() {
        return frc::TrajectoryGenerator::GenerateTrajectory(
            GetStartPose(),
            GetInteriorWaypoints(),
            GetEndPose(),
            GetConfig()
        );
    }

    template<typename DriveSubsystem>
    static frc2::CommandPtr GetCommand(DriveSubsystem* drive);
};

}
```

### Python Output

```python
# Auto-generated path: ScoreSpeaker
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

class ScoreSpeakerPath:
    START_POSE = Pose2d(1.5, 4.0, Rotation2d(0.0))
    END_POSE = Pose2d(3.0, 5.5, Rotation2d(0.785))

    @classmethod
    def get_trajectory(cls):
        config = TrajectoryConfig(4.0, 3.0)
        return TrajectoryGenerator.generateTrajectory(
            cls.START_POSE,
            cls.INTERIOR_WAYPOINTS,
            cls.END_POSE,
            config
        )

    @classmethod
    def get_command(cls, drive):
        # ... command generation
```

## Integration with Vision System

The PathPlanner integrates with the vision-based pose estimation system to:

1. **Tag-triggered actions**: Execute commands when specific AprilTags are detected
2. **Vision-corrected paths**: Use vision feedback to improve path following accuracy
3. **Dynamic retargeting**: Adjust paths based on real-time tag detection

### Example: Tag-Triggered Scoring

```cpp
PathAction score_action;
score_action.name = "Score when aligned";
score_action.command_name = "shootNote";
score_action.trigger = TriggerCondition::DISTANCE_TO_TAG;
score_action.tag_id = 7;        // Blue speaker
score_action.distance = 1.5;    // Within 1.5m
score_action.wait_for_completion = true;

planner.add_action(path, score_action);
```

## Best Practices

1. **Set realistic constraints**: Don't exceed your robot's actual capabilities
2. **Use control points**: Smooth paths by adding Bezier control points
3. **Test trajectories**: Generate and visualize trajectories before deploying
4. **Validate on field**: Test generated code on the actual field
5. **Version control**: Save paths to JSON for version control

## Supported Games

- **2024 CRESCENDO**: 16 AprilTags, Speaker/Amp/Stage scoring
- **2025 REEFSCAPE**: 22 AprilTags, Coral/Algae/Reef scoring
- **2026 REBUILT**: 32 AprilTags, Artifact/Fuel/Tower scoring

## File Format Compatibility

The PathPlanner uses a JSON format compatible with FRC PathPlanner 2025:

```json
{
  "version": "2025.0",
  "name": "ScoreSpeaker",
  "globalConstraints": {
    "maxVelocity": 4.0,
    "maxAcceleration": 3.0
  },
  "waypoints": [
    {
      "anchor": {"x": 1.5, "y": 4.0},
      "holonomicAngle": 0.0,
      "name": "Start"
    }
  ],
  "eventMarkers": []
}
```
