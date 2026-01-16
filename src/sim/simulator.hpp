#pragma once
/**
 * @file simulator.hpp
 * @brief Main Mac simulator orchestration
 *
 * Coordinates all simulator components:
 * - Robot dynamics (WASD/QE control)
 * - Field rendering (synthetic AprilTags)
 * - Webcam capture and compositing
 * - Vision pipeline (detection, pose, fusion)
 * - Auto-align controller
 * - PathPlanner integration with waypoint plotting
 * - Multi-game field selection (2024/2025/2026)
 * - Visualization
 */

#include "sim_types.hpp"
#include "robot_dynamics.hpp"
#include "field_renderer.hpp"
#include "auto_align.hpp"
#include "game_scoring.hpp"
#include "path_planner.hpp"
#include "../types.hpp"
#include "../fusion.hpp"
#include "../config.hpp"

#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>

namespace frc_vision {
namespace sim {

/**
 * @brief Simulator mode
 */
enum class SimMode {
    DRIVE,          // Manual WASD driving
    PATH_EDIT,      // Editing path waypoints (click to add)
    PATH_EXECUTE,   // Executing planned path
    AUTO_ALIGN      // Auto-aligning to tag
};

/**
 * @brief Main simulator class
 */
class Simulator {
public:
    Simulator();
    ~Simulator();

    /**
     * @brief Initialize simulator
     * @param config_path Path to sim_config.yml
     * @return true on success
     */
    bool initialize(const std::string& config_path);

    /**
     * @brief Run simulator main loop
     * @return Exit code
     */
    int run();

    /**
     * @brief Request shutdown
     */
    void shutdown() { shutdown_requested_.store(true); }

private:
    // Initialization
    bool load_config(const std::string& path);
    bool load_field_layout();
    bool load_field_for_year(GameYear year);
    bool load_camera_calibration();
    bool initialize_webcam();
    bool initialize_vision_pipeline();
    void print_keybinds();

    // Main loop
    void process_input();
    void update_dynamics(double dt);
    void capture_and_render();
    void run_detection();
    void update_fusion();
    void update_visualization();
    void handle_auto_align(double dt);
    void handle_path_execution(double dt);

    // Input handling
    void handle_key(int key);
    void handle_mouse(int event, int x, int y, int flags);
    static void mouse_callback(int event, int x, int y, int flags, void* userdata);

    // Field selection
    void cycle_field();
    void set_game_year(GameYear year);

    // Path planning
    void add_waypoint_at_click(double field_x, double field_y);
    void remove_last_waypoint();
    void clear_path();
    void execute_path();
    void stop_path_execution();
    void generate_path_code();
    cv::Point2d field_to_screen(double fx, double fy) const;
    std::pair<double, double> screen_to_field(int sx, int sy) const;

    // Timing
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    TimePoint last_update_time_;
    TimePoint last_render_time_;
    TimePoint key_hold_time_;  // Last time a movement key was pressed

    // Configuration
    SimConfig sim_config_;
    std::string config_dir_;
    Config vision_config_;

    // Game and field
    GameYear current_game_year_ = GameYear::REBUILT_2026;
    FieldLayout field_;
    GameScoringSystem scoring_;

    // Robot state
    RobotDynamics dynamics_;
    InputState input_;

    // Renderers
    FieldRenderer field_renderer_;
    TopDownRenderer topdown_renderer_;

    // Vision
    std::unique_ptr<VisionPipeline> pipeline_;
    CameraIntrinsics intrinsics_;
    Pose3D camera_to_robot_;

    // Auto-align
    AutoAlignController auto_align_;

    // PathPlanner
    PathPlanner path_planner_;
    AutoPath current_path_;
    SimMode mode_ = SimMode::DRIVE;
    double path_time_ = 0;  // Current time in path execution
    bool path_executing_ = false;

    // Webcam
    cv::VideoCapture webcam_;
    cv::Mat webcam_frame_;
    std::mutex webcam_mutex_;
    std::thread webcam_thread_;
    std::atomic<bool> webcam_running_{false};

    // Frame processing
    cv::Mat current_frame_;      // Frame for detection (composite or synthetic)
    cv::Mat display_frame_;      // Frame for display
    FrameDetections last_detections_;

    // Field view dimensions (for coordinate conversion)
    int field_view_width_ = 0;
    int field_view_height_ = 0;
    double field_length_ = 16.54;
    double field_width_ = 8.21;

    // Statistics
    struct Stats {
        double fps = 0;
        double detect_ms = 0;
        double pose_ms = 0;
        int tag_count = 0;
        double reproj_error = 0;
        double confidence = 0;
    } stats_;

    // FPS calculation
    int frame_count_ = 0;
    TimePoint fps_start_time_;

    // Windows
    const std::string CAMERA_WINDOW = "FRC Vision Simulator - Camera View";
    const std::string FIELD_WINDOW = "FRC Vision Simulator - Field View";

    // State
    std::atomic<bool> shutdown_requested_{false};
};

} // namespace sim
} // namespace frc_vision
