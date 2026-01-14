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
 * - Visualization
 */

#include "sim_types.hpp"
#include "robot_dynamics.hpp"
#include "field_renderer.hpp"
#include "auto_align.hpp"
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

    // Input handling
    void handle_key(int key);

    // Timing
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    TimePoint last_update_time_;
    TimePoint last_render_time_;

    // Configuration
    SimConfig sim_config_;
    std::string config_dir_;
    Config vision_config_;

    // Field
    FieldLayout field_;

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
