/**
 * @file main.cpp
 * @brief FRC AprilTag Vision Coprocessor - Main Entry Point
 *
 * Production-grade vision system for FRC 2026 season.
 * Supports 3 cameras, multi-tag pose estimation, and roboRIO integration.
 *
 * Architecture:
 * - 1 capture thread per camera
 * - 1 processing thread per camera (detect + pose)
 * - 1 fusion thread (combines all cameras)
 * - 1 NT publishing thread
 * - 1 web server thread
 */

#include "config.hpp"
#include "camera.hpp"
#include "detector.hpp"
#include "tracker.hpp"
#include "pose.hpp"
#include "fusion.hpp"
#include "nt_publisher.hpp"
#include "web_server.hpp"
#include "field_layout.hpp"
#include "ring_buffer.hpp"

#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <signal.h>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

// Global shutdown flag
static std::atomic<bool> g_shutdown{false};

void signal_handler(int /*sig*/) {
    std::cout << "\n[Main] Shutdown requested..." << std::endl;
    g_shutdown.store(true);
}

// Get CPU temperature (Orange Pi / Rockchip specific)
double get_cpu_temp() {
    std::ifstream f("/sys/class/thermal/thermal_zone0/temp");
    if (f.is_open()) {
        int temp_milli;
        f >> temp_milli;
        return temp_milli / 1000.0;
    }
    return 0;
}

int main(int argc, char* argv[]) {
    using namespace frc_vision;

    std::cout << "========================================" << std::endl;
    std::cout << "FRC AprilTag Vision Coprocessor v2.0" << std::endl;
    std::cout << "FRC 2026 Season - Production Ready" << std::endl;
    std::cout << "========================================" << std::endl;

    // Setup signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // ==========================================================================
    // Load Configuration
    // ==========================================================================

    // Determine paths
    fs::path exe_path = fs::canonical("/proc/self/exe").parent_path();
    fs::path config_path = exe_path / "config" / "config.yml";

    // Check command line argument for config path
    if (argc > 1) {
        config_path = argv[1];
    }

    // Fallback paths
    if (!fs::exists(config_path)) {
        config_path = "/etc/frc_vision/config.yml";
    }
    if (!fs::exists(config_path)) {
        config_path = "config/config.yml";
    }

    std::cout << "[Main] Loading configuration from: " << config_path << std::endl;

    ConfigManager config_manager;
    if (!config_manager.load(config_path.string())) {
        std::cerr << "[Main] Failed to load configuration!" << std::endl;
        return 1;
    }

    const Config& config = config_manager.get();

    // ==========================================================================
    // Load Field Layout
    // ==========================================================================

    fs::path field_layout_path = fs::path(config_path).parent_path() / "field_layout.json";
    FieldLayout field_layout;

    if (fs::exists(field_layout_path)) {
        field_layout = load_field_layout(field_layout_path.string(), config.field.tag_size_m);
    } else {
        std::cout << "[Main] No field_layout.json found, using default layout" << std::endl;
        field_layout = create_default_field_layout(config.field.tag_size_m);
    }

    // ==========================================================================
    // Initialize Cameras
    // ==========================================================================

    int num_cameras = static_cast<int>(config.cameras.size());
    std::cout << "[Main] Initializing " << num_cameras << " cameras..." << std::endl;

    CameraManager camera_manager;
    int started_cameras = camera_manager.initialize(config.cameras);

    if (started_cameras == 0) {
        std::cerr << "[Main] No cameras started! Check device paths." << std::endl;
        return 1;
    }

    std::cout << "[Main] " << started_cameras << "/" << num_cameras << " cameras started" << std::endl;

    // Load camera intrinsics
    std::vector<CameraIntrinsics> intrinsics(num_cameras);
    for (int i = 0; i < num_cameras; i++) {
        const auto& cam_config = config.cameras[i];
        if (!cam_config.intrinsics_file.empty()) {
            fs::path intr_path = fs::path(config_path).parent_path() / cam_config.intrinsics_file;
            auto intr = ConfigManager::load_intrinsics(intr_path.string());
            if (intr) {
                intrinsics[i] = *intr;
            } else {
                std::cerr << "[Main] Warning: Could not load intrinsics for camera " << i << std::endl;
            }
        }

        // Set intrinsics on camera object
        if (auto* cam = camera_manager.get_camera(i)) {
            cam->set_intrinsics(intrinsics[i]);
        }
    }

    // ==========================================================================
    // Initialize Vision Pipeline
    // ==========================================================================

    VisionPipeline pipeline;
    pipeline.initialize(num_cameras, field_layout,
                       config.detector, config.tracker,
                       config.tracker.filter_alpha);

    // ==========================================================================
    // Initialize NetworkTables
    // ==========================================================================

    NTPublisher nt_publisher;
    if (config.output.nt_enable) {
        if (nt_publisher.initialize(config.output.nt_server,
                                   config.output.nt_table_root,
                                   num_cameras)) {
            nt_publisher.start(config.output.publish_rate_hz);
            std::cout << "[Main] NetworkTables started, connecting to "
                      << config.output.nt_server << std::endl;
        } else {
            std::cerr << "[Main] Warning: Failed to initialize NetworkTables" << std::endl;
        }
    }

    // ==========================================================================
    // Initialize Web Server
    // ==========================================================================

    fs::path web_root = exe_path / "web";
    if (!fs::exists(web_root)) {
        web_root = "web";
    }

    WebServer web_server;
    if (web_server.initialize(config.output.web_port, web_root.string(),
                              num_cameras, &config_manager)) {
        web_server.set_jpeg_quality(config.performance.jpeg_quality);
        web_server.start();
        std::cout << "[Main] Web server started at http://0.0.0.0:"
                  << config.output.web_port << std::endl;
    } else {
        std::cerr << "[Main] Warning: Failed to start web server" << std::endl;
    }

    // ==========================================================================
    // Processing Threads (one per camera)
    // ==========================================================================

    std::vector<std::thread> processing_threads;
    std::vector<std::atomic<double>> processing_fps(num_cameras);
    std::vector<std::atomic<uint64_t>> processing_frames(num_cameras);

    for (int i = 0; i < num_cameras; i++) {
        processing_fps[i].store(0);
        processing_frames[i].store(0);

        processing_threads.emplace_back([&, i]() {
            Camera* cam = camera_manager.get_camera(i);
            if (!cam) return;

            const CameraIntrinsics& intr = intrinsics[i];
            const Pose3D& cam_to_robot = config.cameras[i].camera_to_robot;

            auto last_fps_time = SteadyClock::now();
            int fps_count = 0;
            std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, config.performance.jpeg_quality};

            while (!g_shutdown.load()) {
                // Get latest frame
                auto frame_opt = cam->frame_buffer().pop_latest();
                if (!frame_opt) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }

                Frame& frame = *frame_opt;

                // Process frame
                auto processed = pipeline.process_frame(frame, intr, cam_to_robot);

                // Encode JPEG for web streaming
                std::vector<uint8_t> jpeg;
                cv::imencode(".jpg", processed.annotated_image, jpeg, jpeg_params);
                processed.jpeg = std::move(jpeg);

                // Push to web server
                web_server.push_frame(i, processed.jpeg);
                web_server.push_detections(processed.detections);

                // Push to NT publisher
                nt_publisher.publish_camera(i, processed.detections);

                // Update stats
                processing_frames[i].fetch_add(1);
                fps_count++;

                auto now = SteadyClock::now();
                auto elapsed = std::chrono::duration<double>(now - last_fps_time).count();
                if (elapsed >= 1.0) {
                    processing_fps[i].store(fps_count / elapsed);
                    fps_count = 0;
                    last_fps_time = now;
                }

                // Check for config reload
                if (config_manager.reload_requested()) {
                    pipeline.update_config(config_manager.get().detector,
                                          config_manager.get().tracker);
                    config_manager.clear_reload_flag();
                }
            }
        });
    }

    // ==========================================================================
    // Fusion Thread
    // ==========================================================================

    std::thread fusion_thread([&]() {
        auto last_publish = SteadyClock::now();
        int publish_period_ms = 1000 / config.output.publish_rate_hz;

        while (!g_shutdown.load()) {
            // Get fused pose
            auto fused = pipeline.get_fused_pose();

            // Publish to NT and web
            nt_publisher.publish_fused(fused);
            web_server.push_fused_pose(fused);

            // Rate limit
            auto now = SteadyClock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_publish).count();
            if (elapsed < publish_period_ms) {
                std::this_thread::sleep_for(std::chrono::milliseconds(publish_period_ms - elapsed));
            }
            last_publish = SteadyClock::now();
        }
    });

    // ==========================================================================
    // Status Thread
    // ==========================================================================

    std::thread status_thread([&]() {
        auto start_time = SteadyClock::now();

        while (!g_shutdown.load()) {
            SystemStatus status;
            status.start_time = start_time;
            status.uptime_seconds = std::chrono::duration<double>(
                SteadyClock::now() - start_time).count();
            status.cpu_temp = get_cpu_temp();

            // Camera stats
            for (int i = 0; i < num_cameras; i++) {
                CameraStatus cam_status;
                cam_status.camera_id = i;

                if (auto* cam = camera_manager.get_camera(i)) {
                    cam_status.connected = cam->is_connected();
                    cam_status.fps = cam->fps();
                    cam_status.frames_captured = cam->frames_captured();
                    cam_status.frames_dropped = cam->frames_dropped();
                }

                cam_status.avg_total_ms = 1000.0 / std::max(0.1, processing_fps[i].load());
                status.cameras.push_back(cam_status);
            }

            status.fused_pose_valid = pipeline.get_fused_pose().valid;

            // Push to NT and web
            nt_publisher.publish_status(status);
            web_server.push_status(status);

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    });

    // ==========================================================================
    // Main Loop (just wait for shutdown)
    // ==========================================================================

    std::cout << "[Main] Running... Press Ctrl+C to stop." << std::endl;
    std::cout << "[Main] Dashboard: http://localhost:" << config.output.web_port << std::endl;

    while (!g_shutdown.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Print periodic status
        static auto last_print = SteadyClock::now();
        auto now = SteadyClock::now();
        if (std::chrono::duration<double>(now - last_print).count() >= 5.0) {
            std::cout << "[Status] ";
            for (int i = 0; i < num_cameras; i++) {
                if (auto* cam = camera_manager.get_camera(i)) {
                    std::cout << "Cam" << i << ": " << std::fixed << std::setprecision(1)
                              << cam->fps() << " fps | ";
                }
            }
            std::cout << "NT: " << (nt_publisher.is_connected() ? "Connected" : "Disconnected")
                      << " | Web clients: " << web_server.websocket_client_count()
                      << std::endl;
            last_print = now;
        }
    }

    // ==========================================================================
    // Shutdown
    // ==========================================================================

    std::cout << "[Main] Shutting down..." << std::endl;

    // Stop all threads
    status_thread.join();
    fusion_thread.join();

    for (auto& t : processing_threads) {
        t.join();
    }

    // Stop services
    web_server.stop();
    nt_publisher.stop();
    camera_manager.stop_all();

    std::cout << "[Main] Shutdown complete." << std::endl;
    return 0;
}
