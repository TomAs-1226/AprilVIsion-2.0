/**
 * @file main.cpp
 * @brief FRC AprilTag Vision Coprocessor - Main Entry Point
 *
 * Production-grade vision system for FRC 2026 season.
 * Designed for FAST BOOT with non-blocking initialization.
 *
 * Boot Design:
 * 1. Load config immediately (no blocking)
 * 2. Start web server FIRST (for readiness checks)
 * 3. Start cameras ASYNC (don't block if cameras slow)
 * 4. Start NT publisher (with internal retry loop)
 * 5. Log "READY" when at least 1 camera + NT initialized
 *
 * Degraded Mode:
 * - Cameras not ready: keep retrying, serve "warming up" status
 * - NT not reachable: keep retrying without exiting
 * - Never block boot or hang on network issues
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
#include <iomanip>
#include <signal.h>
#include <filesystem>
#include <fstream>
#include <getopt.h>

namespace fs = std::filesystem;

// =============================================================================
// Global State
// =============================================================================

static std::atomic<bool> g_shutdown{false};
static std::atomic<bool> g_ready{false};
static std::atomic<bool> g_cameras_ready{false};
static std::atomic<bool> g_nt_initialized{false};
static std::atomic<bool> g_web_ready{false};
static std::atomic<int> g_cameras_connected{0};

// Startup timestamp
static frc_vision::SteadyTimePoint g_startup_time;

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

// Get uptime in seconds
double get_uptime() {
    return std::chrono::duration<double>(
        frc_vision::SteadyClock::now() - g_startup_time).count();
}

// Check and update ready state
void check_ready_state() {
    if (g_ready.load()) return;

    bool cameras_ok = g_cameras_connected.load() > 0;
    bool nt_ok = g_nt_initialized.load();
    bool web_ok = g_web_ready.load();

    if (cameras_ok && web_ok) {
        g_ready.store(true);
        double startup_time = get_uptime();
        std::cout << "\n========================================" << std::endl;
        std::cout << "[READY] System ready in " << std::fixed << std::setprecision(2)
                  << startup_time << " seconds" << std::endl;
        std::cout << "[READY] Cameras: " << g_cameras_connected.load()
                  << " | NT: " << (nt_ok ? "Connected" : "Retrying")
                  << " | Web: OK" << std::endl;
        std::cout << "========================================\n" << std::endl;
    }
}

void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [OPTIONS] [config_path]\n"
              << "\nOptions:\n"
              << "  --fast-start    Boot optimization mode (default if run as service)\n"
              << "  --help          Show this help\n"
              << "\nExamples:\n"
              << "  " << prog << " config/config.yml\n"
              << "  " << prog << " --fast-start /opt/frc-vision/config/config.yml\n";
}

int main(int argc, char* argv[]) {
    using namespace frc_vision;

    g_startup_time = SteadyClock::now();

    // =========================================================================
    // Parse Command Line
    // =========================================================================

    bool fast_start = false;
    std::string config_path_arg;

    static struct option long_options[] = {
        {"fast-start", no_argument, nullptr, 'f'},
        {"help", no_argument, nullptr, 'h'},
        {nullptr, 0, nullptr, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "fh", long_options, nullptr)) != -1) {
        switch (opt) {
            case 'f':
                fast_start = true;
                break;
            case 'h':
                print_usage(argv[0]);
                return 0;
            default:
                print_usage(argv[0]);
                return 1;
        }
    }

    // Remaining argument is config path
    if (optind < argc) {
        config_path_arg = argv[optind];
    }

    std::cout << "========================================" << std::endl;
    std::cout << "FRC AprilTag Vision Coprocessor v2.0" << std::endl;
    std::cout << "FRC 2026 Season - Production Ready" << std::endl;
    if (fast_start) {
        std::cout << "Mode: FAST START (boot optimized)" << std::endl;
    }
    std::cout << "========================================" << std::endl;

    // Setup signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // =========================================================================
    // Load Configuration (FAST - no blocking)
    // =========================================================================

    fs::path exe_path;
    try {
        exe_path = fs::canonical("/proc/self/exe").parent_path();
    } catch (...) {
        exe_path = ".";
    }

    fs::path config_path;
    if (!config_path_arg.empty()) {
        config_path = config_path_arg;
    } else {
        config_path = exe_path / "config" / "config.yml";
    }

    // Fallback paths
    if (!fs::exists(config_path)) {
        config_path = "/opt/frc-vision/config/config.yml";
    }
    if (!fs::exists(config_path)) {
        config_path = "/etc/frc_vision/config.yml";
    }
    if (!fs::exists(config_path)) {
        config_path = "config/config.yml";
    }

    std::cout << "[Main] Loading config: " << config_path << std::endl;

    ConfigManager config_manager;
    if (!config_manager.load(config_path.string())) {
        std::cerr << "[Main] FATAL: Cannot load configuration!" << std::endl;
        // In fast-start mode, don't exit - try to continue with defaults
        if (!fast_start) {
            return 1;
        }
    }

    const Config& config = config_manager.get();
    int num_cameras = static_cast<int>(config.cameras.size());

    // =========================================================================
    // Load Field Layout (FAST - use defaults if file missing)
    // =========================================================================

    fs::path field_layout_path = fs::path(config_path).parent_path() / "field_layout.json";
    FieldLayout field_layout;

    if (fs::exists(field_layout_path)) {
        field_layout = load_field_layout(field_layout_path.string(), config.field.tag_size_m);
    } else {
        field_layout = create_default_field_layout(config.field.tag_size_m);
    }

    // =========================================================================
    // Initialize Vision Pipeline (FAST - before cameras)
    // =========================================================================

    VisionPipeline pipeline;
    pipeline.initialize(num_cameras, field_layout,
                       config.detector, config.tracker,
                       config.tracker.filter_alpha);

    // =========================================================================
    // START WEB SERVER FIRST (for readiness checks)
    // =========================================================================

    fs::path web_root = exe_path / "web";
    if (!fs::exists(web_root)) {
        web_root = "/opt/frc-vision/web";
    }
    if (!fs::exists(web_root)) {
        web_root = "web";
    }

    WebServer web_server;
    if (web_server.initialize(config.output.web_port, web_root.string(),
                              num_cameras, &config_manager)) {
        web_server.set_jpeg_quality(config.performance.jpeg_quality);
        web_server.start();
        g_web_ready.store(true);
        std::cout << "[Main] Web server started at http://0.0.0.0:"
                  << config.output.web_port << " (" << std::fixed << std::setprecision(2)
                  << get_uptime() << "s)" << std::endl;
    } else {
        std::cerr << "[Main] Warning: Web server failed to start" << std::endl;
    }

    // =========================================================================
    // Initialize NetworkTables (NON-BLOCKING with internal retry)
    // =========================================================================

    NTPublisher nt_publisher;
    std::thread nt_init_thread;

    if (config.output.nt_enable) {
        // Start NT initialization in background thread
        nt_init_thread = std::thread([&]() {
            std::cout << "[NT] Initializing connection to " << config.output.nt_server
                      << " (non-blocking)" << std::endl;

            // NT publisher has internal retry logic
            if (nt_publisher.initialize(config.output.nt_server,
                                        config.output.nt_table_root,
                                        num_cameras)) {
                nt_publisher.start(config.output.publish_rate_hz);
                g_nt_initialized.store(true);
                std::cout << "[NT] Publisher started (" << std::fixed << std::setprecision(2)
                          << get_uptime() << "s)" << std::endl;
            }
        });
    }

    // =========================================================================
    // Initialize Cameras (ASYNC - don't block main thread)
    // =========================================================================

    std::cout << "[Main] Starting " << num_cameras << " cameras (async)..." << std::endl;

    CameraManager camera_manager;
    std::vector<CameraIntrinsics> intrinsics(num_cameras);

    // Start camera initialization in background
    std::thread camera_init_thread([&]() {
        // Load intrinsics first (fast)
        for (int i = 0; i < num_cameras; i++) {
            const auto& cam_config = config.cameras[i];
            bool loaded = false;

            if (!cam_config.intrinsics_file.empty()) {
                fs::path intr_path = fs::path(config_path).parent_path() / cam_config.intrinsics_file;
                auto intr = ConfigManager::load_intrinsics(intr_path.string());
                if (intr) {
                    intrinsics[i] = *intr;
                    loaded = true;
                }
            }

            // Generate default intrinsics if calibration file not found.
            // Uses typical USB webcam parameters (~65 deg HFOV).
            // Approximate but MUCH better than no intrinsics (which disables PnP entirely).
            if (!loaded || !intrinsics[i].valid()) {
                intrinsics[i] = CameraIntrinsics::create_default(cam_config.width, cam_config.height);
                std::cout << "[Main] Camera " << i << ": using default intrinsics"
                          << " (fx=" << intrinsics[i].fx << "). Calibrate for better accuracy!"
                          << std::endl;
            }
        }

        // Initialize all cameras in a single call - each camera's capture_loop
        // has its own internal retry logic, so we don't need to re-initialize.
        // The old code called initialize() in a loop, but initialize() calls
        // stop_all() which kills cameras that were still connecting.
        int started = camera_manager.initialize(config.cameras);

        // Set intrinsics on all camera objects (even ones still connecting)
        for (int i = 0; i < num_cameras; i++) {
            if (auto* cam = camera_manager.get_camera(i)) {
                cam->set_intrinsics(intrinsics[i]);
            }
        }

        if (started > 0) {
            g_cameras_connected.store(started);
            g_cameras_ready.store(true);
        }

        std::cout << "[Main] " << started << "/" << num_cameras
                  << " cameras started initially (" << std::fixed << std::setprecision(2)
                  << get_uptime() << "s)" << std::endl;

        // Wait for remaining cameras to connect via their internal capture_loop retries.
        // Each camera's capture thread keeps running and will reconnect on its own.
        if (started < num_cameras) {
            std::cout << "[Main] Waiting for remaining cameras to connect..." << std::endl;
            int wait_count = 0;
            int max_wait = 30;  // 30 seconds

            while (!g_shutdown.load() && wait_count < max_wait) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                wait_count++;

                int connected = 0;
                for (int i = 0; i < num_cameras; i++) {
                    if (auto* cam = camera_manager.get_camera(i)) {
                        if (cam->is_connected()) connected++;
                    }
                }

                if (connected > started) {
                    g_cameras_connected.store(connected);
                    if (!g_cameras_ready.load() && connected > 0) {
                        g_cameras_ready.store(true);
                    }
                    std::cout << "[Main] " << connected << "/" << num_cameras
                              << " cameras now connected (" << std::fixed << std::setprecision(2)
                              << get_uptime() << "s)" << std::endl;
                    started = connected;
                }

                if (connected >= num_cameras) {
                    std::cout << "[Main] All " << num_cameras << " cameras connected!" << std::endl;
                    break;
                }
            }
        }

        if (g_cameras_connected.load() == 0) {
            std::cerr << "[Main] Warning: No cameras available after waiting" << std::endl;
        }

        check_ready_state();
    });

    // =========================================================================
    // Processing Threads (start immediately, wait for frames)
    // =========================================================================

    std::vector<std::thread> processing_threads;
    std::vector<std::atomic<double>> processing_fps(num_cameras);
    std::vector<std::atomic<uint64_t>> processing_frames(num_cameras);

    for (int i = 0; i < num_cameras; i++) {
        processing_fps[i].store(0);
        processing_frames[i].store(0);

        processing_threads.emplace_back([&, i]() {
            Camera* cam = nullptr;
            const CameraIntrinsics& intr = intrinsics[i];
            const Pose3D& cam_to_robot = config.cameras[i].camera_to_robot;

            auto last_fps_time = SteadyClock::now();
            int fps_count = 0;
            int empty_count = 0;
            std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, config.performance.jpeg_quality};

            while (!g_shutdown.load()) {
                // Wait for camera to be ready (handles initial connect AND reconnects)
                cam = camera_manager.get_camera(i);
                if (!cam || !cam->is_connected()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }

                if (empty_count == 0) {
                    std::cout << "[Processing " << i << "] Processing thread active for camera " << i << std::endl;
                }

                // Get latest frame (ring buffer drops old frames to prevent lag)
                auto frame_opt = cam->frame_buffer().pop_latest();
                if (!frame_opt) {
                    empty_count++;
                    // If camera disconnects while we're waiting, go back to connection check
                    if (!cam->is_connected()) {
                        std::cout << "[Processing " << i << "] Camera disconnected, waiting for reconnect..." << std::endl;
                        empty_count = 0;
                        continue;
                    }
                    if (empty_count == 100 || empty_count == 1000 || empty_count % 10000 == 0) {
                        std::cout << "[Processing " << i << "] Waiting for frames... (empty reads: "
                                  << empty_count << ", cam connected: " << cam->is_connected()
                                  << ", captured: " << cam->frames_captured() << ")" << std::endl;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }

                Frame& frame = *frame_opt;
                empty_count = 0;

                try {
                    // Process frame
                    auto processed = pipeline.process_frame(frame, intr, cam_to_robot);

                    // Encode JPEG for web streaming
                    std::vector<uint8_t> jpeg;
                    if (processed.annotated_image.empty()) {
                        continue;
                    }

                    cv::imencode(".jpg", processed.annotated_image, jpeg, jpeg_params);
                    if (jpeg.empty()) {
                        continue;
                    }
                    processed.jpeg = std::move(jpeg);

                    // Push to web server
                    web_server.push_frame(i, processed.jpeg);
                    web_server.push_detections(processed.detections);

                    // Push to NT publisher
                    nt_publisher.publish_camera(i, processed.detections);

                    // Update stats
                    uint64_t frame_count = processing_frames[i].fetch_add(1) + 1;
                    fps_count++;

                    // Log first few frames to verify pipeline working
                    if (frame_count <= 3 || frame_count % 500 == 0) {
                        std::cout << "[Processing " << i << "] Frame " << frame_count
                                  << " processed, JPEG size: " << processed.jpeg.size() << " bytes"
                                  << ", detections: " << processed.detections.detections.size() << std::endl;
                    }

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
                } catch (const cv::Exception& e) {
                    std::cerr << "[Processing " << i << "] OpenCV error: " << e.what() << std::endl;
                    continue;
                } catch (const std::exception& e) {
                    std::cerr << "[Processing " << i << "] Error: " << e.what() << std::endl;
                    continue;
                }
            }
        });
    }

    // =========================================================================
    // Fusion Thread
    // =========================================================================

    std::thread fusion_thread([&]() {
        int publish_period_ms = 1000 / config.output.publish_rate_hz;

        while (!g_shutdown.load()) {
            auto start = SteadyClock::now();

            // Read odometry from RoboRIO (via NT) for innovation gating
            if (config.output.nt_enable && nt_publisher.is_connected()) {
                auto rio_odom = nt_publisher.get_rio_odometry();
                if (rio_odom.valid) {
                    OdometryData odom;
                    odom.pose.x = rio_odom.x;
                    odom.pose.y = rio_odom.y;
                    odom.pose.theta = rio_odom.theta;
                    odom.angular_velocity = rio_odom.angular_velocity;
                    odom.timestamp = SteadyClock::now();
                    odom.valid = true;
                    pipeline.fusion().update_odometry(odom);
                }
            }

            // Get fused pose
            auto fused = pipeline.get_fused_pose();

            // Publish to NT and web
            nt_publisher.publish_fused(fused);
            web_server.push_fused_pose(fused);

            // Rate limit
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                SteadyClock::now() - start).count();
            if (elapsed < publish_period_ms) {
                std::this_thread::sleep_for(std::chrono::milliseconds(publish_period_ms - elapsed));
            }
        }
    });

    // =========================================================================
    // Status Thread
    // =========================================================================

    std::thread status_thread([&]() {
        while (!g_shutdown.load()) {
            SystemStatus status;
            status.start_time = g_startup_time;
            status.uptime_seconds = get_uptime();
            status.cpu_temp = get_cpu_temp();

            // Camera stats
            int connected_count = 0;
            for (int i = 0; i < num_cameras; i++) {
                CameraStatus cam_status;
                cam_status.camera_id = i;

                if (auto* cam = camera_manager.get_camera(i)) {
                    cam_status.connected = cam->is_connected();
                    cam_status.fps = cam->fps();
                    cam_status.frames_captured = cam->frames_captured();
                    cam_status.frames_dropped = cam->frames_dropped();
                    if (cam_status.connected) connected_count++;
                }

                cam_status.avg_total_ms = 1000.0 / std::max(0.1, processing_fps[i].load());
                status.cameras.push_back(cam_status);
            }

            g_cameras_connected.store(connected_count);
            status.fused_pose_valid = pipeline.get_fused_pose().valid;

            // Push to NT and web
            nt_publisher.publish_status(status);
            web_server.push_status(status);

            // Check ready state
            check_ready_state();

            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
    });

    // =========================================================================
    // Wait for init threads
    // =========================================================================

    if (nt_init_thread.joinable()) {
        nt_init_thread.join();
    }
    if (camera_init_thread.joinable()) {
        camera_init_thread.join();
    }

    // =========================================================================
    // Main Loop
    // =========================================================================

    std::cout << "[Main] Running... Press Ctrl+C to stop." << std::endl;
    std::cout << "[Main] Dashboard: http://localhost:" << config.output.web_port << std::endl;
    std::cout << "[Main] Readiness: curl http://localhost:" << config.output.web_port
              << "/api/status" << std::endl;

    while (!g_shutdown.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Print periodic status
        static auto last_print = SteadyClock::now();
        auto now = SteadyClock::now();
        if (std::chrono::duration<double>(now - last_print).count() >= 5.0) {
            std::cout << "[Status] ";
            for (int i = 0; i < num_cameras; i++) {
                if (auto* cam = camera_manager.get_camera(i)) {
                    if (cam->is_connected()) {
                        std::cout << "Cam" << i << ": " << std::fixed << std::setprecision(1)
                                  << cam->fps() << " fps | ";
                    } else {
                        std::cout << "Cam" << i << ": waiting | ";
                    }
                }
            }
            std::cout << "NT: " << (nt_publisher.is_connected() ? "OK" : "retry")
                      << " | Ready: " << (g_ready.load() ? "YES" : "no")
                      << std::endl;
            last_print = now;
        }
    }

    // =========================================================================
    // Shutdown
    // =========================================================================

    std::cout << "[Main] Shutting down..." << std::endl;

    // Stop all threads
    status_thread.join();
    fusion_thread.join();

    for (auto& t : processing_threads) {
        if (t.joinable()) t.join();
    }

    // Stop services
    web_server.stop();
    nt_publisher.stop();
    camera_manager.stop_all();

    std::cout << "[Main] Shutdown complete." << std::endl;
    return 0;
}
