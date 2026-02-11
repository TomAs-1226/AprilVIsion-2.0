/**
 * @file web_server.cpp
 * @brief Web server implementation using cpp-httplib
 */

#include "web_server.hpp"
#include "config.hpp"
#include "calibration.hpp"
#include <httplib.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>

namespace frc_vision {

using json = nlohmann::json;

// =============================================================================
// WebServer Implementation
// =============================================================================

class WebServer::Impl {
public:
    httplib::Server server;

    // Latest frames for MJPEG streaming (per camera)
    std::vector<std::unique_ptr<LatestValue<std::vector<uint8_t>>>> latest_frames;

    // Latest data for WebSocket-like SSE
    LatestValue<std::string> latest_detections_json;
    LatestValue<std::string> latest_fused_json;
    LatestValue<std::string> latest_status_json;

    // SSE client management
    std::atomic<int> sse_client_count{0};

    // Current test mode state
    std::atomic<int> current_mode{0}; // 0=normal, 1=calibration, 2=validation, 3=diagnostics
    std::string current_mode_name = "normal";
    std::mutex mode_mutex;

    // Configuration
    int num_cameras = 0;
    std::string web_root;
    ConfigManager* config_manager = nullptr;
};

WebServer::WebServer()
    : impl_(std::make_unique<Impl>())
{
}

WebServer::~WebServer() {
    stop();
}

bool WebServer::initialize(int port, const std::string& web_root,
                           int num_cameras, ConfigManager* config_manager)
{
    port_ = port;
    web_root_ = web_root;
    num_cameras_ = num_cameras;
    config_manager_ = config_manager;

    impl_->num_cameras = num_cameras;
    impl_->web_root = web_root;
    impl_->config_manager = config_manager;
    impl_->latest_frames.reserve(num_cameras);
    for (int i = 0; i < num_cameras; i++) {
        impl_->latest_frames.push_back(std::make_unique<LatestValue<std::vector<uint8_t>>>());
    }

    // ==========================================================================
    // Static file serving
    // ==========================================================================
    if (!impl_->server.set_mount_point("/", web_root.c_str())) {
        std::cerr << "[WebServer] Warning: Could not mount " << web_root << std::endl;
        // Continue anyway - may work with embedded files
    }

    // ==========================================================================
    // MJPEG streams per camera
    // ==========================================================================
    for (int i = 0; i < num_cameras; i++) {
        std::string path = "/cam" + std::to_string(i) + ".mjpeg";

        impl_->server.Get(path.c_str(), [this, i](const httplib::Request& req, httplib::Response& res) {
            std::cout << "[WebServer] MJPEG client connected to cam" << i
                      << " from " << req.remote_addr << std::endl;

            res.set_header("Cache-Control", "no-cache, no-store, must-revalidate");
            res.set_header("Pragma", "no-cache");
            res.set_header("Expires", "0");
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_header("Connection", "keep-alive");

            res.set_content_provider(
                "multipart/x-mixed-replace; boundary=--frame",
                [this, i](size_t /*offset*/, httplib::DataSink& sink) {
                    int frames_sent = 0;
                    int wait_count = 0;
                    const int MAX_WAIT = 600;  // 30 seconds at 50ms intervals
                    uint64_t last_version = 0;

                    // Wait for first frame with timeout
                    while (!should_stop_.load() && wait_count < MAX_WAIT) {
                        auto frame_opt = impl_->latest_frames[i]->get();
                        if (frame_opt && !frame_opt->empty()) {
                            break;
                        }
                        wait_count++;
                        if (wait_count % 20 == 0) {
                            std::cout << "[WebServer] MJPEG cam" << i << " waiting for frames..." << std::endl;
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    }

                    if (wait_count >= MAX_WAIT) {
                        std::cerr << "[WebServer] MJPEG cam" << i << " timeout waiting for frames" << std::endl;
                        return false;
                    }

                    std::cout << "[WebServer] MJPEG cam" << i << " starting stream" << std::endl;

                    // Track last frame for keepalive resend
                    std::vector<uint8_t> last_frame_data;
                    auto last_frame_sent_time = std::chrono::steady_clock::now();

                    while (!should_stop_.load()) {
                        try {
                            auto frame_changed = impl_->latest_frames[i]->get_if_changed(last_version);
                            const std::vector<uint8_t>* frame_to_send = nullptr;

                            if (frame_changed) {
                                const auto& [frame, version] = *frame_changed;
                                last_version = version;
                                if (!frame.empty()) {
                                    last_frame_data = frame;
                                    frame_to_send = &last_frame_data;
                                }
                            } else {
                                // KEY FIX: If no new frame in 1 second, re-send the last frame.
                                // This prevents the browser from considering the MJPEG stream dead
                                // when a camera temporarily stalls (e.g., USB bus contention).
                                // Without this, multi-camera setups freeze in the browser.
                                auto now = std::chrono::steady_clock::now();
                                auto since_last = std::chrono::duration<double>(now - last_frame_sent_time).count();
                                if (since_last > 1.0 && !last_frame_data.empty()) {
                                    frame_to_send = &last_frame_data;
                                }
                            }

                            if (frame_to_send && !frame_to_send->empty()) {
                                std::string header = "--frame\r\n"
                                    "Content-Type: image/jpeg\r\n"
                                    "Content-Length: " + std::to_string(frame_to_send->size()) + "\r\n\r\n";

                                if (!sink.write(header.data(), header.size())) break;
                                if (!sink.write(reinterpret_cast<const char*>(frame_to_send->data()), frame_to_send->size())) break;
                                if (!sink.write("\r\n", 2)) break;

                                last_frame_sent_time = std::chrono::steady_clock::now();
                                frames_sent++;
                                if (frames_sent == 1) {
                                    std::cout << "[WebServer] MJPEG cam" << i << " first frame sent" << std::endl;
                                }
                            }

                            // Poll at ~30fps
                            std::this_thread::sleep_for(std::chrono::milliseconds(33));
                        } catch (const std::exception& e) {
                            std::cerr << "[WebServer] MJPEG cam" << i << " error: " << e.what() << std::endl;
                            break;
                        } catch (...) {
                            std::cerr << "[WebServer] MJPEG cam" << i << " unknown error" << std::endl;
                            break;
                        }
                    }

                    if (frames_sent > 0) {
                        std::cout << "[WebServer] MJPEG cam" << i << " ended after "
                                  << frames_sent << " frames" << std::endl;
                    }
                    return false;
                });
        });
    }

    // ==========================================================================
    // Single frame snapshot endpoints (for debugging)
    // ==========================================================================
    for (int i = 0; i < num_cameras; i++) {
        std::string path = "/cam" + std::to_string(i) + ".jpg";

        impl_->server.Get(path.c_str(), [this, i](const httplib::Request&, httplib::Response& res) {
            res.set_header("Cache-Control", "no-cache");
            res.set_header("Access-Control-Allow-Origin", "*");

            auto frame_opt = impl_->latest_frames[i]->get();
            if (frame_opt && !frame_opt->empty()) {
                res.set_content(
                    std::string(reinterpret_cast<const char*>(frame_opt->data()), frame_opt->size()),
                    "image/jpeg");
            } else {
                res.status = 503;
                res.set_content("{\"error\": \"No frame available for camera " + std::to_string(i) + "\"}", "application/json");
            }
        });
    }

    // Debug endpoint to check frame status
    impl_->server.Get("/api/debug/frames", [this](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");

        json j;
        j["num_cameras"] = impl_->num_cameras;
        for (int i = 0; i < impl_->num_cameras; i++) {
            auto frame_opt = impl_->latest_frames[i]->get();
            j["cameras"][i]["has_frame"] = frame_opt.has_value();
            j["cameras"][i]["frame_size"] = frame_opt ? frame_opt->size() : 0;
        }
        res.set_content(j.dump(), "application/json");
    });

    // ==========================================================================
    // Server-Sent Events for real-time data
    // ==========================================================================
    impl_->server.Get("/events", [this](const httplib::Request&, httplib::Response& res) {
        impl_->sse_client_count.fetch_add(1);

        res.set_header("Content-Type", "text/event-stream");
        res.set_header("Cache-Control", "no-cache");
        res.set_header("Connection", "keep-alive");
        res.set_header("Access-Control-Allow-Origin", "*");

        res.set_content_provider(
            "text/event-stream",
            [this](size_t /*offset*/, httplib::DataSink& sink) {
                uint64_t last_det_version = 0;
                uint64_t last_fused_version = 0;
                uint64_t last_status_version = 0;

                auto last_heartbeat = std::chrono::steady_clock::now();

                while (!should_stop_.load()) {
                    bool sent = false;

                    // Check detections
                    auto det_opt = impl_->latest_detections_json.get_if_changed(last_det_version);
                    if (det_opt) {
                        const auto& [data, version] = *det_opt;
                        last_det_version = version;
                        std::string msg = "event: detections\ndata: " + data + "\n\n";
                        if (!sink.write(msg.data(), msg.size())) break;
                        sent = true;
                    }

                    // Check fused pose
                    auto fused_opt = impl_->latest_fused_json.get_if_changed(last_fused_version);
                    if (fused_opt) {
                        const auto& [data, version] = *fused_opt;
                        last_fused_version = version;
                        std::string msg = "event: fused\ndata: " + data + "\n\n";
                        if (!sink.write(msg.data(), msg.size())) break;
                        sent = true;
                    }

                    // Check status
                    auto status_opt = impl_->latest_status_json.get_if_changed(last_status_version);
                    if (status_opt) {
                        const auto& [data, version] = *status_opt;
                        last_status_version = version;
                        std::string msg = "event: status\ndata: " + data + "\n\n";
                        if (!sink.write(msg.data(), msg.size())) break;
                        sent = true;
                    }

                    // SSE heartbeat every 15s to prevent browser/proxy timeouts
                    auto now_hb = std::chrono::steady_clock::now();
                    if (std::chrono::duration<double>(now_hb - last_heartbeat).count() > 15.0) {
                        std::string heartbeat = ":heartbeat\n\n";
                        if (!sink.write(heartbeat.data(), heartbeat.size())) break;
                        last_heartbeat = now_hb;
                    }

                    if (!sent) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    }
                }

                impl_->sse_client_count.fetch_sub(1);
                return false;
            });
    });

    // ==========================================================================
    // REST API endpoints
    // ==========================================================================

    // Get current configuration
    impl_->server.Get("/api/config", [this](const httplib::Request&, httplib::Response& res) {
        if (!config_manager_) {
            res.status = 500;
            res.set_content("{\"error\": \"No config manager\"}", "application/json");
            return;
        }

        const auto& cfg = config_manager_->get();
        json j;

        // Detector config
        j["detector"]["family"] = cfg.detector.family;
        j["detector"]["decimation"] = cfg.detector.decimation;
        j["detector"]["sigma"] = cfg.detector.sigma;
        j["detector"]["nthreads"] = cfg.detector.nthreads;
        j["detector"]["refine_edges"] = cfg.detector.refine_edges;
        j["detector"]["max_hamming"] = cfg.detector.max_hamming;
        j["detector"]["min_margin"] = cfg.detector.min_margin;
        j["detector"]["max_tags_per_frame"] = cfg.detector.max_tags_per_frame;

        // Tracker config
        j["tracker"]["enable"] = cfg.tracker.enable;
        j["tracker"]["dropout_ms"] = cfg.tracker.dropout_ms;
        j["tracker"]["filter_alpha"] = cfg.tracker.filter_alpha;
        j["tracker"]["roi_enable"] = cfg.tracker.roi_enable;

        // Performance config
        j["performance"]["jpeg_quality"] = cfg.performance.jpeg_quality;
        j["performance"]["enable_annotations"] = cfg.performance.enable_annotations;

        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_content(j.dump(), "application/json");
    });

    // Update configuration
    impl_->server.Post("/api/config", [this](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");

        if (!config_manager_) {
            res.status = 500;
            res.set_content("{\"error\": \"No config manager\"}", "application/json");
            return;
        }

        try {
            json j = json::parse(req.body);
            auto& cfg = config_manager_->get_mutable();

            // Update detector config
            if (j.contains("detector")) {
                auto& det = j["detector"];
                if (det.contains("decimation")) cfg.detector.decimation = det["decimation"];
                if (det.contains("sigma")) cfg.detector.sigma = det["sigma"];
                if (det.contains("nthreads")) cfg.detector.nthreads = det["nthreads"];
                if (det.contains("refine_edges")) cfg.detector.refine_edges = det["refine_edges"];
                if (det.contains("max_hamming")) cfg.detector.max_hamming = det["max_hamming"];
                if (det.contains("min_margin")) cfg.detector.min_margin = det["min_margin"];
                if (det.contains("max_tags_per_frame")) cfg.detector.max_tags_per_frame = det["max_tags_per_frame"];
            }

            // Update tracker config
            if (j.contains("tracker")) {
                auto& trk = j["tracker"];
                if (trk.contains("enable")) cfg.tracker.enable = trk["enable"];
                if (trk.contains("dropout_ms")) cfg.tracker.dropout_ms = trk["dropout_ms"];
                if (trk.contains("filter_alpha")) cfg.tracker.filter_alpha = trk["filter_alpha"];
                if (trk.contains("roi_enable")) cfg.tracker.roi_enable = trk["roi_enable"];
            }

            // Update performance config
            if (j.contains("performance")) {
                auto& perf = j["performance"];
                if (perf.contains("jpeg_quality")) {
                    cfg.performance.jpeg_quality = perf["jpeg_quality"];
                    jpeg_quality_ = cfg.performance.jpeg_quality;
                }
                if (perf.contains("enable_annotations")) cfg.performance.enable_annotations = perf["enable_annotations"];
            }

            // Request config reload for other components
            config_manager_->request_reload();

            res.set_content("{\"status\": \"ok\"}", "application/json");

        } catch (const json::exception& e) {
            res.status = 400;
            res.set_content("{\"error\": \"" + std::string(e.what()) + "\"}", "application/json");
        }
    });

    // Reload configuration from file
    impl_->server.Post("/api/config/reload", [this](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");

        if (!config_manager_) {
            res.status = 500;
            res.set_content("{\"error\": \"No config manager\"}", "application/json");
            return;
        }

        if (config_manager_->reload()) {
            res.set_content("{\"status\": \"ok\"}", "application/json");
        } else {
            res.status = 500;
            res.set_content("{\"error\": \"Reload failed\"}", "application/json");
        }
    });

    // ==========================================================================
    // Readiness / Health Check Endpoint (CRITICAL for boot monitoring)
    // ==========================================================================
    impl_->server.Get("/api/status", [this](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Cache-Control", "no-cache");

        // Get latest status from stored JSON
        auto status_opt = impl_->latest_status_json.get();

        json j;

        if (status_opt) {
            try {
                j = json::parse(*status_opt);
            } catch (...) {
                j = json::object();
            }
        }

        // Determine readiness
        bool cameras_ok = false;
        int cameras_connected = 0;

        if (j.contains("cameras") && j["cameras"].is_array()) {
            for (const auto& cam : j["cameras"]) {
                if (cam.contains("connected") && cam["connected"].get<bool>()) {
                    cameras_connected++;
                    cameras_ok = true;
                }
            }
        }

        bool web_ok = running_.load();
        bool ready = cameras_ok && web_ok;

        // Build response
        json response;
        response["ready"] = ready;
        response["state"] = ready ? "ready" : "warming_up";
        response["uptime"] = j.value("uptime", 0.0);
        response["cameras_connected"] = cameras_connected;
        response["cameras_configured"] = num_cameras_;
        response["web_server"] = web_ok;
        response["sse_clients"] = impl_->sse_client_count.load();

        if (j.contains("cpu_temp")) {
            response["cpu_temp"] = j["cpu_temp"];
        }
        if (j.contains("fused_valid")) {
            response["pose_valid"] = j["fused_valid"];
        }

        // NT connection status
        if (j.contains("nt_connected")) {
            response["nt_connected"] = j["nt_connected"];
        }
        if (j.contains("fused_valid")) {
            response["fused_valid"] = j["fused_valid"];
        }

        // Detailed camera status
        response["cameras"] = json::array();
        if (j.contains("cameras") && j["cameras"].is_array()) {
            for (const auto& cam : j["cameras"]) {
                response["cameras"].push_back({
                    {"id", cam.value("id", -1)},
                    {"connected", cam.value("connected", false)},
                    {"fps", cam.value("fps", 0.0)},
                    {"tags_detected", cam.value("tags_detected", 0)}
                });
            }
        }

        res.set_content(response.dump(), "application/json");
    });

    // Simple health check (just returns 200 if web server is up)
    impl_->server.Get("/health", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_content("{\"status\": \"ok\"}", "application/json");
    });

    // ==========================================================================
    // Accuracy Testing Endpoints
    // ==========================================================================

    // Set reference position for accuracy comparison.
    // POST /api/debug/reference  {"x": 1.5, "y": 3.0, "theta_deg": 0}
    // Place robot at known position, set reference, then see error in real time.
    impl_->server.Post("/api/debug/reference", [this](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        try {
            auto j = json::parse(req.body);
            double x = j.value("x", 0.0);
            double y = j.value("y", 0.0);
            double theta = j.value("theta_deg", 0.0) * M_PI / 180.0;
            if (j.contains("theta")) theta = j["theta"].get<double>();

            set_reference_pose(x, y, theta);

            json response;
            response["status"] = "set";
            response["x"] = x;
            response["y"] = y;
            response["theta_deg"] = theta * 180.0 / M_PI;
            res.set_content(response.dump(), "application/json");
        } catch (const std::exception& e) {
            res.status = 400;
            res.set_content("{\"error\": \"" + std::string(e.what()) + "\"}", "application/json");
        }
    });

    // Clear reference position
    impl_->server.Delete("/api/debug/reference", [this](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        clear_reference_pose();
        res.set_content("{\"status\": \"cleared\"}", "application/json");
    });

    // Get current reference position
    impl_->server.Get("/api/debug/reference", [this](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        json j;
        if (ref_pose_set_.load()) {
            j["set"] = true;
            j["x"] = ref_pose_.x;
            j["y"] = ref_pose_.y;
            j["theta_deg"] = ref_pose_.theta * 180.0 / M_PI;
        } else {
            j["set"] = false;
        }
        res.set_content(j.dump(), "application/json");
    });

    // ==========================================================================
    // Calibration Endpoints
    // ==========================================================================

    // Static calibrator instance for calibration session
    static std::unique_ptr<CameraCalibrator> calibrator;
    static cv::Size calibration_image_size;

    // Start calibration session
    impl_->server.Post("/api/calibration/start", [](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");

        try {
            CalibrationBoardConfig config;

            // Parse optional config from request body
            if (!req.body.empty()) {
                auto j = json::parse(req.body);
                if (j.contains("squares_x")) config.squares_x = j["squares_x"];
                if (j.contains("squares_y")) config.squares_y = j["squares_y"];
                if (j.contains("square_length")) config.square_length = j["square_length"];
                if (j.contains("marker_length")) config.marker_length = j["marker_length"];
            }

            calibrator = std::make_unique<CameraCalibrator>();
            calibrator->initialize(config);

            json response;
            response["status"] = "started";
            response["config"]["squares_x"] = config.squares_x;
            response["config"]["squares_y"] = config.squares_y;
            response["config"]["square_length"] = config.square_length;
            response["config"]["marker_length"] = config.marker_length;
            response["min_frames"] = CameraCalibrator::min_frames();
            response["recommended_frames"] = CameraCalibrator::recommended_frames();

            res.set_content(response.dump(), "application/json");

        } catch (const std::exception& e) {
            res.status = 500;
            res.set_content("{\"error\": \"" + std::string(e.what()) + "\"}", "application/json");
        }
    });

    // Capture calibration frame from camera
    impl_->server.Post("/api/calibration/capture", [this](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");

        if (!calibrator) {
            res.status = 400;
            res.set_content("{\"error\": \"Calibration not started\"}", "application/json");
            return;
        }

        int camera_id = 0;
        if (!req.body.empty()) {
            try {
                auto j = json::parse(req.body);
                if (j.contains("camera_id")) camera_id = j["camera_id"];
            } catch (...) {}
        }

        // Get latest frame from camera
        if (camera_id >= static_cast<int>(impl_->latest_frames.size())) {
            res.status = 400;
            res.set_content("{\"error\": \"Invalid camera ID\"}", "application/json");
            return;
        }

        // We need raw frame, not JPEG - capture directly
        cv::VideoCapture cap;
        cap.open(camera_id, cv::CAP_V4L2);
        if (!cap.isOpened()) {
            cap.open(camera_id, cv::CAP_ANY);
        }

        if (!cap.isOpened()) {
            res.status = 500;
            res.set_content("{\"error\": \"Cannot open camera\"}", "application/json");
            return;
        }

        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

        cv::Mat frame;
        cap.read(frame);
        cap.release();

        if (frame.empty()) {
            res.status = 500;
            res.set_content("{\"error\": \"Failed to capture frame\"}", "application/json");
            return;
        }

        calibration_image_size = frame.size();
        bool success = calibrator->capture_frame(frame);

        json response;
        response["success"] = success;
        response["frames_captured"] = calibrator->captured_frame_count();
        response["frames_needed"] = CameraCalibrator::min_frames();

        res.set_content(response.dump(), "application/json");
    });

    // Get calibration status
    impl_->server.Get("/api/calibration/status", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");

        if (!calibrator) {
            res.set_content("{\"active\": false}", "application/json");
            return;
        }

        json response;
        response["active"] = true;
        response["frames_captured"] = calibrator->captured_frame_count();
        response["min_frames"] = CameraCalibrator::min_frames();
        response["recommended_frames"] = CameraCalibrator::recommended_frames();
        response["ready"] = calibrator->captured_frame_count() >= CameraCalibrator::min_frames();

        res.set_content(response.dump(), "application/json");
    });

    // Compute calibration
    impl_->server.Post("/api/calibration/compute", [](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");

        if (!calibrator) {
            res.status = 400;
            res.set_content("{\"error\": \"Calibration not started\"}", "application/json");
            return;
        }

        if (calibrator->captured_frame_count() < CameraCalibrator::min_frames()) {
            res.status = 400;
            json err;
            err["error"] = "Not enough frames";
            err["frames_captured"] = calibrator->captured_frame_count();
            err["frames_needed"] = CameraCalibrator::min_frames();
            res.set_content(err.dump(), "application/json");
            return;
        }

        auto result = calibrator->compute_calibration(calibration_image_size);

        json response;
        response["success"] = result.success;
        response["rms_error"] = result.rms_error;
        response["frames_used"] = result.frames_used;

        if (result.success) {
            response["intrinsics"]["fx"] = result.intrinsics.fx;
            response["intrinsics"]["fy"] = result.intrinsics.fy;
            response["intrinsics"]["cx"] = result.intrinsics.cx;
            response["intrinsics"]["cy"] = result.intrinsics.cy;
            response["intrinsics"]["width"] = result.intrinsics.width;
            response["intrinsics"]["height"] = result.intrinsics.height;

            // Save to file
            std::string save_path = "config/cam0_intrinsics.yml";
            if (!req.body.empty()) {
                try {
                    auto j = json::parse(req.body);
                    if (j.contains("save_path")) save_path = j["save_path"].get<std::string>();
                } catch (...) {}
            }

            if (calibrator->save_calibration(save_path, result)) {
                response["saved_to"] = save_path;
            }
        } else {
            response["error"] = result.error_message;
        }

        res.set_content(response.dump(), "application/json");
    });

    // Generate calibration board image
    impl_->server.Get("/api/calibration/board", [](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");

        CalibrationBoardConfig config;

        // Parse query params
        if (req.has_param("squares_x")) config.squares_x = std::stoi(req.get_param_value("squares_x"));
        if (req.has_param("squares_y")) config.squares_y = std::stoi(req.get_param_value("squares_y"));

        cv::Mat board = CameraCalibrator::generate_board_image(config, 100);

        std::vector<uint8_t> png;
        cv::imencode(".png", board, png);

        res.set_content(std::string(reinterpret_cast<char*>(png.data()), png.size()), "image/png");
    });

    // Reset calibration
    impl_->server.Post("/api/calibration/reset", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");

        if (calibrator) {
            calibrator->clear();
        }

        res.set_content("{\"status\": \"reset\"}", "application/json");
    });

    // ==========================================================================
    // Testing Mode Endpoints (Calibration, Validation, Diagnostics)
    // ==========================================================================

    impl_->server.Post("/api/mode/calibration", [this](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        {
            std::lock_guard<std::mutex> lock(impl_->mode_mutex);
            impl_->current_mode.store(1);
            impl_->current_mode_name = "calibration";
        }
        std::cout << "[WebServer] Starting calibration mode..." << std::endl;
        res.set_content("{\"success\": true, \"mode\": \"calibration\", \"message\": \"Calibration mode started. Point camera at ChArUco board.\"}", "application/json");
    });

    impl_->server.Post("/api/mode/validation", [this](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        {
            std::lock_guard<std::mutex> lock(impl_->mode_mutex);
            impl_->current_mode.store(2);
            impl_->current_mode_name = "validation";
        }
        std::cout << "[WebServer] Starting validation mode..." << std::endl;
        res.set_content("{\"success\": true, \"mode\": \"validation\", \"message\": \"Validation mode started. Place tag at 1.5m.\"}", "application/json");
    });

    impl_->server.Post("/api/mode/diagnostics", [this](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        {
            std::lock_guard<std::mutex> lock(impl_->mode_mutex);
            impl_->current_mode.store(3);
            impl_->current_mode_name = "diagnostics";
        }
        std::cout << "[WebServer] Running diagnostics..." << std::endl;
        res.set_content("{\"success\": true, \"mode\": \"diagnostics\", \"message\": \"Diagnostics running.\"}", "application/json");
    });

    // Both /api/mode/stop and /api/mode/normal return to normal mode
    auto stop_mode_handler = [this](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        {
            std::lock_guard<std::mutex> lock(impl_->mode_mutex);
            impl_->current_mode.store(0);
            impl_->current_mode_name = "normal";
        }
        std::cout << "[WebServer] Returning to normal mode." << std::endl;
        res.set_content("{\"success\": true, \"mode\": \"normal\", \"message\": \"Returned to normal mode.\"}", "application/json");
    };
    impl_->server.Post("/api/mode/stop", stop_mode_handler);
    impl_->server.Post("/api/mode/normal", stop_mode_handler);

    // GET current mode status
    impl_->server.Get("/api/mode", [this](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        json j;
        {
            std::lock_guard<std::mutex> lock(impl_->mode_mutex);
            j["mode"] = impl_->current_mode_name;
            j["mode_id"] = impl_->current_mode.load();
        }
        res.set_content(j.dump(), "application/json");
    });

    // ==========================================================================
    // Diagnostics Endpoint — full system health report
    // ==========================================================================
    impl_->server.Get("/api/diagnostics", [this](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Cache-Control", "no-cache");

        json diag;
        diag["timestamp"] = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        // Get current status
        auto status_opt = impl_->latest_status_json.get();
        if (status_opt) {
            try {
                json status = json::parse(*status_opt);
                diag["uptime_s"] = status.value("uptime", 0.0);
                diag["cpu_temp_c"] = status.value("cpu_temp", 0.0);
                diag["cpu_usage_pct"] = status.value("cpu_usage", 0.0);
                diag["nt_connected"] = status.value("nt_connected", false);
                diag["fused_valid"] = status.value("fused_valid", false);
                diag["fused_fps"] = status.value("fused_fps", 0.0);

                // Per-camera diagnostics
                diag["cameras"] = json::array();
                if (status.contains("cameras") && status["cameras"].is_array()) {
                    for (const auto& cam : status["cameras"]) {
                        json cam_diag;
                        cam_diag["id"] = cam.value("id", -1);
                        cam_diag["connected"] = cam.value("connected", false);
                        cam_diag["fps"] = cam.value("fps", 0.0);
                        cam_diag["frames_captured"] = cam.value("frames_captured", 0);
                        cam_diag["frames_dropped"] = cam.value("frames_dropped", 0);
                        cam_diag["avg_detect_ms"] = cam.value("avg_detect_ms", 0.0);
                        cam_diag["avg_pose_ms"] = cam.value("avg_pose_ms", 0.0);
                        cam_diag["tags_detected"] = cam.value("tags_detected", 0);

                        // Health assessment
                        bool connected = cam.value("connected", false);
                        double fps = cam.value("fps", 0.0);
                        if (!connected) {
                            cam_diag["health"] = "disconnected";
                            cam_diag["health_detail"] = "Camera not connected or USB device not found";
                        } else if (fps < 1.0) {
                            cam_diag["health"] = "stalled";
                            cam_diag["health_detail"] = "Camera connected but producing <1 FPS - USB bus contention?";
                        } else if (fps < 10.0) {
                            cam_diag["health"] = "degraded";
                            cam_diag["health_detail"] = "Low FPS - check USB bandwidth or CPU load";
                        } else {
                            cam_diag["health"] = "healthy";
                            cam_diag["health_detail"] = "Operating normally";
                        }

                        // Stream health
                        auto frame_opt = impl_->latest_frames[cam.value("id", 0)]->get();
                        cam_diag["stream_has_frame"] = frame_opt.has_value();
                        cam_diag["stream_frame_size"] = frame_opt ? frame_opt->size() : 0;

                        diag["cameras"].push_back(cam_diag);
                    }
                }
            } catch (...) {
                diag["error"] = "Failed to parse status";
            }
        } else {
            diag["error"] = "No status data available";
        }

        // Get latest detection info
        auto det_opt = impl_->latest_detections_json.get();
        if (det_opt) {
            try {
                json det = json::parse(*det_opt);
                diag["last_detection"] = {
                    {"tags_count", det.contains("tags") ? det["tags"].size() : 0},
                    {"latency_ms", det.value("latency_ms", 0.0)},
                    {"has_robot_pose", det.contains("robot_pose")}
                };
            } catch (...) {}
        }

        diag["sse_clients"] = impl_->sse_client_count.load();
        diag["web_server_running"] = running_.load();

        res.set_content(diag.dump(2), "application/json");
    });

    // CORS preflight
    impl_->server.Options(".*", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        res.set_header("Access-Control-Allow-Headers", "Content-Type");
        res.status = 204;
    });

    std::cout << "[WebServer] Initialized on port " << port
              << " with " << num_cameras << " cameras" << std::endl;

    return true;
}

void WebServer::start() {
    if (running_.load()) {
        return;
    }

    should_stop_.store(false);

    server_thread_ = std::thread([this]() {
        running_.store(true);
        std::cout << "[WebServer] Starting on port " << port_ << std::endl;

        if (!impl_->server.listen("0.0.0.0", port_)) {
            std::cerr << "[WebServer] Failed to start server" << std::endl;
        }

        running_.store(false);
    });
}

void WebServer::stop() {
    should_stop_.store(true);
    impl_->server.stop();

    if (server_thread_.joinable()) {
        server_thread_.join();
    }

    running_.store(false);
}

void WebServer::push_frame(int camera_id, const std::vector<uint8_t>& jpeg) {
    static std::vector<uint64_t> push_counts(8, 0);  // Track per camera

    if (camera_id >= 0 && camera_id < static_cast<int>(impl_->latest_frames.size())) {
        impl_->latest_frames[camera_id]->set(jpeg);
        push_counts[camera_id]++;

        // Log first few pushes to verify frames are reaching web server
        if (push_counts[camera_id] <= 3 || push_counts[camera_id] % 500 == 0) {
            std::cout << "[WebServer] Camera " << camera_id << " frame " << push_counts[camera_id]
                      << " received, size: " << jpeg.size() << " bytes" << std::endl;
        }
    }
}

void WebServer::push_detections(const FrameDetections& detections) {
    json j;
    j["camera_id"] = detections.camera_id;
    j["frame_number"] = detections.frame_number;
    j["timestamp"] = std::chrono::duration<double>(
        detections.timestamps.capture_wall.time_since_epoch()).count();
    j["latency_ms"] = detections.timestamps.total_pipeline_ms();

    // Send original image dimensions so JS overlay can scale correctly.
    // The MJPEG stream is 320x240 but corners are in original resolution.
    j["image_width"] = detections.image_width;
    j["image_height"] = detections.image_height;

    j["tags"] = json::array();
    for (const auto& det : detections.detections) {
        json tag;
        tag["id"] = det.id;
        tag["margin"] = det.decision_margin;
        tag["hamming"] = det.hamming;
        tag["confidence"] = det.confidence;

        tag["corners"] = json::array();
        for (const auto& c : det.corners.corners) {
            tag["corners"].push_back({c.x, c.y});
        }

        auto center = det.corners.center();
        tag["center"] = {center.x, center.y};

        tag["ambiguity"] = det.ambiguity;
        if (det.pose_valid) {
            tag["reproj_error"] = det.reprojection_error;
            tag["distance_m"] = det.distance_m;

            // Per-tag PnP pose (tag-to-camera translation gives distance components)
            tag["depth"] = {
                {"x", det.tag_to_camera.position.x},
                {"y", det.tag_to_camera.position.y},
                {"z", det.tag_to_camera.position.z}
            };
        }

        // Pixel size for debug
        double edge_sum = 0;
        for (int i = 0; i < 4; i++) {
            int ni = (i + 1) % 4;
            double dx = det.corners.corners[ni].x - det.corners.corners[i].x;
            double dy = det.corners.corners[ni].y - det.corners.corners[i].y;
            edge_sum += std::sqrt(dx * dx + dy * dy);
        }
        tag["pixel_size"] = edge_sum / 4.0;

        j["tags"].push_back(tag);
    }

    // Latency breakdown
    j["latency"] = {
        {"detect_ms", detections.timestamps.detect_ms()},
        {"pose_ms", detections.timestamps.pose_ms()},
        {"total_ms", detections.timestamps.total_pipeline_ms()}
    };

    if (detections.multi_tag_pose_valid) {
        j["robot_pose"] = {
            {"x", detections.robot_pose_field.x},
            {"y", detections.robot_pose_field.y},
            {"theta", detections.robot_pose_field.theta},
            {"theta_deg", detections.robot_pose_field.theta * 180.0 / M_PI}
        };
        j["reproj_error"] = detections.multi_tag_reproj_error;
        j["tags_used"] = detections.tags_used_for_pose;

        // Quality and standard deviations for RoboRIO pose estimator fusion
        j["quality"] = {
            {"confidence", detections.quality.confidence},
            {"std_dev_x", detections.quality.std_dev_x},
            {"std_dev_y", detections.quality.std_dev_y},
            {"std_dev_theta", detections.quality.std_dev_theta}
        };
    }

    impl_->latest_detections_json.set(j.dump());
}

void WebServer::push_fused_pose(const FusedPose& fused) {
    json j;
    j["valid"] = fused.valid;
    j["timestamp"] = std::chrono::duration<double>(
        SystemClock::now().time_since_epoch()).count();

    j["pose_raw"] = {
        {"x", fused.pose_raw.x},
        {"y", fused.pose_raw.y},
        {"theta", fused.pose_raw.theta},
        {"theta_deg", fused.pose_raw.theta * 180.0 / M_PI}
    };

    j["pose_filtered"] = {
        {"x", fused.pose_filtered.x},
        {"y", fused.pose_filtered.y},
        {"theta", fused.pose_filtered.theta},
        {"theta_deg", fused.pose_filtered.theta * 180.0 / M_PI}
    };

    j["quality"] = {
        {"tag_count", fused.quality.tag_count},
        {"avg_margin", fused.quality.avg_margin},
        {"avg_reproj_error", fused.quality.avg_reproj_error},
        {"confidence", fused.quality.confidence},
        {"std_dev_x", fused.quality.std_dev_x},
        {"std_dev_y", fused.quality.std_dev_y},
        {"std_dev_theta", fused.quality.std_dev_theta}
    };

    j["cameras_contributing"] = fused.cameras_contributing;
    j["total_tags"] = fused.total_tags;

    // Reference pose error for accuracy testing
    if (ref_pose_set_.load()) {
        double dx = fused.pose_filtered.x - ref_pose_.x;
        double dy = fused.pose_filtered.y - ref_pose_.y;
        double dtheta = fused.pose_filtered.theta - ref_pose_.theta;
        // Normalize theta error to [-pi, pi]
        while (dtheta > M_PI) dtheta -= 2 * M_PI;
        while (dtheta < -M_PI) dtheta += 2 * M_PI;

        j["reference"] = {
            {"set", true},
            {"ref_x", ref_pose_.x},
            {"ref_y", ref_pose_.y},
            {"ref_theta", ref_pose_.theta},
            {"ref_theta_deg", ref_pose_.theta * 180.0 / M_PI},
            {"error_x", dx},
            {"error_y", dy},
            {"error_theta", dtheta},
            {"error_theta_deg", dtheta * 180.0 / M_PI},
            {"error_distance", std::sqrt(dx * dx + dy * dy)}
        };
    } else {
        j["reference"] = {{"set", false}};
    }

    impl_->latest_fused_json.set(j.dump());
}

void WebServer::push_status(const SystemStatus& status) {
    json j;
    j["uptime"] = status.uptime_seconds;
    j["cpu_temp"] = status.cpu_temp;
    j["cpu_usage"] = status.cpu_usage;

    j["cameras"] = json::array();
    for (const auto& cam : status.cameras) {
        json cam_json = {
            {"id", cam.camera_id},
            {"connected", cam.connected},
            {"fps", cam.fps},
            {"frames_captured", cam.frames_captured},
            {"frames_dropped", cam.frames_dropped},
            {"avg_detect_ms", cam.avg_detect_ms},
            {"avg_pose_ms", cam.avg_pose_ms},
            {"avg_total_ms", cam.avg_total_ms},
            {"tags_detected", cam.tags_detected}
        };
        // Diagnostics: health state per camera
        cam_json["health"] = cam.connected ? "healthy" : "disconnected";
        if (cam.connected && cam.fps < 5.0) {
            cam_json["health"] = "degraded";
        }
        j["cameras"].push_back(cam_json);
    }

    j["fused_fps"] = status.fused_fps;
    j["fused_valid"] = status.fused_pose_valid;
    j["sse_clients"] = impl_->sse_client_count.load();

    // NT connection status (so JS can display it)
    j["nt_connected"] = status.nt_connected;
    j["nt_connection_count"] = status.nt_connection_count;
    j["nt_server_ip"] = status.nt_server_ip;

    // Current test mode
    {
        std::lock_guard<std::mutex> lock(impl_->mode_mutex);
        j["current_mode"] = impl_->current_mode_name;
    }

    impl_->latest_status_json.set(j.dump());
}

int WebServer::websocket_client_count() const {
    return impl_->sse_client_count.load();
}

void WebServer::set_reference_pose(double x, double y, double theta) {
    ref_pose_.x = x;
    ref_pose_.y = y;
    ref_pose_.theta = theta;
    ref_pose_set_.store(true);
    std::cout << "[WebServer] Reference pose set: (" << x << ", " << y
              << ", " << (theta * 180.0 / M_PI) << " deg)" << std::endl;
}

void WebServer::clear_reference_pose() {
    ref_pose_set_.store(false);
    std::cout << "[WebServer] Reference pose cleared" << std::endl;
}

} // namespace frc_vision
