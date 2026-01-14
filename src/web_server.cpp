/**
 * @file web_server.cpp
 * @brief Web server implementation using cpp-httplib
 */

#include "web_server.hpp"
#include "config.hpp"
#include <httplib.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
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
    std::vector<LatestValue<std::vector<uint8_t>>> latest_frames;

    // Latest data for WebSocket-like SSE
    LatestValue<std::string> latest_detections_json;
    LatestValue<std::string> latest_fused_json;
    LatestValue<std::string> latest_status_json;

    // SSE client management
    std::atomic<int> sse_client_count{0};

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
    impl_->latest_frames.resize(num_cameras);

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

        impl_->server.Get(path.c_str(), [this, i](const httplib::Request&, httplib::Response& res) {
            res.set_header("Content-Type", "multipart/x-mixed-replace; boundary=frame");
            res.set_header("Cache-Control", "no-cache");
            res.set_header("Connection", "keep-alive");
            res.set_header("Access-Control-Allow-Origin", "*");

            // Stream frames
            res.set_content_provider(
                "multipart/x-mixed-replace; boundary=frame",
                [this, i](size_t /*offset*/, httplib::DataSink& sink) {
                    uint64_t last_version = 0;

                    while (!should_stop_.load()) {
                        auto frame_opt = impl_->latest_frames[i].get_if_changed(last_version);

                        if (frame_opt) {
                            const auto& [frame, version] = *frame_opt;
                            last_version = version;

                            if (!frame.empty()) {
                                std::string header = "--frame\r\n"
                                    "Content-Type: image/jpeg\r\n"
                                    "Content-Length: " + std::to_string(frame.size()) + "\r\n\r\n";

                                if (!sink.write(header.data(), header.size())) {
                                    return false;
                                }
                                if (!sink.write(reinterpret_cast<const char*>(frame.data()), frame.size())) {
                                    return false;
                                }
                                if (!sink.write("\r\n", 2)) {
                                    return false;
                                }
                            }
                        }

                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }
                    return false;
                });
        });
    }

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

                while (!should_stop_.load()) {
                    bool sent = false;

                    // Check detections
                    auto det_opt = impl_->latest_detections_json.get_if_changed(last_det_version);
                    if (det_opt) {
                        const auto& [data, version] = *det_opt;
                        last_det_version = version;
                        std::string msg = "event: detections\ndata: " + data + "\n\n";
                        if (!sink.write(msg.data(), msg.size())) {
                            break;
                        }
                        sent = true;
                    }

                    // Check fused pose
                    auto fused_opt = impl_->latest_fused_json.get_if_changed(last_fused_version);
                    if (fused_opt) {
                        const auto& [data, version] = *fused_opt;
                        last_fused_version = version;
                        std::string msg = "event: fused\ndata: " + data + "\n\n";
                        if (!sink.write(msg.data(), msg.size())) {
                            break;
                        }
                        sent = true;
                    }

                    // Check status
                    auto status_opt = impl_->latest_status_json.get_if_changed(last_status_version);
                    if (status_opt) {
                        const auto& [data, version] = *status_opt;
                        last_status_version = version;
                        std::string msg = "event: status\ndata: " + data + "\n\n";
                        if (!sink.write(msg.data(), msg.size())) {
                            break;
                        }
                        sent = true;
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
    if (camera_id >= 0 && camera_id < static_cast<int>(impl_->latest_frames.size())) {
        impl_->latest_frames[camera_id].set(jpeg);
    }
}

void WebServer::push_detections(const FrameDetections& detections) {
    json j;
    j["camera_id"] = detections.camera_id;
    j["frame_number"] = detections.frame_number;
    j["timestamp"] = std::chrono::duration<double>(
        detections.timestamps.capture_wall.time_since_epoch()).count();
    j["latency_ms"] = detections.timestamps.total_pipeline_ms();

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

        if (det.pose_valid) {
            tag["reproj_error"] = det.reprojection_error;
        }

        j["tags"].push_back(tag);
    }

    if (detections.multi_tag_pose_valid) {
        j["robot_pose"] = {
            {"x", detections.robot_pose_field.x},
            {"y", detections.robot_pose_field.y},
            {"theta", detections.robot_pose_field.theta},
            {"theta_deg", detections.robot_pose_field.theta * 180.0 / M_PI}
        };
        j["reproj_error"] = detections.multi_tag_reproj_error;
        j["tags_used"] = detections.tags_used_for_pose;
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

    impl_->latest_fused_json.set(j.dump());
}

void WebServer::push_status(const SystemStatus& status) {
    json j;
    j["uptime"] = status.uptime_seconds;
    j["cpu_temp"] = status.cpu_temp;
    j["cpu_usage"] = status.cpu_usage;

    j["cameras"] = json::array();
    for (const auto& cam : status.cameras) {
        j["cameras"].push_back({
            {"id", cam.camera_id},
            {"connected", cam.connected},
            {"fps", cam.fps},
            {"frames_captured", cam.frames_captured},
            {"frames_dropped", cam.frames_dropped},
            {"avg_detect_ms", cam.avg_detect_ms},
            {"avg_pose_ms", cam.avg_pose_ms},
            {"avg_total_ms", cam.avg_total_ms},
            {"tags_detected", cam.tags_detected}
        });
    }

    j["fused_fps"] = status.fused_fps;
    j["fused_valid"] = status.fused_pose_valid;
    j["sse_clients"] = impl_->sse_client_count.load();

    impl_->latest_status_json.set(j.dump());
}

int WebServer::websocket_client_count() const {
    return impl_->sse_client_count.load();
}

} // namespace frc_vision
