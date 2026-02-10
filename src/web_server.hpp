#pragma once
/**
 * @file web_server.hpp
 * @brief Web dashboard with MJPEG streaming and WebSocket data
 *
 * Provides:
 * - MJPEG video streams per camera
 * - WebSocket for real-time detection data
 * - Static file serving for dashboard
 * - REST API for configuration
 */

#include "types.hpp"
#include "ring_buffer.hpp"
#include <thread>
#include <atomic>
#include <memory>
#include <functional>

namespace frc_vision {

// Forward declarations
class ConfigManager;

/**
 * @brief Web server for dashboard and streaming
 */
class WebServer {
public:
    WebServer();
    ~WebServer();

    // Non-copyable
    WebServer(const WebServer&) = delete;
    WebServer& operator=(const WebServer&) = delete;

    /**
     * @brief Initialize web server
     * @param port HTTP port
     * @param web_root Path to static web files
     * @param num_cameras Number of cameras to stream
     * @param config_manager Configuration manager for hot reload
     */
    bool initialize(int port, const std::string& web_root, int num_cameras,
                   ConfigManager* config_manager = nullptr);

    /**
     * @brief Start server in background thread
     */
    void start();

    /**
     * @brief Stop server
     */
    void stop();

    /**
     * @brief Check if server is running
     */
    bool is_running() const { return running_.load(); }

    /**
     * @brief Push new frame for MJPEG streaming
     * @param camera_id Camera index
     * @param jpeg JPEG-encoded frame data
     */
    void push_frame(int camera_id, const std::vector<uint8_t>& jpeg);

    /**
     * @brief Push detection data for WebSocket broadcast
     * @param detections Frame detections to broadcast
     */
    void push_detections(const FrameDetections& detections);

    /**
     * @brief Push fused pose for WebSocket broadcast
     * @param fused Fused pose to broadcast
     */
    void push_fused_pose(const FusedPose& fused);

    /**
     * @brief Push system status for WebSocket broadcast
     * @param status System status to broadcast
     */
    void push_status(const SystemStatus& status);

    /**
     * @brief Set JPEG quality for encoding (1-100)
     */
    void set_jpeg_quality(int quality) { jpeg_quality_ = quality; }

    /**
     * @brief Get number of connected WebSocket clients
     */
    int websocket_client_count() const;

    /**
     * @brief Set reference position for accuracy testing.
     * Place the robot at a known field position and call this.
     * The dashboard will then show error = (measured - reference).
     */
    void set_reference_pose(double x, double y, double theta);

    /**
     * @brief Clear reference position
     */
    void clear_reference_pose();

    /**
     * @brief Check if reference pose is set
     */
    bool has_reference_pose() const { return ref_pose_set_; }

    /**
     * @brief Get reference pose
     */
    Pose2D get_reference_pose() const { return ref_pose_; }

private:
    class Impl;
    std::unique_ptr<Impl> impl_;

    std::thread server_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> should_stop_{false};

    int port_ = 5800;
    std::string web_root_;
    int num_cameras_ = 0;
    int jpeg_quality_ = 70;

    ConfigManager* config_manager_ = nullptr;

    // Reference pose for accuracy testing
    Pose2D ref_pose_;
    std::atomic<bool> ref_pose_set_{false};
};

} // namespace frc_vision
