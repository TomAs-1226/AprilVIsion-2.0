#pragma once
/**
 * @file camera.hpp
 * @brief Multi-camera capture with ring buffer output
 *
 * Each camera runs in its own thread, pushing frames to a SPSC ring buffer.
 * Designed for low-latency "latest frame wins" processing.
 */

#include "types.hpp"
#include "ring_buffer.hpp"
#include <opencv2/videoio.hpp>
#include <thread>
#include <atomic>
#include <memory>
#include <functional>

namespace frc_vision {

/**
 * @brief Single camera capture thread
 */
class Camera {
public:
    using FrameCallback = std::function<void(Frame)>;

    Camera(int id, const CameraConfig& config);
    ~Camera();

    // Non-copyable and non-movable (RingBuffer contains mutex)
    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;
    Camera(Camera&&) = delete;
    Camera& operator=(Camera&&) = delete;

    /**
     * @brief Start capture thread
     * @return true if camera opened successfully
     */
    bool start();

    /**
     * @brief Stop capture thread
     */
    void stop();

    /**
     * @brief Check if camera is running
     */
    bool is_running() const { return running_.load(); }

    /**
     * @brief Check if camera is connected
     */
    bool is_connected() const { return connected_.load(); }

    /**
     * @brief Get camera ID
     */
    int id() const { return id_; }

    /**
     * @brief Get camera name
     */
    const std::string& name() const { return config_.name; }

    /**
     * @brief Get output ring buffer (for consumer to read frames)
     */
    RingBuffer<Frame, 4>& frame_buffer() { return frame_buffer_; }

    /**
     * @brief Get current FPS
     */
    double fps() const { return fps_.load(); }

    /**
     * @brief Get total frames captured
     */
    uint64_t frames_captured() const { return frames_captured_.load(); }

    /**
     * @brief Get total frames dropped
     */
    uint64_t frames_dropped() const { return frame_buffer_.dropped_count(); }

    /**
     * @brief Get camera intrinsics (must be loaded externally)
     */
    const CameraIntrinsics& intrinsics() const { return intrinsics_; }
    void set_intrinsics(const CameraIntrinsics& intr) { intrinsics_ = intr; }

    /**
     * @brief Get camera-to-robot transform
     */
    const Pose3D& camera_to_robot() const { return config_.camera_to_robot; }

    /**
     * @brief Update configuration (for hot reload)
     */
    void update_config(const CameraConfig& config);

private:
    void capture_loop();
    bool open_camera();
    void configure_camera();

    int id_;
    CameraConfig config_;
    CameraIntrinsics intrinsics_;

    cv::VideoCapture cap_;
    RingBuffer<Frame, 4> frame_buffer_;

    std::thread capture_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::atomic<bool> should_stop_{false};

    std::atomic<double> fps_{0.0};
    std::atomic<uint64_t> frames_captured_{0};
    uint64_t frame_number_{0};

    // FPS calculation
    SteadyTimePoint last_fps_time_;
    int fps_frame_count_{0};
};

/**
 * @brief Multi-camera manager
 */
class CameraManager {
public:
    CameraManager();
    ~CameraManager();

    /**
     * @brief Initialize cameras from configuration
     * @param configs Vector of camera configurations
     * @return Number of cameras successfully started
     */
    int initialize(const std::vector<CameraConfig>& configs);

    /**
     * @brief Stop all cameras
     */
    void stop_all();

    /**
     * @brief Get camera by index
     */
    Camera* get_camera(int index);

    /**
     * @brief Get number of cameras
     */
    size_t camera_count() const { return cameras_.size(); }

    /**
     * @brief Get all cameras
     */
    std::vector<std::unique_ptr<Camera>>& cameras() { return cameras_; }

    /**
     * @brief Check if all cameras are running
     */
    bool all_running() const;

    /**
     * @brief Get total frames dropped across all cameras
     */
    uint64_t total_frames_dropped() const;

private:
    std::vector<std::unique_ptr<Camera>> cameras_;
};

} // namespace frc_vision
