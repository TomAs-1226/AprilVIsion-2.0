/**
 * @file camera.cpp
 * @brief Camera capture implementation
 */

#include "camera.hpp"
#include <iostream>
#include <chrono>

namespace frc_vision {

// =============================================================================
// Camera
// =============================================================================

Camera::Camera(int id, const CameraConfig& config)
    : id_(id)
    , config_(config)
    , last_fps_time_(SteadyClock::now())
{
}

Camera::~Camera() {
    stop();
}

bool Camera::start() {
    if (running_.load()) {
        return true;
    }

    should_stop_.store(false);
    capture_thread_ = std::thread(&Camera::capture_loop, this);

    // Wait briefly for camera to connect
    auto start = SteadyClock::now();
    while (!connected_.load() && !should_stop_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (SteadyClock::now() - start > std::chrono::seconds(5)) {
            std::cerr << "[Camera " << id_ << "] Timeout waiting for connection" << std::endl;
            break;
        }
    }

    return connected_.load();
}

void Camera::stop() {
    should_stop_.store(true);

    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }

    if (cap_.isOpened()) {
        cap_.release();
    }

    running_.store(false);
    connected_.store(false);
}

bool Camera::open_camera() {
    // Try different backends in order of preference
    std::vector<int> backends = {
        cv::CAP_V4L2,
        cv::CAP_ANY
    };

    for (int backend : backends) {
        // Parse device - could be /dev/videoX or just index
        int device_index = -1;
        std::string device_path = config_.device;

        if (device_path.find("/dev/video") == 0) {
            device_index = std::stoi(device_path.substr(10));
        } else {
            try {
                device_index = std::stoi(device_path);
            } catch (...) {
                device_index = id_;
            }
        }

        cap_.open(device_index, backend);

        if (cap_.isOpened()) {
            std::cout << "[Camera " << id_ << "] Opened " << device_path
                      << " with backend " << backend << std::endl;
            return true;
        }
    }

    std::cerr << "[Camera " << id_ << "] Failed to open " << config_.device << std::endl;
    return false;
}

void Camera::configure_camera() {
    if (!cap_.isOpened()) return;

    // Set fourcc for format
    if (config_.format == "MJPG" || config_.format == "MJPEG") {
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    } else if (config_.format == "YUYV") {
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    }

    // Set resolution
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, config_.width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height);

    // Set FPS
    cap_.set(cv::CAP_PROP_FPS, config_.fps);

    // Set exposure (if specified)
    if (config_.exposure > 0) {
        cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);  // Manual mode
        cap_.set(cv::CAP_PROP_EXPOSURE, config_.exposure);
    } else {
        cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);  // Auto mode
    }

    // Set gain (if specified)
    if (config_.gain >= 0) {
        cap_.set(cv::CAP_PROP_GAIN, config_.gain);
    }

    // Minimize buffer size for low latency
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

    // Verify settings
    double actual_width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
    double actual_height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
    double actual_fps = cap_.get(cv::CAP_PROP_FPS);

    std::cout << "[Camera " << id_ << "] Configured: "
              << actual_width << "x" << actual_height << " @ " << actual_fps << " fps"
              << std::endl;

    if (actual_width != config_.width || actual_height != config_.height) {
        std::cerr << "[Camera " << id_ << "] Warning: Resolution mismatch, requested "
                  << config_.width << "x" << config_.height << std::endl;
    }
}

void Camera::capture_loop() {
    running_.store(true);

    // Open camera
    if (!open_camera()) {
        running_.store(false);
        return;
    }

    configure_camera();
    connected_.store(true);

    cv::Mat frame;
    last_fps_time_ = SteadyClock::now();
    fps_frame_count_ = 0;

    while (!should_stop_.load()) {
        // Grab frame with minimal latency
        if (!cap_.grab()) {
            std::cerr << "[Camera " << id_ << "] Grab failed, reconnecting..." << std::endl;
            connected_.store(false);

            cap_.release();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            if (open_camera()) {
                configure_camera();
                connected_.store(true);
            }
            continue;
        }

        // Timestamp immediately after grab
        auto capture_time = SteadyClock::now();
        auto capture_wall = SystemClock::now();

        // Retrieve frame
        if (!cap_.retrieve(frame)) {
            continue;
        }

        // Verify frame has actual data (not empty)
        if (frame.empty() || frame.cols == 0 || frame.rows == 0) {
            frames_dropped_.fetch_add(1);
            // Log occasionally to avoid spam
            if (frames_dropped_.load() % 100 == 1) {
                std::cerr << "[Camera " << id_ << "] Warning: Empty frame received" << std::endl;
            }
            continue;
        }

        // Create frame object
        Frame f;
        f.camera_id = id_;
        f.frame_number = frame_number_++;
        f.capture_time = capture_time;
        f.capture_wall_time = capture_wall;

        // Clone frame to avoid buffer reuse issues
        f.image = frame.clone();

        // Push to ring buffer (may drop older frames if consumer is slow)
        frame_buffer_.push(std::move(f));
        frames_captured_.fetch_add(1);

        // Update FPS calculation
        fps_frame_count_++;
        auto now = SteadyClock::now();
        auto elapsed = std::chrono::duration<double>(now - last_fps_time_).count();

        if (elapsed >= 1.0) {
            fps_.store(fps_frame_count_ / elapsed);
            fps_frame_count_ = 0;
            last_fps_time_ = now;
        }
    }

    cap_.release();
    running_.store(false);
    connected_.store(false);
}

void Camera::update_config(const CameraConfig& config) {
    // Only update values that can be changed without reopening
    if (cap_.isOpened()) {
        if (config.exposure != config_.exposure && config.exposure > 0) {
            cap_.set(cv::CAP_PROP_EXPOSURE, config.exposure);
        }
        if (config.gain != config_.gain && config.gain >= 0) {
            cap_.set(cv::CAP_PROP_GAIN, config.gain);
        }
    }
    config_ = config;
}

// =============================================================================
// CameraManager
// =============================================================================

CameraManager::CameraManager() = default;

CameraManager::~CameraManager() {
    stop_all();
}

int CameraManager::initialize(const std::vector<CameraConfig>& configs) {
    stop_all();
    cameras_.clear();

    int started = 0;
    for (size_t i = 0; i < configs.size(); i++) {
        auto cam = std::make_unique<Camera>(static_cast<int>(i), configs[i]);

        if (cam->start()) {
            started++;
            std::cout << "[CameraManager] Camera " << i << " (" << configs[i].name
                      << ") started successfully" << std::endl;
        } else {
            std::cerr << "[CameraManager] Camera " << i << " (" << configs[i].name
                      << ") failed to start" << std::endl;
        }

        cameras_.push_back(std::move(cam));
    }

    return started;
}

void CameraManager::stop_all() {
    for (auto& cam : cameras_) {
        cam->stop();
    }
}

Camera* CameraManager::get_camera(int index) {
    if (index >= 0 && index < static_cast<int>(cameras_.size())) {
        return cameras_[index].get();
    }
    return nullptr;
}

bool CameraManager::all_running() const {
    for (const auto& cam : cameras_) {
        if (!cam->is_running()) {
            return false;
        }
    }
    return !cameras_.empty();
}

uint64_t CameraManager::total_frames_dropped() const {
    uint64_t total = 0;
    for (const auto& cam : cameras_) {
        total += cam->frames_dropped();
    }
    return total;
}

} // namespace frc_vision
