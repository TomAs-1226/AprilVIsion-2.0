/**
 * @file camera.cpp
 * @brief Camera capture implementation
 */

#include "camera.hpp"
#include <iostream>
#include <chrono>
#include <set>
#include <cstdlib>

namespace frc_vision {

// Track which devices are already in use to prevent fallback conflicts
static std::mutex opened_devices_mutex;
static std::set<int> opened_devices;

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

    // Release device from global tracking
    if (opened_device_index_ >= 0) {
        std::lock_guard<std::mutex> lock(opened_devices_mutex);
        opened_devices.erase(opened_device_index_);
        opened_device_index_ = -1;
    }

    running_.store(false);
    connected_.store(false);
}

bool Camera::open_camera() {
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

    // Check if this device is already in use by another camera
    {
        std::lock_guard<std::mutex> lock(opened_devices_mutex);
        if (opened_devices.find(device_index) != opened_devices.end()) {
            std::cerr << "[Camera " << id_ << "] Device " << device_path
                      << " is already in use by another camera" << std::endl;
            return false;
        }
    }

    // Try V4L2 backend (preferred for Linux cameras)
    cap_.open(device_index, cv::CAP_V4L2);

    if (!cap_.isOpened()) {
        // Fallback to any backend
        cap_.open(device_index, cv::CAP_ANY);
    }

    if (cap_.isOpened()) {
        // Register this device as in use
        {
            std::lock_guard<std::mutex> lock(opened_devices_mutex);
            opened_devices.insert(device_index);
        }
        opened_device_index_ = device_index;

        std::cout << "[Camera " << id_ << "] Opened " << device_path
                  << " (index " << device_index << ")" << std::endl;
        return true;
    }

    std::cerr << "[Camera " << id_ << "] Failed to open " << device_path << std::endl;
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

    // Configure exposure using v4l2-ctl for more reliable control
    std::string device_path = "/dev/video" + std::to_string(opened_device_index_);

    if (config_.exposure > 0) {
        // Manual exposure mode
        std::string cmd = "v4l2-ctl -d " + device_path + " --set-ctrl=auto_exposure=1 --set-ctrl=exposure_time_absolute=" +
                          std::to_string(config_.exposure) + " 2>/dev/null";
        int ret = std::system(cmd.c_str());
        if (ret != 0) {
            // Fallback to OpenCV method
            cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
            cap_.set(cv::CAP_PROP_EXPOSURE, config_.exposure);
        }
        std::cout << "[Camera " << id_ << "] Set manual exposure: " << config_.exposure << std::endl;
    } else {
        // Auto exposure mode - use v4l2-ctl for reliable control
        // auto_exposure: 0=manual, 1=auto, 3=aperture-priority
        std::string cmd = "v4l2-ctl -d " + device_path + " --set-ctrl=auto_exposure=3 2>/dev/null";
        int ret = std::system(cmd.c_str());

        if (ret != 0) {
            // Try alternate control name for some cameras
            cmd = "v4l2-ctl -d " + device_path + " --set-ctrl=exposure_auto=3 2>/dev/null";
            ret = std::system(cmd.c_str());
        }

        if (ret != 0) {
            // Fallback to OpenCV (less reliable)
            cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);
        }

        // Also enable auto white balance for better image quality
        cmd = "v4l2-ctl -d " + device_path + " --set-ctrl=white_balance_automatic=1 2>/dev/null";
        std::system(cmd.c_str());

        std::cout << "[Camera " << id_ << "] Set auto exposure mode" << std::endl;
    }

    // Set gain (if specified)
    if (config_.gain >= 0) {
        cap_.set(cv::CAP_PROP_GAIN, config_.gain);
    } else {
        // Enable auto gain if available
        std::string cmd = "v4l2-ctl -d " + device_path + " --set-ctrl=gain_automatic=1 2>/dev/null";
        std::system(cmd.c_str());
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

    // Open camera with retry
    int open_retries = 0;
    while (!should_stop_.load() && open_retries < 10) {
        if (open_camera()) {
            break;
        }
        open_retries++;
        std::cout << "[Camera " << id_ << "] Retry " << open_retries << "/10..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (!cap_.isOpened()) {
        std::cerr << "[Camera " << id_ << "] Failed to open after retries" << std::endl;
        running_.store(false);
        return;
    }

    configure_camera();

    // Warm up camera - grab a few frames to let auto-exposure settle
    std::cout << "[Camera " << id_ << "] Warming up camera..." << std::endl;
    cv::Mat warmup_frame;
    for (int i = 0; i < 10 && !should_stop_.load(); i++) {
        cap_.read(warmup_frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    connected_.store(true);
    std::cout << "[Camera " << id_ << "] Camera ready" << std::endl;

    cv::Mat frame;
    last_fps_time_ = SteadyClock::now();
    fps_frame_count_ = 0;
    uint64_t local_frame_count = 0;
    auto last_successful_frame = SteadyClock::now();
    int consecutive_failures = 0;

    std::cout << "[Camera " << id_ << "] Capture loop starting..." << std::endl;

    while (!should_stop_.load()) {
        // Grab frame with minimal latency
        if (!cap_.grab()) {
            consecutive_failures++;

            // After 30 consecutive failures (~1 second), try to reconnect
            if (consecutive_failures >= 30) {
                std::cerr << "[Camera " << id_ << "] Too many grab failures, reconnecting..." << std::endl;
                connected_.store(false);
                cap_.release();

                // Release device from tracking so we can re-open it
                if (opened_device_index_ >= 0) {
                    std::lock_guard<std::mutex> lock(opened_devices_mutex);
                    opened_devices.erase(opened_device_index_);
                    opened_device_index_ = -1;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                if (open_camera()) {
                    configure_camera();
                    // Warm up again
                    for (int i = 0; i < 5 && !should_stop_.load(); i++) {
                        cap_.read(warmup_frame);
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    }
                    connected_.store(true);
                    consecutive_failures = 0;
                    std::cout << "[Camera " << id_ << "] Reconnected successfully" << std::endl;
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(33));
            }
            continue;
        }

        consecutive_failures = 0;

        // Timestamp immediately after grab
        auto capture_time = SteadyClock::now();
        auto capture_wall = SystemClock::now();

        // Retrieve frame
        if (!cap_.retrieve(frame)) {
            continue;
        }

        // Verify frame has actual data (not empty)
        if (frame.empty() || frame.cols == 0 || frame.rows == 0) {
            empty_frames_dropped_.fetch_add(1);
            // Log occasionally to avoid spam
            if (empty_frames_dropped_.load() % 100 == 1) {
                std::cerr << "[Camera " << id_ << "] Warning: Empty frame received" << std::endl;
            }
            continue;
        }

        last_successful_frame = capture_time;

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
        local_frame_count++;

        // Log first few frames to verify capture is working
        if (local_frame_count <= 3 || local_frame_count == 100 || local_frame_count % 1000 == 0) {
            std::cout << "[Camera " << id_ << "] Frame " << local_frame_count
                      << " captured (" << frame.cols << "x" << frame.rows << ")"
                      << ", buffer size: " << frame_buffer_.size() << std::endl;
        }

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

    // Release device from tracking
    if (opened_device_index_ >= 0) {
        std::lock_guard<std::mutex> lock(opened_devices_mutex);
        opened_devices.erase(opened_device_index_);
        opened_device_index_ = -1;
    }

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
