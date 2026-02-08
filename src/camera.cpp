/**
 * @file camera.cpp
 * @brief Camera capture implementation
 */

#include "camera.hpp"
#include <iostream>
#include <chrono>
#include <set>
#include <cstdlib>
#include <filesystem>

namespace fs = std::filesystem;

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

    // Wait for camera to connect (longer timeout for USB cameras with autofocus settling)
    auto start = SteadyClock::now();
    while (!connected_.load() && !should_stop_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (SteadyClock::now() - start > std::chrono::seconds(15)) {
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

bool Camera::try_open_device(int device_index) {
    // Check if this device is already in use by another camera
    {
        std::lock_guard<std::mutex> lock(opened_devices_mutex);
        if (opened_devices.find(device_index) != opened_devices.end()) {
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
        // Verify it's a real capture device by trying to grab a frame
        cv::Mat test;
        if (!cap_.read(test) || test.empty()) {
            cap_.release();
            return false;
        }

        // Register this device as in use
        {
            std::lock_guard<std::mutex> lock(opened_devices_mutex);
            opened_devices.insert(device_index);
        }
        opened_device_index_ = device_index;
        return true;
    }

    return false;
}

bool Camera::open_camera() {
    // Parse configured device - could be /dev/videoX or just index
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

    // First try the configured device
    if (try_open_device(device_index)) {
        std::cout << "[Camera " << id_ << "] Opened configured device " << device_path
                  << " (index " << device_index << ")" << std::endl;
        return true;
    }

    // Plug-and-play fallback: scan for available capture devices
    // USB cameras can get different /dev/video* numbers each boot
    std::cout << "[Camera " << id_ << "] Configured device " << device_path
              << " not available, scanning for cameras..." << std::endl;

    // Scan even-numbered devices first (USB cameras typically use even indices,
    // odd indices are metadata devices)
    std::vector<int> scan_order;
    for (int i = 0; i <= 20; i += 2) {
        if (i != device_index) scan_order.push_back(i);
    }
    // Then try odd ones as fallback
    for (int i = 1; i <= 20; i += 2) {
        if (i != device_index) scan_order.push_back(i);
    }

    for (int idx : scan_order) {
        // Only try devices that exist in /dev
        std::string dev_path = "/dev/video" + std::to_string(idx);
        if (!fs::exists(dev_path)) continue;

        if (try_open_device(idx)) {
            std::cout << "[Camera " << id_ << "] Found available camera at /dev/video"
                      << idx << " (plug-and-play)" << std::endl;
            return true;
        }
    }

    std::cerr << "[Camera " << id_ << "] No available camera devices found" << std::endl;
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

    // Enable autofocus - critical to avoid white/blurry images
    {
        std::string cmd = "v4l2-ctl -d " + device_path + " --set-ctrl=focus_auto=1 2>/dev/null";
        int ret = std::system(cmd.c_str());
        if (ret != 0) {
            // Try alternate control name used by some cameras
            cmd = "v4l2-ctl -d " + device_path + " --set-ctrl=focus_automatic_continuous=1 2>/dev/null";
            ret = std::system(cmd.c_str());
        }
        if (ret == 0) {
            std::cout << "[Camera " << id_ << "] Autofocus enabled" << std::endl;
        } else {
            // Try OpenCV property as last resort
            cap_.set(cv::CAP_PROP_AUTOFOCUS, 1);
            std::cout << "[Camera " << id_ << "] Autofocus set via OpenCV fallback" << std::endl;
        }
    }

    // Disable backlight compensation to prevent white-out
    {
        std::string cmd = "v4l2-ctl -d " + device_path + " --set-ctrl=backlight_compensation=0 2>/dev/null";
        std::system(cmd.c_str());
    }

    // Set brightness and contrast to balanced defaults
    {
        std::string cmd = "v4l2-ctl -d " + device_path + " --set-ctrl=brightness=128 --set-ctrl=contrast=128 --set-ctrl=saturation=128 2>/dev/null";
        std::system(cmd.c_str());
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

    // Warm up camera - grab frames to let auto-exposure and autofocus settle
    std::cout << "[Camera " << id_ << "] Warming up camera (auto-exposure + autofocus settling)..." << std::endl;
    cv::Mat warmup_frame;
    for (int i = 0; i < 30 && !should_stop_.load(); i++) {
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

            // After 15 consecutive failures (~500ms), try to reconnect quickly
            // This enables fast plug-and-play: camera unplugged -> detected in <1s
            if (consecutive_failures >= 15) {
                std::cerr << "[Camera " << id_ << "] Camera disconnected, scanning for devices..." << std::endl;
                connected_.store(false);
                cap_.release();

                // Release device from tracking so we (or another camera) can re-open it
                if (opened_device_index_ >= 0) {
                    std::lock_guard<std::mutex> lock(opened_devices_mutex);
                    opened_devices.erase(opened_device_index_);
                    opened_device_index_ = -1;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(500));

                // open_camera() will try configured device first, then scan all available
                if (open_camera()) {
                    configure_camera();
                    // Warm up again (more frames for autofocus settling)
                    for (int i = 0; i < 15 && !should_stop_.load(); i++) {
                        cap_.read(warmup_frame);
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    }
                    connected_.store(true);
                    consecutive_failures = 0;
                    std::cout << "[Camera " << id_ << "] Reconnected successfully" << std::endl;
                } else {
                    // Wait a bit before retrying to avoid busy-loop
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    consecutive_failures = 0;  // Reset to try again
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
