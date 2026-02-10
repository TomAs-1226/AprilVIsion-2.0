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
#include <algorithm>

namespace fs = std::filesystem;

namespace frc_vision {

// Track which devices are already in use to prevent conflicts
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

    // Wait for camera to connect
    auto start = SteadyClock::now();
    while (!connected_.load() && !should_stop_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (SteadyClock::now() - start > std::chrono::seconds(15)) {
            std::cerr << "[Camera " << id_ << "] Timeout waiting for connection (thread still running)" << std::endl;
            break;
        }
    }

    // Always return true - the capture thread keeps running and will connect eventually
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
    // Parse configured device - could be /dev/videoX or just index
    int device_index = -1;
    std::string device_path = config_.device;

    if (device_path.find("/dev/video") == 0) {
        device_index = std::stoi(device_path.substr(10));
    } else {
        try {
            device_index = std::stoi(device_path);
        } catch (...) {
            device_index = id_ * 2;  // Default to even indices
        }
    }

    // Build scan order: configured device first, then all available devices
    std::vector<int> scan_order;
    scan_order.push_back(device_index);

    // Add all even devices (capture devices), then odd (fallback)
    for (int i = 0; i <= 20; i += 2) {
        if (i != device_index) scan_order.push_back(i);
    }
    for (int i = 1; i <= 20; i += 2) {
        if (i != device_index) scan_order.push_back(i);
    }

    for (int idx : scan_order) {
        std::string dev_path = "/dev/video" + std::to_string(idx);
        if (!fs::exists(dev_path)) continue;

        // Check if already claimed by another camera
        {
            std::lock_guard<std::mutex> lock(opened_devices_mutex);
            if (opened_devices.find(idx) != opened_devices.end()) {
                continue;
            }
        }

        // Try to open with V4L2
        cap_.open(idx, cv::CAP_V4L2);
        if (!cap_.isOpened()) {
            cap_.open(idx, cv::CAP_ANY);
        }

        if (cap_.isOpened()) {
            // Claim this device
            {
                std::lock_guard<std::mutex> lock(opened_devices_mutex);
                opened_devices.insert(idx);
            }
            opened_device_index_ = idx;

            if (idx == device_index) {
                std::cout << "[Camera " << id_ << "] Opened configured device /dev/video"
                          << idx << std::endl;
            } else {
                std::cout << "[Camera " << id_ << "] Opened /dev/video"
                          << idx << " (auto-detected)" << std::endl;
            }
            return true;
        }
    }

    std::cerr << "[Camera " << id_ << "] No available camera device found" << std::endl;
    return false;
}

void Camera::configure_camera() {
    if (!cap_.isOpened()) return;

    // Set fourcc for format - do this FIRST before resolution/fps
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

    // Buffer of 2 prevents driver-level drops while keeping latency low.
    // Buffer of 1 causes the V4L2 driver to drop frames under CPU load,
    // which triggers consecutive_failures and false disconnects.
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 2);

    // Configure camera controls via v4l2-ctl for reliable control
    std::string device_path = "/dev/video" + std::to_string(opened_device_index_);

    // Autofocus FIRST - enable before other controls
    {
        std::string cmd = "v4l2-ctl -d " + device_path + " --set-ctrl=focus_auto=1 2>/dev/null";
        if (std::system(cmd.c_str()) != 0) {
            cmd = "v4l2-ctl -d " + device_path + " --set-ctrl=focus_automatic_continuous=1 2>/dev/null";
            if (std::system(cmd.c_str()) != 0) {
                cap_.set(cv::CAP_PROP_AUTOFOCUS, 1);
            }
        }
    }

    if (config_.exposure > 0) {
        // Manual exposure mode
        std::string cmd = "v4l2-ctl -d " + device_path +
            " --set-ctrl=auto_exposure=1 --set-ctrl=exposure_time_absolute=" +
            std::to_string(config_.exposure) + " 2>/dev/null";
        if (std::system(cmd.c_str()) != 0) {
            cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
            cap_.set(cv::CAP_PROP_EXPOSURE, config_.exposure);
        }
        std::cout << "[Camera " << id_ << "] Manual exposure: " << config_.exposure << std::endl;
    } else {
        // Auto exposure - set APERTURE_PRIORITY mode (auto exposure, manual other)
        std::string cmd = "v4l2-ctl -d " + device_path + " --set-ctrl=auto_exposure=3 2>/dev/null";
        if (std::system(cmd.c_str()) != 0) {
            cmd = "v4l2-ctl -d " + device_path + " --set-ctrl=exposure_auto=3 2>/dev/null";
            if (std::system(cmd.c_str()) != 0) {
                cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);
            }
        }

        // Auto white balance
        cmd = "v4l2-ctl -d " + device_path + " --set-ctrl=white_balance_automatic=1 2>/dev/null";
        std::system(cmd.c_str());

        // Auto white balance temperature (let camera decide)
        cmd = "v4l2-ctl -d " + device_path + " --set-ctrl=white_balance_temperature_auto=1 2>/dev/null";
        std::system(cmd.c_str());

        std::cout << "[Camera " << id_ << "] Auto exposure enabled" << std::endl;
    }

    // Disable backlight compensation to prevent white-out
    {
        std::string cmd = "v4l2-ctl -d " + device_path + " --set-ctrl=backlight_compensation=0 2>/dev/null";
        std::system(cmd.c_str());
    }

    // Let the camera's auto algorithms handle brightness/contrast/saturation.
    // Do NOT set fixed values - they override auto-exposure and cause wonky brightness.
    // Only set power_line_frequency to reduce flicker (1=50Hz, 2=60Hz).
    {
        std::string cmd = "v4l2-ctl -d " + device_path + " --set-ctrl=power_line_frequency=2 2>/dev/null";
        std::system(cmd.c_str());
    }

    // Gain
    if (config_.gain >= 0) {
        cap_.set(cv::CAP_PROP_GAIN, config_.gain);
    } else {
        std::string cmd = "v4l2-ctl -d " + device_path + " --set-ctrl=gain_automatic=1 2>/dev/null";
        std::system(cmd.c_str());
    }

    // Verify settings
    double actual_width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
    double actual_height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
    double actual_fps = cap_.get(cv::CAP_PROP_FPS);

    std::cout << "[Camera " << id_ << "] Configured: "
              << actual_width << "x" << actual_height << " @ " << actual_fps << " fps"
              << std::endl;
}

void Camera::capture_loop() {
    running_.store(true);

    cv::Mat frame;
    cv::Mat warmup_frame;

    // Keep trying to open a camera FOREVER until shutdown
    // Never let this thread die - it must always be ready to reconnect
    while (!should_stop_.load()) {
        // Try to open a camera
        if (!open_camera()) {
            std::cout << "[Camera " << id_ << "] No camera found, retrying in 2s..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(2));
            continue;
        }

        configure_camera();

        // Warm up - read and discard frames to flush stale buffers and
        // let auto-exposure settle. Short warmup = faster startup.
        std::cout << "[Camera " << id_ << "] Warming up..." << std::endl;
        for (int i = 0; i < 15 && !should_stop_.load(); i++) {
            cap_.read(warmup_frame);
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }

        connected_.store(true);
        std::cout << "[Camera " << id_ << "] Camera ready on /dev/video"
                  << opened_device_index_ << std::endl;

        last_fps_time_ = SteadyClock::now();
        fps_frame_count_ = 0;
        uint64_t local_frame_count = 0;
        int consecutive_failures = 0;

        // Capture loop - runs until camera disconnects or shutdown
        while (!should_stop_.load()) {
            // grab() can sometimes hang on a dead USB bus. We detect that
            // via the consecutive_failures counter and reconnect.
            if (!cap_.grab()) {
                consecutive_failures++;

                if (consecutive_failures >= 30) {
                    std::cerr << "[Camera " << id_ << "] Camera lost after "
                              << consecutive_failures << " failures, reconnecting..."
                              << std::endl;
                    break;  // Break inner loop to reconnect
                }

                // Short sleep - don't busy-loop on failure
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            consecutive_failures = 0;

            auto capture_time = SteadyClock::now();
            auto capture_wall = SystemClock::now();

            // retrieve() decodes into 'frame'. We then MOVE this mat into
            // the ring buffer frame to avoid a deep copy (clone).
            if (!cap_.retrieve(frame)) continue;

            if (frame.empty() || frame.cols == 0 || frame.rows == 0) {
                empty_frames_dropped_.fetch_add(1);
                continue;
            }

            Frame f;
            f.camera_id = id_;
            f.frame_number = frame_number_++;
            f.capture_time = capture_time;
            f.capture_wall_time = capture_wall;
            // Move the mat data - avoids expensive deep copy.
            // retrieve() will allocate a new buffer on the next call.
            f.image = std::move(frame);

            frame_buffer_.push(std::move(f));
            frames_captured_.fetch_add(1);
            local_frame_count++;

            if (local_frame_count <= 3 || local_frame_count % 2000 == 0) {
                std::cout << "[Camera " << id_ << "] Frame " << local_frame_count
                          << " (" << f.image.cols << "x" << f.image.rows << ")" << std::endl;
            }

            fps_frame_count_++;
            auto now = SteadyClock::now();
            auto elapsed = std::chrono::duration<double>(now - last_fps_time_).count();
            if (elapsed >= 1.0) {
                fps_.store(fps_frame_count_ / elapsed);
                fps_frame_count_ = 0;
                last_fps_time_ = now;
            }
        }

        // Camera disconnected or shutting down - clean up for reconnect
        connected_.store(false);
        cap_.release();

        if (opened_device_index_ >= 0) {
            std::lock_guard<std::mutex> lock(opened_devices_mutex);
            opened_devices.erase(opened_device_index_);
            opened_device_index_ = -1;
        }

        if (!should_stop_.load()) {
            std::cout << "[Camera " << id_ << "] Reconnecting in 1s..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    // Final cleanup
    if (cap_.isOpened()) cap_.release();
    if (opened_device_index_ >= 0) {
        std::lock_guard<std::mutex> lock(opened_devices_mutex);
        opened_devices.erase(opened_device_index_);
        opened_device_index_ = -1;
    }

    running_.store(false);
    connected_.store(false);
}

void Camera::update_config(const CameraConfig& config) {
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

    // Clear global device tracking for fresh start
    {
        std::lock_guard<std::mutex> lock(opened_devices_mutex);
        opened_devices.clear();
    }

    // Start ALL cameras in parallel - each has its own thread that
    // scans for available devices and keeps retrying forever
    for (size_t i = 0; i < configs.size(); i++) {
        auto cam = std::make_unique<Camera>(static_cast<int>(i), configs[i]);
        cam->start();  // Launches capture thread - doesn't block long
        cameras_.push_back(std::move(cam));

        // Small delay between starts so cameras don't race for the same device
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Count how many connected
    int started = 0;
    for (auto& cam : cameras_) {
        if (cam->is_connected()) started++;
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
