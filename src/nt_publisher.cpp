/**
 * @file nt_publisher.cpp
 * @brief NetworkTables 4 publisher implementation
 */

#include "nt_publisher.hpp"
#include "pose.hpp"
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/StringTopic.h>
#include <iostream>
#include <chrono>
#include <vector>

namespace frc_vision {

// Publisher structures to avoid header pollution
struct NTPublisher::CameraPublishers {
    nt::DoublePublisher timestamp_capture;
    nt::DoublePublisher timestamp_publish;
    nt::DoublePublisher latency_ms;

    nt::IntegerArrayPublisher tag_ids;
    nt::DoubleArrayPublisher corners_px;  // [id, x1,y1,x2,y2,x3,y3,x4,y4, ...]
    nt::DoubleArrayPublisher margins;

    // Camera pose (in field frame)
    nt::DoubleArrayPublisher pose_cam;    // [x,y,z, qw,qx,qy,qz]
    nt::DoubleArrayPublisher pose_robot;  // [x, y, theta]

    // Quality metrics
    nt::IntegerPublisher tag_count;
    nt::DoublePublisher avg_margin;
    nt::DoublePublisher avg_reproj_error;
    nt::DoubleArrayPublisher std_devs;    // [x, y, theta]
    nt::DoublePublisher confidence;

    nt::BooleanPublisher pose_valid;
};

struct NTPublisher::FusedPublishers {
    nt::DoublePublisher timestamp;
    nt::DoubleArrayPublisher pose;        // [x, y, theta]
    nt::DoubleArrayPublisher std_devs;    // [x, y, theta]

    nt::IntegerPublisher tag_count;
    nt::IntegerPublisher cameras_contributing;
    nt::DoublePublisher confidence;
    nt::BooleanPublisher valid;
};

struct NTPublisher::StatusPublishers {
    nt::DoublePublisher uptime;
    nt::DoublePublisher cpu_temp;
    nt::DoublePublisher cpu_usage;

    std::vector<nt::DoublePublisher> cam_fps;
    std::vector<nt::IntegerPublisher> cam_dropped;
    std::vector<nt::BooleanPublisher> cam_connected;
};

NTPublisher::NTPublisher() = default;

NTPublisher::~NTPublisher() {
    stop();
}

bool NTPublisher::initialize(const std::string& server, const std::string& table_root, int num_cameras) {
    try {
        table_root_ = table_root;
        num_cameras_ = num_cameras;

        // Create NT instance
        nt_instance_ = std::make_unique<nt::NetworkTableInstance>(
            nt::NetworkTableInstance::GetDefault());

        // Set identity
        nt_instance_->SetServerTeam(0);  // Will be overridden by server address
        nt_instance_->StartClient4("FRC-Vision-Coprocessor");

        // Parse server address
        // Could be "10.TE.AM.2" or "roborio-TEAM-frc.local"
        if (server.find('.') != std::string::npos) {
            // IP address
            nt_instance_->SetServer(server.c_str(), 5810);  // NT4 port
        } else {
            // Hostname - try mDNS
            nt_instance_->SetServer(server.c_str());
        }

        // Initialize data arrays
        latest_detections_.resize(num_cameras);

        // Setup publishers
        setup_publishers();

        std::cout << "[NT] Initialized, connecting to " << server << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[NT] Initialization error: " << e.what() << std::endl;
        return false;
    }
}

void NTPublisher::setup_publishers() {
    auto table = nt_instance_->GetTable(table_root_);

    // Per-camera publishers
    camera_pubs_.clear();
    for (int i = 0; i < num_cameras_; i++) {
        auto cam_table = table->GetSubTable("cam" + std::to_string(i));
        auto pubs = std::make_unique<CameraPublishers>();

        pubs->timestamp_capture = cam_table->GetDoubleTopic("timestamp_capture").Publish();
        pubs->timestamp_publish = cam_table->GetDoubleTopic("timestamp_publish").Publish();
        pubs->latency_ms = cam_table->GetDoubleTopic("latency_ms").Publish();

        pubs->tag_ids = cam_table->GetIntegerArrayTopic("tag_ids").Publish();
        pubs->corners_px = cam_table->GetDoubleArrayTopic("corners_px").Publish();
        pubs->margins = cam_table->GetDoubleArrayTopic("margins").Publish();

        pubs->pose_cam = cam_table->GetDoubleArrayTopic("pose_cam").Publish();
        pubs->pose_robot = cam_table->GetDoubleArrayTopic("pose_robot").Publish();

        pubs->tag_count = cam_table->GetIntegerTopic("tag_count").Publish();
        pubs->avg_margin = cam_table->GetDoubleTopic("avg_margin").Publish();
        pubs->avg_reproj_error = cam_table->GetDoubleTopic("avg_reproj_error").Publish();
        pubs->std_devs = cam_table->GetDoubleArrayTopic("std_devs").Publish();
        pubs->confidence = cam_table->GetDoubleTopic("confidence").Publish();
        pubs->pose_valid = cam_table->GetBooleanTopic("pose_valid").Publish();

        camera_pubs_.push_back(std::move(pubs));
    }

    // Fused pose publishers (filtered)
    auto fused_table = table->GetSubTable("fused");
    fused_pubs_ = std::make_unique<FusedPublishers>();
    fused_pubs_->timestamp = fused_table->GetDoubleTopic("timestamp").Publish();
    fused_pubs_->pose = fused_table->GetDoubleArrayTopic("pose").Publish();
    fused_pubs_->std_devs = fused_table->GetDoubleArrayTopic("std_devs").Publish();
    fused_pubs_->tag_count = fused_table->GetIntegerTopic("tag_count").Publish();
    fused_pubs_->cameras_contributing = fused_table->GetIntegerTopic("cameras_contributing").Publish();
    fused_pubs_->confidence = fused_table->GetDoubleTopic("confidence").Publish();
    fused_pubs_->valid = fused_table->GetBooleanTopic("valid").Publish();

    // Fused raw publishers
    auto fused_raw_table = table->GetSubTable("fused_raw");
    fused_raw_pubs_ = std::make_unique<FusedPublishers>();
    fused_raw_pubs_->timestamp = fused_raw_table->GetDoubleTopic("timestamp").Publish();
    fused_raw_pubs_->pose = fused_raw_table->GetDoubleArrayTopic("pose").Publish();
    fused_raw_pubs_->std_devs = fused_raw_table->GetDoubleArrayTopic("std_devs").Publish();
    fused_raw_pubs_->tag_count = fused_raw_table->GetIntegerTopic("tag_count").Publish();
    fused_raw_pubs_->cameras_contributing = fused_raw_table->GetIntegerTopic("cameras_contributing").Publish();
    fused_raw_pubs_->confidence = fused_raw_table->GetDoubleTopic("confidence").Publish();
    fused_raw_pubs_->valid = fused_raw_table->GetBooleanTopic("valid").Publish();

    // Status publishers
    auto status_table = table->GetSubTable("status");
    status_pubs_ = std::make_unique<StatusPublishers>();
    status_pubs_->uptime = status_table->GetDoubleTopic("uptime").Publish();
    status_pubs_->cpu_temp = status_table->GetDoubleTopic("cpu_temp").Publish();
    status_pubs_->cpu_usage = status_table->GetDoubleTopic("cpu_usage").Publish();

    for (int i = 0; i < num_cameras_; i++) {
        status_pubs_->cam_fps.push_back(
            status_table->GetDoubleTopic("cam" + std::to_string(i) + "_fps").Publish());
        status_pubs_->cam_dropped.push_back(
            status_table->GetIntegerTopic("cam" + std::to_string(i) + "_dropped").Publish());
        status_pubs_->cam_connected.push_back(
            status_table->GetBooleanTopic("cam" + std::to_string(i) + "_connected").Publish());
    }
}

void NTPublisher::start(int rate_hz) {
    if (running_.load()) {
        return;
    }

    publish_rate_hz_ = rate_hz;
    should_stop_.store(false);
    publish_thread_ = std::thread(&NTPublisher::publish_loop, this);
}

void NTPublisher::stop() {
    should_stop_.store(true);

    if (publish_thread_.joinable()) {
        publish_thread_.join();
    }

    running_.store(false);

    if (nt_instance_) {
        nt_instance_->StopClient();
    }
}

bool NTPublisher::is_connected() const {
    if (!nt_instance_) {
        return false;
    }
    return nt_instance_->IsConnected();
}

void NTPublisher::publish_camera(int camera_id, const FrameDetections& detections) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (camera_id >= 0 && camera_id < static_cast<int>(latest_detections_.size())) {
        latest_detections_[camera_id] = detections;
    }
}

void NTPublisher::publish_fused(const FusedPose& fused) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_fused_ = fused;
}

void NTPublisher::publish_status(const SystemStatus& status) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_status_ = status;
}

void NTPublisher::publish_loop() {
    running_.store(true);

    auto period = std::chrono::microseconds(1000000 / publish_rate_hz_);
    bool was_connected = false;
    auto last_connection_log = SteadyClock::now();

    while (!should_stop_.load()) {
        auto start = SteadyClock::now();

        // Log connection status changes (non-blocking)
        bool now_connected = is_connected();
        if (now_connected != was_connected) {
            if (now_connected) {
                std::cout << "[NT] Connected to roboRIO" << std::endl;
            } else {
                std::cout << "[NT] Disconnected from roboRIO, retrying..." << std::endl;
            }
            was_connected = now_connected;
        }

        // Periodic connection retry logging (every 10 seconds if disconnected)
        if (!now_connected) {
            auto elapsed_since_log = std::chrono::duration<double>(
                SteadyClock::now() - last_connection_log).count();
            if (elapsed_since_log >= 10.0) {
                std::cout << "[NT] Still trying to connect..." << std::endl;
                last_connection_log = SteadyClock::now();
            }
        }

        // Get latest data
        std::vector<FrameDetections> detections;
        FusedPose fused;
        SystemStatus status;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            detections = latest_detections_;
            fused = latest_fused_;
            status = latest_status_;
        }

        // Publish even if disconnected - ntcore buffers and will send when connected
        // This is non-blocking and won't cause hangs

        // Publish camera data
        for (size_t i = 0; i < detections.size() && i < camera_pubs_.size(); i++) {
            auto& det = detections[i];
            auto& pub = camera_pubs_[i];

            // Timestamps (seconds since epoch for roboRIO fusion)
            pub->timestamp_capture.Set(
                std::chrono::duration<double>(det.timestamps.capture_wall.time_since_epoch()).count());
            pub->timestamp_publish.Set(
                std::chrono::duration<double>(SystemClock::now().time_since_epoch()).count());
            pub->latency_ms.Set(det.timestamps.total_pipeline_ms());

            // Tag IDs
            std::vector<int64_t> ids;
            std::vector<double> corners;
            std::vector<double> margins;

            for (const auto& tag : det.detections) {
                ids.push_back(tag.id);
                margins.push_back(tag.decision_margin);

                // Flatten corners [id, x1,y1,x2,y2,x3,y3,x4,y4]
                corners.push_back(static_cast<double>(tag.id));
                for (const auto& c : tag.corners.corners) {
                    corners.push_back(c.x);
                    corners.push_back(c.y);
                }
            }

            pub->tag_ids.Set(ids);
            pub->corners_px.Set(corners);
            pub->margins.Set(margins);

            // Camera pose
            if (det.multi_tag_pose_valid) {
                auto& cam_pose = det.camera_to_field;
                pub->pose_cam.Set(std::vector<double>{
                    cam_pose.position.x, cam_pose.position.y, cam_pose.position.z,
                    cam_pose.orientation.w, cam_pose.orientation.x,
                    cam_pose.orientation.y, cam_pose.orientation.z
                });

                pub->pose_robot.Set(std::vector<double>{
                    det.robot_pose_field.x,
                    det.robot_pose_field.y,
                    det.robot_pose_field.theta
                });
            } else {
                pub->pose_cam.Set(std::vector<double>{});
                pub->pose_robot.Set(std::vector<double>{});
            }

            // Quality metrics
            pub->tag_count.Set(static_cast<int>(det.detections.size()));
            pub->avg_margin.Set(det.avg_margin());
            pub->avg_reproj_error.Set(det.avg_reproj_error());
            pub->pose_valid.Set(det.multi_tag_pose_valid);

            // Compute quality and std devs
            auto quality = PoseEstimator::compute_quality(det.detections, det.multi_tag_reproj_error);
            pub->std_devs.Set(std::vector<double>{quality.std_dev_x, quality.std_dev_y, quality.std_dev_theta});
            pub->confidence.Set(quality.confidence);
        }

        // Publish fused pose (filtered)
        fused_pubs_->timestamp.Set(
            std::chrono::duration<double>(SystemClock::now().time_since_epoch()).count());
        fused_pubs_->pose.Set(std::vector<double>{
            fused.pose_filtered.x,
            fused.pose_filtered.y,
            fused.pose_filtered.theta
        });
        fused_pubs_->std_devs.Set(std::vector<double>{
            fused.quality.std_dev_x,
            fused.quality.std_dev_y,
            fused.quality.std_dev_theta
        });
        fused_pubs_->tag_count.Set(fused.total_tags);
        fused_pubs_->cameras_contributing.Set(fused.cameras_contributing);
        fused_pubs_->confidence.Set(fused.quality.confidence);
        fused_pubs_->valid.Set(fused.valid);

        // Publish fused raw pose
        fused_raw_pubs_->timestamp.Set(
            std::chrono::duration<double>(SystemClock::now().time_since_epoch()).count());
        fused_raw_pubs_->pose.Set(std::vector<double>{
            fused.pose_raw.x,
            fused.pose_raw.y,
            fused.pose_raw.theta
        });
        fused_raw_pubs_->std_devs.Set(std::vector<double>{
            fused.quality.std_dev_x,
            fused.quality.std_dev_y,
            fused.quality.std_dev_theta
        });
        fused_raw_pubs_->tag_count.Set(fused.total_tags);
        fused_raw_pubs_->cameras_contributing.Set(fused.cameras_contributing);
        fused_raw_pubs_->confidence.Set(fused.quality.confidence);
        fused_raw_pubs_->valid.Set(fused.valid);

        // Publish status
        status_pubs_->uptime.Set(status.uptime_seconds);
        status_pubs_->cpu_temp.Set(status.cpu_temp);
        status_pubs_->cpu_usage.Set(status.cpu_usage);

        for (size_t i = 0; i < status.cameras.size() && i < status_pubs_->cam_fps.size(); i++) {
            status_pubs_->cam_fps[i].Set(status.cameras[i].fps);
            status_pubs_->cam_dropped[i].Set(static_cast<int64_t>(status.cameras[i].frames_dropped));
            status_pubs_->cam_connected[i].Set(status.cameras[i].connected);
        }

        // Flush NT
        nt_instance_->Flush();

        // Sleep for remainder of period
        auto elapsed = SteadyClock::now() - start;
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }

    running_.store(false);
}

void NTPublisher::flush() {
    if (nt_instance_) {
        nt_instance_->Flush();
    }
}

int64_t NTPublisher::to_nt_timestamp(SystemTimePoint time) const {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        time.time_since_epoch()).count();
}

} // namespace frc_vision
