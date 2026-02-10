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
#include <cmath>

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
    nt::DoublePublisher latency_ms;       // Processing latency for odometry sync
    nt::DoubleArrayPublisher pose;        // [x, y, theta]
    nt::DoubleArrayPublisher std_devs;    // [x, y, theta]

    nt::IntegerPublisher tag_count;
    nt::IntegerPublisher cameras_contributing;
    nt::DoublePublisher confidence;
    nt::BooleanPublisher valid;
    nt::IntegerPublisher heartbeat;       // Incrementing counter for connection check
};

struct NTPublisher::StatusPublishers {
    nt::DoublePublisher uptime;
    nt::DoublePublisher cpu_temp;
    nt::DoublePublisher cpu_usage;

    std::vector<nt::DoublePublisher> cam_fps;
    std::vector<nt::IntegerPublisher> cam_dropped;
    std::vector<nt::BooleanPublisher> cam_connected;

    // NEW: NetworkTables connection status
    nt::BooleanPublisher nt_connected;
    nt::IntegerPublisher nt_connection_count;
    nt::StringPublisher nt_server_ip;
};

// Auto-alignment publishers/subscribers
struct NTPublisher::AlignPublishers {
    // Subscribers (robot -> vision)
    nt::IntegerSubscriber target_tag_id_sub;
    nt::DoubleArraySubscriber target_offset_sub;  // [distance_m, angle_rad]

    // Publishers (vision -> robot)
    nt::BooleanPublisher target_visible;
    nt::DoubleArrayPublisher target_pose;    // [x, y, theta] - target pose for alignment
    nt::DoubleArrayPublisher robot_pose;     // [x, y, theta] - current vision pose
    nt::DoubleArrayPublisher error;          // [x, y, theta] - alignment error
    nt::DoublePublisher distance_m;          // Distance to target
    nt::DoublePublisher angle_error_rad;     // Rotation error
    nt::BooleanPublisher ready;              // Within tolerance
    nt::BooleanPublisher has_target;         // Target is set
    nt::IntegerPublisher current_target_id;  // Echo back current target
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
    fused_pubs_->latency_ms = fused_table->GetDoubleTopic("latency_ms").Publish();
    fused_pubs_->pose = fused_table->GetDoubleArrayTopic("pose").Publish();
    fused_pubs_->std_devs = fused_table->GetDoubleArrayTopic("std_devs").Publish();
    fused_pubs_->tag_count = fused_table->GetIntegerTopic("tag_count").Publish();
    fused_pubs_->cameras_contributing = fused_table->GetIntegerTopic("cameras_contributing").Publish();
    fused_pubs_->confidence = fused_table->GetDoubleTopic("confidence").Publish();
    fused_pubs_->valid = fused_table->GetBooleanTopic("valid").Publish();
    fused_pubs_->heartbeat = fused_table->GetIntegerTopic("heartbeat").Publish();

    // Fused raw publishers
    auto fused_raw_table = table->GetSubTable("fused_raw");
    fused_raw_pubs_ = std::make_unique<FusedPublishers>();
    fused_raw_pubs_->timestamp = fused_raw_table->GetDoubleTopic("timestamp").Publish();
    fused_raw_pubs_->latency_ms = fused_raw_table->GetDoubleTopic("latency_ms").Publish();
    fused_raw_pubs_->pose = fused_raw_table->GetDoubleArrayTopic("pose").Publish();
    fused_raw_pubs_->std_devs = fused_raw_table->GetDoubleArrayTopic("std_devs").Publish();
    fused_raw_pubs_->tag_count = fused_raw_table->GetIntegerTopic("tag_count").Publish();
    fused_raw_pubs_->cameras_contributing = fused_raw_table->GetIntegerTopic("cameras_contributing").Publish();
    fused_raw_pubs_->confidence = fused_raw_table->GetDoubleTopic("confidence").Publish();
    fused_raw_pubs_->valid = fused_raw_table->GetBooleanTopic("valid").Publish();
    fused_raw_pubs_->heartbeat = fused_raw_table->GetIntegerTopic("heartbeat").Publish();

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

    // NEW: NetworkTables connection status publishers
    status_pubs_->nt_connected = status_table->GetBooleanTopic("nt_connected").Publish();
    status_pubs_->nt_connection_count = status_table->GetIntegerTopic("nt_connection_count").Publish();
    status_pubs_->nt_server_ip = status_table->GetStringTopic("nt_server_ip").Publish();

    // Auto-alignment publishers/subscribers
    auto align_table = table->GetSubTable("auto_align");
    align_pubs_ = std::make_unique<AlignPublishers>();

    // Subscribers - robot sets these to request alignment
    align_pubs_->target_tag_id_sub = align_table->GetIntegerTopic("target_tag_id").Subscribe(-1);
    std::vector<double> default_offset = {0.5, 0.0};
    align_pubs_->target_offset_sub = align_table->GetDoubleArrayTopic("target_offset").Subscribe(default_offset);

    // Publishers - vision publishes alignment data
    align_pubs_->target_visible = align_table->GetBooleanTopic("target_visible").Publish();
    align_pubs_->target_pose = align_table->GetDoubleArrayTopic("target_pose").Publish();
    align_pubs_->robot_pose = align_table->GetDoubleArrayTopic("robot_pose").Publish();
    align_pubs_->error = align_table->GetDoubleArrayTopic("error").Publish();
    align_pubs_->distance_m = align_table->GetDoubleTopic("distance_m").Publish();
    align_pubs_->angle_error_rad = align_table->GetDoubleTopic("angle_error_rad").Publish();
    align_pubs_->ready = align_table->GetBooleanTopic("ready").Publish();
    align_pubs_->has_target = align_table->GetBooleanTopic("has_target").Publish();
    align_pubs_->current_target_id = align_table->GetIntegerTopic("current_target_id").Publish();

    // Subscribe to RoboRIO odometry for innovation gating.
    // The robot publishes its odometry pose so the coprocessor can reject
    // vision measurements that disagree too strongly with where the robot is.
    auto odom_table = table->GetSubTable("rio_odometry");
    rio_odom_pose_sub_ = std::make_unique<nt::DoubleArraySubscriber>(
        odom_table->GetDoubleArrayTopic("pose").Subscribe({}));
    rio_odom_angular_vel_sub_ = std::make_unique<nt::DoubleSubscriber>(
        odom_table->GetDoubleTopic("angular_velocity").Subscribe(0.0));

    std::cout << "[NT] Publishers initialized (including RIO odometry subscriber)" << std::endl;
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

    // Check both IsConnected() and GetConnections()
    // IsConnected() returns true if NT client is running and has connections
    bool nt_connected = nt_instance_->IsConnected();
    auto connections = nt_instance_->GetConnections();

    // Log connection status for debugging
    static bool last_status = false;
    static auto last_log_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();

    // Log every 5 seconds or on status change
    bool should_log = (nt_connected != last_status) ||
                     (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 5);

    if (should_log) {
        std::cout << "[NT] Connection status: " << (nt_connected ? "CONNECTED" : "DISCONNECTED")
                  << ", active connections: " << connections.size() << std::endl;

        if (!connections.empty()) {
            for (const auto& conn : connections) {
                std::cout << "[NT]   - Connected to: " << conn.remote_ip
                         << ":" << conn.remote_port
                         << " (protocol " << conn.protocol_version << ")" << std::endl;
            }
        }

        last_status = nt_connected;
        last_log_time = now;
    }

    return nt_connected && !connections.empty();
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
                std::vector<double> pose_cam_data = {
                    cam_pose.position.x, cam_pose.position.y, cam_pose.position.z,
                    cam_pose.orientation.w, cam_pose.orientation.x,
                    cam_pose.orientation.y, cam_pose.orientation.z
                };
                pub->pose_cam.Set(pose_cam_data);

                std::vector<double> pose_robot_data = {
                    det.robot_pose_field.x,
                    det.robot_pose_field.y,
                    det.robot_pose_field.theta
                };
                pub->pose_robot.Set(pose_robot_data);
            } else {
                std::vector<double> empty_vec;
                pub->pose_cam.Set(empty_vec);
                pub->pose_robot.Set(empty_vec);
            }

            // Quality metrics
            pub->tag_count.Set(static_cast<int>(det.detections.size()));
            pub->avg_margin.Set(det.avg_margin());
            pub->avg_reproj_error.Set(det.avg_reproj_error());
            pub->pose_valid.Set(det.multi_tag_pose_valid);

            // Use pre-computed quality from the pipeline (includes proper distance calc)
            const auto& quality = det.quality;
            std::vector<double> std_devs_data = {quality.std_dev_x, quality.std_dev_y, quality.std_dev_theta};
            pub->std_devs.Set(std_devs_data);
            pub->confidence.Set(quality.confidence);
        }

        // Publish fused pose (filtered)
        auto now = SystemClock::now();
        fused_pubs_->timestamp.Set(
            std::chrono::duration<double>(now.time_since_epoch()).count());

        // Compute total latency from capture to now (for odometry sync)
        double latency_ms = std::chrono::duration<double, std::milli>(
            SteadyClock::now() - fused.timestamps.capture).count();
        fused_pubs_->latency_ms.Set(latency_ms);

        std::vector<double> fused_pose_data = {
            fused.pose_filtered.x,
            fused.pose_filtered.y,
            fused.pose_filtered.theta
        };
        fused_pubs_->pose.Set(fused_pose_data);
        std::vector<double> fused_std_devs_data = {
            fused.quality.std_dev_x,
            fused.quality.std_dev_y,
            fused.quality.std_dev_theta
        };
        fused_pubs_->std_devs.Set(fused_std_devs_data);
        fused_pubs_->tag_count.Set(fused.total_tags);
        fused_pubs_->cameras_contributing.Set(fused.cameras_contributing);
        fused_pubs_->confidence.Set(fused.quality.confidence);
        fused_pubs_->valid.Set(fused.valid);
        fused_pubs_->heartbeat.Set(heartbeat_counter_++);

        // Publish fused raw pose
        fused_raw_pubs_->timestamp.Set(
            std::chrono::duration<double>(now.time_since_epoch()).count());
        fused_raw_pubs_->latency_ms.Set(latency_ms);
        std::vector<double> fused_raw_pose_data = {
            fused.pose_raw.x,
            fused.pose_raw.y,
            fused.pose_raw.theta
        };
        fused_raw_pubs_->pose.Set(fused_raw_pose_data);
        std::vector<double> fused_raw_std_devs_data = {
            fused.quality.std_dev_x,
            fused.quality.std_dev_y,
            fused.quality.std_dev_theta
        };
        fused_raw_pubs_->std_devs.Set(fused_raw_std_devs_data);
        fused_raw_pubs_->tag_count.Set(fused.total_tags);
        fused_raw_pubs_->cameras_contributing.Set(fused.cameras_contributing);
        fused_raw_pubs_->confidence.Set(fused.quality.confidence);
        fused_raw_pubs_->valid.Set(fused.valid);
        fused_raw_pubs_->heartbeat.Set(heartbeat_counter_);

        // Publish status
        status_pubs_->uptime.Set(status.uptime_seconds);
        status_pubs_->cpu_temp.Set(status.cpu_temp);
        status_pubs_->cpu_usage.Set(status.cpu_usage);

        for (size_t i = 0; i < status.cameras.size() && i < status_pubs_->cam_fps.size(); i++) {
            status_pubs_->cam_fps[i].Set(status.cameras[i].fps);
            status_pubs_->cam_dropped[i].Set(static_cast<int64_t>(status.cameras[i].frames_dropped));
            status_pubs_->cam_connected[i].Set(status.cameras[i].connected);
        }

        // NEW: Publish NetworkTables connection status
        bool nt_connected = is_connected();
        auto connections = nt_instance_->GetConnections();
        status_pubs_->nt_connected.Set(nt_connected);
        status_pubs_->nt_connection_count.Set(static_cast<int64_t>(connections.size()));

        if (!connections.empty()) {
            // Publish first connection's IP
            status_pubs_->nt_server_ip.Set(connections[0].remote_ip);
        } else {
            status_pubs_->nt_server_ip.Set("disconnected");
        }

        // Check for alignment target changes from robot
        check_align_subscriptions();

        // Publish auto-alignment data
        AlignResult align_result;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            align_result = latest_align_result_;
        }

        if (align_pubs_) {
            align_pubs_->target_visible.Set(align_result.target_visible);
            std::vector<double> target_pose_data = {
                align_result.target_pose.x,
                align_result.target_pose.y,
                align_result.target_pose.theta
            };
            align_pubs_->target_pose.Set(target_pose_data);
            std::vector<double> robot_pose_data = {
                align_result.robot_pose.x,
                align_result.robot_pose.y,
                align_result.robot_pose.theta
            };
            align_pubs_->robot_pose.Set(robot_pose_data);
            std::vector<double> error_data = {
                align_result.error.x,
                align_result.error.y,
                align_result.error.theta
            };
            align_pubs_->error.Set(error_data);
            align_pubs_->distance_m.Set(align_result.distance_m);
            align_pubs_->angle_error_rad.Set(align_result.error.theta);
            align_pubs_->ready.Set(align_result.ready);
            align_pubs_->has_target.Set(align_result.has_target);

            AlignTarget target = get_align_target();
            align_pubs_->current_target_id.Set(target.target_tag_id);
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

NTPublisher::RioOdometry NTPublisher::get_rio_odometry() const {
    RioOdometry odom;

    if (!rio_odom_pose_sub_ || !rio_odom_angular_vel_sub_) {
        return odom;
    }

    auto pose_data = rio_odom_pose_sub_->Get();
    if (pose_data.size() >= 3) {
        odom.x = pose_data[0];
        odom.y = pose_data[1];
        odom.theta = pose_data[2];
        odom.angular_velocity = rio_odom_angular_vel_sub_->Get();
        odom.valid = true;
    }

    return odom;
}

void NTPublisher::flush() {
    if (nt_instance_) {
        nt_instance_->Flush();
    }
}

void NTPublisher::set_align_target_callback(std::function<void(int tag_id)> callback) {
    std::lock_guard<std::mutex> lock(align_mutex_);
    align_target_callback_ = callback;
}

void NTPublisher::publish_align_result(const AlignResult& result) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_align_result_ = result;
}

AlignTarget NTPublisher::get_align_target() const {
    std::lock_guard<std::mutex> lock(align_mutex_);
    return align_target_;
}

void NTPublisher::check_align_subscriptions() {
    if (!align_pubs_) return;

    // Read target tag ID from robot
    int64_t new_target_id = align_pubs_->target_tag_id_sub.Get();

    // Read target offset [distance_m, angle_rad]
    auto offset = align_pubs_->target_offset_sub.Get();
    double distance = 0.5;
    double angle = 0.0;
    if (offset.size() >= 2) {
        distance = offset[0];
        angle = offset[1];
    }

    // Check if target changed
    {
        std::lock_guard<std::mutex> lock(align_mutex_);
        if (new_target_id != align_target_.target_tag_id) {
            align_target_.target_tag_id = static_cast<int>(new_target_id);
            align_target_.has_target = (new_target_id >= 0);
            align_target_.approach_distance = distance;
            align_target_.approach_angle = angle;

            std::cout << "[NT] Alignment target changed to tag " << new_target_id << std::endl;

            // Call callback if set
            if (align_target_callback_) {
                align_target_callback_(static_cast<int>(new_target_id));
            }
        }
        // Update offset even if target didn't change
        align_target_.approach_distance = distance;
        align_target_.approach_angle = angle;
    }
}

int64_t NTPublisher::to_nt_timestamp(SystemTimePoint time) const {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        time.time_since_epoch()).count();
}

} // namespace frc_vision
