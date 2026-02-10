#pragma once
/**
 * @file nt_publisher.hpp
 * @brief NetworkTables 4 publisher for roboRIO integration
 *
 * Publishes all vision data to NetworkTables for fusion with robot odometry
 * and auto-alignment. Follows WPILib conventions for timestamps and pose estimation.
 *
 * ## NetworkTables Schema for Auto-Align
 *
 * /FRCVision/
 *   fused/
 *     pose           - [x, y, theta] robot pose in field frame (meters, radians)
 *     std_devs       - [x, y, theta] standard deviations for pose estimator
 *     timestamp      - FPGA timestamp (seconds)
 *     valid          - boolean, true if pose is trustworthy
 *     confidence     - 0.0-1.0 confidence score
 *
 *   auto_align/
 *     target_tag_id  - (subscribe) robot sets target tag ID for alignment
 *     target_visible - boolean, true if target tag is currently visible
 *     target_pose    - [x, y, theta] where robot should be for alignment
 *     robot_pose     - [x, y, theta] current robot pose from vision
 *     error          - [x, y, theta] error between current and target
 *     distance_m     - distance to target position (meters)
 *     ready          - boolean, true if aligned (error within tolerance)
 *     has_target     - boolean, true if target pose is set
 */

#include "types.hpp"
#include <memory>
#include <atomic>
#include <thread>
#include <functional>

// Forward declarations for NT types
namespace nt {
    class NetworkTableInstance;
    class NetworkTable;
    class DoublePublisher;
    class DoubleSubscriber;
    class DoubleArrayPublisher;
    class IntegerPublisher;
    class IntegerArrayPublisher;
    class BooleanPublisher;
    class StringPublisher;
    class IntegerSubscriber;
    class DoubleSubscriber;
    class DoubleArraySubscriber;
}

namespace frc_vision {

/**
 * @brief Auto-alignment target data
 */
struct AlignTarget {
    int target_tag_id = -1;          // Tag ID to align to (-1 = none)
    Pose2D target_pose;              // Where robot should be for alignment
    double approach_distance = 0.5;   // Distance from tag (meters)
    double approach_angle = 0.0;      // Angle offset from perpendicular (radians)
    bool has_target = false;
};

/**
 * @brief Auto-alignment result
 */
struct AlignResult {
    bool target_visible = false;     // Is target tag currently visible?
    Pose2D robot_pose;               // Current robot pose from vision
    Pose2D target_pose;              // Target pose for alignment
    Pose2D error;                    // Error (target - current)
    double distance_m = 0.0;         // Distance to target
    bool ready = false;              // True if within alignment tolerance
    bool has_target = false;         // True if target is set
};

/**
 * @brief NetworkTables 4 publisher with auto-align support
 */
class NTPublisher {
public:
    NTPublisher();
    ~NTPublisher();

    // Non-copyable
    NTPublisher(const NTPublisher&) = delete;
    NTPublisher& operator=(const NTPublisher&) = delete;

    /**
     * @brief Initialize NT connection
     * @param server Server address (e.g., "10.TE.AM.2" or "roborio-TEAM-frc.local")
     * @param table_root Root table path (e.g., "/FRCVision")
     * @param num_cameras Number of cameras to publish
     * @return true on success
     */
    bool initialize(const std::string& server, const std::string& table_root, int num_cameras);

    /**
     * @brief Start background publishing thread
     * @param rate_hz Publishing rate in Hz
     */
    void start(int rate_hz = 50);

    /**
     * @brief Stop publishing thread
     */
    void stop();

    /**
     * @brief Check if connected to NT server
     */
    bool is_connected() const;

    /**
     * @brief Publish camera detection results
     */
    void publish_camera(int camera_id, const FrameDetections& detections);

    /**
     * @brief Publish fused pose
     */
    void publish_fused(const FusedPose& fused);

    /**
     * @brief Publish system status
     */
    void publish_status(const SystemStatus& status);

    /**
     * @brief Set alignment target callback (called when robot requests alignment)
     */
    void set_align_target_callback(std::function<void(int tag_id)> callback);

    /**
     * @brief Publish auto-alignment result
     */
    void publish_align_result(const AlignResult& result);

    /**
     * @brief Get current alignment target from robot
     */
    AlignTarget get_align_target() const;

    /**
     * @brief Get latest odometry data from RoboRIO (subscribed via NT).
     * The coprocessor uses this for innovation gating: rejecting vision
     * measurements that disagree too much with where the robot thinks it is.
     * @return Latest odometry pose, angular velocity, and validity
     */
    struct RioOdometry {
        double x = 0, y = 0, theta = 0;
        double angular_velocity = 0;
        bool valid = false;
    };
    RioOdometry get_rio_odometry() const;

    /**
     * @brief Force flush all published values
     */
    void flush();

private:
    struct CameraPublishers;
    struct FusedPublishers;
    struct StatusPublishers;
    struct AlignPublishers;

    void publish_loop();
    void setup_publishers();
    void check_align_subscriptions();
    int64_t to_nt_timestamp(SystemTimePoint time) const;

    std::unique_ptr<nt::NetworkTableInstance> nt_instance_;
    std::string table_root_;
    int num_cameras_ = 0;

    std::vector<std::unique_ptr<CameraPublishers>> camera_pubs_;
    std::unique_ptr<FusedPublishers> fused_pubs_;
    std::unique_ptr<FusedPublishers> fused_raw_pubs_;
    std::unique_ptr<StatusPublishers> status_pubs_;
    std::unique_ptr<AlignPublishers> align_pubs_;

    // Latest data for background publishing
    std::vector<FrameDetections> latest_detections_;
    FusedPose latest_fused_;
    SystemStatus latest_status_;
    AlignResult latest_align_result_;
    mutable std::mutex data_mutex_;

    // Alignment target from robot
    mutable std::mutex align_mutex_;
    AlignTarget align_target_;
    std::function<void(int)> align_target_callback_;

    std::thread publish_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> should_stop_{false};
    int publish_rate_hz_ = 50;
    int64_t heartbeat_counter_ = 0;

    // RoboRIO odometry subscribers (for innovation gating on the coprocessor)
    std::unique_ptr<nt::DoubleArraySubscriber> rio_odom_pose_sub_;
    std::unique_ptr<nt::DoubleSubscriber> rio_odom_angular_vel_sub_;
};

} // namespace frc_vision
