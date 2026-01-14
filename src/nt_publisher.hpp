#pragma once
/**
 * @file nt_publisher.hpp
 * @brief NetworkTables 4 publisher for roboRIO integration
 *
 * Publishes all vision data to NetworkTables for fusion with robot odometry.
 * Follows WPILib conventions for timestamps and pose estimation.
 */

#include "types.hpp"
#include <memory>
#include <atomic>
#include <thread>

// Forward declarations for NT types
namespace nt {
    class NetworkTableInstance;
    class NetworkTable;
    class DoublePublisher;
    class DoubleArrayPublisher;
    class IntegerPublisher;
    class IntegerArrayPublisher;
    class BooleanPublisher;
    class StringPublisher;
}

namespace frc_vision {

/**
 * @brief NetworkTables 4 publisher
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
     * @brief Force flush all published values
     */
    void flush();

private:
    struct CameraPublishers;
    struct FusedPublishers;
    struct StatusPublishers;

    void publish_loop();
    void setup_publishers();
    int64_t to_nt_timestamp(SystemTimePoint time) const;

    std::unique_ptr<nt::NetworkTableInstance> nt_instance_;
    std::string table_root_;
    int num_cameras_ = 0;

    std::vector<std::unique_ptr<CameraPublishers>> camera_pubs_;
    std::unique_ptr<FusedPublishers> fused_pubs_;
    std::unique_ptr<FusedPublishers> fused_raw_pubs_;
    std::unique_ptr<StatusPublishers> status_pubs_;

    // Latest data for background publishing
    std::vector<FrameDetections> latest_detections_;
    FusedPose latest_fused_;
    SystemStatus latest_status_;
    std::mutex data_mutex_;

    std::thread publish_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> should_stop_{false};
    int publish_rate_hz_ = 50;
};

} // namespace frc_vision
