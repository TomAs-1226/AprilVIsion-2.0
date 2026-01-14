#pragma once
/**
 * @file frame_source.hpp
 * @brief Abstract frame source interface for platform abstraction
 *
 * Allows the vision pipeline to consume frames from:
 * - Real cameras (Orange Pi V4L2)
 * - Mac webcam
 * - Simulator with synthetic rendering
 */

#include "../types.hpp"
#include "../ring_buffer.hpp"
#include <memory>
#include <functional>

namespace frc_vision {

/**
 * @brief Abstract interface for frame sources
 *
 * Implemented by:
 * - Camera (real hardware via V4L2/AVFoundation)
 * - WebcamSource (Mac webcam)
 * - SimulatorSource (synthetic rendering + optional webcam composite)
 */
class IFrameSource {
public:
    virtual ~IFrameSource() = default;

    /**
     * @brief Start the frame source
     * @return true if started successfully
     */
    virtual bool start() = 0;

    /**
     * @brief Stop the frame source
     */
    virtual void stop() = 0;

    /**
     * @brief Check if source is running
     */
    virtual bool is_running() const = 0;

    /**
     * @brief Check if source is connected/available
     */
    virtual bool is_connected() const = 0;

    /**
     * @brief Get source ID
     */
    virtual int id() const = 0;

    /**
     * @brief Get source name
     */
    virtual const std::string& name() const = 0;

    /**
     * @brief Get output frame buffer
     */
    virtual RingBuffer<Frame, 4>& frame_buffer() = 0;

    /**
     * @brief Get current FPS
     */
    virtual double fps() const = 0;

    /**
     * @brief Get total frames captured
     */
    virtual uint64_t frames_captured() const = 0;

    /**
     * @brief Get total frames dropped
     */
    virtual uint64_t frames_dropped() const = 0;

    /**
     * @brief Get camera intrinsics
     */
    virtual const CameraIntrinsics& intrinsics() const = 0;

    /**
     * @brief Set camera intrinsics
     */
    virtual void set_intrinsics(const CameraIntrinsics& intr) = 0;

    /**
     * @brief Get camera-to-robot transform
     */
    virtual const Pose3D& camera_to_robot() const = 0;
};

/**
 * @brief Factory for creating frame sources based on platform
 */
class FrameSourceFactory {
public:
    enum class SourceType {
        HARDWARE_CAMERA,    // Real camera (Orange Pi/Linux)
        MAC_WEBCAM,         // macOS webcam via AVFoundation/OpenCV
        SIMULATOR           // Synthetic frames with optional webcam composite
    };

    /**
     * @brief Create a frame source
     * @param type Source type
     * @param id Source ID
     * @param config Camera configuration
     * @return Unique pointer to frame source
     */
    static std::unique_ptr<IFrameSource> create(
        SourceType type,
        int id,
        const CameraConfig& config);
};

} // namespace frc_vision
