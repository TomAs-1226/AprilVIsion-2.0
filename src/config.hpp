#pragma once
/**
 * @file config.hpp
 * @brief Configuration loading and management
 */

#include "types.hpp"
#include <string>
#include <memory>
#include <atomic>

namespace frc_vision {

/**
 * @brief Configuration manager with hot-reload support
 */
class ConfigManager {
public:
    ConfigManager();
    ~ConfigManager();

    /**
     * @brief Load configuration from YAML file
     * @param path Path to config.yml
     * @return true on success
     */
    bool load(const std::string& path);

    /**
     * @brief Reload configuration (hot reload)
     * @return true on success
     */
    bool reload();

    /**
     * @brief Get current configuration
     */
    const Config& get() const { return config_; }
    Config& get_mutable() { return config_; }

    /**
     * @brief Load camera intrinsics from YAML file
     * @param path Path to intrinsics YAML (OpenCV format)
     * @return CameraIntrinsics on success
     */
    static std::optional<CameraIntrinsics> load_intrinsics(const std::string& path);

    /**
     * @brief Get config file path
     */
    const std::string& config_path() const { return config_path_; }

    /**
     * @brief Check if reload was requested
     */
    bool reload_requested() const { return config_.reload_requested.load(); }

    /**
     * @brief Clear reload flag
     */
    void clear_reload_flag() { config_.reload_requested.store(false); }

    /**
     * @brief Request a reload
     */
    void request_reload() { config_.reload_requested.store(true); }

private:
    std::string config_path_;
    Config config_;

    bool parse_cameras(void* node);
    bool parse_detector(void* node);
    bool parse_tracker(void* node);
    bool parse_calibration(void* node);  // Phase 1
    bool parse_field(void* node);
    bool parse_output(void* node);
    bool parse_performance(void* node);
    Pose3D parse_transform(void* node);
};

} // namespace frc_vision
