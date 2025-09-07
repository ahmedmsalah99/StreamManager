#pragma once

#include <string>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace stream_manager {

struct ScalingConfig {
    bool enabled = false;
    int target_width = 640;
    int target_height = 480;
    int interpolation_method = 1; // cv::INTER_LINEAR
};

struct BufferConfig {
    size_t max_size = 30;
    double target_fps = 10.0;
    bool enable_timestamp_indexing = true;
};

struct IceoryxConfig {
    std::string service_name = "StreamManager";
    std::string instance_name = "VideoStream";
    std::string event_name = "FrameEvent";
    size_t max_chunk_size = 1920 * 1080 * 3; // Max frame size in bytes
};

struct VideoSourceConfig {
    enum class SourceType {
        GAZEBO_ROS2,
        USB_CAMERA,
        MAVLINK
    };
    
    SourceType type = SourceType::USB_CAMERA;
    std::string topic_name = "/camera/image_raw"; // For ROS2
    int device_id = 0; // For USB camera
    std::string mavlink_address = "udp://127.0.0.1:14550"; // For MAVLink
    double timeout_seconds = 5.0;
};

struct StreamManagerConfig {
    VideoSourceConfig video_source;
    BufferConfig buffer;
    ScalingConfig scaling;
    IceoryxConfig iceoryx;
    bool enable_debug_logging = false;
};

class ConfigManager {
public:
    ConfigManager() = default;
    ~ConfigManager() = default;

    bool loadFromFile(const std::string& config_path);
    bool loadFromYAML(const YAML::Node& yaml_node);
    bool saveToFile(const std::string& config_path) const;
    
    const StreamManagerConfig& getConfig() const { return config_; }
    StreamManagerConfig& getConfig() { return config_; }
    
    void setConfig(const StreamManagerConfig& config) { config_ = config; }
    
    // Validation methods
    bool validateConfig() const;
    std::string getValidationErrors() const;

private:
    StreamManagerConfig config_;
    mutable std::string validation_errors_;
    
    void setDefaults();
    VideoSourceConfig::SourceType stringToSourceType(const std::string& type_str) const;
    std::string sourceTypeToString(VideoSourceConfig::SourceType type) const;
};

} // namespace stream_manager