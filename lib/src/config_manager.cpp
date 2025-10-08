#include "stream_manager/config_manager.hpp"
#include <fstream>
#include <iostream>
#include <sstream>

namespace stream_manager {

bool ConfigManager::loadFromFile(const std::string &config_path) {
  try {
    YAML::Node yaml_config = YAML::LoadFile(config_path);
    return loadFromYAML(yaml_config);
  } catch (const YAML::Exception &e) {
    validation_errors_ = "YAML parsing error: " + std::string(e.what());
    return false;
  } catch (const std::exception &e) {
    validation_errors_ = "File loading error: " + std::string(e.what());
    return false;
  }
}

bool ConfigManager::loadFromYAML(const YAML::Node &yaml_node) {
  try {
    setDefaults();

    // Load video source configuration
    if (yaml_node["video_source"]) {
      const auto &vs_node = yaml_node["video_source"];

      if (vs_node["type"]) {
        config_.video_source.type =
            stringToSourceType(vs_node["type"].as<std::string>());
      }

      if (vs_node["topic_name"]) {
        config_.video_source.topic_name =
            vs_node["topic_name"].as<std::string>();
      }

      if (vs_node["device_id"]) {
        config_.video_source.device_id = vs_node["device_id"].as<int>();
      }

      if (vs_node["mavlink_address"]) {
        config_.video_source.mavlink_address =
            vs_node["mavlink_address"].as<std::string>();
      }

      if (vs_node["timeout_seconds"]) {
        config_.video_source.timeout_seconds =
            vs_node["timeout_seconds"].as<double>();
      }
    }

    // Load buffer configuration
    if (yaml_node["buffer"]) {
      const auto &buffer_node = yaml_node["buffer"];

      if (buffer_node["max_size"]) {
        config_.buffer.max_size = buffer_node["max_size"].as<size_t>();
      }

      if (buffer_node["target_fps"]) {
        config_.buffer.target_fps = buffer_node["target_fps"].as<double>();
      }
    }

    // Load scaling configuration
    if (yaml_node["scaling"]) {
      const auto &scaling_node = yaml_node["scaling"];

      if (scaling_node["enabled"]) {
        config_.scaling.enabled = scaling_node["enabled"].as<bool>();
      }

      if (scaling_node["target_width"]) {
        config_.scaling.target_width = scaling_node["target_width"].as<int>();
      }

      if (scaling_node["target_height"]) {
        config_.scaling.target_height = scaling_node["target_height"].as<int>();
      }

      if (scaling_node["interpolation_method"]) {
        config_.scaling.interpolation_method =
            scaling_node["interpolation_method"].as<int>();
      }
    }

    // Load publishing configuration
    if (yaml_node["publishing"]) {
      const auto &pub_node = yaml_node["publishing"];

      if (pub_node["current_frame_topic"]) {
        config_.publishing.current_frame_topic =
            pub_node["current_frame_topic"].as<std::string>();
      }

      if (pub_node["delayed_frame_topic"]) {
        config_.publishing.delayed_frame_topic =
            pub_node["delayed_frame_topic"].as<std::string>();
      }

      if (pub_node["delay_ms"]) {
        config_.publishing.delay_ms = pub_node["delay_ms"].as<uint32_t>();
      }

      if (pub_node["enable_current_publishing"]) {
        config_.publishing.enable_current_publishing =
            pub_node["enable_current_publishing"].as<bool>();
      }

      if (pub_node["enable_delayed_publishing"]) {
        config_.publishing.enable_delayed_publishing =
            pub_node["enable_delayed_publishing"].as<bool>();
      }
    }

    // Load debug flag
    if (yaml_node["enable_debug_logging"]) {
      config_.enable_debug_logging =
          yaml_node["enable_debug_logging"].as<bool>();
    }

    return validateConfig();

  } catch (const YAML::Exception &e) {
    validation_errors_ = "YAML parsing error: " + std::string(e.what());
    return false;
  } catch (const std::exception &e) {
    validation_errors_ =
        "Configuration loading error: " + std::string(e.what());
    return false;
  }
}

bool ConfigManager::saveToFile(const std::string &config_path) const {
  try {
    YAML::Node yaml_config;

    // Video source configuration
    yaml_config["video_source"]["type"] =
        sourceTypeToString(config_.video_source.type);
    yaml_config["video_source"]["topic_name"] = config_.video_source.topic_name;
    yaml_config["video_source"]["device_id"] = config_.video_source.device_id;
    yaml_config["video_source"]["mavlink_address"] =
        config_.video_source.mavlink_address;
    yaml_config["video_source"]["timeout_seconds"] =
        config_.video_source.timeout_seconds;

    // Buffer configuration
    yaml_config["buffer"]["max_size"] = config_.buffer.max_size;
    yaml_config["buffer"]["target_fps"] = config_.buffer.target_fps;

    // Scaling configuration
    yaml_config["scaling"]["enabled"] = config_.scaling.enabled;
    yaml_config["scaling"]["target_width"] = config_.scaling.target_width;
    yaml_config["scaling"]["target_height"] = config_.scaling.target_height;
    yaml_config["scaling"]["interpolation_method"] =
        config_.scaling.interpolation_method;

    // Publishing configuration
    yaml_config["publishing"]["current_frame_topic"] =
        config_.publishing.current_frame_topic;
    yaml_config["publishing"]["delayed_frame_topic"] =
        config_.publishing.delayed_frame_topic;
    yaml_config["publishing"]["delay_ms"] = config_.publishing.delay_ms;
    yaml_config["publishing"]["enable_current_publishing"] =
        config_.publishing.enable_current_publishing;
    yaml_config["publishing"]["enable_delayed_publishing"] =
        config_.publishing.enable_delayed_publishing;

    // Debug flag
    yaml_config["enable_debug_logging"] = config_.enable_debug_logging;

    std::ofstream file(config_path);
    file << yaml_config;

    return file.good();

  } catch (const std::exception &e) {
    return false;
  }
}

bool ConfigManager::validateConfig() const {
  validation_errors_.clear();
  std::stringstream errors;

  // Validate buffer configuration
  if (config_.buffer.max_size == 0) {
    errors << "Buffer max_size must be greater than 0. ";
  }

  if (config_.buffer.target_fps <= 0.0) {
    errors << "Buffer target_fps must be greater than 0. ";
  }

  // Validate scaling configuration
  if (config_.scaling.enabled) {
    if (config_.scaling.target_width <= 0 ||
        config_.scaling.target_height <= 0) {
      errors << "Scaling target dimensions must be positive. ";
    }

    if (config_.scaling.interpolation_method < 0 ||
        config_.scaling.interpolation_method > 5) {
      errors << "Invalid interpolation method (should be 0-5). ";
    }
  }

  // Validate video source configuration
  if (config_.video_source.timeout_seconds <= 0.0) {
    errors << "Video source timeout must be positive. ";
  }

  switch (config_.video_source.type) {
  case VideoSourceConfig::SourceType::USB_CAMERA:
    if (config_.video_source.device_id < 0) {
      errors << "USB camera device_id must be non-negative. ";
    }
    break;

  case VideoSourceConfig::SourceType::GAZEBO_ROS2:
    if (config_.video_source.topic_name.empty()) {
      errors << "ROS2 topic name cannot be empty. ";
    }
    break;

  case VideoSourceConfig::SourceType::MAVLINK:
    if (config_.video_source.mavlink_address.empty()) {
      errors << "MAVLink address cannot be empty. ";
    }
    break;
  }

  // No iceoryx validation

  validation_errors_ = errors.str();
  return validation_errors_.empty();
}

std::string ConfigManager::getValidationErrors() const {
  return validation_errors_;
}

void ConfigManager::setDefaults() {
  config_ = StreamManagerConfig{}; // Use default constructor values
}

VideoSourceConfig::SourceType
ConfigManager::stringToSourceType(const std::string &type_str) const {
  if (type_str == "gazebo_ros2" || type_str == "GAZEBO_ROS2") {
    return VideoSourceConfig::SourceType::GAZEBO_ROS2;
  } else if (type_str == "usb_camera" || type_str == "USB_CAMERA") {
    return VideoSourceConfig::SourceType::USB_CAMERA;
  } else if (type_str == "mavlink" || type_str == "MAVLINK") {
    return VideoSourceConfig::SourceType::MAVLINK;
  }

  return VideoSourceConfig::SourceType::USB_CAMERA; // Default
}

std::string
ConfigManager::sourceTypeToString(VideoSourceConfig::SourceType type) const {
  switch (type) {
  case VideoSourceConfig::SourceType::GAZEBO_ROS2:
    return "gazebo_ros2";
  case VideoSourceConfig::SourceType::USB_CAMERA:
    return "usb_camera";
  case VideoSourceConfig::SourceType::MAVLINK:
    return "mavlink";
  default:
    return "usb_camera";
  }
}

} // namespace stream_manager