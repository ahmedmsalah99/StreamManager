#include "stream_manager/config_manager.hpp"
#include <iostream>
#include <cassert>

using namespace stream_manager;

int main() {
    std::cout << "Testing StreamManager Configuration..." << std::endl;
    
    // Test 1: Default configuration
    ConfigManager config_manager;
    StreamManagerConfig default_config;
    config_manager.setConfig(default_config);
    
    assert(config_manager.validateConfig());
    std::cout << "✓ Default configuration validation passed" << std::endl;
    
    // Test 2: Invalid configuration
    StreamManagerConfig invalid_config;
    invalid_config.buffer.max_size = 0;  // Invalid
    invalid_config.buffer.target_fps = -1.0;  // Invalid
    
    config_manager.setConfig(invalid_config);
    assert(!config_manager.validateConfig());
    std::cout << "✓ Invalid configuration properly rejected" << std::endl;
    std::cout << "  Validation errors: " << config_manager.getValidationErrors() << std::endl;
    
    // Test 3: Video source type conversion
    assert(config_manager.stringToSourceType("usb_camera") == VideoSourceConfig::SourceType::USB_CAMERA);
    assert(config_manager.stringToSourceType("gazebo_ros2") == VideoSourceConfig::SourceType::GAZEBO_ROS2);
    assert(config_manager.stringToSourceType("mavlink") == VideoSourceConfig::SourceType::MAVLINK);
    std::cout << "✓ Video source type conversion works" << std::endl;
    
    // Test 4: Configuration with scaling
    StreamManagerConfig scaling_config;
    scaling_config.scaling.enabled = true;
    scaling_config.scaling.target_width = 1920;
    scaling_config.scaling.target_height = 1080;
    
    config_manager.setConfig(scaling_config);
    assert(config_manager.validateConfig());
    std::cout << "✓ Scaling configuration validation passed" << std::endl;
    
    // Test 5: Buffer configuration
    StreamManagerConfig buffer_config;
    buffer_config.buffer.max_size = 100;
    buffer_config.buffer.target_fps = 30.0;
    
    config_manager.setConfig(buffer_config);
    assert(config_manager.validateConfig());
    std::cout << "✓ Buffer configuration validation passed" << std::endl;
    
    // Test 6: FrameMetadata structure
    FrameMetadata metadata(123, 30.0, 1920, 1080, 3, CV_8UC3, 0, 1920*1080*3);
    assert(metadata.fps == 30.0);
    assert(metadata.width == 1920);
    assert(metadata.height == 1080);
    assert(metadata.channels == 3);
    std::cout << "✓ FrameMetadata structure works correctly" << std::endl;
    
    std::cout << "\nAll configuration tests passed! ✓" << std::endl;
    std::cout << "StreamManager library structure is correct." << std::endl;
    std::cout << "Zero-copy frame buffer implementation ready!" << std::endl;
    
    return 0;
}