# StreamManager ROS2 Node

A comprehensive ROS2 video stream management node designed for robotics applications, featuring multiple video source support, intelligent buffering, and configurable frame publishing with delay capabilities.

## Features

### 1. **Abstracted Video Sources**
- **Gazebo ROS2**: Subscribe to camera topics from Gazebo simulation
- **USB Camera**: Direct access to USB/webcam devices
- **MAVLink**: Video streaming from MAVLink-compatible drones/cameras

### 2. **Core Abstract Interface**
- `getFrame()`: Get the most recent frame from the video source
- `getFPS()`: Get current frames per second
- Unified interface regardless of video source type

### 3. **Intelligent Frame Buffer**
- Configurable buffer size for storing recent frames
- Custom target FPS (can be lower than input video FPS)
- Timestamp-based frame indexing
- `getClosestFrame(timestamp)`: Find frame closest to specific time
- `getFrameByIdx(index)`: Access frames by buffer index

### 4. **ROS2 Publishing**
- Publishes current frames in real-time
- Publishes delayed frames with configurable delay
- Uses standard sensor_msgs/Image messages
- Configurable topic names and publishing rates

### 5. **Automatic Frame Scaling**
- Optional frame scaling before buffering
- Configurable target dimensions
- Multiple interpolation methods supported
- Automatic bypass when scaling not needed

### 6. **Comprehensive Configuration Management**
- YAML-based configuration files
- Runtime configuration updates
- Validation and error reporting
- Default fallback values

## Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Video Source  │───▶│  StreamManager   │───▶│  Frame Buffer   │
│   (Abstract)    │    │   (ROS2 Node)    │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
        │                        │                        │
        ▼                        ▼                        ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│ • Gazebo ROS2   │    │ • Configuration  │    │ • Timestamped   │
│ • USB Camera    │    │ • Frame Scaling  │    │   Frames        │
│ • MAVLink       │    │ • ROS2 Publishers│    │ • CPU Buffers   │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                │
                                ▼
                    ┌──────────────────┐
                    │  ROS2 Topics     │
                    │ • Current Frame  │
                    │ • Delayed Frame  │
                    └──────────────────┘
```

## Installation

### Prerequisites

```bash
# ROS2 (required)
sudo apt install ros-humble-desktop

# OpenCV
sudo apt install libopencv-dev

# yaml-cpp
sudo apt install libyaml-cpp-dev

# ros2_shm_msgs (for shared memory messages)
sudo apt install ros-humble-shm-msgs
```

### Build

```bash
# Clone the repository
cd /path/to/your/workspace
git clone <repository-url>

# Build with colcon (ROS2)
colcon build --packages-select stream_manager

# Or build with CMake directly
cd StreamManager
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## Usage

### Basic Usage

```cpp
#include "stream_manager/stream_manager.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Create StreamManager as a ROS2 node
    auto manager = std::make_shared<stream_manager::StreamManager>(
        "config/default_config.yaml", "stream_manager_node");
    
    // Initialize
    if (!manager->initialize()) {
        std::cerr << "Failed to initialize" << std::endl;
        return -1;
    }
    
    // Run the node
    rclcpp::spin(manager);
    
    // Shutdown
    rclcpp::shutdown();
    return 0;
}
```

### Running the Node

```bash
# Source the workspace
source install/setup.bash

# Run the example
ros2 run stream_manager basic_usage

# Or run with custom config
ros2 run stream_manager basic_usage --ros-args --params-file config/default_config.yaml
```

### Published Topics

The StreamManager node publishes the following topics:

- `/stream_manager/current_frame` (sensor_msgs/Image): Real-time frames
- `/stream_manager/delayed_frame` (sensor_msgs/Image): Frames delayed by configured amount

### Subscribed Topics

- `/camera/image_raw` (sensor_msgs/Image): When using Gazebo video source

### Configuration

Create a YAML configuration file:

```yaml
video_source:
  type: "usb_camera"  # "gazebo_ros2", "usb_camera", "mavlink"
  device_id: 0
  timeout_seconds: 5.0

buffer:
  max_size: 30
  target_fps: 10.0

scaling:
  enabled: true
  target_width: 640
  target_height: 480
  interpolation_method: 1

publishing:
  current_frame_topic: "/stream_manager/current_frame"
  delayed_frame_topic: "/stream_manager/delayed_frame"
  delay_seconds: 1.0
  enable_current_publishing: true
  enable_delayed_publishing: true
```

### Video Source Types

#### USB Camera
```yaml
video_source:
  type: "usb_camera"
  device_id: 0  # Camera device ID
```

#### Gazebo ROS2
```yaml
video_source:
  type: "gazebo_ros2"
  topic_name: "/camera/image_raw"
```

#### MAVLink
```yaml
video_source:
  type: "mavlink"
  mavlink_address: "udp://127.0.0.1:14550"
```

## API Reference

### StreamManager Class

#### Core Methods
- `bool initialize()`: Initialize the stream manager
- `void shutdown()`: Shutdown and cleanup resources
- `std::shared_ptr<cv::Mat> getFrame()`: Get latest frame
- `double getFPS() const`: Get current FPS

#### Buffer Access
- `std::shared_ptr<FrameData> getClosestFrame(timestamp)`: Get frame closest to timestamp
- `std::shared_ptr<FrameData> getFrameByIdx(index)`: Get frame by buffer index
- `std::shared_ptr<FrameData> getLatestBufferedFrame()`: Get latest buffered frame

#### Shared Memory
- `std::shared_ptr<cv::Mat> getFrameFromSharedMemory()`: Get frame from shared memory

#### Configuration
- `bool loadConfig(path)`: Load configuration from file
- `bool updateConfig(config)`: Update configuration at runtime
- `const StreamManagerConfig& getConfig()`: Get current configuration

#### Status and Statistics
- `bool isVideoSourceConnected()`: Check video source connection
- `std::string getVideoSourceInfo()`: Get detailed source information
- `size_t getBufferSize()`: Get current buffer size
- `double getAverageSourceFPS()`: Get average source FPS

### FrameData Structure

```cpp
struct FrameData {
    std::chrono::steady_clock::time_point timestamp;
    cv::Mat frame;
    size_t frame_id;
    double fps;
};
```

## Examples

See the `examples/` directory for complete usage examples:

- `basic_usage.cpp`: Basic video capture and display
- `multi_source_example.cpp`: Switching between video sources
- `shared_memory_example.cpp`: Inter-process communication demo

## Performance Considerations

### Memory Usage
- Frame buffer size directly affects memory usage
- Shared memory chunks are allocated based on max frame size
- Consider frame scaling to reduce memory footprint

### CPU Usage
- Buffer target FPS controls processing load
- Frame scaling adds computational overhead
- USB camera capture runs in separate thread

### Latency
- Direct `getFrame()` provides lowest latency
- Buffer access adds minimal latency
- Shared memory access is near zero-copy

## Troubleshooting

### Common Issues

1. **Camera not detected**
   - Check device permissions: `sudo chmod 666 /dev/video*`
   - Verify device ID with `v4l2-ctl --list-devices`

2. **ROS2 topic not found**
   - Ensure ROS2 is sourced: `source /opt/ros/humble/setup.bash`
   - Check topic exists: `ros2 topic list`

3. **iceoryx initialization fails**
   - Start iceoryx daemon: `iox-roudi`
   - Check shared memory permissions

4. **High CPU usage**
   - Reduce buffer target FPS
   - Disable frame scaling if not needed
   - Check video source resolution

### Debug Mode

Enable debug logging in configuration:

```yaml
enable_debug_logging: true
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure all tests pass
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Dependencies

- **OpenCV**: Computer vision and image processing
- **ROS2**: Robot Operating System (optional, for Gazebo support)
- **iceoryx**: Zero-copy inter-process communication
- **yaml-cpp**: YAML configuration file parsing

## Roadmap

- [ ] H.264/H.265 hardware encoding/decoding support
- [ ] Network streaming (RTSP/WebRTC)
- [ ] Multi-camera synchronization
- [ ] Advanced frame interpolation
- [ ] GPU acceleration support
- [ ] Python bindings