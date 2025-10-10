# Stream Manager

The Stream Manager is a ROS2 node responsible for handling video streams from various sources, managing scaling, and publishing frames at configurable rates.

## Overview

This module provides abstracted video source interfaces (Gazebo, USB, MAVLink), and ROS2 publishing capabilities. It supports frame scaling for performance optimization and configurable target FPS for efficient processing.

## Key Features

- Multiple video source support
- Configurable frame buffer with timestamp indexing
- Automatic frame scaling (up/down) for CPU optimization
- ROS2 topic publishing for current and delayed frames
- YAML-based configuration management

## Stream Manager Notes

### 1. Scaling Configuration
Scaling can be enabled from configs. Take care that upscaling is going to put more pressure on the CPU while downscaling can make your pipeline faster. Check your detection module's required image size for detection to make a better decision.

### 2. Target FPS
`target_fps` is the FPS the frames are published according to. It's advised to keep it below the source's FPS to avoid excessive CPU usage for no reason.

## Installation

1. Ensure ROS2 Humble and dependencies (OpenCV, yaml-cpp) are installed.
2. Build with `colcon build --packages-select stream_manager`.

## Usage

Run the node:
```bash
ros2 launch stream_manager def.launch.py
```

Published topics:
- `/stream_manager/current_frame` (shm_msgs::msg::Image1m)
- `/stream_manager/delayed_frame` (shm_msgs::msg::Image1m)


## Dependencies

- ROS2 Humble
- OpenCV
- yaml-cpp
- ros2 shm_msgs
## License

MIT License
