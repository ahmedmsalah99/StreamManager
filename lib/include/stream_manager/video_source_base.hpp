#pragma once

#include "config_manager.hpp"
#include <atomic>
#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace stream_manager {

class VideoSourceBase {
public:
  virtual ~VideoSourceBase();

  // Pure virtual methods that must be implemented by derived classes
  virtual std::shared_ptr<cv::Mat> getFrame() = 0;
  virtual double getFPS() const = 0;

  // Common interface methods
  virtual bool initialize() = 0;
  virtual void shutdown(){};
  virtual bool isConnected() const = 0;
  virtual std::string getSourceInfo() const = 0;
  virtual void setNode(rclcpp::Node::SharedPtr node) { node_ = node; }
  explicit VideoSourceBase(const VideoSourceConfig &config,
                           rclcpp::Node::SharedPtr node)
      : config_(config), initialized_(false),
        last_frame_time_(std::chrono::steady_clock::now()),
        calculated_fps_(0.0), node_(std::move(node)) {
    std::cout << "in video base" << std::endl;
    resetStatistics();
  }

  // Configuration and status
  const VideoSourceConfig &getConfig() const { return config_; }
  bool isInitialized() const { return initialized_; }

  // Frame scaling functionality
  cv::Mat scaleFrame(const cv::Mat &input_frame,
                     const ScalingConfig &scaling_config) const;
  bool needsScaling(const cv::Mat &frame,
                    const ScalingConfig &scaling_config) const;

  // Statistics
  virtual std::chrono::steady_clock::time_point getLastFrameTime() const {
    return last_frame_time_;
  }
  virtual double getAverageFPS() const;

protected:
  VideoSourceConfig config_;
  std::atomic<bool> initialized_;

  // Frame statistics (no overflow-prone counters)
  mutable std::chrono::steady_clock::time_point last_frame_time_;
  mutable rclcpp::Time last_frame_time_ros_;

  // FPS calculation using sliding window (bounded size)
  mutable std::mutex fps_mutex_;
  mutable std::deque<rclcpp::Time> frame_timestamps_ros_;
  mutable double calculated_fps_;
  static constexpr size_t FPS_CALCULATION_WINDOW = 30;

  // Helper methods
  void updateFPSCalculation() const;
  void resetStatistics();

  // Timeout handling
  bool isTimedOut() const;
  std::chrono::steady_clock::time_point timeout_start_;

  // ROS2 integration for timers/clock
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace stream_manager