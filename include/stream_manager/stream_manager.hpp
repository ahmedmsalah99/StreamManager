#pragma once

#include "shm_msgs/opencv_conversions.hpp"
#include "stream_manager/config_manager.hpp"
#include "stream_manager/gazebo_video_source.hpp"
#include "stream_manager/mavlink_video_source.hpp"
#include "stream_manager/usb_video_source.hpp"
#include "stream_manager/video_source_base.hpp"
#include <atomic>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <shm_msgs/msg/image.hpp>

// Define FrameMetadata struct
struct FrameMetadata {
  uint64_t timestamp_ns_system;
  double fps;
  FrameMetadata(uint64_t ts, double f) : timestamp_ns_system(ts), fps(f) {}
};

namespace stream_manager {

class StreamManager : public rclcpp::Node {
public:
  explicit StreamManager(const std::string &config_path = "");
  explicit StreamManager(const StreamManagerConfig &config);
  ~StreamManager();
  bool initialize();
  // Core interface methods - these abstract the video source complexity
  std::shared_ptr<cv::Mat> getFrame();
  double getFPS() const;



  // Lifecycle management
  void shutdown();

  // Configuration management
  bool loadConfig(const std::string &config_path);
  bool updateConfig(const StreamManagerConfig &new_config);
  const StreamManagerConfig &getConfig() const;

  // Status and statistics
  bool isVideoSourceConnected() const;
  std::string getVideoSourceInfo() const;
  double getAverageSourceFPS() const;


  // Scaling configuration (read-only during processing)
  const ScalingConfig &getScalingConfig() const;

  // ROS2 publishing methods
  auto populateShmFrame(std::shared_ptr<FrameMetadata> metadata,
                        std::shared_ptr<cv::Mat> frame,
                        std::shared_ptr<shm_msgs::CvImage> cvimage);
  void publishCurrentFrame();
  void publishDelayedFrame();

private:
  // Core components
  std::unique_ptr<ConfigManager> config_manager_;
  std::unique_ptr<VideoSourceBase> video_source_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // in include/stream_manager/stream_manager.hpp
  rclcpp::Time last_process_time_ros_{this->get_clock()->now()};
  rclcpp::Duration process_interval_ros_{0, 0};

  // Statistics (no overflow-prone counters)
  mutable std::atomic<double> current_processing_fps_;
  mutable rclcpp::Time last_fps_update_;
  mutable size_t frames_in_current_second_;

  // ROS2 publishers
  rclcpp::Publisher<shm_msgs::msg::Image1m>::SharedPtr current_frame_publisher_;
  rclcpp::Publisher<shm_msgs::msg::Image1m>::SharedPtr delayed_frame_publisher_;

  std::shared_ptr<shm_msgs::CvImage> current_cvimage{
      std::make_shared<shm_msgs::CvImage>()};
  std::shared_ptr<shm_msgs::CvImage> delayed_cvimage{
      std::make_shared<shm_msgs::CvImage>()};

  // Current processed frame for publishing
  std::shared_ptr<cv::Mat> current_frame_;
  std::shared_ptr<FrameMetadata> current_metadata_;

  // Methods
  void processingThreadFunction();
  bool createVideoSource();
  void destroyVideoSource();
  bool processFrame();

  // Configuration helpers
  bool validateConfiguration() const;
  void applyConfiguration();

  // Frame processing
  cv::Mat applyScaling(const cv::Mat &input_frame) const;
  bool shouldProcessFrame();

  // Error handling and logging
  void logError(const std::string &message) const;
  void logInfo(const std::string &message) const;
  void logDebug(const std::string &message) const;

  // Cleanup helpers
  void resetStatistics();

  // Factory method for video sources
  std::unique_ptr<VideoSourceBase>
  createVideoSourceFromConfig(const VideoSourceConfig &config);

  // ROS2 setup methods
  void setupPublishers();
};

} // namespace stream_manager