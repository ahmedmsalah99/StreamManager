#pragma once

#include "video_source_base.hpp"
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace stream_manager {

class GazeboVideoSource : public VideoSourceBase {
public:
  explicit GazeboVideoSource(const VideoSourceConfig &config,
                             rclcpp::Node::SharedPtr node);
  ~GazeboVideoSource() override;

  // Implement pure virtual methods
  std::shared_ptr<cv::Mat> getFrame() override;
  double getFPS() const override;

  bool initialize() override;
  void shutdown() override;
  bool isConnected() const override;
  std::string getSourceInfo() const override;

  // ROS2 integration - node provided in constructor
  void createSubscription();

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

  // Frame data
  mutable std::mutex frame_mutex_;
  std::shared_ptr<cv::Mat> latest_frame_;
  rclcpp::Time last_message_time_;

  // Connection status
  std::atomic<bool> receiving_messages_;
  rclcpp::Time last_connection_check_;
  static constexpr double CONNECTION_TIMEOUT_SEC = 5.0;


  // Callbacks and helper methods
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void checkConnectionStatus() const;
};

} // namespace stream_manager