#include "stream_manager/gazebo_video_source.hpp"
#include <iostream>

namespace stream_manager {

GazeboVideoSource::GazeboVideoSource(const VideoSourceConfig &config,
                                     rclcpp::Node::SharedPtr node)
    : VideoSourceBase(config, node), receiving_messages_(false) {

  last_connection_check_ = node_->get_clock()->now();
}

GazeboVideoSource::~GazeboVideoSource() { shutdown(); }

std::shared_ptr<cv::Mat> GazeboVideoSource::getFrame() {
  std::lock_guard<std::mutex> lock(frame_mutex_);

  if (!latest_frame_ || latest_frame_->empty()) {
    return nullptr;
  }

  // Return a copy to avoid shared access issues
  return std::make_shared<cv::Mat>(latest_frame_->clone());
}

double GazeboVideoSource::getFPS() const { return getAverageFPS(); }

bool GazeboVideoSource::initialize() {
  if (initialized_) {
    return true;
  }
  std::cout << "initialize gazebo video source" << std::endl;
  // node_ is guaranteed by constructor

  createSubscription();

  initialized_ = true;

  return true;
}

void GazeboVideoSource::shutdown() {

  image_subscription_.reset();

  initialized_ = false;
}

bool GazeboVideoSource::isConnected() const {
  checkConnectionStatus();
  return receiving_messages_;
}

std::string GazeboVideoSource::getSourceInfo() const {
  std::stringstream info;
  info << "Gazebo ROS2 Video Source\n";
  info << "Topic: " << config_.topic_name << "\n";
  info << "Connected: " << (isConnected() ? "Yes" : "No") << "\n";
  info << "Average FPS: " << getAverageFPS() << "\n";

  if (latest_frame_ && !latest_frame_->empty()) {
    info << "Frame size: " << latest_frame_->cols << "x" << latest_frame_->rows
         << "\n";
    info << "Channels: " << latest_frame_->channels();
  }

  return info.str();
}

void GazeboVideoSource::imageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    // Convert ROS image to OpenCV format
    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(msg, msg->encoding);//sensor_msgs::image_encodings::BGR8);
    // Check encoding and convert to BGR if needed
    if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
      cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_RGB2BGR);
    }
    if (!cv_ptr || cv_ptr->image.empty()) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_frame_ = std::make_shared<cv::Mat>(cv_ptr->image.clone());
      last_message_time_ = node_->get_clock()->now();
    }

    receiving_messages_ = true;
    updateFPSCalculation();

  } catch (const cv_bridge::Exception &e) {
    std::cerr << "cv_bridge exception: " << e.what() << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Error in image callback: " << e.what() << std::endl;
  }
}

// node provided in ctor

void GazeboVideoSource::createSubscription() {
  if (!node_) {
    std::cerr << "Node not provided for GazeboVideoSource" << std::endl;
    return;
  }

  try {
    // Create image subscription
    // auto qos =
    // rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    std::cout << "Creating subscription to topic: " << config_.topic_name
              << std::endl;
    image_subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
        config_.topic_name, 10,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
          this->imageCallback(msg);
        });

  } catch (const std::exception &e) {
    std::cerr << "Failed to create subscription: " << e.what() << std::endl;
  }
}

void GazeboVideoSource::checkConnectionStatus() const {
  auto now = node_->get_clock()->now();

  // Check connection periodically
  if ((now - last_connection_check_) > rclcpp::Duration::from_seconds(1.0)) {
    const_cast<GazeboVideoSource *>(this)->last_connection_check_ = now;

    if (receiving_messages_) {
      // Check if we've received messages recently
      auto time_since_last_message = now - last_message_time_;
      if (time_since_last_message > rclcpp::Duration::from_seconds(CONNECTION_TIMEOUT_SEC)) {
        const_cast<GazeboVideoSource *>(this)->receiving_messages_ = false;
      }
    }
  }
}

} // namespace stream_manager