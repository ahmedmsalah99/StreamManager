#include "stream_manager/video_source_base.hpp"
#include <algorithm>
#include <opencv2/imgproc.hpp>

namespace stream_manager {

VideoSourceBase::~VideoSourceBase() {}

cv::Mat VideoSourceBase::scaleFrame(const cv::Mat &input_frame,
                                    const ScalingConfig &scaling_config) const {
  if (!scaling_config.enabled || input_frame.empty()) {
    return input_frame;
  }

  if (!needsScaling(input_frame, scaling_config)) {
    return input_frame;
  }

  cv::Mat scaled_frame;
  cv::Size target_size(scaling_config.target_width,
                       scaling_config.target_height);

  try {
    cv::resize(input_frame, scaled_frame, target_size, 0, 0,
               scaling_config.interpolation_method);
    return scaled_frame;
  } catch (const cv::Exception &e) {
    // If scaling fails, return original frame
    return input_frame;
  }
}

bool VideoSourceBase::needsScaling(const cv::Mat &frame,
                                   const ScalingConfig &scaling_config) const {
  if (!scaling_config.enabled || frame.empty()) {
    return false;
  }

  return (frame.cols != scaling_config.target_width ||
          frame.rows != scaling_config.target_height);
}

double VideoSourceBase::getAverageFPS() const {
  std::lock_guard<std::mutex> lock(fps_mutex_);
  return calculated_fps_;
}

void VideoSourceBase::updateFPSCalculation() const {
  std::lock_guard<std::mutex> lock(fps_mutex_);

  frame_timestamps_ros_.push_back(node_->get_clock()->now());
  while (frame_timestamps_ros_.size() > FPS_CALCULATION_WINDOW) {
    frame_timestamps_ros_.pop_front();
  }
  if (frame_timestamps_ros_.size() >= 2) {
    auto span = (frame_timestamps_ros_.back() - frame_timestamps_ros_.front())
                    .seconds();
    if (span > 0.0) {
      calculated_fps_ =
          static_cast<double>(frame_timestamps_ros_.size() - 1) / span;
    }
  }
  last_frame_time_ros_ = node_->get_clock()->now();
}

void VideoSourceBase::resetStatistics() {
  std::lock_guard<std::mutex> lock(fps_mutex_);

  calculated_fps_ = 0.0;
  frame_timestamps_ros_.clear();
  std::cout << "cleared ros timestamp" << std::endl;
  auto now_ros = node_->get_clock()->now();
  last_frame_time_ros_ = now_ros;
  std::cout << "got now ros timestamp" << std::endl;
  // Mirror in steady clock as well for safety
  auto now = node_->get_clock()->now();
  std::cout << "got now timestamp" << std::endl;
  last_frame_time_ = now;
  timeout_start_ = now;
  std::cout << "Statistics reset in video base" << std::endl;
}

bool VideoSourceBase::isTimedOut() const {
  auto now_ros2 = node_->get_clock()->now();
  if (frame_timestamps_ros_.empty()) {
    // since init (use steady timeout_start_ mirrored time if needed)
    auto elapsed = (now_ros2 - last_frame_time_ros_).seconds();
    return elapsed > config_.timeout_seconds;
  } else {
    auto elapsed = (now_ros2 - last_frame_time_ros_).seconds();
    return elapsed > config_.timeout_seconds;
  }
}

} // namespace stream_manager