#include "stream_manager/stream_manager.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <iostream>
#include <std_msgs/msg/header.hpp>

namespace stream_manager {





StreamManager::StreamManager(const std::string &config_path)
    : rclcpp::Node("StreamManager"), current_processing_fps_(0.0),
      frames_in_current_second_(0) {

  config_manager_ = std::make_unique<ConfigManager>();
  last_fps_update_ = this->get_clock()->now();

  if (!config_path.empty()) {
    loadConfig(config_path);
  } else {
    // Use default configuration
    useParametersConfigs();
  }
}

StreamManager::StreamManager(const StreamManagerConfig &config)
    : rclcpp::Node("StreamManager"), current_processing_fps_(0.0),
      frames_in_current_second_(0) {

  config_manager_ = std::make_unique<ConfigManager>();
  config_manager_->setConfig(config);
  last_fps_update_ = this->get_clock()->now();
}

StreamManager::~StreamManager() { shutdown(); }

void StreamManager::useParametersConfigs(){
  // --- Video source configuration ---
  this->declare_parameter<std::string>("video_source.type", "gazebo_ros2");
  this->declare_parameter<std::string>("video_source.topic_name", "/camera/image_raw");
  this->declare_parameter<int>("video_source.device_id", 0);
  this->declare_parameter<std::string>("video_source.mavlink_address", "udp://127.0.0.1:14550");
  this->declare_parameter<double>("video_source.timeout_seconds", 5.0);

  // --- Frame buffer configuration ---
  this->declare_parameter<double>("buffer.target_fps", 50.0);

  // --- Frame scaling configuration ---
  this->declare_parameter<bool>("scaling.enabled", false);
  this->declare_parameter<int>("scaling.target_width", 640);
  this->declare_parameter<int>("scaling.target_height", 480);
  this->declare_parameter<int>("scaling.interpolation_method", 1);

  // --- ROS2 publishing configuration ---
  this->declare_parameter<std::string>("publishing.current_frame_topic", "/stream_manager/current_frame");
  this->declare_parameter<bool>("publishing.enable_current_publishing", true);

  // --- Debug settings ---
  this->declare_parameter<bool>("enable_debug_logging", true);

  StreamManagerConfig config = paramsToConfig();
  config_manager_->setConfig(config);
}
StreamManagerConfig StreamManager::paramsToConfig()
{
  StreamManagerConfig config;

  // --- Video source configuration ---
  std::string type_str = this->get_parameter("video_source.type").as_string();
  if (type_str == "gazebo_ros2")
    config.video_source.type = VideoSourceConfig::SourceType::GAZEBO_ROS2;
  else if (type_str == "usb_camera")
    config.video_source.type = VideoSourceConfig::SourceType::USB_CAMERA;
  else if (type_str == "mavlink")
    config.video_source.type = VideoSourceConfig::SourceType::MAVLINK;
  else {
    RCLCPP_WARN(this->get_logger(),
                "Unknown video_source.type '%s', defaulting to USB_CAMERA",
                type_str.c_str());
    config.video_source.type = VideoSourceConfig::SourceType::USB_CAMERA;
  }

  config.video_source.topic_name =
      this->get_parameter("video_source.topic_name").as_string();
  config.video_source.device_id =
      this->get_parameter("video_source.device_id").as_int();
  config.video_source.mavlink_address =
      this->get_parameter("video_source.mavlink_address").as_string();
  config.video_source.timeout_seconds =
      this->get_parameter("video_source.timeout_seconds").as_double();

  // --- Frame buffer configuration ---
  config.buffer.target_fps =
      this->get_parameter("buffer.target_fps").as_double();

  // --- Frame scaling configuration ---
  config.scaling.enabled =
      this->get_parameter("scaling.enabled").as_bool();
  config.scaling.target_width =
      this->get_parameter("scaling.target_width").as_int();
  config.scaling.target_height =
      this->get_parameter("scaling.target_height").as_int();
  config.scaling.interpolation_method =
      this->get_parameter("scaling.interpolation_method").as_int();

  // --- ROS2 publishing configuration ---
  config.publishing.current_frame_topic =
      this->get_parameter("publishing.current_frame_topic").as_string();
  config.publishing.enable_current_publishing =
      this->get_parameter("publishing.enable_current_publishing").as_bool();

  // --- Debug settings ---
  config.enable_debug_logging =
      this->get_parameter("enable_debug_logging").as_bool();

  return config;
}
std::shared_ptr<cv::Mat> StreamManager::getFrame() {
  if (!video_source_) {
    return nullptr;
  }

  return video_source_->getFrame();
}

double StreamManager::getFPS() const {
  if (!video_source_) {
    return 0.0;
  }

  return video_source_->getFPS();
}















bool StreamManager::initialize() {

  if (!validateConfiguration()) {
    logError("Configuration validation failed: " +
             config_manager_->getValidationErrors());
    return false;
  }
  // Setup ROS2 publishers
  setupPublishers();

  // Create video source
  if (!createVideoSource()) {
    logError("Failed to create video source");
    return false;
  }

  // Initialize video source
  if (!video_source_->initialize()) {
    logError("Failed to initialize video source");
    destroyVideoSource();
    return false;
  }
  // Apply configuration
  applyConfiguration();

  // Start processing thread
  timer_ = rclcpp::create_timer(
      this, this->get_clock(), rclcpp::Duration::from_seconds(0.02),
      std::bind(&StreamManager::processingThreadFunction, this));
  logInfo("StreamManager initialized successfully");
  return true;
}

void StreamManager::shutdown() {
  // Cleanup resources
  logInfo("StreamManager shutdown complete");
}

bool StreamManager::loadConfig(const std::string &config_path) {
  if (!config_manager_->loadFromFile(config_path)) {
    logError("Failed to load configuration from: " + config_path);
    return false;
  }

  applyConfiguration();

  return true;
}

bool StreamManager::updateConfig(const StreamManagerConfig &new_config) {
  config_manager_->setConfig(new_config);

  if (!validateConfiguration()) {
    logError("New configuration is invalid: " +
             config_manager_->getValidationErrors());
    return false;
  }

  applyConfiguration();

  return true;
}

const StreamManagerConfig &StreamManager::getConfig() const {
  return config_manager_->getConfig();
}

bool StreamManager::isVideoSourceConnected() const {
  return video_source_ && video_source_->isConnected();
}

std::string StreamManager::getVideoSourceInfo() const {
  if (!video_source_) {
    return "No video source available";
  }

  return video_source_->getSourceInfo();
}


double StreamManager::getAverageSourceFPS() const {
  if (!video_source_) {
    return 0.0;
  }

  return video_source_->getAverageFPS();
}


const ScalingConfig &StreamManager::getScalingConfig() const {
  return config_manager_->getConfig().scaling;
}

void StreamManager::processingThreadFunction() {

  const auto &config = config_manager_->getConfig();
  process_interval_ros_ =
      rclcpp::Duration::from_seconds(1.0 / config.buffer.target_fps);

  try {
    if (shouldProcessFrame()) {
      processFrame();
      last_process_time_ros_ = this->get_clock()->now();
    }
  } catch (const std::exception &e) {
    logError("Error in processing thread: " + std::string(e.what()));
  }
}

bool StreamManager::createVideoSource() {
  const auto &config = config_manager_->getConfig().video_source;
  video_source_ = createVideoSourceFromConfig(config);
  return video_source_ != nullptr;
}

void StreamManager::destroyVideoSource() {
  if (video_source_) {
    video_source_->shutdown();
    video_source_.reset();
  }
}

bool StreamManager::processFrame() {
  if (!video_source_) {
    return false;
  }

  // Get frame from video source
  auto frame = video_source_->getFrame();
  if (!frame || frame->empty()) {
    return false;
  }

  // Apply scaling if needed
  cv::Mat processed_frame = applyScaling(*frame);

  // Store current frame and metadata
  current_frame_ = std::make_shared<cv::Mat>(processed_frame);
  double current_fps = video_source_->getFPS();
  uint64_t ros_now_ns =
      static_cast<uint64_t>(this->get_clock()->now().nanoseconds());
  current_metadata_ = std::make_shared<FrameMetadata>(ros_now_ns, current_fps);

  // Update processing FPS using sliding window approach (no overflow)
  auto now = this->get_clock()->now();
  auto elapsed = (now - last_fps_update_).seconds();

  frames_in_current_second_++;

  if (elapsed >= 1.0) {
    current_processing_fps_ =
        static_cast<double>(frames_in_current_second_) / elapsed;
    last_fps_update_ = now;
    frames_in_current_second_ = 0;
  }

  // Publish frames if enabled
  publishCurrentFrame();
  publishDelayedFrame();

  return true;
}

bool StreamManager::validateConfiguration() const {
  return config_manager_->validateConfig();
}

void StreamManager::applyConfiguration() {
  const auto &config = config_manager_->getConfig();

  // Update processing interval
  process_interval_ros_ =
      rclcpp::Duration::from_seconds(1.0 / config.buffer.target_fps);
}

cv::Mat StreamManager::applyScaling(const cv::Mat &input_frame) const {
  if (!video_source_) {
    return input_frame;
  }

  const auto &scaling_config = config_manager_->getConfig().scaling;
  return video_source_->scaleFrame(input_frame, scaling_config);
}

bool StreamManager::shouldProcessFrame() {
  auto now = this->get_clock()->now();
  return (now - last_process_time_ros_) >= process_interval_ros_;
}

void StreamManager::logError(const std::string &message) const {
  std::cerr << "[StreamManager ERROR] " << message << std::endl;
}

void StreamManager::logInfo(const std::string &message) const {
  const auto &config = config_manager_->getConfig();
  if (config.enable_debug_logging) {
    std::cout << "[StreamManager INFO] " << message << std::endl;
  }
}

void StreamManager::logDebug(const std::string &message) const {
  const auto &config = config_manager_->getConfig();
  if (config.enable_debug_logging) {
    std::cout << "[StreamManager DEBUG] " << message << std::endl;
  }
}


void StreamManager::resetStatistics() {
  current_processing_fps_ = 0.0;
  frames_in_current_second_ = 0;
  last_fps_update_ = this->get_clock()->now();
}

std::unique_ptr<VideoSourceBase>
StreamManager::createVideoSourceFromConfig(const VideoSourceConfig &config) {
  switch (config.type) {
  case VideoSourceConfig::SourceType::GAZEBO_ROS2:
    logInfo("gazebo ros2 video source");
    return std::make_unique<GazeboVideoSource>(config, shared_from_this());

  case VideoSourceConfig::SourceType::USB_CAMERA:
    return std::make_unique<USBVideoSource>(config, shared_from_this());

  case VideoSourceConfig::SourceType::MAVLINK:
    return std::make_unique<MAVLinkVideoSource>(config, shared_from_this());

  default:
    logError("Unknown video source type");
    return nullptr;
  }
}

void StreamManager::setupPublishers() {
  const auto &config = config_manager_->getConfig().publishing;

  if (config.enable_current_publishing) {
    current_frame_publisher_ = this->create_publisher<shm_msgs::msg::Image1m>(
        config.current_frame_topic, 10);
  }

  if (config.enable_delayed_publishing) {
    delayed_frame_publisher_ = this->create_publisher<shm_msgs::msg::Image1m>(
        config.delayed_frame_topic, 10);
  }
}

auto StreamManager::populateShmFrame(
    std::shared_ptr<FrameMetadata> metadata, std::shared_ptr<cv::Mat> frame,
    std::shared_ptr<shm_msgs::CvImage> cvimage) {
  cvimage->image = *frame;
  cvimage->header.frame_id = "camera_frame";
  cvimage->encoding = "bgr8";

  if (metadata) {
    // use system timestamp from metadata
    rclcpp::Time stamp(static_cast<int64_t>(metadata->timestamp_ns_system));
    cvimage->header.stamp = stamp;
  } else {
    cvimage->header.stamp = this->get_clock()->now();
  }
  auto loaned_msg = current_frame_publisher_->borrow_loaned_message();
  auto &msg = loaned_msg.get();
  cvimage->toImageMsg(msg);
  return loaned_msg;
}

void StreamManager::publishCurrentFrame() {
  if (!current_frame_publisher_ || !current_frame_ || current_frame_->empty()) {
    return;
  }

  try {
    auto loaned_msg = populateShmFrame(current_metadata_, current_frame_, current_cvimage);
    current_frame_publisher_->publish(std::move(loaned_msg));
  } catch (const cv_bridge::Exception &e) {
    logError("Failed to convert frame for current publishing: " +
             std::string(e.what()));
  }
}

void StreamManager::publishDelayedFrame() {
  // Disabled delayed publishing since frame_buffer is removed
  return;
}

} // namespace stream_manager

int main(int argc, char *argv[]) {
  // std::string config_path =
  //     ament_index_cpp::get_package_share_directory("stream_manager") +
  //     "/config/default_config.yaml";
  rclcpp::init(argc, argv);
  // auto node = std::make_shared<stream_manager::StreamManager>(config_path);
  auto node = std::make_shared<stream_manager::StreamManager>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}