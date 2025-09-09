#include "stream_manager/stream_manager.hpp"
#include <std_msgs/msg/header.hpp>
#include <iostream>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>


namespace stream_manager {



StreamManager::StreamManager(const std::string& config_path)
    : rclcpp::Node("StreamManager")
    , current_processing_fps_(0.0)
    , frames_in_current_second_(0) 
    {
    
    config_manager_ = std::make_unique<ConfigManager>();
    last_fps_update_ = this->get_clock()->now();
    
    if (!config_path.empty()) {
        loadConfig(config_path);
    } else {
        // Use default configuration
        config_manager_->setConfig(StreamManagerConfig{});
    }
}

StreamManager::StreamManager(const StreamManagerConfig& config)
    : rclcpp::Node("StreamManager")
    , current_processing_fps_(0.0)
    , frames_in_current_second_(0) {
    
    config_manager_ = std::make_unique<ConfigManager>();
    config_manager_->setConfig(config);
    last_fps_update_ = this->get_clock()->now();
}

StreamManager::~StreamManager() {
    shutdown();
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

std::shared_ptr<cv::Mat> StreamManager::getClosestFrame(const rclcpp::Time& target_time) const {
    if (!frame_buffer_) {
        return nullptr;
    }
    
    // convert ROS time to ns epoch and query buffer
    uint64_t target_time_ns = static_cast<uint64_t>(target_time.nanoseconds());
    auto metadata = frame_buffer_->getClosestFrameMetadataBySystemNs(target_time_ns);
    if (!metadata) {
        return nullptr;
    }
    return frame_buffer_->getFrameFromOwnedData(*metadata);
}

std::shared_ptr<cv::Mat> StreamManager::getFrameByIdx(size_t index) const {
    if (!frame_buffer_) {
        return nullptr;
    }
    
    return frame_buffer_->getFrameByIdx(index);
}

std::shared_ptr<cv::Mat> StreamManager::getLatestBufferedFrame() const {
    if (!frame_buffer_) {
        return nullptr;
    }
    
    return frame_buffer_->getLatestFrame();
}

std::shared_ptr<FrameMetadata> StreamManager::getClosestFrameMetadata(const rclcpp::Time& target_time) const {
    if (!frame_buffer_) {
        return nullptr;
    }
    
    // convert ROS time to ns epoch and leverage FrameBuffer's ns-based search
    uint64_t target_time_ns = static_cast<uint64_t>(target_time.nanoseconds());
    return frame_buffer_->getClosestFrameMetadataBySystemNs(target_time_ns);
}

std::shared_ptr<FrameMetadata> StreamManager::getFrameMetadataByIdx(size_t index) const {
    if (!frame_buffer_) {
        return nullptr;
    }
    
    return frame_buffer_->getFrameMetadataByIdx(index);
}

std::shared_ptr<FrameMetadata> StreamManager::getLatestBufferedFrameMetadata() const {
    if (!frame_buffer_) {
        return nullptr;
    }
    
    return frame_buffer_->getLatestFrameMetadata();
}

std::shared_ptr<cv::Mat> StreamManager::getFrameFromOwnedData(const FrameMetadata& metadata) const {
    if (!frame_buffer_) {
        return nullptr;
    }
    return frame_buffer_->getFrameFromOwnedData(metadata);
}


bool StreamManager::initialize() {
    

    if (!validateConfiguration()) {
        logError("Configuration validation failed: " + config_manager_->getValidationErrors());
        return false;
    }
    // Create frame buffer
    const auto& config = config_manager_->getConfig();
    frame_buffer_ = std::make_unique<FrameBuffer>(config.buffer);
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
    timer_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(0.02), std::bind(&StreamManager::processingThreadFunction, this));
    logInfo("StreamManager initialized successfully");
    return true;
}

void StreamManager::shutdown() {
    // Cleanup resources
    cleanupResources();
    logInfo("StreamManager shutdown complete");
}

bool StreamManager::loadConfig(const std::string& config_path) {
    if (!config_manager_->loadFromFile(config_path)) {
        logError("Failed to load configuration from: " + config_path);
        return false;
    }
    
    applyConfiguration();
    
    return true;
}

bool StreamManager::updateConfig(const StreamManagerConfig& new_config) {
    config_manager_->setConfig(new_config);
    
    if (!validateConfiguration()) {
        logError("New configuration is invalid: " + config_manager_->getValidationErrors());
        return false;
    }
    

    applyConfiguration();

    
    return true;
}

const StreamManagerConfig& StreamManager::getConfig() const {
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

size_t StreamManager::getBufferSize() const {
    if (!frame_buffer_) {
        return 0;
    }
    
    return frame_buffer_->getBufferSize();
}

double StreamManager::getBufferTargetFPS() const {
    if (!frame_buffer_) {
        return 0.0;
    }
    
    return frame_buffer_->getTargetFPS();
}

double StreamManager::getAverageSourceFPS() const {
    if (!video_source_) {
        return 0.0;
    }
    
    return video_source_->getAverageFPS();
}

void StreamManager::setBufferTargetFPS(double fps) {
    if (frame_buffer_ && fps > 0.0) {
        frame_buffer_->setTargetFPS(fps);
    }
}

void StreamManager::clearBuffer() {
    if (frame_buffer_) {
        frame_buffer_->clearBuffer();
    }
}

const ScalingConfig& StreamManager::getScalingConfig() const {
    return config_manager_->getConfig().scaling;
}



void StreamManager::processingThreadFunction() {
    
    const auto& config = config_manager_->getConfig();
    process_interval_ros_ = rclcpp::Duration::from_seconds(1.0 / config.buffer.target_fps);
    
    try {
        if (shouldProcessFrame()) {
            processFrame();
            last_process_time_ros_ = this->get_clock()->now();
        }        
    } catch (const std::exception& e) {
        logError("Error in processing thread: " + std::string(e.what()));
    }
    
    
}

bool StreamManager::createVideoSource() {
    const auto& config = config_manager_->getConfig().video_source;
    video_source_ = createVideoSourceFromConfig(config);
    // Gazebo video source needs to be set up with the node
    if (video_source_ != nullptr && config.type == VideoSourceConfig::SourceType::GAZEBO_ROS2) {
        auto gazebo_source = dynamic_cast<GazeboVideoSource*>(video_source_.get());
        if (gazebo_source) {
            gazebo_source->setNode(shared_from_this());
        }
    }
    return video_source_ != nullptr;
}

void StreamManager::destroyVideoSource() {
    if (video_source_) {
        video_source_->shutdown();
        video_source_.reset();
    }
}

bool StreamManager::processFrame() {
    if (!video_source_ || !frame_buffer_) {
        return false;
    }
    
    // Get frame from video source
    auto frame = video_source_->getFrame();
    if (!frame || frame->empty()) {
        return false;
    }
    
    // Apply scaling if needed
    cv::Mat processed_frame = applyScaling(*frame);
    
    // Add to buffer with ROS time
    double current_fps = video_source_->getFPS();
    uint64_t ros_now_ns = static_cast<uint64_t>(this->get_clock()->now().nanoseconds());
    bool success = frame_buffer_->addFrame(processed_frame, current_fps, ros_now_ns);
    
    if (success) {
        // Update processing FPS using sliding window approach (no overflow)
        auto now = this->get_clock()->now();
        auto elapsed = (now - last_fps_update_).seconds();
        
        frames_in_current_second_++;
        
        if (elapsed >= 1.0) {
            current_processing_fps_ = static_cast<double>(frames_in_current_second_) / elapsed;
            last_fps_update_ = now;
            frames_in_current_second_ = 0;
        }
    }
    
    // Publish frames if enabled
    if (success) {
        publishCurrentFrame();
        publishDelayedFrame();
    }
    
    return success;
}

bool StreamManager::validateConfiguration() const {
    return config_manager_->validateConfig();
}

void StreamManager::applyConfiguration() {
    const auto& config = config_manager_->getConfig();
    
    // Update frame buffer target FPS
    if (frame_buffer_) {
        frame_buffer_->setTargetFPS(config.buffer.target_fps);
    }
    
    // Update processing interval
    process_interval_ros_ = rclcpp::Duration::from_seconds(1.0 / config.buffer.target_fps);
}

cv::Mat StreamManager::applyScaling(const cv::Mat& input_frame) const {
    if (!video_source_) {
        return input_frame;
    }
    
    const auto& scaling_config = config_manager_->getConfig().scaling;
    return video_source_->scaleFrame(input_frame, scaling_config);
}

bool StreamManager::shouldProcessFrame() {
    auto now = this->get_clock()->now();
    return (now - last_process_time_ros_) >= process_interval_ros_;
}

void StreamManager::logError(const std::string& message) const {
    std::cerr << "[StreamManager ERROR] " << message << std::endl;
}

void StreamManager::logInfo(const std::string& message) const {
    const auto& config = config_manager_->getConfig();
    if (config.enable_debug_logging) {
        std::cout << "[StreamManager INFO] " << message << std::endl;
    }
}

void StreamManager::logDebug(const std::string& message) const {
    const auto& config = config_manager_->getConfig();
    if (config.enable_debug_logging) {
        std::cout << "[StreamManager DEBUG] " << message << std::endl;
    }
}

void StreamManager::cleanupResources() {
    destroyVideoSource();
    frame_buffer_.reset();
    resetStatistics();
}

void StreamManager::resetStatistics() {
    current_processing_fps_ = 0.0;
    frames_in_current_second_ = 0;
    last_fps_update_ = this->get_clock()->now();
}

std::unique_ptr<VideoSourceBase> StreamManager::createVideoSourceFromConfig(const VideoSourceConfig& config) {
    switch (config.type) {
        case VideoSourceConfig::SourceType::GAZEBO_ROS2:
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
    const auto& config = config_manager_->getConfig().publishing;
    
    if (config.enable_current_publishing) {
        current_frame_publisher_ = this->create_publisher<shm_msgs::msg::Image1m>(
            config.current_frame_topic, 10);
    }
    
    if (config.enable_delayed_publishing) {
        delayed_frame_publisher_ = this->create_publisher<shm_msgs::msg::Image1m>(
            config.delayed_frame_topic, 10);
    }
}


auto StreamManager::populateShmFrame(std::shared_ptr<FrameMetadata> metadata,std::shared_ptr<cv::Mat> frame,std::shared_ptr<shm_msgs::CvImage> cvimage){
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
    auto& msg = loaned_msg.get();
    cvimage->toImageMsg(msg);
    return loaned_msg;
}

void StreamManager::publishCurrentFrame() {
    if (!current_frame_publisher_) {
        return;
    }
    
    auto frame = getLatestBufferedFrame();
    if (!frame || frame->empty()) {
        return;
    }
    
    try {
        auto metadata = getLatestBufferedFrameMetadata();
        auto loaned_msg = populateShmFrame(metadata, frame,current_cvimage);
        current_frame_publisher_->publish(std::move(loaned_msg));
    } catch (const cv_bridge::Exception& e) {
        logError("Failed to convert frame for current publishing: " + std::string(e.what()));
    }
}

void StreamManager::publishDelayedFrame() {
    if (!delayed_frame_publisher_) {
        return;
    }
    // new logic: pop oldest only if buffer is full
    auto oldest_metadata = frame_buffer_->popOldestIfFull();
    if (!oldest_metadata) {
        return;
    }
    auto frame = getFrameFromOwnedData(*oldest_metadata);
    if (!frame || frame->empty()) {
        return;
    }

    try {
        auto metadata = getLatestBufferedFrameMetadata();
        auto loaned_msg = populateShmFrame(metadata, frame,delayed_cvimage);
        delayed_frame_publisher_->publish(std::move(loaned_msg));
    } catch (const cv_bridge::Exception& e) {
        logError("Failed to convert frame for delayed publishing: " + std::string(e.what()));
    }
}

} // namespace stream_manager

  int main(int argc, char * argv[])
  {
    std::string config_path = ament_index_cpp::get_package_share_directory("stream_manager") + "/config/default_config.yaml";
    rclcpp::init(argc, argv);
    auto node = std::make_shared<stream_manager::StreamManager>(config_path);
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  }