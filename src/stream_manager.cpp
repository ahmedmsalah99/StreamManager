#include "stream_manager/stream_manager.hpp"
#include <iostream>
#include <fstream>

namespace stream_manager {

StreamManager::StreamManager(const std::string& config_path)
    : running_(false)
    , initialized_(false)
    , shutdown_requested_(false)
    , current_processing_fps_(0.0)
    , frames_in_current_second_(0) {
    
    config_manager_ = std::make_unique<ConfigManager>();
    last_fps_update_ = std::chrono::steady_clock::now();
    
    if (!config_path.empty()) {
        loadConfig(config_path);
    } else {
        // Use default configuration
        config_manager_->setConfig(StreamManagerConfig{});
    }
}

StreamManager::StreamManager(const StreamManagerConfig& config)
    : running_(false)
    , initialized_(false)
    , shutdown_requested_(false)
    , current_processing_fps_(0.0)
    , frames_in_current_second_(0) {
    
    config_manager_ = std::make_unique<ConfigManager>();
    config_manager_->setConfig(config);
    last_fps_update_ = std::chrono::steady_clock::now();
}

StreamManager::~StreamManager() {
    shutdown();
}

std::shared_ptr<cv::Mat> StreamManager::getFrame() {
    if (!initialized_ || !video_source_) {
        return nullptr;
    }
    
    return video_source_->getFrame();
}

double StreamManager::getFPS() const {
    if (!initialized_ || !video_source_) {
        return 0.0;
    }
    
    return video_source_->getFPS();
}

std::shared_ptr<cv::Mat> StreamManager::getClosestFrame(const std::chrono::steady_clock::time_point& target_time) const {
    if (!initialized_ || !frame_buffer_) {
        return nullptr;
    }
    
    return frame_buffer_->getClosestFrame(target_time);
}

std::shared_ptr<cv::Mat> StreamManager::getFrameByIdx(size_t index) const {
    if (!initialized_ || !frame_buffer_) {
        return nullptr;
    }
    
    return frame_buffer_->getFrameByIdx(index);
}

std::shared_ptr<cv::Mat> StreamManager::getLatestBufferedFrame() const {
    if (!initialized_ || !frame_buffer_) {
        return nullptr;
    }
    
    return frame_buffer_->getLatestFrame();
}

std::shared_ptr<FrameMetadata> StreamManager::getClosestFrameMetadata(const std::chrono::steady_clock::time_point& target_time) const {
    if (!initialized_ || !frame_buffer_) {
        return nullptr;
    }
    
    return frame_buffer_->getClosestFrameMetadata(target_time);
}

std::shared_ptr<FrameMetadata> StreamManager::getFrameMetadataByIdx(size_t index) const {
    if (!initialized_ || !frame_buffer_) {
        return nullptr;
    }
    
    return frame_buffer_->getFrameMetadataByIdx(index);
}

std::shared_ptr<FrameMetadata> StreamManager::getLatestBufferedFrameMetadata() const {
    if (!initialized_ || !frame_buffer_) {
        return nullptr;
    }
    
    return frame_buffer_->getLatestFrameMetadata();
}

std::shared_ptr<cv::Mat> StreamManager::getFrameFromSharedMemory() const {
    if (!initialized_ || !frame_buffer_) {
        return nullptr;
    }
    
    return frame_buffer_->getLatestFrameFromSharedMemory();
}

std::shared_ptr<cv::Mat> StreamManager::getFrameFromSharedMemory(const FrameMetadata& metadata) const {
    if (!initialized_ || !frame_buffer_) {
        return nullptr;
    }
    
    return frame_buffer_->getFrameFromSharedMemory(metadata);
}

bool StreamManager::initialize() {
    std::lock_guard<std::mutex> lock(manager_mutex_);
    
    if (initialized_) {
        return true;
    }
    
    if (!validateConfiguration()) {
        logError("Configuration validation failed: " + config_manager_->getValidationErrors());
        return false;
    }
    
    // Create frame buffer
    const auto& config = config_manager_->getConfig();
    frame_buffer_ = std::make_unique<FrameBuffer>(config.buffer, config.iceoryx, config.scaling);
    
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
    
    initialized_ = true;
    running_ = true;
    shutdown_requested_ = false;
    
    // Start processing thread
    processing_thread_ = std::thread(&StreamManager::processingThreadFunction, this);
    
    logInfo("StreamManager initialized successfully");
    return true;
}

void StreamManager::shutdown() {
    {
        std::lock_guard<std::mutex> lock(manager_mutex_);
        if (!running_) {
            return;
        }
        
        shutdown_requested_ = true;
        running_ = false;
    }
    
    // Wait for processing thread to finish
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
    
    // Cleanup resources
    cleanupResources();
    
    initialized_ = false;
    logInfo("StreamManager shutdown complete");
}

bool StreamManager::loadConfig(const std::string& config_path) {
    if (!config_manager_->loadFromFile(config_path)) {
        logError("Failed to load configuration from: " + config_path);
        return false;
    }
    
    if (initialized_) {
        // Apply new configuration
        applyConfiguration();
    }
    
    return true;
}

bool StreamManager::updateConfig(const StreamManagerConfig& new_config) {
    config_manager_->setConfig(new_config);
    
    if (!validateConfiguration()) {
        logError("New configuration is invalid: " + config_manager_->getValidationErrors());
        return false;
    }
    
    if (initialized_) {
        applyConfiguration();
    }
    
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

size_t StreamManager::getPoolSlotSize() const {
    const auto& config = config_manager_->getConfig();
    if (config.scaling.enabled) {
        // Calculate frame data size from scaling config
        size_t frame_data_size = config.scaling.target_width * config.scaling.target_height * 3;
        
        // Ensure frame data doesn't exceed iceoryx max chunk size
        if (frame_data_size > config.iceoryx.max_chunk_size) {
            return sizeof(SharedFrameHeader) + config.iceoryx.max_chunk_size;
        } else {
            return sizeof(SharedFrameHeader) + frame_data_size;
        }
    } else {
        // Use the configurable iceoryx max chunk size for frame data + header
        return sizeof(SharedFrameHeader) + config.iceoryx.max_chunk_size;
    }
}

size_t StreamManager::getTotalPoolMemoryUsage() const {
    // Note: With iceoryx memory management, actual memory usage is managed by iceoryx
    // This method returns the theoretical maximum if all buffer slots were used
    const auto& config = config_manager_->getConfig();
    return getPoolSlotSize() * config.buffer.max_size;
}

void StreamManager::processingThreadFunction() {
    logInfo("Processing thread started");
    
    const auto& config = config_manager_->getConfig();
    process_interval_ = std::chrono::duration<double>(1.0 / config.buffer.target_fps);
    last_process_time_ = std::chrono::steady_clock::now();
    
    while (running_ && !shutdown_requested_) {
        try {
            if (shouldProcessFrame()) {
                processFrame();
                last_process_time_ = std::chrono::steady_clock::now();
            }
            
            // Small sleep to prevent excessive CPU usage
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            
        } catch (const std::exception& e) {
            logError("Error in processing thread: " + std::string(e.what()));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    logInfo("Processing thread stopped");
}

bool StreamManager::createVideoSource() {
    const auto& config = config_manager_->getConfig().video_source;
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
    
    // Add to buffer
    double current_fps = video_source_->getFPS();
    bool success = frame_buffer_->addFrame(processed_frame, current_fps);
    
    if (success) {
        // Update processing FPS using sliding window approach (no overflow)
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(now - last_fps_update_).count();
        
        frames_in_current_second_++;
        
        if (elapsed >= 1.0) {
            current_processing_fps_ = static_cast<double>(frames_in_current_second_) / elapsed;
            last_fps_update_ = now;
            frames_in_current_second_ = 0;
        }
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
    process_interval_ = std::chrono::duration<double>(1.0 / config.buffer.target_fps);
}

cv::Mat StreamManager::applyScaling(const cv::Mat& input_frame) const {
    if (!video_source_) {
        return input_frame;
    }
    
    const auto& scaling_config = config_manager_->getConfig().scaling;
    return video_source_->scaleFrame(input_frame, scaling_config);
}

bool StreamManager::shouldProcessFrame() const {
    auto now = std::chrono::steady_clock::now();
    return (now - last_process_time_) >= process_interval_;
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
    last_fps_update_ = std::chrono::steady_clock::now();
}

std::unique_ptr<VideoSourceBase> StreamManager::createVideoSourceFromConfig(const VideoSourceConfig& config) {
    switch (config.type) {
#if ENABLE_GAZEBO
        case VideoSourceConfig::SourceType::GAZEBO_ROS2:
            return std::make_unique<GazeboVideoSource>(config);
#endif

        case VideoSourceConfig::SourceType::USB_CAMERA:
            return std::make_unique<USBVideoSource>(config);
            
        case VideoSourceConfig::SourceType::MAVLINK:
            return std::make_unique<MAVLinkVideoSource>(config);
            
        default:
            logError("Unknown video source type");
            return nullptr;
    }
}

// Utility functions
namespace utils {

std::string videoSourceTypeToString(VideoSourceConfig::SourceType type) {
    switch (type) {
#if ENABLE_GAZEBO
        case VideoSourceConfig::SourceType::GAZEBO_ROS2:
            return "Gazebo ROS2";
#endif
        case VideoSourceConfig::SourceType::USB_CAMERA:
            return "USB Camera";
        case VideoSourceConfig::SourceType::MAVLINK:
            return "MAVLink";
        default:
            return "Unknown";
    }
}

VideoSourceConfig::SourceType stringToVideoSourceType(const std::string& type_str) {
    if (type_str == "usb_camera" || type_str == "USB_CAMERA") {
        return VideoSourceConfig::SourceType::USB_CAMERA;
    } else if (type_str == "mavlink" || type_str == "MAVLINK") {
        return VideoSourceConfig::SourceType::MAVLINK;
    }
#if ENABLE_GAZEBO
    else if (type_str == "gazebo_ros2" || type_str == "GAZEBO_ROS2") {
        return VideoSourceConfig::SourceType::GAZEBO_ROS2;
    }
#endif

    
    return VideoSourceConfig::SourceType::USB_CAMERA; // Default
}

bool isValidFrameSize(int width, int height) {
    return width > 0 && height > 0 && width <= 7680 && height <= 4320; // Up to 8K
}

cv::Size getOptimalFrameSize(const cv::Size& input_size, const cv::Size& target_size) {
    if (!isValidFrameSize(target_size.width, target_size.height)) {
        return input_size;
    }
    
    // Calculate aspect ratios
    double input_aspect = static_cast<double>(input_size.width) / input_size.height;
    double target_aspect = static_cast<double>(target_size.width) / target_size.height;
    
    // If aspect ratios are close, use target size
    if (std::abs(input_aspect - target_aspect) < 0.1) {
        return target_size;
    }
    
    // Otherwise, maintain aspect ratio
    if (input_aspect > target_aspect) {
        // Input is wider, fit to width
        int new_height = static_cast<int>(target_size.width / input_aspect);
        return cv::Size(target_size.width, new_height);
    } else {
        // Input is taller, fit to height
        int new_width = static_cast<int>(target_size.height * input_aspect);
        return cv::Size(new_width, target_size.height);
    }
}

} // namespace utils

} // namespace stream_manager