#include "stream_manager/mavlink_video_source.hpp"
#include <iostream>
#include <sstream>
#include <regex>

namespace stream_manager {

MAVLinkVideoSource::MAVLinkVideoSource(const VideoSourceConfig& config, rclcpp::Node::SharedPtr node)
    : VideoSourceBase(config,node)
    , connection_string_(config.mavlink_address)
    , streaming_(false)
    , stream_requested_(false)
    , current_stream_id_(DEFAULT_STREAM_ID)
    , mavlink_connected_(false) {
}

MAVLinkVideoSource::~MAVLinkVideoSource() {
    shutdown();
}

std::shared_ptr<cv::Mat> MAVLinkVideoSource::getFrame() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    
    if (!latest_frame_ || latest_frame_->empty()) {
        return nullptr;
    }
    
    // Return a copy to avoid shared access issues
    return std::make_shared<cv::Mat>(latest_frame_->clone());
}

double MAVLinkVideoSource::getFPS() const {
    return getAverageFPS();
}

bool MAVLinkVideoSource::initialize() {
    if (initialized_) {
        return true;
    }
    
    if (!parseConnectionString()) {
        std::cerr << "Failed to parse MAVLink connection string: " << connection_string_ << std::endl;
        return false;
    }
    
    if (!initializeMAVLink()) {
        return false;
    }
    
    initialized_ = true;
    
    stream_timer_ = rclcpp::create_timer(node_.get(), node_->get_clock(), std::chrono::milliseconds(20), std::bind(&MAVLinkVideoSource::onStreamTimer, this));
    
    return true;
}

void MAVLinkVideoSource::shutdown() {
    streaming_ = false;
    if (stream_requested_) {
        stopVideoStream();
    }
    if (stream_timer_) {
        stream_timer_.reset();
    }
    shutdownMAVLink();
    initialized_ = false;
}

bool MAVLinkVideoSource::isConnected() const {
    return mavlink_connected_;
}

std::string MAVLinkVideoSource::getSourceInfo() const {
    std::stringstream info;
    info << "MAVLink Video Source\n";
    info << "Connection: " << connection_string_ << "\n";
    info << "Connected: " << (isConnected() ? "Yes" : "No") << "\n";
    info << "Streaming: " << (streaming_ ? "Yes" : "No") << "\n";
    info << "Current stream ID: " << current_stream_id_ << "\n";
    info << "Average FPS: " << getAverageFPS() << "\n";
    
    std::lock_guard<std::mutex> lock(streams_mutex_);
    info << "Available streams: " << available_streams_.size() << "\n";
    
    for (const auto& stream : available_streams_) {
        info << "  Stream " << stream.stream_id << ": " << stream.name 
             << " (" << stream.width << "x" << stream.height 
             << " @ " << stream.framerate << " FPS)\n";
    }
    
    return info.str();
}

bool MAVLinkVideoSource::requestVideoStream() {
    if (!mavlink_connected_) {
        return false;
    }
    
    // This is a placeholder implementation
    // In a real implementation, you would send MAVLink VIDEO_START_STREAMING message
    stream_requested_ = true;
    streaming_ = true;
    
    std::cout << "Requested video stream " << current_stream_id_ << std::endl;
    return true;
}

bool MAVLinkVideoSource::stopVideoStream() {
    if (!mavlink_connected_) {
        return false;
    }
    
    // Placeholder for stopping the stream
    stream_requested_ = false;
    streaming_ = false;
    std::cout << "Stopped video stream " << current_stream_id_ << std::endl;
    return true;
}

// std::vector<VideoStreamInfo> MAVLinkVideoSource::getAvailableStreams() const {
//     std::lock_guard<std::mutex> lock(streams_mutex_);
//     std::vector<VideoStreamInfo> result;
//     for (const auto& s : available_streams_) {
//         result.push_back({s.stream_id, s.name, s.uri, s.width, s.height, s.framerate, s.active});
//     }
//     return result;
// }

bool MAVLinkVideoSource::selectStream(int stream_id) {
    std::lock_guard<std::mutex> lock(streams_mutex_);
    for (auto& s : available_streams_) {
        if (s.stream_id == stream_id) {
            current_stream_id_ = stream_id;
            return true;
        }
    }
    return false;
}

bool MAVLinkVideoSource::initializeMAVLink() {
    try {
        // Initialize MAVLink connection
        mavlink_connection_ = std::make_unique<MAVLinkConnection>();
        
        if (!connectToMAVLink()) {
            return false;
        }
        
        // Initialize available streams with default stream
        {
            std::lock_guard<std::mutex> lock(streams_mutex_);
            available_streams_.clear();
            available_streams_.push_back({DEFAULT_STREAM_ID, "Default Stream", connection_string_, 640, 480, 30.0, false});
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize MAVLink: " << e.what() << std::endl;
        return false;
    }
}

void MAVLinkVideoSource::shutdownMAVLink() {
    mavlink_connection_.reset();
    mavlink_connected_ = false;
}

bool MAVLinkVideoSource::connectToMAVLink() {
    // Placeholder: simulate connection success
    mavlink_connected_ = true;
    return true;
}

void MAVLinkVideoSource::handleMAVLinkMessages() {
    // Placeholder for handling MAVLink messages
}

bool MAVLinkVideoSource::processVideoFrame(const std::vector<uint8_t>& frame_data) {
    try {
        cv::Mat frame = decodeVideoFrame(frame_data);
        if (frame.empty()) {
            return false;
        }
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_frame_ = std::make_shared<cv::Mat>(frame.clone());
        }
        last_frame_received_ = std::chrono::steady_clock::now();
        updateFPSCalculation();
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

void MAVLinkVideoSource::handleHeartbeat() {
    last_heartbeat_ = std::chrono::steady_clock::now();
}

void MAVLinkVideoSource::handleVideoStreamInformation(const std::vector<uint8_t>& payload) {
    // Placeholder parsing
}

void MAVLinkVideoSource::handleVideoStreamStatus(const std::vector<uint8_t>& payload) {
    // Placeholder parsing
}

cv::Mat MAVLinkVideoSource::decodeVideoFrame(const std::vector<uint8_t>& encoded_data) {
    // Placeholder decode
    return cv::Mat();
}

bool MAVLinkVideoSource::parseConnectionString() {
    // Basic validation for now
    return !connection_string_.empty();
}

void MAVLinkVideoSource::updateConnectionStatus() {
    auto now = std::chrono::steady_clock::now();
    if ((now - last_heartbeat_) > HEARTBEAT_TIMEOUT) {
        mavlink_connected_ = false;
    }
}

void MAVLinkVideoSource::resetStreamState() {
    streaming_ = false;
    stream_requested_ = false;
}

void MAVLinkVideoSource::onStreamTimer() {
    try {
        updateConnectionStatus();
        if (mavlink_connected_) {
            handleMAVLinkMessages();
            if (!streaming_ && !stream_requested_) {
                requestVideoStream();
            }
        }
    } catch (const std::exception&) {
    }
}

} // namespace stream_manager