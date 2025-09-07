#include "stream_manager/mavlink_video_source.hpp"
#include <iostream>
#include <sstream>
#include <regex>

namespace stream_manager {

MAVLinkVideoSource::MAVLinkVideoSource(const VideoSourceConfig& config)
    : VideoSourceBase(config)
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
    running_ = true;
    
    // Start streaming thread
    stream_thread_ = std::thread(&MAVLinkVideoSource::streamThreadFunction, this);
    
    return true;
}

void MAVLinkVideoSource::shutdown() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    streaming_ = false;
    
    if (stream_requested_) {
        stopVideoStream();
    }
    
    if (stream_thread_.joinable()) {
        stream_thread_.join();
    }
    
    shutdownMAVLink();
    initialized_ = false;
}

bool MAVLinkVideoSource::isConnected() const {
    return mavlink_connected_ && running_;
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
    
    // This is a placeholder implementation
    // In a real implementation, you would send MAVLink VIDEO_STOP_STREAMING message
    stream_requested_ = false;
    streaming_ = false;
    
    std::cout << "Stopped video stream " << current_stream_id_ << std::endl;
    return true;
}

std::vector<VideoStreamInfo> MAVLinkVideoSource::getAvailableStreams() const {
    std::lock_guard<std::mutex> lock(streams_mutex_);
    
    std::vector<VideoStreamInfo> result;
    for (const auto& stream : available_streams_) {
        VideoStreamInfo info;
        info.stream_id = stream.stream_id;
        info.name = stream.name;
        info.uri = stream.uri;
        info.width = stream.width;
        info.height = stream.height;
        info.framerate = stream.framerate;
        info.active = stream.active;
        result.push_back(info);
    }
    
    return result;
}

bool MAVLinkVideoSource::selectStream(int stream_id) {
    std::lock_guard<std::mutex> lock(streams_mutex_);
    
    // Check if stream exists
    auto it = std::find_if(available_streams_.begin(), available_streams_.end(),
        [stream_id](const StreamInfo& stream) {
            return stream.stream_id == stream_id;
        });
    
    if (it == available_streams_.end()) {
        return false;
    }
    
    current_stream_id_ = stream_id;
    
    // If currently streaming, restart with new stream
    if (streaming_) {
        stopVideoStream();
        return requestVideoStream();
    }
    
    return true;
}

void MAVLinkVideoSource::streamThreadFunction() {
    while (running_) {
        try {
            updateConnectionStatus();
            
            if (mavlink_connected_) {
                handleMAVLinkMessages();
                
                // Auto-request stream if not already streaming
                if (!streaming_ && !stream_requested_) {
                    requestVideoStream();
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
        } catch (const std::exception& e) {
            std::cerr << "Error in MAVLink stream thread: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
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
            
            StreamInfo default_stream;
            default_stream.stream_id = DEFAULT_STREAM_ID;
            default_stream.name = "Default Camera";
            default_stream.uri = "rtsp://192.168.1.1:554/stream";
            default_stream.width = 1920;
            default_stream.height = 1080;
            default_stream.framerate = 30.0;
            default_stream.active = false;
            
            available_streams_.push_back(default_stream);
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize MAVLink: " << e.what() << std::endl;
        return false;
    }
}

void MAVLinkVideoSource::shutdownMAVLink() {
    mavlink_connected_ = false;
    mavlink_connection_.reset();
    resetStreamState();
}

bool MAVLinkVideoSource::connectToMAVLink() {
    // This is a placeholder implementation
    // In a real implementation, you would establish actual MAVLink connection
    
    if (!mavlink_connection_) {
        return false;
    }
    
    // Parse connection string and connect
    mavlink_connection_->address = "127.0.0.1";
    mavlink_connection_->port = 14550;
    mavlink_connection_->connected = true;
    
    mavlink_connected_ = true;
    last_heartbeat_ = std::chrono::steady_clock::now();
    
    std::cout << "Connected to MAVLink at " << connection_string_ << std::endl;
    return true;
}

void MAVLinkVideoSource::handleMAVLinkMessages() {
    // This is a placeholder implementation
    // In a real implementation, you would:
    // 1. Read MAVLink messages from the connection
    // 2. Handle HEARTBEAT messages
    // 3. Handle VIDEO_STREAM_INFORMATION messages
    // 4. Handle VIDEO_STREAM_STATUS messages
    // 5. Handle actual video data
    
    // Simulate receiving heartbeat
    auto now = std::chrono::steady_clock::now();
    if (now - last_heartbeat_ < HEARTBEAT_TIMEOUT) {
        handleHeartbeat();
    }
    
    // Simulate receiving video frames when streaming
    if (streaming_) {
        // Generate dummy frame for demonstration
        cv::Mat dummy_frame(480, 640, CV_8UC3, cv::Scalar(0, 100, 200));
        
        // Add some dynamic content
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();
        cv::putText(dummy_frame, "MAVLink Frame " + std::to_string(timestamp % 1000),
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_frame_ = std::make_shared<cv::Mat>(dummy_frame.clone());
            last_frame_received_ = now;
        }
        
        updateFPSCalculation();
    }
}

bool MAVLinkVideoSource::processVideoFrame(const std::vector<uint8_t>& frame_data) {
    if (frame_data.empty()) {
        return false;
    }
    
    try {
        cv::Mat frame = decodeVideoFrame(frame_data);
        
        if (frame.empty()) {
            return false;
        }
        
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_frame_ = std::make_shared<cv::Mat>(frame.clone());
            last_frame_received_ = std::chrono::steady_clock::now();
        }
        
        updateFPSCalculation();
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error processing video frame: " << e.what() << std::endl;
        return false;
    }
}

void MAVLinkVideoSource::handleHeartbeat() {
    last_heartbeat_ = std::chrono::steady_clock::now();
    mavlink_connected_ = true;
}

void MAVLinkVideoSource::handleVideoStreamInformation(const std::vector<uint8_t>& payload) {
    // This would parse VIDEO_STREAM_INFORMATION MAVLink message
    // and update available_streams_
    
    // Placeholder implementation
    std::lock_guard<std::mutex> lock(streams_mutex_);
    // Update stream information based on received data
}

void MAVLinkVideoSource::handleVideoStreamStatus(const std::vector<uint8_t>& payload) {
    // This would parse VIDEO_STREAM_STATUS MAVLink message
    // and update streaming status
    
    // Placeholder implementation
}

cv::Mat MAVLinkVideoSource::decodeVideoFrame(const std::vector<uint8_t>& encoded_data) {
    // This is a placeholder implementation
    // In a real implementation, you would:
    // 1. Decode the video frame from the encoded data (H.264, MJPEG, etc.)
    // 2. Convert to OpenCV Mat format
    
    // For now, return empty Mat
    return cv::Mat();
}

bool MAVLinkVideoSource::parseConnectionString() {
    // Parse connection strings like:
    // "udp://127.0.0.1:14550"
    // "tcp://192.168.1.100:5760"
    // "serial:///dev/ttyUSB0:57600"
    
    std::regex udp_regex(R"(udp://([^:]+):(\d+))");
    std::regex tcp_regex(R"(tcp://([^:]+):(\d+))");
    std::regex serial_regex(R"(serial://([^:]+):(\d+))");
    
    std::smatch match;
    
    if (std::regex_match(connection_string_, match, udp_regex) ||
        std::regex_match(connection_string_, match, tcp_regex)) {
        // Valid UDP or TCP connection string
        return true;
    } else if (std::regex_match(connection_string_, match, serial_regex)) {
        // Valid serial connection string
        return true;
    }
    
    return false;
}

void MAVLinkVideoSource::updateConnectionStatus() {
    auto now = std::chrono::steady_clock::now();
    
    // Check heartbeat timeout
    if (mavlink_connected_ && (now - last_heartbeat_) > HEARTBEAT_TIMEOUT) {
        mavlink_connected_ = false;
        streaming_ = false;
        std::cout << "MAVLink connection lost (heartbeat timeout)" << std::endl;
    }
    
    // Check frame timeout when streaming
    if (streaming_ && (now - last_frame_received_) > FRAME_TIMEOUT) {
        std::cout << "Video frame timeout" << std::endl;
    }
}

void MAVLinkVideoSource::resetStreamState() {
    streaming_ = false;
    stream_requested_ = false;
    current_stream_id_ = DEFAULT_STREAM_ID;
    
    std::lock_guard<std::mutex> lock(streams_mutex_);
    available_streams_.clear();
}

} // namespace stream_manager