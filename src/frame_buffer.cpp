#include "stream_manager/frame_buffer.hpp"
#include <algorithm>
#include <iostream>
#include <cstring>

namespace stream_manager {

FrameBuffer::FrameBuffer(const BufferConfig& config, const IceoryxConfig& iceoryx_config, const ScalingConfig& scaling_config)
    : max_buffer_size_(config.max_size)
    , target_fps_(config.target_fps)
    , frame_interval_(1.0 / config.target_fps)
    , next_frame_id_(0)
    , iceoryx_config_(iceoryx_config) {
    
    last_add_time_ = std::chrono::steady_clock::now();
    last_add_time_ -= std::chrono::duration_cast<std::chrono::steady_clock::duration>(frame_interval_);
    
    
    if (!initializeIceoryx()) {
        std::cerr << "Warning: Failed to initialize iceoryx shared memory" << std::endl;
    }
}

FrameBuffer::~FrameBuffer() {
    shutdownIceoryx();
}

bool FrameBuffer::addFrame(const cv::Mat& frame, double current_fps) {
    if (frame.empty()) {
        return false;
    }
    
    if (!shouldAddFrame()) {
        return false; // Skip frame to maintain target FPS
    }
    
    // Publish frame to iceoryx shared memory
    if (!publishFrameToSharedMemory(frame, current_fps)) {
        return false;
    }
    
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        
        // Remove old frames if buffer is full
        removeOldFrames();
        
        // Update timing
        last_add_time_ = std::chrono::steady_clock::now();
    }
    
    return true;
}

std::shared_ptr<cv::Mat> FrameBuffer::getLatestFrame() const {
    auto metadata = getLatestFrameMetadata();
    if (!metadata) {
        return nullptr;
    }
    
    return getFrameFromSharedMemory(*metadata);
}

std::shared_ptr<cv::Mat> FrameBuffer::getClosestFrame(const std::chrono::steady_clock::time_point& target_time) const {
    auto metadata = getClosestFrameMetadata(target_time);
    if (!metadata) {
        return nullptr;
    }
    
    return getFrameFromSharedMemory(*metadata);
}

std::shared_ptr<cv::Mat> FrameBuffer::getFrameByIdx(size_t index) const {
    auto metadata = getFrameMetadataByIdx(index);
    if (!metadata) {
        return nullptr;
    }
    
    return getFrameFromSharedMemory(*metadata);
}

std::shared_ptr<FrameMetadata> FrameBuffer::getLatestFrameMetadata() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    if (metadata_buffer_.empty()) {
        return nullptr;
    }
    
    return metadata_buffer_.back();
}

std::shared_ptr<FrameMetadata> FrameBuffer::getClosestFrameMetadata(const std::chrono::steady_clock::time_point& target_time) const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    if (metadata_buffer_.empty()) {
        return nullptr;
    }
    
    // Find the metadata with timestamp closest to target_time
    auto closest_it = std::min_element(metadata_buffer_.begin(), metadata_buffer_.end(),
        [&target_time](const std::shared_ptr<FrameMetadata>& a, const std::shared_ptr<FrameMetadata>& b) {
            auto diff_a = std::abs(std::chrono::duration_cast<std::chrono::microseconds>(
                a->timestamp - target_time).count());
            auto diff_b = std::abs(std::chrono::duration_cast<std::chrono::microseconds>(
                b->timestamp - target_time).count());
            return diff_a < diff_b;
        });
    
    return *closest_it;
}

std::shared_ptr<FrameMetadata> FrameBuffer::getFrameMetadataByIdx(size_t index) const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    if (index >= metadata_buffer_.size()) {
        return nullptr;
    }
    
    return metadata_buffer_[index];
}

std::shared_ptr<cv::Mat> FrameBuffer::getFrameFromSharedMemory(const FrameMetadata& metadata) const {
    try {
        cv::Mat frame = reconstructFrameFromSharedMemory(metadata);
        if (frame.empty()) {
            return nullptr;
        }
        
        return std::make_shared<cv::Mat>(frame);
        
    } catch (const std::exception& e) {
        std::cerr << "Error reconstructing frame from shared memory: " << e.what() << std::endl;
        return nullptr;
    }
}

std::shared_ptr<cv::Mat> FrameBuffer::getLatestFrameFromSharedMemory() const {
    if (!subscriber_) {
        return nullptr;
    }
    
    try {
        auto chunk_result = subscriber_->take();
        if (chunk_result.has_error()) {
            return nullptr; // No new frame available or error
        }
        
        const auto& sample = chunk_result.value();
        const auto* shared_frame = sample.get();
        
        // Create cv::Mat directly from shared memory (zero-copy view)
        cv::Mat frame(shared_frame->header.height, shared_frame->header.width, 
                      shared_frame->header.type, (void*)shared_frame->data);
        
        // Return a clone to ensure data persistence after shared memory is released
        return std::make_shared<cv::Mat>(frame.clone());
        
    } catch (const std::exception& e) {
        std::cerr << "Error reading frame from shared memory: " << e.what() << std::endl;
        return nullptr;
    }
}

void FrameBuffer::setTargetFPS(double fps) {
    if (fps > 0.0) {
        target_fps_ = fps;
        frame_interval_ = std::chrono::duration<double>(1.0 / fps);
    }
}

size_t FrameBuffer::getBufferSize() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return metadata_buffer_.size();
}

void FrameBuffer::clearBuffer() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    metadata_buffer_.clear();
}

double FrameBuffer::getAverageFPS() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    if (metadata_buffer_.size() < 2) {
        return 0.0;
    }
    
    auto time_span = metadata_buffer_.back()->timestamp - metadata_buffer_.front()->timestamp;
    auto duration_seconds = std::chrono::duration<double>(time_span).count();
    
    if (duration_seconds <= 0.0) {
        return 0.0;
    }
    
    return static_cast<double>(metadata_buffer_.size() - 1) / duration_seconds;
}

std::chrono::steady_clock::time_point FrameBuffer::getOldestFrameTime() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    if (metadata_buffer_.empty()) {
        return std::chrono::steady_clock::time_point{};
    }
    
    return metadata_buffer_.front()->timestamp;
}

std::chrono::steady_clock::time_point FrameBuffer::getNewestFrameTime() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    if (metadata_buffer_.empty()) {
        return std::chrono::steady_clock::time_point{};
    }
    
    return metadata_buffer_.back()->timestamp;
}

void FrameBuffer::removeOldFrames() {
    while (metadata_buffer_.size() > max_buffer_size_) {
        metadata_buffer_.pop_front();
    }
}

bool FrameBuffer::shouldAddFrame() const {
    auto now = std::chrono::steady_clock::now();
    return (now - last_add_time_) >= frame_interval_;
}

size_t FrameBuffer::calculateFrameDataSize(const cv::Mat& frame) const {
    return frame.total() * frame.elemSize();
}

bool FrameBuffer::publishFrameToSharedMemory(const cv::Mat& frame, double current_fps) {
    if (!publisher_) {
        return false;
    }
    
    try {
        size_t frame_data_size = calculateFrameDataSize(frame);
        size_t total_size = sizeof(SharedFrameHeader) + frame_data_size;
        
        if (frame_data_size > iceoryx_config_.max_chunk_size) {
            std::cerr << "Frame data too large for iceoryx max chunk size" << std::endl;
            return false;
        }
        
        // Use iceoryx loan to get shared memory
        auto loan_result = publisher_->loan(total_size);
        if (loan_result.has_error()) {
            std::cerr << "Failed to loan memory from iceoryx" << std::endl;
            return false;
        }
        
        auto& sample = loan_result.value();
        SharedFrame* shared_frame = sample.get();
        
        // Fill header
        auto now = std::chrono::steady_clock::now();
        auto timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        
        shared_frame->header.timestamp_ns = static_cast<uint64_t>(timestamp_ns);
        shared_frame->header.width = static_cast<uint32_t>(frame.cols);
        shared_frame->header.height = static_cast<uint32_t>(frame.rows);
        shared_frame->header.channels = static_cast<uint32_t>(frame.channels());
        shared_frame->header.type = static_cast<uint32_t>(frame.type());
        shared_frame->header.frame_id = next_frame_id_++;
        shared_frame->header.fps = current_fps;
        shared_frame->header.data_size = static_cast<uint32_t>(frame_data_size);
        
        // Copy frame data (this is the only copy operation)
        if (frame.isContinuous()) {
            std::memcpy(shared_frame->data, frame.data, frame_data_size);
        } else {
            // Handle non-continuous matrices
            size_t offset = 0;
            for (int i = 0; i < frame.rows; ++i) {
                size_t row_size = frame.cols * frame.elemSize();
                std::memcpy(shared_frame->data + offset, frame.ptr(i), row_size);
                offset += row_size;
            }
        }
        
        // Create metadata with direct pointer to iceoryx memory
        auto metadata = std::make_shared<FrameMetadata>(
            shared_frame->header.frame_id,
            current_fps,
            frame.cols,
            frame.rows,
            frame.channels(),
            frame.type(),
            shared_frame,  // Direct pointer to iceoryx memory
            frame_data_size
        );
        
        // Add metadata to buffer before publishing
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            metadata_buffer_.push_back(metadata);
        }
        
        // Publish the frame
        sample.publish();
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error publishing frame to shared memory: " << e.what() << std::endl;
        return false;
    }
}

cv::Mat FrameBuffer::reconstructFrameFromSharedMemory(const FrameMetadata& metadata) const {
    if (!metadata.shared_frame_ptr) {
        return cv::Mat();
    }
    
    const SharedFrame* shared_frame = metadata.shared_frame_ptr;
    
    // Create cv::Mat that references the shared memory data (zero-copy view)
    cv::Mat frame(metadata.height, metadata.width, metadata.cv_type, (void*)shared_frame->data);
    
    // Return a clone to ensure data persistence
    return frame.clone();
}

bool FrameBuffer::initializeIceoryx() {
    try {
        // Initialize iceoryx runtime
        iox::runtime::PoshRuntime::initRuntime("StreamManager");
        
        // Create publisher and subscriber
        iox::capro::ServiceDescription service_desc(
            iox::capro::IdString_t(iox::cxx::TruncateToCapacity, iceoryx_config_.service_name),
            iox::capro::IdString_t(iox::cxx::TruncateToCapacity, iceoryx_config_.instance_name),
            iox::capro::IdString_t(iox::cxx::TruncateToCapacity, iceoryx_config_.event_name)
        );
        
        publisher_ = std::make_unique<iox::popo::Publisher<SharedFrame>>(service_desc);
        subscriber_ = std::make_unique<iox::popo::Subscriber<SharedFrame>>(service_desc);
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize iceoryx: " << e.what() << std::endl;
        return false;
    }
}

void FrameBuffer::shutdownIceoryx() {
    publisher_.reset();
    subscriber_.reset();
}

} // namespace stream_manager