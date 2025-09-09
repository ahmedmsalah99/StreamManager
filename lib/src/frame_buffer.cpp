#include "stream_manager/frame_buffer.hpp"
#include <algorithm>
#include <iostream>
#include <cstring>

namespace stream_manager {

FrameBuffer::FrameBuffer(const BufferConfig& config)
    : max_buffer_size_(config.max_size)
    , target_fps_(config.target_fps)
    , frame_interval_(1.0 / config.target_fps) {
    
    last_add_time_ = std::chrono::steady_clock::now();
    last_add_time_ -= std::chrono::duration_cast<std::chrono::steady_clock::duration>(frame_interval_);
}

FrameBuffer::~FrameBuffer() {}

bool FrameBuffer::addFrame(const cv::Mat& frame, double current_fps) {
    if (frame.empty()) {
        return false;
    }
    
    if (!shouldAddFrame()) {
        return false; // Skip frame to maintain target FPS
    }
    
    // Store frame in owned CPU buffer
    if (!storeFrameInOwnedBuffer(frame, current_fps)) {
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

bool FrameBuffer::addFrame(const cv::Mat& frame, double current_fps, uint64_t ros_time_ns) {
    if (frame.empty()) {
        return false;
    }
    if (!shouldAddFrame()) {
        return false;
    }
    if (!storeFrameInOwnedBuffer(frame, current_fps, ros_time_ns)) {
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        removeOldFrames();
        last_add_time_ = std::chrono::steady_clock::now();
    }
    return true;
}

std::shared_ptr<cv::Mat> FrameBuffer::getLatestFrame() const {
    auto metadata = getLatestFrameMetadata();
    if (!metadata) {
        return nullptr;
    }
    
    return getFrameFromOwnedData(*metadata);
}

std::shared_ptr<cv::Mat> FrameBuffer::getClosestFrame(const std::chrono::steady_clock::time_point& target_time) const {
    auto metadata = getClosestFrameMetadata(target_time);
    if (!metadata) {
        return nullptr;
    }
    
    return getFrameFromOwnedData(*metadata);
}

std::shared_ptr<cv::Mat> FrameBuffer::getFrameByIdx(size_t index) const {
    auto metadata = getFrameMetadataByIdx(index);
    if (!metadata) {
        return nullptr;
    }
    
    return getFrameFromOwnedData(*metadata);
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
    auto target_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        target_time.time_since_epoch()).count();
    
    auto closest_it = std::min_element(metadata_buffer_.begin(), metadata_buffer_.end(),
        [&target_time_ns](const std::shared_ptr<FrameMetadata>& a, const std::shared_ptr<FrameMetadata>& b) {
            auto diff_a = std::abs(static_cast<int64_t>(a->timestamp_ns_system - target_time_ns));
            auto diff_b = std::abs(static_cast<int64_t>(b->timestamp_ns_system - target_time_ns));
            return diff_a < diff_b;
        });
    
    return *closest_it;
}

std::shared_ptr<FrameMetadata> FrameBuffer::getClosestFrameMetadataBySystemNs(uint64_t target_time_ns) const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (metadata_buffer_.empty()) {
        return nullptr;
    }
    auto closest_it = std::min_element(metadata_buffer_.begin(), metadata_buffer_.end(),
        [&target_time_ns](const std::shared_ptr<FrameMetadata>& a, const std::shared_ptr<FrameMetadata>& b) {
            auto diff_a = std::llabs(static_cast<long long>(a->timestamp_ns_system) - static_cast<long long>(target_time_ns));
            auto diff_b = std::llabs(static_cast<long long>(b->timestamp_ns_system) - static_cast<long long>(target_time_ns));
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

std::shared_ptr<cv::Mat> FrameBuffer::getFrameFromOwnedData(const FrameMetadata& metadata) const {
    try {
        cv::Mat frame = reconstructFrameFromOwnedData(metadata);
        if (frame.empty()) {
            return nullptr;
        }
        
        return std::make_shared<cv::Mat>(frame);
        
    } catch (const std::exception& e) {
        std::cerr << "Error reconstructing frame from shared memory: " << e.what() << std::endl;
        return nullptr;
    }
}

// Removed: getLatestFrameFromSharedMemory (not applicable without iceoryx)

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
    
    auto time_span_ns = metadata_buffer_.back()->timestamp_ns_system - metadata_buffer_.front()->timestamp_ns_system;
    auto duration_seconds = static_cast<double>(time_span_ns) / 1e9;
    
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
    
    // Convert nanoseconds back to time_point
    auto time_since_epoch = std::chrono::nanoseconds(metadata_buffer_.front()->timestamp_ns_system);
    return std::chrono::steady_clock::time_point(time_since_epoch);
}

std::chrono::steady_clock::time_point FrameBuffer::getNewestFrameTime() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    if (metadata_buffer_.empty()) {
        return std::chrono::steady_clock::time_point{};
    }
    
    // Convert nanoseconds back to time_point
    auto time_since_epoch = std::chrono::nanoseconds(metadata_buffer_.back()->timestamp_ns_system);
    return std::chrono::steady_clock::time_point(time_since_epoch);
}

void FrameBuffer::removeOldFrames() {
    while (metadata_buffer_.size() > max_buffer_size_) {
        metadata_buffer_.pop_front();
    }
}

std::shared_ptr<FrameMetadata> FrameBuffer::popOldestIfFull() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (metadata_buffer_.size() < max_buffer_size_) {
        return nullptr;
    }
    auto oldest = metadata_buffer_.front();
    metadata_buffer_.pop_front();
    return oldest;
}

bool FrameBuffer::shouldAddFrame() const {
    auto now = std::chrono::steady_clock::now();
    return (now - last_add_time_) >= frame_interval_;
}

size_t FrameBuffer::calculateFrameDataSize(const cv::Mat& frame) const {
    return frame.total() * frame.elemSize();
}

bool FrameBuffer::storeFrameInOwnedBuffer(const cv::Mat& frame, double current_fps) {
    try {
        size_t frame_data_size = calculateFrameDataSize(frame);
        auto data = std::make_shared<std::vector<uint8_t>>(frame_data_size);
        
        if (frame.isContinuous()) {
            std::memcpy(data->data(), frame.data, frame_data_size);
        } else {
            size_t offset = 0;
            for (int i = 0; i < frame.rows; ++i) {
                size_t row_size = frame.cols * frame.elemSize();
                std::memcpy(data->data() + offset, frame.ptr(i), row_size);
                offset += row_size;
            }
        }
        
        auto now_system = std::chrono::system_clock::now();
        uint64_t ts_ns_system = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now_system.time_since_epoch()).count());

        auto metadata = std::make_shared<FrameMetadata>(
            ts_ns_system,
            current_fps,
            frame.cols,
            frame.rows,
            frame.channels(),
            frame.type(),
            data,
            frame_data_size
        );
        
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        metadata_buffer_.push_back(metadata);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error storing frame in owned buffer: " << e.what() << std::endl;
        return false;
    }
}

bool FrameBuffer::storeFrameInOwnedBuffer(const cv::Mat& frame, double current_fps, uint64_t ts_ns_system) {
    try {
        size_t frame_data_size = calculateFrameDataSize(frame);
        auto data = std::make_shared<std::vector<uint8_t>>(frame_data_size);
        if (frame.isContinuous()) {
            std::memcpy(data->data(), frame.data, frame_data_size);
        } else {
            size_t offset = 0;
            for (int i = 0; i < frame.rows; ++i) {
                size_t row_size = frame.cols * frame.elemSize();
                std::memcpy(data->data() + offset, frame.ptr(i), row_size);
                offset += row_size;
            }
        }
        auto metadata = std::make_shared<FrameMetadata>(
            ts_ns_system,
            current_fps,
            frame.cols,
            frame.rows,
            frame.channels(),
            frame.type(),
            data,
            frame_data_size
        );
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        metadata_buffer_.push_back(metadata);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error storing frame in owned buffer: " << e.what() << std::endl;
        return false;
    }
}

cv::Mat FrameBuffer::reconstructFrameFromOwnedData(const FrameMetadata& metadata) const {
    if (!metadata.owned_data || metadata.owned_data->empty()) {
        return cv::Mat();
    }
    return cv::Mat(metadata.height, metadata.width, metadata.cv_type, (void*)metadata.owned_data->data()).clone();
}

// Removed iceoryx initialization and shutdown

} // namespace stream_manager