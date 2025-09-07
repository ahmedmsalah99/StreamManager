#pragma once

#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <mutex>
#include <chrono>
#include <deque>
#include <iceoryx_posh/popo/publisher.hpp>
#include <iceoryx_posh/popo/subscriber.hpp>
#include <iceoryx_posh/runtime/posh_runtime.hpp>
#include "config_manager.hpp"

namespace stream_manager {



// Shared memory frame structure for iceoryx
struct SharedFrameHeader {
    uint64_t timestamp_ns;
    uint32_t width;
    uint32_t height;
    uint32_t channels;
    uint32_t type; // CV_8UC3, etc.
    uint64_t frame_id;
    double fps;
    uint32_t data_size;
};

struct SharedFrame {
    SharedFrameHeader header;
    uint8_t data[]; // Variable length data
    // Add constructor that iceoryx can use
    explicit SharedFrame(size_t data_size) {
        // Initialize header with default values
        std::memset(&header, 0, sizeof(header));
        header.data_size = static_cast<uint32_t>(data_size);
        // Don't initialize data[] - it's flexible array member
    }
};
struct FrameMetadata {
    std::chrono::steady_clock::time_point timestamp;
    size_t frame_id;
    double fps;
    uint32_t width;
    uint32_t height;
    uint32_t channels;
    uint32_t cv_type;
    const SharedFrame* shared_frame_ptr;  // Direct pointer to iceoryx shared memory
    size_t data_size;
    
    FrameMetadata() : frame_id(0), fps(0.0), width(0), height(0), channels(0), cv_type(0), shared_frame_ptr(nullptr), data_size(0) {}
    FrameMetadata(size_t id, double current_fps, uint32_t w, uint32_t h, uint32_t ch, uint32_t type, const SharedFrame* ptr, size_t size) 
        : timestamp(std::chrono::steady_clock::now()), frame_id(id), fps(current_fps), 
          width(w), height(h), channels(ch), cv_type(type), shared_frame_ptr(ptr), data_size(size) {}
};
class FrameBuffer {
public:
    explicit FrameBuffer(const BufferConfig& config, const IceoryxConfig& iceoryx_config, const ScalingConfig& scaling_config);
    ~FrameBuffer();

    // Core buffer operations - all through shared memory
    bool addFrame(const cv::Mat& frame, double current_fps);
    std::shared_ptr<cv::Mat> getLatestFrame() const;
    std::shared_ptr<cv::Mat> getClosestFrame(const std::chrono::steady_clock::time_point& target_time) const;
    std::shared_ptr<cv::Mat> getFrameByIdx(size_t index) const;
    
    // Metadata access (zero-copy)
    std::shared_ptr<FrameMetadata> getLatestFrameMetadata() const;
    std::shared_ptr<FrameMetadata> getClosestFrameMetadata(const std::chrono::steady_clock::time_point& target_time) const;
    std::shared_ptr<FrameMetadata> getFrameMetadataByIdx(size_t index) const;
    
    // Direct shared memory frame access
    std::shared_ptr<cv::Mat> getFrameFromSharedMemory(const FrameMetadata& metadata) const;
    std::shared_ptr<cv::Mat> getLatestFrameFromSharedMemory() const;
    
    // Buffer management
    void setTargetFPS(double fps);
    double getTargetFPS() const { return target_fps_; }
    size_t getBufferSize() const;
    size_t getMaxBufferSize() const { return max_buffer_size_; }
    void clearBuffer();
    
    // Statistics
    double getAverageFPS() const;
    std::chrono::steady_clock::time_point getOldestFrameTime() const;
    std::chrono::steady_clock::time_point getNewestFrameTime() const;

private:
    mutable std::mutex buffer_mutex_;
    std::deque<std::shared_ptr<FrameMetadata>> metadata_buffer_;  // Only metadata, no frame data
    
    size_t max_buffer_size_;
    double target_fps_;
    std::chrono::steady_clock::time_point last_add_time_;
    std::chrono::duration<double> frame_interval_;
    
    size_t next_frame_id_;
    
    // Iceoryx shared memory
    std::unique_ptr<iox::popo::Publisher<SharedFrame>> publisher_;
    std::unique_ptr<iox::popo::Subscriber<SharedFrame>> subscriber_;
    IceoryxConfig iceoryx_config_;
    
    // Helper methods
    void removeOldFrames();
    bool shouldAddFrame() const;
    size_t calculateFrameDataSize(const cv::Mat& frame) const;
    
    // Frame operations using iceoryx memory management
    bool publishFrameToSharedMemory(const cv::Mat& frame, double current_fps);
    cv::Mat reconstructFrameFromSharedMemory(const FrameMetadata& metadata) const;
    
    // Initialize iceoryx
    bool initializeIceoryx();
    void shutdownIceoryx();
};

} // namespace stream_manager