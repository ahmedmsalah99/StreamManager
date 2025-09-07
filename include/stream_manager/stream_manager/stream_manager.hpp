#pragma once

#include "config_manager.hpp"
#include "frame_buffer.hpp"
#include "video_source_base.hpp"
#if ENABLE_GAZEBO
    #include "gazebo_video_source.hpp"
#endif
#include "usb_video_source.hpp"
#include "mavlink_video_source.hpp"
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>

namespace stream_manager {

class StreamManager {
public:
    explicit StreamManager(const std::string& config_path = "");
    explicit StreamManager(const StreamManagerConfig& config);
    ~StreamManager();

    // Core interface methods - these abstract the video source complexity
    std::shared_ptr<cv::Mat> getFrame();
    double getFPS() const;
    
    // Buffer access methods (returns actual frames from shared memory)
    std::shared_ptr<cv::Mat> getClosestFrame(const std::chrono::steady_clock::time_point& target_time) const;
    std::shared_ptr<cv::Mat> getFrameByIdx(size_t index) const;
    std::shared_ptr<cv::Mat> getLatestBufferedFrame() const;
    
    // Metadata access methods (zero-copy, only metadata)
    std::shared_ptr<FrameMetadata> getClosestFrameMetadata(const std::chrono::steady_clock::time_point& target_time) const;
    std::shared_ptr<FrameMetadata> getFrameMetadataByIdx(size_t index) const;
    std::shared_ptr<FrameMetadata> getLatestBufferedFrameMetadata() const;
    
    // Direct shared memory access
    std::shared_ptr<cv::Mat> getFrameFromSharedMemory() const;
    std::shared_ptr<cv::Mat> getFrameFromSharedMemory(const FrameMetadata& metadata) const;
    
    // Lifecycle management
    bool initialize();
    void shutdown();
    bool isRunning() const { return running_; }
    bool isInitialized() const { return initialized_; }
    
    // Configuration management
    bool loadConfig(const std::string& config_path);
    bool updateConfig(const StreamManagerConfig& new_config);
    const StreamManagerConfig& getConfig() const;
    
    // Status and statistics
    bool isVideoSourceConnected() const;
    std::string getVideoSourceInfo() const;
    size_t getBufferSize() const;
    double getBufferTargetFPS() const;
    double getAverageSourceFPS() const;
    
    // Buffer management
    void setBufferTargetFPS(double fps);
    void clearBuffer();
    
    // Scaling configuration (read-only during processing)
    const ScalingConfig& getScalingConfig() const;
    
    // Get current memory usage information
    size_t getPoolSlotSize() const;
    size_t getTotalPoolMemoryUsage() const;

private:
    // Core components
    std::unique_ptr<ConfigManager> config_manager_;
    std::unique_ptr<VideoSourceBase> video_source_;
    std::unique_ptr<FrameBuffer> frame_buffer_;
    
    // Threading
    std::thread processing_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> initialized_;
    std::atomic<bool> shutdown_requested_;
    
    // Synchronization
    mutable std::mutex manager_mutex_;
    
    // Processing control
    std::chrono::steady_clock::time_point last_process_time_;
    std::chrono::duration<double> process_interval_;
    
    // Statistics (no overflow-prone counters)
    mutable std::atomic<double> current_processing_fps_;
    mutable std::chrono::steady_clock::time_point last_fps_update_;
    mutable size_t frames_in_current_second_;
    
    // Methods
    void processingThreadFunction();
    bool createVideoSource();
    void destroyVideoSource();
    bool processFrame();
    
    // Configuration helpers
    bool validateConfiguration() const;
    void applyConfiguration();
    
    // Frame processing
    cv::Mat applyScaling(const cv::Mat& input_frame) const;
    bool shouldProcessFrame() const;
    
    // Error handling and logging
    void logError(const std::string& message) const;
    void logInfo(const std::string& message) const;
    void logDebug(const std::string& message) const;
    
    // Cleanup helpers
    void cleanupResources();
    void resetStatistics();
    
    // Factory method for video sources
    std::unique_ptr<VideoSourceBase> createVideoSourceFromConfig(const VideoSourceConfig& config);
};

// Utility functions
namespace utils {
    std::string videoSourceTypeToString(VideoSourceConfig::SourceType type);
    VideoSourceConfig::SourceType stringToVideoSourceType(const std::string& type_str);
    bool isValidFrameSize(int width, int height);
    cv::Size getOptimalFrameSize(const cv::Size& input_size, const cv::Size& target_size);
}

} // namespace stream_manager