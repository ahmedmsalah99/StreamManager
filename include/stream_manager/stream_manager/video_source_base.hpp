#pragma once

#include <opencv2/opencv.hpp>
#include <memory>
#include <atomic>
#include <thread>
#include <chrono>
#include "config_manager.hpp"

namespace stream_manager {

class VideoSourceBase {
public:
    explicit VideoSourceBase(const VideoSourceConfig& config);
    virtual ~VideoSourceBase();

    // Pure virtual methods that must be implemented by derived classes
    virtual std::shared_ptr<cv::Mat> getFrame() = 0;
    virtual double getFPS() const = 0;
    
    // Common interface methods
    virtual bool initialize() = 0;
    virtual void shutdown(){};
    virtual bool isConnected() const = 0;
    virtual std::string getSourceInfo() const = 0;
    
    // Configuration and status
    const VideoSourceConfig& getConfig() const { return config_; }
    bool isInitialized() const { return initialized_; }
    
    // Frame scaling functionality
    cv::Mat scaleFrame(const cv::Mat& input_frame, const ScalingConfig& scaling_config) const;
    bool needsScaling(const cv::Mat& frame, const ScalingConfig& scaling_config) const;
    
    // Statistics
    virtual std::chrono::steady_clock::time_point getLastFrameTime() const { return last_frame_time_; }
    virtual double getAverageFPS() const;

protected:
    VideoSourceConfig config_;
    std::atomic<bool> initialized_;
    std::atomic<bool> running_;
    
    // Frame statistics (no overflow-prone counters)
    mutable std::chrono::steady_clock::time_point last_frame_time_;
    
    // FPS calculation using sliding window (bounded size)
    mutable std::mutex fps_mutex_;
    mutable std::deque<std::chrono::steady_clock::time_point> frame_timestamps_;
    mutable double calculated_fps_;
    static constexpr size_t FPS_CALCULATION_WINDOW = 30;
    
    // Helper methods
    void updateFPSCalculation() const;
    void resetStatistics();
    
    // Timeout handling
    bool isTimedOut() const;
    std::chrono::steady_clock::time_point timeout_start_;
};

} // namespace stream_manager