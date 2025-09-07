#include "stream_manager/video_source_base.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>

namespace stream_manager {

VideoSourceBase::VideoSourceBase(const VideoSourceConfig& config)
    : config_(config)
    , initialized_(false)
    , running_(false)
    , calculated_fps_(0.0) {
    
    resetStatistics();
}

VideoSourceBase::~VideoSourceBase() {
    if (running_) {
        shutdown();
    }
}

cv::Mat VideoSourceBase::scaleFrame(const cv::Mat& input_frame, const ScalingConfig& scaling_config) const {
    if (!scaling_config.enabled || input_frame.empty()) {
        return input_frame;
    }
    
    if (!needsScaling(input_frame, scaling_config)) {
        return input_frame;
    }
    
    cv::Mat scaled_frame;
    cv::Size target_size(scaling_config.target_width, scaling_config.target_height);
    
    try {
        cv::resize(input_frame, scaled_frame, target_size, 0, 0, scaling_config.interpolation_method);
        return scaled_frame;
    } catch (const cv::Exception& e) {
        // If scaling fails, return original frame
        return input_frame;
    }
}

bool VideoSourceBase::needsScaling(const cv::Mat& frame, const ScalingConfig& scaling_config) const {
    if (!scaling_config.enabled || frame.empty()) {
        return false;
    }
    
    return (frame.cols != scaling_config.target_width || 
            frame.rows != scaling_config.target_height);
}

double VideoSourceBase::getAverageFPS() const {
    std::lock_guard<std::mutex> lock(fps_mutex_);
    return calculated_fps_;
}

void VideoSourceBase::updateFPSCalculation() const {
    std::lock_guard<std::mutex> lock(fps_mutex_);
    
    auto now = std::chrono::steady_clock::now();
    frame_timestamps_.push_back(now);
    
    // Keep only recent timestamps for FPS calculation (bounded size)
    while (frame_timestamps_.size() > FPS_CALCULATION_WINDOW) {
        frame_timestamps_.pop_front();
    }
    
    // Calculate FPS based on recent frames
    if (frame_timestamps_.size() >= 2) {
        auto time_span = frame_timestamps_.back() - frame_timestamps_.front();
        auto duration_seconds = std::chrono::duration<double>(time_span).count();
        
        if (duration_seconds > 0.0) {
            calculated_fps_ = static_cast<double>(frame_timestamps_.size() - 1) / duration_seconds;
        }
    }
    
    // Update last frame time
    last_frame_time_ = now;
}

void VideoSourceBase::resetStatistics() {
    std::lock_guard<std::mutex> lock(fps_mutex_);
    
    calculated_fps_ = 0.0;
    frame_timestamps_.clear();
    
    auto now = std::chrono::steady_clock::now();
    last_frame_time_ = now;
    timeout_start_ = now;
}

bool VideoSourceBase::isTimedOut() const {
    auto now = std::chrono::steady_clock::now();
    
    // Check if we have received any frames recently
    if (frame_timestamps_.empty()) {
        // Check initialization timeout
        auto elapsed = std::chrono::duration<double>(now - timeout_start_).count();
        return elapsed > config_.timeout_seconds;
    } else {
        // Check frame reception timeout
        auto elapsed = std::chrono::duration<double>(now - last_frame_time_).count();
        return elapsed > config_.timeout_seconds;
    }
}

} // namespace stream_manager