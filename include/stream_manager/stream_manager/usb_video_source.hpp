#pragma once

#include "video_source_base.hpp"
#include <opencv2/videoio.hpp>
#include <memory>
#include <mutex>
#include <thread>

namespace stream_manager {

class USBVideoSource : public VideoSourceBase {
public:
    explicit USBVideoSource(const VideoSourceConfig& config);
    ~USBVideoSource() override;

    // Implement pure virtual methods
    std::shared_ptr<cv::Mat> getFrame() override;
    double getFPS() const override;
    
    bool initialize() override;
    void shutdown() override;
    bool isConnected() const override;
    std::string getSourceInfo() const override;
    
    // USB camera specific methods
    bool setProperty(int property_id, double value);
    double getProperty(int property_id) const;
    cv::Size getFrameSize() const;
    bool setFrameSize(int width, int height);

private:
    cv::VideoCapture capture_;
    mutable std::mutex frame_mutex_;
    std::shared_ptr<cv::Mat> latest_frame_;
    
    // Background capture thread
    std::thread capture_thread_;
    std::atomic<bool> capture_running_;
    
    // Camera properties
    double camera_fps_;
    cv::Size frame_size_;
    
    // Connection monitoring
    std::atomic<bool> camera_connected_;
    std::chrono::steady_clock::time_point last_successful_read_;
    
    // Methods
    void captureThreadFunction();
    bool openCamera();
    void closeCamera();
    bool readFrame(cv::Mat& frame);
    void updateCameraProperties();
    bool testCameraConnection();
    
    // Helper methods for camera setup
    void configureCameraProperties();
    bool verifyCameraCapabilities();
};

} // namespace stream_manager