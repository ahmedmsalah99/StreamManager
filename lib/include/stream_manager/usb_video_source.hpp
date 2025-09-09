#pragma once

#include "video_source_base.hpp"
#include <opencv2/videoio.hpp>
#include <memory>
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>

namespace stream_manager {

class USBVideoSource : public VideoSourceBase {
public:
    explicit USBVideoSource(const VideoSourceConfig& config, rclcpp::Node::SharedPtr node);
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
    
    // ROS capture timer
    rclcpp::TimerBase::SharedPtr capture_timer_;
    void onCaptureTimer();
    
    // Camera properties
    double camera_fps_;
    cv::Size frame_size_;
    
    // Connection monitoring
    std::atomic<bool> camera_connected_;
    std::chrono::steady_clock::time_point last_successful_read_;
    
    // Methods
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