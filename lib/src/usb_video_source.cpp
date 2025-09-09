#include "stream_manager/usb_video_source.hpp"
#include <iostream>

namespace stream_manager {

USBVideoSource::USBVideoSource(const VideoSourceConfig& config, rclcpp::Node::SharedPtr node)
    : VideoSourceBase(config,node)
    , camera_fps_(30.0)
    , frame_size_(640, 480)
    , camera_connected_(false)
     {
    std::cout << "Statistics reset in usb video" << std::endl;
}

USBVideoSource::~USBVideoSource() {
    shutdown();
}

std::shared_ptr<cv::Mat> USBVideoSource::getFrame() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    
    if (!latest_frame_ || latest_frame_->empty()) {
        return nullptr;
    }
    
    // Return a copy to avoid shared access issues
    return std::make_shared<cv::Mat>(latest_frame_->clone());
}

double USBVideoSource::getFPS() const {
    return getAverageFPS();
}

bool USBVideoSource::initialize() {
    if (initialized_) {
        return true;
    }
    
    if (!openCamera()) {
        return false;
    }
    
    updateCameraProperties();
    
    if (!verifyCameraCapabilities()) {
        closeCamera();
        return false;
    }
    
    initialized_ = true;
    
    // Use ROS timer
    capture_timer_ = rclcpp::create_timer(node_.get(), node_->get_clock(), std::chrono::milliseconds(20), std::bind(&USBVideoSource::onCaptureTimer, this));
    
    return true;
}

void USBVideoSource::shutdown() {
    if (capture_timer_) {
        capture_timer_.reset();
    }
    closeCamera();
    initialized_ = false;
}

bool USBVideoSource::isConnected() const {
    return camera_connected_ && capture_.isOpened();
}

std::string USBVideoSource::getSourceInfo() const {
    std::stringstream info;
    info << "USB Camera Video Source\n";
    info << "Device ID: " << config_.device_id << "\n";
    info << "Connected: " << (isConnected() ? "Yes" : "No") << "\n";
    info << "Frame size: " << frame_size_.width << "x" << frame_size_.height << "\n";
    info << "Camera FPS: " << camera_fps_ << "\n";
    info << "Average FPS: " << getAverageFPS() << "\n";
    
    return info.str();
}

bool USBVideoSource::setProperty(int property_id, double value) {
    if (!capture_.isOpened()) {
        return false;
    }
    
    return capture_.set(property_id, value);
}

double USBVideoSource::getProperty(int property_id) const {
    if (!capture_.isOpened()) {
        return -1.0;
    }
    
    return capture_.get(property_id);
}

cv::Size USBVideoSource::getFrameSize() const {
    return frame_size_;
}

bool USBVideoSource::setFrameSize(int width, int height) {
    if (!capture_.isOpened()) {
        return false;
    }
    
    bool success = true;
    success &= capture_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    success &= capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    
    if (success) {
        frame_size_ = cv::Size(width, height);
        updateCameraProperties();
    }
    
    return success;
}



void USBVideoSource::onCaptureTimer() {

    cv::Mat frame;
    if (!readFrame(frame)) {
        if (!testCameraConnection()) {
            camera_connected_ = false;
            return;
        }
    } else {
        camera_connected_ = true;
        last_successful_read_ = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_frame_ = std::make_shared<cv::Mat>(frame.clone());
        }
        updateFPSCalculation();
    }
}

bool USBVideoSource::openCamera() {
    try {
        capture_.open(config_.device_id);
        
        if (!capture_.isOpened()) {
            std::cerr << "Failed to open camera with device ID: " << config_.device_id << std::endl;
            return false;
        }
        
        configureCameraProperties();
        camera_connected_ = true;
        
        return true;
        
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV exception while opening camera: " << e.what() << std::endl;
        return false;
    }
}

void USBVideoSource::closeCamera() {
    if (capture_.isOpened()) {
        capture_.release();
    }
    camera_connected_ = false;
}

bool USBVideoSource::readFrame(cv::Mat& frame) {
    if (!capture_.isOpened()) {
        return false;
    }
    
    try {
        bool success = capture_.read(frame);
        return success && !frame.empty();
    } catch (const cv::Exception& e) {
        std::cerr << "Error reading frame: " << e.what() << std::endl;
        return false;
    }
}

void USBVideoSource::updateCameraProperties() {
    if (!capture_.isOpened()) {
        return;
    }
    
    camera_fps_ = capture_.get(cv::CAP_PROP_FPS);
    
    int width = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH));
    int height = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT));
    frame_size_ = cv::Size(width, height);
    
    if (camera_fps_ <= 0) {
        camera_fps_ = 30.0; // Default fallback
    }
}

bool USBVideoSource::testCameraConnection() {
    if (!capture_.isOpened()) {
        return false;
    }
    
    cv::Mat test_frame;
    return capture_.read(test_frame) && !test_frame.empty();
}

void USBVideoSource::configureCameraProperties() {
    if (!capture_.isOpened()) {
        return;
    }
    
    // Set buffer size to reduce latency
    capture_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    
    // Try to set reasonable defaults
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    capture_.set(cv::CAP_PROP_FPS, 30);
    
    // Enable auto exposure and auto white balance if available
    capture_.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.75);
    capture_.set(cv::CAP_PROP_AUTO_WB, 1);
}

bool USBVideoSource::verifyCameraCapabilities() {
    if (!capture_.isOpened()) {
        return false;
    }
    
    // Test if we can read at least one frame
    cv::Mat test_frame;
    if (!capture_.read(test_frame) || test_frame.empty()) {
        std::cerr << "Camera verification failed: cannot read frames" << std::endl;
        return false;
    }
    
    // Verify frame dimensions are reasonable
    if (test_frame.cols < 32 || test_frame.rows < 32) {
        std::cerr << "Camera verification failed: frame size too small" << std::endl;
        return false;
    }
    
    return true;
}

} // namespace stream_manager