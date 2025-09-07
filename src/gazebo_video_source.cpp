#include "stream_manager_ros2/gazebo_video_source.hpp"
#include <iostream>

namespace stream_manager {

GazeboVideoSource::GazeboVideoSource(const VideoSourceConfig& config)
    : VideoSourceBase(config)
    , receiving_messages_(false) {
    
    last_connection_check_ = std::chrono::steady_clock::now();
}

GazeboVideoSource::~GazeboVideoSource() {
    shutdown();
}

std::shared_ptr<cv::Mat> GazeboVideoSource::getFrame() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    
    if (!latest_frame_ || latest_frame_->empty()) {
        return nullptr;
    }
    
    // Return a copy to avoid shared access issues
    return std::make_shared<cv::Mat>(latest_frame_->clone());
}

double GazeboVideoSource::getFPS() const {
    return getAverageFPS();
}

bool GazeboVideoSource::initialize() {
    if (initialized_) {
        return true;
    }
    
    if (!initializeROS2()) {
        return false;
    }
    
    initialized_ = true;
    running_ = true;
    
    // Start executor thread
    executor_thread_ = std::thread(&GazeboVideoSource::executorThreadFunction, this);
    
    return true;
}

void GazeboVideoSource::shutdown() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    
    shutdownROS2();
    
    if (executor_thread_.joinable()) {
        executor_thread_.join();
    }
    
    initialized_ = false;
}

bool GazeboVideoSource::isConnected() const {
    checkConnectionStatus();
    return receiving_messages_ && running_;
}

std::string GazeboVideoSource::getSourceInfo() const {
    std::stringstream info;
    info << "Gazebo ROS2 Video Source\n";
    info << "Topic: " << config_.topic_name << "\n";
    info << "Connected: " << (isConnected() ? "Yes" : "No") << "\n";
    info << "Average FPS: " << getAverageFPS() << "\n";
    
    if (latest_frame_ && !latest_frame_->empty()) {
        info << "Frame size: " << latest_frame_->cols << "x" << latest_frame_->rows << "\n";
        info << "Channels: " << latest_frame_->channels();
    }
    
    return info.str();
}

void GazeboVideoSource::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        // Convert ROS image to OpenCV format
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        if (!cv_ptr || cv_ptr->image.empty()) {
            return;
        }
        
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_frame_ = std::make_shared<cv::Mat>(cv_ptr->image.clone());
            last_message_time_ = std::chrono::steady_clock::now();
        }
        
        receiving_messages_ = true;
        updateFPSCalculation();
        
    } catch (const cv_bridge::Exception& e) {
        std::cerr << "cv_bridge exception: " << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error in image callback: " << e.what() << std::endl;
    }
}

void GazeboVideoSource::executorThreadFunction() {
    while (running_ && executor_) {
        try {
            executor_->spin_some(std::chrono::milliseconds(10));
        } catch (const std::exception& e) {
            std::cerr << "Error in executor thread: " << e.what() << std::endl;
            break;
        }
    }
}

bool GazeboVideoSource::initializeROS2() {
    try {
        // Initialize ROS2 if not already done
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        
        // Create node
        node_ = rclcpp::Node::make_shared("gazebo_video_source_" + std::to_string(std::time(nullptr)));
        
        // Create image subscription
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        
        image_subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
            config_.topic_name,
            qos,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                this->imageCallback(msg);
            }
        );
        
        // Create executor
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize ROS2: " << e.what() << std::endl;
        return false;
    }
}

void GazeboVideoSource::shutdownROS2() {
    if (executor_) {
        executor_->cancel();
    }
    
    image_subscription_.reset();
    
    if (node_) {
        executor_->remove_node(node_);
        node_.reset();
    }
    
    executor_.reset();
}

void GazeboVideoSource::checkConnectionStatus() const {
    auto now = std::chrono::steady_clock::now();
    
    // Check connection periodically
    if (now - last_connection_check_ > std::chrono::seconds(1)) {
        const_cast<GazeboVideoSource*>(this)->last_connection_check_ = now;
        
        if (receiving_messages_) {
            // Check if we've received messages recently
            auto time_since_last_message = now - last_message_time_;
            if (time_since_last_message > CONNECTION_TIMEOUT) {
                const_cast<GazeboVideoSource*>(this)->receiving_messages_ = false;
            }
        }
    }
}

} // namespace stream_manager