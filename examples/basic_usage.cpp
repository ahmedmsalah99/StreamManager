#include "stream_manager/stream_manager.hpp"
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

using namespace stream_manager;

int main(int argc, char *argv[]) {
  std::cout << "StreamManager ROS2 Node Example" << std::endl;

  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create StreamManager as a ROS2 node
  auto manager = std::make_shared<StreamManager>("config/default_config.yaml",
                                                 "stream_manager_node");

  // Initialize the manager
  if (!manager->initialize()) {
    std::cerr << "Failed to initialize StreamManager" << std::endl;
    rclcpp::shutdown();
    return -1;
  }

  std::cout << "StreamManager initialized successfully" << std::endl;
  std::cout << "Video source info:\n"
            << manager->getVideoSourceInfo() << std::endl;

  // Wait for video source to connect
  std::cout << "Waiting for video source connection..." << std::endl;
  int connection_attempts = 0;
  while (!manager->isVideoSourceConnected() && connection_attempts < 50) {
    rclcpp::spin_some(manager);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    connection_attempts++;
  }

  if (!manager->isVideoSourceConnected()) {
    std::cerr << "Failed to connect to video source" << std::endl;
    rclcpp::shutdown();
    return -1;
  }

  std::cout << "Video source connected!" << std::endl;
  std::cout << "Publishing topics:" << std::endl;
  std::cout << "  Current frame: "
            << manager->getConfig().publishing.current_frame_topic << std::endl;
  std::cout << "  Delayed frame: "
            << manager->getConfig().publishing.delayed_frame_topic << std::endl;

  // Create OpenCV window for display
  cv::namedWindow("StreamManager Output", cv::WINDOW_AUTOSIZE);

  auto start_time = std::chrono::steady_clock::now();
  int frame_count = 0;

  std::cout << "Starting frame capture loop (press 'q' to quit)..."
            << std::endl;

  while (rclcpp::ok()) {
    // Spin ROS2 to handle callbacks and publishing
    rclcpp::spin_some(manager);

    // Method 1: Get latest frame directly from video source
    auto frame = manager->getFrame();

    if (frame && !frame->empty()) {
      frame_count++;

      // Add some overlay information
      cv::Mat display_frame = frame->clone();

      // Display FPS and frame count
      std::string fps_text = "FPS: " + std::to_string(manager->getFPS());
      std::string frame_text = "Frame: " + std::to_string(frame_count);

      cv::putText(display_frame, fps_text, cv::Point(10, 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
      cv::putText(display_frame, frame_text, cv::Point(10, 60),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

      cv::imshow("StreamManager Output", display_frame);

      // Print statistics every 30 frames
      if (frame_count % 30 == 0) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed =
            std::chrono::duration<double>(current_time - start_time).count();
        double avg_fps = frame_count / elapsed;

        std::cout << "Statistics after " << frame_count
                  << " frames:" << std::endl;
        std::cout << "  Average FPS: " << avg_fps << std::endl;
        std::cout << "  Source FPS: " << manager->getAverageSourceFPS()
                  << std::endl;
      }
    }
  }
  // Check for quit key
  char key = cv::waitKey(1) & 0xFF;
  if (key == 'q' || key == 27) { // 'q' or ESC
    break;
  }

  // Demonstrate dynamic configuration changes
  if (key == 's') {
    // Toggle scaling
    auto config = manager->getConfig();
    config.scaling.enabled = !config.scaling.enabled;
    if (manager->updateConfig(config)) {
      std::cout << "Scaling "
                << (config.scaling.enabled ? "enabled" : "disabled")
                << std::endl;
    }
  }

  // Small delay to prevent excessive CPU usage
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

cv::destroyAllWindows();

// Final statistics
auto end_time = std::chrono::steady_clock::now();
auto total_elapsed =
    std::chrono::duration<double>(end_time - start_time).count();
double final_avg_fps = frame_count / total_elapsed;

std::cout << "\nFinal Statistics:" << std::endl;
std::cout << "  Total frames processed: " << frame_count << std::endl;
std::cout << "  Total time: " << total_elapsed << " seconds" << std::endl;
std::cout << "  Average FPS: " << final_avg_fps << std::endl;

// Shutdown manager
manager->shutdown();
std::cout << "StreamManager shutdown complete" << std::endl;

// Shutdown ROS2
rclcpp::shutdown();

return 0;
}