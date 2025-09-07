#include "stream_manager/stream_manager.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>

using namespace stream_manager;

int main() {
    std::cout << "StreamManager Basic Usage Example" << std::endl;
    
    // Create StreamManager with default configuration
    StreamManager manager("config/default_config.yaml");
    
    // Initialize the manager
    if (!manager.initialize()) {
        std::cerr << "Failed to initialize StreamManager" << std::endl;
        return -1;
    }
    
    std::cout << "StreamManager initialized successfully" << std::endl;
    std::cout << "Video source info:\n" << manager.getVideoSourceInfo() << std::endl;
    
    // Wait for video source to connect
    std::cout << "Waiting for video source connection..." << std::endl;
    int connection_attempts = 0;
    while (!manager.isVideoSourceConnected() && connection_attempts < 50) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        connection_attempts++;
    }
    
    if (!manager.isVideoSourceConnected()) {
        std::cerr << "Failed to connect to video source" << std::endl;
        return -1;
    }
    
    std::cout << "Video source connected!" << std::endl;
    
    // Create OpenCV window for display
    cv::namedWindow("StreamManager Output", cv::WINDOW_AUTOSIZE);
    
    auto start_time = std::chrono::steady_clock::now();
    int frame_count = 0;
    
    std::cout << "Starting frame capture loop (press 'q' to quit)..." << std::endl;
    
    while (true) {
        // Method 1: Get latest frame directly from video source
        auto frame = manager.getFrame();
        
        if (frame && !frame->empty()) {
            frame_count++;
            
            // Add some overlay information
            cv::Mat display_frame = frame->clone();
            
            // Display FPS and frame count
            std::string fps_text = "FPS: " + std::to_string(manager.getFPS());
            std::string frame_text = "Frame: " + std::to_string(frame_count);
            std::string buffer_text = "Buffer: " + std::to_string(manager.getBufferSize()) + "/" + 
                                    std::to_string(manager.getConfig().buffer.max_size);
            
            cv::putText(display_frame, fps_text, cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            cv::putText(display_frame, frame_text, cv::Point(10, 60), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            cv::putText(display_frame, buffer_text, cv::Point(10, 90), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            
            cv::imshow("StreamManager Output", display_frame);
            
            // Print statistics every 30 frames
            if (frame_count % 30 == 0) {
                auto current_time = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration<double>(current_time - start_time).count();
                double avg_fps = frame_count / elapsed;
                
                std::cout << "Statistics after " << frame_count << " frames:" << std::endl;
                std::cout << "  Average FPS: " << avg_fps << std::endl;
                std::cout << "  Source FPS: " << manager.getAverageSourceFPS() << std::endl;
                std::cout << "  Buffer size: " << manager.getBufferSize() << std::endl;
                std::cout << "  Buffer target FPS: " << manager.getBufferTargetFPS() << std::endl;
            }
        }
        
        // Method 2: Demonstrate zero-copy metadata access
        if (frame_count % 60 == 0 && frame_count > 0) {
            std::cout << "\nDemonstrating zero-copy metadata access:" << std::endl;
            
            // Get latest frame metadata (zero-copy)
            auto latest_metadata = manager.getLatestBufferedFrameMetadata();
            if (latest_metadata) {
                std::cout << "  Latest frame metadata - ID: " << latest_metadata->frame_id 
                         << ", Size: " << latest_metadata->width << "x" << latest_metadata->height
                         << ", FPS: " << latest_metadata->fps << std::endl;
            }
            
            // Get frame metadata by index (zero-copy)
            auto metadata_by_idx = manager.getFrameMetadataByIdx(0);
            if (metadata_by_idx) {
                std::cout << "  Oldest frame metadata - ID: " << metadata_by_idx->frame_id << std::endl;
            }
            
            // Get closest frame metadata to 1 second ago (zero-copy)
            auto target_time = std::chrono::steady_clock::now() - std::chrono::seconds(1);
            auto closest_metadata = manager.getClosestFrameMetadata(target_time);
            if (closest_metadata) {
                auto time_diff = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - closest_metadata->timestamp).count();
                std::cout << "  Closest metadata to 1s ago: ID " << closest_metadata->frame_id 
                         << " (actual diff: " << time_diff << "s)" << std::endl;
                
                // Now get the actual frame from shared memory using metadata
                auto frame_from_metadata = manager.getFrameFromSharedMemory(*closest_metadata);
                if (frame_from_metadata) {
                    std::cout << "  Successfully reconstructed frame from metadata: " 
                             << frame_from_metadata->cols << "x" << frame_from_metadata->rows << std::endl;
                }
            }
        }
        
        // Method 3: Demonstrate shared memory access (every 120 frames)
        if (frame_count % 120 == 0 && frame_count > 0) {
            std::cout << "\nTesting shared memory access:" << std::endl;
            auto shared_frame = manager.getFrameFromSharedMemory();
            if (shared_frame) {
                std::cout << "  Successfully retrieved frame from shared memory: " 
                         << shared_frame->cols << "x" << shared_frame->rows << std::endl;
            } else {
                std::cout << "  No frame available in shared memory" << std::endl;
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
            auto config = manager.getConfig();
            config.scaling.enabled = !config.scaling.enabled;
            if (manager.updateConfig(config)) {
                std::cout << "Scaling " << (config.scaling.enabled ? "enabled" : "disabled") << std::endl;
            }
        }
        
        if (key == '+') {
            // Increase buffer target FPS
            double current_fps = manager.getBufferTargetFPS();
            manager.setBufferTargetFPS(current_fps + 1.0);
            std::cout << "Buffer target FPS increased to: " << manager.getBufferTargetFPS() << std::endl;
        }
        
        if (key == '-') {
            // Decrease buffer target FPS
            double current_fps = manager.getBufferTargetFPS();
            if (current_fps > 1.0) {
                manager.setBufferTargetFPS(current_fps - 1.0);
                std::cout << "Buffer target FPS decreased to: " << manager.getBufferTargetFPS() << std::endl;
            }
        }
        
        if (key == 'c') {
            // Clear buffer
            manager.clearBuffer();
            std::cout << "Buffer cleared" << std::endl;
        }
        
        // Small delay to prevent excessive CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    cv::destroyAllWindows();
    
    // Final statistics
    auto end_time = std::chrono::steady_clock::now();
    auto total_elapsed = std::chrono::duration<double>(end_time - start_time).count();
    double final_avg_fps = frame_count / total_elapsed;
    
    std::cout << "\nFinal Statistics:" << std::endl;
    std::cout << "  Total frames processed: " << frame_count << std::endl;
    std::cout << "  Total time: " << total_elapsed << " seconds" << std::endl;
    std::cout << "  Average FPS: " << final_avg_fps << std::endl;
    std::cout << "  Final buffer size: " << manager.getBufferSize() << std::endl;
    
    // Shutdown manager
    manager.shutdown();
    std::cout << "StreamManager shutdown complete" << std::endl;
    
    return 0;
}