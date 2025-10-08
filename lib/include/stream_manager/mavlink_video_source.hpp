#pragma once

#include "video_source_base.hpp"
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace stream_manager {

// Forward declarations for MAVLink structures
struct MAVLinkConnection;
struct VideoStreamInfo;

class MAVLinkVideoSource : public VideoSourceBase {
public:
  explicit MAVLinkVideoSource(const VideoSourceConfig &config,
                              rclcpp::Node::SharedPtr node);
  ~MAVLinkVideoSource() override;

  // Implement pure virtual methods
  std::shared_ptr<cv::Mat> getFrame() override;
  double getFPS() const override;

  bool initialize() override;
  void shutdown() override;
  bool isConnected() const override;
  std::string getSourceInfo() const override;

  // MAVLink specific methods
  bool requestVideoStream();
  bool stopVideoStream();
  // std::vector<VideoStreamInfo> getAvailableStreams() const;
  bool selectStream(int stream_id);

private:
  // MAVLink connection
  std::unique_ptr<MAVLinkConnection> mavlink_connection_;
  std::string connection_string_;

  // Video stream data
  mutable std::mutex frame_mutex_;
  std::shared_ptr<cv::Mat> latest_frame_;

  // Stream management
  std::atomic<bool> streaming_;
  std::atomic<bool> stream_requested_;
  int current_stream_id_;
  rclcpp::TimerBase::SharedPtr stream_timer_;
  void onStreamTimer();

  // Connection status
  std::atomic<bool> mavlink_connected_;
  std::chrono::steady_clock::time_point last_heartbeat_;
  std::chrono::steady_clock::time_point last_frame_received_;

  // Stream information
  struct StreamInfo {
    int stream_id;
    std::string name;
    std::string uri;
    int width;
    int height;
    double framerate;
    bool active;
  };

  mutable std::mutex streams_mutex_;
  std::vector<StreamInfo> available_streams_;

  // Methods
  bool initializeMAVLink();
  void shutdownMAVLink();
  bool connectToMAVLink();
  void handleMAVLinkMessages();
  bool processVideoFrame(const std::vector<uint8_t> &frame_data);

  // MAVLink message handlers
  void handleHeartbeat();
  void handleVideoStreamInformation(const std::vector<uint8_t> &payload);
  void handleVideoStreamStatus(const std::vector<uint8_t> &payload);

  // Stream decoding (placeholder for actual implementation)
  cv::Mat decodeVideoFrame(const std::vector<uint8_t> &encoded_data);

  // Helper methods
  bool parseConnectionString();
  void updateConnectionStatus();
  void resetStreamState();

  // Constants
  static constexpr std::chrono::seconds HEARTBEAT_TIMEOUT{10};
  static constexpr std::chrono::seconds FRAME_TIMEOUT{5};
  static constexpr int DEFAULT_STREAM_ID = 0;
};

// Helper structures for MAVLink video streaming
struct VideoStreamInfo {
  int stream_id;
  std::string name;
  std::string uri;
  int width;
  int height;
  double framerate;
  bool active;

  VideoStreamInfo()
      : stream_id(-1), width(0), height(0), framerate(0.0), active(false) {}
};

struct MAVLinkConnection {
  // This would contain actual MAVLink connection details
  // Implementation depends on the specific MAVLink library used
  std::string address;
  int port;
  bool connected;

  MAVLinkConnection() : port(0), connected(false) {}
};

} // namespace stream_manager