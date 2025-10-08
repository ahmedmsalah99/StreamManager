#pragma once

#include "config_manager.hpp"
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <vector>

namespace stream_manager {

// In-process frame container
struct SharedFrameHeader {
  uint64_t timestamp_ns;
  uint32_t width;
  uint32_t height;
  uint32_t channels;
  uint32_t type;
  uint64_t frame_id;
  double fps;
  uint32_t data_size;
};
struct FrameMetadata {
  uint64_t timestamp_ns_system; // system_clock nanoseconds for ROS stamps
  double fps;
  uint32_t width;
  uint32_t height;
  uint32_t channels;
  uint32_t cv_type;
  std::shared_ptr<std::vector<uint8_t>> owned_data; // CPU-owned copy
  size_t data_size;

  FrameMetadata()
      : timestamp_ns_system(0), fps(0.0), width(0), height(0), channels(0),
        cv_type(0), data_size(0) {}
  FrameMetadata(uint64_t ts_ns_system, double current_fps, uint32_t w,
                uint32_t h, uint32_t ch, uint32_t type,
                std::shared_ptr<std::vector<uint8_t>> data, size_t size)
      : timestamp_ns_system(ts_ns_system), fps(current_fps), width(w),
        height(h), channels(ch), cv_type(type), owned_data(std::move(data)),
        data_size(size) {}
};
class FrameBuffer {
public:
  explicit FrameBuffer(const BufferConfig &config);
  ~FrameBuffer();

  // Core buffer operations - all through shared memory
  bool addFrame(const cv::Mat &frame, double current_fps);
  bool addFrame(const cv::Mat &frame, double current_fps, uint64_t ros_time_ns);
  std::shared_ptr<cv::Mat> getLatestFrame() const;
  std::shared_ptr<cv::Mat> getClosestFrame(
      const std::chrono::steady_clock::time_point &target_time) const;
  std::shared_ptr<cv::Mat> getFrameByIdx(size_t index) const;

  // Metadata access (zero-copy)
  std::shared_ptr<FrameMetadata> getLatestFrameMetadata() const;
  std::shared_ptr<FrameMetadata> getClosestFrameMetadata(
      const std::chrono::steady_clock::time_point &target_time) const;
  // New: search by system-clock nanoseconds to avoid std::chrono in higher
  // layers
  std::shared_ptr<FrameMetadata>
  getClosestFrameMetadataBySystemNs(uint64_t target_time_ns) const;
  std::shared_ptr<FrameMetadata> getFrameMetadataByIdx(size_t index) const;

  // Direct frame access from owned data
  std::shared_ptr<cv::Mat>
  getFrameFromOwnedData(const FrameMetadata &metadata) const;

  // Buffer management
  void setTargetFPS(double fps);
  double getTargetFPS() const { return target_fps_; }
  size_t getBufferSize() const;
  size_t getMaxBufferSize() const { return max_buffer_size_; }
  void clearBuffer();
  std::shared_ptr<FrameMetadata> popOldestIfFull();

  // Statistics
  double getAverageFPS() const;
  std::chrono::steady_clock::time_point getOldestFrameTime() const;
  std::chrono::steady_clock::time_point getNewestFrameTime() const;

private:
  mutable std::mutex buffer_mutex_;
  std::deque<std::shared_ptr<FrameMetadata>>
      metadata_buffer_; // Only metadata, no frame data

  size_t max_buffer_size_;
  double target_fps_;
  std::chrono::steady_clock::time_point last_add_time_;
  std::chrono::duration<double> frame_interval_;

  // Helper methods
  void removeOldFrames();
  bool shouldAddFrame() const;
  size_t calculateFrameDataSize(const cv::Mat &frame) const;

  // Frame operations using CPU memory
  bool storeFrameInOwnedBuffer(const cv::Mat &frame, double current_fps);
  bool storeFrameInOwnedBuffer(const cv::Mat &frame, double current_fps,
                               uint64_t ts_ns_system);
  cv::Mat reconstructFrameFromOwnedData(const FrameMetadata &metadata) const;
};

} // namespace stream_manager