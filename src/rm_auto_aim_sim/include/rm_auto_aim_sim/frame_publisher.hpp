#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

#include "rm_auto_aim_sim/types.hpp"

namespace rm_auto_aim_sim {

struct PublicationMetrics {
  double fps = 0, publish_ms = 0, latency_ms = 0;
  std::uint64_t published = 0, dropped = 0;
};

// One in-flight publish plus one pending latest frame. A slow DDS writer cannot
// block GL capture or create an unbounded queue of old images.
class FramePublisher {
 public:
  using Publish = std::function<void(CameraFrame&&)>;
  explicit FramePublisher(Publish publish);
  ~FramePublisher();
  void Start();
  void Stop();
  bool Submit(CameraFrame& frame);
  void DiscardPending();
  bool Failed() const { return failed_.load(); }
  PublicationMetrics Metrics() const;

 private:
  void Run();
  Publish publish_;
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  std::thread thread_;
  bool stop_ = false, pending_ = false;
  CameraFrame pending_frame_;
  PublicationMetrics metrics_;
  std::atomic_bool failed_{false};
};
}  // namespace rm_auto_aim_sim
