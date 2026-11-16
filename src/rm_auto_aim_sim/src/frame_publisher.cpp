#include "rm_auto_aim_sim/frame_publisher.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace rm_auto_aim_sim {
FramePublisher::FramePublisher(Publish publish) : publish_(std::move(publish)) {}
FramePublisher::~FramePublisher() { Stop(); }
void FramePublisher::Start() { thread_ = std::thread(&FramePublisher::Run, this); }
void FramePublisher::Stop() {
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  ready_.notify_one();
  if (thread_.joinable()) thread_.join();
}
bool FramePublisher::Submit(CameraFrame& frame) {
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (stop_ || failed_.load()) return false;
    if (pending_) ++metrics_.dropped;
    std::swap(frame, pending_frame_);
    pending_ = true;
  }
  ready_.notify_one();
  return true;
}
PublicationMetrics FramePublisher::Metrics() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return metrics_;
}
void FramePublisher::DiscardPending() {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (pending_) ++metrics_.dropped;
  pending_ = false;
}
void FramePublisher::Run() {
  using Clock = std::chrono::steady_clock;
  CameraFrame frame;
  auto start = Clock::now();
  auto deadline = start + std::chrono::seconds(1);
  std::size_t count = 0;
  try {
    std::unique_lock<std::mutex> lock(mutex_);
    for (;;) {
      ready_.wait_until(lock, deadline, [&]() { return stop_ || pending_; });
      const auto now = Clock::now();
      if (now >= deadline) {
        metrics_.fps = count / std::chrono::duration<double>(now - start).count();
        count = 0;
        start = now;
        deadline = now + std::chrono::seconds(1);
      }
      if (!pending_) {
        if (stop_) return;
        continue;
      }
      std::swap(frame, pending_frame_);
      pending_ = false;
      frame.wall_fps = metrics_.fps;
      const double capture_wall_time = frame.capture_wall_time;
      lock.unlock();
      const auto begin = Clock::now();
      publish_(std::move(frame));
      const auto end = Clock::now();
      lock.lock();
      metrics_.publish_ms = std::chrono::duration<double, std::milli>(end - begin).count();
      metrics_.latency_ms =
          (std::chrono::duration<double>(end.time_since_epoch()).count() - capture_wall_time) *
          1000.0;
      ++metrics_.published;
      ++count;
    }
  } catch (const std::exception& error) {
    std::cerr << "[camera publisher] " << error.what() << '\n';
    failed_.store(true);
  }
}
}  // namespace rm_auto_aim_sim
