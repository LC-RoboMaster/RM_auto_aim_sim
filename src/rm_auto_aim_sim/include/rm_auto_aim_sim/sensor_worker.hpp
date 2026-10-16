#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <thread>

#include "rm_auto_aim_sim/engine.hpp"
#include "rm_auto_aim_sim/frame_publisher.hpp"

struct GLFWwindow;

namespace rm_auto_aim_sim {

// The hidden window is created/destroyed on the main thread; only its GL context
// moves to the worker. A model copy isolates rendering from viewer model edits.
class SensorWorker {
 public:
  using Snapshot = std::function<StateFrame(mjModel*, mjData*)>;
  using Publish = std::function<void(CameraFrame&&)>;
  SensorWorker(const Engine& engine, Snapshot snapshot, Publish publish);
  ~SensorWorker();
  void Start();
  void Stop();
  bool Failed() const { return failed_.load() || publisher_.Failed(); }
  void SetRendering(bool shader, bool shadows, double exposure, double bloom,
                    double lamp_intensity);
  void SetProjectileOpacity(double opacity);
  CameraMetrics Metrics() const;

 private:
  void Run();
  Config config_;
  int camera_id_;
  Snapshot snapshot_;
  FramePublisher publisher_;
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  GLFWwindow* window_ = nullptr;
  std::thread thread_;
  std::atomic_bool stop_{false}, failed_{false};
  std::atomic_bool shader_{true}, shadows_{false};
  std::atomic<double> projectile_opacity_{1};
  std::atomic<double> exposure_{1}, bloom_{.55}, lamp_intensity_{4};
  mutable std::mutex metrics_mutex_;
  CameraMetrics metrics_;
};

}  // namespace rm_auto_aim_sim
