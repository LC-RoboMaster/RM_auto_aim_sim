#pragma once

#include <atomic>
#include <functional>
#include <thread>

#include "rm_auto_aim_sim/engine.hpp"

struct GLFWwindow;

namespace rm_auto_aim_sim {

// The hidden window is created/destroyed on the main thread; only its GL context
// moves to the worker. A model copy isolates rendering from viewer model edits.
class SensorWorker {
 public:
  using Snapshot = std::function<StateFrame(mjModel*, mjData*)>;
  using Publish = std::function<void(const CameraFrame&)>;
  SensorWorker(const Engine& engine, Snapshot snapshot, Publish publish);
  ~SensorWorker();
  void Start();
  void Stop();
  bool Failed() const { return failed_.load(); }

 private:
  void Run();
  Config config_;
  int camera_id_;
  Snapshot snapshot_;
  Publish publish_;
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  GLFWwindow* window_ = nullptr;
  std::thread thread_;
  std::atomic_bool stop_{false}, failed_{false};
};

}  // namespace rm_auto_aim_sim
