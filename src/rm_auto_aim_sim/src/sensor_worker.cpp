#include "rm_auto_aim_sim/sensor_worker.hpp"

#include <GLFW/glfw3.h>

#include <chrono>
#include <iostream>
#include <stdexcept>

#include "rm_auto_aim_sim/camera_renderer.hpp"

namespace rm_auto_aim_sim {
SensorWorker::SensorWorker(const Engine& engine, Snapshot snapshot, Publish publish)
    : config_(engine.GetConfig()),
      camera_id_(engine.CameraId()),
      snapshot_(std::move(snapshot)),
      publish_(std::move(publish)) {
  try {
    model_ = mj_copyModel(nullptr, engine.Model());
    if (!model_) throw std::runtime_error("Cannot allocate camera model");
    data_ = mj_makeData(model_);
    if (!data_) throw std::runtime_error("Cannot allocate camera snapshot");
    if (!glfwInit()) throw std::runtime_error("GLFW init failed: display unavailable");
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_SAMPLES, 0);
    window_ = glfwCreateWindow(32, 32, "RM sensor context", nullptr, nullptr);
    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
    if (!window_) throw std::runtime_error("Cannot create camera OpenGL context");
  } catch (...) {
    if (window_) glfwDestroyWindow(window_);
    if (data_) mj_deleteData(data_);
    if (model_) mj_deleteModel(model_);
    throw;
  }
}

SensorWorker::~SensorWorker() {
  Stop();
  if (window_) glfwDestroyWindow(window_);
  mj_deleteData(data_);
  mj_deleteModel(model_);
}

void SensorWorker::Start() { thread_ = std::thread(&SensorWorker::Run, this); }

void SensorWorker::Stop() {
  stop_.store(true);
  if (thread_.joinable()) thread_.join();
}

void SensorWorker::Run() {
  using Clock = std::chrono::steady_clock;
  glfwMakeContextCurrent(window_);
  try {
    CameraRenderer renderer(model_, camera_id_, config_.camera_width, config_.camera_height);
    CameraFrame frame;
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / config_.camera_fps));
    auto deadline = Clock::now();
    auto metric_start = deadline;
    std::size_t frame_count = 0;
    double last_time = -1;
    double last_fps = 0;
    while (!stop_.load()) {
      frame.state =
          snapshot_(model_, data_);  // Short physics mutex hold; GPU render runs unlocked.
      if (frame.state.time != last_time) {
        mj_forward(model_, data_);
        renderer.Render(data_, frame);
        last_time = frame.state.time;
        ++frame_count;
        const auto now = Clock::now();
        const double elapsed = std::chrono::duration<double>(now - metric_start).count();
        if (elapsed >= 2.0) {
          last_fps = frame_count / elapsed;
          std::cout << "[camera] actual " << last_fps << " FPS, requested " << config_.camera_fps
                    << '\n';
          frame_count = 0;
          metric_start = now;
        }
        frame.wall_fps = last_fps;
        publish_(frame);
      }
      deadline += period;
      // Drop overdue captures instead of building a backlog of stale images.
      if (deadline < Clock::now()) deadline = Clock::now();
      std::this_thread::sleep_until(deadline);
    }
  } catch (const std::exception& error) {
    std::cerr << "[camera] " << error.what() << '\n';
    failed_.store(true);
  }
  glfwMakeContextCurrent(nullptr);
}
}  // namespace rm_auto_aim_sim
