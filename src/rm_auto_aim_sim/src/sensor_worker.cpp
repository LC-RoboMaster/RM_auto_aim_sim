#include "rm_auto_aim_sim/sensor_worker.hpp"

#include <GLFW/glfw3.h>

#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <stdexcept>
#ifdef __linux__
#include <unistd.h>
#endif

#include "rm_auto_aim_sim/camera_renderer.hpp"

namespace rm_auto_aim_sim {
namespace {
double WallTime() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
double RssMiB() {
#ifdef __linux__
  std::ifstream statm("/proc/self/statm");
  long pages = 0, resident = 0;
  if (statm >> pages >> resident)
    return static_cast<double>(resident) * sysconf(_SC_PAGESIZE) / (1024.0 * 1024.0);
#endif
  return 0;
}
}  // namespace
SensorWorker::SensorWorker(const Engine& engine, Snapshot snapshot, Publish publish)
    : config_(engine.GetConfig()),
      camera_id_(engine.CameraId()),
      snapshot_(std::move(snapshot)),
      publish_(std::move(publish)) {
  SetRendering(config_.camera_shader, config_.camera_shadows,
               config_.camera_exposure, config_.camera_bloom, config_.camera_lamp_intensity);
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
void SensorWorker::SetRendering(bool shader, bool shadows, double exposure, double bloom,
                                double lamp_intensity) {
  shader_.store(shader);
  shadows_.store(shadows);
  exposure_.store(exposure);
  bloom_.store(bloom);
  lamp_intensity_.store(lamp_intensity);
}
CameraMetrics SensorWorker::Metrics() const {
  const std::lock_guard<std::mutex> lock(metrics_mutex_);
  return metrics_;
}

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
    CameraFrame ready;
    CameraMetrics metrics;
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / config_.camera_fps));
    auto deadline = Clock::now();
    auto metric_start = deadline;
    std::size_t frame_count = 0;
    double last_time = -1;
    double last_fps = 0;
    auto cpu_start = deadline;
    std::clock_t process_start = std::clock();
    while (!stop_.load()) {
      const auto snapshot_start = Clock::now();
      frame.state =
          snapshot_(model_, data_);  // Short physics mutex hold; GPU render runs unlocked.
      metrics.snapshot_ms = std::chrono::duration<double, std::milli>(Clock::now() - snapshot_start).count();
      if (frame.state.time != last_time) {
        mj_forward(model_, data_);
        frame.capture_wall_time = WallTime();
        const double capture_sim_time = frame.state.time;
        const bool completed = renderer.RenderAsync(data_, std::move(frame), ready,
            shader_.load(), shadows_.load(), exposure_.load(), bloom_.load(),
            lamp_intensity_.load(), metrics);
        last_time = capture_sim_time;
        if (completed) {
          ++frame_count;
          const double capture_wall_time = ready.capture_wall_time;
          const auto publish_start = Clock::now();
          ready.wall_fps = last_fps;
          publish_(std::move(ready));
          metrics.publish_ms = std::chrono::duration<double, std::milli>(
              Clock::now() - publish_start).count();
          metrics.latency_ms = (WallTime() - capture_wall_time) * 1000.0;
        }
        const auto now = Clock::now();
        const double elapsed = std::chrono::duration<double>(now - metric_start).count();
        if (elapsed >= 1.0) {
          last_fps = frame_count / elapsed;
          metrics.fps = last_fps;
          std::cout << "[camera] actual " << last_fps << " FPS, requested " << config_.camera_fps
                    << '\n';
          frame_count = 0;
          metric_start = now;
        }
        const double cpu_wall = std::chrono::duration<double>(now - cpu_start).count();
        if (cpu_wall >= .5) {
          metrics.cpu_percent = 100.0 * (std::clock() - process_start) /
                                (CLOCKS_PER_SEC * cpu_wall);
          metrics.rss_mib = RssMiB();
          cpu_start = now;
          process_start = std::clock();
        }
        {
          const std::lock_guard<std::mutex> lock(metrics_mutex_);
          metrics_ = metrics;
        }
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
