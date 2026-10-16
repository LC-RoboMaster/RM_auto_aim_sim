#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "rm_auto_aim_sim/camera_renderer.hpp"
#include "rm_auto_aim_sim/engine.hpp"
extern "C" int RunCameraBenchmark(const char* path, double warmup, double duration) {
  namespace aim = rm_auto_aim_sim;
  using Clock = std::chrono::steady_clock;
  try {
    if (!std::isfinite(warmup) || !std::isfinite(duration) || warmup < 0 || duration <= 0)
      throw std::runtime_error("Invalid benchmark duration");
    aim::Engine engine(aim::Config::Load(path));
    auto begin = Clock::now();
    for (int i = 0; i < 1000; ++i) engine.Step();
    const double step_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - begin).count() / 1000;
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(mj_copyModel(nullptr, engine.Model()),
                                                              mj_deleteModel);
    if (!model) throw std::runtime_error("Cannot copy benchmark model");
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
    if (!data) throw std::runtime_error("Cannot allocate benchmark snapshot");
    begin = Clock::now();
    for (int i = 0; i < 1000; ++i) mj_copyData(data.get(), model.get(), engine.Data());
    const double copy_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - begin).count() / 1000;
    begin = Clock::now();
    for (int i = 0; i < 1000; ++i) mj_forward(model.get(), data.get());
    const double forward_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - begin).count() / 1000;
    {
      const auto& config = engine.GetConfig();
      aim::CameraRenderer renderer(model.get(), engine.CameraId(), config.camera_width,
                                   config.camera_height);
      aim::CameraFrame ready;
      aim::CameraMetrics m;
      int completed = 0, attempt = 0, gpu_samples = 0;
      double gpu = 0, render = 0, readback = 0;
      auto start = Clock::now();
      while (std::chrono::duration<double>(Clock::now() - start).count() < warmup) {
        aim::CameraFrame frame;
        frame.state = engine.State();
        frame.state.time = ++attempt;
        renderer.RenderAsync(data.get(), std::move(frame), ready, config.camera_shader,
                             config.camera_shadows, config.camera_exposure, config.camera_bloom,
                             config.camera_lamp_intensity, m);
      }
      start = Clock::now();
      while (std::chrono::duration<double>(Clock::now() - start).count() < duration) {
        aim::CameraFrame frame;
        frame.state = engine.State();
        frame.state.time = ++attempt;
        if (renderer.RenderAsync(data.get(), std::move(frame), ready, config.camera_shader,
                                 config.camera_shadows, config.camera_exposure, config.camera_bloom,
                                 config.camera_lamp_intensity, m)) {
          ++completed;
          if (m.gpu_timer_valid) {
            gpu += m.gpu_ms;
            ++gpu_samples;
          }
          render += m.render_ms;
          readback += m.readback_ms;
        }
      }
      if (!completed) throw std::runtime_error("No completed benchmark frames");
      const double seconds = std::chrono::duration<double>(Clock::now() - start).count();
      std::cout << "{\"step_ms\":" << step_ms << ",\"copy_ms\":" << copy_ms
                << ",\"redundant_forward_ms\":" << forward_ms
                << ",\"render_fps\":" << completed / seconds
                << ",\"gpu_ms\":" << (gpu_samples ? std::to_string(gpu / gpu_samples) : "null")
                << ",\"render_submit_ms\":" << render / completed
                << ",\"map_ms\":" << readback / completed << "}" << std::endl;
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "[camera benchmark] " << error.what() << '\n';
    return 1;
  }
}
