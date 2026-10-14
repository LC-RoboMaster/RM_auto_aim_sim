#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include "glfw_adapter.h"
#include "rm_auto_aim_sim/engine.hpp"
#include "rm_auto_aim_sim/ros_bridge.hpp"
#include "rm_auto_aim_sim/sensor_worker.hpp"
#include "rm_auto_aim_sim/viewer_hooks.hpp"
#include "simulate.h"

namespace aim = rm_auto_aim_sim;

int main(int argc, char** argv) {
  try {
    std::string config_path;
    double duration = 0;
    for (int index = 1; index < argc; ++index) {
      const std::string argument(argv[index]);
      if (argument == "--ros-args") break;  // Remaining arguments belong to rclcpp.
      if (argument == "--config" && index + 1 < argc)
        config_path = argv[++index];
      else if (argument == "--duration" && index + 1 < argc)
        duration = std::stod(argv[++index]);
      else if (argument == "--help") {
        std::cout << "rm_auto_aim_sim --config PATH [--duration WALL_SECONDS] [--ros-args ...]\n";
        return 0;
      } else
        throw std::runtime_error("Unknown or incomplete argument: " + argument);
    }
    if (config_path.empty()) throw std::runtime_error("Use --config /path/to/sim.yaml");
    aim::Config config = aim::Config::Load(config_path);
    // Declare engine before bridge: ROS callbacks stop before their mailbox is destroyed.
    std::unique_ptr<aim::Engine> engine;
    aim::RosBridge bridge(argc, argv, config);
    engine = std::make_unique<aim::Engine>(config);
    bridge.Attach(engine->Mailbox(), engine->State());
    aim::BindViewerEngine(engine.get());
    mjvCamera camera;
    mjvOption option;
    mjvPerturb perturb;
    mjv_defaultCamera(&camera);
    mjv_defaultOption(&option);
    mjv_defaultPerturb(&perturb);
    auto sim = std::make_unique<mujoco::Simulate>(std::make_unique<mujoco::GlfwAdapter>(), &camera,
                                                  &option, &perturb, false);
    std::unique_ptr<aim::SensorWorker> sensor;
    if (config.camera_enabled) {
      sensor = std::make_unique<aim::SensorWorker>(
          *engine,
          [&](mjModel* render_model, mjData* destination) {
            const mujoco::MutexLock lock(sim->mtx);
            engine->CopyRenderSettings(render_model);
            mj_copyData(destination, render_model, engine->Data());
            return engine->State();
          },
          [&](const aim::CameraFrame& frame) { bridge.PublishCamera(frame); });
      sensor->Start();
    }
    // The official rendering loop owns the main thread. All physics changes share its mutex.
    std::thread physics([&]() {
      using Clock = std::chrono::steady_clock;
      sim->Load(engine->Model(), engine->Data(), config.scene_path.c_str());
      const auto start = Clock::now();
      auto previous_wall = start;
      double budget = 0;
      double next_state_time = 0;
      double previous_sim_time = 0;
      while (!sim->exitrequest.load() && bridge.Running()) {
        const auto now = Clock::now();
        if (duration > 0 && std::chrono::duration<double>(now - start).count() >= duration) break;
        if (sensor && sensor->Failed()) break;
        const double elapsed = std::chrono::duration<double>(now - previous_wall).count();
        previous_wall = now;
        aim::StateFrame state;
        std::vector<aim::HitEvent> hits;
        bool publish = false;
        {
          const mujoco::MutexLock lock(sim->mtx);
          const double speed = sim->percentRealTime[sim->real_time_index] / 100.0;
          if (engine->Data()->time < previous_sim_time) next_state_time = 0;
          if (sim->run) {
            // Bound each lock hold to eight steps; do not stall the camera snapshot/UI.
            budget = std::min(budget + elapsed * speed, 0.05);
            for (int count = 0; count < 8 && budget >= engine->Model()->opt.timestep; ++count) {
              engine->Step();
              budget -= engine->Model()->opt.timestep;
            }
          } else {
            budget = 0;
            if (engine->Data()->time < previous_sim_time) engine->Reset();
          }
          previous_sim_time = engine->Data()->time;
          if (engine->Data()->time + 1e-10 >= next_state_time) {
            state = engine->State();
            hits = engine->TakeHitEvents();
            next_state_time += 1.0 / config.state_hz;
            if (next_state_time <= state.time) next_state_time = state.time + 1.0 / config.state_hz;
            publish = true;
          }
        }
        if (publish)
          bridge.PublishState(state, hits);  // DDS serialization runs outside physics lock.
        if (budget < config.physics_timestep)
          std::this_thread::sleep_for(std::chrono::microseconds(200));
        else
          std::this_thread::yield();
      }
      sim->exitrequest.store(1);
    });
    std::cout << "[auto_aim] MANUAL/AUTO and Free/Gimbal camera: right Auto Aim menu\n";
    sim->RenderLoop();
    physics.join();
    if (sensor) sensor->Stop();
    const auto final_state = engine->State();
    std::cout << "[auto_aim] exit sim_time=" << final_state.time << " yaw=" << final_state.yaw
              << " pitch=" << final_state.pitch << " shots=" << final_state.shot_count
              << " session_shots=" << final_state.statistics_shots
              << " armor_hits=" << final_state.armor_hits << " hit_rate=" << final_state.hit_rate
              << '\n';
    const bool failed = sensor && sensor->Failed();
    aim::BindViewerEngine(nullptr);
    return failed ? 1 : 0;
  } catch (const std::exception& error) {
    std::cerr << "[auto_aim] " << error.what() << '\n';
    return 1;
  }
}
