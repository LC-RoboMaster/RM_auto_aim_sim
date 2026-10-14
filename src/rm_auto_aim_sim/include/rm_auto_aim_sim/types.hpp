#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace rm_auto_aim_sim {

enum class ControlMode { kManual = 0, kAuto = 1 };

struct Pose {
  std::array<double, 3> position{};
  std::array<double, 4> quaternion{1, 0, 0, 0};  // MuJoCo order: w,x,y,z.
};

struct HitEvent {
  std::uint64_t shot_id = 0;
  double time = 0;
  std::string geom_name;
  std::array<double, 3> position{};
};

struct StateFrame {
  double time = 0;
  double pitch = 0;
  double yaw = 0;
  double pitch_velocity = 0;
  double yaw_velocity = 0;
  std::uint64_t shot_count = 0;
  std::uint64_t pool_exhausted_count = 0;
  int active_projectiles = 0;
  ControlMode mode = ControlMode::kManual;
  Pose base, yaw_link, pitch_link, camera_link, optical, muzzle;
};

struct CameraFrame {
  StateFrame state;
  int width = 0;
  int height = 0;
  double focal_length = 0;
  double wall_fps = 0;
  std::vector<std::uint8_t> rgb;
};

struct ManualInput {
  double forward = 0;
  double left = 0;
  double up = 0;
  double pitch_rate = 0;
  double yaw_rate = 0;
  bool fire = false;
};

}  // namespace rm_auto_aim_sim
