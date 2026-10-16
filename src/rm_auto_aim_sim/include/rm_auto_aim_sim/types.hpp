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
  bool armor_hit = false;
  int armor_index = -1;
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
  bool dark_sky = false;
  std::uint64_t statistics_shots = 0, armor_hits = 0, statistics_epoch = 0;
  std::array<std::uint64_t, 4> plate_hits{};
  double hit_rate = 0;
  ControlMode mode = ControlMode::kManual;
  Pose base, yaw_link, pitch_link, camera_link, optical, muzzle;
};

struct CameraFrame {
  StateFrame state;
  int width = 0;
  int height = 0;
  double focal_length = 0;  // Compatibility alias for fx.
  double fx = 0, fy = 0, cx = 0, cy = 0;
  double wall_fps = 0;
  double capture_wall_time = 0;
  std::vector<std::uint8_t> rgb;
};

struct CameraMetrics {
  double fps = 0;
  double snapshot_ms = 0, scene_ms = 0, render_ms = 0, gpu_ms = 0;
  double readback_ms = 0, publish_ms = 0;
  double latency_ms = 0, cpu_percent = 0, rss_mib = 0;
  double gpu_estimate_mib = 0;
  std::uint64_t dropped_frames = 0;
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
