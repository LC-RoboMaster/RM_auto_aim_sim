#pragma once

#include <mujoco/mujoco.h>

#include <array>

#include "rm_auto_aim_sim/config.hpp"

namespace rm_auto_aim_sim {

// Pose controls use world metres, radians and signed rad/s. Heights are plate centres.
struct TargetSettings {
  std::array<double, 3> center{5, 0, 0};
  double r1 = 0.6, r2 = 0.6, h1 = 0.8, h2 = 0.8;
  double tilt = 15.0 * mjPI / 180.0;
  double angular_speed = 0, angle = 0;
  double light_whiteness = 0;
  int color = 0;  // 0 red, 1 blue.
};

class ArmorTarget {
 public:
  ArmorTarget(const Config& config, mjModel* model);
  const TargetSettings& Settings() const { return settings_; }
  void Configure(const TargetSettings& settings, mjData* data);
  void Apply(mjData* data) const;
  void Advance(double dt, mjData* data);
  void Reset(mjData* data);
  void SetLightWhiteness(double whiteness);

 private:
  void ApplyLampMaterial();
  mjModel* model_;
  TargetSettings settings_, initial_;
  int center_mocap_, qpos_, dof_, material_;
  std::array<int, 4> bodies_;
};

}  // namespace rm_auto_aim_sim
