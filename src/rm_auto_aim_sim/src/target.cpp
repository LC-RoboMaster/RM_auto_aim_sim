#include "rm_auto_aim_sim/target.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rm_auto_aim_sim {
namespace {
int Id(const mjModel* model, mjtObj type, const std::string& name) {
  const int id = mj_name2id(model, type, name.c_str());
  if (id < 0) throw std::runtime_error("Target object missing: " + name);
  return id;
}
}  // namespace

ArmorTarget::ArmorTarget(const Config& config, mjModel* model) : model_(model) {
  center_mocap_ = model->body_mocapid[Id(model, mjOBJ_BODY, "target_center")];
  const int joint = Id(model, mjOBJ_JOINT, "target_spin");
  qpos_ = model->jnt_qposadr[joint];
  dof_ = model->jnt_dofadr[joint];
  material_ = Id(model, mjOBJ_MATERIAL, "target_glow");
  for (int i = 0; i < 4; ++i) bodies_[i] = Id(model, mjOBJ_BODY, "armor_" + std::to_string(i));
  settings_.center = {config.target_x, config.target_y, config.target_z};
  settings_.r1 = config.target_r1;
  settings_.r2 = config.target_r2;
  settings_.h1 = config.target_h1;
  settings_.h2 = config.target_h2;
  settings_.tilt = config.target_tilt_deg * mjPI / 180.0;
  settings_.angular_speed = config.target_angular_speed;
  settings_.angle = config.target_initial_angle;
  settings_.color = config.target_color == "blue" ? 1 : 0;
  settings_.light_whiteness = config.target_light_whiteness;
  initial_ = settings_;
}

void ArmorTarget::Configure(const TargetSettings& value, mjData* data) {
  for (double x : value.center)
    if (!std::isfinite(x)) throw std::runtime_error("Target XYZ invalid");
  if (!std::isfinite(value.r1) || !std::isfinite(value.r2) || value.r1 < .08 || value.r2 < .08 ||
      value.r1 > 5 || value.r2 > 5 || !std::isfinite(value.h1) || !std::isfinite(value.h2) ||
      value.h1 < .08 || value.h2 < .08 || value.h1 > 5 || value.h2 > 5 ||
      !std::isfinite(value.angular_speed) || std::abs(value.angular_speed) > 30 ||
      !std::isfinite(value.angle) || !std::isfinite(value.tilt) ||
      std::abs(value.tilt) > mjPI / 3 || value.color < 0 || value.color > 1 ||
      !std::isfinite(value.light_whiteness) || value.light_whiteness < 0 ||
      value.light_whiteness > 1)
    throw std::runtime_error("Target settings out of range");
  settings_ = value;
  for (int i = 0; i < 4; ++i) {
    const double phase = i * mjPI / 2;
    const double radius = i % 2 ? value.r2 : value.r1;
    auto* pos = model_->body_pos + 3 * bodies_[i];
    pos[0] = radius * std::cos(phase);
    pos[1] = radius * std::sin(phase);
    pos[2] = i % 2 ? value.h2 : value.h1;
    // Local +X is the front normal. A negative Y rotation tips it toward the sky.
    const mjtNum yaw[4] = {std::cos(phase / 2), 0, 0, std::sin(phase / 2)};
    const mjtNum tilt[4] = {std::cos(value.tilt / 2), 0, -std::sin(value.tilt / 2), 0};
    mju_mulQuat(model_->body_quat + 4 * bodies_[i], yaw, tilt);
  }
  ApplyLampMaterial();
  // Update subtree inertia after editing fixed-body offsets; preserve live state.
  // mj_setConst also recomputes extent from parked projectile bodies. Keep the
  // authored scene bounds so those far-away slots cannot move the near clip plane.
  const mjStatistic statistics = model_->stat;
  mj_setConst(model_, data);
  model_->stat = statistics;
  Apply(data);
  mj_forward(model_, data);
}

void ArmorTarget::SetLightWhiteness(double whiteness) {
  if (!std::isfinite(whiteness) || whiteness < 0 || whiteness > 1)
    throw std::runtime_error("Lamp whiteness must be in [0,1]");
  settings_.light_whiteness = whiteness;
  ApplyLampMaterial();
}

void ArmorTarget::ApplyLampMaterial() {
  const float color[3] = {settings_.color == 0 ? 1.f : .02f, settings_.color == 0 ? .03f : .12f,
                          settings_.color == 0 ? .02f : 1.f};
  auto* rgba = model_->mat_rgba + 4 * material_;
  // Raising emission above one is not an exposure model. Blend toward white
  // to simulate saturated LED colour and raise grayscale luminance explicitly.
  for (int i = 0; i < 3; ++i) rgba[i] = color[i] + (1 - color[i]) * settings_.light_whiteness;
  rgba[3] = 1;
  model_->mat_emission[material_] = 1;
}

void ArmorTarget::Apply(mjData* data) const {
  std::copy(settings_.center.begin(), settings_.center.end(), data->mocap_pos + 3 * center_mocap_);
  data->qpos[qpos_] = settings_.angle;
  data->qvel[dof_] = settings_.angular_speed;
}

void ArmorTarget::Advance(double dt, mjData* data) {
  settings_.angle = std::remainder(settings_.angle + dt * settings_.angular_speed, 2 * mjPI);
  Apply(data);
}

void ArmorTarget::Reset(mjData* data) { Configure(initial_, data); }
}  // namespace rm_auto_aim_sim
