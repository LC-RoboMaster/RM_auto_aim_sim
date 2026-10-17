#include "rm_auto_aim_sim/engine.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace rm_auto_aim_sim {
namespace {
int RequireId(const mjModel* model, mjtObj type, const std::string& name) {
  const int id = mj_name2id(model, type, name.c_str());
  if (id < 0) throw std::runtime_error("Required model object missing: " + name);
  return id;
}

mjModel* CompileModel(const Config& config) {
  char error[2048] = {};
  std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
      mj_parseXML(config.scene_path.c_str(), nullptr, error, sizeof(error)), mj_deleteSpec);
  if (!spec) throw std::runtime_error(error);
  auto* world = mjs_findBody(spec.get(), "world");
  if (!world) throw std::runtime_error("Model has no world body");
  if (!config.armor_enabled) {
    if (auto* target = mjs_findBody(spec.get(), "target_center"))
      mjs_delete(spec.get(), target->element);
  }
  if (!config.outpost_enabled) {
    if (auto* outpost = mjs_findBody(spec.get(), "outpost_center"))
      mjs_delete(spec.get(), outpost->element);
  }
  // Attach exported upstream field with a prefix, avoiding robot asset/name conflicts.
  std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> field(nullptr, mj_deleteSpec);
  if (!config.field_xml.empty()) {
    field.reset(mj_parseXML(config.field_xml.c_str(), nullptr, error, sizeof(error)));
    if (!field) throw std::runtime_error(error);
    mjs_setDeepCopy(spec.get(), 1);
    auto* frame = mjs_addFrame(world, nullptr);
    if (!mjs_attach(frame->element, mjs_findBody(field.get(), "world")->element, "rmuc/", ""))
      throw std::runtime_error(mjs_getError(spec.get()));
  }
  auto* material = mjs_addMaterial(spec.get(), nullptr);
  mjs_setName(material->element, "sim_projectile_glow");
  material->rgba[0] = 0.05f;
  material->rgba[1] = 1.0f;
  material->rgba[2] = 0.05f;
  material->rgba[3] = 1.0f;
  material->emission = 1.0f;
  for (int index = 0; index < config.projectile_pool_size; ++index) {
    const auto name = "projectile_" + std::to_string(index);
    auto* body = mjs_addBody(world, nullptr);
    mjs_setName(body->element, name.c_str());
    body->pos[0] = 10000.0 + 0.1 * index;
    body->pos[1] = 10000;
    body->pos[2] = 100;
    mjs_addFreeJoint(body);
    auto* geom = mjs_addGeom(body, nullptr);
    mjs_setName(geom->element, (name + "_geom").c_str());
    geom->type = mjGEOM_SPHERE;
    geom->size[0] = config.projectile_diameter / 2;
    geom->mass = config.projectile_mass;
    geom->contype = 2;
    geom->conaffinity = 1;  // Environment=1/2; projectile-projectile pairs are disabled.
    geom->condim = 3;
    geom->friction[0] = 0.3;
    geom->solref[0] = 0.002;
    geom->solref[1] = 1;
    mjs_setString(geom->material, "sim_projectile_glow");
  }
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> compiled(mj_compile(spec.get(), nullptr),
                                                               mj_deleteModel);
  if (!compiled) throw std::runtime_error(mjs_getError(spec.get()));
  mjModel* model = compiled.get();
  model->opt.timestep = config.physics_timestep;
  model->vis.global.offwidth = config.camera_width;
  model->vis.global.offheight = config.camera_height;
  model->vis.quality.offsamples = config.camera_msaa;
  // Parked bodies must not enlarge the automatic far plane or free-camera distance.
  model->stat.extent = 25;
  mju_zero(model->stat.center, 3);
  const int camera = RequireId(model, mjOBJ_CAMERA, config.model_camera_name);
  model->cam_fovy[camera] = config.camera_fovy_deg;
  SetCameraIntrinsics(model, camera, config.camera_width, config.camera_height,
                      ConfigIntrinsics(config));
  if (!config.field_xml.empty()) {
    // Upstream exact NPZ samples are exported as a compact binary sidecar.
    const int hfield = RequireId(model, mjOBJ_HFIELD, "rmuc/rmuc2026_collision");
    std::ifstream samples(config.field_xml + ".heights", std::ios::binary);
    std::int32_t rows = 0, cols = 0;
    samples.read(reinterpret_cast<char*>(&rows), sizeof(rows));
    samples.read(reinterpret_cast<char*>(&cols), sizeof(cols));
    if (!samples || rows != model->hfield_nrow[hfield] || cols != model->hfield_ncol[hfield])
      throw std::runtime_error("Missing/mismatched exact RMUC heightfield sidecar");
    samples.read(reinterpret_cast<char*>(model->hfield_data + model->hfield_adr[hfield]),
                 static_cast<std::streamsize>(rows) * cols * sizeof(float));
    if (!samples) throw std::runtime_error("Truncated RMUC heightfield sidecar");
  }
  const int yaw = RequireId(model, mjOBJ_JOINT, config.model_yaw_joint);
  const int pitch = RequireId(model, mjOBJ_JOINT, config.model_pitch_joint);
  for (const auto& joint :
       {std::make_pair(yaw, std::make_pair(config.yaw_min, config.yaw_max)),
        std::make_pair(pitch, std::make_pair(config.pitch_min, config.pitch_max))}) {
    model->jnt_range[2 * joint.first] = joint.second.first;
    model->jnt_range[2 * joint.first + 1] = joint.second.second;
    model->jnt_limited[joint.first] = 1;
  }
  model->jnt_limited[yaw] = !config.yaw_continuous;
  return compiled.release();
}
}  // namespace

void CommandMailbox::SetMode(ControlMode mode) {
  std::lock_guard<std::mutex> lock(mutex_);
  mode_ = mode;
  pitch_valid_ = yaw_valid_ = fire_ = false;
}

void CommandMailbox::SetPitch(double angle) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_ != ControlMode::kAuto || !std::isfinite(angle)) return;
  pitch_ = angle;
  pitch_valid_ = true;
  pitch_time_ = std::chrono::steady_clock::now();
}

void CommandMailbox::SetYaw(double angle) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_ != ControlMode::kAuto || !std::isfinite(angle)) return;
  yaw_ = angle;
  yaw_valid_ = true;
  yaw_time_ = std::chrono::steady_clock::now();
}

void CommandMailbox::ClearYaw() {
  std::lock_guard<std::mutex> lock(mutex_);
  yaw_valid_ = false;
}

void CommandMailbox::SetFire(bool enabled) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_ != ControlMode::kAuto) return;
  fire_ = enabled;
  fire_time_ = std::chrono::steady_clock::now();
}

void CommandMailbox::Apply(const Config& config, double current_pitch, double current_yaw,
                           double& target_pitch, double& target_yaw, bool& fire) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = std::chrono::steady_clock::now();
  auto fresh = [&](auto time, double timeout) {
    return timeout == 0 || std::chrono::duration<double>(now - time).count() <= timeout;
  };
  if (pitch_valid_)
    target_pitch = fresh(pitch_time_, config.angle_command_timeout) ? pitch_ : current_pitch;
  if (yaw_valid_) target_yaw = fresh(yaw_time_, config.angle_command_timeout) ? yaw_ : current_yaw;
  fire = fire_ && fresh(fire_time_, config.fire_command_timeout);
}

Engine::Engine(Config config) : config_(std::move(config)) {
  config_.Validate();
  try {
    model_ = CompileModel(config_);
    data_ = mj_makeData(model_);
    if (!data_) throw std::runtime_error("Cannot allocate mjData");
    base_id_ = RequireId(model_, mjOBJ_BODY, config_.model_base_body);
    yaw_id_ = RequireId(model_, mjOBJ_BODY, config_.model_yaw_body);
    pitch_id_ = RequireId(model_, mjOBJ_BODY, config_.model_pitch_body);
    camera_body_id_ = RequireId(model_, mjOBJ_BODY, config_.model_camera_body);
    camera_id_ = RequireId(model_, mjOBJ_CAMERA, config_.model_camera_name);
    muzzle_id_ = RequireId(model_, mjOBJ_SITE, config_.model_muzzle_site);
    const int yaw_joint = RequireId(model_, mjOBJ_JOINT, config_.model_yaw_joint);
    const int pitch_joint = RequireId(model_, mjOBJ_JOINT, config_.model_pitch_joint);
    mocap_id_ = model_->body_mocapid[base_id_];
    if (mocap_id_ < 0 || model_->body_parentid[yaw_id_] != base_id_ ||
        model_->body_parentid[pitch_id_] != yaw_id_ ||
        model_->body_parentid[camera_body_id_] != pitch_id_ ||
        model_->cam_bodyid[camera_id_] != camera_body_id_ ||
        model_->site_bodyid[muzzle_id_] != pitch_id_ || model_->body_jntnum[camera_body_id_] != 0 ||
        model_->jnt_type[yaw_joint] != mjJNT_HINGE ||
        model_->jnt_type[pitch_joint] != mjJNT_HINGE || model_->jnt_bodyid[yaw_joint] != yaw_id_ ||
        model_->jnt_bodyid[pitch_joint] != pitch_id_) {
      throw std::runtime_error(
          "Expected mocap base -> yaw hinge -> pitch hinge -> fixed camera; muzzle on pitch");
    }
    yaw_qpos_ = model_->jnt_qposadr[yaw_joint];
    pitch_qpos_ = model_->jnt_qposadr[pitch_joint];
    yaw_dof_ = model_->jnt_dofadr[yaw_joint];
    pitch_dof_ = model_->jnt_dofadr[pitch_joint];
    pool_ = std::make_unique<ProjectilePool>(config_, model_);
    if (config_.armor_enabled) target_ = std::make_unique<ArmorTarget>(config_, model_);
    if (config_.outpost_enabled) outpost_ = std::make_unique<ArmorTarget>(config_, model_, true);
    yaw_continuous_ = config_.yaw_continuous;
    original_diffuse_.assign(model_->light_diffuse, model_->light_diffuse + 3 * model_->nlight);
    original_ambient_.assign(model_->light_ambient, model_->light_ambient + 3 * model_->nlight);
    original_specular_.assign(model_->light_specular, model_->light_specular + 3 * model_->nlight);
    environment_ = std::make_unique<EnvironmentAppearance>(model_);
    environment_->SetAlbedo(config_.environment_albedo);
    environment_->SetDarkSky(config_.dark_sky);
    original_visual_ = model_->vis;
    SetBrightness(config_.lighting_brightness);
    mode_ = config_.initial_mode == "auto" ? ControlMode::kAuto : ControlMode::kManual;
    Reset();
  } catch (...) {
    if (data_) mj_deleteData(data_);
    if (model_) mj_deleteModel(model_);
    throw;
  }
}

Engine::~Engine() {
  mj_deleteData(data_);
  mj_deleteModel(model_);
}

void Engine::Reset() {
  pool_->Reset(data_);
  if (target_) target_->Reset(data_);
  if (outpost_) outpost_->Reset(data_);
  pitch_ = target_pitch_ =
      std::clamp(data_->qpos[pitch_qpos_], config_.pitch_min, config_.pitch_max);
  yaw_ = target_yaw_ = yaw_continuous_ ? data_->qpos[yaw_qpos_]
      : std::clamp(data_->qpos[yaw_qpos_], config_.yaw_min, config_.yaw_max);
  pitch_velocity_ = yaw_velocity_ = 0;
  mailbox_.SetMode(mode_);
  manual_input_ = {};
  last_time_ = data_->time;
  next_shot_time_ = data_->time;
  was_firing_ = false;
  ApplyGimbalPose();
  mj_forward(model_, data_);
}

void Engine::SetMode(ControlMode mode) {
  if (mode_ == mode) return;
  mode_ = mode;
  target_pitch_ = pitch_;
  target_yaw_ = yaw_;
  manual_input_ = {};
  was_firing_ = false;
  mailbox_.SetMode(mode);  // Entering AUTO requires fresh ROS commands.
}

void Engine::AddManualAngles(double pitch_delta, double yaw_delta) {
  if (mode_ != ControlMode::kManual) return;
  target_pitch_ = std::clamp(target_pitch_ + pitch_delta, config_.pitch_min, config_.pitch_max);
  target_yaw_ = yaw_continuous_ ? std::remainder(target_yaw_ + yaw_delta, 2 * mjPI)
      : std::clamp(target_yaw_ + yaw_delta, config_.yaw_min, config_.yaw_max);
}

void Engine::SetYawContinuous(bool enabled) {
  mailbox_.ClearYaw();
  yaw_continuous_ = enabled;
  const int joint = model_->dof_jntid[yaw_dof_];
  model_->jnt_limited[joint] = !enabled;
  yaw_ = std::remainder(yaw_, 2 * mjPI);
  if (!enabled) yaw_ = std::clamp(yaw_, config_.yaw_min, config_.yaw_max);
  target_yaw_ = yaw_;
  yaw_velocity_ = 0;
  ApplyGimbalPose();
  mj_forward(model_, data_);
}

void Engine::ApplyGimbalPose() {
  // Ideal angle servo: no artificial recoil or gravity-induced axis drift.
  data_->qpos[pitch_qpos_] = pitch_;
  data_->qpos[yaw_qpos_] = yaw_;
  data_->qvel[pitch_dof_] = pitch_velocity_;
  data_->qvel[yaw_dof_] = yaw_velocity_;
}

void Engine::Step() {
  if (data_->time < last_time_ - 1e-10) Reset();  // Viewer reset/history rewind.
  const double dt = model_->opt.timestep;
  bool fire = false;
  if (mode_ == ControlMode::kManual) {
    AddManualAngles(manual_input_.pitch_rate * config_.manual_angular_speed * dt,
                    manual_input_.yaw_rate * config_.manual_angular_speed * dt);
    auto* base = data_->mocap_pos + 3 * mocap_id_;
    const double speed = config_.manual_translation_speed * dt;
    base[0] = std::clamp(base[0] + speed * (std::cos(yaw_) * manual_input_.forward -
                                            std::sin(yaw_) * manual_input_.left),
                         config_.base_x_min, config_.base_x_max);
    base[1] = std::clamp(base[1] + speed * (std::sin(yaw_) * manual_input_.forward +
                                            std::cos(yaw_) * manual_input_.left),
                         config_.base_y_min, config_.base_y_max);
    base[2] =
        std::clamp(base[2] + speed * manual_input_.up, config_.base_z_min, config_.base_z_max);
    fire = manual_input_.fire;
  } else {
    mailbox_.Apply(config_, pitch_, yaw_, target_pitch_, target_yaw_, fire);
  }
  target_pitch_ = std::clamp(target_pitch_, config_.pitch_min, config_.pitch_max);
  target_yaw_ = yaw_continuous_ ? std::remainder(target_yaw_, 2 * mjPI)
      : std::clamp(target_yaw_, config_.yaw_min, config_.yaw_max);
  const double max_delta = config_.servo_max_speed * dt;
  const double pitch_delta = std::clamp(target_pitch_ - pitch_, -max_delta, max_delta);
  const double yaw_error = yaw_continuous_ ? std::remainder(target_yaw_ - yaw_, 2 * mjPI)
                                          : target_yaw_ - yaw_;
  const double yaw_delta = std::clamp(yaw_error, -max_delta, max_delta);
  pitch_ += pitch_delta;
  yaw_ += yaw_delta;
  pitch_velocity_ = pitch_delta / dt;
  yaw_velocity_ = yaw_delta / dt;
  pool_->BeforeStep(data_);
  if (target_) target_->Apply(data_);
  if (outpost_) outpost_->Apply(data_);
  ApplyGimbalPose();
  // Only the muzzle pose is needed before mj_step, which evaluates full
  // dynamics itself. Avoid a redundant collision/constraint solve each step.
  mj_kinematics(model_, data_);
  if (fire) {
    if (!was_firing_) next_shot_time_ = data_->time;
    if (data_->time + 1e-10 >= next_shot_time_) {
      pool_->Spawn(data_, State().muzzle);
      next_shot_time_ += 1.0 / config_.fire_rate_hz;
    }
  }
  was_firing_ = fire;
  mj_step(model_, data_);
  pool_->AfterStep(data_);
  if (target_) target_->Advance(dt, data_);
  if (outpost_) outpost_->Advance(dt, data_);
  ApplyGimbalPose();
  mj_forward(model_, data_);  // Recompute poses at the published timestamp, not the previous step.
  last_time_ = data_->time;
}

void Engine::SetIntrinsics(const CameraIntrinsics& k) {
  SetCameraIntrinsics(model_, camera_id_, config_.camera_width, config_.camera_height, k);
}
CameraIntrinsics Engine::Intrinsics() const {
  return ModelIntrinsics(model_, camera_id_, config_.camera_width, config_.camera_height);
}
void Engine::SetBrightness(double brightness) {
  if (!std::isfinite(brightness) || brightness < 0 || brightness > 3)
    throw std::runtime_error("Brightness must be in [0,3]");
  brightness_ = brightness;
  for (int i = 0; i < 3 * model_->nlight; ++i) {
    model_->light_diffuse[i] = original_diffuse_[i] * brightness;
    model_->light_ambient[i] = original_ambient_[i] * brightness;
    model_->light_specular[i] = original_specular_[i] * brightness;
  }
  for (int i = 0; i < 3; ++i) {
    model_->vis.headlight.diffuse[i] = original_visual_.headlight.diffuse[i] * brightness;
    model_->vis.headlight.ambient[i] = original_visual_.headlight.ambient[i] * brightness;
    model_->vis.headlight.specular[i] = original_visual_.headlight.specular[i] * brightness;
  }
}
void Engine::CopyRenderSettings(mjModel* destination) const {
  std::copy(model_->body_pos, model_->body_pos + 3 * model_->nbody, destination->body_pos);
  std::copy(model_->body_quat, model_->body_quat + 4 * model_->nbody, destination->body_quat);
  std::copy(model_->mat_rgba, model_->mat_rgba + 4 * model_->nmat, destination->mat_rgba);
  std::copy(model_->geom_rgba, model_->geom_rgba + 4 * model_->ngeom, destination->geom_rgba);
  std::copy(model_->mat_specular, model_->mat_specular + model_->nmat, destination->mat_specular);
  std::copy(model_->mat_reflectance, model_->mat_reflectance + model_->nmat,
            destination->mat_reflectance);
  std::copy(model_->mat_emission, model_->mat_emission + model_->nmat, destination->mat_emission);
  std::copy(model_->light_diffuse, model_->light_diffuse + 3 * model_->nlight,
            destination->light_diffuse);
  std::copy(model_->light_ambient, model_->light_ambient + 3 * model_->nlight,
            destination->light_ambient);
  std::copy(model_->light_specular, model_->light_specular + 3 * model_->nlight,
            destination->light_specular);
  std::copy(model_->cam_intrinsic, model_->cam_intrinsic + 4 * model_->ncam,
            destination->cam_intrinsic);
  destination->vis.headlight = model_->vis.headlight;
}

Pose Engine::BodyPose(const mjData* source, int body) const {
  Pose pose;
  std::copy(source->xpos + 3 * body, source->xpos + 3 * body + 3, pose.position.begin());
  std::copy(source->xquat + 4 * body, source->xquat + 4 * body + 4, pose.quaternion.begin());
  return pose;
}

StateFrame Engine::State(const mjData* source) const {
  if (!source) source = data_;
  StateFrame frame;
  frame.time = source->time;
  frame.pitch = source->qpos[pitch_qpos_];
  frame.yaw = yaw_continuous_ ? std::remainder(source->qpos[yaw_qpos_], 2 * mjPI)
                             : source->qpos[yaw_qpos_];
  frame.pitch_velocity = pitch_velocity_;
  frame.yaw_velocity = yaw_velocity_;
  frame.mode = mode_;
  frame.shot_count = pool_->ShotCount();
  frame.pool_exhausted_count = pool_->ExhaustedCount();
  frame.active_projectiles = pool_->ActiveCount();
  frame.dark_sky = environment_->DarkSky();
  frame.statistics_shots = pool_->StatisticsShots();
  frame.armor_hits = pool_->ArmorHits();
  frame.plate_hits = pool_->PlateHits();
  frame.outpost_plate_hits = pool_->OutpostPlateHits();
  frame.statistics_epoch = pool_->StatisticsEpoch();
  frame.hit_rate = frame.statistics_shots ? double(frame.armor_hits) / frame.statistics_shots : 0;
  frame.base = BodyPose(source, base_id_);
  frame.yaw_link = BodyPose(source, yaw_id_);
  frame.pitch_link = BodyPose(source, pitch_id_);
  frame.camera_link = BodyPose(source, camera_body_id_);
  std::copy(source->cam_xpos + 3 * camera_id_, source->cam_xpos + 3 * camera_id_ + 3,
            frame.optical.position.begin());
  mjtNum optical_rotation[9];
  const auto* camera_rotation = source->cam_xmat + 9 * camera_id_;
  for (int row = 0; row < 3; ++row) {
    optical_rotation[3 * row] = camera_rotation[3 * row];
    optical_rotation[3 * row + 1] = -camera_rotation[3 * row + 1];
    optical_rotation[3 * row + 2] = -camera_rotation[3 * row + 2];
  }
  mju_mat2Quat(frame.optical.quaternion.data(), optical_rotation);
  std::copy(source->site_xpos + 3 * muzzle_id_, source->site_xpos + 3 * muzzle_id_ + 3,
            frame.muzzle.position.begin());
  mju_mat2Quat(frame.muzzle.quaternion.data(), source->site_xmat + 9 * muzzle_id_);
  return frame;
}
}  // namespace rm_auto_aim_sim
