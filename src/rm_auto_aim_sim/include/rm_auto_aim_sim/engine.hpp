#pragma once

#include <chrono>
#include <memory>
#include <mutex>

#include "rm_auto_aim_sim/config.hpp"
#include "rm_auto_aim_sim/projectile_pool.hpp"
#include "rm_auto_aim_sim/types.hpp"

namespace rm_auto_aim_sim {

// ROS callbacks write only this mailbox, never mjModel/mjData. Wall timeouts are optional.
class CommandMailbox {
 public:
  void SetMode(ControlMode mode);
  void SetPitch(double angle);
  void SetYaw(double angle);
  void SetFire(bool enabled);
  void Apply(const Config& config, double current_pitch, double current_yaw, double& target_pitch,
             double& target_yaw, bool& fire);

 private:
  std::mutex mutex_;
  ControlMode mode_ = ControlMode::kManual;
  bool pitch_valid_ = false, yaw_valid_ = false, fire_ = false;
  double pitch_ = 0, yaw_ = 0;
  std::chrono::steady_clock::time_point pitch_time_, yaw_time_, fire_time_;
};

// Except for Mailbox(), all calls require the application's physics/viewer mutex.
class Engine {
 public:
  explicit Engine(Config config);
  ~Engine();
  Engine(const Engine&) = delete;
  Engine& operator=(const Engine&) = delete;

  void Step();
  void Reset();
  void SetMode(ControlMode mode);
  void SetManualInput(const ManualInput& input) { manual_input_ = input; }
  void AddManualAngles(double pitch_delta, double yaw_delta);
  StateFrame State(const mjData* source = nullptr) const;
  std::vector<HitEvent> TakeHitEvents() { return pool_->TakeHitEvents(); }
  CommandMailbox& Mailbox() { return mailbox_; }
  const Config& GetConfig() const { return config_; }
  mjModel* Model() const { return model_; }
  mjData* Data() const { return data_; }
  int CameraId() const { return camera_id_; }

 private:
  void ApplyGimbalPose();
  Pose BodyPose(const mjData* source, int body) const;
  Config config_;
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  std::unique_ptr<ProjectilePool> pool_;
  CommandMailbox mailbox_;
  ControlMode mode_ = ControlMode::kManual;
  ManualInput manual_input_;
  int base_id_, yaw_id_, pitch_id_, camera_body_id_, camera_id_, muzzle_id_;
  int mocap_id_, yaw_qpos_, pitch_qpos_, yaw_dof_, pitch_dof_;
  double target_pitch_ = 0, target_yaw_ = 0;
  double pitch_ = 0, yaw_ = 0, pitch_velocity_ = 0, yaw_velocity_ = 0;
  double last_time_ = 0, next_shot_time_ = 0;
  bool was_firing_ = false;
};

}  // namespace rm_auto_aim_sim
