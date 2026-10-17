#include "rm_auto_aim_sim/projectile_pool.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace rm_auto_aim_sim {
ProjectilePool::ProjectilePool(const Config& config, const mjModel* model)
    : config_(config), model_(model), geom_to_slot_(model->ngeom, -1) {
  for (int index = 0; index < config.projectile_pool_size; ++index) {
    const auto name = "projectile_" + std::to_string(index);
    const int body = mj_name2id(model, mjOBJ_BODY, name.c_str());
    const int geom = mj_name2id(model, mjOBJ_GEOM, (name + "_geom").c_str());
    if (body < 0 || geom < 0) throw std::runtime_error("Missing projectile pool body: " + name);
    const int joint = model->body_jntadr[body];
    slots_.push_back({model->jnt_qposadr[joint], model->jnt_dofadr[joint], geom});
    geom_to_slot_[geom] = index;
  }
  geom_to_armor_.assign(model->ngeom, -1);
  for (int i = 0; i < 4; ++i) {
    const int geom = mj_name2id(model, mjOBJ_GEOM, ("armor_hitbox_" + std::to_string(i)).c_str());
    if (geom >= 0) geom_to_armor_[geom] = i;
  }
  geom_to_outpost_.assign(model->ngeom, -1);
  for (int i = 0; i < 3; ++i) {
    const int geom = mj_name2id(model, mjOBJ_GEOM, ("outpost_hitbox_" + std::to_string(i)).c_str());
    if (geom >= 0) geom_to_outpost_[geom] = i;
  }
  hits_.reserve(config.projectile_pool_size);
}

void ProjectilePool::Park(mjData* data, std::size_t index) {
  auto& slot = slots_[index];
  auto* position = data->qpos + slot.qpos_address;
  // Above and far outside the arena: no floor penetration, no visible parked balls.
  position[0] = 10000.0 + 0.1 * index;
  position[1] = 10000.0;
  position[2] = 100.0;
  position[3] = 1;
  std::fill(position + 4, position + 7, 0);
  std::fill(data->qvel + slot.dof_address, data->qvel + slot.dof_address + 6, 0);
  std::fill(data->qacc_warmstart + slot.dof_address, data->qacc_warmstart + slot.dof_address + 6,
            0);
  slot.active = false;
}

void ProjectilePool::Reset(mjData* data) {
  for (std::size_t index = 0; index < slots_.size(); ++index) Park(data, index);
  hits_.clear();
  shot_count_ = 0;
  exhausted_count_ = 0;
  ResetStatistics();
}

bool ProjectilePool::Spawn(mjData* data, const Pose& muzzle) {
  for (auto& slot : slots_) {
    if (slot.active) continue;
    auto* position = data->qpos + slot.qpos_address;
    std::copy(muzzle.position.begin(), muzzle.position.end(), position);
    std::copy(muzzle.quaternion.begin(), muzzle.quaternion.end(), position + 3);
    mjtNum rotation[9];
    mju_quat2Mat(rotation, muzzle.quaternion.data());
    auto* velocity = data->qvel + slot.dof_address;
    // Muzzle +X is forward. Initial speed is exactly the configured world speed.
    for (int axis = 0; axis < 3; ++axis)
      velocity[axis] = config_.projectile_speed * rotation[3 * axis];
    std::fill(velocity + 3, velocity + 6, 0);
    std::fill(data->qacc_warmstart + slot.dof_address, data->qacc_warmstart + slot.dof_address + 6,
              0);
    slot.active = true;
    slot.birth_time = data->time;
    slot.hit_time = -1;
    slot.shot_id = ++shot_count_;
    slot.statistics_epoch = statistics_epoch_;
    ++statistics_shots_;
    return true;
  }
  ++exhausted_count_;  // Drop a shot instead of stealing a still-active projectile.
  return false;
}

void ProjectilePool::BeforeStep(mjData* data) {
  for (std::size_t index = 0; index < slots_.size(); ++index) {
    auto& slot = slots_[index];
    if (slot.active &&
        (data->time - slot.birth_time >= config_.projectile_lifetime ||
         (slot.hit_time >= 0 && data->time - slot.hit_time >= config_.projectile_hit_lifetime))) {
      Park(data, index);
    } else if (!slot.active) {
      Park(data, index);
    }
  }
}

void ProjectilePool::AfterStep(mjData* data) {
  for (int index = 0; index < data->ncon; ++index) {
    const auto& contact = data->contact[index];
    for (int side = 0; side < 2; ++side) {
      const int geom = contact.geom[side];
      if (geom < 0 || geom >= static_cast<int>(geom_to_slot_.size())) continue;
      const int slot_index = geom_to_slot_[geom];
      if (slot_index < 0) continue;
      auto& slot = slots_[slot_index];
      if (!slot.active || slot.hit_time >= 0 || contact.dist > 0) continue;
      slot.hit_time = data->time;
      const char* other = mj_id2name(model_, mjOBJ_GEOM, contact.geom[1 - side]);
      HitEvent event;
      event.shot_id = slot.shot_id;
      event.armor_index = geom_to_armor_[contact.geom[1 - side]];
      const int outpost_index = geom_to_outpost_[contact.geom[1 - side]];
      if (outpost_index >= 0) event.armor_index = outpost_index;
      event.armor_hit = event.armor_index >= 0;
      event.target_type = event.armor_hit ? (outpost_index >= 0 ? "outpost" : "armor") : "environment";
      // Only the first contact counts. Old in-flight shots cannot contaminate a reset session.
      if (event.armor_hit && slot.statistics_epoch == statistics_epoch_) {
        ++armor_hits_;
        if (outpost_index >= 0) ++outpost_plate_hits_[outpost_index];
        else ++plate_hits_[event.armor_index];
      }
      event.time = data->time;
      event.geom_name = other ? other : "unnamed";
      std::copy(contact.pos, contact.pos + 3, event.position.begin());
      // A bounded queue also limits memory if no ROS consumer is present.
      if (hits_.size() < slots_.size()) hits_.push_back(std::move(event));
    }
  }
}

void ProjectilePool::ResetStatistics() {
  statistics_shots_ = armor_hits_ = 0;
  plate_hits_.fill(0);
  outpost_plate_hits_.fill(0);
  ++statistics_epoch_;
}

int ProjectilePool::ActiveCount() const {
  return std::count_if(slots_.begin(), slots_.end(), [](const Slot& slot) { return slot.active; });
}

std::vector<HitEvent> ProjectilePool::TakeHitEvents() {
  std::vector<HitEvent> events;
  events.swap(hits_);
  hits_.reserve(slots_.size());
  return events;
}
}  // namespace rm_auto_aim_sim
