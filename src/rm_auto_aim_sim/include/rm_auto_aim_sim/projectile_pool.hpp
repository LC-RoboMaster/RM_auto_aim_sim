#pragma once

#include <mujoco/mujoco.h>

#include <vector>

#include "rm_auto_aim_sim/config.hpp"
#include "rm_auto_aim_sim/types.hpp"

namespace rm_auto_aim_sim {

// Bodies are compiled once. Reusing a slot never recompiles or resizes mjModel.
class ProjectilePool {
 public:
  ProjectilePool(const Config& config, const mjModel* model);
  void Reset(mjData* data);
  bool Spawn(mjData* data, const Pose& muzzle);
  void BeforeStep(mjData* data);
  void AfterStep(mjData* data);
  int ActiveCount() const;
  std::uint64_t ShotCount() const { return shot_count_; }
  std::uint64_t ExhaustedCount() const { return exhausted_count_; }
  std::vector<HitEvent> TakeHitEvents();

 private:
  struct Slot {
    int qpos_address = -1;
    int dof_address = -1;
    int geom_id = -1;
    bool active = false;
    double birth_time = 0;
    double hit_time = -1;
    std::uint64_t shot_id = 0;
  };
  void Park(mjData* data, std::size_t index);
  const Config& config_;
  const mjModel* model_;
  std::vector<Slot> slots_;
  std::vector<int> geom_to_slot_;
  std::vector<HitEvent> hits_;
  std::uint64_t shot_count_ = 0;
  std::uint64_t exhausted_count_ = 0;
};

}  // namespace rm_auto_aim_sim
