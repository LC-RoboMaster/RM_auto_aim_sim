#pragma once

#include <memory>

#include "rm_auto_aim_sim/engine.hpp"

namespace rm_auto_aim_sim {

// The stub keeps the same interface so standalone and ROS builds share all physics/UI code.
class RosBridge {
 public:
  RosBridge(int argc, char** argv, Config& config);
  ~RosBridge();
  void Attach(CommandMailbox& mailbox, const StateFrame& initial_state);
  void PublishState(const StateFrame& state, const std::vector<HitEvent>& hits);
  void PublishCamera(CameraFrame&& frame);
  bool Running() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rm_auto_aim_sim
