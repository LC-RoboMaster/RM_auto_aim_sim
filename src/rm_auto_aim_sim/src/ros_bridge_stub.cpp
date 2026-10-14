#include "rm_auto_aim_sim/ros_bridge.hpp"

namespace rm_auto_aim_sim {
struct RosBridge::Impl {};

RosBridge::RosBridge(int, char**, Config&) : impl_(std::make_unique<Impl>()) {}

RosBridge::~RosBridge() = default;

void RosBridge::Attach(CommandMailbox&, const StateFrame&) {}

void RosBridge::PublishState(const StateFrame&, const std::vector<HitEvent>&) {}

void RosBridge::PublishCamera(const CameraFrame&) {}

bool RosBridge::Running() const { return true; }
}  // namespace rm_auto_aim_sim
