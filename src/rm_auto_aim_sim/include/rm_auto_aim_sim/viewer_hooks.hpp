#pragma once

#include <mujoco/mujoco.h>

#include "rm_auto_aim_sim/engine.hpp"

namespace mujoco {
class Simulate;
}

namespace rm_auto_aim_sim {

// These hooks run under Simulate::mtx. The official free-camera path is untouched.
void BindViewerEngine(Engine* engine);
void AddViewerMenu(mujoco::Simulate* sim);
bool HandleViewerMenu(mujoco::Simulate* sim, const mjuiItem* item);
bool HandleViewerEvent(mujoco::Simulate* sim, const mjuiState* state);
void UpdateViewerInput(mujoco::Simulate* sim);
void StepFromViewer();

}  // namespace rm_auto_aim_sim
