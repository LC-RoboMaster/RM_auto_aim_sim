#pragma once

#include <mujoco/mujoco.h>

#include "rm_auto_aim_sim/engine.hpp"

namespace mujoco {
class Simulate;
}

namespace rm_auto_aim_sim {
class SensorWorker;
void BindViewerSensor(SensorWorker* worker);

// These hooks run under Simulate::mtx. The official free-camera path is untouched.
void BindViewerEngine(Engine* engine);
bool HasViewerEngine();
void AddViewerMenu(mujoco::Simulate* sim);
bool HandleViewerMenu(mujoco::Simulate* sim, const mjuiItem* item);
bool HandleViewerEvent(mujoco::Simulate* sim, mjuiState* state);
void UpdateViewerInput(mujoco::Simulate* sim);
void StepFromViewer();
void ApplyViewerEnvironment(mujoco::Simulate* sim);
void DrawViewerPlots(mujoco::Simulate* sim, mjrRect viewport);
// Called outside the physics mutex after each viewer frame.
void WaitForViewerFrame();

}  // namespace rm_auto_aim_sim
