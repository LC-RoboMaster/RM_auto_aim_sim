#pragma once
#include <mujoco/mujoco.h>

#include "rm_auto_aim_sim/config.hpp"
namespace rm_auto_aim_sim {
struct CameraIntrinsics {
  double fx, fy, cx, cy;
};
CameraIntrinsics ConfigIntrinsics(const Config& config);
CameraIntrinsics ModelIntrinsics(const mjModel* model, int camera, int width, int height);
void SetCameraIntrinsics(mjModel* model, int camera, int width, int height,
                         const CameraIntrinsics& intrinsics);
}  // namespace rm_auto_aim_sim
