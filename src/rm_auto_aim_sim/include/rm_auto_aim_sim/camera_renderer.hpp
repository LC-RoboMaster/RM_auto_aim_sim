#pragma once

#include <mujoco/mujoco.h>

#include "rm_auto_aim_sim/types.hpp"

namespace rm_auto_aim_sim {

// Owns a separate scene/FBO. Initialize, Render and destruction need its current GL context.
class CameraRenderer {
 public:
  CameraRenderer(const mjModel* model, int camera_id, int width, int height);
  ~CameraRenderer();
  CameraRenderer(const CameraRenderer&) = delete;
  void Render(mjData* data, CameraFrame& frame);

 private:
  const mjModel* model_;
  int camera_id_, width_, height_;
  mjvScene scene_{};
  mjrContext context_{};
  mjvOption option_{};
  mjvCamera camera_{};
  std::vector<std::uint8_t> readback_;
};

}  // namespace rm_auto_aim_sim
