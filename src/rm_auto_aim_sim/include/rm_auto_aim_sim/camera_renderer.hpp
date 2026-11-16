#pragma once

#include <mujoco/mujoco.h>

#include <memory>

#include "rm_auto_aim_sim/types.hpp"

namespace rm_auto_aim_sim {

// Owns a separate scene/FBO. Initialize, Render and destruction need its current GL context.
class CameraRenderer {
 public:
  CameraRenderer(const mjModel* model, int camera_id, int width, int height);
  ~CameraRenderer();
  CameraRenderer(const CameraRenderer&) = delete;
  void Render(mjData* data, CameraFrame& frame);
  // Sensor scene only; the physics model and desktop viewer remain unchanged.
  void SetProjectileOpacity(double opacity);
  // Returns a completed capture when the bounded PBO queue has one ready.
  // A backwards capture timestamp discards pending images before collection.
  bool RenderAsync(mjData* data, CameraFrame capture, CameraFrame& ready, bool shader, bool shadows,
                   double exposure, double bloom, double lamp_intensity, CameraMetrics& metrics);
  // Poll without submitting a new capture, including while simulation is paused.
  bool CollectReady(CameraFrame& ready, CameraMetrics& metrics);

 private:
  void ApplyProjectileOpacity();
  std::vector<bool> projectile_geoms_;
  float projectile_opacity_ = 1;
  const mjModel* model_;
  int camera_id_, width_, height_;
  mjvScene scene_{};
  mjrContext context_{};
  mjvOption option_{};
  mjvCamera camera_{};
  std::vector<std::uint8_t> readback_;
  struct Async;
  std::unique_ptr<Async> async_;
};

}  // namespace rm_auto_aim_sim
