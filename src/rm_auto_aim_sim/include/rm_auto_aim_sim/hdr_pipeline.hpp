#pragma once

#include <memory>
#include <mujoco/mujoco.h>

namespace rm_auto_aim_sim {

// OpenGL resources live on the sensor thread. MuJoCo rasterizes the textured
// geometry; these shaders add HDR lamp light, bloom and camera response.
class HdrPipeline {
 public:
  HdrPipeline(int width, int height);
  ~HdrPipeline();
  HdrPipeline(const HdrPipeline&) = delete;
  void Process(const mjvScene& scene, const mjrContext& context,
               double fx, double fy, double cx, double cy,
               double exposure, double bloom, double lamp_intensity,
               bool shader_enabled);
  void ReadPixelsToPbo(unsigned int pbo) const;
  double EstimatedMiB() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rm_auto_aim_sim
