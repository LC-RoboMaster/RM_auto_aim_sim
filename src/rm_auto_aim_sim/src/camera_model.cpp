#include "rm_auto_aim_sim/camera_model.hpp"

#include <cmath>
#include <stdexcept>
namespace rm_auto_aim_sim {
CameraIntrinsics ConfigIntrinsics(const Config& config) {
  if (config.camera_use_intrinsics)
    return {config.camera_fx, config.camera_fy, config.camera_cx, config.camera_cy};
  const double f = .5 * config.camera_height / std::tan(config.camera_fovy_deg * mjPI / 360);
  return {f, f, .5 * (config.camera_width - 1), .5 * (config.camera_height - 1)};
}
void SetCameraIntrinsics(mjModel* model, int camera, int width, int height,
                         const CameraIntrinsics& k) {
  if (!std::isfinite(k.fx) || !std::isfinite(k.fy) || k.fx <= 0 || k.fy <= 0 ||
      !std::isfinite(k.cx) || !std::isfinite(k.cy) || k.cx < 0 || k.cx >= width || k.cy < 0 ||
      k.cy >= height)
    throw std::runtime_error("Invalid camera intrinsics");
  // Pixel centres use OpenCV's zero-based convention. MuJoCo principal offsets
  // are measured from the sensor centre, with Y upward (opposite image rows).
  constexpr double pixel = .001;
  model->cam_sensorsize[2 * camera] = width * pixel;
  model->cam_sensorsize[2 * camera + 1] = height * pixel;
  model->cam_resolution[2 * camera] = width;
  model->cam_resolution[2 * camera + 1] = height;
  auto* p = model->cam_intrinsic + 4 * camera;
  p[0] = k.fx * pixel;
  p[1] = k.fy * pixel;
  p[2] = (.5 * width - k.cx - .5) * pixel;
  p[3] = (k.cy + .5 - .5 * height) * pixel;
  model->cam_orthographic[camera] = 0;
  model->cam_ipd[camera] = 0;
}
CameraIntrinsics ModelIntrinsics(const mjModel* model, int camera, int width, int height) {
  const auto* p = model->cam_intrinsic + 4 * camera;
  if (model->cam_sensorsize[2 * camera + 1] > 0) {
    const double sx = width / model->cam_sensorsize[2 * camera];
    const double sy = height / model->cam_sensorsize[2 * camera + 1];
    return {p[0] * sx, p[1] * sy, .5 * (width - 1) - p[2] * sx, .5 * (height - 1) + p[3] * sy};
  }
  const double f = .5 * height / std::tan(model->cam_fovy[camera] * mjPI / 360);
  return {f, f, .5 * (width - 1), .5 * (height - 1)};
}
}  // namespace rm_auto_aim_sim
