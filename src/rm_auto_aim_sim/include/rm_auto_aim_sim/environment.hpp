#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <vector>

namespace rm_auto_aim_sim {

// Scene appearance is independent of light intensity. It changes only render
// data: collision masks, target dimensions and all physical properties remain intact.
class EnvironmentAppearance {
 public:
  explicit EnvironmentAppearance(mjModel* model);
  void SetAlbedo(double scale);
  double Albedo() const { return albedo_; }
  void SetDarkSky(bool enabled) { dark_sky_ = enabled; }
  bool DarkSky() const { return dark_sky_; }
  void ConfigureScene(mjvScene* scene) const;

 private:
  struct Surface {
    int geom;
    std::array<float, 4> rgba;
  };
  struct Material {
    int id;
    float specular, reflectance;
  };
  mjModel* model_;
  std::vector<Surface> surfaces_;
  std::vector<Material> materials_;
  double albedo_ = 1;
  bool dark_sky_ = false;
};

// Shared by the viewer and sensor renderer, including imported RMUC sky textures.
void SetBlackSky(mjvScene* scene, bool enabled);

}  // namespace rm_auto_aim_sim
