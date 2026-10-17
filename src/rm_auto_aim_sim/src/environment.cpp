#include "rm_auto_aim_sim/environment.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string>

namespace rm_auto_aim_sim {
EnvironmentAppearance::EnvironmentAppearance(mjModel* model) : model_(model) {
  std::set<int> material_ids;
  for (int geom = 0; geom < model->ngeom; ++geom) {
    const char* name = mj_id2name(model, mjOBJ_GEOM, geom);
    const bool field = name && std::string(name).rfind("rmuc/", 0) == 0;
    if (model->geom_bodyid[geom] != 0 && !field) continue;
    const int material = model->geom_matid[geom];
    const auto* color = model->geom_rgba + 4 * geom;
    // Match MuJoCo's material inheritance: default geom RGBA uses material RGBA.
    if (material >= 0 && color[0] == .5f && color[1] == .5f && color[2] == .5f && color[3] == 1)
      color = model->mat_rgba + 4 * material;
    Surface surface{geom, {}};
    std::copy(color, color + 4, surface.rgba.begin());
    surfaces_.push_back(surface);
    if (material >= 0) material_ids.insert(material);
  }
  for (int id : material_ids)
    materials_.push_back({id, model->mat_specular[id], model->mat_reflectance[id]});
}

void EnvironmentAppearance::SetAlbedo(double scale) {
  if (!std::isfinite(scale) || scale < 0 || scale > 1)
    throw std::runtime_error("environment_albedo must be in [0,1]");
  albedo_ = scale;
  // Always scale from the source colours, so repeated slider updates do not
  // progressively blacken the scene. Textures are modulated by this colour.
  for (const auto& surface : surfaces_) {
    auto* color = model_->geom_rgba + 4 * surface.geom;
    for (int axis = 0; axis < 3; ++axis) color[axis] = surface.rgba[axis] * scale;
    color[3] = surface.rgba[3];
  }
  for (const auto& material : materials_) {
    model_->mat_specular[material.id] = material.specular * scale;
    model_->mat_reflectance[material.id] = material.reflectance * scale;
  }
}

void SetBlackSky(mjvScene* scene, bool enabled) {
  // Rendering without a skybox clears the background to black; disabling haze
  // also removes the bright horizon strip. GPU textures need no per-frame upload.
  scene->flags[mjRND_SKYBOX] = enabled ? 0 : 1;
  scene->flags[mjRND_HAZE] = enabled ? 0 : 1;
}
void EnvironmentAppearance::ConfigureScene(mjvScene* scene) const { SetBlackSky(scene, dark_sky_); }
}  // namespace rm_auto_aim_sim
