#include "rm_auto_aim_sim/config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <set>
#include <stdexcept>

namespace rm_auto_aim_sim {
Config Config::Load(const std::string& path) {
  Config config;
  const auto root = YAML::LoadFile(path);
  const auto node = root["rm_auto_aim_sim"]["ros__parameters"];
  if (!node || !node.IsMap()) {
    throw std::runtime_error("Config requires rm_auto_aim_sim: ros__parameters:");
  }
  std::set<std::string> known;
#define AIM_PARAMETER(type, name, value) \
  known.insert(#name);                   \
  if (node[#name]) config.name = node[#name].as<type>();
#include "rm_auto_aim_sim/config_fields.def"
#undef AIM_PARAMETER
  for (const auto& entry : node) {
    const auto key = entry.first.as<std::string>();
    if (!known.count(key)) throw std::runtime_error("Unknown parameter: " + key);
  }
  if (std::filesystem::path(config.scene_path).is_relative()) {
    config.scene_path = (std::filesystem::absolute(path).parent_path() / config.scene_path)
                            .lexically_normal()
                            .string();
  }
  if (!config.field_xml.empty() && std::filesystem::path(config.field_xml).is_relative()) {
    config.field_xml = (std::filesystem::absolute(path).parent_path() / config.field_xml)
                           .lexically_normal()
                           .string();
  }
  config.Validate();
  return config;
}

void Config::Validate() const {
  auto require = [](bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
  };
  auto positive = [&](double value) { return std::isfinite(value) && value > 0; };
  require(initial_mode == "manual" || initial_mode == "auto", "initial_mode: manual or auto");
  require(
      camera_width >= 16 && camera_width <= 4096 && camera_height >= 16 && camera_height <= 4096,
      "camera dimensions must be in [16,4096]");
  require(positive(camera_fps) && camera_fps <= 500, "camera_fps must be in (0,500]");
  require(positive(state_hz) && state_hz <= 1.0 / physics_timestep,
          "state_hz must not exceed physics rate");
  require(positive(physics_timestep) && physics_timestep <= 0.001,
          "physics_timestep must be in (0,0.001] for small projectiles");
  require(positive(projectile_speed) && positive(projectile_diameter) &&
              projectile_speed * physics_timestep <= projectile_diameter,
          "projectile_speed * physics_timestep must be <= projectile_diameter");
  require(positive(projectile_mass) && projectile_pool_size >= 1 && projectile_pool_size <= 2048,
          "projectile mass/pool size invalid (pool: 1..2048)");
  require(positive(fire_rate_hz) && fire_rate_hz <= 1.0 / physics_timestep, "fire_rate_hz invalid");
  require(positive(projectile_lifetime) && positive(projectile_hit_lifetime),
          "projectile lifetimes must be positive");
  require(positive(servo_max_speed) && positive(manual_angular_speed) &&
              positive(manual_translation_speed) && positive(mouse_sensitivity),
          "control speeds invalid");
  require(std::isfinite(yaw_min) && std::isfinite(yaw_max) && yaw_min < yaw_max &&
              std::isfinite(pitch_min) && std::isfinite(pitch_max) && pitch_min < pitch_max,
          "joint limits invalid");
  require(std::isfinite(base_x_min) && std::isfinite(base_x_max) && base_x_min < base_x_max &&
              std::isfinite(base_y_min) && std::isfinite(base_y_max) && base_y_min < base_y_max &&
              std::isfinite(base_z_min) && std::isfinite(base_z_max) && base_z_min < base_z_max,
          "base limits invalid");
  require(std::isfinite(fire_command_timeout) && fire_command_timeout >= 0 &&
              std::isfinite(angle_command_timeout) && angle_command_timeout >= 0,
          "timeouts invalid");
  require(camera_fovy_deg > 1 && camera_fovy_deg < 179 && std::isfinite(camera_fovy_deg),
          "camera_fovy_deg must be in (1,179)");
  require(camera_msaa >= 0 && camera_msaa <= 8, "camera_msaa must be in [0,8]");
  require(!camera_use_intrinsics ||
              (positive(camera_fx) && positive(camera_fy) && std::isfinite(camera_cx) &&
               std::isfinite(camera_cy) && camera_cx >= 0 && camera_cx < camera_width &&
               camera_cy >= 0 && camera_cy < camera_height),
          "Camera K: positive fx/fy and principal point inside image");
  require(
      std::isfinite(lighting_brightness) && lighting_brightness >= 0 && lighting_brightness <= 3,
      "lighting_brightness must be in [0,3]");
  require(target_color == "red" || target_color == "blue", "target_color: red or blue");
  require(positive(target_r1) && positive(target_r2) && target_r1 >= .08 && target_r1 <= 5 &&
              target_r2 >= .08 && target_r2 <= 5 && target_h1 >= .08 && target_h1 <= 5 &&
              target_h2 >= .08 && target_h2 <= 5 && std::isfinite(target_h1) &&
              std::isfinite(target_h2) && std::isfinite(target_x) && std::isfinite(target_y) &&
              std::isfinite(target_z) && std::isfinite(target_angular_speed) &&
              std::abs(target_angular_speed) <= 30 && std::isfinite(target_initial_angle) &&
              std::isfinite(target_tilt_deg) && std::abs(target_tilt_deg) <= 60,
          "Target parameters invalid");
  require(std::isfinite(environment_albedo) && environment_albedo >= 0 && environment_albedo <= 1,
          "environment_albedo must be in [0,1]");
  require(std::isfinite(target_light_whiteness) && target_light_whiteness >= 0 &&
              target_light_whiteness <= 1,
          "target_light_whiteness must be in [0,1]");
  std::set<std::string> frames;
  for (const auto& frame : {frame_world, frame_base, frame_yaw, frame_pitch, frame_camera,
                            frame_camera_optical, frame_muzzle}) {
    require(!frame.empty() && frame.front() != '/' && frames.insert(frame).second,
            "TF frame names must be unique, nonempty, without leading slash");
  }
}
}  // namespace rm_auto_aim_sim
