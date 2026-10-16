#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "rm_auto_aim_sim/engine.hpp"

namespace aim = rm_auto_aim_sim;
namespace {
void Check(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
bool Near(double a, double b) { return std::abs(a - b) < 1e-5; }
// MuJoCo stores cam_intrinsic/sensorsize as floats; allow their round-trip error.
bool NearPixel(double actual, double expected) {
  return std::abs(actual - expected) <= 1e-6 * std::max(1.0, std::abs(expected));
}
}  // namespace

int main(int argc, char** argv) {
  try {
    Check(argc == 2, "Pass the documented sim.yaml");
    std::set<std::string> declared;
#define AIM_PARAMETER(type, name, value) declared.insert(#name);
#include "rm_auto_aim_sim/config_fields.def"
#undef AIM_PARAMETER
    const auto parameters = YAML::LoadFile(argv[1])["rm_auto_aim_sim"]["ros__parameters"];
    std::ifstream input(argv[1]);
    std::vector<std::string> lines;
    for (std::string line; std::getline(input, line);) lines.push_back(line);
    std::set<std::string> documented;
    for (const auto& entry : parameters) {
      const auto key = entry.first.as<std::string>();
      Check(documented.insert(key).second, "Duplicate parameter in sample YAML: " + key);
      Check(declared.count(key), "Undeclared sample parameter: " + key);
      const auto& line = lines.at(entry.first.Mark().line);
      Check(line.find('#') != std::string::npos, "Missing inline parameter comment: " + key);
    }
    for (const auto& key : declared)
      Check(documented.count(key), "Parameter missing from sample YAML: " + key);

    const auto config = aim::Config::Load(argv[1]);
    aim::Engine engine(config);
    Check(engine.Environment().DarkSky() == config.dark_sky &&
              Near(engine.Environment().Albedo(), config.environment_albedo) &&
              Near(engine.Brightness(), config.lighting_brightness),
          "Startup environment configuration did not reach Engine");
    const auto expected_k = aim::ConfigIntrinsics(config);
    const auto k = engine.Intrinsics();
    Check(NearPixel(k.fx, expected_k.fx) && NearPixel(k.fy, expected_k.fy) &&
              NearPixel(k.cx, expected_k.cx) && NearPixel(k.cy, expected_k.cy),
          "Startup K did not reach the camera model");
    if (engine.Target()) {
      const auto& target = engine.Target()->Settings();
      Check(Near(target.light_whiteness, config.target_light_whiteness) &&
                Near(target.tilt, config.target_tilt_deg * mjPI / 180.0) &&
                Near(target.angular_speed, config.target_angular_speed) &&
                target.color == (config.target_color == "blue" ? 1 : 0),
            "Startup target configuration did not reach the armor model");
    }
    for (double opacity : {0., .5, 1., -.1, 1.1, std::nan("")}) {
      auto candidate = config;
      candidate.camera_projectile_opacity = opacity;
      bool accepted = true;
      try { candidate.Validate(); } catch (const std::runtime_error&) { accepted = false; }
      Check(accepted == (std::isfinite(opacity) && opacity >= 0 && opacity <= 1),
            "Projectile opacity range validation failed");
    }
    auto alternate = config;
    alternate.initial_view = config.initial_view == "orbit" ? "gimbal" : "orbit";
    alternate.Validate();
    alternate.initial_view = "invalid";
    bool rejected = false;
    try {
      alternate.Validate();
    } catch (const std::runtime_error&) {
      rejected = true;
    }
    Check(rejected, "Invalid startup view must be rejected");
    std::cout << "PASS: " << declared.size()
              << " documented YAML parameters, no missing/duplicate keys, startup "
                 "environment/K/target propagation and view validation.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "CONFIG FAIL: " << error.what() << '\n';
    return 1;
  }
}
