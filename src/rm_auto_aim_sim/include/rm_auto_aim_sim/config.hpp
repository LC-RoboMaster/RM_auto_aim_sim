#pragma once

#include <string>

namespace rm_auto_aim_sim {

struct Config {
#define AIM_PARAMETER(type, name, value) type name = value;
#include "rm_auto_aim_sim/config_fields.def"
#undef AIM_PARAMETER

  static Config Load(const std::string& path);
  void Validate() const;
};

}  // namespace rm_auto_aim_sim
