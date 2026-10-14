#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include "rm_auto_aim_sim/engine.hpp"

namespace aim = rm_auto_aim_sim;
void Check(bool ok, const char* reason) {
  if (!ok) throw std::runtime_error(reason);
}
bool Near(double a, double b) { return std::abs(a - b) < 1e-6; }

int main(int argc, char** argv) {
  try {
    Check(argc == 2, "Pass config YAML");
    auto config = aim::Config::Load(argv[1]);
    config.projectile_pool_size = 8;
    aim::Engine engine(config);
    if (!config.field_xml.empty()) {
      const int id = mj_name2id(engine.Model(), mjOBJ_HFIELD, "rmuc/rmuc2026_collision");
      Check(id >= 0, "Attached RMUC hfield missing");
      std::ifstream input(config.field_xml + ".heights", std::ios::binary);
      std::int32_t rows = 0, cols = 0;
      input.read(reinterpret_cast<char*>(&rows), sizeof(rows));
      input.read(reinterpret_cast<char*>(&cols), sizeof(cols));
      std::vector<float> values(rows * cols);
      input.read(reinterpret_cast<char*>(values.data()), values.size() * sizeof(float));
      Check(input.good(), "Cannot read reference samples");
      for (std::size_t i = 0; i < values.size(); ++i)
        Check(values[i] == engine.Model()->hfield_data[engine.Model()->hfield_adr[id] + i],
              "Field loader lost upstream float precision");
    }
    auto* target = engine.Target();
    Check(target, "Target missing");
    auto settings = target->Settings();
    settings.center = {2, 3, .1};
    settings.r1 = .6;
    settings.r2 = .9;
    settings.h1 = .5;
    settings.h2 = .8;
    settings.angular_speed = 2;
    settings.angle = 0;
    target->Configure(settings, engine.Data());
    for (int i = 0; i < 4; ++i) {
      int body = mj_name2id(engine.Model(), mjOBJ_BODY, ("armor_" + std::to_string(i)).c_str());
      auto* pos = engine.Data()->xpos + 3 * body;
      auto* mat = engine.Data()->xmat + 9 * body;
      const double phase = i * mjPI / 2, r = i % 2 ? .9 : .6;
      Check(Near(pos[0], 2 + r * std::cos(phase)) && Near(pos[1], 3 + r * std::sin(phase)),
            "r1/r2 orbit incorrect");
      Check(Near(pos[2], .1 + (i % 2 ? .8 : .5)), "Opposite heights incorrect");
      Check(Near(mat[6], std::sin(mjPI / 12)), "Armor front normal must point 15 degrees upward");
    }
    Check(engine.Model()->stat.extent == 25, "Target edit changed clipping bounds");
    for (int i = 0; i < 500; ++i) engine.Step();
    Check(Near(target->Settings().angle, .5), "Spin speed or heading drift");
    settings = target->Settings();
    settings.angular_speed = -2;
    target->Configure(settings, engine.Data());
    for (int i = 0; i < 500; ++i) engine.Step();
    Check(Near(target->Settings().angle, 0), "Reverse spin failed");
    settings.color = 1;
    target->Configure(settings, engine.Data());
    const int material = mj_name2id(engine.Model(), mjOBJ_MATERIAL, "target_glow");
    Check(engine.Model()->mat_rgba[4 * material + 2] == 1, "Blue switch failed");
    engine.SetIntrinsics({800, 910, 300, 220});
    const auto k = engine.Intrinsics();
    Check(std::abs(k.fx - 800) < 1e-4 && std::abs(k.fy - 910) < 1e-4 &&
              std::abs(k.cx - 300) < 1e-4 && std::abs(k.cy - 220) < 1e-4,
          "K roundtrip failed");
    engine.SetBrightness(0);
    Check(engine.Model()->vis.headlight.diffuse[0] == 0, "Headlight dimming failed");
    engine.SetBrightness(1);

    // A real 17 mm sphere shot approaches the outward-facing plate 2 from -X.
    settings.center = {5, 0, 0};
    settings.r1 = settings.r2 = .6;
    settings.h1 = settings.h2 = .8;
    settings.angle = 0;
    settings.angular_speed = 0;
    target->Configure(settings, engine.Data());
    aim::ProjectilePool pool(config, engine.Model());
    pool.Reset(engine.Data());
    aim::Pose muzzle;
    muzzle.position = {3.5, 0, .8};
    auto advance = [&](int steps) {
      for (int i = 0; i < steps; ++i) {
        pool.BeforeStep(engine.Data());
        target->Apply(engine.Data());
        mj_step(engine.Model(), engine.Data());
        pool.AfterStep(engine.Data());
      }
    };
    Check(pool.Spawn(engine.Data(), muzzle), "Spawn failed");
    advance(150);
    Check(pool.StatisticsShots() == 1 && pool.ArmorHits() == 1 && pool.PlateHits()[2] == 1,
          "Ballistic armor hit not counted");
    advance(500);
    Check(pool.ArmorHits() == 1, "Repeated contact double-counted");
    const auto events = pool.TakeHitEvents();
    Check(events.size() == 1 && events[0].armor_hit && events[0].armor_index == 2,
          "Hit identity wrong");
    pool.ResetStatistics();
    Check(pool.StatisticsShots() == 0 && pool.ArmorHits() == 0, "Stats reset failed");
    muzzle.position = {3.5, .5, .8};
    pool.Spawn(engine.Data(), muzzle);
    advance(1200);
    Check(pool.StatisticsShots() == 1 && pool.ArmorHits() == 0, "Miss counted as hit");
    muzzle.position = {3.5, 0, .8};
    pool.Spawn(engine.Data(), muzzle);
    pool.ResetStatistics();
    advance(150);
    Check(pool.StatisticsShots() == 0 && pool.ArmorHits() == 0,
          "Old in-flight bullet polluted new epoch");
    pool.Spawn(engine.Data(), muzzle);
    advance(150);
    Check(pool.StatisticsShots() == 1 && pool.ArmorHits() == 1, "New epoch shot failed");
    std::cout << "PASS: pair radii/heights, outward tilt, spin/reverse, blue, K, light,\n"
                 "ballistic armor hit, repeated contacts, miss, reset with in-flight shots.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
