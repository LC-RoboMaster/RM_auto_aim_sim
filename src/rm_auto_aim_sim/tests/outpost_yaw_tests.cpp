#include <cmath>
#include <iostream>
#include <stdexcept>

#include "rm_auto_aim_sim/engine.hpp"

namespace aim = rm_auto_aim_sim;
namespace {
void Check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
bool Near(double a, double b) { return std::abs(a - b) < 1e-7; }
void Step(aim::Engine& engine, int count) { while (count-- > 0) engine.Step(); }
}
int main(int argc, char** argv) {
  try {
    Check(argc == 2, "Pass fixture config");
    auto config = aim::Config::Load(argv[1]);
    config.projectile_pool_size = 4;
    config.outpost_enabled = true;
    config.yaw_continuous = true;
    config.outpost_x = 2;
    config.outpost_y = 1;
    config.outpost_initial_angle = .37;
    aim::Engine engine(config);
    Check(engine.Target() && engine.Outpost(), "Both targets must coexist");
    auto* outpost = engine.Outpost();
    auto setting = outpost->Settings();
    const double heights[] = {1.114, 1.216, 1.318};
    for (int i = 0; i < 3; ++i) {
      int body = mj_name2id(engine.Model(), mjOBJ_BODY, ("outpost_" + std::to_string(i)).c_str());
      const auto* pos = engine.Data()->xpos + 3 * body;
      const auto* rot = engine.Data()->xmat + 9 * body;
      const double phase = .37 + i * 2 * mjPI / 3;
      Check(Near(pos[0], 2 + .275 * std::cos(phase)) &&
            Near(pos[1], 1 + .275 * std::sin(phase)) && Near(pos[2], heights[i]),
            "Outpost 550 mm circle / 120 deg / centre heights incorrect");
      Check(Near(rot[0], std::cos(phase) * std::cos(mjPI / 12)) &&
            Near(rot[3], std::sin(phase) * std::cos(mjPI / 12)) &&
            Near(rot[6], -std::sin(mjPI / 12)), "Number face must point outward/down 15 deg");
      const int number = mj_name2id(engine.Model(), mjOBJ_GEOM,
                                   ("outpost_" + std::to_string(i) + "_number").c_str());
      Check(engine.Model()->geom_pos[3 * number] > 0, "Digit texture is behind front face");
    }
    const auto armor_start = engine.Target()->Settings();
    setting.angular_speed = 2;
    outpost->Configure(setting, engine.Data());
    Step(engine, 400);
    Check(Near(outpost->Settings().angle, .77), "Outpost CCW speed incorrect");
    setting = outpost->Settings(); setting.angular_speed = -2;
    setting.center[2] = .1; setting.r1 = .4; setting.h3 = 1.5;
    outpost->Configure(setting, engine.Data());
    Step(engine, 400);
    Check(Near(outpost->Settings().angle, .37) &&
          Near(engine.Target()->Settings().angle, armor_start.angle), "Independent reverse rotation failed");
    const int third = mj_name2id(engine.Model(), mjOBJ_BODY, "outpost_2");
    Check(Near(engine.Data()->xpos[3 * third + 2], 1.6), "Live outpost height edit failed");
    const int armor_lamp = mj_name2id(engine.Model(), mjOBJ_MATERIAL, "target_glow");
    const float original_red = engine.Model()->mat_rgba[4 * armor_lamp];
    setting.color = 1; setting.light_whiteness = .4;
    outpost->Configure(setting, engine.Data());
    Check(engine.Model()->mat_rgba[4 * armor_lamp] == original_red, "Outpost changed armor material");
    mjModel* copied = mj_copyModel(nullptr, engine.Model());
    engine.CopyRenderSettings(copied);
    Check(Near(copied->body_pos[3 * third + 2], 1.5), "Sensor snapshot missed outpost edits");
    mj_deleteModel(copied);
    outpost->Reset(engine.Data());
    Check(Near(outpost->Settings().h3, 1.318) && Near(outpost->Settings().angle, .37),
          "Outpost reset did not restore startup mechanics");

    // Fire toward outward plate 0 at angle zero, from outside its orbit.
    setting = outpost->Settings(); setting.angle = 0; setting.angular_speed = 0;
    outpost->Configure(setting, engine.Data());
    aim::ProjectilePool pool(config, engine.Model()); pool.Reset(engine.Data());
    aim::Pose muzzle; muzzle.position = {2.8, 1, 1.114}; muzzle.quaternion = {0, 0, 0, 1};
    Check(pool.Spawn(engine.Data(), muzzle), "Cannot spawn outpost test shot");
    for (int i = 0; i < 120; ++i) {
      pool.BeforeStep(engine.Data()); outpost->Apply(engine.Data());
      engine.Target()->Apply(engine.Data()); mj_step(engine.Model(), engine.Data());
      pool.AfterStep(engine.Data());
    }
    const auto hits = pool.TakeHitEvents();
    Check(pool.ArmorHits() == 1 && pool.OutpostPlateHits()[0] == 1 && pool.PlateHits()[0] == 0 &&
          hits.size() == 1 && hits[0].target_type == "outpost" && hits[0].armor_index == 0,
          "Outpost contact identity / counters incorrect");
    pool.ResetStatistics();
    Check(pool.OutpostPlateHits()[0] == 0, "Outpost counters did not reset");

    mj_resetData(engine.Model(), engine.Data()); engine.Reset();
    const int yaw_joint = mj_name2id(engine.Model(), mjOBJ_JOINT, config.model_yaw_joint.c_str());
    Check(!engine.Model()->jnt_limited[yaw_joint], "Continuous yaw still has hard stops");
    engine.SetMode(aim::ControlMode::kAuto);
    engine.Mailbox().SetYaw(179 * mjPI / 180); Step(engine, 1000);
    engine.Mailbox().SetYaw(-179 * mjPI / 180);
    engine.Step();
    Check(engine.State().yaw_velocity > 0 && engine.State().yaw_velocity <= config.servo_max_speed + 1e-9,
          "+179 to -179 must take positive 2-degree route");
    Step(engine, 30);
    Check(Near(engine.State().yaw, -179 * mjPI / 180), "Positive yaw seam failed");
    engine.Mailbox().SetYaw(179 * mjPI / 180); engine.Step();
    Check(engine.State().yaw_velocity < 0, "Reverse seam must take negative 2-degree route");
    Step(engine, 30);
    Check(Near(engine.State().yaw, 179 * mjPI / 180), "Negative yaw seam failed");
    engine.Mailbox().SetYaw(4 * mjPI + .2); Step(engine, 1000);
    Check(Near(engine.State().yaw, .2), "Multi-turn-equivalent command failed");
    engine.SetMode(aim::ControlMode::kManual);
    aim::ManualInput input; input.yaw_rate = 1; engine.SetManualInput(input);
    bool crossed = false;
    for (int i = 0; i < 18000; ++i) {
      double before = engine.State().yaw; engine.Step(); const auto now = engine.State();
      crossed |= before > 3 && now.yaw < -3;
      Check(now.yaw >= -mjPI && now.yaw <= mjPI && now.yaw_velocity > 0 &&
            std::abs(now.yaw_velocity - config.manual_angular_speed) < 1e-7,
            "Manual full-turn yaw / seam velocity incorrect");
    }
    Check(crossed && engine.Data()->qpos[engine.Model()->jnt_qposadr[yaw_joint]] > 4 * mjPI,
          "Manual yaw failed to complete multiple full rotations");
    engine.SetManualInput({}); engine.SetYawContinuous(false);
    engine.AddManualAngles(0, 100); Step(engine, 1000);
    Check(engine.Model()->jnt_limited[yaw_joint] && Near(engine.State().yaw, config.yaw_max),
          "Optional hard-stop yaw mode failed");
    engine.SetYawContinuous(true);
    mj_resetData(engine.Model(), engine.Data()); engine.Reset();
    Check(Near(engine.State().yaw, 0), "Yaw reset failed");
    config.armor_enabled = false; config.outpost_enabled = false;
    aim::Engine neither(config);
    Check(!neither.Target() && !neither.Outpost(), "Disabled target modules were compiled");
    std::cout << "PASS: outpost mechanics, outward/down digits, independent spin/materials, snapshot/reset/hits, "
                 "AUTO shortest paths, manual multiple turns, wrapped feedback, hard-stop fallback.\n";
    return 0;
  } catch (const std::exception& e) { std::cerr << "OUTPOST/YAW FAIL: " << e.what() << '\n'; return 1; }
}
