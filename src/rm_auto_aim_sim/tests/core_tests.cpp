#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "rm_auto_aim_sim/engine.hpp"

namespace aim = rm_auto_aim_sim;

void Check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
void Advance(aim::Engine& engine, double seconds) {
  const int steps = static_cast<int>(std::llround(seconds / engine.GetConfig().physics_timestep));
  for (int index = 0; index < steps; ++index) engine.Step();
}

int main(int argc, char** argv) {
  try {
    Check(argc == 2, "Pass sim.yaml");
    auto config = aim::Config::Load(argv[1]);
    config.projectile_pool_size = 16;
    aim::Engine engine(config);
    const auto initial = engine.State();
    mjtNum optical_rotation[9], muzzle_rotation[9];
    mju_quat2Mat(optical_rotation, initial.optical.quaternion.data());
    mju_quat2Mat(muzzle_rotation, initial.muzzle.quaternion.data());
    Check(std::abs(optical_rotation[2] - 1.0) < 1e-12 &&
              std::abs(optical_rotation[3] + 1.0) < 1e-12 &&
              std::abs(optical_rotation[7] + 1.0) < 1e-12,
          "Optical axes must be right=-Y, down=-Z, forward=+X");
    Check(std::abs(muzzle_rotation[0] - 1.0) < 1e-12, "Muzzle axis incorrect");

    engine.Mailbox().SetYaw(1.0);
    engine.Mailbox().SetFire(true);
    Advance(engine, 0.05);
    Check(engine.State().yaw == 0 && engine.State().shot_count == 0,
          "Manual mode must ignore ROS commands");
    aim::ManualInput input;
    input.forward = 1;
    input.pitch_rate = 1;
    input.yaw_rate = 1;
    engine.SetManualInput(input);
    Advance(engine, 0.1);
    Check(engine.State().base.position[0] > initial.base.position[0] + 0.19,
          "Base translation failed");
    Check(engine.State().pitch > 0.14 && engine.State().yaw > 0.14,
          "Manual angular control failed");
    engine.SetMode(aim::ControlMode::kAuto);
    engine.Mailbox().SetPitch(0.2);
    engine.Mailbox().SetYaw(0.5);
    Advance(engine, 0.2);
    Check(
        std::abs(engine.State().pitch - 0.2) < 1e-10 && std::abs(engine.State().yaw - 0.5) < 1e-10,
        "AUTO angle tracking failed");
    engine.Mailbox().SetPitch(std::numeric_limits<double>::quiet_NaN());
    Advance(engine, 0.02);
    Check(std::abs(engine.State().pitch - 0.2) < 1e-10, "NaN command corrupted state");
    engine.Mailbox().SetPitch(100);
    engine.Mailbox().SetYaw(-100);
    Advance(engine, 1.0);
    Check(engine.State().pitch == config.pitch_max && engine.State().yaw == config.yaw_min,
          "Angle limits failed");
    engine.Mailbox().SetPitch(0);
    engine.Mailbox().SetYaw(0);
    Advance(engine, 1.0);

    // One shot's trajectory must use gravity, not an animated straight line.
    const auto muzzle = engine.State().muzzle.position;
    engine.Mailbox().SetFire(true);
    engine.Step();
    engine.Mailbox().SetFire(false);
    const int body = mj_name2id(engine.Model(), mjOBJ_BODY, "projectile_0");
    const int joint = engine.Model()->body_jntadr[body];
    const int position = engine.Model()->jnt_qposadr[joint];
    const int velocity = engine.Model()->jnt_dofadr[joint];
    Check(std::abs(engine.Data()->qvel[velocity] - 22.0) < 1e-8, "Launch speed incorrect");
    Check(engine.Data()->qvel[velocity + 2] < 0, "Gravity not applied");
    Advance(engine, 0.0995);
    Check(std::abs(engine.Data()->qpos[position] - (muzzle[0] + 2.2)) < 1e-4,
          "Horizontal ballistic motion incorrect");
    Check(std::abs(engine.Data()->qpos[position + 2] - (muzzle[2] - 0.04905)) < 0.001,
          "Vertical ballistic motion incorrect");
    Advance(engine, 1.0);
    auto hits = engine.TakeHitEvents();
    Check(!hits.empty(), "Floor/environment contact event missing");
    Advance(engine, 9.0);
    Check(engine.State().active_projectiles == 0, "Projectile lifecycle did not reclaim the slot");

    mj_resetData(engine.Model(), engine.Data());
    engine.Reset();
    engine.Mailbox().SetFire(true);
    Advance(engine, 0.8);
    Check(engine.State().shot_count == 12, "15 Hz continuous fire must produce 12 shots in 0.8 s");
    engine.Mailbox().SetFire(false);
    Advance(engine, 0.1);
    Check(engine.State().shot_count == 12, "False must stop continuous fire");
    engine.SetMode(aim::ControlMode::kManual);
    engine.SetMode(aim::ControlMode::kAuto);
    Advance(engine, 0.1);
    Check(engine.State().shot_count == 12, "Mode switch must clear fire latch");
    mj_resetData(engine.Model(), engine.Data());
    engine.Step();
    Check(engine.State().shot_count == 0, "Viewer reset must reset pool and fire latch");

    auto small_config = config;
    small_config.projectile_pool_size = 2;
    small_config.projectile_lifetime = 0.3;
    small_config.projectile_hit_lifetime = 0.1;
    aim::Engine small(small_config);
    small.SetMode(aim::ControlMode::kAuto);
    small.Mailbox().SetFire(true);
    const int nq = small.Model()->nq;
    Advance(small, 2.0);
    Check(small.State().active_projectiles <= 2 && small.State().pool_exhausted_count > 0 &&
              small.State().shot_count > 4 && small.Model()->nq == nq,
          "Bounded pool reuse/exhaustion handling failed");

    std::cout << "PASS: optical axes, manual translation, AUTO tracking, NaN rejection, limits,\n"
                 "22 m/s gravity trajectory, contacts, lifetime, 15 Hz firing, stop, mode/reset, "
                 "bounded pool\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
