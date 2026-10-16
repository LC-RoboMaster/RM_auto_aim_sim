#include "rm_auto_aim_sim/viewer_hooks.hpp"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <thread>

#include "glfw_adapter.h"
#include "rm_auto_aim_sim/sensor_worker.hpp"
#include "simulate.h"

namespace rm_auto_aim_sim {
namespace {
Engine* engine = nullptr;
SensorWorker* sensor = nullptr;
int mode_ui = 0, view_ui = 0, direction_ui = 0, plots_ui = 1;
int control_section = -1, target_section = -1, camera_section = -1, stats_section = -1;
int performance_section = -1, performance_plots_ui = 0;
TargetSettings target_ui;
CameraIntrinsics camera_ui{};
double speed_ui = 0, brightness_ui = 1, albedo_ui = 1, lamp_white_ui = 0;
int dark_sky_ui = 0;
int shader_ui = 1, shadows_ui = 0;
double projectile_opacity_ui = 1;
double exposure_ui = 1, bloom_ui = .55, lamp_intensity_ui = 4;
bool camera_was_fixed = false;
mjvCamera saved_free_camera{};
mjvFigure angles_figure{}, rate_figure{};
mjvFigure timing_figure{}, process_figure{};
struct Sample {
  double time, pitch, yaw, rate;
};
std::deque<Sample> history;
struct PerfSample {
  double time, render, transfer, readback, publish, cpu, rss;
};
std::deque<PerfSample> perf_history;
double last_perf_sample = -1;
double last_sample = -1;
std::uint64_t epoch = 0;

bool FirstPerson(const mujoco::Simulate* sim) {
  return sim->cam.type == mjCAMERA_FIXED && sim->cam.fixedcamid == engine->CameraId();
}
void ChangeView(mujoco::Simulate* sim, bool update_ui = true) {
  if (view_ui == 1) {
    if (sim->cam.type == mjCAMERA_FREE) saved_free_camera = sim->cam;
    sim->cam.type = mjCAMERA_FIXED;
    sim->cam.fixedcamid = engine->CameraId();
    sim->camera = engine->CameraId() + 2;
    camera_was_fixed = true;
  } else {
    sim->cam = saved_free_camera;
    sim->cam.type = mjCAMERA_FREE;
    sim->camera = 0;
    camera_was_fixed = false;
  }
  if (!update_ui) return;  // Initial menus are resized and drawn by the caller.
  mjui_update(-1, -1, &sim->ui0, &sim->uistate, &sim->platform_ui->mjr_context());
  if (control_section >= 0)
    mjui_update(control_section, 1, &sim->ui1, &sim->uistate, &sim->platform_ui->mjr_context());
}
void Color(float* to, float r, float g, float b) {
  to[0] = r;
  to[1] = g;
  to[2] = b;
}
void Theme(mjUI& ui) {
  auto& c = ui.color;
  Color(c.master, .055f, .065f, .085f);
  Color(c.sectpane, .085f, .10f, .13f);
  Color(c.secttitle, .12f, .15f, .19f);
  Color(c.secttitle2, .12f, .15f, .19f);
  Color(c.separator, .10f, .12f, .16f);
  Color(c.separator2, .10f, .12f, .16f);
  Color(c.button, .13f, .26f, .34f);
  Color(c.select, .15f, .23f, .30f);
  Color(c.select2, .10f, .15f, .20f);
  Color(c.edit, .12f, .15f, .19f);
  Color(c.slider, .10f, .17f, .21f);
  Color(c.slider2, .22f, .72f, .72f);
  Color(c.check, .22f, .72f, .72f);
  Color(c.thumb, .25f, .33f, .39f);
  Color(c.fontactive, .89f, .94f, .97f);
  Color(c.fontinactive, .42f, .50f, .57f);
  ui.spacing.total = 330;
  ui.spacing.label = 130;
  ui.spacing.section = 8;
  ui.spacing.cornersect = 0;
  ui.spacing.cornersep = 0;
}
void Figure(mjvFigure& fig, const char* title, const char* label) {
  mjv_defaultFigure(&fig);
  std::snprintf(fig.title, sizeof(fig.title), "%s", title);
  std::snprintf(fig.xlabel, sizeof(fig.xlabel), "%s", label);
  std::strcpy(fig.yformat, "%.2f");
  fig.flg_legend = 1;
  fig.flg_extend = 0;
  fig.flg_symmetric = 0;
  fig.gridsize[0] = 4;
  fig.gridsize[1] = 4;
  fig.figurergba[0] = .055f;
  fig.figurergba[1] = .065f;
  fig.figurergba[2] = .085f;
  fig.figurergba[3] = .94f;
  Color(fig.panergba, .085f, .10f, .13f);
  Color(fig.gridrgb, .20f, .25f, .30f);
  Color(fig.linergb[0], .25f, .85f, .80f);
  Color(fig.linergb[1], .95f, .65f, .28f);
}
void Error(mujoco::Simulate* sim, const char* message) {
  auto& text = sim->ui1.sect[camera_section].item[8].multi;
  std::snprintf(text.name[0], mjMAXUINAME, "%s", message);
}
void RestoreCameraLook(mujoco::Simulate* sim) {
  const auto& config = engine->GetConfig();
  brightness_ui = config.lighting_brightness;
  dark_sky_ui = config.dark_sky;
  albedo_ui = config.environment_albedo;
  lamp_white_ui = config.target_light_whiteness;
  shader_ui = config.camera_shader;
  shadows_ui = config.camera_shadows;
  exposure_ui = config.camera_exposure;
  bloom_ui = config.camera_bloom;
  lamp_intensity_ui = config.camera_lamp_intensity;
  projectile_opacity_ui = config.camera_projectile_opacity;
  if (sensor) sensor->SetProjectileOpacity(projectile_opacity_ui);
  engine->SetBrightness(brightness_ui);
  engine->Environment().SetDarkSky(dark_sky_ui != 0);
  engine->Environment().SetAlbedo(albedo_ui);
  if (engine->Target()) {
    engine->Target()->SetLightWhiteness(lamp_white_ui);
    target_ui.light_whiteness = lamp_white_ui;
  }
  if (sensor)
    sensor->SetRendering(shader_ui != 0, shadows_ui != 0, exposure_ui,
                         bloom_ui, lamp_intensity_ui);
  Error(sim, "YAML camera look restored");
}
}  // namespace

void BindViewerEngine(Engine* value) { engine = value; }
void BindViewerSensor(SensorWorker* value) { sensor = value; }
bool HasViewerEngine() { return engine != nullptr; }

void AddViewerMenu(mujoco::Simulate* sim) {
  if (!engine) return;
  mode_ui = static_cast<int>(engine->State().mode);
  view_ui = engine->GetConfig().initial_view == "gimbal" ? 1 : 0;
  plots_ui = engine->GetConfig().show_charts;
  performance_plots_ui = engine->GetConfig().show_perf_plots;
  camera_was_fixed = false;
  saved_free_camera = sim->cam;
  if (saved_free_camera.type != mjCAMERA_FREE)
    mjv_defaultFreeCamera(engine->Model(), &saved_free_camera);
  if (engine->Target()) target_ui = engine->Target()->Settings();
  speed_ui = std::abs(target_ui.angular_speed);
  direction_ui = target_ui.angular_speed < 0 ? 1 : 0;
  camera_ui = engine->Intrinsics();
  brightness_ui = engine->Brightness();
  albedo_ui = engine->Environment().Albedo();
  dark_sky_ui = engine->Environment().DarkSky() ? 1 : 0;
  lamp_white_ui = target_ui.light_whiteness;
  const auto& camera_config = engine->GetConfig();
  shader_ui = camera_config.camera_shader;
  shadows_ui = camera_config.camera_shadows;
  exposure_ui = camera_config.camera_exposure;
  bloom_ui = camera_config.camera_bloom;
  lamp_intensity_ui = camera_config.camera_lamp_intensity;
  projectile_opacity_ui = camera_config.camera_projectile_opacity;
  history.clear();
  perf_history.clear();
  last_perf_sample = -1;
  last_sample = -1;
  epoch = engine->State().statistics_epoch;
  Figure(angles_figure, "Gimbal angles [rad]", "seconds ago");
  std::strcpy(angles_figure.linename[0], "pitch");
  std::strcpy(angles_figure.linename[1], "yaw");
  Figure(rate_figure, "Armor hit rate [%]", "seconds ago");
  std::strcpy(rate_figure.linename[0], "cumulative %");
  Figure(timing_figure, "Camera stages [ms]", "seconds ago");
  std::strcpy(timing_figure.linename[0], "GPU render");
  std::strcpy(timing_figure.linename[1], "GPU transfer");
  std::strcpy(timing_figure.linename[2], "PBO map/copy");
  std::strcpy(timing_figure.linename[3], "ROS publish");
  Color(timing_figure.linergb[2], .6f, .7f, .95f);
  Color(timing_figure.linergb[3], .9f, .45f, .75f);
  Figure(process_figure, "Process load", "seconds ago");
  std::strcpy(process_figure.linename[0], "CPU % (one core)");
  std::strcpy(process_figure.linename[1], "RSS MiB");
  // The laboratory controls replace the native right-hand joint sliders. The
  // native left panel stays available with Tab for debugging/solver settings.
  sim->ui1.nsect = 0;
  control_section = 0;
  target_section = 1;
  camera_section = 2;
  stats_section = 3;
  performance_section = 4;
  mjuiDef definition[] = {
      {mjITEM_SECTION, "AUTO AIM LAB", 1},
      {mjITEM_SELECT, "Control mode", 1, &mode_ui, "MANUAL\nAUTO (ROS2)"},
      {mjITEM_SELECT, "Camera view", 1, &view_ui, "Orbit\nGimbal"},
      {mjITEM_STATIC, "Angles [rad]", 1, nullptr, "Pitch 0.000\nYaw 0.000"},
      {mjITEM_STATIC, "Keys", 1, nullptr, "WASD move / QE up\nArrows / RMB aim\nF view / LMB fire"},
      {mjITEM_SECTION, "ARMOR TARGET", 1},
      {mjITEM_EDITNUM, "Center XYZ [m]", 1, target_ui.center.data(), "3"},
      {mjITEM_SLIDERNUM, "r1 [m]", 1, &target_ui.r1, "0.08 5"},
      {mjITEM_SLIDERNUM, "r2 [m]", 1, &target_ui.r2, "0.08 5"},
      {mjITEM_SLIDERNUM, "h1 [m]", 1, &target_ui.h1, "0.08 5"},
      {mjITEM_SLIDERNUM, "h2 [m]", 1, &target_ui.h2, "0.08 5"},
      {mjITEM_SELECT, "Color", 1, &target_ui.color, "Red\nBlue"},
      {mjITEM_SLIDERNUM, "Speed [rad/s]", 1, &speed_ui, "0 30"},
      {mjITEM_SELECT, "Direction", 1, &direction_ui, "CCW (+Z)\nCW (-Z)"},
      {mjITEM_BUTTON, "Reset heading", 1},
      {mjITEM_STATIC, "Geometry", 1, nullptr, "Pairs: 0,2 / 1,3\nZ + h1/h2 [m]\nNormal up 15 deg"},
      {mjITEM_SECTION, "CAMERA / LIGHT", 0},
      {mjITEM_EDITNUM, "fx / fy [px]", 1, &camera_ui.fx, "2 1 100000 1 100000"},
      {mjITEM_EDITNUM, "cx / cy [px]", 1, &camera_ui.cx, "2"},
      {mjITEM_BUTTON, "Apply intrinsics", 1},
      {mjITEM_SLIDERNUM, "Scene light", 1, &brightness_ui, "0 3"},
      {mjITEM_CHECKINT, "Black sky", 1, &dark_sky_ui},
      {mjITEM_SLIDERNUM, "Floor / scene color", 1, &albedo_ui, "0 1"},
      {mjITEM_SLIDERNUM, "LED white mix", 1, &lamp_white_ui, "0 1"},
      {mjITEM_STATIC, "Camera model", 1, nullptr, "Rectified pinhole RGB\nK alters projection"},
      {mjITEM_STATIC, "Status", 1, nullptr, "Ready"},
      {mjITEM_CHECKINT, "Shader camera", 1, &shader_ui},
      {mjITEM_CHECKINT, "Shadows", 1, &shadows_ui},
      {mjITEM_SLIDERNUM, "Exposure", 1, &exposure_ui, "0.1 8"},
      {mjITEM_SLIDERNUM, "Halo strength", 1, &bloom_ui, "0 2"},
      {mjITEM_SLIDERNUM, "LED HDR brightness", 1, &lamp_intensity_ui, "0 20"},
      {mjITEM_BUTTON, "Restore YAML look", 1},
      {mjITEM_SLIDERNUM, "ROS bullet opacity", 1, &projectile_opacity_ui, "0 1"},
      {mjITEM_SECTION, "HIT STATISTICS", 1},
      {mjITEM_STATIC, "Session", 1, nullptr, "Fired 0\nArmor hits 0\nHit rate 0.00%"},
      {mjITEM_STATIC, "Plate 0/1/2/3", 1, nullptr, "0 / 0 / 0 / 0"},
      {mjITEM_STATIC, "Pool", 1, nullptr, "Active 0\nDropped 0"},
      {mjITEM_BUTTON, "Reset statistics", 1},
      {mjITEM_CHECKINT, "Show charts", 1, &plots_ui},
      {mjITEM_SECTION, "PERFORMANCE", 1},
      {mjITEM_STATIC, "Camera", 1, nullptr, "Publish FPS 0\nCapture FPS 0\nLatency 0 ms\nGPU drops 0\nPublish drops 0"},
      {mjITEM_STATIC, "Stages", 1, nullptr, "Snapshot 0 ms\nScene 0 ms\nGPU render 0 ms\nRender submit 0 ms"},
      {mjITEM_STATIC, "Output", 1, nullptr, "GPU transfer 0 ms\nPBO map/copy 0 ms\nROS publish 0 ms"},
      {mjITEM_STATIC, "Process", 1, nullptr, "CPU 0% (100%=1 core)\nRSS 0 MiB\nRender buffers ~0 MiB"},
      {mjITEM_CHECKINT, "Show perf plots", 1, &performance_plots_ui},
      {mjITEM_END}};
  mjui_add(&sim->ui1, definition);
  Theme(sim->ui0);
  Theme(sim->ui1);
  sim->ui0_enable = 0;
  sim->ui1_enable = 1;
  sim->info = 0;
  // Hitbox proxies are invisible in the camera and normal viewer, but still collide.
  sim->opt.geomgroup[3] = 0;
  std::snprintf(sim->ui1.sect[target_section].item[9].multi.name[2], mjMAXUINAME,
                "Normal up %.1f deg", target_ui.tilt * 180.0 / mjPI);
  ChangeView(sim, false);
}

bool HandleViewerMenu(mujoco::Simulate* sim, const mjuiItem* item) {
  if (!engine || !item) return false;
  try {
    if (item->sectionid == control_section) {
      if (item->itemid == 0) engine->SetMode(static_cast<ControlMode>(mode_ui));
      if (item->itemid == 1) ChangeView(sim);
    } else if (item->sectionid == target_section && engine->Target()) {
      target_ui.angle = item->itemid == 8 ? 0 : engine->Target()->Settings().angle;
      target_ui.angular_speed = direction_ui ? -speed_ui : speed_ui;
      engine->Target()->Configure(target_ui, engine->Data());
    } else if (item->sectionid == camera_section) {
      if (item->itemid == 2) {
        engine->SetIntrinsics(camera_ui);
        Error(sim, "Intrinsics applied");
      }
      if (item->itemid == 3) engine->SetBrightness(brightness_ui);
      if (item->itemid == 4) engine->Environment().SetDarkSky(dark_sky_ui != 0);
      if (item->itemid == 5) engine->Environment().SetAlbedo(albedo_ui);
      if (item->itemid == 6 && engine->Target()) {
        engine->Target()->SetLightWhiteness(lamp_white_ui);
        target_ui.light_whiteness = lamp_white_ui;
      }
      if (item->itemid >= 9 && item->itemid <= 13 && sensor)
        sensor->SetRendering(shader_ui != 0, shadows_ui != 0, exposure_ui,
                             bloom_ui, lamp_intensity_ui);
      if (item->itemid == 14) RestoreCameraLook(sim);
      if (item->itemid == 15 && sensor)
        sensor->SetProjectileOpacity(projectile_opacity_ui);
    } else if (item->sectionid == stats_section) {
      if (item->itemid == 3) engine->ResetStatistics();
    } else
      return false;
  } catch (const std::exception&) {
    Error(sim, "Invalid value; edit again");
  }
  mjui_update(-1, -1, &sim->ui1, &sim->uistate, &sim->platform_ui->mjr_context());
  return true;
}

bool HandleViewerEvent(mujoco::Simulate* sim, mjuiState* state) {
  if (!engine) return false;
  // The native left panel has a global Backspace reset shortcut, including
  // while hidden. Give an active right-hand editor exclusive keyboard focus.
  if (state->type == mjEVENT_KEY && sim->ui1.editsect > 0) {
    auto* item = mjui_event(&sim->ui1, state, &sim->platform_ui->mjr_context());
    HandleViewerMenu(sim, item);
    return true;
  }
  if (state->mouserect != 3 && state->dragrect != 3) return false;
  if (state->type == mjEVENT_KEY && (state->key == 'F' || state->key == 'f')) {
    view_ui = FirstPerson(sim) ? 0 : 1;
    ChangeView(sim);
    return true;
  }
  if (state->type == mjEVENT_KEY && engine->State().mode == ControlMode::kManual &&
      (state->key == mjKEY_LEFT || state->key == mjKEY_RIGHT || state->key == mjKEY_UP ||
       state->key == mjKEY_DOWN || state->key == 'W' || state->key == 'A' || state->key == 'S' ||
       state->key == 'D' || state->key == 'Q' || state->key == 'E'))
    return true;
  if (!FirstPerson(sim) || engine->State().mode != ControlMode::kManual) return false;
  if (state->type == mjEVENT_MOVE && state->right) {
    const double sensitivity = engine->GetConfig().mouse_sensitivity;
    engine->AddManualAngles(state->dy * sensitivity, -state->dx * sensitivity);
    return true;
  }
  return (state->type == mjEVENT_PRESS || state->type == mjEVENT_RELEASE) &&
         state->button == mjBUTTON_LEFT;
}

void UpdateViewerInput(mujoco::Simulate* sim) {
  if (!engine) return;
  const auto state = engine->State();
  auto* adapter = dynamic_cast<mujoco::GlfwAdapter*>(sim->platform_ui.get());
  if (!adapter) return;
  auto* window = adapter->ApplicationWindow();
  const bool focused = glfwGetWindowAttrib(window, GLFW_FOCUSED) == GLFW_TRUE;
  auto key = [&](int code) {
    return focused && glfwGetKey(window, code) == GLFW_PRESS ? 1.0 : 0.0;
  };
  ManualInput input;
  if (focused && sim->uistate.mouserect == 3 && state.mode == ControlMode::kManual) {
    input.forward = key(GLFW_KEY_W) - key(GLFW_KEY_S);
    input.left = key(GLFW_KEY_A) - key(GLFW_KEY_D);
    input.up = key(GLFW_KEY_E) - key(GLFW_KEY_Q);
    input.pitch_rate = key(GLFW_KEY_UP) - key(GLFW_KEY_DOWN);
    input.yaw_rate = key(GLFW_KEY_LEFT) - key(GLFW_KEY_RIGHT);
    input.fire =
        FirstPerson(sim) && glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  }
  engine->SetManualInput(input);
  mode_ui = static_cast<int>(state.mode);
  view_ui = FirstPerson(sim) ? 1 : 0;
  if (!view_ui && !camera_was_fixed) saved_free_camera = sim->cam;
  camera_was_fixed = view_ui;
  if (control_section < 0 || control_section >= sim->ui1.nsect) return;
  auto& angles = sim->ui1.sect[control_section].item[2].multi;
  std::snprintf(angles.name[0], mjMAXUINAME, "Pitch %.3f", state.pitch);
  std::snprintf(angles.name[1], mjMAXUINAME, "Yaw %.3f", state.yaw);
  auto& totals = sim->ui1.sect[stats_section].item[0].multi;
  std::snprintf(totals.name[0], mjMAXUINAME, "Fired %llu",
                (unsigned long long)state.statistics_shots);
  std::snprintf(totals.name[1], mjMAXUINAME, "Armor hits %llu",
                (unsigned long long)state.armor_hits);
  std::snprintf(totals.name[2], mjMAXUINAME, "Hit rate %.2f%%", state.hit_rate * 100);
  auto& plates = sim->ui1.sect[stats_section].item[1].multi;
  std::snprintf(plates.name[0], mjMAXUINAME, "%llu / %llu / %llu / %llu",
                (unsigned long long)state.plate_hits[0], (unsigned long long)state.plate_hits[1],
                (unsigned long long)state.plate_hits[2], (unsigned long long)state.plate_hits[3]);
  auto& pool = sim->ui1.sect[stats_section].item[2].multi;
  std::snprintf(pool.name[0], mjMAXUINAME, "Active %d", state.active_projectiles);
  std::snprintf(pool.name[1], mjMAXUINAME, "Dropped %llu",
                (unsigned long long)state.pool_exhausted_count);
  if (sensor) {
    const auto m = sensor->Metrics();
    auto& camera = sim->ui1.sect[performance_section].item[0].multi;
    std::snprintf(camera.name[0], mjMAXUINAME, "Publish FPS %.1f", m.fps);
    std::snprintf(camera.name[1], mjMAXUINAME, "Capture FPS %.1f", m.capture_fps);
    std::snprintf(camera.name[2], mjMAXUINAME, "Latency %.1f ms", m.latency_ms);
    std::snprintf(camera.name[3], mjMAXUINAME, "GPU drops %llu",
                  (unsigned long long)m.dropped_frames);
    std::snprintf(camera.name[4], mjMAXUINAME, "Publish drops %llu",
                  (unsigned long long)m.publish_dropped_frames);
    auto& stages = sim->ui1.sect[performance_section].item[1].multi;
    std::snprintf(stages.name[0], mjMAXUINAME, "Snapshot %.2f ms", m.snapshot_ms);
    std::snprintf(stages.name[1], mjMAXUINAME, "Scene %.2f ms", m.scene_ms);
    std::snprintf(stages.name[2], mjMAXUINAME, "GPU render %.2f ms", m.gpu_ms);
    if (!m.gpu_timer_valid) std::snprintf(stages.name[2], mjMAXUINAME, "GPU render n/a");
    std::snprintf(stages.name[3], mjMAXUINAME, "Render submit %.2f ms", m.render_ms);
    auto& output = sim->ui1.sect[performance_section].item[2].multi;
    std::snprintf(output.name[0], mjMAXUINAME, "GPU transfer %.2f ms", m.gpu_transfer_ms);
    if (!m.gpu_timer_valid) std::snprintf(output.name[0], mjMAXUINAME, "GPU transfer n/a");
    std::snprintf(output.name[1], mjMAXUINAME, "PBO map/copy %.2f ms", m.readback_ms);
    std::snprintf(output.name[2], mjMAXUINAME, "ROS publish %.2f ms", m.publish_ms);
    auto& process = sim->ui1.sect[performance_section].item[3].multi;
    std::snprintf(process.name[0], mjMAXUINAME, "CPU %.1f%% (100%%=1 core)", m.cpu_percent);
    std::snprintf(process.name[1], mjMAXUINAME, "RSS %.1f MiB", m.rss_mib);
    std::snprintf(process.name[2], mjMAXUINAME, "Render buffers ~%.1f MiB", m.gpu_estimate_mib);
    for (int i = 0; i < 4; ++i)
      mjui_update(performance_section, i, &sim->ui1, &sim->uistate,
                  &sim->platform_ui->mjr_context());
    const double wall = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    if (last_perf_sample < 0 || wall - last_perf_sample >= .2) {
      perf_history.push_back({wall, m.gpu_ms, m.gpu_transfer_ms, m.readback_ms, m.publish_ms,
                              m.cpu_percent, m.rss_mib});
      last_perf_sample = wall;
      if (perf_history.size() > 75) perf_history.pop_front();
    }
  }
  // UI fields being edited retain their staging values. State text refreshes at display rate.
  mjui_update(control_section, 2, &sim->ui1, &sim->uistate, &sim->platform_ui->mjr_context());
  for (int i = 0; i < 3; ++i)
    mjui_update(stats_section, i, &sim->ui1, &sim->uistate, &sim->platform_ui->mjr_context());
  if (state.time < last_sample || state.statistics_epoch != epoch) {
    if (state.time < last_sample && engine->Target()) {
      target_ui = engine->Target()->Settings();
      lamp_white_ui = target_ui.light_whiteness;
      mjui_update(camera_section, 6, &sim->ui1, &sim->uistate, &sim->platform_ui->mjr_context());
      speed_ui = std::abs(target_ui.angular_speed);
      direction_ui = target_ui.angular_speed < 0 ? 1 : 0;
      mjui_update(target_section, -1, &sim->ui1, &sim->uistate, &sim->platform_ui->mjr_context());
    }
    history.clear();
    last_sample = -1;
    epoch = state.statistics_epoch;
  }
  if (last_sample < 0 || state.time - last_sample >= .05) {
    history.push_back({state.time, state.pitch, state.yaw, state.hit_rate * 100});
    last_sample = state.time;
    if (history.size() > 300) history.pop_front();
  }
}

void ApplyViewerEnvironment(mujoco::Simulate* sim) {
  if (engine) engine->Environment().ConfigureScene(&sim->scn);
}

void DrawViewerPlots(mujoco::Simulate* sim, mjrRect viewport) {
  if (!engine || viewport.width < 420 || viewport.height < 360) return;
  if (plots_ui && !history.empty()) {
  // Figures are populated only by the viewer thread; history is bounded to 15 seconds at 20 Hz.
  const double now = history.back().time;
  double low = -.5, high = .5;
  for (std::size_t i = 0; i < history.size(); ++i) {
    const auto& s = history[i];
    angles_figure.linedata[0][2 * i] = s.time - now;
    angles_figure.linedata[0][2 * i + 1] = s.pitch;
    angles_figure.linedata[1][2 * i] = s.time - now;
    angles_figure.linedata[1][2 * i + 1] = s.yaw;
    rate_figure.linedata[0][2 * i] = s.time - now;
    rate_figure.linedata[0][2 * i + 1] = s.rate;
    low = std::min({low, s.pitch - .1, s.yaw - .1});
    high = std::max({high, s.pitch + .1, s.yaw + .1});
  }
  angles_figure.linepnt[0] = angles_figure.linepnt[1] = history.size();
  rate_figure.linepnt[0] = history.size();
  angles_figure.range[0][0] = rate_figure.range[0][0] = -15;
  angles_figure.range[0][1] = rate_figure.range[0][1] = 0;
  angles_figure.range[1][0] = low;
  angles_figure.range[1][1] = high;
  rate_figure.range[1][0] = 0;
  rate_figure.range[1][1] = 100;
  const int width = std::min(360, viewport.width / 2), height = std::min(175, viewport.height / 4);
  const mjrRect top{viewport.left + 12, viewport.bottom + 12 + height + 8, width, height};
  const mjrRect bottom{viewport.left + 12, viewport.bottom + 12, width, height};
  mjr_figure(top, &angles_figure, &sim->platform_ui->mjr_context());
  mjr_figure(bottom, &rate_figure, &sim->platform_ui->mjr_context());
  }
  if (performance_plots_ui && !perf_history.empty()) {
    const double now = perf_history.back().time;
    double timing_max = 1, process_max = 10;
    for (std::size_t i = 0; i < perf_history.size(); ++i) {
      const auto& s = perf_history[i];
      const double values[4] = {s.render, s.transfer, s.readback, s.publish};
      for (int line = 0; line < 4; ++line) {
        timing_figure.linedata[line][2 * i] = s.time - now;
        timing_figure.linedata[line][2 * i + 1] = values[line];
      }
      for (int line = 0; line < 2; ++line) {
        process_figure.linedata[line][2 * i] = s.time - now;
        process_figure.linedata[line][2 * i + 1] = line == 0 ? s.cpu : s.rss;
      }
      timing_max = std::max({timing_max, s.render, s.transfer, s.readback, s.publish});
      process_max = std::max({process_max, s.cpu, s.rss});
    }
    for (int i = 0; i < 4; ++i) timing_figure.linepnt[i] = perf_history.size();
    for (int i = 0; i < 2; ++i) process_figure.linepnt[i] = perf_history.size();
    for (auto* fig : {&timing_figure, &process_figure}) {
      fig->range[0][0] = -15;
      fig->range[0][1] = 0;
      fig->range[1][0] = 0;
    }
    timing_figure.range[1][1] = timing_max * 1.15;
    process_figure.range[1][1] = process_max * 1.15;
    const int width = std::min(360, viewport.width / 2);
    const int height = std::min(175, viewport.height / 4);
    const int left = viewport.left + viewport.width - width - 12;
    mjr_figure({left, viewport.bottom + 12 + height + 8, width, height},
               &timing_figure, &sim->platform_ui->mjr_context());
    mjr_figure({left, viewport.bottom + 12, width, height},
               &process_figure, &sim->platform_ui->mjr_context());
  }
}
void StepFromViewer() {
  if (engine) engine->Step();
}
void WaitForViewerFrame() {
  if (!engine || engine->GetConfig().viewer_max_fps == 0) return;
  using Clock = std::chrono::steady_clock;
  static auto deadline = Clock::now();
  deadline += std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(1.0 / engine->GetConfig().viewer_max_fps));
  if (deadline < Clock::now()) deadline = Clock::now();
  std::this_thread::sleep_until(deadline);
}
}  // namespace rm_auto_aim_sim
