#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>

#include "rm_auto_aim_sim/camera_renderer.hpp"
#include "rm_auto_aim_sim/engine.hpp"

// Called by Python after EGL has made an OpenGL context current.
extern "C" int RunRenderProbe(const char* config_path, const char* output_path) {
  try {
    auto config = rm_auto_aim_sim::Config::Load(config_path);
    rm_auto_aim_sim::Engine engine(config);
    rm_auto_aim_sim::CameraRenderer renderer(engine.Model(), engine.CameraId(), config.camera_width,
                                             config.camera_height);
    rm_auto_aim_sim::CameraFrame frame;
    renderer.Render(engine.Data(), frame);
    std::ofstream image(output_path, std::ios::binary);
    image << "P6\n" << frame.width << ' ' << frame.height << "\n255\n";
    image.write(reinterpret_cast<const char*>(frame.rgb.data()), frame.rgb.size());
    image.close();
    int red_pixels = 0, blue_pixels = 0;
    double blue_x = 0, blue_y = 0;
    for (std::size_t index = 0; index < frame.rgb.size(); index += 3) {
      if (frame.rgb[index] > 150 && frame.rgb[index + 1] < 100 && frame.rgb[index + 2] < 100)
        ++red_pixels;
      if (frame.rgb[index + 2] > 150 && frame.rgb[index] < 100 && frame.rgb[index + 1] < 100) {
        ++blue_pixels;
        blue_x += (index / 3) % frame.width;
        blue_y += (index / 3) / frame.width;
      }
    }
    if (!image || frame.width != config.camera_width || frame.height != config.camera_height ||
        red_pixels < 5 || blue_pixels < 5)
      throw std::runtime_error("RGB target visibility check failed");
    // Check a visible target against the published pinhole model and optical TF.
    const int target = mj_name2id(engine.Model(), mjOBJ_BODY, "blue_target");
    const auto optical = engine.State().optical;
    mjtNum difference[3], inverse[4], point[3];
    mju_sub3(difference, engine.Data()->xpos + 3 * target, optical.position.data());
    mju_negQuat(inverse, optical.quaternion.data());
    mju_rotVecQuat(point, difference, inverse);
    const double expected_x = (frame.width - 1) * 0.5 + frame.focal_length * point[0] / point[2];
    const double expected_y = (frame.height - 1) * 0.5 + frame.focal_length * point[1] / point[2];
    if (std::abs(blue_x / blue_pixels - expected_x) > 2.0 ||
        std::abs(blue_y / blue_pixels - expected_y) > 2.0) {
      throw std::runtime_error("Rendered projection disagrees with CameraInfo/optical frame");
    }
    engine.SetMode(rm_auto_aim_sim::ControlMode::kAuto);
    engine.Mailbox().SetFire(true);
    engine.Step();
    engine.Mailbox().SetFire(false);
    for (int index = 0; index < 59; ++index) engine.Step();
    renderer.Render(engine.Data(), frame);
    int green_pixels = 0;
    for (std::size_t index = 0; index < frame.rgb.size(); index += 3) {
      if (frame.rgb[index + 1] > 180 && frame.rgb[index] < 100 && frame.rgb[index + 2] < 100)
        ++green_pixels;
    }
    if (green_pixels < 2) throw std::runtime_error("Green projectile emission is not visible");
    const auto start = std::chrono::steady_clock::now();
    for (int index = 0; index < 3; ++index) renderer.Render(engine.Data(), frame);
    const double seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    std::cout << "PASS: RGB8 size, target projection, green emissive projectile, camera render. "
                 "Unthrottled render FPS="
              << 3 / seconds << " (this machine only)\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "RENDER FAIL: " << error.what() << '\n';
    return 1;
  }
}
