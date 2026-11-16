#include <GL/gl.h>

#include <fstream>
#include <iostream>
#include <stdexcept>

#include "rm_auto_aim_sim/camera_renderer.hpp"
#include "rm_auto_aim_sim/engine.hpp"

extern "C" int RunOutpostRenderProbe(const char* config_path, const char* output_path) {
  namespace aim = rm_auto_aim_sim;
  try {
    auto config = aim::Config::Load(config_path);
    config.armor_enabled = false;
    config.outpost_enabled = true;
    config.outpost_x = -2; config.outpost_y = 0;
    config.outpost_initial_angle = mjPI;
    config.projectile_pool_size = 4;
    aim::Engine engine(config);
    engine.SetBrightness(0);
    aim::CameraRenderer renderer(engine.Model(), engine.CameraId(), config.camera_width, config.camera_height);
    aim::CameraFrame frame;
    frame.state = engine.State(); renderer.Render(engine.Data(), frame);
    std::size_t red_pixels = 0;
    for (std::size_t i = 0; i < frame.rgb.size(); i += 3)
      if (frame.rgb[i] > 100 && frame.rgb[i] > frame.rgb[i + 1] + 50 &&
          frame.rgb[i] > frame.rgb[i + 2] + 50) ++red_pixels;
    if (red_pixels < 8) throw std::runtime_error("Outpost red lamps did not reach sensor RGB");
    const auto red = frame.rgb;
    auto setting = engine.Outpost()->Settings(); setting.color = 1;
    engine.Outpost()->Configure(setting, engine.Data());
    frame.state = engine.State(); renderer.Render(engine.Data(), frame);
    if (frame.rgb == red) throw std::runtime_error("Outpost UI/model colour edit did not reach RGB");
    aim::CameraMetrics metrics; aim::CameraFrame ready;
    bool completed = renderer.RenderAsync(engine.Data(), frame, ready, true, false, 1, .55, 4, metrics);
    glFinish();
    if (!completed && !renderer.CollectReady(ready, metrics))
      throw std::runtime_error("Outpost HDR frame missing");
    std::ofstream output(output_path, std::ios::binary);
    output << "P6\n" << ready.width << ' ' << ready.height << "\n255\n";
    output.write(reinterpret_cast<const char*>(ready.rgb.data()), ready.rgb.size());
    if (!output) throw std::runtime_error("Cannot write outpost image");
    if (glGetError() != GL_NO_ERROR) throw std::runtime_error("Outpost render left a GL error");
    std::cout << "PASS: three-plate outpost RGB lamps, live colour edit, HDR/PBO output.\n";
    return 0;
  } catch (const std::exception& e) { std::cerr << "OUTPOST RENDER FAIL: " << e.what() << '\n'; return 1; }
}
