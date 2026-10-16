#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>

#include "rm_auto_aim_sim/camera_renderer.hpp"
#include "rm_auto_aim_sim/engine.hpp"

// Called by Python after EGL has made an OpenGL context current.
extern "C" int RunRenderProbe(const char* config_path, const char* output_path) {
  try {
    auto config = rm_auto_aim_sim::Config::Load(config_path);
    config.scene_path =
        (std::filesystem::path(config.scene_path).parent_path() / "arena.xml").string();
    config.target_enabled = false;
    config.camera_use_intrinsics = true;
    config.camera_fx = 800;
    config.camera_fy = 910;
    config.camera_cx = 301;
    config.camera_cy = 218;
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
    const double expected_x = frame.cx + frame.fx * point[0] / point[2];
    const double expected_y = frame.cy + frame.fy * point[1] / point[2];
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

// Check the imported user mesh lamps with zero environmental lighting, then
// switch colour and pose; the real render must observe all UI-mutable arrays.
extern "C" int RunLabRenderProbe(const char* config_path, const char* output_path) {
  try {
    auto config = rm_auto_aim_sim::Config::Load(config_path);
    config.projectile_pool_size = 8;
    rm_auto_aim_sim::Engine engine(config);
    auto settings = engine.Target()->Settings();
    settings.center = {-2, 0, 0};
    settings.h1 = settings.h2 = .99;
    settings.color = 0;
    engine.Target()->Configure(settings, engine.Data());
    engine.SetBrightness(0);
    rm_auto_aim_sim::CameraRenderer renderer(engine.Model(), engine.CameraId(), 640, 480);
    rm_auto_aim_sim::CameraFrame frame;
    auto pixels = [&]() {
      int red = 0, blue = 0;
      for (std::size_t i = 0; i < frame.rgb.size(); i += 3) {
        if (frame.rgb[i] > 180 && frame.rgb[i + 1] < 100 && frame.rgb[i + 2] < 100) ++red;
        if (frame.rgb[i + 2] > 180 && frame.rgb[i] < 100 && frame.rgb[i + 1] < 100) ++blue;
      }
      return std::make_pair(red, blue);
    };
    renderer.Render(engine.Data(), frame);
    if (pixels().first < 5) throw std::runtime_error("Red emissive mesh bars not visible");
    settings.color = 1;
    engine.Target()->Configure(settings, engine.Data());
    renderer.Render(engine.Data(), frame);
    if (pixels().second < 5 || pixels().first)
      throw std::runtime_error("Blue lamp switch not rendered");
    // Snapshot model must see colour, radius, lighting and calibration edits.
    mjModel* copy = mj_copyModel(nullptr, engine.Model());
    mjData* data = mj_makeData(copy);
    settings.color = 0;
    settings.r1 = .9;
    engine.Target()->Configure(settings, engine.Data());
    engine.SetIntrinsics({700, 850, 300, 220});
    engine.SetBrightness(.4);
    engine.CopyRenderSettings(copy);
    mj_copyData(data, copy, engine.Data());
    mj_forward(copy, data);
    {
      rm_auto_aim_sim::CameraRenderer worker(copy, engine.CameraId(), 640, 480);
      worker.Render(data, frame);
      if (pixels().first < 5 || std::abs(frame.fy - 850) > 1e-4 || std::abs(frame.cx - 300) > 1e-4)
        throw std::runtime_error("Worker snapshot did not observe UI edits");
    }
    mj_deleteData(data);
    mj_deleteModel(copy);
    std::ofstream image(output_path, std::ios::binary);
    image << "P6\n640 480\n255\n";
    image.write(reinterpret_cast<const char*>(frame.rgb.data()), frame.rgb.size());
    std::cout << "PASS: imported red/blue emission, radius edit, camera/light snapshot sync.\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "LAB RENDER FAIL: " << e.what() << '\n';
    return 1;
  }
}

// RGB luminance and background checks target the detector's threshold input,
// including a live style change on an existing independent camera context.
extern "C" int RunDarkRenderProbe(const char* config_path, const char* output_path) {
  namespace aim = rm_auto_aim_sim;
  try {
    auto config = aim::Config::Load(config_path);
    config.projectile_pool_size = 8;
    aim::Engine engine(config);
    auto settings = engine.Target()->Settings();
    settings.center = {-2, 0, 0};
    settings.h1 = settings.h2 = .99;
    settings.angle = 0;
    settings.angular_speed = 0;
    settings.color = 0;
    engine.Target()->Configure(settings, engine.Data());
    engine.SetBrightness(.03);
    engine.Environment().SetAlbedo(0);
    engine.Environment().SetDarkSky(true);
    engine.Target()->SetLightWhiteness(.65);
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> copy(mj_copyModel(nullptr, engine.Model()),
                                                             mj_deleteModel);
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(copy.get()), mj_deleteData);
    aim::CameraRenderer renderer(copy.get(), engine.CameraId(), config.camera_width,
                                 config.camera_height);
    aim::CameraFrame frame;
    auto render = [&]() {
      engine.CopyRenderSettings(copy.get());
      mj_copyData(data.get(), copy.get(), engine.Data());
      mj_forward(copy.get(), data.get());
      frame.state = engine.State();
      renderer.Render(data.get(), frame);
    };
    auto save = [&](const std::string& path) {
      std::ofstream image(path, std::ios::binary);
      image << "P6\n" << frame.width << ' ' << frame.height << "\n255\n";
      image.write(reinterpret_cast<const char*>(frame.rgb.data()), frame.rgb.size());
    };
    auto check = [&](int dominant, int other) {
      int white_lamps = 0;
      for (std::size_t i = 0; i < frame.rgb.size(); i += 3) {
        const double gray = .299 * frame.rgb[i] + .587 * frame.rgb[i + 1] + .114 * frame.rgb[i + 2];
        if (gray > 150 && frame.rgb[i + dominant] > frame.rgb[i + other] + 60) ++white_lamps;
      }
      if (white_lamps < 5)
        throw std::runtime_error("Lamp grayscale threshold/color difference failed");
      // Corners in this view are sky (top) and floor (bottom), away from targets.
      for (int row : {5, frame.height - 6})
        for (int col : {5, frame.width - 6})
          for (int ch = 0; ch < 3; ++ch)
            if (frame.rgb[(row * frame.width + col) * 3 + ch] > 2)
              throw std::runtime_error("Sky/floor is not black in sensor RGB");
    };
    render();
    check(0, 2);
    save(std::string(output_path) + ".red.ppm");
    settings = engine.Target()->Settings();
    settings.color = 1;
    engine.Target()->Configure(settings, engine.Data());
    render();
    check(2, 0);
    save(std::string(output_path) + ".blue.ppm");
    // Restoring sky on the same GL context must restore source sky pixels.
    engine.Environment().SetDarkSky(false);
    render();
    if (frame.rgb[5 * frame.width * 3 + 5 * 3 + 2] < 50)
      throw std::runtime_error("Sky toggle cannot restore original sky");
    std::cout
        << "PASS: black RGB sky/floor, red/blue gray>150 and color gap>60, live sky restore.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "DARK RENDER FAIL: " << error.what() << '\n';
    return 1;
  }
}

extern "C" int RunShaderRenderProbe(const char* config_path, const char* output_path) {
  namespace aim = rm_auto_aim_sim;
  try {
    auto config = aim::Config::Load(config_path);
    aim::Engine engine(config);
    auto settings = engine.Target()->Settings();
    settings.center = {-2, 0, 0};
    settings.h1 = settings.h2 = .99;
    settings.color = 0;
    engine.Target()->Configure(settings, engine.Data());
    aim::CameraRenderer renderer(engine.Model(), engine.CameraId(), config.camera_width,
                                 config.camera_height);
    aim::CameraFrame native, output;
    renderer.Render(engine.Data(), native);
    {
      std::ofstream image(std::string(output_path) + ".native.ppm", std::ios::binary);
      image << "P6\n" << native.width << ' ' << native.height << "\n255\n";
      image.write(reinterpret_cast<const char*>(native.rgb.data()), native.rgb.size());
    }
    aim::CameraMetrics metrics;
    auto capture = [&](bool shader) {
      for (int attempt = 0; attempt < 30; ++attempt) {
        aim::CameraFrame frame;
        frame.state.time = 17.0 + attempt * .01;
        frame.capture_wall_time = 123.0 + attempt;
        if (renderer.RenderAsync(engine.Data(), std::move(frame), output,
                                 shader, false, 1.0, .55, 4.0, metrics)) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      throw std::runtime_error("PBO queue did not complete a camera frame");
    };
    capture(false);
    if (output.rgb != native.rgb || output.fx != native.fx || output.fy != native.fy)
      throw std::runtime_error("Native async readback changed RGB orientation or intrinsics");
    // Drain the previously submitted native captures; then look for a shader
    // frame with its own capture timestamp (queue contents must not be mixed).
    bool got_shader = false;
    for (int attempt = 0; attempt < 40; ++attempt) {
      aim::CameraFrame frame;
      frame.state.time = 100 + attempt;
      frame.capture_wall_time = 1000 + attempt;
      if (renderer.RenderAsync(engine.Data(), std::move(frame), output,
                               true, false, 1.0, .7, 5.0, metrics) &&
          output.state.time >= 100) {
        got_shader = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (!got_shader || !std::isfinite(metrics.gpu_ms) || metrics.gpu_ms <= 0 ||
        output.rgb.size() != native.rgb.size() ||
        output.capture_wall_time != 1000 + (output.state.time - 100))
      throw std::runtime_error("Shader output or PBO capture metadata mismatched");
    std::size_t changed = 0;
    for (std::size_t i = 0; i < output.rgb.size(); ++i)
      changed += output.rgb[i] != native.rgb[i];
    if (changed < 1000) throw std::runtime_error("HDR shader did not change camera image");
    std::ofstream image(output_path, std::ios::binary);
    image << "P6\n" << output.width << ' ' << output.height << "\n255\n";
    image.write(reinterpret_cast<const char*>(output.rgb.data()), output.rgb.size());
    if (!image) throw std::runtime_error("Cannot save Shader probe image");
    const auto lit = output.rgb;
    bool got_unlit = false;
    for (int attempt = 0; attempt < 40; ++attempt) {
      aim::CameraFrame frame;
      frame.state.time = 200 + attempt;
      if (renderer.RenderAsync(engine.Data(), std::move(frame), output,
                               true, false, 1.0, 0.0, 0.0, metrics) &&
          output.state.time >= 200) {
        got_unlit = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (!got_unlit) throw std::runtime_error("Cannot capture no-lamp baseline");
    std::size_t lamp_changed = 0;
    for (std::size_t i = 0; i < lit.size(); ++i)
      lamp_changed += std::abs(int(lit[i]) - int(output.rgb[i])) >= 3;
    if (lamp_changed < 100)
      throw std::runtime_error("LED radiance/bloom control has no visible effect");
    std::cout << "PASS: async RGB orientation, HDR shader, capture metadata, estimated buffers "
              << metrics.gpu_estimate_mib << " MiB\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "SHADER RENDER FAIL: " << error.what() << '\n';
    return 1;
  }
}
