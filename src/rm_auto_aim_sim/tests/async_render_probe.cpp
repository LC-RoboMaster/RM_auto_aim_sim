#include <GL/gl.h>

#include <cmath>
#include <iostream>
#include <stdexcept>

#include "rm_auto_aim_sim/camera_renderer.hpp"
#include "rm_auto_aim_sim/engine.hpp"

// Runs in the real EGL context supplied by run_render_test.py.
extern "C" int RunAsyncQueueProbe(const char* config_path) {
  namespace aim = rm_auto_aim_sim;
  try {
    auto config = aim::Config::Load(config_path);
    aim::Engine engine(config);
    aim::CameraRenderer renderer(engine.Model(), engine.CameraId(), config.camera_width,
                                 config.camera_height);
    aim::CameraMetrics metrics;
    aim::CameraFrame ready, reference;
    reference.state = engine.State();
    renderer.Render(engine.Data(), reference);

    auto submit = [&](double time) {
      aim::CameraFrame capture;
      capture.state = engine.State();
      capture.state.time = time;
      capture.capture_wall_time = 1000 + time;
      return renderer.RenderAsync(engine.Data(), std::move(capture), ready, false, false, 1, 0, 0,
                                  metrics);
    };
    auto complete = [&](bool completed) {
      // Synchronize only in the test, making GPU completion deterministic. The
      // production worker polls without waiting or submitting duplicate frames.
      glFinish();
      if (!completed && !renderer.CollectReady(ready, metrics))
        throw std::runtime_error("Paused capture cannot drain without another render");
    };
    if (renderer.CollectReady(ready, metrics))
      throw std::runtime_error("Empty queue returned an image");
    complete(submit(10));
    if (ready.state.time != 10 || ready.capture_wall_time != 1010 || ready.rgb != reference.rgb ||
        ready.fx != reference.fx || ready.fy != reference.fy)
      throw std::runtime_error("Drained capture changed RGB or capture metadata");
    for (int i = 0; i < 4; ++i)
      if (renderer.CollectReady(ready, metrics))
        throw std::runtime_error("Paused camera returned a duplicate frame");

    // Leave old captures in flight, then reset to time zero with a different K.
    // Old frames are now ready: a collector that runs before discarding them
    // would publish a pre-reset timestamp/calibration into the new timeline.
    submit(20);
    submit(21);
    glFinish();
    engine.SetIntrinsics({710, 790, 300, 210});
    complete(submit(0));
    if (ready.state.time != 0 || ready.capture_wall_time != 1000 ||
        std::abs(ready.fx - 710) > .001 || std::abs(ready.fy - 790) > .001 ||
        std::abs(ready.cx - 300) > .001 || std::abs(ready.cy - 210) > .001)
      throw std::runtime_error("Reset published a stale frame or calibration");
    if (renderer.CollectReady(ready, metrics))
      throw std::runtime_error("Pre-reset frame survived queue discard");

    // Exercise ring reuse after repeated resets, including a history rewind to
    // a nonzero time. Every capture must keep its own timestamp and K.
    for (double time : {1., 2., .5, 1.5, 0.}) {
      complete(submit(time));
      if (ready.state.time != time || ready.capture_wall_time != 1000 + time)
        throw std::runtime_error("PBO ring metadata changed after rewind");
      if (renderer.CollectReady(ready, metrics))
        throw std::runtime_error("Drained ring returned an extra frame");
    }
    // Place a pool bullet in front of the optical camera, with physics unchanged.
    // Verify actual RGB for native and HDR paths, and exact disappearance at zero.
    const int body = mj_name2id(engine.Model(), mjOBJ_BODY, "projectile_0");
    const int geom = mj_name2id(engine.Model(), mjOBJ_GEOM, "projectile_0_geom");
    const int joint = engine.Model()->body_jntadr[body];
    const int address = engine.Model()->jnt_qposadr[joint];
    double time = 100;
    auto image = [&](bool shader, double opacity) {
      renderer.SetProjectileOpacity(opacity);
      aim::CameraFrame capture;
      capture.state = engine.State();
      capture.state.time = time++;
      complete(renderer.RenderAsync(engine.Data(), std::move(capture), ready,
                                    shader, true, 1, .55, 4, metrics));
      return ready.rgb;
    };
    for (bool shader : {false, true}) {
      const auto background = image(shader, 1);
      const int camera = engine.CameraId();
      for (int axis = 0; axis < 3; ++axis)
        engine.Data()->qpos[address + axis] = engine.Data()->cam_xpos[3 * camera + axis] -
            .4 * engine.Data()->cam_xmat[9 * camera + 3 * axis + 2];
      mj_forward(engine.Model(), engine.Data());
      const auto hidden = image(shader, 0);
      const auto translucent = image(shader, .5);
      const auto visible = image(shader, 1);
      if (hidden != background || visible == hidden || translucent == hidden ||
          translucent == visible)
        throw std::runtime_error("Projectile opacity RGB endpoints/intermediate failed");
      if (engine.Model()->geom_rgba[4 * geom + 3] != 1 ||
          engine.Model()->geom_contype[geom] != 2)
        throw std::runtime_error("Camera opacity mutated physics/viewer model");
      engine.Data()->qpos[address] = 10000;
      engine.Data()->qpos[address + 1] = 10000;
      engine.Data()->qpos[address + 2] = 100;
      mj_forward(engine.Model(), engine.Data());
    }
    std::cout << "PASS: native/HDR bullet opacity 0/0.5/1 RGB, hidden shadows/glow, model isolation.\n";
    if (glGetError() != GL_NO_ERROR)
      throw std::runtime_error("Async queue probe left an OpenGL error");
    std::cout << "PASS: pause drain without duplicate capture, reset/rewind discard, "
                 "PBO ring reuse and per-capture K/time.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "ASYNC QUEUE FAIL: " << error.what() << '\n';
    return 1;
  }
}
