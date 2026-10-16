#include "rm_auto_aim_sim/camera_renderer.hpp"

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include "rm_auto_aim_sim/camera_model.hpp"
#include "rm_auto_aim_sim/environment.hpp"
#include "rm_auto_aim_sim/hdr_pipeline.hpp"

namespace rm_auto_aim_sim {
namespace {
using Clock = std::chrono::steady_clock;
double Ms(Clock::time_point start, Clock::time_point end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}
void CheckGl(const char* stage) {
  const GLenum error = glGetError();
  if (error != GL_NO_ERROR)
    throw std::runtime_error(std::string("Camera OpenGL error after ") + stage +
                             ": " + std::to_string(static_cast<unsigned>(error)));
}
}  // namespace

struct CameraRenderer::Async {
  struct Slot {
    GLuint pbo = 0;
    GLuint timer = 0;
    GLsync fence = nullptr;
    CameraFrame frame;
  };
  HdrPipeline pipeline;
  std::array<Slot, 3> slots;
  std::size_t read = 0, write = 0;
  std::size_t bytes;
  Async(int width, int height, const mjrContext& context)
      : pipeline(width, height, context), bytes(static_cast<std::size_t>(width) * height * 3) {
    for (auto& slot : slots) {
      glGenBuffers(1, &slot.pbo);
      glGenQueries(1, &slot.timer);
      glBindBuffer(GL_PIXEL_PACK_BUFFER, slot.pbo);
      glBufferData(GL_PIXEL_PACK_BUFFER, bytes, nullptr, GL_STREAM_READ);
    }
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
  }
  ~Async() {
    for (auto& slot : slots) {
      if (slot.fence) glDeleteSync(slot.fence);
      glDeleteQueries(1, &slot.timer);
      glDeleteBuffers(1, &slot.pbo);
    }
  }
  bool Collect(CameraFrame& ready, CameraMetrics& metrics) {
    auto& slot = slots[read];
    if (!slot.fence) return false;
    const GLenum result = glClientWaitSync(slot.fence, 0, 0);
    if (result == GL_TIMEOUT_EXPIRED) return false;
    if (result == GL_WAIT_FAILED) throw std::runtime_error("Camera PBO fence wait failed");
    GLuint64 gpu_nanoseconds = 0;
    glGetQueryObjectui64v(slot.timer, GL_QUERY_RESULT, &gpu_nanoseconds);
    metrics.gpu_ms = gpu_nanoseconds / 1e6;
    const auto begin = Clock::now();
    glBindBuffer(GL_PIXEL_PACK_BUFFER, slot.pbo);
    void* pixels = glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0, bytes, GL_MAP_READ_BIT);
    if (!pixels) throw std::runtime_error("Camera PBO mapping failed");
    slot.frame.rgb.resize(bytes);
    std::memcpy(slot.frame.rgb.data(), pixels, bytes);
    glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glDeleteSync(slot.fence);
    slot.fence = nullptr;
    ready = std::move(slot.frame);
    read = (read + 1) % slots.size();
    metrics.readback_ms = Ms(begin, Clock::now());
    return true;
  }
};

CameraRenderer::CameraRenderer(const mjModel* model, int camera_id, int width, int height)
    : model_(model),
      camera_id_(camera_id),
      width_(width),
      height_(height),
      readback_(static_cast<std::size_t>(width) * height * 3) {
  mjv_defaultScene(&scene_);
  mjr_defaultContext(&context_);
  mjv_defaultOption(&option_);
  mjv_defaultCamera(&camera_);
  camera_.type = mjCAMERA_FIXED;
  camera_.fixedcamid = camera_id;
  mjv_makeScene(model, &scene_, model->ngeom + 100);
  mjr_makeContext(model, &context_, mjFONTSCALE_100);
  mjr_resizeOffscreen(width, height, &context_);
  mjr_setBuffer(mjFB_OFFSCREEN, &context_);
  if (context_.currentBuffer != mjFB_OFFSCREEN) {
    mjv_freeScene(&scene_);
    mjr_freeContext(&context_);
    throw std::runtime_error("MuJoCo offscreen framebuffer is unavailable");
  }
}

CameraRenderer::~CameraRenderer() {
  async_.reset();
  mjv_freeScene(&scene_);
  mjr_freeContext(&context_);
}

void CameraRenderer::Render(mjData* data, CameraFrame& frame) {
  mjv_updateScene(model_, data, &option_, nullptr, &camera_, mjCAT_ALL, &scene_);
  SetBlackSky(&scene_, frame.state.dark_sky);
  mjr_setBuffer(mjFB_OFFSCREEN, &context_);
  const mjrRect viewport{0, 0, width_, height_};
  mjr_render(viewport, &scene_, &context_);
  // RGB only: no extra depth/gray conversion and no per-frame intermediate allocation.
  mjr_readPixels(readback_.data(), nullptr, viewport, &context_);
  frame.width = width_;
  frame.height = height_;
  const auto k = ModelIntrinsics(model_, camera_id_, width_, height_);
  frame.fx = frame.focal_length = k.fx;
  frame.fy = k.fy;
  frame.cx = k.cx;
  frame.cy = k.cy;
  frame.rgb.resize(readback_.size());
  const std::size_t row_size = static_cast<std::size_t>(width_) * 3;
  for (int row = 0; row < height_; ++row) {
    std::memcpy(frame.rgb.data() + row * row_size,
                readback_.data() + (height_ - row - 1) * row_size, row_size);
  }
}

bool CameraRenderer::RenderAsync(mjData* data, CameraFrame capture, CameraFrame& ready,
                                 bool shader, bool shadows, double exposure, double bloom,
                                 double lamp_intensity, CameraMetrics& metrics) {
  if (!async_) async_ = std::make_unique<Async>(width_, height_, context_);
  while (glGetError() != GL_NO_ERROR) {}  // Isolate this frame's GL errors.
  bool completed = async_->Collect(ready, metrics);
  auto& slot = async_->slots[async_->write];
  if (slot.fence) {
    ++metrics.dropped_frames;  // Never wait for a busy GPU and build stale camera frames.
    return completed;
  }
  const auto scene_begin = Clock::now();
  mjv_updateScene(model_, data, &option_, nullptr, &camera_, mjCAT_ALL, &scene_);
  SetBlackSky(&scene_, capture.state.dark_sky);
  scene_.flags[mjRND_SHADOW] = shadows;
  if (shadows)
    for (int i = 0; i < scene_.nlight; ++i)
      if (!scene_.lights[i].headlight) scene_.lights[i].castshadow = 1;
  metrics.scene_ms = Ms(scene_begin, Clock::now());
  const auto render_begin = Clock::now();
  glBeginQuery(GL_TIME_ELAPSED, slot.timer);
  mjr_setBuffer(mjFB_OFFSCREEN, &context_);
  mjr_render({0, 0, width_, height_}, &scene_, &context_);
  CheckGl("MuJoCo render");
  const auto k = ModelIntrinsics(model_, camera_id_, width_, height_);
  async_->pipeline.Process(scene_, context_, k.fx, k.fy, k.cx, k.cy,
                           exposure, bloom, lamp_intensity, shader);
  glEndQuery(GL_TIME_ELAPSED);
  CheckGl("HDR pipeline");
  metrics.render_ms = Ms(render_begin, Clock::now());
  metrics.gpu_estimate_mib = async_->pipeline.EstimatedMiB() +
                              3.0 * async_->bytes / (1024.0 * 1024.0);
  capture.width = width_;
  capture.height = height_;
  capture.fx = capture.focal_length = k.fx;
  capture.fy = k.fy;
  capture.cx = k.cx;
  capture.cy = k.cy;
  slot.frame = std::move(capture);
  async_->pipeline.ReadPixelsToPbo(slot.pbo);
  slot.fence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
  glFlush();
  CheckGl("PBO readback");
  async_->write = (async_->write + 1) % async_->slots.size();
  return completed || async_->Collect(ready, metrics);
}
}  // namespace rm_auto_aim_sim
