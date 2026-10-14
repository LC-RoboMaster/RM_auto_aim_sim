#include "rm_auto_aim_sim/camera_renderer.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace rm_auto_aim_sim
{
    CameraRenderer::CameraRenderer(const mjModel* model, int camera_id, int width, int height)
        : model_(model),
          camera_id_(camera_id),
          width_(width),
          height_(height),
          readback_(static_cast<std::size_t>(width) * height * 3)
    {
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
        if (context_.currentBuffer != mjFB_OFFSCREEN)
        {
            mjv_freeScene(&scene_);
            mjr_freeContext(&context_);
            throw std::runtime_error("MuJoCo offscreen framebuffer is unavailable");
        }
    }

    CameraRenderer::~CameraRenderer()
    {
        mjv_freeScene(&scene_);
        mjr_freeContext(&context_);
    }

    void CameraRenderer::Render(mjData* data, CameraFrame& frame)
    {
        mjv_updateScene(model_, data, &option_, nullptr, &camera_, mjCAT_ALL, &scene_);
        mjr_setBuffer(mjFB_OFFSCREEN, &context_);
        const mjrRect viewport{0, 0, width_, height_};
        mjr_render(viewport, &scene_, &context_);
        // RGB only: no extra depth/gray conversion and no per-frame intermediate allocation.
        mjr_readPixels(readback_.data(), nullptr, viewport, &context_);
        frame.width = width_;
        frame.height = height_;
        frame.focal_length =
            0.5 * height_ / std::tan(model_->cam_fovy[camera_id_] * 3.141592653589793 / 360.0);
        frame.rgb.resize(readback_.size());
        const std::size_t row_size = static_cast<std::size_t>(width_) * 3;
        for (int row = 0; row < height_; ++row)
        {
            std::memcpy(frame.rgb.data() + row * row_size,
                        readback_.data() + (height_ - row - 1) * row_size, row_size);
        }
    }
} // namespace rm_auto_aim_sim
