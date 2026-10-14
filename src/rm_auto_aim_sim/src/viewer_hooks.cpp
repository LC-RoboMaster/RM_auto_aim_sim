#include "rm_auto_aim_sim/viewer_hooks.hpp"

#include <GLFW/glfw3.h>

#include <cstdio>

#include "glfw_adapter.h"
#include "simulate.h"

namespace rm_auto_aim_sim
{
    namespace
    {
        Engine* engine = nullptr;
        int mode_ui = 0;
        int view_ui = 0;
        int section_id = -1;
        bool camera_was_fixed = false;
        mjvCamera saved_free_camera{};

        void ChangeView(mujoco::Simulate* sim)
        {
            if (view_ui == 1)
            {
                if (sim->cam.type == mjCAMERA_FREE) saved_free_camera = sim->cam;
                sim->cam.type = mjCAMERA_FIXED;
                sim->cam.fixedcamid = engine->CameraId();
                sim->camera = engine->CameraId() + 2;
                camera_was_fixed = true;
            }
            else
            {
                sim->cam = saved_free_camera;
                sim->cam.type = mjCAMERA_FREE;
                sim->camera = 0;
                camera_was_fixed = false;
            }
            mjui_update(-1, -1, &sim->ui0, &sim->uistate, &sim->platform_ui->mjr_context());
        }

        bool FirstPerson(const mujoco::Simulate* sim)
        {
            return sim->cam.type == mjCAMERA_FIXED && sim->cam.fixedcamid == engine->CameraId();
        }
    } // namespace

    void BindViewerEngine(Engine* value) { engine = value; }

    void AddViewerMenu(mujoco::Simulate* sim)
    {
        if (!engine) return;
        mode_ui = static_cast<int>(engine->State().mode);
        view_ui = 0;
        saved_free_camera = sim->cam;
        section_id = sim->ui1.nsect;
        mjuiDef definition[] = {
            {mjITEM_SECTION, "Auto Aim", 1},
            {mjITEM_SELECT, "Control mode", 1, &mode_ui, "MANUAL\nAUTO (ROS2)"},
            {mjITEM_SELECT, "View", 1, &view_ui, "Free\nGimbal"},
            {
                mjITEM_STATIC, "Controls", 1, nullptr,
                "WASD: move\nQ/E: height\nArrows: aim\nRMB: aim\nHold LMB: fire\nF: camera"
            },
            {mjITEM_STATIC, "Angles (rad)", 1, nullptr, "Pitch 0.000\nYaw 0.000"},
            {mjITEM_STATIC, "Projectiles", 1, nullptr, "Shots 0\nActive 0\nDropped 0"},
            {mjITEM_END}
        };
        mjui_add(&sim->ui1, definition);
    }

    bool HandleViewerMenu(mujoco::Simulate* sim, const mjuiItem* item)
    {
        if (!engine || !item || item->sectionid != section_id) return false;
        if (item->itemid == 0) engine->SetMode(static_cast<ControlMode>(mode_ui));
        if (item->itemid == 1) ChangeView(sim);
        mjui_update(section_id, -1, &sim->ui1, &sim->uistate, &sim->platform_ui->mjr_context());
        return true;
    }

    bool HandleViewerEvent(mujoco::Simulate* sim, const mjuiState* state)
    {
        if (!engine || (state->mouserect != 3 && state->dragrect != 3)) return false;
        if (state->type == mjEVENT_KEY && (state->key == 'F' || state->key == 'f'))
        {
            view_ui = FirstPerson(sim) ? 0 : 1;
            ChangeView(sim);
            return true;
        }
        // Native rendering shortcuts also use WASDQE; consume them before mjui_event.
        if (state->type == mjEVENT_KEY && engine->State().mode == ControlMode::kManual &&
            (state->key == mjKEY_LEFT || state->key == mjKEY_RIGHT || state->key == mjKEY_UP ||
                state->key == mjKEY_DOWN || state->key == 'W' || state->key == 'A' || state->key == 'S' ||
                state->key == 'D' || state->key == 'Q' || state->key == 'E'))
        {
            return true;
        }
        if (!FirstPerson(sim) || engine->State().mode != ControlMode::kManual) return false;
        if (state->type == mjEVENT_MOVE && state->right)
        {
            const double sensitivity = engine->GetConfig().mouse_sensitivity;
            engine->AddManualAngles(state->dy * sensitivity, -state->dx * sensitivity);
            return true;
        }
        // Do not let firing clicks trigger native selection/perturbation in this view.
        return (state->type == mjEVENT_PRESS || state->type == mjEVENT_RELEASE) &&
            state->button == mjBUTTON_LEFT;
    }

    void UpdateViewerInput(mujoco::Simulate* sim)
    {
        if (!engine) return;
        const auto state = engine->State();
        auto* adapter = dynamic_cast<mujoco::GlfwAdapter*>(sim->platform_ui.get());
        if (!adapter) return;
        auto* window = adapter->ApplicationWindow();
        const bool focused = glfwGetWindowAttrib(window, GLFW_FOCUSED) == GLFW_TRUE;
        auto key = [&](int code)
        {
            return focused && glfwGetKey(window, code) == GLFW_PRESS ? 1.0 : 0.0;
        };
        ManualInput input;
        // Restrict controls to the 3D region so editing native UI fields does not move the gimbal.
        if (focused && sim->uistate.mouserect == 3 && state.mode == ControlMode::kManual)
        {
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
        if (section_id >= 0 && section_id < sim->ui1.nsect)
        {
            auto& angles = sim->ui1.sect[section_id].item[3].multi;
            std::snprintf(angles.name[0], mjMAXUINAME, "Pitch %.3f", state.pitch);
            std::snprintf(angles.name[1], mjMAXUINAME, "Yaw %.3f", state.yaw);
            auto& projectiles = sim->ui1.sect[section_id].item[4].multi;
            std::snprintf(projectiles.name[0], mjMAXUINAME, "Shots %llu",
                          static_cast<unsigned long long>(state.shot_count));
            std::snprintf(projectiles.name[1], mjMAXUINAME, "Active %d", state.active_projectiles);
            std::snprintf(projectiles.name[2], mjMAXUINAME, "Dropped %llu",
                          static_cast<unsigned long long>(state.pool_exhausted_count));
            mjui_update(section_id, -1, &sim->ui1, &sim->uistate, &sim->platform_ui->mjr_context());
        }
    }

    void StepFromViewer()
    {
        if (engine) engine->Step();
    }
} // namespace rm_auto_aim_sim
