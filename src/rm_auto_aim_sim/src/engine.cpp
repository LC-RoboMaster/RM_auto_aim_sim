#include "rm_auto_aim_sim/engine.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace rm_auto_aim_sim
{
    namespace
    {
        int RequireId(const mjModel* model, mjtObj type, const std::string& name)
        {
            const int id = mj_name2id(model, type, name.c_str());
            if (id < 0) throw std::runtime_error("Required model object missing: " + name);
            return id;
        }

        mjModel* CompileModel(const Config& config)
        {
            char error[2048] = {};
            std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
                mj_parseXML(config.scene_path.c_str(), nullptr, error, sizeof(error)), mj_deleteSpec);
            if (!spec) throw std::runtime_error(error);
            auto* world = mjs_findBody(spec.get(), "world");
            if (!world) throw std::runtime_error("Model has no world body");
            auto* material = mjs_addMaterial(spec.get(), nullptr);
            mjs_setName(material->element, "sim_projectile_glow");
            material->rgba[0] = 0.05f;
            material->rgba[1] = 1.0f;
            material->rgba[2] = 0.05f;
            material->rgba[3] = 1.0f;
            material->emission = 1.0f;
            for (int index = 0; index < config.projectile_pool_size; ++index)
            {
                const auto name = "projectile_" + std::to_string(index);
                auto* body = mjs_addBody(world, nullptr);
                mjs_setName(body->element, name.c_str());
                body->pos[0] = 10000.0 + 0.1 * index;
                body->pos[1] = 10000;
                body->pos[2] = 100;
                mjs_addFreeJoint(body);
                auto* geom = mjs_addGeom(body, nullptr);
                mjs_setName(geom->element, (name + "_geom").c_str());
                geom->type = mjGEOM_SPHERE;
                geom->size[0] = config.projectile_diameter / 2;
                geom->mass = config.projectile_mass;
                geom->contype = 2;
                geom->conaffinity = 1; // Environment=1/2; projectile-projectile pairs are disabled.
                geom->condim = 3;
                geom->friction[0] = 0.3;
                geom->solref[0] = 0.002;
                geom->solref[1] = 1;
                mjs_setString(geom->material, "sim_projectile_glow");
            }
            std::unique_ptr<mjModel, decltype(&mj_deleteModel)> compiled(mj_compile(spec.get(), nullptr),
                                                                         mj_deleteModel);
            if (!compiled) throw std::runtime_error(mjs_getError(spec.get()));
            mjModel* model = compiled.get();
            model->opt.timestep = config.physics_timestep;
            model->vis.global.offwidth = config.camera_width;
            model->vis.global.offheight = config.camera_height;
            model->vis.quality.offsamples = config.camera_msaa;
            // Parked bodies must not enlarge the automatic far plane or free-camera distance.
            model->stat.extent = 25;
            mju_zero(model->stat.center, 3);
            const int camera = RequireId(model, mjOBJ_CAMERA, config.model_camera_name);
            if (model->cam_orthographic[camera] || model->cam_sensorsize[2 * camera] > 0 ||
                model->cam_sensorsize[2 * camera + 1] > 0)
            {
                throw std::runtime_error(
                    "Camera must use centered perspective FOV, without sensor/focal attributes");
            }
            model->cam_fovy[camera] = config.camera_fovy_deg;
            model->cam_ipd[camera] = 0;
            const int yaw = RequireId(model, mjOBJ_JOINT, config.model_yaw_joint);
            const int pitch = RequireId(model, mjOBJ_JOINT, config.model_pitch_joint);
            for (const auto& joint :
                 {
                     std::make_pair(yaw, std::make_pair(config.yaw_min, config.yaw_max)),
                     std::make_pair(pitch, std::make_pair(config.pitch_min, config.pitch_max))
                 })
            {
                model->jnt_range[2 * joint.first] = joint.second.first;
                model->jnt_range[2 * joint.first + 1] = joint.second.second;
                model->jnt_limited[joint.first] = 1;
            }
            return compiled.release();
        }
    } // namespace

    void CommandMailbox::SetMode(ControlMode mode)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        mode_ = mode;
        pitch_valid_ = yaw_valid_ = fire_ = false;
    }

    void CommandMailbox::SetPitch(double angle)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mode_ != ControlMode::kAuto || !std::isfinite(angle)) return;
        pitch_ = angle;
        pitch_valid_ = true;
        pitch_time_ = std::chrono::steady_clock::now();
    }

    void CommandMailbox::SetYaw(double angle)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mode_ != ControlMode::kAuto || !std::isfinite(angle)) return;
        yaw_ = angle;
        yaw_valid_ = true;
        yaw_time_ = std::chrono::steady_clock::now();
    }

    void CommandMailbox::SetFire(bool enabled)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mode_ != ControlMode::kAuto) return;
        fire_ = enabled;
        fire_time_ = std::chrono::steady_clock::now();
    }

    void CommandMailbox::Apply(const Config& config, double current_pitch, double current_yaw,
                               double& target_pitch, double& target_yaw, bool& fire)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = std::chrono::steady_clock::now();
        auto fresh = [&](auto time, double timeout)
        {
            return timeout == 0 || std::chrono::duration<double>(now - time).count() <= timeout;
        };
        if (pitch_valid_)
            target_pitch = fresh(pitch_time_, config.angle_command_timeout) ? pitch_ : current_pitch;
        if (yaw_valid_) target_yaw = fresh(yaw_time_, config.angle_command_timeout) ? yaw_ : current_yaw;
        fire = fire_ && fresh(fire_time_, config.fire_command_timeout);
    }

    Engine::Engine(Config config) : config_(std::move(config))
    {
        config_.Validate();
        try
        {
            model_ = CompileModel(config_);
            data_ = mj_makeData(model_);
            if (!data_) throw std::runtime_error("Cannot allocate mjData");
            base_id_ = RequireId(model_, mjOBJ_BODY, config_.model_base_body);
            yaw_id_ = RequireId(model_, mjOBJ_BODY, config_.model_yaw_body);
            pitch_id_ = RequireId(model_, mjOBJ_BODY, config_.model_pitch_body);
            camera_body_id_ = RequireId(model_, mjOBJ_BODY, config_.model_camera_body);
            camera_id_ = RequireId(model_, mjOBJ_CAMERA, config_.model_camera_name);
            muzzle_id_ = RequireId(model_, mjOBJ_SITE, config_.model_muzzle_site);
            const int yaw_joint = RequireId(model_, mjOBJ_JOINT, config_.model_yaw_joint);
            const int pitch_joint = RequireId(model_, mjOBJ_JOINT, config_.model_pitch_joint);
            mocap_id_ = model_->body_mocapid[base_id_];
            if (mocap_id_ < 0 || model_->body_parentid[yaw_id_] != base_id_ ||
                model_->body_parentid[pitch_id_] != yaw_id_ ||
                model_->body_parentid[camera_body_id_] != pitch_id_ ||
                model_->cam_bodyid[camera_id_] != camera_body_id_ ||
                model_->site_bodyid[muzzle_id_] != pitch_id_ || model_->body_jntnum[camera_body_id_] != 0 ||
                model_->jnt_type[yaw_joint] != mjJNT_HINGE ||
                model_->jnt_type[pitch_joint] != mjJNT_HINGE || model_->jnt_bodyid[yaw_joint] != yaw_id_ ||
                model_->jnt_bodyid[pitch_joint] != pitch_id_)
            {
                throw std::runtime_error(
                    "Expected mocap base -> yaw hinge -> pitch hinge -> fixed camera; muzzle on pitch");
            }
            yaw_qpos_ = model_->jnt_qposadr[yaw_joint];
            pitch_qpos_ = model_->jnt_qposadr[pitch_joint];
            yaw_dof_ = model_->jnt_dofadr[yaw_joint];
            pitch_dof_ = model_->jnt_dofadr[pitch_joint];
            pool_ = std::make_unique<ProjectilePool>(config_, model_);
            mode_ = config_.initial_mode == "auto" ? ControlMode::kAuto : ControlMode::kManual;
            Reset();
        }
        catch (...)
        {
            if (data_) mj_deleteData(data_);
            if (model_) mj_deleteModel(model_);
            throw;
        }
    }

    Engine::~Engine()
    {
        mj_deleteData(data_);
        mj_deleteModel(model_);
    }

    void Engine::Reset()
    {
        pool_->Reset(data_);
        pitch_ = target_pitch_ =
            std::clamp(data_->qpos[pitch_qpos_], config_.pitch_min, config_.pitch_max);
        yaw_ = target_yaw_ = std::clamp(data_->qpos[yaw_qpos_], config_.yaw_min, config_.yaw_max);
        pitch_velocity_ = yaw_velocity_ = 0;
        mailbox_.SetMode(mode_);
        manual_input_ = {};
        last_time_ = data_->time;
        next_shot_time_ = data_->time;
        was_firing_ = false;
        ApplyGimbalPose();
        mj_forward(model_, data_);
    }

    void Engine::SetMode(ControlMode mode)
    {
        if (mode_ == mode) return;
        mode_ = mode;
        target_pitch_ = pitch_;
        target_yaw_ = yaw_;
        manual_input_ = {};
        was_firing_ = false;
        mailbox_.SetMode(mode); // Entering AUTO requires fresh ROS commands.
    }

    void Engine::AddManualAngles(double pitch_delta, double yaw_delta)
    {
        if (mode_ != ControlMode::kManual) return;
        target_pitch_ = std::clamp(target_pitch_ + pitch_delta, config_.pitch_min, config_.pitch_max);
        target_yaw_ = std::clamp(target_yaw_ + yaw_delta, config_.yaw_min, config_.yaw_max);
    }

    void Engine::ApplyGimbalPose()
    {
        // Ideal angle servo: no artificial recoil or gravity-induced axis drift.
        data_->qpos[pitch_qpos_] = pitch_;
        data_->qpos[yaw_qpos_] = yaw_;
        data_->qvel[pitch_dof_] = 0;
        data_->qvel[yaw_dof_] = 0;
    }

    void Engine::Step()
    {
        if (data_->time < last_time_ - 1e-10) Reset(); // Viewer reset/history rewind.
        const double dt = model_->opt.timestep;
        bool fire = false;
        if (mode_ == ControlMode::kManual)
        {
            AddManualAngles(manual_input_.pitch_rate * config_.manual_angular_speed * dt,
                            manual_input_.yaw_rate * config_.manual_angular_speed * dt);
            auto* base = data_->mocap_pos + 3 * mocap_id_;
            const double speed = config_.manual_translation_speed * dt;
            base[0] = std::clamp(base[0] + speed * (std::cos(yaw_) * manual_input_.forward -
                                     std::sin(yaw_) * manual_input_.left),
                                 config_.base_x_min, config_.base_x_max);
            base[1] = std::clamp(base[1] + speed * (std::sin(yaw_) * manual_input_.forward +
                                     std::cos(yaw_) * manual_input_.left),
                                 config_.base_y_min, config_.base_y_max);
            base[2] =
                std::clamp(base[2] + speed * manual_input_.up, config_.base_z_min, config_.base_z_max);
            fire = manual_input_.fire;
        }
        else
        {
            mailbox_.Apply(config_, pitch_, yaw_, target_pitch_, target_yaw_, fire);
        }
        target_pitch_ = std::clamp(target_pitch_, config_.pitch_min, config_.pitch_max);
        target_yaw_ = std::clamp(target_yaw_, config_.yaw_min, config_.yaw_max);
        const double max_delta = config_.servo_max_speed * dt;
        const double pitch_delta = std::clamp(target_pitch_ - pitch_, -max_delta, max_delta);
        const double yaw_delta = std::clamp(target_yaw_ - yaw_, -max_delta, max_delta);
        pitch_ += pitch_delta;
        yaw_ += yaw_delta;
        pitch_velocity_ = pitch_delta / dt;
        yaw_velocity_ = yaw_delta / dt;
        pool_->BeforeStep(data_);
        ApplyGimbalPose();
        mj_forward(model_, data_); // Muzzle pose must match this step's commanded axes.
        if (fire)
        {
            if (!was_firing_) next_shot_time_ = data_->time;
            if (data_->time + 1e-10 >= next_shot_time_)
            {
                pool_->Spawn(data_, State().muzzle);
                next_shot_time_ += 1.0 / config_.fire_rate_hz;
            }
        }
        was_firing_ = fire;
        mj_step(model_, data_);
        pool_->AfterStep(data_);
        ApplyGimbalPose();
        mj_forward(model_, data_); // Recompute poses at the published timestamp, not the previous step.
        last_time_ = data_->time;
    }

    Pose Engine::BodyPose(const mjData* source, int body) const
    {
        Pose pose;
        std::copy(source->xpos + 3 * body, source->xpos + 3 * body + 3, pose.position.begin());
        std::copy(source->xquat + 4 * body, source->xquat + 4 * body + 4, pose.quaternion.begin());
        return pose;
    }

    StateFrame Engine::State(const mjData* source) const
    {
        if (!source) source = data_;
        StateFrame frame;
        frame.time = source->time;
        frame.pitch = source->qpos[pitch_qpos_];
        frame.yaw = source->qpos[yaw_qpos_];
        frame.pitch_velocity = pitch_velocity_;
        frame.yaw_velocity = yaw_velocity_;
        frame.mode = mode_;
        frame.shot_count = pool_->ShotCount();
        frame.pool_exhausted_count = pool_->ExhaustedCount();
        frame.active_projectiles = pool_->ActiveCount();
        frame.base = BodyPose(source, base_id_);
        frame.yaw_link = BodyPose(source, yaw_id_);
        frame.pitch_link = BodyPose(source, pitch_id_);
        frame.camera_link = BodyPose(source, camera_body_id_);
        std::copy(source->cam_xpos + 3 * camera_id_, source->cam_xpos + 3 * camera_id_ + 3,
                  frame.optical.position.begin());
        mjtNum optical_rotation[9];
        const auto* camera_rotation = source->cam_xmat + 9 * camera_id_;
        for (int row = 0; row < 3; ++row)
        {
            optical_rotation[3 * row] = camera_rotation[3 * row];
            optical_rotation[3 * row + 1] = -camera_rotation[3 * row + 1];
            optical_rotation[3 * row + 2] = -camera_rotation[3 * row + 2];
        }
        mju_mat2Quat(frame.optical.quaternion.data(), optical_rotation);
        std::copy(source->site_xpos + 3 * muzzle_id_, source->site_xpos + 3 * muzzle_id_ + 3,
                  frame.muzzle.position.begin());
        mju_mat2Quat(frame.muzzle.quaternion.data(), source->site_xmat + 9 * muzzle_id_);
        return frame;
    }
} // namespace rm_auto_aim_sim
