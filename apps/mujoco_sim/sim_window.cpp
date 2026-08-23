/**
 * @file sim_window.cpp
 * @brief 接入 MuJoCo 官方 Simulate UI，并把键盘事件转换为控制输入。
 */

#include "sim_window.hpp"

#include "glfw_adapter.h"
#include "simulate.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

namespace quadruped::apps::mujoco_sim
{
class SimWindow::Impl
{
public:
    explicit Impl(const bool vsync)
    {
        mjv_defaultCamera(&camera);
        mjv_defaultOption(&option);
        mjv_defaultPerturb(&perturb);

        auto adapter = std::make_unique<mujoco::GlfwAdapter>();
        simulate = std::make_unique<mujoco::Simulate>(
            std::move(adapter), &camera, &option, &perturb, true);
        simulate->vsync = vsync ? 1 : 0;
    }

    ~Impl()
    {
        if (display_data != nullptr)
        {
            mj_deleteData(display_data);
        }
        if (display_model != nullptr)
        {
            mj_deleteModel(display_model);
        }
    }

    mjvCamera camera{};
    mjvOption option{};
    mjvPerturb perturb{};
    std::unique_ptr<mujoco::Simulate> simulate{};
    mjModel* display_model{nullptr};
    mjData* display_data{nullptr};
    std::atomic<bool> paused{false};
};

SimWindow::SimWindow(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
SimWindow::~SimWindow() = default;

SimWindow::CreateResult SimWindow::create(const bool vsync)
{
    CreateResult result;
    try
    {
        result.window.reset(new SimWindow(std::make_unique<Impl>(vsync)));
    }
    catch (const std::exception& error)
    {
        result.error_message = error.what();
    }
    return result;
}

void SimWindow::load(
    const mjModel* const model,
    mjData* const data,
    const std::string& displayed_filename)
{
    if (model == nullptr || data == nullptr)
    {
        return;
    }
    if (impl_->display_data != nullptr)
    {
        mj_deleteData(impl_->display_data);
        impl_->display_data = nullptr;
    }
    if (impl_->display_model != nullptr)
    {
        mj_deleteModel(impl_->display_model);
        impl_->display_model = nullptr;
    }
    impl_->display_model = mj_copyModel(nullptr, model);
    if (impl_->display_model == nullptr)
    {
        throw std::runtime_error("复制 MuJoCo 显示模型失败");
    }
    impl_->display_data = mj_makeData(impl_->display_model);
    if (impl_->display_data == nullptr)
    {
        mj_deleteModel(impl_->display_model);
        impl_->display_model = nullptr;
        throw std::runtime_error("创建 MuJoCo 显示数据失败");
    }
    mj_copyData(impl_->display_data, impl_->display_model, data);
    impl_->simulate->Load(
        impl_->display_model, impl_->display_data, displayed_filename.c_str());
    focus_on_robot(impl_->display_model, impl_->display_data);
}

SimWindow::SimulationAction SimWindow::sync(
    const mjModel* const model,
    mjData* const data)
{
    if (model == nullptr || data == nullptr || impl_->display_model == nullptr ||
        impl_->display_data == nullptr)
    {
        return {};
    }
    // pending_ 由渲染线程写入。先取走会改变 mjData 的动作，防止官方 Sync
    // 只修改显示副本，随后又被物理数据覆盖而造成闪烁。
    mujoco::MutexLock lock(impl_->simulate->mtx, std::try_to_lock);
    if (!lock.owns_lock())
    {
        // 窗口线程持锁时跳过这一帧，不能让渲染事件阻塞物理步进和 heartbeat。
        return {};
    }
    SimulationAction action;
    if (impl_->simulate->pending_.load_key)
    {
        action.type = SimulationAction::Type::LoadKey;
        action.keyframe_id = impl_->simulate->key;
    }
    else if (impl_->simulate->pending_.reset)
    {
        action.type = SimulationAction::Type::Reset;
    }
    impl_->simulate->pending_.reset = false;
    impl_->simulate->pending_.load_key = false;
    mj_copyData(impl_->display_data, impl_->display_model, data);
    impl_->simulate->Sync(true);
    impl_->paused.store(impl_->simulate->run == 0, std::memory_order_release);
    // 官方 passive viewer 把鼠标扰动力写入显示副本；显式复制回物理数据，
    // 选择、拖拽和渲染仍留在 GUI 线程拥有的副本中。
    mju_copy(
        data->xfrc_applied,
        impl_->display_data->xfrc_applied,
        static_cast<int>(6 * model->nbody));
    return action;
}

bool SimWindow::paused() const
{
    return impl_->paused.load(std::memory_order_acquire);
}

void SimWindow::toggle_pause()
{
    mujoco::MutexLock lock(impl_->simulate->mtx);
    impl_->simulate->run = impl_->simulate->run == 0 ? 1 : 0;
    impl_->paused.store(impl_->simulate->run == 0, std::memory_order_release);
}

void SimWindow::focus_on_robot(const mjModel* const model, const mjData* const data)
{
    if (model == nullptr || data == nullptr)
    {
        return;
    }

    const int trunk_id = mj_name2id(model, mjOBJ_BODY, "trunk");
    if (trunk_id < 0)
    {
        return;
    }

    // 只统计 trunk 子树，避免大地形拉大 model.stat.extent 后把机器人缩成远处小点。
    double robot_radius = 0.0;
    for (int body_id = 1; body_id < model->nbody; ++body_id)
    {
        int ancestor_id = body_id;
        while (ancestor_id > 0 && ancestor_id != trunk_id)
        {
            ancestor_id = model->body_parentid[ancestor_id];
        }
        if (ancestor_id != trunk_id)
        {
            continue;
        }

        const mjtNum* const body_position = data->xpos + 3 * body_id;
        const mjtNum* const trunk_position = data->xpos + 3 * trunk_id;
        const double dx = body_position[0] - trunk_position[0];
        const double dy = body_position[1] - trunk_position[1];
        const double dz = body_position[2] - trunk_position[2];
        robot_radius = std::max(robot_radius, std::sqrt(dx * dx + dy * dy + dz * dz));
    }

    mujoco::MutexLock lock(impl_->simulate->mtx);
    // 只用 trunk 当前位置设置初始取景点，后续由用户自由操作相机。
    impl_->camera.type = mjCAMERA_FREE;
    impl_->camera.trackbodyid = -1;
    impl_->camera.fixedcamid = -1;
    mju_copy3(impl_->camera.lookat, data->xpos + 3 * trunk_id);
    impl_->camera.distance = std::clamp(robot_radius * 3.0, 1.5, 4.0);
    impl_->camera.azimuth = 135.0;
    impl_->camera.elevation = -20.0;
}

void SimWindow::render_loop()
{
    impl_->simulate->RenderLoop();
}

void SimWindow::request_exit()
{
    impl_->simulate->exitrequest.store(1);
}

bool SimWindow::should_close() const
{
    return impl_->simulate->exitrequest.load() != 0;
}

}  // 命名空间 quadruped::apps::mujoco_sim
