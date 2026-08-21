/**
 * @file sim_window.cpp
 * @brief 接入 MuJoCo 官方 Simulate UI，并把键盘事件转换为控制输入。
 */

#include "sim_window.hpp"

#include "glfw_adapter.h"
#include "simulate.h"

#include <algorithm>
#include <cmath>
#include <exception>
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

    mjvCamera camera{};
    mjvOption option{};
    mjvPerturb perturb{};
    std::unique_ptr<mujoco::Simulate> simulate{};
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
    // 后端持有的模型实际可写；只有官方调参面板需要通过其 C API 修改模型选项。
    impl_->simulate->Load(const_cast<mjModel*>(model), data, displayed_filename.c_str());
    focus_on_robot(model, data);
}

bool SimWindow::sync()
{
    // pending_ 由渲染线程写入；持有官方互斥锁后取走 Reset，
    // 防止 Simulate::Sync() 绕过上层会话状态单独执行 mj_resetData()。
    mujoco::MutexLock lock(impl_->simulate->mtx);
    const bool reset_requested = impl_->simulate->pending_.reset;
    impl_->simulate->pending_.reset = false;
    impl_->simulate->Sync();
    return reset_requested;
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
