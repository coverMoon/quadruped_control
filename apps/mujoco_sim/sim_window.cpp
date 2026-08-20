/**
 * @file sim_window.cpp
 * @brief 接入 MuJoCo 官方 Simulate UI，并把键盘事件转换为控制输入。
 */

#include "sim_window.hpp"

#include "glfw_adapter.h"
#include "simulate.h"

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
}

void SimWindow::sync()
{
    impl_->simulate->Sync();
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
