/**
 * @file sim_window.hpp
 * @brief 封装 MuJoCo 官方 Simulate 界面及线程安全的仿真控制输入。
 */

#pragma once

#include <mujoco/mujoco.h>

#include <memory>
#include <string>

namespace quadruped::apps::mujoco_sim
{

// 官方 Simulate 在主线程渲染；物理线程通过 load、sync 和 take_input 与它交换数据。
class SimWindow
{
public:
    struct CreateResult
    {
        std::unique_ptr<SimWindow> window{};
        std::string error_message{};

        [[nodiscard]] bool ok() const noexcept
        {
            return window != nullptr;
        }
    };

    SimWindow(const SimWindow&) = delete;
    SimWindow& operator=(const SimWindow&) = delete;
    ~SimWindow();

    // vsync 决定 GPU 换帧是否等待显示器刷新，不改变物理或控制频率。
    static CreateResult create(bool vsync);

    // load() 由物理线程调用，并等待主线程完成 OpenGL 资源装载。
    // 装载后自由相机会对准 trunk，不使用包含大地形的 model.stat 作为取景范围。
    void load(const mjModel* model, mjData* data, const std::string& displayed_filename);

    // 同步官方界面输入；返回 true 表示用户点击了官方 Reset。
    // 该 Reset 会被拦截，由上层统一恢复 MJCF 零位并重建运动会话。
    [[nodiscard]] bool sync();

    // 把自由相机重新对准机器人躯干；只在加载和 reset 时调用，不持续跟随。
    void focus_on_robot(const mjModel* model, const mjData* data);

    // render_loop() 必须在创建窗口的主线程调用，直至窗口关闭。
    void render_loop();
    void request_exit();

    [[nodiscard]] bool should_close() const;
private:
    class Impl;

    explicit SimWindow(std::unique_ptr<Impl> impl);

    std::unique_ptr<Impl> impl_{};
};

}  // 命名空间 quadruped::apps::mujoco_sim
