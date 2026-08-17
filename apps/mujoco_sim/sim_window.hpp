/**
 * @file sim_window.hpp
 * @brief 定义 mujoco_sim 的 GLFW 窗口封装：渲染、相机控制和键盘输入快照。
 */

#pragma once

#include "sim_input.hpp"

#include <mujoco/mujoco.h>

#include <memory>
#include <string>

struct GLFWwindow;

namespace quadruped::apps::mujoco_sim
{

// 窗口创建参数：尺寸和标题集中描述，避免 create() 参数过多。
struct WindowOptions
{
    int width{1280};
    int height{720};
    const char* title{""};
};

// 单窗口的 MuJoCo 渲染和输入封装；键盘回调只记录边沿事件，
// 不直接运行状态机或推进物理，物理和控制由主循环统一调度。
class SimWindow
{
public:
    // create() 的返回结果：成功时 window 非空，失败时 error_message 含有可读原因。
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

    // 创建窗口并初始化 MuJoCo 渲染资源；model 必须长于窗口生命周期。
    static CreateResult create(const mjModel* model, const WindowOptions& options);

    // 用户请求关闭窗口（关闭按钮或 Esc）时返回 true。
    [[nodiscard]] bool should_close() const;

    // 处理窗口事件并刷新输入快照；每个主循环周期调用一次。
    void poll_events();

    // 本周期产生的按键边沿事件。
    [[nodiscard]] const SimInput& input() const noexcept
    {
        return input_;
    }

    // 渲染当前 mjData 状态，并在左上角叠加一行状态文本和一行错误/提示文本。
    void render(mjData* data, const std::string& status_text, const std::string& detail_text);

private:
    SimWindow(const mjModel* model, GLFWwindow* window);

    // GLFW 回调入口，通过 window user pointer 转发到实例方法。
    static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
    static void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos);
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

    void handle_key(int key, int action);
    void handle_mouse_move(double xpos, double ypos);

    const mjModel* model_{nullptr};
    GLFWwindow* window_{nullptr};

    mjvCamera camera_{};
    mjvOption option_{};
    mjvScene scene_{};
    mjrContext context_{};

    SimInput input_{};

    // 鼠标相机控制状态：记录按键按下状态和上一帧光标位置。
    bool mouse_left_{false};
    bool mouse_right_{false};
    double last_cursor_x_{0.0};
    double last_cursor_y_{0.0};
};

}  // 命名空间 quadruped::apps::mujoco_sim
