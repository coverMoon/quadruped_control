/**
 * @file sim_window.cpp
 * @brief 实现 GLFW 窗口、MuJoCo 场景渲染和键盘鼠标输入处理。
 */

#include "sim_window.hpp"

#include <GLFW/glfw3.h>

#include <cmath>
#include <string>
#include <utility>

namespace quadruped::apps::mujoco_sim
{
namespace
{

// 场景允许的最大几何体数量；平地加 12 关节机器人远小于该值。
inline constexpr int kMaxSceneGeoms = 2000;

// 初始相机方位：从斜上方观察机器人，便于确认起立和趴下。
inline constexpr double kInitialAzimuth = 120.0;
inline constexpr double kInitialElevation = -20.0;
inline constexpr double kInitialDistance = 2.0;
inline constexpr double kInitialLookatZ = 0.2;

}  // 匿名命名空间

SimWindow::SimWindow(const mjModel* model, GLFWwindow* window)
    : model_(model), window_(window)
{
}

SimWindow::~SimWindow()
{
    // 先释放 MuJoCo 渲染资源，再销毁窗口并终止 GLFW。
    mjv_freeScene(&scene_);
    mjr_freeContext(&context_);
    if (window_ != nullptr)
    {
        glfwDestroyWindow(window_);
    }
    glfwTerminate();
}

SimWindow::CreateResult SimWindow::create(
    const mjModel* model,
    const WindowOptions& options)
{
    CreateResult result;
    if (!glfwInit())
    {
        result.error_message = "glfwInit failed";
        return result;
    }

    GLFWwindow* window =
        glfwCreateWindow(options.width, options.height, options.title, nullptr, nullptr);
    if (window == nullptr)
    {
        glfwTerminate();
        result.error_message = "glfwCreateWindow failed";
        return result;
    }

    glfwMakeContextCurrent(window);
    // 开启垂直同步，渲染帧率受显示器限制，不影响主循环的控制调度。
    glfwSwapInterval(1);

    result.window.reset(new SimWindow(model, window));
    auto& self = *result.window;

    mjv_defaultCamera(&self.camera_);
    self.camera_.azimuth = kInitialAzimuth;
    self.camera_.elevation = kInitialElevation;
    self.camera_.distance = kInitialDistance;
    self.camera_.lookat[2] = kInitialLookatZ;
    mjv_defaultOption(&self.option_);
    mjv_defaultScene(&self.scene_);
    mjv_makeScene(model, &self.scene_, kMaxSceneGeoms);
    mjr_defaultContext(&self.context_);
    mjr_makeContext(model, &self.context_, mjFONTSCALE_150);

    glfwSetWindowUserPointer(window, &self);
    glfwSetKeyCallback(window, &SimWindow::key_callback);
    glfwSetMouseButtonCallback(window, &SimWindow::mouse_button_callback);
    glfwSetCursorPosCallback(window, &SimWindow::cursor_pos_callback);
    glfwSetScrollCallback(window, &SimWindow::scroll_callback);
    return result;
}

bool SimWindow::should_close() const
{
    return input_.quit || glfwWindowShouldClose(window_);
}

void SimWindow::poll_events()
{
    // 先清零上一周期的边沿事件，再由回调写入本周期的按下事件。
    input_ = SimInput{};
    glfwPollEvents();
}

void SimWindow::render(
    mjData* data,
    const std::string& status_text,
    const std::string& detail_text)
{
    mjrRect viewport{0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

    mjv_updateScene(model_, data, &option_, nullptr, &camera_, mjCAT_ALL, &scene_);
    mjr_render(viewport, &scene_, &context_);
    mjr_overlay(
        mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, status_text.c_str(), detail_text.c_str(),
        &context_);

    glfwSwapBuffers(window_);
}

void SimWindow::key_callback(
    GLFWwindow* window,
    const int key,
    const int /*scancode*/,
    const int action,
    const int /*mods*/)
{
    auto* self = static_cast<SimWindow*>(glfwGetWindowUserPointer(window));
    if (self != nullptr)
    {
        self->handle_key(key, action);
    }
}

void SimWindow::handle_key(const int key, const int action)
{
    // 只处理按下边沿，忽略释放和长按重复，避免同一按键触发多个请求。
    if (action != GLFW_PRESS)
    {
        return;
    }
    switch (key)
    {
    case GLFW_KEY_0:
    case GLFW_KEY_KP_0:
        input_.getup = true;
        break;
    case GLFW_KEY_9:
    case GLFW_KEY_KP_9:
        input_.getdown = true;
        break;
    case GLFW_KEY_P:
        input_.enter_passive = true;
        break;
    case GLFW_KEY_R:
        input_.reset = true;
        break;
    case GLFW_KEY_SPACE:
        input_.toggle_pause = true;
        break;
    case GLFW_KEY_ESCAPE:
        input_.quit = true;
        break;
    default:
        break;
    }
}

void SimWindow::mouse_button_callback(
    GLFWwindow* window,
    const int button,
    const int action,
    const int /*mods*/)
{
    auto* self = static_cast<SimWindow*>(glfwGetWindowUserPointer(window));
    if (self == nullptr)
    {
        return;
    }
    const bool pressed = (action == GLFW_PRESS);
    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        self->mouse_left_ = pressed;
    }
    else if (button == GLFW_MOUSE_BUTTON_RIGHT)
    {
        self->mouse_right_ = pressed;
    }
    glfwGetCursorPos(window, &self->last_cursor_x_, &self->last_cursor_y_);
}

void SimWindow::cursor_pos_callback(GLFWwindow* window, const double xpos, const double ypos)
{
    auto* self = static_cast<SimWindow*>(glfwGetWindowUserPointer(window));
    if (self != nullptr)
    {
        self->handle_mouse_move(xpos, ypos);
    }
}

void SimWindow::handle_mouse_move(const double xpos, const double ypos)
{
    const double dx = xpos - last_cursor_x_;
    const double dy = ypos - last_cursor_y_;
    last_cursor_x_ = xpos;
    last_cursor_y_ = ypos;

    int width = 0;
    int height = 0;
    glfwGetWindowSize(window_, &width, &height);
    if (height <= 0)
    {
        return;
    }

    // 左键拖动旋转，右键拖动平移；位移量按窗口高度归一化。
    int action = mjMOUSE_NONE;
    if (mouse_left_)
    {
        action = mjMOUSE_ROTATE_V;
    }
    else if (mouse_right_)
    {
        action = mjMOUSE_MOVE_V;
    }
    if (action != mjMOUSE_NONE)
    {
        mjv_moveCamera(model_, action, dx / height, dy / height, &scene_, &camera_);
    }
}

void SimWindow::scroll_callback(
    GLFWwindow* window,
    const double /*xoffset*/,
    const double yoffset)
{
    auto* self = static_cast<SimWindow*>(glfwGetWindowUserPointer(window));
    if (self == nullptr)
    {
        return;
    }
    mjv_moveCamera(self->model_, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &self->scene_,
        &self->camera_);
}

}  // 命名空间 quadruped::apps::mujoco_sim
