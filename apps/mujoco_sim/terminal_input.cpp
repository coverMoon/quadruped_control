/**
 * @file terminal_input.cpp
 * @brief 实现终端非阻塞单键读取、速度增量和退出时终端恢复。
 */

#include "terminal_input.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <unistd.h>

namespace quadruped::apps::mujoco_sim
{
namespace
{

inline constexpr double kCommandIncrement = 0.1;

}  // 匿名命名空间

TerminalInput::TerminalInput(std::array<double, 3> command_limits)
    : command_limits_(command_limits)
{
    if (isatty(STDIN_FILENO) == 0 || tcgetattr(STDIN_FILENO, &original_termios_) != 0)
    {
        return;
    }

    termios raw = original_termios_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    interactive_ = tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
}

TerminalInput::~TerminalInput()
{
    if (interactive_)
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
    }
}

void TerminalInput::adjust(
    double& value,
    const double increment,
    const double limit,
    SimInput& input)
{
    // 速度始终落在 0.1 网格上，避免反复加减后出现接近零的二进制残差。
    const double quantized = std::round((value + increment) / kCommandIncrement) *
        kCommandIncrement;
    const double adjusted = std::clamp(quantized, -limit, limit);
    input.command_changed = input.command_changed || adjusted != value;
    value = adjusted;
}

void TerminalInput::adjust_command(
    double& value,
    const double increment,
    const std::size_t axis,
    const bool velocity_enabled,
    SimInput& input)
{
    if (velocity_enabled)
    {
        adjust(value, increment, command_limits_[axis], input);
    }
    else if (!warned_outside_running_)
    {
        input.command_rejected = true;
        warned_outside_running_ = true;
    }
}

void TerminalInput::handle_character(
    const char character,
    const bool velocity_enabled,
    SimInput& input)
{
    const char key = static_cast<char>(
        std::tolower(static_cast<unsigned char>(character)));
    switch (key)
    {
    case '0':
        input.getup = true;
        break;
    case '1':
        input.start_rl = true;
        break;
    case '2':
        input.switch_flat = true;
        break;
    case '3':
        input.switch_obstacle = true;
        break;
    case '9':
        input.getdown = true;
        break;
    case 'p':
        input.enter_passive = true;
        break;
    case 'r':
        input.reset = true;
        break;
    case 'k':
        input.toggle_pause = true;
        break;
    case 'x':
    case 27:
        input.quit = true;
        break;
    case 'h':
        input.show_help = true;
        break;
    case 'w':
        adjust_command(vx_, kCommandIncrement, 0, velocity_enabled, input);
        break;
    case 's':
        adjust_command(vx_, -kCommandIncrement, 0, velocity_enabled, input);
        break;
    case 'a':
        adjust_command(vy_, kCommandIncrement, 1, velocity_enabled, input);
        break;
    case 'd':
        adjust_command(vy_, -kCommandIncrement, 1, velocity_enabled, input);
        break;
    case 'q':
        adjust_command(wz_, kCommandIncrement, 2, velocity_enabled, input);
        break;
    case 'e':
        adjust_command(wz_, -kCommandIncrement, 2, velocity_enabled, input);
        break;
    case ' ':
        input.command_changed =
            input.command_changed || vx_ != 0.0 || vy_ != 0.0 || wz_ != 0.0;
        vx_ = 0.0;
        vy_ = 0.0;
        wz_ = 0.0;
        break;
    default:
        break;
    }
}

SimInput TerminalInput::poll(const bool velocity_enabled)
{
    SimInput input;
    if (velocity_enabled)
    {
        warned_outside_running_ = false;
    }
    else if (vx_ != 0.0 || vy_ != 0.0 || wz_ != 0.0)
    {
        vx_ = 0.0;
        vy_ = 0.0;
        wz_ = 0.0;
        input.command_changed = true;
    }
    if (interactive_)
    {
        char character = 0;
        while (read(STDIN_FILENO, &character, 1) == 1)
        {
            handle_character(character, velocity_enabled, input);
        }
    }
    input.vx = vx_;
    input.vy = vy_;
    input.wz = wz_;
    return input;
}

}  // 命名空间 quadruped::apps::mujoco_sim
