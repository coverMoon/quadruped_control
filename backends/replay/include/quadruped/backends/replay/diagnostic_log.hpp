/**
 * @file diagnostic_log.hpp
 * @brief 定义由低频调用方驱动的运动诊断 CSV 记录器。
 */

#pragma once

#include "quadruped/core/robot_model.hpp"
#include "quadruped/motion/motion_runtime.hpp"

#include <memory>
#include <string>

namespace quadruped::backends::replay
{

class DiagnosticLogWriter
{
public:
    struct CreateResult
    {
        std::unique_ptr<DiagnosticLogWriter> writer{};
        std::string error_message{};

        [[nodiscard]] bool ok() const noexcept
        {
            return writer != nullptr;
        }
    };

    static CreateResult create(const std::string& path, core::RobotModel model);

    ~DiagnosticLogWriter();
    DiagnosticLogWriter(const DiagnosticLogWriter&) = delete;
    DiagnosticLogWriter& operator=(const DiagnosticLogWriter&) = delete;

    // 只允许低频观察侧调用。generated_command 为空表示本次抽样没有可对照的
    // 目标命令。
    bool write(
        const core::StateFrame& state,
        const motion::MotionUpdateOutput& update,
        const core::CommandFrame* generated_command,
        std::string& error_message);

private:
    class Impl;
    explicit DiagnosticLogWriter(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

}  // namespace quadruped::backends::replay
