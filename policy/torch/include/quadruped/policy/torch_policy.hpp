/**
 * @file torch_policy.hpp
 * @brief 定义单个 TorchScript black 策略的轻量推理适配器。
 */

#pragma once

#include "quadruped/motion/rl_controller.hpp"

#include <memory>
#include <string>

namespace quadruped::policy
{

class TorchPolicy final : public motion::Policy
{
public:
    struct CreateResult
    {
        std::unique_ptr<TorchPolicy> policy{};
        std::string error_message{};

        [[nodiscard]] bool ok() const noexcept
        {
            return policy != nullptr;
        }
    };

    static CreateResult create(motion::RlConfig config);

    ~TorchPolicy() override;
    TorchPolicy(const TorchPolicy&) = delete;
    TorchPolicy& operator=(const TorchPolicy&) = delete;
    TorchPolicy(TorchPolicy&&) noexcept;
    TorchPolicy& operator=(TorchPolicy&&) noexcept;

    motion::RlInferenceOutput forward(const motion::RlInferenceInput& input) override;
    [[nodiscard]] const std::string& name() const noexcept;

private:
    class Impl;
    explicit TorchPolicy(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

}  // namespace quadruped::policy
