/**
 * @file torch_policy.cpp
 * @brief 实现固定 270 输入、12 输出的 TorchScript 策略推理。
 */

#include "quadruped/policy/torch_policy.hpp"

#include <torch/script.h>
#include <torch/torch.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

namespace quadruped::policy
{
namespace
{

std::once_flag g_torch_thread_once;

torch::Tensor extract_tensor(const torch::jit::IValue& value)
{
    if (value.isTensor())
    {
        return value.toTensor();
    }
    if (value.isTuple() && !value.toTuple()->elements().empty() &&
        value.toTuple()->elements()[0].isTensor())
    {
        return value.toTuple()->elements()[0].toTensor();
    }
    return {};
}

bool valid_output(const torch::jit::IValue& value, torch::Tensor& output)
{
    output = extract_tensor(value);
    return output.defined() && output.device().is_cpu() &&
        output.scalar_type() == torch::kFloat32 && output.dim() == 2 &&
        output.size(0) == 1 && output.size(1) == static_cast<long>(motion::kRlActionDim) &&
        torch::isfinite(output).all().item<bool>();
}

}  // namespace

class TorchPolicy::Impl
{
public:
    motion::RlConfig config{};
    torch::jit::script::Module module{};
};

TorchPolicy::TorchPolicy(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
TorchPolicy::~TorchPolicy() = default;
TorchPolicy::TorchPolicy(TorchPolicy&&) noexcept = default;
TorchPolicy& TorchPolicy::operator=(TorchPolicy&&) noexcept = default;

TorchPolicy::CreateResult TorchPolicy::create(motion::RlConfig config)
{
    CreateResult result;
    if (config.name.empty() || config.model_path.empty())
    {
        result.error_message = "Torch policy requires a name and model path";
        return result;
    }
    try
    {
        std::call_once(g_torch_thread_once, [] {
            torch::set_num_threads(1);
            torch::set_num_interop_threads(1);
        });
        auto impl = std::make_unique<Impl>();
        impl->config = std::move(config);
        impl->module = torch::jit::load(impl->config.model_path);
        impl->module.eval();

        // 必须通过正式 forward 路径预热；首次 from_blob/clone 也可能初始化 Torch CPU 运行时。
        result.policy.reset(new TorchPolicy(std::move(impl)));
        const motion::RlInferenceInput warmup_input;
        constexpr int kWarmupRuns = 3;
        for (int run = 0; run < kWarmupRuns; ++run)
        {
            const auto warmup_output = result.policy->forward(warmup_input);
            if (!warmup_output.ok)
            {
                result.error_message =
                    "Torch policy warmup failed: " + warmup_output.error_message;
                result.policy.reset();
                return result;
            }
        }
    }
    catch (const std::exception& error)
    {
        result.error_message = "Torch policy load failed: " + std::string(error.what());
    }
    return result;
}

motion::RlInferenceOutput TorchPolicy::forward(const motion::RlInferenceInput& input)
{
    motion::RlInferenceOutput result;
    const auto start = std::chrono::steady_clock::now();
    try
    {
        const auto options = torch::TensorOptions().dtype(torch::kFloat32);
        const auto tensor = torch::from_blob(
            const_cast<float*>(input.observation.data()),
            {1, static_cast<long>(motion::kRlInputDim)},
            options).clone();
        torch::InferenceMode guard;
        torch::Tensor output;
        if (!valid_output(impl_->module.forward({tensor}), output))
        {
            result.error_message = "Torch policy returned an invalid output";
        }
        else
        {
            output = output.contiguous();
            std::copy_n(output.data_ptr<float>(), motion::kRlActionDim, result.actions.begin());
            result.ok = true;
        }
    }
    catch (const std::exception& error)
    {
        result.error_message = "Torch inference failed: " + std::string(error.what());
    }
    result.elapsed_ns = static_cast<core::Nanoseconds>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - start).count());
    return result;
}

const std::string& TorchPolicy::name() const noexcept
{
    return impl_->config.name;
}

}  // namespace quadruped::policy
