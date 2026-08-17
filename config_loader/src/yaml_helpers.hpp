/**
 * @file yaml_helpers.hpp
 * @brief 配置加载内部共用的 YAML 字段读取辅助，保证非法配置以错误返回而不是抛异常。
 */

#pragma once

#include <yaml-cpp/yaml.h>

#include <string>
#include <utility>

namespace quadruped::config::detail
{

// 单个 YAML 映射节点的字段读取器：共享节点和错误输出，避免每个辅助函数都携带
// 重复的位置参数。节点按值持有：YAML::Node 是轻量共享句柄，且 Node::operator[]
// 按值返回临时对象，保存引用会悬挂，因此这里必须拷贝而不能保存调用方节点的引用。
class FieldReader
{
public:
    FieldReader(YAML::Node node, std::string& error) : node_(std::move(node)), error_(error)
    {
    }

    // 读取必需的标量字段；字段缺失、类型错误或节点不是映射时返回 false 并写出原因。
    template <typename T>
    bool required(const char* key, T& out)
    {
        try
        {
            const YAML::Node value = node_[key];
            if (!value)
            {
                error_ = std::string("missing required key \"") + key + "\"";
                return false;
            }
            out = value.as<T>();
        }
        catch (const YAML::Exception& e)
        {
            error_ = std::string("invalid value for key \"") + key + "\": " + e.what();
            return false;
        }
        return true;
    }

    // 读取必需的双精度数组字段，并要求元素数量与 expected_size 一致。
    bool double_array(const char* key, double* out, std::size_t expected_size);

    // 写入自定义错误并返回 false，方便调用方统一返回失败。
    bool fail(const std::string& message)
    {
        error_ = message;
        return false;
    }

    [[nodiscard]] const YAML::Node& node() const noexcept
    {
        return node_;
    }

    [[nodiscard]] std::string& error_message() noexcept
    {
        return error_;
    }

private:
    YAML::Node node_;
    std::string& error_;
};

inline bool FieldReader::double_array(
    const char* key,
    double* out,
    const std::size_t expected_size)
{
    try
    {
        const YAML::Node value = node_[key];
        if (!value)
        {
            error_ = std::string("missing required key \"") + key + "\"";
            return false;
        }
        if (!value.IsSequence() || value.size() != expected_size)
        {
            error_ = std::string("key \"") + key + "\" must be a sequence of " +
                std::to_string(expected_size) + " numbers";
            return false;
        }
        for (std::size_t i = 0; i < expected_size; ++i)
        {
            out[i] = value[i].as<double>();
        }
    }
    catch (const YAML::Exception& e)
    {
        error_ = std::string("invalid number in key \"") + key + "\": " + e.what();
        return false;
    }
    return true;
}

}  // 命名空间 quadruped::config::detail
