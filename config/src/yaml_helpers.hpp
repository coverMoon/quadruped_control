/**
 * @file yaml_helpers.hpp
 * @brief 配置加载内部共用的 YAML 字段读取辅助，保证非法配置以错误返回而不是抛异常。
 */

#pragma once

#include <yaml-cpp/yaml.h>

#include <cctype>
#include <cstdint>
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

    // 读取 64 位无符号整数。严格文本语法：仅 0x/0X 前缀按 16 进制解析，
    // 其余一律按 10 进制；拒绝任何符号和前后空白。
    bool uint64(const char* key, std::uint64_t& out);

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
    // 读取指定键的原始标量文本；失败时写出错误。uint64 的读取部分。
    bool read_scalar_text(const char* key, std::string& text);

    // 逐字符校验数字部分符合所选进制；stoull 会跳过空白，不能只依赖它。
    bool validate_digits(const char* key, const std::string& digits, int base);

    // 按严格文本语法解析无符号整数；失败时写出错误。uint64 的解析部分。
    bool parse_unsigned_text(const char* key, const std::string& text, std::uint64_t& out);

    YAML::Node node_;
    std::string& error_;
};

inline bool FieldReader::read_scalar_text(const char* key, std::string& text)
{
    // 读取原始标量文本而不是解析后的值，保证前导零按十进制解释、
    // 0x 前缀按十六进制解释，并且符号与空白可以被可靠检查。
    try
    {
        const YAML::Node value = node_[key];
        if (!value)
        {
            error_ = std::string("missing required key \"") + key + "\"";
            return false;
        }
        if (!value.IsScalar())
        {
            error_ = std::string("value for key \"") + key + "\" must be a scalar";
            return false;
        }
        text = value.Scalar();
    }
    catch (const YAML::Exception& e)
    {
        error_ = std::string("invalid value for key \"") + key + "\": " + e.what();
        return false;
    }
    return true;
}

inline bool FieldReader::validate_digits(
    const char* key,
    const std::string& digits,
    const int base)
{
    if (digits.empty())
    {
        error_ = std::string("unsigned integer for key \"") + key + "\" has no digits";
        return false;
    }
    for (const char c : digits)
    {
        const bool decimal_digit = c >= '0' && c <= '9';
        const bool hex_digit = decimal_digit || (c >= 'a' && c <= 'f') ||
            (c >= 'A' && c <= 'F');
        if (!(base == 10 ? decimal_digit : hex_digit))
        {
            error_ = std::string("invalid unsigned integer for key \"") + key +
                "\": " + digits;
            return false;
        }
    }
    return true;
}

inline bool FieldReader::parse_unsigned_text(
    const char* key,
    const std::string& text,
    std::uint64_t& out)
{
    // 拒绝前后空白和正负号；stoull 会跳过空白并接受负号，不能直接依赖它。
    if (text.empty() ||
        std::isspace(static_cast<unsigned char>(text.front())) != 0 ||
        std::isspace(static_cast<unsigned char>(text.back())) != 0)
    {
        error_ = std::string("unsigned integer for key \"") + key +
            "\" must not have surrounding whitespace";
        return false;
    }
    int base = 10;
    std::size_t offset = 0;
    if (text.size() > 2 && text[0] == '0' && (text[1] == 'x' || text[1] == 'X'))
    {
        base = 16;
        offset = 2;
    }
    if (text.size() > offset && (text[offset] == '-' || text[offset] == '+'))
    {
        error_ = std::string("unsigned integer for key \"") + key +
            "\" must not carry a sign";
        return false;
    }

    // 数字部分逐字符校验后再解析，避免 stoull 跳过前缀后的空白或接受非法字符。
    const std::string digits = text.substr(offset);
    if (!validate_digits(key, digits, base))
    {
        return false;
    }
    try
    {
        std::size_t parsed = 0;
        out = std::stoull(digits, &parsed, base);
        if (parsed != digits.size())
        {
            error_ = std::string("invalid unsigned integer for key \"") + key +
                "\": " + text;
            return false;
        }
    }
    catch (const std::exception&)
    {
        error_ = std::string("invalid unsigned integer for key \"") + key + "\": " + text;
        return false;
    }
    return true;
}

inline bool FieldReader::uint64(const char* key, std::uint64_t& out)
{
    std::string text;
    return read_scalar_text(key, text) && parse_unsigned_text(key, text, out);
}

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
