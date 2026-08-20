/**
 * @file lodepng.h
 * @brief 为 MuJoCo Simulate 的截图功能提供兼容的 PNG 写入接口。
 */

#pragma once

#include <string>

enum LodePNGColorType
{
    LCT_RGB = 2
};

namespace lodepng
{

unsigned encode(
    const std::string& filename,
    const unsigned char* image,
    unsigned width,
    unsigned height,
    LodePNGColorType color_type);

}  // 命名空间 lodepng
