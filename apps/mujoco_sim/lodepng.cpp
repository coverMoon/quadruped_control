/**
 * @file lodepng.cpp
 * @brief 使用系统 libpng 实现 MuJoCo Simulate 所需的 RGB 截图写入接口。
 */

#include "lodepng.h"

#include <png.h>

#include <cstdio>
#include <vector>

namespace lodepng
{

unsigned encode(
    const std::string& filename,
    const unsigned char* const image,
    const unsigned width,
    const unsigned height,
    const LodePNGColorType color_type)
{
    if (image == nullptr || width == 0 || height == 0 || color_type != LCT_RGB)
    {
        return 1;
    }

    FILE* const file = std::fopen(filename.c_str(), "wb");
    if (file == nullptr)
    {
        return 2;
    }
    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    png_infop info = png != nullptr ? png_create_info_struct(png) : nullptr;
    if (png == nullptr || info == nullptr)
    {
        png_destroy_write_struct(&png, &info);
        std::fclose(file);
        return 3;
    }
    if (setjmp(png_jmpbuf(png)) != 0)
    {
        png_destroy_write_struct(&png, &info);
        std::fclose(file);
        return 4;
    }

    png_init_io(png, file);
    png_set_IHDR(png, info, width, height, 8, PNG_COLOR_TYPE_RGB,
        PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);

    std::vector<png_bytep> rows(height);
    for (unsigned row = 0; row < height; ++row)
    {
        rows[row] = const_cast<png_bytep>(image + row * width * 3);
    }
    png_write_image(png, rows.data());
    png_write_end(png, nullptr);
    png_destroy_write_struct(&png, &info);
    std::fclose(file);
    return 0;
}

}  // 命名空间 lodepng
