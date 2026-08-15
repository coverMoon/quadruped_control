# 文件：CMakeLists.txt
# 作用：配置 quadruped_control 工程及其核心库和测试入口。

cmake_minimum_required(VERSION 3.16)

project(quadruped_control VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(core)

include(CTest)
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
