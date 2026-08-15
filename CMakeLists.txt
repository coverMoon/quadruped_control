# 文件：CMakeLists.txt
# 作用：配置 quadruped_control 工程及其核心库和测试入口。

cmake_minimum_required(VERSION 3.16)

project(quadruped_control VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(QUADRUPED_ENABLE_MUJOCO "构建需要 MuJoCo 的仿真模块" OFF)

if(QUADRUPED_ENABLE_MUJOCO)
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
    find_package(mujoco 3.9.0 EXACT REQUIRED MODULE)
endif()

add_subdirectory(core)

if(QUADRUPED_ENABLE_MUJOCO)
    add_subdirectory(backends/mujoco)
endif()

add_subdirectory(apps)

include(CTest)
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
