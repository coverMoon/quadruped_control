# 文件：CMakeLists.txt
# 作用：配置 quadruped_control 工程及其核心库和测试入口。

cmake_minimum_required(VERSION 3.16)

project(quadruped_control VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(QUADRUPED_ENABLE_MUJOCO "构建需要 MuJoCo 的仿真模块" OFF)
option(QUADRUPED_ENABLE_TORCH "构建需要固定 LibTorch 的策略模块" OFF)

if(QUADRUPED_ENABLE_MUJOCO)
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
    find_package(mujoco 3.9.0 EXACT REQUIRED MODULE)
endif()

add_subdirectory(core)
add_subdirectory(adapters/ipc)
add_subdirectory(motion)
add_subdirectory(config_loader)
add_subdirectory(backends/replay)

if(QUADRUPED_ENABLE_MUJOCO)
    add_subdirectory(backends/mujoco)
endif()

if(QUADRUPED_ENABLE_TORCH)
    # 固定仓库内依赖路径，避免 cache、Torch_DIR 或环境变量切换到外部 LibTorch。
    set(QUADRUPED_LIBTORCH_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.deps/libtorch-2.0.1-cpu")
    unset(Torch_DIR CACHE)
    unset(Torch_DIR)
    find_package(Torch 2.0.1 EXACT REQUIRED CONFIG
        PATHS "${QUADRUPED_LIBTORCH_ROOT}/share/cmake/Torch"
        NO_DEFAULT_PATH)
    file(REAL_PATH "${Torch_DIR}" quadruped_torch_config_dir)
    file(REAL_PATH "${QUADRUPED_LIBTORCH_ROOT}/share/cmake/Torch" quadruped_expected_torch_dir)
    if(NOT quadruped_torch_config_dir STREQUAL quadruped_expected_torch_dir)
        message(FATAL_ERROR "TorchConfig must come from the repository-local LibTorch directory")
    endif()
    add_subdirectory(policy/torch)
endif()

add_subdirectory(apps)

include(CTest)
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
