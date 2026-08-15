# 文件：core/CMakeLists.txt
# 作用：构建不依赖外部机器人框架的 quadruped_core 静态库。

add_library(quadruped_core
    src/validation.cpp
)

add_library(quadruped::core ALIAS quadruped_core)

target_include_directories(quadruped_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(quadruped_core PUBLIC cxx_std_17)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(quadruped_core PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
elseif(MSVC)
    target_compile_options(quadruped_core PRIVATE /W4)
endif()
