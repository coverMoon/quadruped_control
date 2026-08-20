# 文件：CMakeLists.txt
# 作用：构建只依赖 core 的基础运动运行时库 quadruped_motion。

add_library(quadruped_motion
    src/motion_requests.cpp
    src/motion_results.cpp
    src/motion_runtime.cpp
    src/rl_controller.cpp
)

add_library(quadruped::motion ALIAS quadruped_motion)

target_include_directories(quadruped_motion
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(quadruped_motion
    PUBLIC
        quadruped::core
)

target_compile_features(quadruped_motion PUBLIC cxx_std_17)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(quadruped_motion PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
elseif(MSVC)
    target_compile_options(quadruped_motion PRIVATE /W4)
endif()
