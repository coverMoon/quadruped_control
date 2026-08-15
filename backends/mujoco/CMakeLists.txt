# 文件：CMakeLists.txt
# 作用：构建 MuJoCo 后端库，提供模型加载和名称映射。

add_library(quadruped_mujoco
    src/mujoco_model.cpp
    src/mujoco_robot_io.cpp
    src/sim_time.cpp
    src/state_frame_fill.cpp
)

add_library(quadruped::mujoco ALIAS quadruped_mujoco)

target_include_directories(quadruped_mujoco
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(quadruped_mujoco
    PUBLIC
        quadruped::core
        mujoco::mujoco
)

target_compile_features(quadruped_mujoco PUBLIC cxx_std_17)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(quadruped_mujoco PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
elseif(MSVC)
    target_compile_options(quadruped_mujoco PRIVATE /W4)
endif()
