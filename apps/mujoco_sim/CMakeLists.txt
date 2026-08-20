# 文件：CMakeLists.txt
# 作用：构建带 GLFW 界面的交互式 MuJoCo 仿真程序 quadruped_mujoco_sim。

find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PNG REQUIRED)

set(QUADRUPED_MUJOCO_SIMULATE_DIR "${mujoco_ROOT}/simulate")
if(NOT EXISTS "${QUADRUPED_MUJOCO_SIMULATE_DIR}/simulate.cc")
    message(FATAL_ERROR "固定 MuJoCo 发行包缺少 Simulate 源码")
endif()

# 官方发行包只提供 Simulate 可执行文件，因此从同一固定版本源码构建可嵌入界面。
add_library(quadruped_mujoco_simulate STATIC
    lodepng.cpp
    "${QUADRUPED_MUJOCO_SIMULATE_DIR}/glfw_adapter.cc"
    "${QUADRUPED_MUJOCO_SIMULATE_DIR}/glfw_dispatch.cc"
    "${QUADRUPED_MUJOCO_SIMULATE_DIR}/platform_ui_adapter.cc"
    "${QUADRUPED_MUJOCO_SIMULATE_DIR}/simulate.cc"
)

target_include_directories(quadruped_mujoco_simulate
    SYSTEM PUBLIC "${QUADRUPED_MUJOCO_SIMULATE_DIR}"
)

target_include_directories(quadruped_mujoco_simulate PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(quadruped_mujoco_simulate
    PUBLIC mujoco::mujoco glfw OpenGL::GL
    PRIVATE PNG::PNG
)

target_compile_features(quadruped_mujoco_simulate PUBLIC cxx_std_17)

add_executable(quadruped_mujoco_sim
    main.cpp
    sim_controller.cpp
    sim_window.cpp
    terminal_input.cpp
)

target_link_libraries(quadruped_mujoco_sim
    PRIVATE
        quadruped::config
        quadruped::motion
        quadruped::mujoco
        quadruped_mujoco_simulate
)

target_compile_features(quadruped_mujoco_sim PRIVATE cxx_std_17)

# 编译期固定默认场景和配置路径，程序运行时不需要从当前目录查找仓库。
target_compile_definitions(quadruped_mujoco_sim
    PRIVATE
        QUADRUPED_DEFAULT_SCENE_PATH="${PROJECT_SOURCE_DIR}/assets/robots/black/mujoco/scene.xml"
        QUADRUPED_DEFAULT_ROBOT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/robots/black.yaml"
        QUADRUPED_DEFAULT_CONTROLLER_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/controllers/black.yaml"
        QUADRUPED_DEFAULT_SIMULATION_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/simulation/black_mujoco.yaml"
)

if(QUADRUPED_ENABLE_TORCH)
    target_link_libraries(quadruped_mujoco_sim PRIVATE quadruped::torch_policy)
    target_compile_definitions(quadruped_mujoco_sim PRIVATE
        QUADRUPED_WITH_TORCH=1
        QUADRUPED_PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
        QUADRUPED_DEFAULT_POLICY_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/policies/black/flat.yaml"
    )
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(quadruped_mujoco_sim PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
elseif(MSVC)
    target_compile_options(quadruped_mujoco_sim PRIVATE /W4)
endif()
