# 文件：CMakeLists.txt
# 作用：构建并注册 MuJoCo 模型加载器、状态生成和故障路径的自动测试。

add_executable(quadruped_mujoco_tests
    mujoco_loader_tests.cpp
)

add_executable(quadruped_mujoco_state_tests
    mujoco_state_tests.cpp
)

add_executable(quadruped_mujoco_fault_tests
    mujoco_fault_tests.cpp
)

add_executable(quadruped_mujoco_time_tests
    mujoco_time_tests.cpp
)

add_executable(quadruped_mujoco_command_tests
    mujoco_command_tests.cpp
)

add_executable(quadruped_mujoco_step_tests
    mujoco_step_tests.cpp
)

add_executable(quadruped_mujoco_step_state_tests
    mujoco_step_state_tests.cpp
)

add_executable(quadruped_mujoco_step_safety_tests
    mujoco_step_safety_tests.cpp
)

add_executable(quadruped_mujoco_determinism_tests
    mujoco_determinism_tests.cpp
)

add_executable(quadruped_mujoco_motion_tests
    mujoco_motion_tests.cpp
)

# SimController 回归测试直接编译应用调度实现，避免为测试新建应用库。
add_executable(quadruped_mujoco_sim_controller_tests
    mujoco_sim_controller_tests.cpp
    ${PROJECT_SOURCE_DIR}/apps/mujoco_sim/sim_controller.cpp
)
target_include_directories(quadruped_mujoco_sim_controller_tests PRIVATE
    ${PROJECT_SOURCE_DIR}/apps/mujoco_sim
)
target_link_libraries(quadruped_mujoco_sim_controller_tests PRIVATE
    quadruped::config
    quadruped::motion
)

# 测试共用辅助函数，只在本目录使用。
add_library(quadruped_mujoco_test_helpers STATIC
    test_helpers.cpp
)
target_link_libraries(quadruped_mujoco_test_helpers PRIVATE quadruped::config quadruped::mujoco)
target_compile_features(quadruped_mujoco_test_helpers PRIVATE cxx_std_17)
target_compile_definitions(quadruped_mujoco_test_helpers PRIVATE
    QUADRUPED_ROBOT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/robots/black.yaml"
)

# 正式模型和错误 fixture 的路径在构建期固定传入，测试不依赖运行时当前目录。
set(QUADRUPED_TEST_COMPILE_DEFS
    QUADRUPED_BLACK_SCENE_PATH="${PROJECT_SOURCE_DIR}/assets/robots/black/mujoco/scene.xml"
    QUADRUPED_MUJOCO_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures"
    QUADRUPED_ROBOT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/robots/black.yaml"
    QUADRUPED_CONTROLLER_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/controllers/black.yaml"
)

# M2 运动集成测试还需要 MotionRuntime 和配置加载。
target_link_libraries(quadruped_mujoco_motion_tests PRIVATE quadruped::config quadruped::motion)

foreach(target quadruped_mujoco_tests quadruped_mujoco_state_tests quadruped_mujoco_fault_tests quadruped_mujoco_time_tests quadruped_mujoco_command_tests quadruped_mujoco_step_tests quadruped_mujoco_step_state_tests quadruped_mujoco_step_safety_tests quadruped_mujoco_determinism_tests quadruped_mujoco_motion_tests quadruped_mujoco_sim_controller_tests)
    target_link_libraries(${target} PRIVATE quadruped::mujoco quadruped_mujoco_test_helpers)
    target_compile_features(${target} PRIVATE cxx_std_17)
    target_compile_definitions(${target} PRIVATE ${QUADRUPED_TEST_COMPILE_DEFS})
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
    elseif(MSVC)
        target_compile_options(${target} PRIVATE /W4)
    endif()
    add_test(NAME ${target} COMMAND ${target})
endforeach()
