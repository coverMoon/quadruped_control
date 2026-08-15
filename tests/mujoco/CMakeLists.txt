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

# 正式模型和错误 fixture 的路径在构建期固定传入，测试不依赖运行时当前目录。
set(QUADRUPED_TEST_COMPILE_DEFS
    QUADRUPED_BLACK_SCENE_PATH="${PROJECT_SOURCE_DIR}/assets/robots/black/mujoco/scene.xml"
    QUADRUPED_MUJOCO_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures"
)

foreach(target quadruped_mujoco_tests quadruped_mujoco_state_tests quadruped_mujoco_fault_tests)
    target_link_libraries(${target} PRIVATE quadruped::mujoco)
    target_compile_features(${target} PRIVATE cxx_std_17)
    target_compile_definitions(${target} PRIVATE ${QUADRUPED_TEST_COMPILE_DEFS})
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
    elseif(MSVC)
        target_compile_options(${target} PRIVATE /W4)
    endif()
    add_test(NAME ${target} COMMAND ${target})
endforeach()
