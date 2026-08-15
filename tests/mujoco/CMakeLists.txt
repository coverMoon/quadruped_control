# 文件：CMakeLists.txt
# 作用：构建并注册 MuJoCo 模型加载器的自动测试。

add_executable(quadruped_mujoco_tests
    mujoco_loader_tests.cpp
)

target_link_libraries(quadruped_mujoco_tests PRIVATE quadruped::mujoco)
target_compile_features(quadruped_mujoco_tests PRIVATE cxx_std_17)

# 正式模型和错误 fixture 的路径在构建期固定传入，测试不依赖运行时当前目录。
target_compile_definitions(quadruped_mujoco_tests PRIVATE
    QUADRUPED_BLACK_SCENE_PATH="${PROJECT_SOURCE_DIR}/assets/robots/black/mujoco/scene.xml"
    QUADRUPED_MUJOCO_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures"
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(quadruped_mujoco_tests PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
elseif(MSVC)
    target_compile_options(quadruped_mujoco_tests PRIVATE /W4)
endif()

add_test(NAME quadruped_mujoco_tests COMMAND quadruped_mujoco_tests)
