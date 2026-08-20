# 文件：CMakeLists.txt
# 作用：构建一个覆盖 MuJoCo、RobotIO 和 MotionRuntime 的闭环测试。

add_library(quadruped_mujoco_test_helpers STATIC test_helpers.cpp)
target_link_libraries(quadruped_mujoco_test_helpers PRIVATE quadruped::config quadruped::mujoco)
target_compile_features(quadruped_mujoco_test_helpers PRIVATE cxx_std_17)
target_compile_definitions(quadruped_mujoco_test_helpers PRIVATE
    QUADRUPED_ROBOT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/robots/black.yaml"
)

add_executable(quadruped_mujoco_tests mujoco_motion_tests.cpp)
target_link_libraries(quadruped_mujoco_tests PRIVATE
    quadruped::config
    quadruped::motion
    quadruped::mujoco
    quadruped_mujoco_test_helpers
)
target_compile_features(quadruped_mujoco_tests PRIVATE cxx_std_17)
target_compile_definitions(quadruped_mujoco_tests PRIVATE
    QUADRUPED_BLACK_SCENE_PATH="${PROJECT_SOURCE_DIR}/assets/robots/black/mujoco/scene.xml"
    QUADRUPED_ROBOT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/robots/black.yaml"
    QUADRUPED_CONTROLLER_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/controllers/black.yaml"
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(quadruped_mujoco_tests PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
elseif(MSVC)
    target_compile_options(quadruped_mujoco_tests PRIVATE /W4)
endif()
add_test(NAME quadruped_mujoco_tests COMMAND quadruped_mujoco_tests)

if(QUADRUPED_ENABLE_TORCH)
    target_link_libraries(quadruped_mujoco_tests PRIVATE quadruped::torch_policy)
    target_compile_definitions(quadruped_mujoco_tests PRIVATE
        QUADRUPED_WITH_TORCH=1
        QUADRUPED_PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
        QUADRUPED_POLICY_FLAT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/policies/black/flat.yaml"
    )
endif()
