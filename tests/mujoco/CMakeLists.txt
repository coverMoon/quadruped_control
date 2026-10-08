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
    QUADRUPED_BLACK_TERRAIN_SCENE_PATH="${PROJECT_SOURCE_DIR}/assets/robots/black/mujoco/scene_terrain.xml"
    QUADRUPED_BLACK_NWBT_SCENE_PATH="${PROJECT_SOURCE_DIR}/assets/robots/black/mujoco/scene_nwbt.xml"
    QUADRUPED_BLACK_DOG26_SCENE_PATH="${PROJECT_SOURCE_DIR}/assets/robots/black/mujoco/scene_dog26.xml"
    QUADRUPED_BLACK_DOG27_SCENE_PATH="${PROJECT_SOURCE_DIR}/assets/robots/black/mujoco/scene_dog27.xml"
    QUADRUPED_BLACKW_SCENE_PATH="${PROJECT_SOURCE_DIR}/assets/robots/blackW/mujoco/scene.xml"
    QUADRUPED_BLACKW_TERRAIN_SCENE_PATH="${PROJECT_SOURCE_DIR}/assets/robots/blackW/mujoco/scene_terrain.xml"
    QUADRUPED_BLACKW_NWBT_SCENE_PATH="${PROJECT_SOURCE_DIR}/assets/robots/blackW/mujoco/scene_nwbt.xml"
    QUADRUPED_BLACKW_DOG26_SCENE_PATH="${PROJECT_SOURCE_DIR}/assets/robots/blackW/mujoco/scene_dog26.xml"
    QUADRUPED_BLACKW_DOG27_SCENE_PATH="${PROJECT_SOURCE_DIR}/assets/robots/blackW/mujoco/scene_dog27.xml"
    QUADRUPED_BLACKW_ROBOT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/robots/blackW.yaml"
    QUADRUPED_BLACKW_CONTROLLER_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/controllers/blackW.yaml"
    QUADRUPED_BLACKW_RETRY_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/behaviors/blackW/retry.yaml"
    QUADRUPED_BLACKW_EVENT_CHAIN_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/behaviors/blackW/event_chain.yaml"
    QUADRUPED_BLACKW_BRIDGE_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/behaviors/blackW/bridge_drive.yaml"
    QUADRUPED_BLACKW_LOW_BAR_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/behaviors/blackW/low_bar_drive.yaml"
    QUADRUPED_BLACKW_CAR_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/behaviors/blackW/car_drive.yaml"
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
        QUADRUPED_POLICY_OBSTACLE_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/policies/black/obstacle.yaml"
        QUADRUPED_BLACKW_POLICY_FLAT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/policies/blackW/flat.yaml"
        QUADRUPED_BLACKW_POLICY_OBSTACLE_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/policies/blackW/obstacle.yaml"
        QUADRUPED_BLACKW_POLICY_STAIR_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/policies/blackW/stair.yaml"
    )
endif()
