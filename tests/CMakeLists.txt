# 文件：CMakeLists.txt
# 作用：构建少量按模块组织的自动测试。

function(quadruped_add_test target source)
    add_executable(${target} ${source})
    target_compile_features(${target} PRIVATE cxx_std_17)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
    elseif(MSVC)
        target_compile_options(${target} PRIVATE /W4)
    endif()
    add_test(NAME ${target} COMMAND ${target})
endfunction()

quadruped_add_test(quadruped_core_tests core_tests.cpp)
target_link_libraries(quadruped_core_tests PRIVATE quadruped::core)

quadruped_add_test(quadruped_config_tests config_tests.cpp)
target_link_libraries(quadruped_config_tests PRIVATE quadruped::config)
target_compile_definitions(quadruped_config_tests PRIVATE
    QUADRUPED_ROBOT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/robots/black.yaml"
    QUADRUPED_CONTROLLER_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/controllers/black.yaml"
    QUADRUPED_SIMULATION_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/simulation/black_mujoco.yaml"
    QUADRUPED_POLICY_SWITCH_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/policies/black/policy_switch.yaml"
    QUADRUPED_RETRY_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/behaviors/black/retry.yaml"
    QUADRUPED_EVENT_CHAIN_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/behaviors/black/event_chain.yaml"
    QUADRUPED_BLACKW_ROBOT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/robots/blackW.yaml"
    QUADRUPED_BLACKW_CONTROLLER_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/controllers/blackW.yaml"
    QUADRUPED_BLACKW_SIMULATION_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/simulation/blackW_mujoco.yaml"
    QUADRUPED_BLACKW_POLICY_SWITCH_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/policies/blackW/policy_switch.yaml"
    QUADRUPED_BLACKW_RETRY_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/behaviors/blackW/retry.yaml"
    QUADRUPED_BLACKW_EVENT_CHAIN_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/behaviors/blackW/event_chain.yaml"
    QUADRUPED_BLACKW_BRIDGE_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/behaviors/blackW/bridge_drive.yaml"
    QUADRUPED_BLACKW_LOW_BAR_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/behaviors/blackW/low_bar_drive.yaml"
    QUADRUPED_BLACKW_CAR_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/behaviors/blackW/car_drive.yaml"
)

quadruped_add_test(quadruped_motion_tests motion_tests.cpp)
target_link_libraries(quadruped_motion_tests PRIVATE quadruped::motion)

quadruped_add_test(quadruped_replay_tests replay_tests.cpp)
target_link_libraries(quadruped_replay_tests PRIVATE quadruped::motion quadruped::replay)

quadruped_add_test(quadruped_ipc_tests ipc_tests.cpp)
target_link_libraries(quadruped_ipc_tests PRIVATE quadruped::config quadruped::ipc)
target_compile_definitions(quadruped_ipc_tests PRIVATE
    QUADRUPED_ROBOT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/robots/black.yaml"
)

quadruped_add_test(quadruped_rl_tests rl_tests.cpp)
target_link_libraries(quadruped_rl_tests PRIVATE quadruped::config quadruped::motion)
target_compile_definitions(quadruped_rl_tests PRIVATE
    QUADRUPED_PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
    QUADRUPED_ROBOT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/robots/black.yaml"
    QUADRUPED_POLICY_FLAT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/policies/black/flat.yaml"
    QUADRUPED_POLICY_OBSTACLE_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/policies/black/obstacle.yaml"
    QUADRUPED_BLACKW_ROBOT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/robots/blackW.yaml"
    QUADRUPED_BLACKW_POLICY_FLAT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/policies/blackW/flat.yaml"
    QUADRUPED_BLACKW_POLICY_OBSTACLE_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/policies/blackW/obstacle.yaml"
    QUADRUPED_BLACKW_POLICY_STAIR_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/policies/blackW/stair.yaml"
)

if(QUADRUPED_ENABLE_MUJOCO)
    add_subdirectory(mujoco)
endif()

if(QUADRUPED_ENABLE_TORCH)
    quadruped_add_test(quadruped_rl_torch_tests rl_torch_tests.cpp)
    target_link_libraries(quadruped_rl_torch_tests PRIVATE
        quadruped::config
        quadruped::torch_policy
    )
    target_include_directories(quadruped_rl_torch_tests PRIVATE ${TORCH_INCLUDE_DIRS})
    target_compile_options(quadruped_rl_torch_tests PRIVATE ${TORCH_CXX_FLAGS})
    target_compile_definitions(quadruped_rl_torch_tests PRIVATE
        QUADRUPED_PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
        QUADRUPED_ROBOT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/robots/black.yaml"
        QUADRUPED_POLICY_FLAT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/policies/black/flat.yaml"
        QUADRUPED_POLICY_OBSTACLE_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/policies/black/obstacle.yaml"
        QUADRUPED_BLACKW_ROBOT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/robots/blackW.yaml"
        QUADRUPED_BLACKW_POLICY_FLAT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/policies/blackW/flat.yaml"
        QUADRUPED_BLACKW_POLICY_OBSTACLE_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/policies/blackW/obstacle.yaml"
        QUADRUPED_BLACKW_POLICY_STAIR_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/policies/blackW/stair.yaml"
    )
endif()
