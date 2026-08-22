# 文件：CMakeLists.txt
# 作用：构建阶段 6 的 motiond 和无界面 MuJoCo 后端进程。

if(QUADRUPED_ENABLE_MUJOCO)
    add_executable(quadruped_mujoco_backendd mujoco_backendd.cpp)
    target_link_libraries(quadruped_mujoco_backendd PRIVATE
        quadruped::config
        quadruped::ipc
        quadruped::mujoco
        quadruped_mujoco_gui
    )
    target_include_directories(quadruped_mujoco_backendd PRIVATE
        "${PROJECT_SOURCE_DIR}/apps/mujoco_sim"
    )
    target_compile_features(quadruped_mujoco_backendd PRIVATE cxx_std_17)
    target_compile_definitions(quadruped_mujoco_backendd PRIVATE
        QUADRUPED_DEFAULT_SCENE_PATH="${PROJECT_SOURCE_DIR}/assets/robots/black/mujoco/scene.xml"
        QUADRUPED_DEFAULT_ROBOT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/robots/black.yaml"
    )
endif()

if(QUADRUPED_ENABLE_TORCH)
    add_executable(quadruped_motiond motiond.cpp)
    target_link_libraries(quadruped_motiond PRIVATE
        quadruped::config
        quadruped::ipc
        quadruped::motion
        quadruped::torch_policy
    )
    target_compile_features(quadruped_motiond PRIVATE cxx_std_17)
    target_compile_definitions(quadruped_motiond PRIVATE
        QUADRUPED_PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
        QUADRUPED_DEFAULT_ROBOT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/robots/black.yaml"
        QUADRUPED_DEFAULT_CONTROLLER_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/controllers/black.yaml"
        QUADRUPED_FLAT_POLICY_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/policies/black/flat.yaml"
        QUADRUPED_OBSTACLE_POLICY_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/policies/black/obstacle.yaml"
    )
endif()

foreach(target quadruped_mujoco_backendd quadruped_motiond)
    if(TARGET ${target})
        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
        elseif(MSVC)
            target_compile_options(${target} PRIVATE /W4)
        endif()
    endif()
endforeach()
