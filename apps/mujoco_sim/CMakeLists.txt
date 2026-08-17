# 文件：CMakeLists.txt
# 作用：构建带 GLFW 界面的交互式 MuJoCo 仿真程序 quadruped_mujoco_sim。

find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(quadruped_mujoco_sim
    main.cpp
    sim_controller.cpp
    sim_window.cpp
)

target_link_libraries(quadruped_mujoco_sim
    PRIVATE
        quadruped::config
        quadruped::motion
        quadruped::mujoco
        glfw
        OpenGL::GL
)

target_compile_features(quadruped_mujoco_sim PRIVATE cxx_std_17)

# 编译期固定默认场景和配置路径，程序运行时不需要从当前目录查找仓库。
target_compile_definitions(quadruped_mujoco_sim
    PRIVATE
        QUADRUPED_DEFAULT_SCENE_PATH="${PROJECT_SOURCE_DIR}/assets/robots/black/mujoco/scene.xml"
        QUADRUPED_DEFAULT_ROBOT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/robots/black.yaml"
        QUADRUPED_DEFAULT_CONTROLLER_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/controllers/black.yaml"
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(quadruped_mujoco_sim PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
elseif(MSVC)
    target_compile_options(quadruped_mujoco_sim PRIVATE /W4)
endif()
