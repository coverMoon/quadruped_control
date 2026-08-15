# 文件：apps/mujoco_headless/CMakeLists.txt
# 作用：构建最小无界面 MuJoCo 仿真可执行程序。

add_executable(quadruped_mujoco_headless
    main.cpp
)

target_link_libraries(quadruped_mujoco_headless
    PRIVATE
        quadruped::mujoco
)

target_compile_features(quadruped_mujoco_headless PRIVATE cxx_std_17)

# 编译期固定默认场景路径，程序运行时不需要从当前目录查找仓库。
target_compile_definitions(quadruped_mujoco_headless
    PRIVATE
        QUADRUPED_DEFAULT_SCENE_PATH="${PROJECT_SOURCE_DIR}/assets/robots/black/mujoco/scene.xml"
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(quadruped_mujoco_headless PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
elseif(MSVC)
    target_compile_options(quadruped_mujoco_headless PRIVATE /W4)
endif()
