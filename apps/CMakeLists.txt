# 文件：apps/CMakeLists.txt
# 作用：按需注册 MuJoCo 调试应用和三进程运行程序。

add_subdirectory(replay)

if(QUADRUPED_ENABLE_MUJOCO)
    add_subdirectory(mujoco_headless)
    add_subdirectory(mujoco_sim)
endif()

if(QUADRUPED_ENABLE_MUJOCO OR QUADRUPED_ENABLE_TORCH)
    add_subdirectory(runtime_daemons)
endif()
