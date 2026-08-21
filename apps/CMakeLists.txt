# 文件：apps/CMakeLists.txt
# 作用：按需注册 MuJoCo 调试应用和三进程运行程序。

if(QUADRUPED_ENABLE_MUJOCO)
    add_subdirectory(runtime_daemons)
    add_subdirectory(mujoco_headless)
    add_subdirectory(mujoco_sim)
endif()
