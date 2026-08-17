# 文件：apps/CMakeLists.txt
# 作用：按需注册可选应用程序；目前只有 MuJoCo 无界面程序。

if(QUADRUPED_ENABLE_MUJOCO)
    add_subdirectory(mujoco_headless)
    add_subdirectory(mujoco_sim)
endif()
