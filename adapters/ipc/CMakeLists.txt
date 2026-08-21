# 文件：CMakeLists.txt
# 作用：构建 Linux 本机三进程运行链路使用的固定容量共享内存适配器。

add_library(quadruped_ipc
    src/conversions.cpp
    src/remote_robot_io.cpp
    src/shared_memory.cpp
)

add_library(quadruped::ipc ALIAS quadruped_ipc)

target_include_directories(quadruped_ipc
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(quadruped_ipc PUBLIC quadruped::core)
target_compile_features(quadruped_ipc PUBLIC cxx_std_17)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(quadruped_ipc PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
elseif(MSVC)
    target_compile_options(quadruped_ipc PRIVATE /W4)
endif()
