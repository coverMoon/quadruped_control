# 文件：CMakeLists.txt
# 作用：构建只依赖 core 的只读日志回放 RobotIO。

add_library(quadruped_replay_backend
    src/diagnostic_log.cpp
    src/replay_log.cpp
    src/replay_robot_io.cpp
)

add_library(quadruped::replay ALIAS quadruped_replay_backend)

target_include_directories(quadruped_replay_backend
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(quadruped_replay_backend PUBLIC quadruped::core quadruped::motion)
target_compile_features(quadruped_replay_backend PUBLIC cxx_std_17)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(quadruped_replay_backend PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
elseif(MSVC)
    target_compile_options(quadruped_replay_backend PRIVATE /W4)
endif()
