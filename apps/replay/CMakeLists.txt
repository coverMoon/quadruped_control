# 文件：CMakeLists.txt
# 作用：构建只读状态日志回放和诊断对比程序。

add_executable(quadruped_replay main.cpp)

target_link_libraries(quadruped_replay PRIVATE
    quadruped::config
    quadruped::motion
    quadruped::replay
)

target_compile_features(quadruped_replay PRIVATE cxx_std_17)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(quadruped_replay PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
elseif(MSVC)
    target_compile_options(quadruped_replay PRIVATE /W4)
endif()
