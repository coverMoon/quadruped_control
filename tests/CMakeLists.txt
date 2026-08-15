# 文件：tests/CMakeLists.txt
# 作用：构建并注册核心模块的自动测试。

add_executable(quadruped_core_tests
    core_tests.cpp
)

target_link_libraries(quadruped_core_tests PRIVATE quadruped::core)
target_compile_features(quadruped_core_tests PRIVATE cxx_std_17)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(quadruped_core_tests PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
elseif(MSVC)
    target_compile_options(quadruped_core_tests PRIVATE /W4)
endif()

add_test(NAME quadruped_core_tests COMMAND quadruped_core_tests)

if(QUADRUPED_ENABLE_MUJOCO)
    add_subdirectory(mujoco)
endif()
