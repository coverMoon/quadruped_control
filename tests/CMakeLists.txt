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

add_executable(quadruped_config_tests
    config_tests.cpp
)

target_link_libraries(quadruped_config_tests PRIVATE quadruped::config)
target_compile_features(quadruped_config_tests PRIVATE cxx_std_17)
target_compile_definitions(quadruped_config_tests PRIVATE
    QUADRUPED_ROBOT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/robots/black.yaml"
    QUADRUPED_CONTROLLER_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/controllers/black.yaml"
)

add_executable(quadruped_config_controller_tests
    config_controller_tests.cpp
)

target_link_libraries(quadruped_config_controller_tests PRIVATE quadruped::config)
target_compile_features(quadruped_config_controller_tests PRIVATE cxx_std_17)
target_compile_definitions(quadruped_config_controller_tests PRIVATE
    QUADRUPED_ROBOT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/robots/black.yaml"
    QUADRUPED_CONTROLLER_CONFIG_PATH="${PROJECT_SOURCE_DIR}/configs/controllers/black.yaml"
)

add_executable(quadruped_motion_tests
    motion_tests.cpp
)

target_link_libraries(quadruped_motion_tests PRIVATE quadruped::motion)
target_compile_features(quadruped_motion_tests PRIVATE cxx_std_17)

add_executable(quadruped_motion_error_tests
    motion_error_tests.cpp
)

target_link_libraries(quadruped_motion_error_tests PRIVATE quadruped::motion)
target_compile_features(quadruped_motion_error_tests PRIVATE cxx_std_17)

add_executable(quadruped_motion_session_tests
    motion_session_tests.cpp
)

target_link_libraries(quadruped_motion_session_tests PRIVATE quadruped::motion)
target_compile_features(quadruped_motion_session_tests PRIVATE cxx_std_17)

add_executable(quadruped_motion_request_tests
    motion_request_tests.cpp
)

target_link_libraries(quadruped_motion_request_tests PRIVATE quadruped::motion)
target_compile_features(quadruped_motion_request_tests PRIVATE cxx_std_17)

add_executable(quadruped_motion_result_events_tests
    motion_result_events_tests.cpp
)

target_link_libraries(quadruped_motion_result_events_tests PRIVATE quadruped::motion)
target_compile_features(quadruped_motion_result_events_tests PRIVATE cxx_std_17)

foreach(target quadruped_config_tests quadruped_config_controller_tests quadruped_motion_tests quadruped_motion_error_tests quadruped_motion_session_tests quadruped_motion_request_tests quadruped_motion_result_events_tests)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
    elseif(MSVC)
        target_compile_options(${target} PRIVATE /W4)
    endif()
    add_test(NAME ${target} COMMAND ${target})
endforeach()

if(QUADRUPED_ENABLE_MUJOCO)
    add_subdirectory(mujoco)
endif()
