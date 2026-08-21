# 文件：CMakeLists.txt
# 作用：构建连接 ROS 2、共享内存和三进程运动运行时的顶层网关。

cmake_minimum_required(VERSION 3.8)
project(quadruped_gateway)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quadruped_interfaces REQUIRED)

file(REAL_PATH "${CMAKE_CURRENT_SOURCE_DIR}" QUADRUPED_GATEWAY_SOURCE_DIR)
set(QUADRUPED_SOURCE_ROOT "${QUADRUPED_GATEWAY_SOURCE_DIR}/../../..")

add_library(quadruped_gateway_ipc
    "${QUADRUPED_SOURCE_ROOT}/adapters/ipc/src/conversions.cpp"
    "${QUADRUPED_SOURCE_ROOT}/adapters/ipc/src/shared_memory.cpp"
)
target_include_directories(quadruped_gateway_ipc PUBLIC
    "${QUADRUPED_SOURCE_ROOT}/core/include"
    "${QUADRUPED_SOURCE_ROOT}/adapters/ipc/include"
)
target_compile_features(quadruped_gateway_ipc PUBLIC cxx_std_17)
target_compile_options(quadruped_gateway_ipc PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
target_link_libraries(quadruped_gateway_ipc PUBLIC rt pthread)

add_executable(ros2_gateway src/ros2_gateway.cpp)
target_compile_features(ros2_gateway PRIVATE cxx_std_17)
target_compile_options(ros2_gateway PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
target_link_libraries(ros2_gateway quadruped_gateway_ipc)
ament_target_dependencies(ros2_gateway
    rclcpp
    rclcpp_action
    geometry_msgs
    quadruped_interfaces
)

add_executable(quadruped_ipc_control src/ipc_control.cpp)
target_compile_features(quadruped_ipc_control PRIVATE cxx_std_17)
target_compile_options(quadruped_ipc_control PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
target_link_libraries(quadruped_ipc_control PRIVATE quadruped_gateway_ipc)

install(TARGETS ros2_gateway quadruped_ipc_control
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
