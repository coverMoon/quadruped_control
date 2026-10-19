# 文件：Ros2Config.cmake
# 作用：读取仓库 ROS 2 发行版配置并校验构建环境和缓存。

get_filename_component(QUADRUPED_SOURCE_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." REALPATH)
set(quadruped_ros_config "${QUADRUPED_SOURCE_ROOT}/rosconfig")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${quadruped_ros_config}")
file(READ "${quadruped_ros_config}" QUADRUPED_ROS_DISTRO)
string(REGEX REPLACE "\n+$" "" QUADRUPED_ROS_DISTRO "${QUADRUPED_ROS_DISTRO}")
if(NOT QUADRUPED_ROS_DISTRO MATCHES "^(humble|jazzy)$")
    message(FATAL_ERROR "rosconfig 只能填写 humble 或 jazzy")
endif()
if(NOT "$ENV{ROS_DISTRO}" STREQUAL "${QUADRUPED_ROS_DISTRO}")
    message(FATAL_ERROR
        "rosconfig=${QUADRUPED_ROS_DISTRO}，但 ROS_DISTRO=$ENV{ROS_DISTRO}；请加载对应 ROS 2 环境")
endif()
if(DEFINED QUADRUPED_CONFIGURED_ROS_DISTRO AND
   NOT QUADRUPED_CONFIGURED_ROS_DISTRO STREQUAL QUADRUPED_ROS_DISTRO)
    message(FATAL_ERROR "ROS 2 发行版已切换，请使用独立工作区或清理旧构建目录")
endif()
set(QUADRUPED_CONFIGURED_ROS_DISTRO "${QUADRUPED_ROS_DISTRO}" CACHE INTERNAL
    "当前构建目录的 ROS 2 发行版")
message(STATUS "quadruped ROS 2: ${QUADRUPED_ROS_DISTRO}")
