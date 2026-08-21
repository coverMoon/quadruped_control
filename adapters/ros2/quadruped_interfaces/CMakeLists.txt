# 文件：CMakeLists.txt
# 作用：独立构建 quadruped_control 的 ROS 2 消息、动作和服务接口。

cmake_minimum_required(VERSION 3.8)

project(quadruped_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/ModeResult.msg"
    "msg/MotionStatus.msg"
    "msg/RobotIOStatus.msg"
    "msg/StateDiagnostic.msg"
    "action/GetUp.action"
    "action/GetDown.action"
    "action/StartBehavior.action"
    "action/SwitchPolicy.action"
    "srv/EnterPassive.srv"
    "srv/ResetFault.srv"
    DEPENDENCIES action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
