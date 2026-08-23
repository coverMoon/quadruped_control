#!/usr/bin/env bash
# 文件：ros2_headless.sh
# 作用：使用模块化正式入口运行三进程 ROS 2 无界面端到端测试。

set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
workspace_root="${QUADRUPED_ROS2_WORKSPACE_ROOT:-${project_dir}/.build/ros2}"
ros_install="${QUADRUPED_ROS2_INSTALL:-${workspace_root}/install}"
for required in "${ros_setup}" "${ros_install}/setup.bash" \
    "${project_dir}/.build/rl/apps/runtime_daemons/quadruped_mujoco_backendd" \
    "${project_dir}/.build/rl/apps/runtime_daemons/quadruped_motiond"; do
    [[ -e "${required}" ]] || { echo "缺少测试依赖: ${required}" >&2; exit 1; }
done
set +u; source "${ros_setup}"; source "${ros_install}/setup.bash"; set -u
export ROS_LOG_DIR="${QUADRUPED_ROS2_LOG_DIR:-/tmp/quadruped_control_ros2_logs}"
mkdir -p "${ROS_LOG_DIR}"
exec /usr/bin/python3 "${project_dir}/scripts/test/ros2_headless_test.py" \
    --backend-script "${project_dir}/scripts/run/backend.sh" \
    --motion-script "${project_dir}/scripts/run/motion.sh" \
    --command-script "${project_dir}/scripts/run/command.sh" \
    --ipc-control "${ros_install}/quadruped_gateway/lib/quadruped_gateway/quadruped_ipc_control" \
    --real-time-factor "${QUADRUPED_TEST_REAL_TIME_FACTOR:-1.0}"
