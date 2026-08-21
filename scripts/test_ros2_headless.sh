#!/usr/bin/env bash
# 文件：test_ros2_headless.sh
# 作用：使用已构建产物运行阶段 6 三进程 ROS 2 无界面端到端和故障测试。

set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
ros_workspace_root="${QUADRUPED_ROS2_WORKSPACE_ROOT:-/tmp/quadruped_control_ros2_ws_stage6}"
ros_install="${QUADRUPED_ROS2_INSTALL:-${ros_workspace_root}/latest/install}"
ros_log_dir="${QUADRUPED_ROS2_LOG_DIR:-/tmp/quadruped_control_ros2_logs}"
backend="${project_dir}/build/rl/apps/runtime_daemons/quadruped_mujoco_backendd"
motion="${project_dir}/build/rl/apps/runtime_daemons/quadruped_motiond"
gateway="${ros_install}/quadruped_gateway/lib/quadruped_gateway/ros2_gateway"
ipc_control="${ros_install}/quadruped_gateway/lib/quadruped_gateway/quadruped_ipc_control"
real_time_factor="${QUADRUPED_TEST_REAL_TIME_FACTOR:-1.0}"

for path in "${ros_setup}" "${ros_install}/setup.bash" \
    "${backend}" "${motion}" "${gateway}" "${ipc_control}"
do
    if [[ ! -e "${path}" ]]
    then
        echo "缺少阶段 6 测试依赖: ${path}" >&2
        echo "请先运行 ./scripts/build.sh --rl 和 ./scripts/build_ros2.sh。" >&2
        exit 1
    fi
done

mkdir -p "${ros_log_dir}"
export ROS_LOG_DIR="${ros_log_dir}"

set +u
# shellcheck disable=SC1090
source "${ros_setup}"
# shellcheck disable=SC1090
source "${ros_install}/setup.bash"
set -u

exec /usr/bin/python3 "${project_dir}/scripts/ros2_headless_test.py" \
    --backend "${backend}" \
    --motion "${motion}" \
    --gateway "${gateway}" \
    --ipc-control "${ipc_control}" \
    --real-time-factor "${real_time_factor}"
