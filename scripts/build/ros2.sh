#!/usr/bin/env bash
# 文件：ros2.sh
# 作用：在仓库外的独立临时工作区构建 ROS 2 接口和网关包。

set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
workspace_root="${QUADRUPED_ROS2_WORKSPACE_ROOT:-/tmp/quadruped_control_ros2_ws_stage8}"
run_id="run-$(date +%Y%m%d-%H%M%S)-$$"
run_dir="${workspace_root}/${run_id}"

if [[ ! -f "${ros_setup}" ]]
then
    echo "未找到 ROS 2 环境脚本: ${ros_setup}" >&2
    exit 1
fi

mkdir -p "${run_dir}/src"
ln -s "${project_dir}/adapters/ros2/quadruped_interfaces" \
    "${run_dir}/src/quadruped_interfaces"
ln -s "${project_dir}/adapters/ros2/quadruped_gateway" \
    "${run_dir}/src/quadruped_gateway"

# ROS 环境脚本会读取若干未预先定义的变量，source 期间暂时关闭 nounset。
set +u
# shellcheck disable=SC1090
source "${ros_setup}"
set -u

package_types="$(colcon list --base-paths "${run_dir}/src")"
if grep -q 'ros.catkin' <<< "${package_types}"
then
    echo "ROS 2 包被错误识别为 catkin 包:" >&2
    echo "${package_types}" >&2
    exit 1
fi

colcon --log-base "${run_dir}/log" build \
    --base-paths "${run_dir}/src" \
    --build-base "${run_dir}/build" \
    --install-base "${run_dir}/install" \
    --packages-select quadruped_interfaces quadruped_gateway \
    --symlink-install

ln -sfn "${run_dir}" "${workspace_root}/latest"

set +u
# shellcheck disable=SC1090
source "${run_dir}/install/setup.bash"
set -u
ros2 pkg prefix quadruped_interfaces >/dev/null
ros2 pkg prefix quadruped_gateway >/dev/null
launch_file="${run_dir}/install/quadruped_gateway/share/quadruped_gateway/launch/black_simulation.launch.py"
if [[ ! -f "${launch_file}" ]]
then
    echo "缺少 ROS 2 仿真 launch 文件: ${launch_file}" >&2
    exit 1
fi
ros2 interface show quadruped_interfaces/action/GetUp >/dev/null
/usr/bin/python3 - <<'PY'
import rclpy
from quadruped_interfaces.action import GetUp

assert GetUp.Goal is not None
assert rclpy.ok() is False
PY

printf 'ROS 2 构建完成。\n安装环境: %s\n' "${run_dir}/install/setup.bash"
