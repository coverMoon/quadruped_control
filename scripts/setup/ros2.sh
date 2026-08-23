#!/usr/bin/env bash
# 文件：ros2.sh
# 作用：构建并安装仓库的 ROS 2 接口和网关包到固定工程构建目录。

set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
workspace_root="${QUADRUPED_ROS2_WORKSPACE_ROOT:-${project_dir}/.build/ros2}"
clean_build=false

while (($# > 0)); do
    case "$1" in
        --clean)
            clean_build=true
            shift
            ;;
        -h|--help)
            cat <<USAGE
用法: $0 [--clean]

作用：将仓库的 ROS 2 接口和网关包安装到：
  ${workspace_root}/install
USAGE
            exit 0
            ;;
        *)
            echo "未知选项: $1" >&2
            exit 2
            ;;
    esac
done

src_dir="${workspace_root}/src"
build_dir="${workspace_root}/build"
install_dir="${workspace_root}/install"
log_dir="${workspace_root}/log"

if [[ ! -f "${ros_setup}" ]]; then
    echo "未找到 ROS 2 环境脚本: ${ros_setup}" >&2
    exit 1
fi

if [[ "${clean_build}" == true && -d "${workspace_root}" ]]; then
    cmake -E remove_directory "${workspace_root}"
fi

mkdir -p "${src_dir}"
ln -sfn "${project_dir}/adapters/ros2/quadruped_interfaces" \
    "${src_dir}/quadruped_interfaces"
ln -sfn "${project_dir}/adapters/ros2/quadruped_gateway" \
    "${src_dir}/quadruped_gateway"

# ROS 环境脚本可能读取未预先定义的变量，source 期间暂时关闭 nounset。
set +u
# shellcheck disable=SC1090
source "${ros_setup}"
set -u

package_types="$(colcon list --base-paths "${src_dir}")"
if grep -q 'ros.catkin' <<< "${package_types}"; then
    echo "ROS 2 包被错误识别为 catkin 包:" >&2
    echo "${package_types}" >&2
    exit 1
fi

colcon --log-base "${log_dir}" build \
    --base-paths "${src_dir}" \
    --build-base "${build_dir}" \
    --install-base "${install_dir}" \
    --packages-select quadruped_interfaces quadruped_gateway \
    --symlink-install

set +u
# shellcheck disable=SC1090
source "${install_dir}/setup.bash"
set -u
ros2 pkg prefix quadruped_interfaces >/dev/null
ros2 pkg prefix quadruped_gateway >/dev/null
launch_file="${install_dir}/quadruped_gateway/share/quadruped_gateway/launch/black_simulation.launch.py"
if [[ ! -f "${launch_file}" ]]; then
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

printf 'ROS 2 包环境安装完成。\n安装环境: %s\n' "${install_dir}/setup.bash"
