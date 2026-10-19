#!/usr/bin/env bash
# 文件：ros2.sh
# 作用：按仓库 ROS 2 配置构建接口和网关包到对应发行版的工作区。

set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "${project_dir}/scripts/ros2_env.sh"
quadruped_load_ros_config
workspace_root="${quadruped_ros_workspace}"
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

quadruped_source_ros_base

if [[ "${clean_build}" == true && -d "${workspace_root}" ]]; then
    cmake -E remove_directory "${workspace_root}"
fi

if [[ -f "${install_dir}/setup.bash" ]]; then
    if [[ ! -f "${install_dir}/.quadruped_ros_distro" ]] ||
        [[ "$(<"${install_dir}/.quadruped_ros_distro")" != "${QUADRUPED_ROS_DISTRO}" ]]; then
        echo "ROS 2 工作区包含旧版或其他发行版产物，请使用 --clean 重新构建。" >&2
        exit 1
    fi
fi

mkdir -p "${src_dir}"
ln -sfn "${project_dir}/adapters/ros2/quadruped_interfaces" \
    "${src_dir}/quadruped_interfaces"
ln -sfn "${project_dir}/adapters/ros2/quadruped_gateway" \
    "${src_dir}/quadruped_gateway"

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
    --symlink-install \
    --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

printf '%s\n' "${QUADRUPED_ROS_DISTRO}" >"${install_dir}/.quadruped_ros_distro"
quadruped_source_ros_workspace "${install_dir}/setup.bash"
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
