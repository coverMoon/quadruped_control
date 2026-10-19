#!/usr/bin/env bash
# 文件：ros2_env.sh
# 作用：为构建、运行和交互终端统一读取 ROS 2 配置并加载匹配环境。

quadruped_ros_source_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)" || return 1

quadruped_load_ros_config() {
    local source_root="${quadruped_ros_source_root}"
    if [[ ! -f "${source_root}/rosconfig" ]]; then
        echo "缺少 ROS 2 配置: ${source_root}/rosconfig" >&2
        return 1
    fi
    QUADRUPED_ROS_DISTRO="$(<"${source_root}/rosconfig")"
    case "${QUADRUPED_ROS_DISTRO}" in
        humble|jazzy) ;;
        *) echo "rosconfig 只能填写 humble 或 jazzy" >&2; return 1 ;;
    esac
    quadruped_ros_setup="${ROS_SETUP:-/opt/ros/${QUADRUPED_ROS_DISTRO}/setup.bash}"
    quadruped_ros_workspace="${QUADRUPED_ROS2_WORKSPACE_ROOT:-${source_root}/.build/ros2/${QUADRUPED_ROS_DISTRO}}"
}

quadruped_source_ros_base() {
    quadruped_load_ros_config || return 1
    if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "${QUADRUPED_ROS_DISTRO}" ]]; then
        echo "当前 ROS_DISTRO=${ROS_DISTRO} 与 rosconfig=${QUADRUPED_ROS_DISTRO} 不一致，请使用新终端。" >&2
        return 1
    fi
    if [[ ! -f "${quadruped_ros_setup}" ]]; then
        echo "未找到 ROS 2 ${QUADRUPED_ROS_DISTRO} 环境: ${quadruped_ros_setup}" >&2
        return 1
    fi
    quadruped_source_ros_file "${quadruped_ros_setup}" || return 1
    if [[ "${ROS_DISTRO:-}" != "${QUADRUPED_ROS_DISTRO}" ]]; then
        echo "ROS_SETUP 加载的发行版与 rosconfig=${QUADRUPED_ROS_DISTRO} 不一致。" >&2
        return 1
    fi
}

quadruped_source_ros_file() {
    # ROS 生成的环境脚本可能读取未定义变量，保留调用方的 nounset 设置。
    local restore_nounset=false
    local source_status=0
    if [[ $- == *u* ]]; then
        restore_nounset=true
    fi
    set +u
    # shellcheck disable=SC1090
    source "$1" || source_status=$?
    if [[ "${restore_nounset}" == true ]]; then
        set -u
    fi
    return "${source_status}"
}

quadruped_source_ros_workspace() {
    quadruped_source_ros_base || return 1
    local install_setup="${1:-${QUADRUPED_ROS2_INSTALL_SETUP:-${quadruped_ros_workspace}/install/setup.bash}}"
    local distro_marker
    distro_marker="$(dirname -- "${install_setup}")/.quadruped_ros_distro"
    if [[ ! -f "${install_setup}" || ! -f "${distro_marker}" ]]; then
        echo "缺少匹配的 ROS 2 安装环境，请运行 ./scripts/build.sh --target command。" >&2
        return 1
    fi
    if [[ "$(<"${distro_marker}")" != "${QUADRUPED_ROS_DISTRO}" ]]; then
        echo "ROS 2 安装目录与 rosconfig=${QUADRUPED_ROS_DISTRO} 不一致，请切换工作区。" >&2
        return 1
    fi
    quadruped_source_ros_file "${install_setup}" || return 1
    if [[ "${ROS_DISTRO:-}" != "${QUADRUPED_ROS_DISTRO}" ]]; then
        echo "ROS 2 安装环境引用了其他发行版，请清理该 ROS 2 工作区后重新构建。" >&2
        return 1
    fi
}
