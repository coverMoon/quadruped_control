#!/usr/bin/env bash
# 文件：command.sh
# 作用：按输入方式启动 ROS 2 指令网关，并复刻旧 rl_sar 的交互入口。

set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
robot_name="black"
input_mode="keyboard"
shm_name="${QUADRUPED_SHM_NAME:-}"
extra_ros_args=()
controller_pid=""
gateway_pid=""
positional_count=0

usage() {
    cat <<USAGE
用法: $0 [robot] [keyboard|joystick]
     $0 [keyboard|joystick]

位置参数:
  robot                 机器人名称，默认 black；可以省略
  keyboard              使用终端键盘，默认值；可直接作为第一个参数
  joystick              使用手柄（兼容 gamepad 别名）；可直接作为第一个参数

保留选项:
  --shm NAME            覆盖共享内存名称，主要用于测试或多实例运行
  --ros-args ...        将后续参数传给 ros2_gateway
USAGE
}

cleanup() {
    if [[ -n "${gateway_pid}" ]] && kill -0 "${gateway_pid}" 2>/dev/null; then
        kill -TERM "${gateway_pid}" 2>/dev/null || true
        wait "${gateway_pid}" 2>/dev/null || true
    fi
    if [[ -n "${controller_pid}" ]] && kill -0 "${controller_pid}" 2>/dev/null; then
        kill -TERM "${controller_pid}" 2>/dev/null || true
        wait "${controller_pid}" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

while (($# > 0)); do
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        --shm)
            [[ $# -ge 2 ]] || { echo "--shm 缺少参数" >&2; exit 2; }
            shm_name="$2"
            shift 2
            ;;
        --ros-args)
            extra_ros_args+=("$1")
            shift
            while (($# > 0)); do
                extra_ros_args+=("$1")
                shift
            done
            ;;
        --*)
            echo "未知选项: $1；主入口只使用位置参数选择输入方式" >&2
            usage >&2
            exit 2
            ;;
        *)
            if [[ ${positional_count} -eq 0 ]]; then
                # 输入方式是最常用的单参数场景，允许 command.sh joystick 直接使用默认机器人。
                case "$1" in
                    keyboard|joystick|gamepad)
                        input_mode="$1"
                        ;;
                    *)
                        robot_name="$1"
                        ;;
                esac
            elif [[ ${positional_count} -eq 1 ]]; then
                input_mode="$1"
            else
                echo "多余位置参数: $1" >&2
                exit 2
            fi
            positional_count=$((positional_count + 1))
            shift
            ;;
    esac
done

case "${input_mode}" in
    keyboard)
        keyboard_enabled=true
        start_controller=false
        ;;
    joystick|gamepad)
        keyboard_enabled=false
        start_controller=true
        ;;
    *)
        echo "不支持的输入方式: ${input_mode}（可选 keyboard|joystick）" >&2
        exit 2
        ;;
esac

if [[ -z "${shm_name}" ]]; then
    shm_name="/quadruped_control_${robot_name}"
fi
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
ros_workspace_root="${QUADRUPED_ROS2_WORKSPACE_ROOT:-${project_dir}/.build/ros2}"
ros_install_setup="${QUADRUPED_ROS2_INSTALL_SETUP:-${ros_workspace_root}/install/setup.bash}"
ros_install_dir="$(dirname "${ros_install_setup}")"
gateway="${ros_install_dir}/quadruped_gateway/lib/quadruped_gateway/ros2_gateway"

if [[ ! -f "${ros_setup}" ]]; then
    echo "未找到系统 ROS 2 基础环境: ${ros_setup}" >&2
    echo "请先安装 ROS 2 Humble，或通过 ROS_SETUP 指定 setup.bash。" >&2
    exit 1
fi

if [[ ! -f "${ros_install_setup}" || ! -x "${gateway}" ]]; then
    echo "[command] ROS 2 工程环境不存在或不完整，开始自动构建安装。"
    ROS_SETUP="${ros_setup}" \
        QUADRUPED_ROS2_WORKSPACE_ROOT="${ros_workspace_root}" \
        "${project_dir}/scripts/setup/ros2.sh"
fi

if [[ ! -f "${ros_install_setup}" || ! -x "${gateway}" ]]; then
    echo "ROS 2 gateway 安装失败: ${gateway}" >&2
    exit 1
fi

set +u
source "${ros_setup}"
source "${ros_install_setup}"
set -u
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/quadruped_control_ros2_logs}"
mkdir -p "${ROS_LOG_DIR}"

echo "[command] robot=${robot_name} input=${input_mode} shm=${shm_name}"

if [[ "${start_controller}" == true ]]; then
    controller_script="${project_dir}/adapters/ros2/quadruped_gateway/scripts/controller_input.py"
    controller_config="${project_dir}/configs/input/gamepads.yaml"
    controller_log="${ROS_LOG_DIR}/controller_input.log"
    /usr/bin/python3 "${controller_script}" \
        --config "${controller_config}" \
        --joy-topic /joy >"${controller_log}" 2>&1 &
    controller_pid=$!
fi

joy_require_connection_frame=true
if [[ "${start_controller}" != true ]]; then
    joy_require_connection_frame=false
fi
fixed_drive_keys_enabled=false
if [[ "${robot_name}" == "blackW" ]]; then
    fixed_drive_keys_enabled=true
fi
"${gateway}" \
    --ros-args \
    -p "shared_memory_name:=${shm_name}" \
    -p "keyboard_enabled:=${keyboard_enabled}" \
    -p "fixed_drive_keys_enabled:=${fixed_drive_keys_enabled}" \
    -p "joy_require_connection_frame:=${joy_require_connection_frame}" \
    -p "joy_topic:=/joy" \
    "${extra_ros_args[@]}" &
gateway_pid=$!
set +e
wait "${gateway_pid}"
status=$?
set -e
gateway_pid=""
exit "${status}"
