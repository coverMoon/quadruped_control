#!/usr/bin/env bash
# 文件：command.sh
# 作用：独立启动 ROS 2 顶层指令网关、键盘输入和可选手柄适配器。

set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
robot_name="black"
shm_name=""
keyboard="auto"
controller="auto"
controller_profile=""
controller_config="${project_dir}/configs/input/gamepads.yaml"
joy_topic="/joy"
cmd_vel_timeout_ns="200000000"
joy_timeout_ns="250000000"
extra_ros_args=()
controller_pid=""
gateway_pid=""
positional_count=0

usage() {
    cat <<USAGE
用法: $0 [robot] [选项]

选项:
  --robot NAME
  --shm NAME
  --keyboard on|off|auto    TTY 默认 auto，非 TTY 自动 off
  --controller auto|off|external|explicit
  --controller-profile NAME explicit 模式的 profile
  --controller-config PATH
  --joy-topic TOPIC
  --cmd-vel-timeout-ns NS
  --joy-timeout-ns NS
  其他 --ros-args ...       传给 ros2_gateway
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
        --robot)
            [[ $# -ge 2 ]] || { echo "--robot 缺少参数" >&2; exit 2; }
            robot_name="$2"; shift 2
            ;;
        --shm)
            [[ $# -ge 2 ]] || { echo "--shm 缺少参数" >&2; exit 2; }
            shm_name="$2"; shift 2
            ;;
        --keyboard)
            [[ $# -ge 2 ]] || { echo "--keyboard 缺少参数" >&2; exit 2; }
            keyboard="$2"; shift 2
            ;;
        --controller)
            [[ $# -ge 2 ]] || { echo "--controller 缺少参数" >&2; exit 2; }
            controller="$2"; shift 2
            ;;
        --controller-profile)
            [[ $# -ge 2 ]] || { echo "--controller-profile 缺少参数" >&2; exit 2; }
            controller_profile="$2"; shift 2
            ;;
        --controller-config)
            [[ $# -ge 2 ]] || { echo "--controller-config 缺少参数" >&2; exit 2; }
            controller_config="$2"; shift 2
            ;;
        --joy-topic)
            [[ $# -ge 2 ]] || { echo "--joy-topic 缺少参数" >&2; exit 2; }
            joy_topic="$2"; shift 2
            ;;
        --cmd-vel-timeout-ns)
            [[ $# -ge 2 ]] || { echo "--cmd-vel-timeout-ns 缺少参数" >&2; exit 2; }
            cmd_vel_timeout_ns="$2"; shift 2
            ;;
        --joy-timeout-ns)
            [[ $# -ge 2 ]] || { echo "--joy-timeout-ns 缺少参数" >&2; exit 2; }
            joy_timeout_ns="$2"; shift 2
            ;;
        --ros-args)
            extra_ros_args+=("$1"); shift
            while (($# > 0)); do extra_ros_args+=("$1"); shift; done
            ;;
        --*)
            echo "未知选项: $1" >&2
            usage >&2
            exit 2
            ;;
        *)
            if [[ ${positional_count} -ne 0 ]]; then
                echo "多余位置参数: $1" >&2
                exit 2
            fi
            robot_name="$1"
            positional_count=1
            shift
            ;;
    esac
done

case "${keyboard}" in
    auto) [[ -t 0 ]] && keyboard_enabled=true || keyboard_enabled=false ;;
    on) keyboard_enabled=true ;;
    off) keyboard_enabled=false ;;
    *) echo "--keyboard 必须是 on|off|auto" >&2; exit 2 ;;
esac
case "${controller}" in auto|off|external|explicit) ;; *) echo "不支持的 controller: ${controller}" >&2; exit 2 ;; esac
if [[ "${controller}" == "explicit" && -z "${controller_profile}" ]]; then
    echo "--controller explicit 必须同时提供 --controller-profile" >&2
    exit 2
fi
if [[ -z "${shm_name}" ]]; then shm_name="/quadruped_control_${robot_name}"; fi
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
ros_workspace_root="${QUADRUPED_ROS2_WORKSPACE_ROOT:-/tmp/quadruped_control_ros2_ws_stage8}"
ros_install_setup="${QUADRUPED_ROS2_INSTALL_SETUP:-${ros_workspace_root}/latest/install/setup.bash}"
ros_install_dir="$(dirname "${ros_install_setup}")"
gateway="${ros_install_dir}/quadruped_gateway/lib/quadruped_gateway/ros2_gateway"
if [[ ! -f "${ros_setup}" || ! -f "${ros_install_setup}" || ! -x "${gateway}" ]]; then
    echo "缺少 ROS 2 gateway 产物或环境，请先运行 ./scripts/build.sh --target command" >&2
    exit 1
fi
if [[ "${controller}" == "auto" || "${controller}" == "explicit" ]]; then
    controller_script="${project_dir}/adapters/ros2/quadruped_gateway/scripts/controller_input.py"
    controller_args=("${controller_script}" --config "${controller_config}" --joy-topic "${joy_topic}")
    [[ "${controller}" == "explicit" ]] && controller_args+=(--profile "${controller_profile}")
    set +u; source "${ros_setup}"; source "${ros_install_setup}"; set -u
    /usr/bin/python3 "${controller_args[@]}" & controller_pid=$!
else
    set +u; source "${ros_setup}"; source "${ros_install_setup}"; set -u
fi
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/quadruped_control_ros2_logs}"
mkdir -p "${ROS_LOG_DIR}"
joy_require_connection_frame=true
if [[ "${controller}" == "external" ]]; then
    joy_require_connection_frame=false
fi
"${gateway}" \
    --ros-args \
    -p "shared_memory_name:=${shm_name}" \
    -p "cmd_vel_timeout_ns:=${cmd_vel_timeout_ns}" \
    -p "joy_timeout_ns:=${joy_timeout_ns}" \
    -p "keyboard_enabled:=${keyboard_enabled}" \
    -p "joy_require_connection_frame:=${joy_require_connection_frame}" \
    -p "joy_topic:=${joy_topic}" \
    "${extra_ros_args[@]}" &
gateway_pid=$!
set +e
wait "${gateway_pid}"
status=$?
set -e
gateway_pid=""
exit "${status}"
