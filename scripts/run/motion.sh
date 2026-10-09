#!/usr/bin/env bash
# 文件：motion.sh
# 作用：按机器人和策略名称启动统一的 MotionRuntime 与策略进程。

set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
robot_name="black"
policy_name=""
load_policy=true
shm_name="${QUADRUPED_SHM_NAME:-}"
positional_count=0

usage() {
    cat <<USAGE
用法: $0 [robot] [policy] [--no-policy]

位置参数:
  robot                 机器人名称，默认 black
  policy                初始策略名，默认使用 policy_switch.yaml 的第一个可用策略

环境变量:
  QUADRUPED_SHM_NAME   覆盖共享内存名称，主要用于测试或多实例运行
USAGE
}

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
        --no-policy)
            load_policy=false
            shift
            ;;
        --*)
            echo "未知选项: $1；配置路径由机器人和策略名称自动选择" >&2
            usage >&2
            exit 2
            ;;
        *)
            if [[ ${positional_count} -eq 0 ]]; then
                robot_name="$1"
            elif [[ ${positional_count} -eq 1 ]]; then
                policy_name="$1"
            else
                echo "多余位置参数: $1" >&2
                usage >&2
                exit 2
            fi
            positional_count=$((positional_count + 1))
            shift
            ;;
    esac
done

if [[ -z "${shm_name}" ]]; then
    shm_name="/quadruped_control_${robot_name}"
fi
robot_config="${project_dir}/configs/robots/${robot_name}.yaml"
controller_config="${project_dir}/configs/controllers/${robot_name}.yaml"
policy_switch_config="${project_dir}/configs/policies/${robot_name}/policy_switch.yaml"
retry_config="${project_dir}/configs/behaviors/${robot_name}/retry.yaml"
event_chain_config="${project_dir}/configs/behaviors/${robot_name}/event_chain.yaml"
fixed_drive_config_dir="${project_dir}/configs/behaviors/${robot_name}"
executable="${project_dir}/.build/rl/apps/runtime_daemons/quadruped_motiond"
required_paths=("${executable}" "${robot_config}" "${controller_config}" "${retry_config}")
if [[ "${load_policy}" == true ]]; then
    required_paths+=("${policy_switch_config}")
fi
if [[ "${robot_name}" == "blackW" ]]; then
    required_paths+=(
        "${event_chain_config}"
        "${fixed_drive_config_dir}/bridge_drive.yaml"
        "${fixed_drive_config_dir}/low_bar_drive.yaml"
        "${fixed_drive_config_dir}/car_drive.yaml"
    )
elif [[ "${robot_name}" == "wolf" ]]; then
    # Wolf 当前只提供 car_drive，不得强制要求 bridge/low-bar 配置同时存在。
    required_paths+=("${fixed_drive_config_dir}/car_drive.yaml")
fi
for required in "${required_paths[@]}"; do
    if [[ ! -e "${required}" ]]; then
        echo "缺少 motion 依赖或配置: ${required}" >&2
        echo "请先运行 ./scripts/build.sh --target motion" >&2
        exit 1
    fi
done

export LD_LIBRARY_PATH="${project_dir}/.deps/libtorch-2.0.1-cpu/lib:${LD_LIBRARY_PATH:-}"
args=(
    "--shm" "${shm_name}"
    "--robot-config" "${robot_config}"
    "--controller-config" "${controller_config}"
    "--retry-config" "${retry_config}"
)
if [[ "${robot_name}" == "blackW" ]]; then
    args+=("--event-chain-config" "${event_chain_config}")
    args+=("--fixed-drive-config-dir" "${fixed_drive_config_dir}")
elif [[ "${robot_name}" == "wolf" ]]; then
    args+=("--fixed-drive-config" "${fixed_drive_config_dir}/car_drive.yaml")
fi
if [[ "${load_policy}" == true ]]; then
    args+=("--policy-switch-config" "${policy_switch_config}")
else
    args+=("--no-policy")
fi
if [[ "${load_policy}" == true && -n "${policy_name}" ]]; then
    args+=("--initial-policy" "${policy_name}")
fi
exec "${executable}" "${args[@]}"
