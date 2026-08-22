#!/usr/bin/env bash
# 文件：motion.sh
# 作用：从任意工作目录启动统一的 MotionRuntime 和 Torch 策略进程。

set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
robot_name="black"
policy_name="flat"
shm_name=""
robot_config=""
controller_config=""
flat_policy_config=""
obstacle_policy_config=""
extra_args=()
positional_count=0

usage() {
    cat <<USAGE
用法: $0 [robot] [policy] [选项]

位置参数:
  robot                 机器人名称，默认 black
  policy                初始策略 flat|obstacle，默认 flat

选项:
  --robot NAME
  --policy NAME         覆盖策略名称
  --shm NAME            共享内存名称，默认 /quadruped_control_<robot>
  --robot-config PATH
  --controller-config PATH
  --flat-policy-config PATH
  --obstacle-policy-config PATH
  其他 --key value      直接传给 quadruped_motiond
USAGE
}

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
        --policy|--initial-policy)
            [[ $# -ge 2 ]] || { echo "$1 缺少参数" >&2; exit 2; }
            policy_name="$2"; shift 2
            ;;
        --shm)
            [[ $# -ge 2 ]] || { echo "--shm 缺少参数" >&2; exit 2; }
            shm_name="$2"; shift 2
            ;;
        --robot-config)
            [[ $# -ge 2 ]] || { echo "--robot-config 缺少参数" >&2; exit 2; }
            robot_config="$2"; shift 2
            ;;
        --controller-config)
            [[ $# -ge 2 ]] || { echo "--controller-config 缺少参数" >&2; exit 2; }
            controller_config="$2"; shift 2
            ;;
        --flat-policy-config)
            [[ $# -ge 2 ]] || { echo "--flat-policy-config 缺少参数" >&2; exit 2; }
            flat_policy_config="$2"; shift 2
            ;;
        --obstacle-policy-config)
            [[ $# -ge 2 ]] || { echo "--obstacle-policy-config 缺少参数" >&2; exit 2; }
            obstacle_policy_config="$2"; shift 2
            ;;
        --*)
            [[ $# -ge 2 ]] || { echo "$1 缺少参数" >&2; exit 2; }
            extra_args+=("$1" "$2"); shift 2
            ;;
        *)
            if [[ ${positional_count} -eq 0 ]]; then
                robot_name="$1"
            elif [[ ${positional_count} -eq 1 ]]; then
                policy_name="$1"
            else
                echo "多余位置参数: $1" >&2
                exit 2
            fi
            positional_count=$((positional_count + 1))
            shift
            ;;
    esac
done

if [[ "${policy_name}" != "flat" && "${policy_name}" != "obstacle" ]]; then
    echo "不支持的策略: ${policy_name}（可选 flat|obstacle）" >&2
    exit 2
fi
if [[ -z "${shm_name}" ]]; then
    shm_name="/quadruped_control_${robot_name}"
fi
robot_config="${robot_config:-${project_dir}/configs/robots/${robot_name}.yaml}"
controller_config="${controller_config:-${project_dir}/configs/controllers/${robot_name}.yaml}"
flat_policy_config="${flat_policy_config:-${project_dir}/configs/policies/${robot_name}/flat.yaml}"
obstacle_policy_config="${obstacle_policy_config:-${project_dir}/configs/policies/${robot_name}/obstacle.yaml}"
executable="${project_dir}/.build/rl/apps/runtime_daemons/quadruped_motiond"
for required in "${executable}" "${robot_config}" "${controller_config}" \
    "${flat_policy_config}" "${obstacle_policy_config}"; do
    if [[ ! -e "${required}" ]]; then
        echo "缺少 motion 依赖或配置: ${required}" >&2
        echo "请先运行 ./scripts/build.sh --target motion" >&2
        exit 1
    fi
done

export LD_LIBRARY_PATH="${project_dir}/.deps/libtorch-2.0.1-cpu/lib:${LD_LIBRARY_PATH:-}"
exec "${executable}" \
    --shm "${shm_name}" \
    --robot-config "${robot_config}" \
    --controller-config "${controller_config}" \
    --flat-policy-config "${flat_policy_config}" \
    --obstacle-policy-config "${obstacle_policy_config}" \
    --initial-policy "${policy_name}" \
    "${extra_args[@]}"
