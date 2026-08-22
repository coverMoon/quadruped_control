#!/usr/bin/env bash
# 文件：backend.sh
# 作用：从任意工作目录启动统一的 MuJoCo 物理后端进程。

set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
robot_name="black"
scene_name="plain"
backend_name="mujoco"
mode="gui"
shm_name=""
robot_config=""
scene_path=""
real_time_factor="1.0"
visual_sync_hz="60.0"
vsync="true"
positional_count=0
scene_from_position=false

usage() {
    cat <<USAGE
用法: $0 [robot] [scene] [选项]

位置参数:
  robot                 机器人名称，默认 black
  scene                 场景变体 plain|terrain，默认 plain

选项:
  --robot NAME          覆盖机器人名称
  --backend NAME        后端名称，目前仅支持 mujoco
  --scene NAME          场景变体，或传入显式 MJCF 路径
  --mode gui|headless   运行模式，默认 gui
  --shm NAME            共享内存名称，默认 /quadruped_control_<robot>
  --robot-config PATH   机器人配置路径
  --real-time-factor X  仿真实时倍率
  --visual-sync-hz HZ  GUI 状态同步频率
  --vsync true|false    是否启用 GUI 垂直同步
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
        --backend)
            [[ $# -ge 2 ]] || { echo "--backend 缺少参数" >&2; exit 2; }
            backend_name="$2"; shift 2
            ;;
        --scene)
            [[ $# -ge 2 ]] || { echo "--scene 缺少参数" >&2; exit 2; }
            scene_name="$2"; shift 2
            ;;
        --mode)
            [[ $# -ge 2 ]] || { echo "--mode 缺少参数" >&2; exit 2; }
            mode="$2"; shift 2
            ;;
        --shm)
            [[ $# -ge 2 ]] || { echo "--shm 缺少参数" >&2; exit 2; }
            shm_name="$2"; shift 2
            ;;
        --robot-config)
            [[ $# -ge 2 ]] || { echo "--robot-config 缺少参数" >&2; exit 2; }
            robot_config="$2"; shift 2
            ;;
        --real-time-factor)
            [[ $# -ge 2 ]] || { echo "--real-time-factor 缺少参数" >&2; exit 2; }
            real_time_factor="$2"; shift 2
            ;;
        --visual-sync-hz)
            [[ $# -ge 2 ]] || { echo "--visual-sync-hz 缺少参数" >&2; exit 2; }
            visual_sync_hz="$2"; shift 2
            ;;
        --vsync)
            [[ $# -ge 2 ]] || { echo "--vsync 缺少参数" >&2; exit 2; }
            vsync="$2"; shift 2
            ;;
        --*)
            echo "未知选项: $1" >&2
            usage >&2
            exit 2
            ;;
        *)
            if [[ ${positional_count} -eq 0 ]]; then
                robot_name="$1"
            elif [[ ${positional_count} -eq 1 ]]; then
                scene_name="$1"
                scene_from_position=true
            else
                echo "多余位置参数: $1" >&2
                exit 2
            fi
            positional_count=$((positional_count + 1))
            shift
            ;;
    esac
done

if [[ "${scene_from_position}" == true && "${scene_name}" != "plain" &&
    "${scene_name}" != "terrain" ]]; then
    echo "不支持的场景: ${scene_name}（可选 plain|terrain）" >&2
    exit 2
fi
if [[ "${backend_name}" != "mujoco" ]]; then
    echo "不支持的 backend: ${backend_name}（当前仅支持 mujoco）" >&2
    exit 2
fi
if [[ "${mode}" != "gui" && "${mode}" != "headless" ]]; then
    echo "不支持的运行模式: ${mode}（可选 gui|headless）" >&2
    exit 2
fi
if [[ "${scene_name}" == "plain" || "${scene_name}" == "terrain" ]]; then
    if [[ -z "${scene_path}" ]]; then
        scene_path="${project_dir}/assets/robots/${robot_name}/mujoco/scene.xml"
        if [[ "${scene_name}" == "terrain" ]]; then
            scene_path="${project_dir}/assets/robots/${robot_name}/mujoco/scene_terrain.xml"
        fi
    fi
else
    scene_path="${scene_name}"
    scene_name="explicit"
fi
if [[ -z "${robot_config}" ]]; then
    robot_config="${project_dir}/configs/robots/${robot_name}.yaml"
fi
if [[ -z "${shm_name}" ]]; then
    shm_name="/quadruped_control_${robot_name}"
fi

executable="${project_dir}/.build/rl/apps/runtime_daemons/quadruped_mujoco_backendd"
if [[ ! -x "${executable}" ]]; then
    executable="${project_dir}/.build/mujoco/apps/runtime_daemons/quadruped_mujoco_backendd"
fi
mujoco_library="${project_dir}/.deps/mujoco-3.9.0/lib/libmujoco.so"
for required in "${mujoco_library}" "${executable}" "${robot_config}" "${scene_path}"; do
    if [[ ! -e "${required}" ]]; then
        echo "缺少 backend 依赖或配置: ${required}" >&2
        echo "请先运行 ./scripts/build.sh --target backend --backend mujoco" >&2
        exit 1
    fi
done

export LD_LIBRARY_PATH="${project_dir}/.deps/mujoco-3.9.0/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
exec "${executable}" \
    --shm "${shm_name}" \
    --scene "${scene_path}" \
    --robot-config "${robot_config}" \
    --mode "${mode}" \
    --real-time-factor "${real_time_factor}" \
    --visual-sync-hz "${visual_sync_hz}" \
    --vsync "${vsync}"
