#!/usr/bin/env bash
# 文件：run_mujoco_sim.sh
# 作用：从任意当前目录启动 RL 或基础 MuJoCo 交互仿真程序。

set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_profile="rl"
if [[ "${1:-}" == "--basic" ]]
then
    build_profile="mujoco"
    shift
fi
build_dir="${project_dir}/build/${build_profile}"
exe="${build_dir}/apps/mujoco_sim/quadruped_mujoco_sim"

if [[ ! -f "${project_dir}/.deps/mujoco-3.9.0/lib/libmujoco.so" ]]
then
    echo "未找到 MuJoCo 3.9.0，请先运行 ./scripts/setup_mujoco.sh。" >&2
    exit 1
fi

if [[ ! -x "${exe}" ]]
then
    if [[ "${build_profile}" == "rl" ]]
    then
        echo "未找到 RL 界面程序，请先运行 ./scripts/build.sh --rl。" >&2
    else
        echo "未找到基础界面程序，请先运行 ./scripts/build.sh --mujoco。" >&2
    fi
    exit 1
fi

exec "${exe}" "$@"
