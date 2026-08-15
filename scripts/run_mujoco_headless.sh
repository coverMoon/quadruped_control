#!/usr/bin/env bash
# 文件：run_mujoco_headless.sh
# 作用：从任意当前目录启动无界面 MuJoCo 仿真程序。

set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${project_dir}/build/mujoco"
exe="${build_dir}/apps/mujoco_headless/quadruped_mujoco_headless"

if [[ ! -f "${project_dir}/.deps/mujoco-3.9.0/lib/libmujoco.so" ]]
then
    echo "未找到 MuJoCo 3.9.0，请先运行 ./scripts/setup_mujoco.sh。" >&2
    exit 1
fi

if [[ ! -f "${exe}" ]]
then
    echo "未找到无界面程序，请先运行 ./scripts/build.sh --mujoco。" >&2
    exit 1
fi

exec "${exe}" "$@"
