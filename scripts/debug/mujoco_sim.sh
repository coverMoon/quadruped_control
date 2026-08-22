#!/usr/bin/env bash
# 文件：mujoco_sim.sh
# 作用：启动保留的单进程 MuJoCo 交互调试程序，不作为三进程正式入口。

set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
profile="rl"
if [[ "${1:-}" == "--basic" ]]; then profile="mujoco"; shift; fi
executable="${project_dir}/.build/${profile}/apps/mujoco_sim/quadruped_mujoco_sim"
[[ -x "${executable}" ]] || { echo "缺少调试程序，请先构建对应 profile。" >&2; exit 1; }
export LD_LIBRARY_PATH="${project_dir}/.deps/mujoco-3.9.0/lib:${project_dir}/.deps/libtorch-2.0.1-cpu/lib:${LD_LIBRARY_PATH:-}"
exec "${executable}" "$@"
