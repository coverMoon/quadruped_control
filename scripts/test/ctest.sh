#!/usr/bin/env bash
# 文件：ctest.sh
# 作用：对选定的 CMake profile 执行已有 CTest 测试集合。

set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
profile="${1:-rl}"
case "${profile}" in default|mujoco|rl) ;; *) echo "profile 必须是 default|mujoco|rl" >&2; exit 2 ;; esac
build_dir="${project_dir}/.build/${profile}"
if [[ ! -f "${build_dir}/CTestTestfile.cmake" ]]; then
    echo "缺少 CTest profile: ${build_dir}，请先运行对应 build.sh target。" >&2
    exit 1
fi
exec ctest --test-dir "${build_dir}" --output-on-failure
