#!/usr/bin/env bash
# 文件：build.sh
# 作用：提供 CMake、ROS 2 和默认全量构建的唯一公共入口。

set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_root="${project_dir}/.build"
target="all"
backend="mujoco"
run_tests=true
clean_build=false
legacy_profile=""

usage() {
    cat <<USAGE
用法: $0 [--target all|core|backend|motion|command] [选项]

选项:
  --target NAME       构建目标，默认 all
  --backend NAME      backend 类型，目前仅支持 mujoco
  --mujoco            兼容入口，等价 --target backend --backend mujoco
  --rl                兼容入口，构建 backend 和 motion 的 RL profile
  --clean             清理所选 CMake profile；command 清理固定 ROS 2 工作区
  --no-test           构建后不运行 CTest
USAGE
}

while (($# > 0)); do
    case "$1" in
        --target)
            [[ $# -ge 2 ]] || { echo "--target 缺少参数" >&2; exit 2; }
            target="$2"; shift 2
            ;;
        --backend)
            [[ $# -ge 2 ]] || { echo "--backend 缺少参数" >&2; exit 2; }
            backend="$2"; shift 2
            ;;
        --mujoco)
            target="backend"; backend="mujoco"; legacy_profile="mujoco"; shift
            ;;
        --rl)
            target="motion"; legacy_profile="rl"; shift
            ;;
        --clean)
            clean_build=true; shift
            ;;
        --no-test)
            run_tests=false; shift
            ;;
        -h|--help)
            usage; exit 0
            ;;
        *)
            echo "未知选项: $1" >&2; usage >&2; exit 2
            ;;
    esac
done

case "${target}" in all|core|backend|motion|command) ;; *) echo "未知 target: ${target}" >&2; exit 2 ;; esac
if [[ "${backend}" != "mujoco" ]]; then
    echo "未知 backend: ${backend}（当前仅支持 mujoco）" >&2
    exit 2
fi
jobs="$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)"

build_cmake_profile() {
    local profile="$1"
    local mujoco="$2"
    local torch="$3"
    local build_dir="${build_root}/${profile}"
    if [[ "${clean_build}" == true ]]; then cmake -E remove_directory "${build_dir}"; fi
    if [[ "${mujoco}" == ON && ! -f "${project_dir}/.deps/mujoco-3.9.0/lib/libmujoco.so" ]]; then
        echo "未找到 MuJoCo 3.9.0，请先运行 ./scripts/setup/mujoco.sh。" >&2; exit 1
    fi
    if [[ "${torch}" == ON && ! -f "${project_dir}/.deps/libtorch-2.0.1-cpu/share/cmake/Torch/TorchConfig.cmake" ]]; then
        echo "未找到 LibTorch 2.0.1 CPU，请先运行 ./scripts/setup/libtorch.sh。" >&2; exit 1
    fi
    cmake -S "${project_dir}" -B "${build_dir}" \
        -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        -DQUADRUPED_ENABLE_MUJOCO="${mujoco}" -DQUADRUPED_ENABLE_TORCH="${torch}"
    cmake --build "${build_dir}" --parallel "${jobs}"
    if [[ "${run_tests}" == true ]]; then
        ctest --test-dir "${build_dir}" --output-on-failure
    fi
}

build_command() {
    local ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
    local workspace_root="${QUADRUPED_ROS2_WORKSPACE_ROOT:-${build_root}/ros2}"
    local clean_option=()
    if [[ "${clean_build}" == true ]]; then
        clean_option=(--clean)
    fi
    ROS_SETUP="${ros_setup}" \
        QUADRUPED_ROS2_WORKSPACE_ROOT="${workspace_root}" \
        "${project_dir}/scripts/setup/ros2.sh" "${clean_option[@]}"
}

case "${target}" in
    core)
        build_cmake_profile default OFF OFF
        ;;
    backend)
        build_cmake_profile mujoco ON OFF
        ;;
    motion)
        build_cmake_profile rl ON ON
        ;;
    command)
        build_command
        ;;
    all)
        build_cmake_profile rl ON ON
        build_command
        ;;
esac

if [[ "${legacy_profile}" == "rl" ]]; then
    echo "提示: --rl 兼容入口仅构建 CMake RL profile；ROS 2 command 请使用 --target command。"
fi
