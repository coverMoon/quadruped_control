#!/usr/bin/env bash
# 文件：build.sh
# 作用：执行本地 Debug 配置、并行编译和自动测试。

set -euo pipefail

# 无论从哪个工作目录调用脚本，都以脚本上一级目录作为工程根目录。
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${project_dir}/build"

# 默认执行测试；命令行参数只覆盖本次构建行为，不修改工程配置。
run_tests=true
clean_build=false

usage() {
    echo "Usage: ./scripts/build.sh [--clean] [--no-test]"
    echo
    echo "  --clean    Remove the local build directory before configuring"
    echo "  --no-test  Build without running CTest"
    echo "  -h, --help Show this help"
}

while (($# > 0)); do
    case "$1" in
        --clean)
            clean_build=true
            ;;
        --no-test)
            run_tests=false
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
    shift
done

if [[ "${clean_build}" == true ]]; then
    cmake -E remove_directory "${build_dir}"
fi

# 优先使用全部在线 CPU；无法读取处理器数量时退化为单线程构建。
jobs="$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)"

cmake \
    -S "${project_dir}" \
    -B "${build_dir}" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

cmake --build "${build_dir}" --parallel "${jobs}"

# 根目录符号链接方便编辑器读取编译数据库，实际文件仍由 CMake 在 build/ 中生成。
cmake -E create_symlink \
    "${build_dir}/compile_commands.json" \
    "${project_dir}/compile_commands.json"

if [[ "${run_tests}" == true ]]; then
    ctest --test-dir "${build_dir}" --output-on-failure
fi
