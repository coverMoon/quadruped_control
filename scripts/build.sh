#!/usr/bin/env bash
# 文件：build.sh
# 作用：按默认或 MuJoCo 配置执行本地 Debug 编译和自动测试。

set -euo pipefail

# 无论从哪个工作目录调用脚本，都以脚本上一级目录作为工程根目录。
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

# 所有构建配置放在 build/ 的子目录中，避免在仓库根目录生成缓存。
build_root="${project_dir}/build"
build_profile="default"
mujoco_cmake_option="OFF"

# 默认执行测试；命令行参数只覆盖本次构建行为，不修改源文件。
run_tests=true
clean_build=false
enable_mujoco=false

usage() {
    echo "Usage: ./scripts/build.sh [--mujoco] [--clean] [--no-test]"
    echo
    echo "  --mujoco   Build the MuJoCo-enabled profile under build/mujoco"
    echo "  --clean    Remove all profiles under build before configuring"
    echo "  --no-test  Build without running CTest"
    echo "  -h, --help Show this help"
}

while (($# > 0)); do
    case "$1" in
        --clean)
            clean_build=true
            ;;
        --mujoco)
            enable_mujoco=true
            build_profile="mujoco"
            mujoco_cmake_option="ON"
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

# 参数解析完成后再确定构建目录，确保每种配置使用独立的 CMake 缓存。
build_dir="${build_root}/${build_profile}"

if [[ "${clean_build}" == true ]]; then
    cmake -E remove_directory "${build_root}"
fi

if [[ "${enable_mujoco}" == true &&
      ! -f "${project_dir}/.deps/mujoco-3.9.0/lib/libmujoco.so" ]]
then
    echo "未找到 MuJoCo 3.9.0，请先运行 ./scripts/setup_mujoco.sh。" >&2
    exit 1
fi

# 优先使用全部在线 CPU；无法读取处理器数量时退化为单线程构建。
jobs="$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)"

cmake \
    -S "${project_dir}" \
    -B "${build_dir}" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DQUADRUPED_ENABLE_MUJOCO="${mujoco_cmake_option}"

cmake --build "${build_dir}" --parallel "${jobs}"

# 根目录符号链接方便编辑器读取编译数据库，实际文件仍由 CMake 在 build/ 中生成。
cmake -E create_symlink \
    "${build_dir}/compile_commands.json" \
    "${project_dir}/compile_commands.json"

if [[ "${run_tests}" == true ]]; then
    ctest --test-dir "${build_dir}" --output-on-failure
fi
