#!/usr/bin/env bash
# 文件：setup_libtorch.sh
# 作用：下载并安装固定版本的 LibTorch CPU C++11 ABI 依赖。

set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
deps_dir="${project_dir}/.deps"
install_dir="${deps_dir}/libtorch-2.0.1-cpu"
archive_name="libtorch-cxx11-abi-shared-with-deps-2.0.1+cpu.zip"
archive_path="${deps_dir}/${archive_name}"
url="https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip"
expected_sha256="137a842d1cf1e9196b419390133a1623ef92f8f84dc7a072f95ada684f394afd"

echo "检查 LibTorch 2.0.1 CPU 归档。"
mkdir -p "${deps_dir}"

if [[ ! -f "${archive_path}" ]]; then
    curl --fail --location --retry 2 --output "${archive_path}" "${url}"
fi

actual_sha256="$(sha256sum "${archive_path}" | awk '{print $1}')"
if [[ "${actual_sha256}" != "${expected_sha256}" ]]; then
    echo "LibTorch 归档 SHA-256 校验失败。" >&2
    echo "期望：${expected_sha256}" >&2
    echo "实际：${actual_sha256}" >&2
    rm -f "${archive_path}"
    exit 1
fi

tmp_dir="$(mktemp -d "${deps_dir}/libtorch-install.XXXXXX")"
trap 'rm -rf "${tmp_dir}"' EXIT
unzip -q "${archive_path}" -d "${tmp_dir}"
rm -rf "${install_dir}"
mv "${tmp_dir}/libtorch" "${install_dir}"

echo "LibTorch 已安装到：${install_dir}"
