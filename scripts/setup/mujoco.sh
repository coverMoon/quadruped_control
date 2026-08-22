#!/usr/bin/env bash
# 文件：mujoco.sh
# 作用：下载、校验并解压工程固定使用的 MuJoCo 发行包。

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly REPOSITORY_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
readonly MUJOCO_VERSION="3.9.0"
readonly MUJOCO_ARCHIVE="mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz"
# 下载包来自外部网络，固定校验值用于拒绝损坏或被替换的依赖二进制。
readonly MUJOCO_SHA256="d11f281540d0d1844e2923bf43b6fff5ad186ec55927a8dae0eb26b9e579eed2"
readonly MUJOCO_URL="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/${MUJOCO_ARCHIVE}"
readonly DEPENDENCY_DIR="${REPOSITORY_ROOT}/.deps"
readonly ARCHIVE_PATH="${DEPENDENCY_DIR}/${MUJOCO_ARCHIVE}"
readonly INSTALL_DIR="${DEPENDENCY_DIR}/mujoco-${MUJOCO_VERSION}"

if [[ "$(uname -s)" != "Linux" || "$(uname -m)" != "x86_64" ]]
then
    echo "当前脚本只支持 Linux x86_64 的 MuJoCo 官方发行包。" >&2
    exit 1
fi

if [[ -f "${INSTALL_DIR}/include/mujoco/mujoco.h" &&
      -f "${INSTALL_DIR}/lib/libmujoco.so" &&
      "$(grep -c '^#define mjVERSION_HEADER 3009000$' \
          "${INSTALL_DIR}/include/mujoco/mujoco.h")" == "1" ]]
then
    echo "MuJoCo ${MUJOCO_VERSION} 已安装在 ${INSTALL_DIR}"
    exit 0
fi

if [[ -e "${INSTALL_DIR}" ]]
then
    echo "发现不完整的安装目录：${INSTALL_DIR}" >&2
    echo "请检查或移除该目录后重试。" >&2
    exit 1
fi

mkdir -p "${DEPENDENCY_DIR}"

if [[ ! -f "${ARCHIVE_PATH}" ]]
then
    readonly DOWNLOAD_PATH="${ARCHIVE_PATH}.part"
    echo "正在下载 MuJoCo ${MUJOCO_VERSION} ……"
    curl --location --fail --show-error --output "${DOWNLOAD_PATH}" "${MUJOCO_URL}"
    mv "${DOWNLOAD_PATH}" "${ARCHIVE_PATH}"
fi

readonly ACTUAL_SHA256="$(sha256sum "${ARCHIVE_PATH}" | awk '{print $1}')"
if [[ "${ACTUAL_SHA256}" != "${MUJOCO_SHA256}" ]]
then
    echo "MuJoCo 发行包 SHA-256 校验失败。" >&2
    echo "期望：${MUJOCO_SHA256}" >&2
    echo "实际：${ACTUAL_SHA256}" >&2
    exit 1
fi

readonly EXTRACT_DIR="$(mktemp -d "${DEPENDENCY_DIR}/.mujoco-extract.XXXXXX")"
trap 'rm -rf "${EXTRACT_DIR}"' EXIT

tar --extract --gzip --file "${ARCHIVE_PATH}" --directory "${EXTRACT_DIR}"
mv "${EXTRACT_DIR}/mujoco-${MUJOCO_VERSION}" "${INSTALL_DIR}"

echo "MuJoCo ${MUJOCO_VERSION} 已安装在 ${INSTALL_DIR}"
