#!/usr/bin/env python3
# 文件：generate_id.py
# 作用：根据稳定名称和版本生成模型或实机标定的 64 位 ID。

"""生成稳定的 64 位模型和标定标识。"""

from __future__ import annotations

import argparse
import hashlib
import re
from dataclasses import dataclass


# 名称规则保证同一机器人不会因大小写或空格差异产生多个 ID。
NAME_PATTERN = re.compile(r"^[a-z][a-z0-9_-]*$")
SERIAL_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


@dataclass(frozen=True)
class GeneratedId:
    """保存生成结果及其可直接写入配置的字段名称。"""

    field_name: str
    canonical_name: str
    value: int


def positive_integer(text: str) -> int:
    """把版本参数转换为正整数，并向 argparse 返回清晰错误。"""

    value = int(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("version/revision must be greater than zero")
    return value


def robot_name(text: str) -> str:
    """校验跨配置长期使用的稳定机器人名称。"""

    if not NAME_PATTERN.fullmatch(text):
        raise argparse.ArgumentTypeError(
            "robot name must start with a lowercase letter and contain only "
            "lowercase letters, digits, '_' or '-'"
        )
    return text


def robot_serial(text: str) -> str:
    """校验制造或装配阶段分配给单台实机的序列号。"""

    if not SERIAL_PATTERN.fullmatch(text):
        raise argparse.ArgumentTypeError(
            "robot serial must contain only letters, digits, '_', '-' or '.'"
        )
    return text


def hash_to_u64(canonical_name: str) -> int:
    """把规范名称映射为协议中使用的无符号 64 位整数。"""

    digest = hashlib.sha256(canonical_name.encode("utf-8")).digest()

    # 固定取摘要前 8 字节并使用大端序，确保不同语言实现得到同一个整数。
    value = int.from_bytes(digest[:8], byteorder="big", signed=False)
    if value == 0:
        raise RuntimeError("generated ID is zero, which is reserved")
    return value


def generate(args: argparse.Namespace) -> GeneratedId:
    """根据子命令建立不会受输出格式影响的规范名称。"""

    if args.kind == "model":
        canonical_name = f"model:{args.robot_name}:{args.version}"
        field_name = "model_id"
    else:
        canonical_name = (
            f"calibration:{args.robot_name}:{args.robot_serial}:{args.revision}"
        )
        field_name = "calibration_id"

    return GeneratedId(
        field_name=field_name,
        canonical_name=canonical_name,
        value=hash_to_u64(canonical_name),
    )


def build_parser() -> argparse.ArgumentParser:
    """定义模型 ID 和标定 ID 两种互斥命令格式。"""

    parser = argparse.ArgumentParser(
        description=(
            "Generate a stable uint64 ID from a canonical model or calibration name."
        )
    )
    subparsers = parser.add_subparsers(dest="kind", required=True)

    model_parser = subparsers.add_parser("model", help="generate a robot model ID")
    model_parser.add_argument("robot_name", type=robot_name, help="for example: black")
    model_parser.add_argument("version", type=positive_integer, help="model version")

    calibration_parser = subparsers.add_parser(
        "calibration", help="generate a physical robot calibration ID"
    )
    calibration_parser.add_argument(
        "robot_name", type=robot_name, help="for example: black"
    )
    calibration_parser.add_argument(
        "robot_serial", type=robot_serial, help="for example: BLACK-001"
    )
    calibration_parser.add_argument(
        "revision", type=positive_integer, help="calibration revision"
    )

    return parser


def main() -> int:
    """解析参数并同时输出规范名称、十六进制 ID 和十进制 ID。"""

    args = build_parser().parse_args()
    generated = generate(args)

    print(f"canonical_name: {generated.canonical_name}")
    print(f"{generated.field_name}: 0x{generated.value:016X}")
    print(f"decimal: {generated.value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
