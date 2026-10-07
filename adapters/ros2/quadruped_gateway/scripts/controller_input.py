#!/usr/bin/env python3
# 文件：controller_input.py
# 作用：按统一 YAML profile 热插拔读取物理手柄并发布固定长度规范化 Joy。

import argparse
import math
import os
from pathlib import Path

import pygame
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import Joy

AXIS_NAMES = ("lx", "ly", "lt", "rx", "ry", "rt")
BUTTON_NAMES = (
    "a", "b", "x", "y", "lb", "rb", "back", "start", "guide",
    "left_stick", "right_stick",
)


def load_config(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError("手柄配置根节点必须是映射")
    output = data.get("output", {})
    axis_count = int(output.get("axis_count", 0))
    button_count = int(output.get("button_count", 0))
    deadband = float(output.get("default_deadband", 0.08))
    if axis_count != 8 or button_count != 12:
        raise ValueError("规范化输出必须固定为 8 个轴和 12 个按钮")
    if not math.isfinite(deadband) or deadband < 0.0 or deadband >= 1.0:
        raise ValueError("default_deadband 必须位于 [0, 1)")
    profiles = data.get("profiles")
    if not isinstance(profiles, dict) or not profiles:
        raise ValueError("profiles 不能为空")
    for name, profile in profiles.items():
        validate_profile(name, profile, axis_count, button_count, deadband)
    return axis_count, button_count, deadband, profiles


def button_indices(mapping):
    values = mapping if isinstance(mapping, list) else [mapping]
    indices = [int(value) for value in values]
    if not indices or any(index < 0 for index in indices):
        raise ValueError("按钮物理索引必须为非空非负列表")
    return indices


def validate_profile(name, profile, axis_count, button_count, default_deadband):
    if not isinstance(profile, dict):
        raise ValueError(f"profile {name} 必须是映射")
    matches = profile.get("match", [])
    if not isinstance(matches, list) or not all(isinstance(value, str) for value in matches):
        raise ValueError(f"profile {name} 的 match 必须是字符串列表")
    axes = profile.get("axes")
    if not isinstance(axes, dict):
        raise ValueError(f"profile {name} 的 axes 必须是映射")
    used_axes = set()
    for axis_name in AXIS_NAMES:
        mapping = axes.get(axis_name)
        if not isinstance(mapping, dict):
            raise ValueError(f"profile {name} 缺少轴 {axis_name}")
        index = int(mapping.get("index", -1))
        scale = float(mapping.get("scale", 0.0))
        deadband = float(mapping.get("deadband", default_deadband))
        if index < 0 or index in used_axes:
            raise ValueError(f"profile {name} 的物理轴索引重复或非法")
        if not math.isfinite(scale) or scale == 0.0:
            raise ValueError(f"profile {name} 的轴 scale 非法")
        if not math.isfinite(deadband) or deadband < 0.0 or deadband >= 1.0:
            raise ValueError(f"profile {name} 的轴 deadband 非法")
        used_axes.add(index)
    buttons = profile.get("buttons")
    if not isinstance(buttons, dict):
        raise ValueError(f"profile {name} 的 buttons 必须是映射")
    used_buttons = set()
    for button_name in BUTTON_NAMES:
        if button_name not in buttons:
            raise ValueError(f"profile {name} 缺少按钮 {button_name}")
        indices = button_indices(buttons[button_name])
        if len(set(indices)) != len(indices) or any(index in used_buttons for index in indices):
            raise ValueError(f"profile {name} 的按钮索引重复或非法")
        used_buttons.update(indices)
    dpad = profile.get("dpad")
    if not isinstance(dpad, dict):
        raise ValueError(f"profile {name} 的 dpad 必须是映射")
    if dpad.get("source") != "hat":
        raise ValueError(f"profile {name} 当前只支持 hat 类型 D-pad")
    for key in ("x_axis", "y_axis"):
        target = int(dpad.get(key, -1))
        if target < 0 or target >= axis_count:
            raise ValueError(f"profile {name} 的 D-pad 目标轴越界")


class ControllerInput(Node):
    def __init__(self, args):
        super().__init__("controller_input")
        self.axis_count, self.button_count, self.default_deadband, self.profiles = (
            load_config(args.config)
        )
        if args.profile and args.profile not in self.profiles:
            raise ValueError(f"未知手柄 profile: {args.profile}")
        self.explicit_profile = args.profile
        self.publisher = self.create_publisher(Joy, args.joy_topic, 10)
        self.joystick = None
        self.profile_name = ""
        self.profile = None
        self.last_warning_ns = 0
        # 采集进程没有 SDL 前台窗口，仍需在 MuJoCo 窗口获得焦点时更新手柄状态。
        os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
        pygame.init()
        pygame.joystick.init()
        self.timer = self.create_timer(0.01, self.poll)

    def select_profile(self, device_name):
        if self.explicit_profile:
            return self.explicit_profile, self.profiles[self.explicit_profile]
        lowered = device_name.casefold()
        best_name = ""
        best_profile = None
        best_match_length = -1
        for name, profile in self.profiles.items():
            for fragment in profile.get("match", []):
                if fragment.casefold() in lowered and len(fragment) > best_match_length:
                    best_name = name
                    best_profile = profile
                    best_match_length = len(fragment)
        return best_name, best_profile

    def connect(self):
        if pygame.joystick.get_count() == 0:
            return
        candidate = pygame.joystick.Joystick(0)
        candidate.init()
        profile_name, profile = self.select_profile(candidate.get_name())
        if profile is None:
            now = self.get_clock().now().nanoseconds
            if now - self.last_warning_ns > 5_000_000_000:
                self.get_logger().warning(
                    f"未知手柄 {candidate.get_name()!r}，未进行危险的默认映射"
                )
                self.last_warning_ns = now
            candidate.quit()
            return
        self.joystick = candidate
        self.profile_name = profile_name
        self.profile = profile
        self.get_logger().info(
            f"手柄已连接: {candidate.get_name()}，profile={profile_name}"
        )

    def disconnect(self):
        if self.joystick is not None:
            try:
                self.joystick.quit()
            except pygame.error:
                pass
        self.joystick = None
        self.profile = None
        self.profile_name = ""
        self.publish_zero("joy_disconnected")

    def publish_zero(self, frame_id):
        message = Joy()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        message.axes = [0.0] * self.axis_count
        message.buttons = [0] * self.button_count
        self.publisher.publish(message)

    def normalized_axis(self, mapping):
        index = int(mapping["index"])
        if self.joystick is None or index >= self.joystick.get_numaxes():
            return 0.0
        value = float(self.joystick.get_axis(index)) * float(mapping["scale"])
        deadband = float(mapping.get("deadband", self.default_deadband))
        if abs(value) < deadband:
            return 0.0
        return max(-1.0, min(1.0, value))

    def poll(self):
        try:
            pygame.event.get()
            if self.joystick is None:
                self.connect()
            if self.joystick is None:
                self.publish_zero("joy_disconnected")
                return
            attached = getattr(self.joystick, "get_attached", lambda: True)()
            if not attached:
                self.disconnect()
                return
            message = Joy()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = f"joy_connected:{self.profile_name}"
            message.axes = [0.0] * self.axis_count
            output_slots = {name: index for index, name in enumerate(AXIS_NAMES)}
            for name, output_index in output_slots.items():
                message.axes[output_index] = self.normalized_axis(
                    self.profile["axes"][name]
                )
            dpad = self.profile["dpad"]
            hat_index = int(dpad.get("index", 0))
            if hat_index < self.joystick.get_numhats():
                hat_x, hat_y = self.joystick.get_hat(hat_index)
                message.axes[int(dpad["x_axis"])] = float(hat_x)
                message.axes[int(dpad["y_axis"])] = float(hat_y)
            message.buttons = [0] * self.button_count
            for output_index, name in enumerate(BUTTON_NAMES):
                physical_indices = button_indices(self.profile["buttons"][name])
                message.buttons[output_index] = int(any(
                    index < self.joystick.get_numbuttons() and
                    self.joystick.get_button(index)
                    for index in physical_indices
                ))
            self.publisher.publish(message)
        except pygame.error as error:
            self.get_logger().warning(f"手柄读取失败: {error}")
            self.disconnect()

    def destroy_node(self):
        self.publish_zero("joy_disconnected")
        self.disconnect()
        pygame.joystick.quit()
        pygame.quit()
        super().destroy_node()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--profile", default="")
    parser.add_argument("--joy-topic", default="/joy")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = None
    try:
        node = ControllerInput(args)
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
