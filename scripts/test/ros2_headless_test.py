#!/usr/bin/env python3
# 文件：ros2_headless_test.py
# 作用：启动并验证指定机器人三进程 ROS 2 无界面仿真链路及进程故障退路。

import argparse
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy

from quadruped_interfaces.action import GetDown, GetUp, StartBehavior, SwitchPolicy
from quadruped_interfaces.msg import MotionStatus, StateDiagnostic
from quadruped_interfaces.srv import EnterPassive


COMPLETED = 3
REJECTED = 1
FAILED = 4
MODE_PASSIVE = 0
MODE_STAND = 2
MODE_RUNNING = 3
SOURCE_NONE = 0
SOURCE_GAMEPAD = 1
SOURCE_NAVIGATION = 2


class RuntimeGroup:
    def __init__(self, args, name):
        self.args = args
        self.shared_memory_name = f"/{name}_{os.getpid()}"
        self.temp_dir = tempfile.TemporaryDirectory(prefix=f"{name}_")
        self.processes = {}
        self.logs = {}

    def _start(self, name, command):
        log_path = Path(self.temp_dir.name) / f"{name}.log"
        log_file = log_path.open("w", encoding="utf-8")
        process = subprocess.Popen(
            command,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.processes[name] = process
        self.logs[name] = (log_file, log_path)
        return process

    def start_backend(self):
        return self._start(
            "backend",
            [
                self.args.backend_script,
                self.args.robot,
                "plain",
                "--mode",
                "headless",
                "--shm",
                self.shared_memory_name,
                "--real-time-factor",
                str(self.args.real_time_factor),
            ],
        )

    def start_motion(self):
        return self._start(
            "motion",
            [
                self.args.motion_script,
                self.args.robot,
                "flat",
                "--shm",
                self.shared_memory_name,
            ],
        )

    def start_gateway(self):
        return self._start(
            "gateway",
            [
                self.args.command_script,
                self.args.robot,
                "keyboard",
                "--shm",
                self.shared_memory_name,
            ],
        )

    def reset(self):
        completed = subprocess.run(
            [self.args.ipc_control, self.shared_memory_name, "reset"],
            check=False,
            capture_output=True,
            text=True,
            timeout=8.0,
        )
        if completed.returncode != 0:
            raise AssertionError(
                "后端 reset 失败: " + completed.stdout + completed.stderr
            )

    def stop(self, name):
        process = self.processes.get(name)
        if process is None or process.poll() is not None:
            return
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3.0)

    def assert_running(self, excluded=()):
        for name, process in self.processes.items():
            if name in excluded:
                continue
            if process.poll() is not None:
                raise AssertionError(
                    f"{name} 意外退出，返回码 {process.returncode}:\n{self.read_log(name)}"
                )

    def read_log(self, name):
        log = self.logs.get(name)
        if log is None:
            return ""
        log[0].flush()
        return log[1].read_text(encoding="utf-8", errors="replace")

    def close(self):
        for name in list(self.processes):
            self.stop(name)
        for log_file, _ in self.logs.values():
            log_file.close()
        self.temp_dir.cleanup()


class TestNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.motion_status = None
        self.diagnostic = None
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        diagnostic_qos = QoSProfile(depth=1)
        diagnostic_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            MotionStatus, "/motion/status", self._on_motion_status, status_qos
        )
        self.create_subscription(
            StateDiagnostic,
            "/state/diagnostic",
            self._on_diagnostic,
            diagnostic_qos,
        )
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 1)
        joy_qos = QoSProfile(depth=1)
        joy_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.joy = self.create_publisher(Joy, "/joy", joy_qos)
        self.get_up = ActionClient(self, GetUp, "/motion/get_up")
        self.get_down = ActionClient(self, GetDown, "/motion/get_down")
        self.start_behavior = ActionClient(
            self, StartBehavior, "/motion/start_behavior"
        )
        self.switch_policy = ActionClient(
            self, SwitchPolicy, "/motion/switch_policy"
        )
        self.enter_passive = self.create_client(
            EnterPassive, "/motion/enter_passive"
        )

    def _on_motion_status(self, message):
        self.motion_status = message

    def _on_diagnostic(self, message):
        self.diagnostic = message

    def spin_until(self, predicate, timeout, message, on_spin=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if on_spin is not None:
                on_spin()
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return
        status = self.motion_status
        diagnostic = self.diagnostic
        details = ""
        if status is not None:
            details += (
                f"; motion_mode={status.mode}, active_source={status.active_source}, "
                f"behavior={status.behavior_name}, phase={status.behavior_phase}, "
                f"error={status.error_message}"
            )
        if diagnostic is not None:
            details += (
                f"; session={diagnostic.session_id}, state_sequence={diagnostic.sequence}, "
                f"effective_command={diagnostic.effective_command_sequence}"
            )
        raise AssertionError(message + details)

    def wait_future(self, future, timeout, message, on_spin=None):
        self.spin_until(future.done, timeout, message, on_spin)
        exception = future.exception()
        if exception is not None:
            raise AssertionError(f"{message}: {exception}")
        return future.result()

    def wait_ready(self, group):
        self.spin_until(
            lambda: self.diagnostic is not None and self.motion_status is not None,
            15.0,
            "未收到三进程状态",
            group.assert_running,
        )
        self.spin_until(
            lambda: self.get_up.wait_for_server(timeout_sec=0.0),
            10.0,
            "GetUp action server 未就绪",
            group.assert_running,
        )

    def publish_velocity(self, vx=0.0, vy=0.0, wz=0.0):
        message = Twist()
        message.linear.x = vx
        message.linear.y = vy
        message.angular.z = wz
        self.cmd_vel.publish(message)

    def publish_joy(self, axes=None, buttons=None, frame_id="joy_connected:test"):
        message = Joy()
        message.header.frame_id = frame_id
        message.axes = list(axes) if axes is not None else [0.0] * 8
        message.buttons = list(buttons) if buttons is not None else [0] * 6
        self.joy.publish(message)

    def assert_rl_stable(self, group, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            group.assert_running()
            self.publish_velocity(0.2, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.motion_status is None:
                continue
            if self.motion_status.mode != MODE_RUNNING:
                raise AssertionError(
                    "RL 持续运行期间意外退出: " + self.motion_status.error_message
                )
            if "frame timestamp is invalid or in the future" in self.motion_status.error_message:
                raise AssertionError("RL 持续运行期间出现跨帧时间戳竞态")

    def begin_action(self, client, goal, group, timeout=10.0):
        self.spin_until(
            lambda: client.wait_for_server(timeout_sec=0.0),
            timeout,
            "action server 未就绪",
            group.assert_running,
        )
        goal_future = client.send_goal_async(goal)
        goal_handle = self.wait_future(
            goal_future, timeout, "action goal 提交超时", group.assert_running
        )
        if not goal_handle.accepted:
            raise AssertionError("action goal 被 gateway 拒绝")
        return goal_handle, goal_handle.get_result_async()

    def finish_action(
        self, client, goal, group, expected_state, timeout=10.0, on_spin=None
    ):
        _, result_future = self.begin_action(client, goal, group, timeout)

        def progress():
            group.assert_running()
            if on_spin is not None:
                on_spin()

        wrapped = self.wait_future(
            result_future, timeout, "action 结果超时", progress
        )
        result = wrapped.result.result
        if result.state != expected_state:
            raise AssertionError(
                f"action 结果错误: state={result.state}, message={result.message}"
            )
        return result

    def call_enter_passive(self, group):
        self.spin_until(
            lambda: self.enter_passive.wait_for_service(timeout_sec=0.0),
            5.0,
            "EnterPassive service 未就绪",
            group.assert_running,
        )
        request = EnterPassive.Request()
        future = self.enter_passive.call_async(request)
        response = self.wait_future(
            future, 10.0, "EnterPassive 结果超时", group.assert_running
        )
        if response.result.state != COMPLETED:
            raise AssertionError(
                "EnterPassive 失败: " + response.result.message
            )
        return response


def get_up_goal():
    return GetUp.Goal()


def get_down_goal():
    return GetDown.Goal()


def start_behavior_goal():
    goal = StartBehavior.Goal()
    goal.behavior_name = "rl_locomotion"
    return goal


def switch_policy_goal(policy_name):
    goal = SwitchPolicy.Goal()
    goal.policy_name = policy_name
    return goal


def start_group(group):
    group.start_backend()
    time.sleep(0.2)
    group.start_motion()
    time.sleep(0.2)
    group.start_gateway()


def functional_and_gateway_timeout(args):
    group = RuntimeGroup(args, "quadruped_stage7_functional")
    node = TestNode("quadruped_stage7_functional_test")
    try:
        start_group(group)
        node.wait_ready(group)
        initial_session = node.diagnostic.session_id

        node.finish_action(node.get_up, get_up_goal(), group, COMPLETED)
        node.spin_until(
            lambda: node.motion_status.mode == MODE_STAND,
            3.0,
            "GetUp 后未进入 Stand",
            group.assert_running,
        )

        retry_buttons = [0] * 6
        retry_buttons[1] = 1
        retry_buttons[4] = 1
        node.publish_joy(buttons=retry_buttons)
        time.sleep(0.05)
        node.publish_joy()
        node.spin_until(
            lambda: node.motion_status.mode == MODE_RUNNING
            and node.motion_status.behavior_name == "retry"
            and node.motion_status.behavior_phase == "locked",
            3.0,
            "LB+B 未启动 Retry 或误触发 GetDown",
            group.assert_running,
        )
        node.finish_action(node.get_up, get_up_goal(), group, COMPLETED)
        node.spin_until(
            lambda: node.motion_status.mode == MODE_STAND,
            3.0,
            "Retry 后 GetUp 未恢复 Stand",
            group.assert_running,
        )

        # 与旧 rl_sar 一致，启动时为 manual，手柄 X 显式进入 navigation。
        node.publish_joy()
        time.sleep(0.05)
        navigation_buttons = [0] * 6
        navigation_buttons[2] = 1
        node.publish_joy(buttons=navigation_buttons)
        time.sleep(0.05)
        node.publish_joy()
        node.publish_velocity(0.2, 0.0, 0.0)
        node.finish_action(
            node.start_behavior,
            start_behavior_goal(),
            group,
            COMPLETED,
            on_spin=lambda: node.publish_velocity(0.2, 0.0, 0.0),
        )
        node.spin_until(
            lambda: node.motion_status.mode == MODE_RUNNING
            and node.motion_status.active_source == SOURCE_NAVIGATION,
            3.0,
            "RL 行为未使用 Navigation 速度源",
            lambda: node.publish_velocity(0.2, 0.0, 0.0),
        )
        node.finish_action(
            node.switch_policy,
            switch_policy_goal("obstacle"),
            group,
            COMPLETED,
            on_spin=lambda: node.publish_velocity(0.2, 0.0, 0.0),
        )
        node.finish_action(
            node.switch_policy,
            switch_policy_goal("flat"),
            group,
            COMPLETED,
            on_spin=lambda: node.publish_velocity(0.2, 0.0, 0.0),
        )
        node.assert_rl_stable(group, 2.0)

        # 导航模式只由 X 显式切换；手柄离散按键仍可操作，重连不能抢占导航。
        node.publish_joy()
        y_buttons = [0] * 6
        y_buttons[3] = 1
        node.publish_joy(buttons=y_buttons)
        time.sleep(0.05)
        node.publish_joy()
        node.spin_until(
            lambda: node.motion_status.policy_name == "obstacle"
            and node.motion_status.active_source == SOURCE_NAVIGATION,
            5.0,
            "导航模式下手柄策略键未生效或速度源被抢占",
            lambda: node.publish_velocity(0.2, 0.0, 0.0),
        )

        x_buttons = [0] * 6
        x_buttons[2] = 1
        node.publish_joy(buttons=x_buttons)
        node.spin_until(
            lambda: node.motion_status.active_source == SOURCE_GAMEPAD,
            3.0,
            "手柄 X 未切换到 manual",
            group.assert_running,
        )
        node.publish_joy()
        time.sleep(0.05)
        node.publish_joy(buttons=x_buttons)
        node.spin_until(
            lambda: node.motion_status.active_source == SOURCE_NAVIGATION,
            3.0,
            "手柄 X 未切回 navigation",
            lambda: node.publish_velocity(0.2, 0.0, 0.0),
        )
        node.publish_joy(frame_id="joy_disconnected")
        node.publish_joy()
        node.spin_until(
            lambda: node.motion_status.active_source == SOURCE_NAVIGATION,
            3.0,
            "手柄重连后退出了 navigation",
            lambda: node.publish_velocity(0.2, 0.0, 0.0),
        )

        node.call_enter_passive(group)
        node.spin_until(
            lambda: node.motion_status.mode == MODE_PASSIVE,
            3.0,
            "EnterPassive 后未进入 Passive",
            group.assert_running,
        )
        node.finish_action(node.get_down, get_down_goal(), group, COMPLETED)
        node.finish_action(node.get_up, get_up_goal(), group, COMPLETED)
        node.finish_action(node.get_down, get_down_goal(), group, COMPLETED)
        node.spin_until(
            lambda: node.motion_status.mode == MODE_PASSIVE,
            3.0,
            "GetDown 后未进入 Passive",
            group.assert_running,
        )

        group.reset()
        node.spin_until(
            lambda: node.diagnostic.session_id > initial_session,
            5.0,
            "reset 后 session_id 未增加",
            group.assert_running,
        )

        node.finish_action(node.get_up, get_up_goal(), group, COMPLETED)
        node.publish_velocity(0.25, 0.0, 0.0)
        node.finish_action(
            node.start_behavior,
            start_behavior_goal(),
            group,
            COMPLETED,
            on_spin=lambda: node.publish_velocity(0.25, 0.0, 0.0),
        )
        node.spin_until(
            lambda: node.motion_status.active_source == SOURCE_NAVIGATION,
            3.0,
            "gateway 故障测试前速度源未激活",
            lambda: node.publish_velocity(0.25, 0.0, 0.0),
        )
        group.stop("gateway")
        time.sleep(0.4)
        group.start_gateway()
        node.spin_until(
            lambda: node.motion_status.mode == MODE_RUNNING
            and node.motion_status.active_source == SOURCE_NONE,
            5.0,
            "gateway 停止后 BaseCommand 未超时归零",
            group.assert_running,
        )
        print("[通过] 功能链路、reset 和 gateway 超时")
    finally:
        node.destroy_node()
        group.close()


def motion_failure(args):
    group = RuntimeGroup(args, "quadruped_stage7_motion_failure")
    node = TestNode("quadruped_stage7_motion_failure_test")
    try:
        start_group(group)
        node.wait_ready(group)
        node.finish_action(node.get_up, get_up_goal(), group, COMPLETED)
        node.spin_until(
            lambda: node.diagnostic.effective_command_sequence != 0,
            3.0,
            "motiond 故障测试前没有有效命令",
            group.assert_running,
        )
        group.stop("motion")
        node.spin_until(
            lambda: node.diagnostic.effective_command_sequence == 0,
            3.0,
            "motiond 停止后后端仍执行旧命令",
            lambda: group.assert_running(("motion",)),
        )
        print("[通过] motiond 停止后的命令过期")
    finally:
        node.destroy_node()
        group.close()


def backend_failure(args):
    group = RuntimeGroup(args, "quadruped_stage7_backend_failure")
    node = TestNode("quadruped_stage7_backend_failure_test")
    try:
        start_group(group)
        node.wait_ready(group)
        _, result_future = node.begin_action(
            node.get_up, get_up_goal(), group
        )
        time.sleep(0.05)
        group.stop("backend")
        wrapped = node.wait_future(
            result_future,
            5.0,
            "backend 停止后活动 action 未返回",
            lambda: None,
        )
        result = wrapped.result.result
        if result.state != FAILED:
            raise AssertionError(
                f"backend 停止后 action 不是 Failed: {result.state}, {result.message}"
            )
        print("[通过] backend 停止后的活动请求失败")
    finally:
        node.destroy_node()
        group.close()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend-script", required=True)
    parser.add_argument("--motion-script", required=True)
    parser.add_argument("--command-script", required=True)
    parser.add_argument("--ipc-control", required=True)
    parser.add_argument("--robot", default="black")
    parser.add_argument("--real-time-factor", type=float, default=1.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    try:
        functional_and_gateway_timeout(args)
        motion_failure(args)
        backend_failure(args)
    finally:
        rclpy.shutdown()
    print(f"{args.robot} ROS 2 headless 端到端测试全部通过")


if __name__ == "__main__":
    main()
