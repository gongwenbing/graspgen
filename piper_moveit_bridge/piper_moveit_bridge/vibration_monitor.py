#!/usr/bin/env python3
"""Read-only PiPER motion diagnostics. Never publishes actuator commands."""
import json
import math
import statistics
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def period_stats(samples):
    values = list(samples)
    if not values:
        return {"count": 0, "mean_ms": None, "std_ms": None, "max_ms": None}
    mean = statistics.fmean(values)
    return {
        "count": len(values),
        "mean_ms": mean * 1000.0,
        "std_ms": statistics.pstdev(values) * 1000.0 if len(values) > 1 else 0.0,
        "max_ms": max(values) * 1000.0,
    }


def error_stats(samples):
    values = list(samples)
    if not values:
        return {"count": 0, "rms_rad": None, "max_abs_rad": None}
    return {
        "count": len(values),
        "rms_rad": math.sqrt(statistics.fmean(v * v for v in values)),
        "max_abs_rad": max(abs(v) for v in values),
    }


class VibrationMonitor(Node):
    """Observe command/feedback topics and report timing and position tracking."""

    def __init__(self):
        super().__init__("piper_vibration_monitor")
        self.window_sec = float(self.declare_parameter("window_sec", 5.0).value)
        if not math.isfinite(self.window_sec) or self.window_sec < 1.0:
            raise ValueError("window_sec must be finite and >= 1")
        self.commands = deque()
        self.feedback_errors = deque()
        self.command_periods = deque()
        self.feedback_periods = deque()
        self.last_command_at = None
        self.last_feedback_at = None
        self.latest_command = None
        self.hardware_status = {}
        self.command_count = 0
        self.feedback_count = 0
        self.metrics_pub = self.create_publisher(String, "/piper_vibration/metrics", 10)
        self.create_subscription(
            JointState, "/moveit/joint_commands", self.on_command, 50
        )
        self.create_subscription(
            JointState, "/joint_states_single", self.on_feedback,
            qos_profile_sensor_data
        )
        self.create_subscription(
            String, "/piper_hardware/status", self.on_hardware_status, 10
        )
        self.create_timer(1.0, self.report)
        self.get_logger().info(
            "Read-only vibration monitor started; no CAN/SDK connection and no actuator commands"
        )

    def _trim(self, now):
        cutoff = now - self.window_sec
        for samples in (
            self.commands, self.feedback_errors,
            self.command_periods, self.feedback_periods
        ):
            while samples and samples[0][0] < cutoff:
                samples.popleft()

    @staticmethod
    def _positions(msg):
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return None
        values = dict(zip(msg.name, msg.position))
        if not all(math.isfinite(v) for v in values.values()):
            return None
        return values

    def on_command(self, msg):
        now = time.monotonic()
        if self.last_command_at is not None:
            self.command_periods.append((now, now - self.last_command_at))
        self.last_command_at = now
        values = self._positions(msg)
        if values is None:
            return
        self.latest_command = values
        self.command_count += 1
        self.commands.append((now, self.command_count))
        self._trim(now)

    def on_feedback(self, msg):
        now = time.monotonic()
        if self.last_feedback_at is not None:
            self.feedback_periods.append((now, now - self.last_feedback_at))
        self.last_feedback_at = now
        actual = self._positions(msg)
        if actual is None:
            return
        if self.latest_command is not None:
            common = [
                name for name in self.latest_command
                if name in actual and name.startswith("joint")
            ]
            for name in common:
                self.feedback_errors.append((now, actual[name] - self.latest_command[name]))
        self.feedback_count += 1
        self._trim(now)

    def on_hardware_status(self, msg):
        try:
            data = json.loads(msg.data)
            self.hardware_status = {
                key: data.get(key) for key in (
                    "sdk_command_call_count", "command_received_count",
                    "command_rejected_count", "last_command_error",
                    "outputs_enabled", "reason"
                )
            }
        except (TypeError, ValueError):
            self.hardware_status = {"parse_error": True}

    def report(self):
        now = time.monotonic()
        self._trim(now)
        command_period_values = [v for _, v in self.command_periods]
        feedback_period_values = [v for _, v in self.feedback_periods]
        error_values = [v for _, v in self.feedback_errors]
        payload = {
            "window_sec": self.window_sec,
            "command_period": period_stats(command_period_values),
            "feedback_period": period_stats(feedback_period_values),
            "tracking_error": error_stats(error_values),
            "observed_command_count_total": self.command_count,
            "observed_feedback_count_total": self.feedback_count,
            "hardware": self.hardware_status,
            "note": (
                "Tracking error compares latest observed command with each feedback sample; "
                "it is a diagnostic indicator, not a time-aligned servo error measurement."
            ),
        }
        out = String()
        out.data = json.dumps(payload, separators=(",", ":"), allow_nan=False)
        self.metrics_pub.publish(out)
        self.get_logger().info(out.data)


def main(args=None):
    rclpy.init(args=args)
    node = VibrationMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
