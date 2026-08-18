#!/usr/bin/env python3

from __future__ import annotations

import os
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

_HERE = os.path.dirname(os.path.abspath(__file__))
for _cand in (os.path.abspath(os.path.join(_HERE, "..", "..", "..")),
              os.environ.get("CANDRIVE_ROOT", "")):
    if _cand and os.path.isdir(os.path.join(_cand, "candrive")):
        sys.path.insert(0, _cand)
        break

from candrive.runner import DEFAULT_SCENARIO, SQUARE_SCENARIO   # noqa: E402

SCENARIOS = {"default": DEFAULT_SCENARIO, "square": SQUARE_SCENARIO}


class ScenarioNode(Node):
    def __init__(self):
        super().__init__("scenario_publisher")
        self.declare_parameter("scenario", "default")
        self.declare_parameter("loop", False)
        self.declare_parameter("rate_hz", 20.0)

        name = self.get_parameter("scenario").value
        self.loop = bool(self.get_parameter("loop").value)
        self.segments = SCENARIOS.get(name, DEFAULT_SCENARIO)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.idx = 0
        self.elapsed = 0.0
        self.dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self.create_timer(self.dt, self.on_tick)
        self.get_logger().info(f"시나리오 '{name}' 시작 ({len(self.segments)} 구간)")

    def on_tick(self) -> None:
        if self.idx >= len(self.segments):
            self.pub.publish(Twist())
            if self.loop:
                self.idx, self.elapsed = 0, 0.0
            return
        seg = self.segments[self.idx]
        if self.elapsed == 0.0:
            self.get_logger().info(
                f"[{self.idx+1}/{len(self.segments)}] {seg.name} "
                f"(v={seg.linear:.2f}, w={seg.angular:.2f}, {seg.duration:.1f}s)")
        msg = Twist()
        msg.linear.x = float(seg.linear)
        msg.angular.z = float(seg.angular)
        self.pub.publish(msg)
        self.elapsed += self.dt
        if self.elapsed >= seg.duration:
            self.idx += 1
            self.elapsed = 0.0


def main(argv=None):
    rclpy.init(args=argv)
    node = ScenarioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
