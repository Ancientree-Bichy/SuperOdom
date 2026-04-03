#!/usr/bin/env python3

import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from super_odometry_msgs.msg import OptimizationStats


class StatsMonitor(Node):
    def __init__(self, topic: str, min_period: float, log_file: str | None):
        super().__init__("superodom_stats_monitor")
        self.topic = topic
        self.min_period = min_period
        self.log_handle = None
        self.message_count = 0
        self.start_time = time.monotonic()
        self.last_print_time = 0.0

        if log_file:
            os.makedirs(os.path.dirname(os.path.abspath(log_file)), exist_ok=True)
            self.log_handle = open(log_file, "a", encoding="utf-8")

        self.subscription = self.create_subscription(
            OptimizationStats,
            topic,
            self.stats_callback,
            10,
        )
        self._emit(f"Listening on {topic}")

    def _emit(self, line: str) -> None:
        print(line, flush=True)
        if self.log_handle is not None:
            self.log_handle.write(line + "\n")
            self.log_handle.flush()

    def stats_callback(self, msg: OptimizationStats) -> None:
        self.message_count += 1
        now = time.monotonic()
        if now - self.last_print_time < self.min_period:
            return

        self.last_print_time = now
        elapsed = now - self.start_time
        line = (
            f"[{elapsed:6.1f}s] "
            f"#{self.message_count:04d} "
            f"iter={msg.n_iterations:02d} "
            f"lat={msg.latency:6.1f}ms "
            f"dT={msg.translation_from_last:6.3f} "
            f"dR={msg.rotation_from_last:6.3f} "
            f"avg={msg.average_distance:6.3f} "
            f"surf={msg.laser_cloud_surf_stack_num:4d}/{msg.laser_cloud_surf_from_map_num:4d} "
            f"corner={msg.laser_cloud_corner_stack_num:4d}/{msg.laser_cloud_corner_from_map_num:4d} "
            f"u_xyz=[{msg.uncertainty_x:0.2f},{msg.uncertainty_y:0.2f},{msg.uncertainty_z:0.2f}] "
            f"u_rpy=[{msg.uncertainty_roll:0.2f},{msg.uncertainty_pitch:0.2f},{msg.uncertainty_yaw:0.2f}]"
        )
        self._emit(line)

    def close(self) -> None:
        if self.log_handle is not None and not self.log_handle.closed:
            self.log_handle.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Print a compact live summary of /super_odometry_stats."
    )
    parser.add_argument(
        "--topic",
        default="/super_odometry_stats",
        help="OptimizationStats topic to subscribe to.",
    )
    parser.add_argument(
        "--min-period",
        type=float,
        default=0.5,
        help="Minimum time between console updates in seconds.",
    )
    parser.add_argument(
        "--log-file",
        default="",
        help="Optional text log path for the printed summaries.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    log_file = args.log_file or None

    rclpy.init()
    node = StatsMonitor(args.topic, args.min_period, log_file)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._emit(f"Stopped after {node.message_count} stats messages")
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
