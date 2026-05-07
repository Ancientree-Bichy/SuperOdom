#!/usr/bin/env python3

import argparse
import csv
import os
import statistics
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class OdomMetricRecorder(Node):
    def __init__(self, topics: list[str], output_dir: str):
        super().__init__("odom_metric_recorder")
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        self.rows: dict[str, list[tuple[float, float, float, float]]] = {
            topic: [] for topic in topics
        }
        self.start_wall = time.monotonic()
        self.odom_subscriptions = [
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, topic=topic: self.odom_callback(topic, msg),
                50,
            )
            for topic in topics
        ]
        self.get_logger().info(f"Recording odometry metrics for: {', '.join(topics)}")

    def odom_callback(self, topic: str, msg: Odometry) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = msg.pose.pose.position
        self.rows[topic].append((stamp, pos.x, pos.y, pos.z))

    def write_outputs(self) -> None:
        summary_path = os.path.join(self.output_dir, "odom_summary.txt")
        with open(summary_path, "w", encoding="utf-8") as summary:
            for topic, rows in self.rows.items():
                safe_name = topic.strip("/").replace("/", "_") or "root"
                csv_path = os.path.join(self.output_dir, f"{safe_name}.csv")
                with open(csv_path, "w", newline="", encoding="utf-8") as handle:
                    writer = csv.writer(handle)
                    writer.writerow(["stamp", "x", "y", "z"])
                    writer.writerows(rows)

                if not rows:
                    summary.write(f"{topic}: count=0\n")
                    continue

                zs = [row[3] for row in rows]
                summary.write(
                    f"{topic}: count={len(rows)} "
                    f"z_first={zs[0]:.6f} z_last={zs[-1]:.6f} "
                    f"z_delta={zs[-1] - zs[0]:.6f} "
                    f"z_min={min(zs):.6f} z_max={max(zs):.6f} "
                    f"z_range={max(zs) - min(zs):.6f} "
                    f"z_mean={statistics.fmean(zs):.6f}\n"
                )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record compact odometry z metrics.")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument(
        "--topics",
        nargs="+",
        default=["/laser_odometry", "/imu_odometry", "/body_odometry"],
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = OdomMetricRecorder(args.topics, args.output_dir)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.write_outputs()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
