#!/usr/bin/env python3

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class InitialPoseWaiter(Node):
    def __init__(self, topic: str):
        super().__init__("initial_pose_waiter")
        self._received = False
        self._subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            topic,
            self._callback,
            10,
        )

    def _callback(self, msg: PoseWithCovarianceStamped) -> None:
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.get_logger().info(
            "Received /initialpose: x=%.3f y=%.3f z=%.3f yaw=%.3f"
            % (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                yaw,
            )
        )
        self._received = True

    @property
    def received(self) -> bool:
        return self._received


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Wait for a single /initialpose message.")
    parser.add_argument("--topic", default="/initialpose")
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=0.0,
        help="0 means wait forever.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = InitialPoseWaiter(args.topic)

    try:
        if args.timeout_sec > 0.0:
            end_time = node.get_clock().now().nanoseconds / 1e9 + args.timeout_sec
            while rclpy.ok() and not node.received:
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.get_clock().now().nanoseconds / 1e9 > end_time:
                    node.get_logger().error("Timed out waiting for /initialpose.")
                    return 1
        else:
            while rclpy.ok() and not node.received:
                rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
