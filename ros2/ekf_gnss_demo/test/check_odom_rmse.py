#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Validate live /ekf_gnss/odometry vs /sim/truth (matched seed 1, PLAN threshold)."""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Empty


def stamp_key(msg) -> int:
    stamp = msg.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def finite_odom(msg: Odometry) -> bool:
    p = msg.pose.pose.position
    t = msg.twist.twist.linear
    vals = (p.x, p.y, p.z, t.x, t.y, t.z)
    return all(math.isfinite(v) for v in vals)


class OdomChecker(Node):
    def __init__(self) -> None:
        super().__init__("odom_rmse_checker")
        self.odoms: list[Odometry] = []
        self.truths: dict[int, tuple[float, float]] = {}
        self.complete = False
        self.create_subscription(Odometry, "/ekf_gnss/odometry", self.on_odom, 10)
        self.create_subscription(PoseStamped, "/sim/truth", self.on_truth, 10)
        self.create_subscription(Empty, "/sim/complete", self.on_complete, 10)

    def on_odom(self, msg: Odometry) -> None:
        self.odoms.append(msg)

    def on_truth(self, msg: PoseStamped) -> None:
        self.truths[stamp_key(msg)] = (msg.pose.position.x, msg.pose.position.y)

    def on_complete(self, _msg: Empty) -> None:
        self.complete = True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--min-messages", type=int, default=4000)
    parser.add_argument("--rmse-max-m", type=float, default=1.0)
    parser.add_argument("--timeout-s", type=float, default=120.0)
    args = parser.parse_args()

    rclpy.init()
    node = OdomChecker()
    t_end = time.monotonic() + args.timeout_s
    while rclpy.ok() and time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.complete:
            break

    if not node.complete:
        print("TIMEOUT: /sim/complete not received", file=sys.stderr)
        rclpy.shutdown()
        return 2

    if len(node.odoms) < args.min_messages:
        print(
            f"FAIL: expected >={args.min_messages} odom messages, got {len(node.odoms)}",
            file=sys.stderr,
        )
        rclpy.shutdown()
        return 1

    for msg in node.odoms:
        if not finite_odom(msg):
            print("FAIL: non-finite odometry field", file=sys.stderr)
            rclpy.shutdown()
            return 1

    sq_err = []
    for odom in node.odoms:
        key = stamp_key(odom)
        truth = node.truths.get(key)
        if truth is None:
            continue
        te, tn = truth
        de = odom.pose.pose.position.x - te
        dn = odom.pose.pose.position.y - tn
        sq_err.append(de * de + dn * dn)

    if len(sq_err) < args.min_messages:
        print(
            f"FAIL: only {len(sq_err)} odom/truth pairs for RMSE (need {args.min_messages})",
            file=sys.stderr,
        )
        rclpy.shutdown()
        return 1

    rmse = math.sqrt(sum(sq_err) / len(sq_err))
    print(f"RMSE horizontal: {rmse:.4f} m over {len(sq_err)} pairs")
    if rmse > args.rmse_max_m:
        print(f"FAIL: RMSE {rmse:.4f} m > {args.rmse_max_m} m", file=sys.stderr)
        rclpy.shutdown()
        return 1

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
