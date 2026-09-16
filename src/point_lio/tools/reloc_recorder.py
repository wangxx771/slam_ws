#!/usr/bin/env python3
"""Recorder for live relocalization tests: subscribes to point_lio's /lio/odom
(already in the map frame once TF is up) and the map->odom transform, writes them
to CSV, and records when map->odom first arrived - that instant is when
small_gicp locked the relocalization.

Usage: reloc_recorder.py <out.csv> [timeout_s]
"""
import sys
import csv
import time

import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage


class Rec(Node):
    def __init__(self, path, timeout, seed_csv=None, seed_after=8.0,
                 err_xy=1.4, err_yaw=12.0):
        super().__init__("reloc_recorder")
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        # mx..mqw hold the latest map->odom as of this row. small_gicp runs with
        # continuous_correction, so the transform keeps being refined; using the
        # value from the lock instant for the whole trajectory is wrong, hence
        # storing it per row. Left empty before lock.
        self.w.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw", "frame",
                         "mx", "my", "mz", "mqx", "mqy", "mqz", "mqw"])
        self.m2o = None
        self.create_subscription(Odometry, "/lio/odom", self.odom, 100)
        self.create_subscription(TFMessage, "/tf", self.tf, 100)
        self.t0 = time.time()
        self.timeout = timeout
        self.lock_wall = None
        self.n = 0
        # The manual seed fires on bag time, not wall clock. Publishing on a wall
        # clock via ros2 topic pub is delayed several seconds by node discovery,
        # and the vehicle covers ten-odd metres in that time, invalidating the
        # seed pose - that produced 15-18 m of localization error, which looked
        # like relocalization failure but was the test script's fault.
        self.seed_gt = np.loadtxt(seed_csv, delimiter=",", skiprows=1) if seed_csv else None
        if self.seed_gt is not None:
            self.seed_gt[:, 0] *= 1e-6
        self.seed_after = seed_after
        self.err_xy, self.err_yaw = err_xy, err_yaw
        self.t_first = None
        self.seed_sent = False
        self.seed_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.create_timer(1.0, self.tick)

    def maybe_seed(self, t_bag):
        if self.seed_gt is None or self.seed_sent:
            return
        if self.t_first is None:
            self.t_first = t_bag
            return
        if t_bag - self.t_first < self.seed_after:
            return
        i = int(np.argmin(np.abs(self.seed_gt[:, 0] - t_bag)))
        x, y, z, qx, qy, qz, qw = self.seed_gt[i, 1:8]
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        yaw += math.radians(self.err_yaw)          # deliberately coarse initial guess
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = float(x + self.err_xy)
        m.pose.pose.position.y = float(y - self.err_xy)
        m.pose.pose.position.z = float(z)
        m.pose.pose.orientation.z = math.sin(yaw / 2)
        m.pose.pose.orientation.w = math.cos(yaw / 2)
        self.seed_pub.publish(m)
        self.seed_sent = True
        print(f"[recorder] sending seed @bag t={t_bag:.2f}: ({x + self.err_xy:.2f}, "
              f"{y - self.err_xy:.2f}) yaw={math.degrees(yaw):.1f}° "
              f"(truth ({x:.2f}, {y:.2f}), offset by {self.err_xy * 1.414:.1f} m / "
              f"{self.err_yaw:.0f}°）", flush=True)

    def odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.maybe_seed(t)
        self.w.writerow([f"{t:.9f}", f"{p.x:.6f}", f"{p.y:.6f}", f"{p.z:.6f}",
                         f"{q.x:.9f}", f"{q.y:.9f}", f"{q.z:.9f}", f"{q.w:.9f}",
                         m.header.frame_id] +
                        ([f"{v:.9f}" for v in self.m2o] if self.m2o else [""] * 7))
        self.n += 1

    def tf(self, m):
        for tr in m.transforms:
            if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                tt, rr = tr.transform.translation, tr.transform.rotation
                self.m2o = (tt.x, tt.y, tt.z, rr.x, rr.y, rr.z, rr.w)
                if self.lock_wall is None:
                    self.lock_wall = time.time() - self.t0
                    t = tr.transform.translation
                    print(f"[recorder] first received map->odom @ {self.lock_wall:.1f}s  "
                          f"t=({t.x:.3f}, {t.y:.3f}, {t.z:.3f})", flush=True)

    def tick(self):
        if time.time() - self.t0 > self.timeout:
            print(f"[recorder] done: {self.n} odom rows, "
                  f"lock took {'%.1fs' % self.lock_wall if self.lock_wall else 'no lock'}",
                  flush=True)
            self.f.close()
            rclpy.shutdown()


def main():
    rclpy.init()
    a = sys.argv
    rclpy.spin(Rec(a[1], float(a[2]) if len(a) > 2 else 120,
                   a[3] if len(a) > 3 and a[3] != "-" else None,
                   float(a[4]) if len(a) > 4 else 8.0))


if __name__ == "__main__":
    main()
