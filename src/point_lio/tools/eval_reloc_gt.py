#!/usr/bin/env python3
"""Evaluate relocalization accuracy against /groundtruth in the bag, for data
that has it (CARLA and similar).

The only difference from eval_reloc.py is where the reference comes from: that
one uses the odom.csv of the mapping run, this one uses simulator ground truth,
which allows evaluating cases far from the mapping trajectory - the mapping
trajectory simply has no reference point there.

Assumes GNSS was on during mapping, so the map frame is close to the truth frame
(CARLA's GNSS origin is the vehicle start).

Usage: eval_reloc_gt.py <bag> <reloc_dir> [<reloc_dir> ...]
"""
import os
import sys

import numpy as np
import rclpy.serialization as ser
import rosbag2_py
from nav_msgs.msg import Odometry


def read_gt(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""),
           rosbag2_py.ConverterOptions("", ""))
    rows = []
    while r.has_next():
        tp, data, _ = r.read_next()
        if tp != "/groundtruth":
            continue
        m = ser.deserialize_message(data, Odometry)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p, q = m.pose.pose.position, m.pose.pose.orientation
        rows.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    return np.array(rows)


def yaw_of(q):
    x, y, z, w = q
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def main():
    gt = read_gt(sys.argv[1])
    print(f"ground truth {len(gt)} rows  t=[{gt[0,0]:.1f}, {gt[-1,0]:.1f}]")
    print(f"{'case':>10} {'lock (s)':>8} {'position error med/p95/max (m)':>32} {'yaw (deg)':>11}")
    ok = []
    for d in sys.argv[2:]:
        tag = os.path.basename(d)
        f = os.path.join(d, "odom.csv")
        if not os.path.exists(f):
            print(f"{tag:>10} {'no output':>8}")
            continue
        rows = [l.split(",") for l in open(f).read().splitlines()[1:]]
        rows = [r for r in rows if len(r) == 16 and r[9] != ""]   # keep only locked rows
        if len(rows) < 20:
            print(f"{tag:>10} {'no lock':>8}")
            ok.append(None)
            continue
        a = np.array([[float(v) for v in r[:8]] for r in rows])
        a = a[a[:, 0] - a[0, 0] > 2.0]      # allow 2 s after lock for TF to settle
        t = a[:, 0]
        ref = np.stack([np.interp(t, gt[:, 0], gt[:, k]) for k in (1, 2, 3)], 1)
        e = np.linalg.norm(a[:, 1:4] - ref, axis=1)
        gy = np.interp(t, gt[:, 0], np.unwrap(yaw_of(gt[:, 4:8].T)))
        ey = np.degrees(np.abs((yaw_of(a[:, 4:8].T) - gy + np.pi) % (2 * np.pi) - np.pi))
        lock = open(os.path.join(d, "rec.log")).read()
        ls = [l for l in lock.splitlines() if "first received" in l]
        lt = ls[0].split("@")[1].split("s")[0].strip() if ls else "?"
        print(f"{tag:>10} {lt+'s':>8}   {np.median(e):8.3f} {np.percentile(e,95):8.3f} "
              f"{e.max():8.3f}   {np.median(ey):7.2f}")
        ok.append(np.median(e))
    good = [x for x in ok if x is not None]
    if good:
        print(f"\nlocked {len(good)}/{len(ok)}   median {np.median(good):.3f} m   "
              f"worst {max(good):.3f} m")


if __name__ == "__main__":
    main()
