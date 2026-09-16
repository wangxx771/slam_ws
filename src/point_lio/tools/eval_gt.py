#!/usr/bin/env python3
"""Evaluate SLAM output against a ground-truth trajectory (ATE and loop closure
error).

Usage: eval_gt.py <odom.csv> <bag_dir_with_/groundtruth> [--align]

Ground truth is /groundtruth (carla_to_bag.py) or /ground_truth/odom (MuJoCo). The
two trajectories are paired by timestamp interpolation first; --align adds a
Umeyama rigid alignment (removing differences in the start-pose definition) and
reports both sets of numbers.
"""

import argparse
import sys

import numpy as np


def read_gt(bag):
    import rclpy.serialization
    import rosbag2_py
    from nav_msgs.msg import Odometry
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    rows = []
    while r.has_next():
        tp, data, _ = r.read_next()
        # MuJoCo scenes publish /ground_truth/odom; carla_to_bag.py writes
        # /groundtruth. Accept either so both sources evaluate the same way.
        if tp not in ("/groundtruth", "/ground_truth/odom"):
            continue
        m = rclpy.serialization.deserialize_message(data, Odometry)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        rows.append((t, p.x, p.y, p.z))
    return np.array(rows)


def umeyama(src, dst):
    """Solve R, t such that R*src + t is approximately dst (no scale)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S, D = src - mu_s, dst - mu_d
    H = S.T @ D / len(src)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    return R, mu_d - R @ mu_s


def report(name, err):
    print(f"  {name}: RMSE={np.sqrt((err**2).mean()):.3f} m  "
          f"mean={err.mean():.3f}  median={np.median(err):.3f}  "
          f"p95={np.percentile(err,95):.3f}  max={err.max():.3f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("odom_csv")
    ap.add_argument("bag")
    ap.add_argument(
        "--keyframes", action="store_true",
        help="the CSV is replay.keyframe_csv; evaluate the optimized keyframe "
             "poses (columns 4-6) instead of columns 1-3. Loop closure is not "
             "visible any other way: pg_correct deliberately leaves /odom "
             "untouched so the control loop stays continuous, so ATE on "
             "odom.csv is identical whatever the loop settings are.")
    a = ap.parse_args()

    est = np.loadtxt(a.odom_csv, delimiter=",", skiprows=1, ndmin=2)
    if a.keyframes:
        est = est[:, [0, 4, 5, 6]]
    gt = read_gt(a.bag)
    if gt.size == 0:
        sys.exit("the bag has neither /groundtruth nor /ground_truth/odom")

    te, pe = est[:, 0], est[:, 1:4]
    tg, pg = gt[:, 0], gt[:, 1:4]

    # Interpolate ground truth onto the estimate's timestamps
    m = (te >= tg[0]) & (te <= tg[-1])
    te, pe = te[m], pe[m]
    if len(te) < 10:
        sys.exit("the estimate and ground truth have no time overlap")
    pg_i = np.stack([np.interp(te, tg, pg[:, k]) for k in range(3)], axis=1)

    print(f"{len(te)} paired points   duration {te[-1]-te[0]:.1f}s")
    print(f"ground-truth path {np.linalg.norm(np.diff(pg_i,axis=0),axis=1).sum():.1f} m   "
          f"estimated path {np.linalg.norm(np.diff(pe,axis=0),axis=1).sum():.1f} m")

    report("ATE (unaligned, absolute)", np.linalg.norm(pe - pg_i, axis=1))

    e0 = pe - pe[0]
    g0 = pg_i - pg_i[0]
    report("ATE (start translation aligned)", np.linalg.norm(e0 - g0, axis=1))

    R, t = umeyama(e0, g0)
    report("ATE (Umeyama rigid alignment)", np.linalg.norm((e0 @ R.T + t) - g0, axis=1))

    print(f"  endpoint error (before alignment) = {np.linalg.norm(e0[-1]-g0[-1]):.3f} m   "
          f"of ground-truth path {100*np.linalg.norm(e0[-1]-g0[-1])/max(np.linalg.norm(np.diff(g0,axis=0),axis=1).sum(),1e-9):.2f}%")


if __name__ == "__main__":
    main()
