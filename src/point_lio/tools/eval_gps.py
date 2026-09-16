#!/usr/bin/env python3
"""Evaluate the ATE of an odom.csv against RTK GPS from the bag.

/gps/fix in the park dataset is an RTK fixed solution (sigma_h ~ 0.08 m), good
enough as quasi ground truth. The LIO world frame and ENU differ by a 4-DoF
transform (yaw + 3D translation); solve it with Umeyama first, then take the
residuals - that is standard ATE.

Usage: eval_gps.py <bag_dir> <odom.csv> [<odom.csv> ...]
"""
import math
import sys

import numpy as np
import rclpy.serialization as ser
import rosbag2_py
from sensor_msgs.msg import NavSatFix

A = 6378137.0
F = 1.0 / 298.257223563
E2 = F * (2.0 - F)


def lla_to_ecef(lat, lon, alt):
    la, lo = math.radians(lat), math.radians(lon)
    s, c = math.sin(la), math.cos(la)
    N = A / math.sqrt(1.0 - E2 * s * s)
    return np.array([(N + alt) * c * math.cos(lo),
                     (N + alt) * c * math.sin(lo),
                     (N * (1.0 - E2) + alt) * s])


def read_gps(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""),
           rosbag2_py.ConverterOptions("", ""))
    out = []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic != "/gps/fix":
            continue
        m = ser.deserialize_message(data, NavSatFix)
        if not math.isfinite(m.latitude) or m.status.status < 0:
            continue
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        out.append((t, m.latitude, m.longitude, m.altitude,
                    math.sqrt(max(m.position_covariance[0], 1e-9)), m.status.status))
    if not out:
        sys.exit("the bag has no /gps/fix")
    lat0, lon0, alt0 = out[0][1], out[0][2], out[0][3]
    e0 = lla_to_ecef(lat0, lon0, alt0)
    la, lo = math.radians(lat0), math.radians(lon0)
    sla, cla, slo, clo = math.sin(la), math.cos(la), math.sin(lo), math.cos(lo)
    R = np.array([[-slo, clo, 0.0],
                  [-sla * clo, -sla * slo, cla],
                  [cla * clo, cla * slo, sla]])
    ts = np.array([o[0] for o in out])
    enu = np.array([R @ (lla_to_ecef(o[1], o[2], o[3]) - e0) for o in out])
    sig = np.array([o[4] for o in out])
    st = np.array([o[5] for o in out])
    return ts, enu, sig, st


def read_odom(path):
    import csv
    seen = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            t = float(r["t"])
            seen.setdefault(t, (float(r["x"]), float(r["y"]), float(r["z"])))
    ts = np.array(sorted(seen))
    return ts, np.array([seen[t] for t in ts])


def align_4dof(src, dst):
    """Solve yaw plus translation aligning src to dst; returns (R, t, per-point residuals)."""
    cs, cd = src.mean(0), dst.mean(0)
    a, b = src - cs, dst - cd
    C = (a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]).sum()
    S = (a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]).sum()
    yaw = math.atan2(S, C)
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    t = cd - R @ cs
    res = np.linalg.norm((R @ src.T).T + t - dst, axis=1)
    return yaw, t, res


def main():
    bag = sys.argv[1]
    gt, genu, gsig, gst = read_gps(bag)
    fix = (gst == 2).sum()
    print(f"reference GPS: {len(gt)} points, {gt[-1]-gt[0]:.0f}s, median sigma_h {np.median(gsig):.3f}m, "
          f"RTK fixed {fix}/{len(gt)} ({100*fix/len(gt):.0f}%)")
    gl = np.linalg.norm(np.diff(genu, axis=0), axis=1).sum()
    print(f"          GPS path length {gl:.1f}m, start-to-end {np.linalg.norm(genu[-1]-genu[0]):.2f}m")

    for path in sys.argv[2:]:
        ot, opos = read_odom(path)
        # Use only GPS points inside the odom time range that can be interpolated
        m = (gt >= ot[0]) & (gt <= ot[-1])
        if m.sum() < 20:
            print(f"{path}: insufficient time overlap ({m.sum()} points)")
            continue
        gi = np.stack([np.interp(gt[m], ot, opos[:, k]) for k in range(3)], axis=1)
        yaw, t, res = align_4dof(gi, genu[m])
        ol = np.linalg.norm(np.diff(opos, axis=0), axis=1).sum()
        # Drift by section: align on the first quarter and see how far the rest diverges
        n4 = max(20, m.sum() // 4)
        _, _, res_e = align_4dof(gi[:n4], genu[m][:n4])
        print(f"\n{path}")
        print(f"  odom {len(ot)} points {ot[-1]-ot[0]:.0f}s, path length {ol:.1f}m, "
              f"start-to-end {np.linalg.norm(opos[-1]-opos[0]):.2f}m")
        print(f"  full 4-DoF alignment: yaw={math.degrees(yaw):.2f} deg  "
              f"ATE RMSE {math.sqrt((res**2).mean()):.3f}m  median {np.median(res):.3f}m  "
              f"max {res.max():.3f}m")
        print(f"  aligning on the first {n4} GPS points only, that section's own residual "
              f"{math.sqrt((res_e**2).mean()):.3f}m")


if __name__ == "__main__":
    main()
