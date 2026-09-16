#!/usr/bin/env python3

import argparse
import os
import shutil
import struct
import sys

import numpy as np
import rclpy.serialization
import rosbag2_py
from builtin_interfaces.msg import Time as TimeMsg
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix

MAGIC = 0x4C494452
HDR = struct.Struct("<IQI")
PT_SIZE = 8
CM = 0.01

LIDAR_TOPIC = "/livox/lidar"
IMU_TOPIC = "/livox/imu"
GNSS_TOPIC = "/gnss"
GT_TOPIC = "/groundtruth"
FRAME_ID = "livox_frame"

N_RINGS = 32       
EL_MIN, EL_MAX = -30.0, 10.0


def stamp_from_us(us):
    t = TimeMsg()
    t.sec = int(us // 1_000_000)
    t.nanosec = int((us % 1_000_000) * 1000)
    return t


def read_lidar_frames(path):
    """Yields (ts_us, xyz[N,3] float32 metres, intensity[N] uint8), deduplicated by timestamp."""
    size = os.path.getsize(path)
    seen = set()
    with open(path, "rb") as f:
        off = 0
        while off + HDR.size <= size:
            head = f.read(HDR.size)
            if len(head) < HDR.size:
                return
            magic, ts, npts = HDR.unpack(head)
            if magic != MAGIC:
                print(f"  !! magic mismatch at offset {off}, stopping", file=sys.stderr)
                return
            body = f.read(npts * PT_SIZE)
            off += HDR.size + npts * PT_SIZE
            if ts in seen:          # the same frame was written more than once
                continue
            seen.add(ts)
            raw = np.frombuffer(body, dtype=np.uint8).reshape(-1, PT_SIZE)
            xyz = raw[:, :6].copy().view("<i2").reshape(-1, 3).astype(np.float32) * CM
            yield ts, xyz, raw[:, 7].copy()


def convert(src_dir, out_dir, merge_rev=False, time_mode='index', time_bins=0,
            raw_gyro=True):
    lidar_bin = os.path.join(src_dir, "lidar_data.bin")
    imu_csv = os.path.join(src_dir, "imu_data.csv")
    gps_csv = os.path.join(src_dir, "gps_data.csv")
    gt_csv = os.path.join(src_dir, "groundtruth_pose.csv")
    for p in (lidar_bin, imu_csv, gps_csv, gt_csv):
        if not os.path.exists(p):
            sys.exit(f"missing input file: {p}")

    if os.path.exists(out_dir):
        shutil.rmtree(out_dir)
    w = rosbag2_py.SequentialWriter()
    w.open(
        rosbag2_py.StorageOptions(uri=out_dir, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    for i, (name, tp) in enumerate([
        (LIDAR_TOPIC, "livox_ros_driver2/msg/CustomMsg"),
        (IMU_TOPIC, "sensor_msgs/msg/Imu"),
        (GNSS_TOPIC, "sensor_msgs/msg/NavSatFix"),
        (GT_TOPIC, "nav_msgs/msg/Odometry"),
    ]):
        w.create_topic(rosbag2_py.TopicMetadata(
            id=i, name=name, type=tp, serialization_format="cdr"))


    aux = []          # (ts_us, topic, msg)

    a = np.genfromtxt(imu_csv, delimiter=",", skip_header=1)
    n_imu = n_imu_bad = 0
    prev = None
    ACC_MAX, GYR_MAX = 100.0, 20.0
    for row in a:
        us = int(row[0])
        if us == prev:
            continue
        prev = us
        if (not np.all(np.isfinite(row[1:7])) or
                np.linalg.norm(row[1:4]) > ACC_MAX or np.abs(row[4:7]).max() > GYR_MAX):
            n_imu_bad += 1
            continue
        m = Imu()
        m.header.frame_id = FRAME_ID
        m.header.stamp = stamp_from_us(us)
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = row[1:4]
        if raw_gyro:
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = row[4:7]
        else:
            m.angular_velocity.x = -row[4]
            m.angular_velocity.y = -row[5]
            m.angular_velocity.z = row[6]
        aux.append((us, IMU_TOPIC, m))
        n_imu += 1

    a = np.genfromtxt(gps_csv, delimiter=",", skip_header=1)
    n_gps = 0
    prev = None
    for row in a:
        us = int(row[0])
        if us == prev:
            continue
        prev = us
        m = NavSatFix()
        m.header.frame_id = FRAME_ID
        m.header.stamp = stamp_from_us(us)
        m.status.status = 0
        m.latitude, m.longitude, m.altitude = row[1], row[2], row[3]
        m.position_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.04]
        m.position_covariance_type = 2
        aux.append((us, GNSS_TOPIC, m))
        n_gps += 1

    a = np.genfromtxt(gt_csv, delimiter=",", skip_header=1)
    n_gt = 0
    prev = None
    for row in a:
        us = int(row[0])
        if us == prev:
            continue
        prev = us
        m = Odometry()
        m.header.frame_id = "map"
        m.child_frame_id = "gt"
        m.header.stamp = stamp_from_us(us)
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = row[1:4]
        (m.pose.pose.orientation.x, m.pose.pose.orientation.y,
         m.pose.pose.orientation.z, m.pose.pose.orientation.w) = row[4:8]
        aux.append((us, GT_TOPIC, m))
        n_gt += 1

    aux.sort(key=lambda x: x[0])
    aux_i = 0

    def flush_aux_until(ts_us):
        """Flush auxiliary messages with timestamps <= ts_us first, keeping the bag strictly time-ordered."""
        nonlocal aux_i
        while aux_i < len(aux) and aux[aux_i][0] <= ts_us:
            us, topic, msg = aux[aux_i]
            w.write(topic, rclpy.serialization.serialize_message(msg), us * 1000)
            aux_i += 1

    # ---------- LiDAR ----------
    n_frames = n_points = 0
    prev_ts = None
    pending = []           # collect first, then derive per-frame duration from the frame interval
    for ts, xyz, inten in read_lidar_frames(lidar_bin):
        pending.append((ts, xyz, inten))
    if len(pending) < 2:
        sys.exit("not enough lidar frames")
    dts = np.diff([p[0] for p in pending])
    frame_us = int(np.median(dts))
    print(f"lidar: {len(pending)} frames after dedup, median frame interval {frame_us} us")


    def sector_center(xyz):
        az = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]))
        return float(np.median(az))

    centers = [sector_center(p[1]) for p in pending if p[1].shape[0]]
    steps = np.diff(centers)
    steps = steps[np.abs(steps) < 180]
    step = float(np.median(steps)) if len(steps) else -72.0
    per_rev = max(1, int(round(360.0 / abs(step)))) if abs(step) > 1e-6 else 1
    if merge_rev:
        print(f"       sector span ~{abs(step):.0f} deg/frame -> {per_rev} frames per revolution "
              f"({per_rev * frame_us / 1000.0:.0f} ms/rev)")
    else:

        print(f"       sector span ~{abs(step):.0f} deg/frame ({per_rev} frames per revolution); "
              f"publishing per sector (--merge-rev aggregates full revolutions)")
        per_rev = 1

    for g0 in range(0, len(pending), per_rev):
        group = pending[g0:g0 + per_rev]
        group = [g for g in group if g[1].shape[0] > 0]
        if not group:
            continue
        ts0 = group[0][0]
        px, pi, po, pl = [], [], [], []
        for (ts_s, xyz, inten) in group:
            rng = np.linalg.norm(xyz, axis=1)
            keep = rng > 1e-3
            xyz, inten = xyz[keep], inten[keep]
            if xyz.shape[0] == 0:
                continue
            if time_mode == "index":

                n_ = xyz.shape[0]
                frac = np.arange(n_, dtype=np.float64) / max(n_ - 1, 1)
            else:
                az = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]))
                c = np.median(az)
                az_c = (az - c + 180.0) % 360.0 - 180.0
                lo, hi = az_c.min(), az_c.max()
                span = hi - lo
                frac = (hi - az_c) / span if span > 1e-9 else np.zeros_like(az_c)
            # Time quantization: Point-LIO's time_compressing() groups points by
            # equal curvature and runs one sequential EKF update per group.
            # Unquantized, a frame splits into thousands of slices of a few points
            # each (indexing time by point order means adjacent points on the same
            # ring, geometrically almost a line), which barely constrains
            # roll/pitch. Quantizing to N slices multiplies points per slice by
            # total/N.
            if time_bins > 0:
                frac = np.floor(frac * time_bins) / max(time_bins - 1, 1)
                frac = np.clip(frac, 0.0, 1.0)
            off = (ts_s - ts0) * 1000.0 + frac * frame_us * 1000.0     # ns
            el = np.degrees(np.arctan2(xyz[:, 2], np.hypot(xyz[:, 0], xyz[:, 1])))
            ring = np.clip(((el - EL_MIN) / (EL_MAX - EL_MIN) * (N_RINGS - 1)).round(),
                           0, N_RINGS - 1).astype(np.uint8)
            px.append(xyz); pi.append(inten); po.append(off); pl.append(ring)
        if not px:
            continue
        xyz = np.concatenate(px); inten = np.concatenate(pi)
        offs = np.concatenate(po).astype(np.uint32); ring = np.concatenate(pl)
        order = np.argsort(offs, kind="stable")          # Point-LIO requires increasing time
        xyz, inten, offs, ring = xyz[order], inten[order], offs[order], ring[order]

        m = CustomMsg()
        m.header.frame_id = FRAME_ID
        m.header.stamp = stamp_from_us(ts0)
        m.timebase = int(ts0) * 1000
        m.point_num = int(xyz.shape[0])
        m.lidar_id = 0
        pts = []
        for j in range(xyz.shape[0]):
            p = CustomPoint()
            p.offset_time = int(offs[j])
            p.x = float(xyz[j, 0]); p.y = float(xyz[j, 1]); p.z = float(xyz[j, 2])
            p.reflectivity = int(inten[j]); p.tag = 0; p.line = int(ring[j])
            pts.append(p)
        m.points = pts
        flush_aux_until(int(ts0))
        w.write(LIDAR_TOPIC, rclpy.serialization.serialize_message(m), int(ts0) * 1000)
        n_frames += 1
        n_points += xyz.shape[0]

    flush_aux_until(1 << 62)   

    del w
    print(f"wrote {out_dir}")
    print(f"  {LIDAR_TOPIC}: {n_frames} frames, {n_points} points")
    print(f"  {IMU_TOPIC}  : {n_imu}" + (f"   (dropped {n_imu_bad} non-physical spikes)" if n_imu_bad else ""))
    print(f"  {GNSS_TOPIC} : {n_gps}")
    print(f"  {GT_TOPIC}   : {n_gt}   (offline evaluation only, do not feed to SLAM)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src_dir", help="directory holding lidar_data.bin and *.csv")
    ap.add_argument("out_dir")
    ap.add_argument("--time-mode", choices=["index", "azimuth"], default="index",
                    help="how per-point time is assigned. index=spread over the frame by point order "
                         "(reproduces the shell carla_handler, ATE 0.122 m); azimuth=by azimuth "
                         "(more physical but 3x worse in practice, ATE 0.439 m)")
    ap.add_argument("--fix-gyro", action="store_true",
                    help="apply the pseudo-vector sign correction (-wx,-wy,wz) to the gyro. Off by "
                         "default: the recorder's sign convention is self-consistent with the "
                         "lidar frame, and correcting it measured slightly worse (ATE 0.140 vs 0.122)")
    ap.add_argument("--time-bins", type=int, default=0,
                    help="quantize in-frame time into this many slices (0 disables). Fewer slices means more points per EKF update")
    ap.add_argument("--merge-rev", action="store_true",
                    help="merge each 5 sectors of 72 deg into a full revolution before publishing (worse on this data, off by default)")
    a = ap.parse_args()
    convert(a.src_dir, a.out_dir, merge_rev=a.merge_rev, time_mode=a.time_mode,
            time_bins=a.time_bins, raw_gyro=not a.fix_gyro)


if __name__ == "__main__":
    main()
