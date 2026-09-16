#!/usr/bin/env python3
"""Convert a Livox .lvx2 recording into a ROS 2 bag with the same topic formats
livox_ros_driver2 publishes online.

Formats follow livox_ros_driver2:
  * /livox/lidar : livox_ros_driver2/msg/CustomMsg (xfer_format=1, what Point-LIO uses)
    - points in metres (raw mm / 1000), line = i % line_num, offset_time in ns
      relative to timebase - see src/comm/pub_handler.cpp: ProcessCartesianHighPoint()
    - timebase / header.stamp = device timestamp (ns) of the frame's first point
      - see src/lddc.cpp: InitCustomMsg()
  * /livox/imu   : sensor_msgs/msg/Imu (acc in g, gyro in rad/s, passed through
      unchanged by the driver) - see src/lddc.cpp: InitImuMsg()

Usage:
  python3 lvx2_to_bag.py IN.lvx2 OUT_BAG_DIR [--freq 10] [--device-index 0]
"""

import argparse
import os
import shutil
import sys

import numpy as np
import rclpy.serialization
import rosbag2_py
from builtin_interfaces.msg import Time as TimeMsg
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from sensor_msgs.msg import Imu

from lvx2_common import (DATA_TYPE_CARTESIAN_HIGH, DATA_TYPE_CARTESIAN_LOW,
                         DATA_TYPE_IMU, Lvx2Reader, imu_from_package,
                         points_from_package)

LIDAR_TOPIC = "/livox/lidar"
IMU_TOPIC = "/livox/imu"
FRAME_ID = "livox_frame"

DEFAULT_POINT_INTERVAL_NS = 5000


def stamp_from_ns(ns):
    t = TimeMsg()
    t.sec = int(ns // 1_000_000_000)
    t.nanosec = int(ns % 1_000_000_000)
    return t


def make_writer(out_dir):
    if os.path.exists(out_dir):
        shutil.rmtree(out_dir)
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=out_dir, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=0, name=LIDAR_TOPIC,
            type="livox_ros_driver2/msg/CustomMsg", serialization_format="cdr")
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=1, name=IMU_TOPIC,
            type="sensor_msgs/msg/Imu", serialization_format="cdr")
    )
    return writer


def probe_point_interval(path, device_lidar_id):
    """Derive the per-point interval (ns) from the timestamp delta between adjacent packages and their point counts."""
    with Lvx2Reader(path) as r:
        prev = None
        deltas = []
        pts = None
        for _, pkgs in r.frames():
            for p in pkgs:
                if p.lidar_id != device_lidar_id:
                    continue
                if p.data_type not in (DATA_TYPE_CARTESIAN_HIGH, DATA_TYPE_CARTESIAN_LOW):
                    continue
                n = len(p.payload) // (14 if p.data_type == DATA_TYPE_CARTESIAN_HIGH else 8)
                if prev is not None and n:
                    deltas.append((p.timestamp - prev) / n)
                    pts = n
                prev = p.timestamp
            if len(deltas) > 500:
                break
        if not deltas:
            return DEFAULT_POINT_INTERVAL_NS, None
        return int(round(float(np.median(deltas)))), pts


def convert(in_path, out_dir, freq, device_index):
    with Lvx2Reader(in_path) as r:
        devices = r.devices
        frame_duration_ms = r.frame_duration_ms
    if not devices:
        sys.exit("no device info block in file")
    if device_index >= len(devices):
        sys.exit(f"--device-index {device_index} out of range (file has {len(devices)})")

    dev = devices[device_index]
    line_num = dev.line_num
    interval_ns, pts_per_pkg = probe_point_interval(in_path, dev.lidar_id)

    print(f"input           : {in_path}")
    print(f"frame_duration  : {frame_duration_ms} ms   devices: {len(devices)}")
    for i, d in enumerate(devices):
        mark = " <== selected" if i == device_index else ""
        print(f"  dev[{i}] sn={d.lidar_sn} lidar_id={d.lidar_id} "
              f"device_type={d.device_type} line_num={d.line_num}{mark}")
    print(f"point_interval  : {interval_ns} ns/point ({pts_per_pkg} pts/pkg)")
    print(f"publish_freq    : {freq} Hz  ->  each CustomMsg covers {1000.0/freq:.0f} ms")

    period_ns = int(round(1e9 / freq))
    writer = make_writer(out_dir)

    n_lidar_msg = 0
    n_imu_msg = 0
    n_points = 0
    n_dropped_other_dev = 0
    cur_base = None          # timebase (ns) of the current CustomMsg
    buf_xyz, buf_ref, buf_tag, buf_line, buf_t = [], [], [], [], []

    def flush():
        nonlocal n_lidar_msg, n_points, cur_base
        nonlocal buf_xyz, buf_ref, buf_tag, buf_line, buf_t
        if cur_base is None or not buf_xyz:
            return
        xyz = np.concatenate(buf_xyz)
        ref = np.concatenate(buf_ref)
        tag = np.concatenate(buf_tag)
        line = np.concatenate(buf_line)
        t_ns = np.concatenate(buf_t)

        msg = CustomMsg()
        msg.header.frame_id = FRAME_ID
        msg.header.stamp = stamp_from_ns(cur_base)
        msg.timebase = int(cur_base)
        msg.point_num = int(xyz.shape[0])
        msg.lidar_id = 0
        offs = (t_ns - cur_base).astype(np.uint32)
        pts = []
        for i in range(xyz.shape[0]):
            p = CustomPoint()
            p.offset_time = int(offs[i])
            p.x = float(xyz[i, 0])
            p.y = float(xyz[i, 1])
            p.z = float(xyz[i, 2])
            p.reflectivity = int(ref[i])
            p.tag = int(tag[i])
            p.line = int(line[i])
            pts.append(p)
        msg.points = pts
        writer.write(LIDAR_TOPIC, rclpy.serialization.serialize_message(msg),
                     int(cur_base))
        n_lidar_msg += 1
        n_points += xyz.shape[0]
        cur_base = None
        buf_xyz, buf_ref, buf_tag, buf_line, buf_t = [], [], [], [], []

    with Lvx2Reader(in_path) as r:
        for _, pkgs in r.frames():
            for p in pkgs:
                if p.data_type == DATA_TYPE_IMU:
                    if p.lidar_id != dev.lidar_id:
                        continue
                    for s in imu_from_package(p):
                        m = Imu()
                        m.header.frame_id = FRAME_ID
                        m.header.stamp = stamp_from_ns(p.timestamp)
                        m.angular_velocity.x = float(s["gyro_x"])
                        m.angular_velocity.y = float(s["gyro_y"])
                        m.angular_velocity.z = float(s["gyro_z"])
                        m.linear_acceleration.x = float(s["acc_x"])
                        m.linear_acceleration.y = float(s["acc_y"])
                        m.linear_acceleration.z = float(s["acc_z"])
                        writer.write(IMU_TOPIC,
                                     rclpy.serialization.serialize_message(m),
                                     int(p.timestamp))
                        n_imu_msg += 1
                    continue

                if p.data_type not in (DATA_TYPE_CARTESIAN_HIGH, DATA_TYPE_CARTESIAN_LOW):
                    continue
                if p.lidar_id != dev.lidar_id:
                    n_dropped_other_dev += 1
                    continue

                if cur_base is not None and p.timestamp - cur_base >= period_ns:
                    flush()
                if cur_base is None:
                    cur_base = p.timestamp

                xyz, ref, tag, line, t_ns = points_from_package(p, line_num, interval_ns)
                buf_xyz.append(xyz)
                buf_ref.append(ref)
                buf_tag.append(tag)
                buf_line.append(line)
                buf_t.append(t_ns)
        flush()

    del writer
    print(f"\nwrote           : {out_dir}")
    print(f"  {LIDAR_TOPIC}: {n_lidar_msg} msgs, {n_points} points")
    print(f"  {IMU_TOPIC}  : {n_imu_msg} msgs"
          + ("   <== the file has no IMU packages; run with imu_en:=False" if n_imu_msg == 0 else ""))
    if n_dropped_other_dev:
        print(f"  skipped {n_dropped_other_dev} packages from other devices "
              f"(use --device-index to pick another device)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument("--freq", type=float, default=10.0,
                    help="CustomMsg publish rate in Hz, matching the driver publish_freq (default 10)")
    ap.add_argument("--device-index", type=int, default=0,
                    help="which device to use in a multi-lidar file (default 0)")
    a = ap.parse_args()
    convert(a.input, a.output, a.freq, a.device_index)


if __name__ == "__main__":
    main()
