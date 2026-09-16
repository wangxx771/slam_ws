#!/usr/bin/env python3
"""Trim a bag to a time interval, keeping the original timestamps so it still
aligns with the original ground truth.

Relocalization tests need this to start from anywhere in the middle of a
trajectory.

Usage:
  slice_bag.py <src_bag> <out_bag> --start 30 --end 90     # seconds from the bag start
"""

import argparse
import os
import shutil

import rosbag2_py

TYPES = {
    "/livox/lidar": "livox_ros_driver2/msg/CustomMsg",
    "/livox/imu": "sensor_msgs/msg/Imu",
    "/gnss": "sensor_msgs/msg/NavSatFix",
    "/groundtruth": "nav_msgs/msg/Odometry",
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src_bag")
    ap.add_argument("out_bag")
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--end", type=float, default=1e9)
    a = ap.parse_args()

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=a.src_bag, storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    msgs = []
    while r.has_next():
        topic, raw, t_ns = r.read_next()
        if topic in TYPES:
            msgs.append((t_ns, topic, raw))
    msgs.sort(key=lambda x: x[0])
    t0 = msgs[0][0]
    lo, hi = t0 + int(a.start * 1e9), t0 + int(a.end * 1e9)

    if os.path.exists(a.out_bag):
        shutil.rmtree(a.out_bag)
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=a.out_bag, storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    for i, (name, tp) in enumerate(TYPES.items()):
        w.create_topic(rosbag2_py.TopicMetadata(
            id=i, name=name, type=tp, serialization_format="cdr"))

    n = 0
    for t_ns, topic, raw in msgs:
        if lo <= t_ns <= hi:
            w.write(topic, raw, t_ns)
            n += 1
    del w
    print(f"wrote {a.out_bag}: {n} messages  [{a.start}, {a.end}]s")


if __name__ == "__main__":
    main()
