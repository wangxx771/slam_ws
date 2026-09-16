#!/usr/bin/env python3
"""Concatenate a bag forwards and backwards alternately to synthesize a long
trajectory with many revisits, for testing loop closure.


Usage:
  make_loop_bag.py <src_bag> <out_bag> [--segments 6]
"""

import argparse
import os
import shutil
import struct
import sys

import rclpy.serialization
import rosbag2_py
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix

LIDAR_TOPIC = "/livox/lidar"
IMU_TOPIC = "/livox/imu"
GNSS_TOPIC = "/gnss"
GT_TOPIC = "/groundtruth"

TYPES = {
    LIDAR_TOPIC: ("livox_ros_driver2/msg/CustomMsg", CustomMsg),
    IMU_TOPIC: ("sensor_msgs/msg/Imu", Imu),
    GNSS_TOPIC: ("sensor_msgs/msg/NavSatFix", NavSatFix),
    GT_TOPIC: ("nav_msgs/msg/Odometry", Odometry),
}

# CDR layout of CustomMsg (frame_id is always "livox_frame", 11 characters):
#   [0:4]   encapsulation header
#   [4:8]   header.stamp.sec      int32
#   [8:12]  header.stamp.nanosec  uint32
#   [12:16] frame_id length       uint32 (=12, including the trailing \0)
#   [16:28] frame_id characters
#   [28:36] timebase              uint64 (8-byte aligned, which lands exactly at 28)
OFF_SEC, OFF_NSEC, OFF_TIMEBASE = 4, 8, 28


def patch_lidar(raw, us):
    """Rewrite a lidar message timestamp without deserializing. us is the new time in microseconds."""
    sec, nsec = int(us // 1_000_000), int((us % 1_000_000) * 1000)
    b = bytearray(raw)
    b[OFF_SEC:OFF_SEC + 4] = struct.pack("<i", sec)
    b[OFF_NSEC:OFF_NSEC + 4] = struct.pack("<I", nsec)
    b[OFF_TIMEBASE:OFF_TIMEBASE + 8] = struct.pack("<Q", int(us) * 1000)
    return bytes(b)


def verify_layout(raw):
    """Verify the offsets above against a real deserialization and exit on mismatch."""
    m = rclpy.serialization.deserialize_message(raw, CustomMsg)
    sec = struct.unpack_from("<i", raw, OFF_SEC)[0]
    nsec = struct.unpack_from("<I", raw, OFF_NSEC)[0]
    tb = struct.unpack_from("<Q", raw, OFF_TIMEBASE)[0]
    ok = (sec == m.header.stamp.sec and nsec == m.header.stamp.nanosec and
          tb == m.timebase and m.header.frame_id == "livox_frame")
    if not ok:
        sys.exit(
            f"CustomMsg byte layout differs from expectation (frame_id={m.header.frame_id!r}): "
            f"sec {sec} vs {m.header.stamp.sec}, nsec {nsec} vs {m.header.stamp.nanosec}, "
            f"timebase {tb} vs {m.timebase}")
    print(f"  byte layout verified (frame_id={m.header.frame_id!r}, {m.point_num} points/frame)")


def read_all(src):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=src, storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    out = []
    while r.has_next():
        topic, raw, t_ns = r.read_next()
        if topic in TYPES:
            out.append((t_ns // 1000, topic, raw))
    out.sort(key=lambda x: x[0])
    return out


def reverse_msg(topic, raw):
    """Messages whose contents must change on a reversed segment: IMU only (negate the gyro)."""
    if topic != IMU_TOPIC:
        return raw
    m = rclpy.serialization.deserialize_message(raw, Imu)
    m.angular_velocity.x = -m.angular_velocity.x
    m.angular_velocity.y = -m.angular_velocity.y
    m.angular_velocity.z = -m.angular_velocity.z
    return rclpy.serialization.serialize_message(m)


def restamp(topic, raw, us):
    if topic == LIDAR_TOPIC:
        return patch_lidar(raw, us)
    cls = TYPES[topic][1]
    m = rclpy.serialization.deserialize_message(raw, cls)
    m.header.stamp.sec = int(us // 1_000_000)
    m.header.stamp.nanosec = int((us % 1_000_000) * 1000)
    return rclpy.serialization.serialize_message(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src_bag")
    ap.add_argument("out_bag")
    ap.add_argument("--segments", type=int, default=6,
                    help="how many segments to concatenate (odd segments are reversed). Default 6 = fwd/rev x3")
    ap.add_argument("--gap-ms", type=int, default=10,
                    help="gap between segments, avoiding duplicate timestamps at the turnaround")
    a = ap.parse_args()

    msgs = read_all(a.src_bag)
    if not msgs:
        sys.exit("the source bag has none of the required topics")
    t0, t1 = msgs[0][0], msgs[-1][0]
    span = t1 - t0
    first_lidar = next(r for _, tp, r in msgs if tp == LIDAR_TOPIC)
    print(f"source bag: {len(msgs)} messages, {span / 1e6:.1f}s")
    verify_layout(first_lidar)

    if os.path.exists(a.out_bag):
        shutil.rmtree(a.out_bag)
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=a.out_bag, storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    for i, (name, (tp, _)) in enumerate(TYPES.items()):
        w.create_topic(rosbag2_py.TopicMetadata(
            id=i, name=name, type=tp, serialization_format="cdr"))

    gap = a.gap_ms * 1000
    n_out = 0
    for k in range(a.segments):
        base = k * (span + gap)
        rev = (k % 2) == 1
        seg = []
        for us, topic, raw in msgs:
            local = (t1 - us) if rev else (us - t0)
            seg.append((base + local, topic, raw))
        seg.sort(key=lambda x: x[0])
        for us, topic, raw in seg:
            data = reverse_msg(topic, raw) if rev else raw
            w.write(topic, restamp(topic, data, us), int(us) * 1000)
            n_out += 1
        print(f"  segment {k}: {'reversed' if rev else 'forward'}  "
              f"[{base / 1e6:.1f}, {(base + span) / 1e6:.1f}]s")
    del w
    print(f"wrote {a.out_bag}: {n_out} messages, total duration "
          f"{(a.segments * (span + gap) - gap) / 1e6:.1f}s")


if __name__ == "__main__":
    main()
