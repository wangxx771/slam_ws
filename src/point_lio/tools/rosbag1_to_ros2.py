#!/usr/bin/env python3
"""ROS 1 bag -> ROS 2 bag (mcap), for Livox datasets.

Topic mapping:
  livox_ros_driver/CustomMsg  -> livox_ros_driver2/msg/CustomMsg   (fields map 1:1)
  sensor_msgs/Imu             -> sensor_msgs/msg/Imu
Other topics (/image_raw and the like) are dropped by default: Point-LIO does not
use them and keeping them doubles the bag size.

The ROS 1 and ROS 2 CustomMsg fields are identical apart from std_msgs/Header,
where ROS 1 has an extra seq field, so a field-by-field copy suffices and no
values change.

Requires: pip install rosbags (used only for reading ROS 1; writing uses rosbag2_py)

Usage:
  rosbag1_to_ros2.py <in.bag> <out_dir> [--keep-image]
"""

import argparse
import os
import shutil
import sys
import time

import rclpy.serialization as ser
import rosbag2_py
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, PointCloud2, PointField
from geometry_msgs.msg import Transform, TransformStamped
from tf2_msgs.msg import TFMessage

try:
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
    HAVE_LIVOX = True
except ImportError:
    # Only Livox datasets need this. If it is missing, treat that topic as
    # unsupported and skip it rather than failing the whole script at import -
    # other bags (plain PointCloud2, say) do not need it.
    HAVE_LIVOX = False

LIVOX_IN = "livox_ros_driver/msg/CustomMsg"
LIVOX_OUT = "livox_ros_driver2/msg/CustomMsg"
IMU_IN = "sensor_msgs/msg/Imu"
IMU_OUT = "sensor_msgs/msg/Imu"
PC2 = "sensor_msgs/msg/PointCloud2"
GNSS = "sensor_msgs/msg/NavSatFix"
TF = "tf2_msgs/msg/TFMessage"


def conv_lidar(m):
    o = CustomMsg()
    o.header.frame_id = m.header.frame_id
    o.header.stamp.sec = int(m.header.stamp.sec)
    o.header.stamp.nanosec = int(m.header.stamp.nanosec)
    o.timebase = int(m.timebase)
    o.point_num = int(m.point_num)
    o.lidar_id = int(m.lidar_id)
    o.rsvd = [int(x) for x in m.rsvd] if len(m.rsvd) == 3 else [0, 0, 0]
    pts = []
    for p in m.points:
        q = CustomPoint()
        q.offset_time = int(p.offset_time)
        q.x = float(p.x)
        q.y = float(p.y)
        q.z = float(p.z)
        q.reflectivity = int(p.reflectivity)
        q.tag = int(p.tag)
        q.line = int(p.line)
        pts.append(q)
    o.points = pts
    return o


def conv_imu(m):
    o = Imu()
    o.header.frame_id = m.header.frame_id
    o.header.stamp.sec = int(m.header.stamp.sec)
    o.header.stamp.nanosec = int(m.header.stamp.nanosec)
    o.orientation.x = float(m.orientation.x)
    o.orientation.y = float(m.orientation.y)
    o.orientation.z = float(m.orientation.z)
    o.orientation.w = float(m.orientation.w)
    o.angular_velocity.x = float(m.angular_velocity.x)
    o.angular_velocity.y = float(m.angular_velocity.y)
    o.angular_velocity.z = float(m.angular_velocity.z)
    o.linear_acceleration.x = float(m.linear_acceleration.x)
    o.linear_acceleration.y = float(m.linear_acceleration.y)
    o.linear_acceleration.z = float(m.linear_acceleration.z)
    o.orientation_covariance = [float(v) for v in m.orientation_covariance]
    o.angular_velocity_covariance = [float(v) for v in m.angular_velocity_covariance]
    o.linear_acceleration_covariance = [float(v) for v in m.linear_acceleration_covariance]
    return o


def conv_pc2(m):
    o = PointCloud2()
    o.header.frame_id = m.header.frame_id
    o.header.stamp.sec = int(m.header.stamp.sec)
    o.header.stamp.nanosec = int(m.header.stamp.nanosec)
    o.height = int(m.height)
    o.width = int(m.width)
    for f in m.fields:
        pf = PointField()
        pf.name = f.name
        pf.offset = int(f.offset)
        pf.datatype = int(f.datatype)
        pf.count = int(f.count)
        o.fields.append(pf)
    o.is_bigendian = bool(m.is_bigendian)
    o.point_step = int(m.point_step)
    o.row_step = int(m.row_step)
    o.data = m.data.tobytes() if hasattr(m.data, "tobytes") else bytes(m.data)
    o.is_dense = bool(m.is_dense)
    return o


def conv_gnss(m):
    o = NavSatFix()
    o.header.frame_id = m.header.frame_id
    o.header.stamp.sec = int(m.header.stamp.sec)
    o.header.stamp.nanosec = int(m.header.stamp.nanosec)
    o.status.status = int(m.status.status)
    o.status.service = int(m.status.service)
    o.latitude = float(m.latitude)
    o.longitude = float(m.longitude)
    o.altitude = float(m.altitude)
    o.position_covariance = [float(v) for v in m.position_covariance]
    o.position_covariance_type = int(m.position_covariance_type)
    return o


def conv_tf(m):
    """Datasets like HeLiMOS carry poses on /tf; without converting it there are no poses."""
    o = TFMessage()
    for t in m.transforms:
        s = TransformStamped()
        s.header.frame_id = t.header.frame_id
        s.header.stamp.sec = int(t.header.stamp.sec)
        s.header.stamp.nanosec = int(t.header.stamp.nanosec)
        s.child_frame_id = t.child_frame_id
        s.transform.translation.x = float(t.transform.translation.x)
        s.transform.translation.y = float(t.transform.translation.y)
        s.transform.translation.z = float(t.transform.translation.z)
        s.transform.rotation.x = float(t.transform.rotation.x)
        s.transform.rotation.y = float(t.transform.rotation.y)
        s.transform.rotation.z = float(t.transform.rotation.z)
        s.transform.rotation.w = float(t.transform.rotation.w)
        o.transforms.append(s)
    return o


def verify_pc2(src, dst):
    if (dst.width, dst.point_step, len(dst.data)) != (
            int(src.width), int(src.point_step), len(src.data)):
        sys.exit("PointCloud2 field check failed")
    print(f"  first cloud frame checks out ({dst.width} points, fields "
          f"{[f.name for f in dst.fields]}, frame_id={dst.header.frame_id!r})")


def verify_lidar(src, dst):
    """Check the first converted frame and exit on mismatch, rather than discovering a wrong field after converting the whole bag."""
    bad = []
    if dst.header.frame_id != src.header.frame_id:
        bad.append("frame_id")
    if (dst.header.stamp.sec, dst.header.stamp.nanosec) != (
            int(src.header.stamp.sec), int(src.header.stamp.nanosec)):
        bad.append("stamp")
    if dst.timebase != int(src.timebase) or dst.point_num != int(src.point_num):
        bad.append("timebase/point_num")
    if len(dst.points) != len(src.points):
        bad.append("point count")
    else:
        for i in (0, len(src.points) // 2, len(src.points) - 1):
            a, b = src.points[i], dst.points[i]
            if (int(a.offset_time) != b.offset_time or float(a.x) != b.x or
                    float(a.y) != b.y or float(a.z) != b.z or
                    int(a.reflectivity) != b.reflectivity or int(a.tag) != b.tag or
                    int(a.line) != b.line):
                bad.append(f"point[{i}]")
    if bad:
        sys.exit(f"field check failed: {', '.join(bad)}")
    print(f"  first frame fields check out ({dst.point_num} points, frame_id={dst.header.frame_id!r})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("in_bag")
    ap.add_argument("out_dir")
    ap.add_argument("--keep-image", action="store_true", help="also convert sensor_msgs/Image")
    ap.add_argument("--topics", default="",
                    help="comma-separated allowlist; empty converts every supported topic. "
                         "Use it to pick one when duplicates like /imu_raw and /imu_correct both exist")
    ap.add_argument("--storage", default="",
                    help="rosbag2 storage backend; empty selects automatically (mcap if available, else sqlite3)")
    a = ap.parse_args()

    if os.path.exists(a.out_dir):
        shutil.rmtree(a.out_dir)

    ts = get_typestore(Stores.ROS1_NOETIC)
    with Reader(a.in_bag) as r:
        extra = {}
        for c in r.connections:
            if c.msgtype not in ts.types:
                extra.update(get_types_from_msg(c.msgdef[1], c.msgtype))
        if extra:
            ts.register(extra)

        # topic -> (ROS 2 type name, conversion function)
        whitelist = {t for t in a.topics.split(",") if t}
        handlers = {
            IMU_IN: (IMU_OUT, conv_imu),
            PC2: (PC2, conv_pc2),
            GNSS: (GNSS, conv_gnss),
            TF: (TF, conv_tf),
        }
        if HAVE_LIVOX:
            handlers[LIVOX_IN] = (LIVOX_OUT, conv_lidar)
        plan = {}
        for c in r.connections:
            if whitelist and c.topic not in whitelist:
                continue
            h = handlers.get(c.msgtype)
            if h:
                plan[c.topic] = h
        if not plan:
            sys.exit("the bag has no convertible topics (cloud / IMU / NavSatFix / TF)")
        missing = whitelist - set(plan)
        if missing:
            sys.exit(f"these --topics do not exist or have unsupported types: {sorted(missing)}")

        print(f"source bag: {(r.end_time - r.start_time) / 1e9:.1f}s, {r.message_count} messages")
        for c in r.connections:
            mark = "->" if c.topic in plan else "skip"
            print(f"  {mark} {c.topic:20s} {c.msgtype:35s} {c.msgcount}")

        # The mcap plugin is not installed by default on ros-humble (only
        # rosbag2_storage_default_plugins/sqlite3). Hard-coding mcap fails at the
        # very last step on machines without it, so pick from what is available.
        storage = a.storage
        if not storage:
            avail = rosbag2_py.get_registered_writers()
            storage = "mcap" if "mcap" in avail else "sqlite3"
            print(f"storage backend: {storage} (available: {sorted(avail)})")

        w = rosbag2_py.SequentialWriter()
        w.open(rosbag2_py.StorageOptions(uri=a.out_dir, storage_id=storage),
               rosbag2_py.ConverterOptions("", ""))
        # The TopicMetadata signature changed across versions: humble takes
        # (name, type, serialization_format, qos) and jazzy added a leading id.
        # Try keywords once and fall back to the old signature.
        for i, (topic, (tp, _)) in enumerate(plan.items()):
            try:
                md = rosbag2_py.TopicMetadata(
                    id=i, name=topic, type=tp, serialization_format="cdr")
            except TypeError:
                md = rosbag2_py.TopicMetadata(
                    name=topic, type=tp, serialization_format="cdr")
            w.create_topic(md)

        conns = [c for c in r.connections if c.topic in plan]
        n = 0
        checked = set()
        t0 = time.time()
        for c, t_ns, raw in r.messages(connections=conns):
            src = ts.deserialize_ros1(raw, c.msgtype)
            dst = plan[c.topic][1](src)
            if c.msgtype not in checked:
                checked.add(c.msgtype)
                if c.msgtype == LIVOX_IN:
                    verify_lidar(src, dst)
                elif c.msgtype == PC2:
                    verify_pc2(src, dst)
            w.write(c.topic, ser.serialize_message(dst), t_ns)
            n += 1
            if n % 5000 == 0:
                print(f"  ... {n} messages ({time.time() - t0:.0f}s)")
        del w
    print(f"wrote {a.out_dir}: {n} messages in {time.time() - t0:.0f}s")


if __name__ == "__main__":
    main()
