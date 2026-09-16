

import struct
from dataclasses import dataclass

import numpy as np

MAGIC_CODE = 0xAC0EA767

PUBLIC_HEADER_SIZE = 24
PRIVATE_HEADER_SIZE = 5
DEVICE_INFO_SIZE = 63
FRAME_HEADER_SIZE = 24
PACKAGE_HEADER_SIZE = 27

DATA_TYPE_IMU = 0
DATA_TYPE_CARTESIAN_HIGH = 1
DATA_TYPE_CARTESIAN_LOW = 2
DATA_TYPE_SPHERICAL = 3

LINE_NUM = {
    9: 4,    # kLivoxLidarTypeMid360
    10: 6,   # kLivoxLidarTypeIndustrialHAP
    15: 6,   # kLivoxLidarTypeHAP
    35: 4,   # kLivoxLidarTypeMid360s
}
LINE_NUM_DEFAULT = 1

DT_HIGH = np.dtype(
    [("x", "<i4"), ("y", "<i4"), ("z", "<i4"), ("reflectivity", "u1"), ("tag", "u1")]
)  # 14 B, millimetres
DT_LOW = np.dtype(
    [("x", "<i2"), ("y", "<i2"), ("z", "<i2"), ("reflectivity", "u1"), ("tag", "u1")]
)  # 8 B, centimetres
DT_IMU = np.dtype(
    [
        ("gyro_x", "<f4"), ("gyro_y", "<f4"), ("gyro_z", "<f4"),
        ("acc_x", "<f4"), ("acc_y", "<f4"), ("acc_z", "<f4"),
    ]
)  # 24 B, gyro rad/s, acc g


@dataclass
class DeviceInfo:
    lidar_sn: str
    hub_sn: str
    lidar_id: int
    lidar_type: int
    device_type: int
    extrinsic_enable: int
    roll: float
    pitch: float
    yaw: float
    x: float
    y: float
    z: float

    @property
    def line_num(self):
        return LINE_NUM.get(self.device_type, LINE_NUM_DEFAULT)


@dataclass
class Package:
    lidar_id: int
    timestamp: int      # ns
    data_type: int
    udp_counter: int
    frame_counter: int
    payload: memoryview


class Lvx2Reader:
    def __init__(self, path):
        self.path = path
        self._f = open(path, "rb")
        self._read_headers()

    def close(self):
        self._f.close()

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()

    def _read_headers(self):
        f = self._f
        pub = f.read(PUBLIC_HEADER_SIZE)
        if len(pub) != PUBLIC_HEADER_SIZE:
            raise ValueError("file too short for public header")
        sig = pub[:16].split(b"\x00")[0].decode("ascii", "replace")
        va, vb, vc, vd = pub[16], pub[17], pub[18], pub[19]
        (magic,) = struct.unpack_from("<I", pub, 20)
        if magic != MAGIC_CODE:
            raise ValueError(f"bad magic 0x{magic:08X}, not an lvx2 file")
        self.signature = sig
        self.version = (va, vb, vc, vd)

        priv = f.read(PRIVATE_HEADER_SIZE)
        self.frame_duration_ms, self.device_count = struct.unpack("<IB", priv)

        self.devices = []
        for _ in range(self.device_count):
            raw = f.read(DEVICE_INFO_SIZE)
            lidar_sn = raw[0:16].split(b"\x00")[0].decode("ascii", "replace")
            hub_sn = raw[16:32].split(b"\x00")[0].decode("ascii", "replace")
            lidar_id, lidar_type, device_type, extrinsic_enable = struct.unpack_from(
                "<IBBB", raw, 32
            )
            roll, pitch, yaw, x, y, z = struct.unpack_from("<6f", raw, 39)
            self.devices.append(
                DeviceInfo(lidar_sn, hub_sn, lidar_id, lidar_type, device_type,
                           extrinsic_enable, roll, pitch, yaw, x, y, z)
            )
        self._data_start = f.tell()

    def frames(self):
        """Yields (frame_index, [Package, ...]) per frame."""
        f = self._f
        f.seek(self._data_start)
        while True:
            fh = f.read(FRAME_HEADER_SIZE)
            if len(fh) < FRAME_HEADER_SIZE:
                return
            cur_off, next_off, frame_index = struct.unpack("<QQQ", fh)
            if next_off <= cur_off:
                return
            body = f.read(next_off - cur_off - FRAME_HEADER_SIZE)
            if not body:
                return
            yield frame_index, list(_parse_packages(body))


def _parse_packages(body):
    mv = memoryview(body)
    off = 0
    n = len(mv)
    while off + PACKAGE_HEADER_SIZE <= n:
        (lidar_id,) = struct.unpack_from("<I", mv, off + 1)
        (timestamp,) = struct.unpack_from("<Q", mv, off + 7)
        (udp_counter,) = struct.unpack_from("<H", mv, off + 15)
        data_type = mv[off + 17]
        (length,) = struct.unpack_from("<I", mv, off + 18)
        frame_counter = mv[off + 22]
        start = off + PACKAGE_HEADER_SIZE
        end = start + length
        if end > n:
            return
        yield Package(lidar_id, timestamp, data_type, udp_counter, frame_counter,
                      mv[start:end])
        off = end


def points_from_package(pkg, line_num, point_interval_ns):
    """Expand one cloud package into (xyz in metres, reflectivity, tag, line, t_ns), following pub_handler.cpp."""
    if pkg.data_type == DATA_TYPE_CARTESIAN_HIGH:
        raw = np.frombuffer(pkg.payload, dtype=DT_HIGH)
        scale = 1.0 / 1000.0          # mm -> m
    elif pkg.data_type == DATA_TYPE_CARTESIAN_LOW:
        raw = np.frombuffer(pkg.payload, dtype=DT_LOW)
        scale = 1.0 / 100.0           # cm -> m
    else:
        raise ValueError(f"not a point package: data_type={pkg.data_type}")

    n = raw.shape[0]
    xyz = np.empty((n, 3), dtype=np.float32)
    xyz[:, 0] = raw["x"] * scale
    xyz[:, 1] = raw["y"] * scale
    xyz[:, 2] = raw["z"] * scale
    idx = np.arange(n, dtype=np.int64)
    line = (idx % line_num).astype(np.uint8)
    t_ns = pkg.timestamp + idx * point_interval_ns
    return xyz, raw["reflectivity"].copy(), raw["tag"].copy(), line, t_ns


def imu_from_package(pkg):
    return np.frombuffer(pkg.payload, dtype=DT_IMU)
