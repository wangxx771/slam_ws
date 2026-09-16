#!/usr/bin/env python3
"""Quantitative evaluation of dynamic-point rejection.

Data: HeLiMOS ouster (/velodyne_points carries a label field, 9=static
251=dynamic; poses come from /tf). Metrics follow the dynamic-removal
literature:
  PR (Preservation Rate) = static points kept     / all static points
  RR (Rejection Rate)    = dynamic points removed / all dynamic points
  F1 = 2*PR*RR/(PR+RR)
RR alone is maxed by rejecting everything and PR alone by keeping everything, so
both must be read together.

Methods under test:
  none      no rejection (gives the trivial PR/RR bounds)
  nn        point_lio's current rule: dynamic if the nearest-neighbour distance
            to the last N frames exceeds a threshold
  freespace FreeDOM's approach: dynamic only inside voxels already traversed by
            a ray and observed empty

Usage: dyn_eval.py <bag> <method> [--frames N] [--voxel 0.5]
"""
import argparse
import math
import time

import numpy as np
import rclpy.serialization as ser
import rosbag2_py
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

STATIC_LABEL = 9
DYNAMIC_LABEL = 251


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def read_bag(bag, max_frames):
    """Returns [(t, xyz_body, label, R, p)] with poses matched by nearest timestamp."""
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    scans, poses = [], []
    while r.has_next():
        tp, data, _ = r.read_next()
        if tp == "/tf":
            m = ser.deserialize_message(data, TFMessage)
            for tr in m.transforms:
                t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                q, v = tr.transform.rotation, tr.transform.translation
                poses.append((t, quat_to_R(q.x, q.y, q.z, q.w),
                              np.array([v.x, v.y, v.z])))
        elif tp == "/velodyne_points":
            m = ser.deserialize_message(data, PointCloud2)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            a = np.frombuffer(m.data, dtype=np.uint8).reshape(-1, m.point_step)
            xyz = a[:, 0:12].copy().view(np.float32).reshape(-1, 3).astype(np.float64)
            lab = a[:, 20:22].copy().view(np.uint16).ravel()
            scans.append((t, xyz, lab))
            if max_frames and len(scans) >= max_frames:
                break
    pt = np.array([p[0] for p in poses])
    out = []
    for t, xyz, lab in scans:
        i = int(np.argmin(np.abs(pt - t)))
        out.append((t, xyz, lab, poses[i][1], poses[i][2]))
    return out


def voxel_downsample(xyz, lab, res):
    """Voxel downsample keeping the first point per voxel, matching point_lio's space_down_sample."""
    key = np.floor(xyz / res).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    return xyz[idx], lab[idx]


def method_nn(frames, args):
    """point_lio's current rule."""
    from scipy.spatial import cKDTree
    hist = []
    keep_s = keep_d = tot_s = tot_d = 0
    for t, xyz_b, lab, R, p in frames:
        xyz_b, lab = voxel_downsample(xyz_b, lab, args.voxel)
        rng = np.linalg.norm(xyz_b, axis=1)
        w = xyz_b @ R.T + p
        dyn = np.zeros(len(w), bool)
        if len(hist) >= args.hist_len:
            H = np.vstack(hist[-args.hist_len:])
            d, _ = cKDTree(H).query(w, k=1, distance_upper_bound=args.search_r)
            thr = args.disp_base + args.disp_slope * rng
            # Points beyond search_r get d=inf; the current implementation passes them as "no historical neighbour"
            dyn = np.isfinite(d) & (d > thr)
        if args.max_range > 0:
            dyn &= rng <= args.max_range
        if dyn.sum() > args.max_reject_ratio * len(w):
            dyn[:] = False           # current implementation's fuse: reject nothing if too much is flagged
        tot_s += (lab == STATIC_LABEL).sum(); tot_d += (lab == DYNAMIC_LABEL).sum()
        keep_s += ((lab == STATIC_LABEL) & ~dyn).sum()
        keep_d += ((lab == DYNAMIC_LABEL) & ~dyn).sum()
        hist.append(w)
        if len(hist) > args.hist_len:
            hist.pop(0)
    return keep_s, tot_s, keep_d, tot_d


class FreeSpace:
    """FreeDOM's conservative free-space estimate, simplified enough to test the
    rule itself.

    Voxel state comes from two counters:
      free_cnt  times a ray passed through
      occ_cnt   times an endpoint landed inside (observed occupied)
    FREE requires free_cnt >= counts_to_free and occ_cnt < counts_to_revert. The
    second condition is essential: grazing rays pass through ground voxels, and
    without the occupancy revert the ground is permanently carved into free
    space (PR drops to 42%).

    The key difference from the nearest-neighbour rule: space no ray has crossed
    is UNKNOWN rather than FREE, so newly observed static structure is never
    misjudged.

    Coordinates pack into a single int64 key so counting can be batched with
    np.unique; the Python loop runs over distinct voxels touched this frame
    rather than every ray sample.
    """

    OFF = 1 << 20    # packing offset, holds +-1M cells

    def __init__(self, res, counts_to_free, counts_to_revert, max_range, margin, erode):
        self.res = res
        self.k_free = counts_to_free
        self.k_revert = counts_to_revert
        self.max_range = max_range
        self.margin = margin
        self.erode = erode
        self.free_cnt = {}
        self.occ_cnt = {}
        self._occ_sorted_keys = np.zeros(0, dtype=np.int64)
        self._occ_cache_n = -1

    def _pack_idx(self, k):
        k = k + self.OFF
        return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]

    def _pack(self, w):
        return self._pack_idx(np.floor(w / self.res).astype(np.int64))

    def _occ_sorted(self):
        """Sorted key array of occupied voxels, for batch neighbourhood erosion via np.searchsorted."""
        if self._occ_cache_n != len(self.occ_cnt):
            self._occ_sorted_keys = np.array(
                sorted(k for k, v in self.occ_cnt.items() if v >= self.k_revert),
                dtype=np.int64)
            self._occ_cache_n = len(self.occ_cnt)
        return self._occ_sorted_keys

    def query_free(self, w):
        idx = np.floor(w / self.res).astype(np.int64)
        uidx, inv = np.unique(idx, axis=0, return_inverse=True)
        keys = self._pack_idx(uidx)
        fr = np.array([self.free_cnt.get(int(k), 0) >= self.k_free for k in keys])

        if self.erode and fr.any():
            # Conservative: any occupied voxel in the neighbourhood disqualifies
            # this one. Grazing rays cross ground voxels, and checking only the
            # voxel itself does not catch that.
            occ = self._occ_sorted()
            if len(occ):
                bad = np.zeros(len(uidx), bool)
                rr = range(-1, 2)
                for dx in rr:
                    for dy in rr:
                        for dz in rr:
                            nk = self._pack_idx(uidx + np.array([dx, dy, dz]))
                            j = np.searchsorted(occ, nk)
                            j = np.clip(j, 0, len(occ) - 1)
                            bad |= occ[j] == nk
                fr &= ~bad
        return fr[inv]

    def _bump(self, d, keys):
        uk, cnt = np.unique(keys, return_counts=True)
        for k, c in zip(uk.tolist(), cnt.tolist()):
            d[k] = d.get(k, 0) + int(c)

    def integrate(self, origin, w):
        # endpoint = occupancy observation
        self._bump(self.occ_cnt, self._pack(w))

        d = w - origin
        rng = np.linalg.norm(d, axis=1)
        m = (rng > 1e-3) & (rng < self.max_range)
        d, rng = d[m], rng[m]
        if len(d) == 0:
            return
        u = d / rng[:, None]
        step = self.res * 0.5
        # Leave a margin at the end unmarked: the ray terminates on a surface, so the last stretch hugs the object
        stop = rng - self.margin
        nmax = int(np.ceil(stop.max() / step)) if len(stop) else 0
        chunks = []
        for s in range(1, nmax):
            t = s * step
            act = t < stop
            if not act.any():
                break
            chunks.append(origin + u[act] * t)
        if not chunks:
            return
        self._bump(self.free_cnt, self._pack(np.vstack(chunks)))


class RangeImage:
    """Range image of one scan (spherical projection). Stores nearest distance; missed pixels are inf."""

    def __init__(self, xyz_body, R, p, rows, cols, el_lo, el_hi):
        self.R, self.p = R, p
        self.rows, self.cols = rows, cols
        self.el_lo, self.el_hi = el_lo, el_hi
        r = np.linalg.norm(xyz_body, axis=1)
        m = r > 1e-3
        xyz_body, r = xyz_body[m], r[m]
        u, v = self._project(xyz_body, r)
        ok = (v >= 0) & (v < rows)
        img = np.full(rows * cols, np.inf)
        np.minimum.at(img, v[ok] * cols + u[ok], r[ok])
        self.img = img.reshape(rows, cols)
        self.win = 0
        self.require_allvalid = True
        self.pool = self.img          # window minimum
        self.allvalid = np.isfinite(self.img)

    def set_window(self, k):
        """Precompute the (2k+1)^2 window minimum distance and a "every pixel in
        the window has a measurement" mask.

        Seeing through must use the window minimum, not a single pixel: a point
        on a target edge projects to a pixel whose reference-frame measurement is
        usually the background (the beam grazed past the object), so per-pixel
        comparison misclassifies many static edge points as dynamic. Require the
        whole window to see further.
        """
        if k == self.win:
            return
        self.win = k
        if k <= 0:
            self.pool, self.allvalid = self.img, np.isfinite(self.img)
            return
        pool = self.img.copy()
        valid = np.isfinite(self.img)
        for dv in range(-k, k + 1):
            for du in range(-k, k + 1):
                if dv == 0 and du == 0:
                    continue
                s = np.roll(self.img, (dv, du), axis=(0, 1))   # azimuth wraps around
                pool = np.minimum(pool, s)
                valid &= np.isfinite(s)
        self.pool, self.allvalid = pool, valid

    def _project(self, xyz, r):
        az = np.arctan2(xyz[:, 1], xyz[:, 0])
        el = np.arcsin(np.clip(xyz[:, 2] / r, -1, 1))
        u = ((az + math.pi) / (2 * math.pi) * self.cols).astype(np.int32) % self.cols
        v = ((self.el_hi - el) / (self.el_hi - self.el_lo) * self.rows).astype(np.int32)
        return u, v

    def sees_through(self, w, margin_base, margin_slope):
        """Is w (world frame) free space from this frame's view, i.e. did this frame see further along the same direction?"""
        b = (w - self.p) @ self.R          # R^T (w-p)
        r = np.linalg.norm(b, axis=1)
        u, v = self._project(b, np.maximum(r, 1e-6))
        ok = (v >= 0) & (v < self.rows) & (r > 1e-3)
        out = np.zeros(len(w), bool)
        if not ok.any():
            return out
        meas = self.pool[v[ok], u[ok]]
        good = self.allvalid[v[ok], u[ok]]
        margin = margin_base + margin_slope * r[ok]
        # meas is this frame's nearest measurement along that direction (window
        # included). A query point closer than that by more than the margin means
        # this frame's beam passed through it, so that spot was empty.
        if self.require_allvalid:
            out[ok] = good & np.isfinite(meas) & (r[ok] < meas - margin)
        else:
            out[ok] = np.isfinite(meas) & (r[ok] < meas - margin)
        return out


def cluster_grow(xyz_b, seed, res, max_extent, seed_ratio, min_pts):
    """Grow per-point dynamic votes into whole clusters.

    Dynamic points come in clusters, so a per-point test necessarily misses some
    (occluded, or below the angular resolution). Run voxel connected components
    on the current frame and mark a whole cluster dynamic once enough of it is
    seeded. Ground and buildings merge into one huge component, blocked by a
    bounding-box size test - that is what keeps this from over-rejecting.
    """
    idx = np.floor(xyz_b / res).astype(np.int64)
    key = {}
    for i, k in enumerate(map(tuple, idx)):
        key.setdefault(k, []).append(i)

    parent = {k: k for k in key}

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    nb = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
          if (dx, dy, dz) != (0, 0, 0)]
    for k in key:
        rk = find(k)
        for d in nb:
            n = (k[0] + d[0], k[1] + d[1], k[2] + d[2])
            if n in parent:
                rn = find(n)
                if rn != rk:
                    parent[rn] = rk
                    rk = find(k)

    groups = {}
    for k, pts in key.items():
        groups.setdefault(find(k), []).extend(pts)

    out = seed.copy()
    for pts in groups.values():
        if len(pts) < min_pts:
            continue
        p = xyz_b[pts]
        if (p.max(0) - p.min(0)).max() > max_extent:
            continue                      # too large - ground or building, leave alone
        s = seed[pts]
        if s.mean() >= seed_ratio:
            out[pts] = True
    return out


def method_visibility(frames, args):
    """Visibility test: project back into the range images of the last N frames
    and mark dynamic once seen through often enough.

    Compared with voxel ray casting this uses the sensor's own angular
    resolution, has no discretization error along the ray, and naturally blocks
    grazing ground by the measured distance in that direction. Memory is just N
    range images.
    """
    hist = []
    keep_s = keep_d = tot_s = tot_d = 0
    el_hi = math.radians(args.vfov_upper)
    el_lo = math.radians(args.vfov_lower)
    for t, xyz_b, lab, R, p in frames:
        ri = RangeImage(xyz_b, R, p, args.ri_rows, args.ri_cols, el_lo, el_hi)
        ri.set_window(args.vis_win)
        ri.require_allvalid = not args.vis_any_valid
        xyz_d, lab_d = voxel_downsample(xyz_b, lab, args.voxel)
        w = xyz_d @ R.T + p
        votes = np.zeros(len(w), np.int32)
        for h in hist:
            votes += h.sees_through(w, args.vis_margin, args.vis_margin_slope)
        dyn = votes >= args.vis_votes
        if args.cluster:
            dyn = cluster_grow(xyz_d, dyn, args.voxel * 1.5, args.cluster_max_extent,
                               args.cluster_seed_ratio, args.cluster_min_pts)
        if args.vis_max_range > 0:
            # Angular resolution is too coarse far out for the test to be reliable; capping the range is the cheapest way to raise PR
            dyn &= np.linalg.norm(xyz_d, axis=1) <= args.vis_max_range
        tot_s += (lab_d == STATIC_LABEL).sum(); tot_d += (lab_d == DYNAMIC_LABEL).sum()
        keep_s += ((lab_d == STATIC_LABEL) & ~dyn).sum()
        keep_d += ((lab_d == DYNAMIC_LABEL) & ~dyn).sum()
        hist.append(ri)
        if len(hist) > args.hist_len:
            hist.pop(0)
    return keep_s, tot_s, keep_d, tot_d


def method_freespace(frames, args):
    fs = FreeSpace(args.fs_res, args.counts_to_free, args.counts_to_revert,
                   args.raycast_max_range, args.ray_margin, not args.no_erode)
    keep_s = keep_d = tot_s = tot_d = 0
    for t, xyz_b, lab, R, p in frames:
        xyz_b, lab = voxel_downsample(xyz_b, lab, args.voxel)
        w = xyz_b @ R.T + p
        dyn = fs.query_free(w)
        tot_s += (lab == STATIC_LABEL).sum(); tot_d += (lab == DYNAMIC_LABEL).sum()
        keep_s += ((lab == STATIC_LABEL) & ~dyn).sum()
        keep_d += ((lab == DYNAMIC_LABEL) & ~dyn).sum()
        fs.integrate(p, w[~dyn])
    return keep_s, tot_s, keep_d, tot_d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("method", choices=["none", "nn", "freespace", "visibility"])
    ap.add_argument("--frames", type=int, default=0)
    ap.add_argument("--voxel", type=float, default=0.5)
    # defaults of point_lio's current rule (see parameters.cpp)
    ap.add_argument("--hist-len", type=int, default=5)
    ap.add_argument("--search-r", type=float, default=2.0)
    ap.add_argument("--disp-base", type=float, default=0.3)
    ap.add_argument("--disp-slope", type=float, default=0.01)
    ap.add_argument("--max-range", type=float, default=0.0)
    ap.add_argument("--max-reject-ratio", type=float, default=0.3)
    # free-space rule
    ap.add_argument("--fs-res", type=float, default=0.4)
    ap.add_argument("--counts-to-free", type=int, default=6)
    ap.add_argument("--counts-to-revert", type=int, default=20)
    ap.add_argument("--ray-margin", type=float, default=0.6)
    ap.add_argument("--no-erode", action="store_true")
    # visibility rule
    ap.add_argument("--ri-rows", type=int, default=64)
    ap.add_argument("--ri-cols", type=int, default=1024)
    ap.add_argument("--vfov-upper", type=float, default=11.25)
    ap.add_argument("--vfov-lower", type=float, default=-11.25)
    ap.add_argument("--vis-margin", type=float, default=0.5)
    ap.add_argument("--vis-margin-slope", type=float, default=0.02)
    ap.add_argument("--vis-votes", type=int, default=2)
    ap.add_argument("--vis-win", type=int, default=1)
    ap.add_argument("--vis-any-valid", action="store_true")
    ap.add_argument("--vis-max-range", type=float, default=0.0)
    ap.add_argument("--cluster", action="store_true")
    ap.add_argument("--cluster-max-extent", type=float, default=8.0)
    ap.add_argument("--cluster-seed-ratio", type=float, default=0.3)
    ap.add_argument("--cluster-min-pts", type=int, default=5)
    ap.add_argument("--raycast-max-range", type=float, default=60.0)
    args = ap.parse_args()

    t0 = time.time()
    frames = read_bag(args.bag, args.frames)
    print(f"read {len(frames)} frames ({time.time()-t0:.1f}s)")

    t0 = time.time()
    if args.method == "none":
        ks = ts = kd = td = 0
        for _, xyz, lab, _, _ in frames:
            xyz, lab = voxel_downsample(xyz, lab, args.voxel)
            ts += (lab == STATIC_LABEL).sum(); td += (lab == DYNAMIC_LABEL).sum()
        ks, kd = ts, td
    elif args.method == "nn":
        ks, ts, kd, td = method_nn(frames, args)
    elif args.method == "visibility":
        ks, ts, kd, td = method_visibility(frames, args)
    else:
        ks, ts, kd, td = method_freespace(frames, args)
    dt = time.time() - t0

    pr = ks / max(ts, 1)
    rr = 1.0 - kd / max(td, 1)
    f1 = 2 * pr * rr / max(pr + rr, 1e-9)
    print(f"method={args.method}  voxel={args.voxel}m  elapsed {dt:.1f}s ({dt/len(frames)*1000:.0f} ms/frame)")
    print(f"  static {ts} points, kept {ks}   ->  PR = {pr*100:.2f}%")
    print(f"  dynamic {td} points, removed {td-kd} ->  RR = {rr*100:.2f}%")
    print(f"  F1 = {f1*100:.2f}%")


if __name__ == "__main__":
    main()
