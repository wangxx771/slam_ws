#!/usr/bin/env python3
"""Map-level dynamic-removal evaluation: accumulate the whole run into a static
map and compare it against the ground-truth map voxel by voxel.

This is the standard measure in the dynamic-removal literature (Removert,
ERASOR and FreeDOM all use it). Unlike the per-frame measure, a dynamic point
missed in any single frame stays in the map as a trail, so map-level RR is
usually lower than per-frame RR and reflects actual map quality better.

  PR = ground-truth static voxels hit    / all ground-truth static voxels
  RR = 1 - ground-truth dynamic voxels hit / all ground-truth dynamic voxels

Ground truth: FreeDOM/ouster.pcd (x y z label, 9=static 251=dynamic)
Input: a converted ROS 2 bag (/velodyne_points with label + /tf poses)

Usage: dyn_map_eval.py <bag> <gt.pcd> <method...>
"""
import argparse
import sys
import time

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from dyn_eval import (STATIC_LABEL, DYNAMIC_LABEL, RangeImage, cluster_grow,
                      read_bag, voxel_downsample)

VOX = 0.2          # matches the FreeDOM paper: 0.2 m outdoors
OFF = 1 << 21


def pack(xyz, res=VOX):
    k = np.floor(xyz / res).astype(np.int64) + OFF
    return (k[:, 0] << 44) | (k[:, 1] << 22) | k[:, 2]


def load_gt(path):
    a = np.loadtxt(path, skiprows=11, dtype=np.float64)
    keys = pack(a[:, :3])
    lab = a[:, 3].astype(np.int32)
    S = np.unique(keys[lab == STATIC_LABEL])
    D = np.unique(keys[lab == DYNAMIC_LABEL])
    # A voxel holding both kinds counts as dynamic: anything left in the map is a trail
    S = S[~np.isin(S, D)]
    return S, D


def run_delayed(frames, args):
    """Deferred decision: points enter a pending area, keep collecting votes from
    frames that arrive afterwards, and only join the map once they expire.

    An admission gate can only vote with frames from before insertion, but when a
    dynamic object first enters view no earlier line of sight has crossed it - it
    cannot be judged and stays in the map forever. That is the architectural gap
    against FreeDOM, which deletes already-inserted points retroactively.
    Deferring N frames is equivalent to also using the next N frames' lines of
    sight, at the cost of the map forming N frames late.
    """
    import math
    el_hi, el_lo = math.radians(args.vfov_upper), math.radians(args.vfov_lower)
    imgs, pts = [], []
    for t, xyz_b, lab, R, p in frames:
        ri = RangeImage(xyz_b, R, p, args.ri_rows, args.ri_cols, el_lo, el_hi)
        ri.set_window(args.vis_win)
        ri.require_allvalid = False
        imgs.append(ri)
        xyz_d, _ = voxel_downsample(xyz_b, lab, VOX)
        pts.append(xyz_d @ R.T + p)

    n = len(frames)
    acc = []
    for k in range(n):
        w = pts[k]
        votes = np.zeros(len(w), np.int32)
        lo, hi = max(0, k - args.hist_len), min(n, k + args.delay + 1)
        for j in range(lo, hi):
            if j == k:
                continue
            votes += imgs[j].sees_through(w, args.vis_margin, args.vis_margin_slope)
        acc.append(pack(w[votes < args.vis_votes]))
        if len(acc) > 40:
            acc = [np.unique(np.concatenate(acc))]
    return np.unique(np.concatenate(acc))


def run_unbounded(frames, args):
    """No window: every point is voted on by every frame that can see it, giving
    the ceiling of this criterion.

    "Can see" means the frame's sensor position is within the raycast range of
    the point. The trajectory is 500 m and the range 60 m, so each point is
    compared against tens to hundreds of frames, not O(N^2).
    This curve answers whether matching FreeDOM needs a wider window or whether
    the criterion itself has topped out.
    """
    import math
    el_hi, el_lo = math.radians(args.vfov_upper), math.radians(args.vfov_lower)
    imgs, pts, org = [], [], []
    for t, xyz_b, lab, R, p in frames:
        ri = RangeImage(xyz_b, R, p, args.ri_rows, args.ri_cols, el_lo, el_hi)
        ri.set_window(args.vis_win)
        ri.require_allvalid = False
        imgs.append(ri)
        xyz_d, _ = voxel_downsample(xyz_b, lab, VOX)
        pts.append(xyz_d @ R.T + p)
        org.append(p)
    org = np.array(org)

    # Occupancy-count protection: a voxel hit in more than occ_protect frames is
    # treated as static structure and never deleted. Static structure is in view
    # for tens to hundreds of frames while a passing dynamic object occupies only
    # a few, so the separation is wide.
    occ = {}
    if args.occ_protect > 0:
        for _, xyz_b, _, R, p in frames:
            w = xyz_b @ R.T + p
            # The offset is required: negative coordinates make k negative, and
            # left-shifting then OR-ing negatives collides keys, merging counts
            # from unrelated voxels. Much of ouster's Y is negative, which
            # over-protected until RR hit zero.
            k = np.floor(w / args.occ_res).astype(np.int64) + OFF
            ku = np.unique((k[:, 0] << 44) | (k[:, 1] << 22) | k[:, 2])  # count each voxel once per frame
            for kk in ku.tolist():
                occ[kk] = occ.get(kk, 0) + 1
        protected = np.array(sorted(k for k, v in occ.items() if v >= args.occ_protect),
                             dtype=np.int64)
        print(f"occupancy protection: {len(protected)} voxels hit in >= {args.occ_protect} frames")
    else:
        protected = np.zeros(0, dtype=np.int64)

    # Vote once and emit results for every threshold: the unbounded pass takes
    # 20 minutes, so re-running it per threshold is not practical.
    thr = args.thresholds
    accs = [[] for _ in thr]
    n = len(frames)
    for k in range(n):
        w = pts[k]
        votes = np.zeros(len(w), np.int32)
        for j in range(n):
            if j == k:
                continue
            # Skip the whole frame when the sensor is too far from these points
            if np.min(np.linalg.norm(w - org[j], axis=1)) > args.reach:
                continue
            votes += imgs[j].sees_through(w, args.vis_margin, args.vis_margin_slope)
        if len(protected):
            kk = np.floor(w / args.occ_res).astype(np.int64) + OFF
            kk = (kk[:, 0] << 44) | (kk[:, 1] << 22) | kk[:, 2]
            j = np.clip(np.searchsorted(protected, kk), 0, len(protected) - 1)
            is_prot = protected[j] == kk
        else:
            is_prot = np.zeros(len(w), bool)
        for t, a in zip(thr, accs):
            dyn = (votes >= t) & ~is_prot
            if args.cluster and dyn.any():
                # Cluster completion, as FreeDOM does: dynamic points come in
                # clusters, so per-point voting necessarily misses some
                # (occluded, or below the angular resolution). Enough seeds in a
                # cluster marks the whole cluster dynamic. Ground and buildings
                # merge into one huge component, blocked by a bounding-box size
                # test so nothing valid is lost.
                dyn = cluster_grow(w, dyn, VOX * 3, args.cluster_max_extent,
                                   args.cluster_seed_ratio, args.cluster_min_pts)
            a.append(pack(w[~dyn]))
        if len(accs[0]) > 40:
            accs = [[np.unique(np.concatenate(a))] for a in accs]
    return {t: np.unique(np.concatenate(a)) for t, a in zip(thr, accs)}


def run(frames, args, method):
    if method == "delayed":
        return run_delayed(frames, args)
    if method == "unbounded":
        return run_unbounded(frames, args)
    """Return the voxel-key set of accumulated kept points."""
    acc = []
    hist = []
    import math
    el_hi, el_lo = math.radians(args.vfov_upper), math.radians(args.vfov_lower)
    for t, xyz_b, lab, R, p in frames:
        if method == "visibility":
            ri = RangeImage(xyz_b, R, p, args.ri_rows, args.ri_cols, el_lo, el_hi)
            ri.set_window(args.vis_win)
            ri.require_allvalid = False
        xyz_d, lab_d = voxel_downsample(xyz_b, lab, VOX)
        w = xyz_d @ R.T + p
        if method == "none":
            dyn = np.zeros(len(w), bool)
        elif method == "visibility":
            votes = np.zeros(len(w), np.int32)
            for h in hist:
                votes += h.sees_through(w, args.vis_margin, args.vis_margin_slope)
            dyn = votes >= args.vis_votes
        elif method == "nn":
            from scipy.spatial import cKDTree
            dyn = np.zeros(len(w), bool)
            if len(hist) >= args.nn_hist:
                H = np.vstack(hist[-args.nn_hist:])
                d, _ = cKDTree(H).query(w, k=1, distance_upper_bound=args.search_r)
                rng = np.linalg.norm(xyz_d, axis=1)
                dyn = np.isfinite(d) & (d > args.disp_base + args.disp_slope * rng)
                if dyn.sum() > args.max_reject_ratio * len(w):
                    dyn[:] = False
        acc.append(pack(w[~dyn]))
        if method == "visibility":
            hist.append(ri)
            if len(hist) > args.hist_len:
                hist.pop(0)
        elif method == "nn":
            hist.append(w)
            if len(hist) > args.nn_hist:
                hist.pop(0)
        if len(acc) > 40:
            acc = [np.unique(np.concatenate(acc))]
    return np.unique(np.concatenate(acc))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("gt")
    ap.add_argument("methods", nargs="+")
    ap.add_argument("--frames", type=int, default=0)
    ap.add_argument("--ri-rows", type=int, default=64)
    ap.add_argument("--ri-cols", type=int, default=1796)
    ap.add_argument("--vfov-upper", type=float, default=11.25)
    ap.add_argument("--vfov-lower", type=float, default=-11.25)
    ap.add_argument("--hist-len", type=int, default=40)
    ap.add_argument("--vis-votes", type=int, default=5)
    ap.add_argument("--vis-win", type=int, default=1)
    ap.add_argument("--vis-margin", type=float, default=1.0)
    ap.add_argument("--vis-margin-slope", type=float, default=0.02)
    ap.add_argument("--delay", type=int, default=40)
    ap.add_argument("--reach", type=float, default=60.0)
    ap.add_argument("--thresholds", type=int, nargs="+", default=[9, 15, 25, 40, 60],
                    help="sweep several vote thresholds from a single voting pass in unbounded mode")
    ap.add_argument("--occ-protect", type=int, default=0)
    ap.add_argument("--occ-res", type=float, default=0.5)
    ap.add_argument("--cluster", action="store_true")
    ap.add_argument("--cluster-max-extent", type=float, default=8.0)
    ap.add_argument("--cluster-seed-ratio", type=float, default=0.2)
    ap.add_argument("--cluster-min-pts", type=int, default=5)
    ap.add_argument("--nn-hist", type=int, default=5)
    ap.add_argument("--search-r", type=float, default=2.0)
    ap.add_argument("--disp-base", type=float, default=0.3)
    ap.add_argument("--disp-slope", type=float, default=0.01)
    ap.add_argument("--max-reject-ratio", type=float, default=0.4)
    args = ap.parse_args()

    t0 = time.time()
    S, D = load_gt(args.gt)
    print(f"ground-truth map: {len(S)} static voxels, {len(D)} dynamic  ({time.time()-t0:.1f}s)")
    frames = read_bag(args.bag, args.frames)
    print(f"read {len(frames)} frames")

    print(f"\n{'method':>12} {'PR':>8} {'RR':>8} {'F1':>8} {'time':>8}")
    for m in args.methods:
        t0 = time.time()
        R = run(frames, args, m)
        dt = time.time() - t0
        if not isinstance(R, dict):
            R = {args.vis_votes: R}
        for t, K in R.items():
            pr = np.isin(S, K).sum() / len(S)
            rr = 1.0 - np.isin(D, K).sum() / len(D)
            f1 = 2 * pr * rr / max(pr + rr, 1e-9)
            print(f"{m + ' v' + str(t):>14} {pr*100:7.2f}% {rr*100:7.2f}% "
                  f"{f1*100:7.2f}% {dt:7.0f}s")


if __name__ == "__main__":
    main()
