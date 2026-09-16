#!/usr/bin/env python3
"""Evaluate global relocalization accuracy.

point_lio already looks up map->odom and left-multiplies it before publishing
/lio/odom (the lookupTransform("map","odom") in laserMapping.cpp), so once locked
the odom columns are already in the map frame - do not multiply again (doing so
produces bogus 90-280 m errors). The m2o column only says whether that row is
locked.

The reference trajectory is the odom.csv from the mapping run, but it must first
be converted into the frame the map lives in: the CSV x/y/z columns are the
output stream after the loop correction as it stood at the time, while the map
was saved at the end under the final correction. The difference between "then"
and "final" was 0.73 m on the stairwell bag - without converting, a 0.198 m
relocalization error is reported as 0.767 m with the error concentrated on a
single horizontal axis, which looks exactly like along-track degeneracy.

The conversion uses the last three CSV columns rx/ry/rz (raw uncorrected front-
end positions): the tail satisfies out = pg_final * raw, which solves for
pg_final and moves the whole raw track over. When no loop fires, out equals raw
and this is a no-op - which is why flat scenes never exposed the problem.

Usage: eval_reloc.py <reference_odom.csv> <reloc_dir> [<reloc_dir> ...]
"""
import os
import sys

import numpy as np


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_of(q):
    x, y, z, w = q
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def to_map_frame(path, ref):
    """Convert the reference trajectory from the odom output stream into the frame
    the map lives in.

    The two differ by a rigid transform: each output row used the loop correction
    as it stood at the time, while the map was saved at the end under the final
    correction. On the stairwell bag that was 1.3 m; without converting, the
    relocalization error is reported several times too large and concentrated on
    a single horizontal axis, looking just like along-track degeneracy.

    point_lio writes the transform directly to <odom_csv>.pgcorrect - do not solve
    for it. Procrustes on the tail was tried: fitting a 6-DoF rigid transform on a
    small stretch and extrapolating over hundreds of metres amplifies error, and
    even a flat bag with loop closure off solved to a spurious 0.026 m transform
    that turned a real 0.016 m error into 0.268 m.

    Returns the input unchanged with a note when the file is missing (old data).
    """
    f = path + ".pgcorrect"
    if not os.path.exists(f):
        print("note: no .pgcorrect, using the reference trajectory as is; "
              "if a loop fired during mapping the error here will read high")
        return ref
    v = np.loadtxt(f).ravel()
    if v.size != 12:
        return ref
    R, t = v[:9].reshape(3, 3), v[9:]
    if np.allclose(R, np.eye(3)) and np.allclose(t, 0):
        return ref
    full = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    if full.shape[1] < 14:
        return ref
    ref = ref.copy()
    ref[:, 1:4] = (R @ full[:, 11:14].T).T + t
    print(f"reference trajectory converted to the map frame (final loop correction, translation {np.linalg.norm(t):.3f} m)")
    return ref


def main():
    window = float(os.environ.get("RELOC_WINDOW", "5.0"))
    ref = np.loadtxt(sys.argv[1], delimiter=",", skiprows=1, usecols=range(8), ndmin=2)
    ref = to_map_frame(sys.argv[1], ref)
    print(f"reference trajectory {len(ref)} rows  t=[{ref[0,0]:.1f}, {ref[-1,0]:.1f}]")
    print(f"error at lock, then over {window:.0f} s "
          f"(RELOC_WINDOW=0 for the whole run)")
    print("the reference is another run of the same bag, not ground truth: both")
    print("odometries drift on their own, so only the value at lock is what")
    print("relocalization actually controls")
    print(f"{'case':>10} {'lock (s)':>8} {'position error med/p95/max (m)':>32} {'yaw err (deg)':>14}")
    errs = []
    for d in sys.argv[2:]:
        f = os.path.join(d, "odom.csv")
        rows = [l.split(",") for l in open(f).read().splitlines()[1:]]
        # Locked rows only, and only those point_lio actually stamped `map`.
        # It labels a row `odom` when the map->odom lookup has never resolved,
        # which is authoritative - the previous filter guessed from the error
        # magnitude and let a 1.9 m artifact through as if it were real.
        rows = [r for r in rows if len(r) == 16 and r[9] != "" and r[8] == "map"]
        tag = os.path.basename(d)
        if len(rows) < 20:
            print(f"{tag:>10} {'no lock':>8}")
            errs.append(None)
            continue
        a = np.array([[float(v) for v in r[:8] + r[9:]] for r in rows])
        # Allow 2 s after lock: point_lio's tf buffer cannot resolve the transform
        # the instant it is published, and during those tens of milliseconds
        # /lio/odom is still in the local frame, contributing a bogus peak of tens
        # of metres.
        a = a[a[:, 0] - a[0, 0] > 2.0]
        # Then stop after `window`. What this tool measures is how accurately the
        # node fixes the pose against the prior map, and only the moments right
        # after the lock carry that. Run longer and the number is dominated by
        # something else entirely: the reference is a different run of the same
        # route, so the two odometries drift apart on their own. Measured over a
        # whole 60 s replay that showed as 13.9 m of "relocalization error" while
        # the pose at lock was 0.05 m from the reference.
        if window > 0 and len(a):
            a = a[a[:, 0] - a[0, 0] <= 2.0 + window]
        t = a[:, 0]
        pm = a[:, 1:4]
        yw = yaw_of(a[:, 4:8].T)
        gt = np.stack([np.interp(t, ref[:, 0], ref[:, k]) for k in (1, 2, 3)], 1)
        gy = np.interp(t, ref[:, 0], np.unwrap(yaw_of(ref[:, 4:8].T)))
        e = np.linalg.norm(pm - gt, axis=1)
        ey = np.degrees(np.abs((yw - gy + np.pi) % (2 * np.pi) - np.pi))

        # First sample after the lock. Everything later is dominated by the two
        # runs' odometry drifting apart, which relocalization does not control.
        e_lock = float(e[0]) if len(e) else float("nan")
        lock = open(os.path.join(d, "rec.log")).read()
        ls = [l for l in lock.splitlines() if "first received" in l]
        lt = ls[0].split("@")[1].split("s")[0].strip() if ls else "?"
        print(f"{tag:>10} {lt+'s':>8}  at-lock {e_lock:6.3f} m | "
              f"{np.median(e):8.3f} {np.percentile(e,95):8.3f} {e.max():8.3f}   "
              f"{np.median(ey):10.2f}")
        errs.append(np.median(e))
    ok = [e for e in errs if e is not None]
    print(f"\nsucceeded {len(ok)}/{len(errs)}   median error {np.median(ok):.3f} m   "
          f"worst {max(ok):.3f} m" if ok else "\nall failed")


if __name__ == "__main__":
    main()
