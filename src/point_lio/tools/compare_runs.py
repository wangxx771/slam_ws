#!/usr/bin/env python3
"""Compare the odom trajectories of two runs.


Usage: compare_runs.py A/odom.csv B/odom.csv
"""

import sys

import numpy as np


def load(p):
    return np.loadtxt(p, delimiter=",", skiprows=1, ndmin=2)


def main(pa, pb):
    A, B = load(pa), load(pb)
    ta, tb = A[:, 0], B[:, 0]
    same_n = len(A) == len(B)

    # With imu_propagate_en=true the /odom timestamps repeat and are non-monotonic:
    # after each lidar frame the propagated state is re-anchored to lidar_end_time
    # and any IMU after that is propagated again under the same timestamp (7311
    # unique timestamps produced 15760 rows, repeating up to 3 times).
    # searchsorted cannot pair these - it only hits the first row of each repeat
    # group and reports two identical trajectories as a regression. When the row
    # counts match, compare row by row.
    if same_n and np.array_equal(ta, tb):
        d = np.linalg.norm(B[:, 1:4] - A[:, 1:4], axis=1)
        n_match = len(d)
    else:
        idx = np.clip(np.searchsorted(ta, tb), 0, len(ta) - 1)
        ok = np.abs(ta[idx] - tb) < 1e-9
        d = np.linalg.norm(B[ok, 1:4] - A[idx[ok], 1:4], axis=1)
        n_match = int(ok.sum())
    print(f"A: {pa}  n={len(A)}")
    print(f"B: {pb}  n={len(B)}")
    print(f"{n_match} timestamp-matched points   max position diff = {d.max():.6e} m   "
          f"mean = {d.mean():.6e} m")

    if not same_n:
        print("!! frame counts differ - the runs did not start on the same frame, so this "
              "comparison is not a valid regression test. Re-run until they match.")
        return 2
    if d.max() < 1e-9:
        print("OK: same frame count and bit-identical - zero regression")
        return 0
    print("FAIL: same frame count but the trajectories differ - a real regression")
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1], sys.argv[2]))
