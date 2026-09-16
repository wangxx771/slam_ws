#!/usr/bin/env python3
"""Measure scan-context matching tolerance: how descriptor distance decays with
viewpoint separation.

The question is how far the initial position may be from the mapping trajectory
during relocalization. Entries are stored every 2 m along that trajectory, so
scoring entries pairwise and binning by viewpoint separation reads off directly
how far apart two viewpoints can be and still be recognized as the same place.

The criterion matches the node: an align distance (1 - per-sector cosine
similarity, taking the best column shift) below sc_score_thresh (0.35).

Note this measures separation along the trajectory. Moving laterally off it
changes the scene faster (nearby objects have more parallax), so the tolerance
there is smaller and this is an optimistic upper bound.

Usage: sc_range.py <map.pcd.sc>
"""
import struct
import sys

import numpy as np


def load(path):
    f = open(path, "rb")
    assert f.read(8) == b"SCTX0001", "not a scan-context database file"
    nr, ns, n = struct.unpack("<3i", f.read(12))
    (maxr,) = struct.unpack("<f", f.read(4))
    pos = np.empty((n, 4), np.float32)
    desc = np.empty((n, nr, ns), np.float32)
    for i in range(n):
        pos[i] = struct.unpack("<4f", f.read(16))
        desc[i] = np.frombuffer(f.read(nr * ns * 4), np.float32).reshape(nr, ns)
    return pos, desc, nr, ns, maxr


def align(a, b):
    """The node's align(): best per-sector cosine distance over all column shifts."""
    R, S = a.shape
    na = np.linalg.norm(a, axis=0)          # norm of each sector
    best = 2.0
    for s in range(S):
        bb = np.roll(b, -s, axis=1)
        nb = np.linalg.norm(bb, axis=0)
        ok = (na > 1e-5) & (nb > 1e-5)
        if ok.sum() < S // 4:
            continue
        dot = (a[:, ok] * bb[:, ok]).sum(0)
        d = (1.0 - dot / (na[ok] * nb[ok])).mean()
        best = min(best, d)
    return best


def main():
    pos, desc, nr, ns, maxr = load(sys.argv[1])
    thresh = float(sys.argv[2]) if len(sys.argv) > 2 else 0.35
    print(f"database: {len(pos)} entries, {nr}x{ns} bins, radius {maxr:.0f} m, threshold {thresh}")

    # Take a query every few entries and score it against those within a range before and after
    q = range(0, len(pos), max(1, len(pos) // 40))
    rows = []
    for i in q:
        for j in range(len(pos)):
            if j == i:
                continue
            d = float(np.linalg.norm(pos[j, :3] - pos[i, :3]))
            if d > 60.0:
                continue
            rows.append((d, align(desc[i], desc[j])))
    rows = np.array(rows)
    print(f"{len(rows)} sample pairs\n")

    print(f"{'separation':>11} {'n':>6} {'median':>9} {'below thr':>11}")
    edges = [0, 2, 4, 6, 8, 10, 15, 20, 30, 40, 60]
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (rows[:, 0] >= lo) & (rows[:, 0] < hi)
        if m.sum() < 5:
            continue
        sc = rows[m, 1]
        print(f"{lo:3.0f}-{hi:<3.0f} m {m.sum():8d} {np.median(sc):9.3f} "
              f"{100*(sc < thresh).mean():11.1f}%")


if __name__ == "__main__":
    main()
