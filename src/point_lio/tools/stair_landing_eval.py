#!/usr/bin/env python3
"""Mapping-consistency metric for stairwells: how far apart is the same landing
measured on the way up and on the way down.

Why it has to be the landings. A stairwell is a vertical shaft with geometrically
identical floors, and every conventional metric is fooled by that self-similarity
- four were tried and all failed:

    metric                        what goes wrong                 how badly
    multi-layer z per xy cell     a stairwell IS multi-layer      reports 72.6% multi-layer; measures floors
    z cluster spacing ("ghost")   median spacing equals riser     identical for three maps of different quality
    xy nearest neighbour up/down  every floor has points at an xy median |dz| 3.9 m
    arc length up/down            not an equal-length retrace     115.5 m up vs 71.4 m down

A landing is different because it is a physical object: the same landing must be
at the same height going up and coming down. That constraint depends on neither
the path shape nor what the cloud looks like, and the difference is exactly what
accumulated over that leg.

Usage:
    stair_landing_eval.py <reference_odom.csv> <candidate.csv> [<candidate2> ...]

The first argument is the reference trajectory. Dwell windows and floor pairing
are computed on it once and applied unchanged to every candidate. This is
required: the candidates are different post-processings of the same run, so the
motion is identical and the windows should be too. Without fixing them the
detected floor count varies by method (4 vs 3 observed) and the means are not
comparable. When comparing different runs, each is its own reference.
"""
import os
import sys

import numpy as np

# Decimate to about 10 Hz before finding dwells. The step must be derived from
# the actual sample rate, not hard-coded: point_lio publishes odom at 200 Hz
# (step 20) while FAST_LIO publishes per lidar frame (10 Hz, step 1). A fixed 20
# thins the latter to 0.5 Hz, detects no dwells at all, and looks like it has no
# layering.
TARGET_HZ = 10.0
WIN = 15            # 1.5 s
FLAT = 0.08         # z range within the window below this counts as standing still
LEVEL_MERGE = 1.0   # merge adjacent dwells into one floor below this height gap (floors are 1.6-2.2 m apart, ample margin)


def dwell_windows(z):
    """Return [start, end) indices of flat-z stretches. z changes continuously while climbing and is flat only on a landing."""
    out, i = [], 0
    while i + WIN < len(z):
        if z[i:i + WIN].max() - z[i:i + WIN].min() < FLAT:
            e = i + WIN
            while e < len(z) and max(z[i:e + 1]) - min(z[i:e + 1]) < FLAT:
                e += 1
            out.append((i, e))
            i = e
        else:
            i += 1
    return out


def to_levels(wins, z):
    """Merge dwells into floors; returns [(height, count, [windows...])]."""
    lv = []
    for a, b in wins:
        m = z[a:b].mean()
        if lv and abs(m - lv[-1][0]) < LEVEL_MERGE:
            h, n, w = lv[-1]
            lv[-1] = ((h * n + m) / (n + 1), n + 1, w + [(a, b)])
        else:
            lv.append((m, 1, [(a, b)]))
    return lv


def flights(d, rise=0.8, win_s=6.0):
    """Segment the individual flights: continuous climb/descent stretches.
    Returns [(start, end, direction)] indices into the raw samples.

    A flight is a ramp monotonic in both z and x. Detection uses whether |dz|
    over win_s seconds exceeds rise, which is steadier than a per-point gradient
    - the dog's gait makes per-point dz flip sign, and a gradient test segments
    nothing.
    """
    t, z = d[:, 0], d[:, 3]
    hz = (len(t) - 1) / max(1e-6, t[-1] - t[0])
    w = max(2, int(win_s * hz))
    out, i = [], 0
    while i + w < len(z):
        dz = z[i + w] - z[i]
        if abs(dz) > rise:
            j = i
            while j + w < len(z) and (z[j + w] - z[j]) * dz > 0:
                j += 1
            out.append((i, min(j + w, len(z) - 1), 1 if dz > 0 else -1))
            i = j + w
        else:
            i += 1
    return out


def flight_mismatch(path):
    """Ascent vs descent on the same flight: pair along x and compare z.

    This is the second valid anchor in this scene. A flight is monotonic in x, so
    x uniquely fixes the position along it and pairing by x is sound - unlike xy
    on a landing, where the dog stands wherever it likes and a 0.5 m difference
    between up and down is not error (half a day was spent chasing that).
    """
    d = np.genfromtxt(path, delimiter=",", skip_header=1)
    fl = flights(d)
    up = [f for f in fl if f[2] > 0]
    dn = [f for f in fl if f[2] < 0]
    res = []
    for k, u in enumerate(up):
        # The descent retraces the route, so ascent flight k pairs with descent flight k from the end
        if k >= len(dn):
            break
        v = dn[len(dn) - 1 - k]
        U, D = d[u[0]:u[1], 1:4], d[v[0]:v[1], 1:4]
        if len(U) < 20 or len(D) < 20:
            continue
        lo = max(U[:, 0].min(), D[:, 0].min()) + 0.3
        hi = min(U[:, 0].max(), D[:, 0].max()) - 0.3
        if hi - lo < 1.0:
            continue
        xs = np.linspace(lo, hi, 40)
        su, sd = U[np.argsort(U[:, 0])], D[np.argsort(D[:, 0])]
        res.append(float(np.abs(np.interp(xs, su[:, 0], su[:, 2])
                                - np.interp(xs, sd[:, 0], sd[:, 2])).mean()))
    return res


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    ref = np.genfromtxt(sys.argv[1], delimiter=",", skip_header=1)
    hz = (len(ref) - 1) / max(1e-6, ref[-1, 0] - ref[0, 0])
    decim = max(1, int(round(hz / TARGET_HZ)))
    zs = ref[::decim, 3]
    apex = int(np.argmax(ref[:, 3])) // decim
    wins = dwell_windows(zs)
    lu = to_levels([w for w in wins if w[0] < apex], zs)
    ld = to_levels([w for w in wins if w[0] >= apex], zs)

    # The top floor is the turnaround, where the two legs necessarily coincide,
    # so pair downward from there. But the top dwell is short (2.3 s measured) and
    # some methods miss it, which shifts a fixed-index pairing by one - that
    # paired +5.411 with +3.006 and reported a bogus 2.4 m inconsistency. So
    # search a global shift over -2..2 and take the one minimising total
    # inconsistency: floor spacing far exceeds the up/down difference, so the
    # error drops off a cliff at the correct alignment.
    best, shift = 9e9, 0
    for sh in range(-2, 3):
        d = [abs(ld[k][0] - lu[len(lu) - 1 - k - sh][0])
             for k in range(len(ld)) if 0 <= len(lu) - 1 - k - sh < len(lu)]
        if len(d) >= 3 and np.mean(d) < best:
            best, shift = np.mean(d), sh
    pairs = [(len(lu) - 1 - k - shift, k) for k in range(len(ld))
             if 0 <= len(lu) - 1 - k - shift < len(lu)]
    if len(pairs) < 3:
        print(f"not enough pairs (up {len(lu)} floors / down {len(ld)}); the metric does not apply")
        return 1
    print(f"reference {os.path.basename(os.path.dirname(sys.argv[1]))} ({hz:.0f} Hz, step {decim}): "
          f"up {len(lu)} floors / down {len(ld)}, {len(pairs)} fixed pairs (shift {shift:+d})")

    for p in sys.argv[2:]:
        raw = np.genfromtxt(p, delimiter=",", skip_header=1)
        hz2 = (len(raw) - 1) / max(1e-6, raw[-1, 0] - raw[0, 0])
        d = raw[::max(1, int(round(hz2 / TARGET_HZ))), 3]
        z_of = lambda wl: float(np.mean([d[a:b].mean() for a, b in wl]))  # noqa: E731
        diff = [z_of(ld[k][2]) - z_of(lu[ia][2]) for ia, k in pairs]
        a = np.abs(diff)
        fm = flight_mismatch(p)
        fs = f"  same-flight |dz| mean {np.mean(fm):.3f} m ({len(fm)} flights)" if fm else "  same-flight metric: no flights segmented"
        print(f"  {os.path.basename(os.path.dirname(p)):<16} "
              f"landing |dz| mean {a.mean():.3f} m  max {a.max():.3f} m{fs}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
