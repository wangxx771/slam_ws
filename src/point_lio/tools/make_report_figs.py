#!/usr/bin/env python3
"""Generate every figure for the test report into test_report/figs/.

Requires result directories from completed runs: /tmp/cmp/* (deterministic
offline replay) and /tmp/reloc/RQ* (live relocalization).
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 150
plt.rcParams["savefig.bbox"] = "tight"
plt.rcParams["axes.grid"] = True
plt.rcParams["grid.alpha"] = 0.25

OUT = "/home/toe/slam_ws/test_report/figs"
os.makedirs(OUT, exist_ok=True)
BAG = "/home/toe/slam_ws/bags/park_ros2"

C_GT, C_BASE, C_GNSS, C_FL = "#333333", "#c0332f", "#2a78d6", "#0f7a43"


def load(tag, root="/tmp/cmp"):
    return np.loadtxt(f"{root}/{tag}/odom.csv", delimiter=",", skiprows=1,
                      usecols=range(8), ndmin=2)


def read_gps():
    """RTK ground truth for park, converted to ENU."""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import eval_gps as E
    ts, enu, _, _ = E.read_gps(BAG)
    return np.column_stack([ts, enu])


def align4(est, gt):
    """Align est to gt with 4 DoF (yaw + translation); returns the aligned est and per-point errors."""
    t = est[:, 0]
    ref = np.stack([np.interp(t, gt[:, 0], gt[:, k]) for k in (1, 2, 3)], 1)
    p = est[:, 1:4]
    mp, mr = p.mean(0), ref.mean(0)
    A, B = p - mp, ref - mr
    num = (A[:, 0] * B[:, 1] - A[:, 1] * B[:, 0]).sum()
    den = (A[:, 0] * B[:, 0] + A[:, 1] * B[:, 1]).sum()
    th = np.arctan2(num, den)
    R = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1]])
    q = (R @ (p - mp).T).T + mr
    return q, ref, np.linalg.norm(q - ref, axis=1)


def fig_park_traj(gt):
    b, g, f = load("M_base"), load("MKMAP"), load("FL_P")
    qb, _, eb = align4(b, gt)
    qg, _, eg = align4(g, gt)
    qf, _, ef = align4(f, gt)
    def hrms(q, src):
        ref = np.stack([np.interp(src[:, 0], gt[:, 0], gt[:, k]) for k in (1, 2)], 1)
        return np.sqrt((np.linalg.norm(q[:, :2] - ref, axis=1) ** 2).mean())

    fig, ax = plt.subplots(1, 2, figsize=(11, 5.6))
    ax[0].plot(gt[:, 1], gt[:, 2], color=C_GT, lw=2.4, label="RTK ground truth", zorder=1)
    ax[0].plot(qb[:, 0], qb[:, 1], color=C_BASE, lw=1.1,
               label=f"point_lio LIO only   horizontal {hrms(qb, b):.2f} m")
    ax[0].plot(qf[:, 0], qf[:, 1], color=C_FL, lw=1.1, ls="--",
               label=f"FAST_LIO_GPU   horizontal {hrms(qf, f):.2f} m")
    ax[0].plot(qg[:, 0], qg[:, 1], color=C_GNSS, lw=1.4,
               label=f"point_lio + GNSS   horizontal {hrms(qg, g):.2f} m")
    ax[0].set_title("park horizontal trajectory (after 4-DoF alignment)\nall three nearly coincide horizontally; the error is entirely in elevation")
    ax[0].set_xlabel("X / m"); ax[0].set_ylabel("Y / m"); ax[0].axis("equal")
    ax[0].legend(fontsize=8, loc="best")

    t0 = gt[0, 0]
    ax[1].plot(gt[:, 0] - t0, gt[:, 3] - gt[0, 3], color=C_GT, lw=2.0, label="RTK elevation")
    for est, c, n in ((qb, C_BASE, "point_lio LIO only"), (qf, C_FL, "FAST_LIO_GPU"),
                      (qg, C_GNSS, "point_lio + GNSS")):
        src = {id(qb): b, id(qf): f, id(qg): g}[id(est)]
        ax[1].plot(src[:, 0] - t0, est[:, 2] - est[0, 2], color=c, lw=1.2, label=n)
    ax[1].set_title("park elevation over time")
    ax[1].set_xlabel("time / s"); ax[1].set_ylabel("elevation relative to start / m")
    ax[1].legend(fontsize=8)
    fig.savefig(f"{OUT}/fig1_park_traj.png"); plt.close(fig)
    return eb, eg, ef


def fig_tilt(gt):
    """Elevation error vs horizontal displacement: shows a tilted world frame rather than random drift."""
    b = load("M_base")
    qb, ref, _ = align4(b, gt)
    d = np.linalg.norm(qb[:, :2] - qb[0, :2], axis=1)
    ez = qb[:, 2] - ref[:, 2]
    k, c = np.polyfit(d, ez, 1)
    r2 = 1 - ((ez - (k * d + c)) ** 2).sum() / ((ez - ez.mean()) ** 2).sum()
    fig, ax = plt.subplots(figsize=(6.4, 4.4))
    ax.scatter(d[::200], ez[::200], s=6, alpha=0.5, color=C_BASE)
    xs = np.linspace(0, d.max(), 50)
    ax.plot(xs, k * xs + c, color="#11161d", lw=2,
            label=f"linear fit, slope={k*100:.2f} cm/m ({np.degrees(np.arctan(k)):.2f} deg)\nR2={r2:.3f}")
    ax.set_xlabel("horizontal displacement from start / m"); ax.set_ylabel("elevation error / m")
    ax.set_title("park elevation error is proportional to displacement -> tilted world frame, not accumulated drift")
    ax.legend(fontsize=9)
    fig.savefig(f"{OUT}/fig2_tilt.png"); plt.close(fig)
    return k, r2


def fig_matrix():
    items = [("outlier off\ndynamic on", 11.974), ("baseline\n(outlier+dynamic on)", 11.435),
             ("dynamic off\n(recommended front end)", 11.235), ("+loop", 11.435),
             ("+GNSS", 0.322), ("all on", 0.321)]
    fig, ax = plt.subplots(figsize=(8.6, 4.2))
    x = np.arange(len(items))
    v = [i[1] for i in items]
    cols = [C_BASE if a > 1 else C_GNSS for a in v]
    bars = ax.bar(x, v, color=cols, width=0.62)
    for b, a in zip(bars, v):
        ax.text(b.get_x() + b.get_width() / 2, a + 0.25, f"{a:.3f}",
                ha="center", fontsize=9)
    ax.set_xticks(x); ax.set_xticklabels([i[0] for i in items], fontsize=9)
    ax.set_ylabel("ATE RMSE / m"); ax.set_ylim(0, 13.6)
    ax.set_title("park feature-switch A/B (all other parameters identical, deterministic offline replay)")
    fig.savefig(f"{OUT}/fig3_matrix.png"); plt.close(fig)


def fig_gnss_err(gt):
    b, g = load("M_base"), load("MKMAP")
    _, _, eb = align4(b, gt)
    _, _, eg = align4(g, gt)
    t0 = gt[0, 0]
    fig, ax = plt.subplots(figsize=(8.6, 4.0))
    ax.plot(b[:, 0] - t0, eb, color=C_BASE, lw=0.9, label=f"LIO only  median {np.median(eb):.2f} m")
    ax.plot(g[:, 0] - t0, eg, color=C_GNSS, lw=0.9, label=f"+GNSS  median {np.median(eg):.3f} m")
    ax.set_yscale("log"); ax.set_xlabel("time / s"); ax.set_ylabel("localization error / m (log axis)")
    ax.set_title("loosely coupled GNSS brings the error from metres to decimetres and stops it growing")
    ax.legend(fontsize=9)
    fig.savefig(f"{OUT}/fig4_gnss.png"); plt.close(fig)


def fig_reloc(gt):
    ref = load("MKMAP")
    tags = [50, 100, 150, 200, 250, 300, 350, 400, 450, 500]
    fig, ax = plt.subplots(1, 2, figsize=(11.4, 6.2),
                           gridspec_kw={"width_ratios": [1, 1.25]})
    ax[0].plot(ref[:, 1], ref[:, 2], color="#aab2bd", lw=1.0, label="prior-map trajectory")
    meds = []
    for o in tags:
        a = np.loadtxt(f"/tmp/reloc/RQ{o}/odom.csv", delimiter=",", skiprows=1,
                       usecols=range(4), ndmin=2)
        i = np.argmin(np.abs(ref[:, 0] - (ref[0, 0] + o)))
        ax[0].plot(ref[i, 1], ref[i, 2], "o", ms=9, mfc="#2a78d6", mec="w", mew=1.4, zorder=5)
        ax[0].annotate(f"{o}s", (ref[i, 1], ref[i, 2]), textcoords="offset points",
                       xytext=(8, 4), fontsize=8, color="#11161d")
    ax[0].set_title("distribution of the 10 relocalization start points\n(89 x 334 m, 29.6k m2)")
    ax[0].set_xlabel("X / m"); ax[0].set_ylabel("Y / m"); ax[0].axis("equal")
    ax[0].legend(fontsize=8, loc="lower right")

    import csv as _csv
    for o in tags:
        rows = [r for r in open(f"/tmp/reloc/RQ{o}/odom.csv").read().splitlines()[1:]]
        rows = [r.split(",") for r in rows]
        rows = [r for r in rows if len(r) == 16 and r[9] != ""]
        a = np.array([[float(v) for v in r[:4]] for r in rows])
        a = a[a[:, 0] - a[0, 0] > 2.0]
        g = np.stack([np.interp(a[:, 0], ref[:, 0], ref[:, k]) for k in (1, 2, 3)], 1)
        e = np.linalg.norm(a[:, 1:4] - g, axis=1)
        meds.append(np.median(e))
        ax[1].plot(a[:, 0] - a[0, 0], e, lw=1.0, label=f"start {o}s  median {np.median(e):.3f} m")
    ax[1].set_yscale("log")
    ax[1].set_xlabel("time since lock / s"); ax[1].set_ylabel("position error against the prior map / m")
    ax[1].set_title(f"error stays at decimetre level throughout after lock\n10/10 succeeded, overall median {np.median(meds):.3f} m")
    ax[1].legend(fontsize=7, ncol=2)
    fig.savefig(f"{OUT}/fig5_reloc.png"); plt.close(fig)


def fig_carla():
    import rclpy.serialization as ser
    import rosbag2_py
    from nav_msgs.msg import Odometry
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri="/home/toe/slam_ws/bags/carla_t", storage_id=""),
           rosbag2_py.ConverterOptions("", ""))
    rows = []
    while r.has_next():
        tp, d, _ = r.read_next()
        if tp != "/groundtruth":
            continue
        m = ser.deserialize_message(d, Odometry)
        rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                     m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z))
    gt = np.array(rows)
    p = load("PL_C")
    src = p[:, 1:4]
    mu_s, mu_d = src.mean(0), None
    ref = np.stack([np.interp(p[:, 0], gt[:, 0], gt[:, k]) for k in (1, 2, 3)], 1)
    mu_d = ref.mean(0)
    H = (src - mu_s).T @ (ref - mu_d) / len(src)
    U, _, Vt = np.linalg.svd(H)
    dsign = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, dsign]) @ U.T
    q = (R @ (src - mu_s).T).T + mu_d
    e = np.linalg.norm(q - ref, axis=1)
    fig, ax = plt.subplots(figsize=(6.6, 5.4))
    ax.plot(gt[:, 1], gt[:, 2], color=C_GT, lw=3.0, alpha=0.5, label="CARLA ground truth")
    ax.plot(q[:, 0], q[:, 1], color=C_GNSS, lw=1.3,
            label=f"point_lio  ATE {np.sqrt((e**2).mean()):.3f} m")
    ax.set_title("CARLA 72 deg sector lidar\nFAST_LIO_GPU diverges on the same data (>1e4 m in all 4 configurations, not plotted)",
                 fontsize=10)
    ax.set_xlabel("X / m"); ax.set_ylabel("Y / m"); ax.axis("equal")
    ax.legend(fontsize=9)
    fig.savefig(f"{OUT}/fig6_carla.png"); plt.close(fig)


def fig_res():
    labels = ["CARLA\n(24 s)", "AVIA\n(64 s)", "park\n(551 s)"]
    pl_cpu, fl_cpu = [11.3, 8.2, 122.9], [27.2, 8.8, 56.2]
    pl_rss, fl_rss = [103, 78, 132], [354, 286, 496]
    x = np.arange(3); w = 0.36
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.8))
    ax[0].bar(x - w / 2, pl_cpu, w, color=C_GNSS, label="point_lio")
    ax[0].bar(x + w / 2, fl_cpu, w, color=C_FL, label="FAST_LIO_GPU")
    ax[0].set_xticks(x); ax[0].set_xticklabels(labels, fontsize=9)
    ax[0].set_ylabel("CPU time / s"); ax[0].set_title("offline replay CPU cost"); ax[0].legend(fontsize=8)
    ax[1].bar(x - w / 2, pl_rss, w, color=C_GNSS, label="point_lio")
    ax[1].bar(x + w / 2, fl_rss, w, color=C_FL, label="FAST_LIO_GPU")
    ax[1].set_xticks(x); ax[1].set_xticklabels(labels, fontsize=9)
    ax[1].set_ylabel("peak RSS / MB"); ax[1].set_title("peak memory"); ax[1].legend(fontsize=8)
    fig.savefig(f"{OUT}/fig7_res.png"); plt.close(fig)


def fig_map():
    """Top-down view of the built park cloud, coloured by elevation and decimated."""
    import struct
    f = open("/home/toe/slam_ws/PCD/park2.pcd", "rb")
    hdr, line = {}, b""
    while True:
        line = f.readline()
        parts = line.decode("ascii", "ignore").split()
        if not parts:
            continue
        hdr[parts[0]] = parts[1:]
        if parts[0] == "DATA":
            break
    n = int(hdr["POINTS"][0])
    sz = sum(int(s) * int(c) for s, c in zip(hdr["SIZE"], hdr["COUNT"]))
    step = max(1, n // 400000)
    xs, ys, zs = [], [], []
    for i in range(0, n, step):
        f.seek(-0, 1)
        pass
    buf = np.frombuffer(f.read(n * sz), dtype=np.uint8).reshape(n, sz)
    xyz = buf[::step, :12].copy().view(np.float32).reshape(-1, 3)
    m = np.isfinite(xyz).all(1)
    xyz = xyz[m]
    fig, ax = plt.subplots(figsize=(5.2, 8.0))
    s = ax.scatter(xyz[:, 0], xyz[:, 1], c=xyz[:, 2], s=0.06, cmap="viridis",
                   vmin=np.percentile(xyz[:, 2], 2), vmax=np.percentile(xyz[:, 2], 98))
    ax.set_aspect("equal"); ax.set_xlabel("X / m"); ax.set_ylabel("Y / m")
    ax.set_title(f"park mapping result park2.pcd\n{n/1e4:.0f}0k points, coloured by elevation", fontsize=10)
    fig.colorbar(s, ax=ax, label="Z / m", shrink=0.6)
    fig.savefig(f"{OUT}/fig8_map.png"); plt.close(fig)


if __name__ == "__main__":
    gt = read_gps()
    print("ground truth", gt.shape)
    eb, eg, ef = fig_park_traj(gt)
    k, r2 = fig_tilt(gt)
    print(f"tilt {np.degrees(np.arctan(k)):.2f} deg  R2={r2:.3f}")
    fig_matrix()
    fig_gnss_err(gt)
    fig_reloc(gt)
    fig_carla()
    fig_res()
    fig_map()
    print("figures written to", OUT)
