"""
Figures for the report, generated from a run folder:

    python -m analysis.plots runs/<run>

perception_lidar.png   one LiDAR scan with the detected treads/risers
perception_kalman.png  raw per-scan height vs Kalman estimate vs truth
perception_accuracy.png  estimated vs true height & depth for each step
control_racks.png      rack positions vs their setpoints (front + rear)
control_tilt.png       body pitch/roll with the safety limits
fsm_timeline.png       state machine timeline
"""
import csv
import json
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config as C  # noqa: E402
from perception.step_detector import detect_step  # noqa: E402
from perception.perception import LIDAR_HEIGHT  # noqa: E402

# validated categorical slots (blue, orange, aqua) + neutral ink for ground truth
S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"
TRUTH = "#52514e"
GRID = "#e4e3df"
INK = "#0b0b0b"
STATES = ["SCAN", "ALIGN_WIDTH", "APPROACH", "CLEAN_TREAD", "FRONT_LIFT", "ADVANCE_FRONT", "PLACE_FRONT",
          "REAR_LIFT", "ADVANCE_REAR", "RETRACT_REAR", "DONE", "SAFE_ABORT"]


def _style(ax, title, xlabel, ylabel):
    ax.set_title(title, loc="left", fontsize=11, color=INK, pad=10)
    ax.set_xlabel(xlabel, color=TRUTH)
    ax.set_ylabel(ylabel, color=TRUTH)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    for sp in ("left", "bottom"):
        ax.spines[sp].set_color("#b5b4ae")
    ax.tick_params(colors=TRUTH)


def _save(fig, out, name):
    fig.tight_layout()
    fig.savefig(os.path.join(out, name), dpi=150, facecolor="white")
    plt.close(fig)


def _read_log(out):
    with open(os.path.join(out, "log.csv")) as f:
        rows = list(csv.DictReader(f))
    cols = {k: [r[k] for r in rows] for k in rows[0]}
    num = {k: np.array(v, float) for k, v in cols.items() if k not in ("state",)}
    num["state"] = cols["state"]
    return num


def plot_lidar(out):
    path = os.path.join(out, "first_scan.npy")
    if not os.path.exists(path):
        return
    pts = np.load(path)
    obs = detect_step(pts, ground_z_prior=-LIDAR_HEIGHT)
    fig, ax = plt.subplots(figsize=(7.5, 4.2))
    ax.scatter(pts[:, 0], pts[:, 1], s=8, color="#b5b4ae", label="raw LiDAR returns", zorder=2)
    first_h = first_v = True
    for s in obs.segments:
        if s.kind == "H":
            ax.plot(s.pts[:, 0], s.pts[:, 1], color=S1, lw=3, solid_capstyle="round", zorder=3,
                    label="tread (horizontal)" if first_h else None)
            first_h = False
        else:
            ax.plot(s.pts[:, 0], s.pts[:, 1], color=S2, lw=3, solid_capstyle="round", zorder=3,
                    label="riser (vertical)" if first_v else None)
            first_v = False
    ax.scatter([0], [0], marker="s", s=60, color=S3, zorder=4, label="sensor")
    if obs.step_ahead and obs.height is not None:
        x = obs.riser_dist
        ax.annotate("", xy=(x - 0.03, obs.ground_z + obs.height), xytext=(x - 0.03, obs.ground_z),
                    arrowprops=dict(arrowstyle="<->", color=INK, lw=1))
        ax.text(x - 0.04, obs.ground_z + obs.height / 2, f"h = {obs.height * 1000:.0f} mm",
                ha="right", va="center", color=INK, fontsize=9)
        if obs.depth:
            yy = obs.ground_z + obs.height + 0.03
            ax.annotate("", xy=(x, yy), xytext=(x + obs.depth, yy), arrowprops=dict(arrowstyle="<->", color=INK, lw=1))
            ax.text(x + obs.depth / 2, yy + 0.012, f"d = {obs.depth * 1000:.0f} mm", ha="center", color=INK, fontsize=9)
    ax.set_aspect("equal")
    _style(ax, "LiDAR scan from the floor, segmented into treads and risers",
           "forward distance from sensor (m)", "height relative to sensor (m)")
    ax.legend(frameon=False, fontsize=8, loc="upper left")
    _save(fig, out, "perception_lidar.png")


def plot_kalman(out):
    with open(os.path.join(out, "scans.json")) as f:
        data = json.load(f)
    steps = [s for s in data["steps"] if s["scans"] and s["scans"][0][0] is not None]
    if not steps:
        return
    st = steps[0]
    raw = np.array([s[0] if s[0] is not None else np.nan for s in st["scans"]]) * 1000
    kf = np.array([s[1] if s[1] is not None else np.nan for s in st["scans"]]) * 1000
    true_h = data["truth"]["heights"][st["tread"]] * 1000
    k = np.arange(1, len(raw) + 1)
    fig, ax = plt.subplots(figsize=(7, 3.6))
    ax.axhline(true_h, color=TRUTH, lw=1.5, ls="--")
    ax.plot(k, raw, "o", color=S2, ms=6, label="single-scan measurement")
    ax.plot(k, kf, "-", color=S1, lw=2, label="Kalman estimate")
    ax.text(k[0], true_h, "true height", va="bottom", color=TRUTH, fontsize=9)
    _style(ax, f"Kalman filter fusing {len(raw)} noisy LiDAR scans (step {st['tread'] + 1})",
           "scan number", "riser height (mm)")
    ax.legend(frameon=False, fontsize=8)
    _save(fig, out, "perception_kalman.png")


def plot_accuracy(out):
    with open(os.path.join(out, "summary.json")) as f:
        summ = json.load(f)
    rows = summ["steps"]
    if not rows:
        return
    n = np.arange(1, len(rows) + 1)
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.6))
    for ax, key, name in ((axes[0], "h", "riser height"), (axes[1], "d", "tread depth")):
        tv = np.array([r[f"{key}_true"] if r[f"{key}_true"] is not None else np.nan for r in rows]) * 1000
        ev = np.array([r[f"{key}_est"] if r[f"{key}_est"] is not None else np.nan for r in rows]) * 1000
        w = 0.36
        ax.bar(n - w / 2 - 0.01, tv, w, color="#c3c2b7", label="true")
        ax.bar(n + w / 2 + 0.01, ev, w, color=S1, label="estimated")
        lo = np.nanmin(np.concatenate([tv, ev])) * 0.85
        ax.set_ylim(lo, np.nanmax(np.concatenate([tv, ev])) * 1.06)
        for i, (a, b) in enumerate(zip(tv, ev)):
            if not np.isnan(a) and not np.isnan(b):
                ax.text(n[i], max(a, b) + (ax.get_ylim()[1] - lo) * 0.02, f"{b - a:+.1f}", ha="center",
                        fontsize=8, color=TRUTH)
        ax.set_xticks(n)
        _style(ax, f"{name}: estimate vs truth (error in mm)", "step", "mm")
        ax.legend(frameon=False, fontsize=8, loc="lower right")
    _save(fig, out, "perception_accuracy.png")


def _shade_states(ax, log, states=("REAR_LIFT", "ADVANCE_REAR", "RETRACT_REAR")):
    t, st = log["t"], log["state"]
    start = None
    for i in range(len(t)):
        on = st[i] in states
        if on and start is None:
            start = t[i]
        if (not on or i == len(t) - 1) and start is not None:
            ax.axvspan(start, t[i], color="#f1f0ec", zorder=0)
            start = None


def plot_racks(out):
    log = _read_log(out)
    fig, axes = plt.subplots(2, 1, figsize=(9, 5.2), sharex=True)
    for ax, key, name, col in ((axes[0], "front_lift", "front rack (lifts red front unit)", S1),
                               (axes[1], "leg", "rear rack (lifts blue back unit)", S2)):
        ax.plot(log["t"], log[f"{key}_target"] * 1000, color=TRUTH, lw=1, ls="--", label="setpoint")
        ax.plot(log["t"], log[key] * 1000, color=col, lw=2, label="measured")
        _style(ax, f"PID tracking: {name}", "", "extension (mm)")
        ax.legend(frameon=False, fontsize=8, loc="lower right", bbox_to_anchor=(1, 1), ncol=2)
    axes[1].set_xlabel("time (s)", color=TRUTH)
    _save(fig, out, "control_racks.png")


def plot_tilt(out):
    log = _read_log(out)
    fig, ax = plt.subplots(figsize=(9, 3.6))
    _shade_states(ax, log)
    lim = np.degrees(C.MAX_PITCH)
    ax.axhline(lim, color=TRUTH, ls="--", lw=1)
    ax.axhline(-lim, color=TRUTH, ls="--", lw=1)
    ax.text(log["t"][0], lim, " pitch safety limit", va="bottom", color=TRUTH, fontsize=8)
    ax.plot(log["t"], log["pitch_deg"], color=S1, lw=2, label="pitch")
    ax.plot(log["t"], log["roll_deg"], color=S2, lw=2, label="roll")
    ax.set_ylim(-lim * 1.25, lim * 1.25)
    _style(ax, "Body attitude during the climb (shaded: rear-unit lift, levelled by IMU PID)",
           "time (s)", "angle (deg)")
    ax.legend(frameon=False, fontsize=8, loc="lower right")
    _save(fig, out, "control_tilt.png")


def plot_timeline(out):
    log = _read_log(out)
    present = [s for s in STATES if s in set(log["state"])]
    idx = np.array([present.index(s) for s in log["state"]])
    fig, ax = plt.subplots(figsize=(9, 3.8))
    ax.step(log["t"], idx, where="post", color=S1, lw=2)
    ax.set_yticks(range(len(present)))
    ax.set_yticklabels(present, fontsize=8)
    ax.invert_yaxis()
    tread = log["tread"]
    for i in range(1, len(tread)):
        if tread[i] != tread[i - 1]:
            ax.axvline(log["t"][i], color=GRID, lw=1)
            ax.text(log["t"][i], -0.6, f" step {int(tread[i])}", fontsize=8, color=TRUTH)
    _style(ax, "Finite state machine timeline", "time (s)", "")
    _save(fig, out, "fsm_timeline.png")


def make_plots(out):
    for fn in (plot_lidar, plot_kalman, plot_accuracy, plot_racks, plot_tilt, plot_timeline):
        try:
            fn(out)
        except Exception as e:  # keep going so one bad plot doesn't hide the rest
            print(f"  plot {fn.__name__} failed: {e}")
    print(f"Plots written to {out}/")


if __name__ == "__main__":
    make_plots(sys.argv[1])
