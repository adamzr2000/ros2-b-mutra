#!/usr/bin/env python3
"""
Formation trajectory plot — D-MUTRA formation experiment.

3 rows (scenarios) × 1 column. Full trajectory from start (0 m) to goal (60 m).
Semantic colour coding: Robot 3 (attack target) in red, others in blue.
Text annotations removed — markers and legend are self-describing.
"""

from pathlib import Path
import glob, json, re
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import MultipleLocator
import numpy as np
import pandas as pd
import seaborn as sns

SCRIPT_DIR  = Path(__file__).parent
DATA_SUBDIR = "formation-v4"
DATA_DIR    = SCRIPT_DIR.parent / "data" / DATA_SUBDIR
OUT_PDF     = SCRIPT_DIR / "formation_impact_trajectory.pdf"

FONT_SCALE  = 1.9
SSP_PICK    = 10
GOAL        = (60.0, 0.0)
D           = 2.0

C_HEALTHY   = "#1f77b4"
C_TARGET    = "#d62728"
C_YAW = "#FFD700"   # gold — rotation intensity overlay

RC = {
    "robot1": C_HEALTHY,
    "robot2": C_HEALTHY,
    "robot3": C_TARGET,
    "robot4": C_HEALTHY,
}
C_CENTROID  = "black"
C_SNAP_EDGE = "dimgrey"
LW_ROBOT    = 2.2
LW_CENTROID = 2.2

SQ_ORDER  = ["robot1", "robot2", "robot3", "robot4"]
TRI_ORDER = ["robot1", "robot2", "robot4"]

sns.set_theme(context="paper", style="ticks",
              rc={"xtick.direction": "out", "ytick.direction": "out"},
              font_scale=FONT_SCALE)
plt.rcParams.update({"font.family": "serif", "pdf.fonttype": 42, "ps.fonttype": 42})


# ── Helpers ───────────────────────────────────────────────────────────────────
def scenario_key(tag):
    m = re.match(r"(baseline|unmitigated|mitigated)(?:-ssp(\d+))?-r(\d+)", tag)
    return m.group(1), (int(m.group(2)) // 1000 if m.group(2) else None)


def load_metrics():
    out = {}
    for f in glob.glob(str(DATA_DIR / "metrics-*.json")):
        d = json.load(open(f))
        out.setdefault(scenario_key(d["run_tag"]), []).append(d)
    return out


def has_yaw(tag):
    """True if the CSV for this run contains real yaw columns."""
    p = DATA_DIR / f"formation-{tag}.csv"
    if not p.exists():
        return False
    with open(p) as f:
        return "yaw_robot3" in f.readline()


def representative(reps, key="ife_compromise", prefer_yaw=False):
    candidates = [r for r in reps if r.get(key) is not None]
    if not candidates:
        return reps[0]
    mean = np.mean([r[key] for r in candidates])
    if prefer_yaw:
        yaw_cands = [r for r in candidates if has_yaw(r["run_tag"])]
        if yaw_cands:
            candidates = yaw_cands
    return min(candidates, key=lambda r: abs(r[key] - mean))


def load_csv(tag):
    return pd.read_csv(DATA_DIR / f"formation-{tag}.csv")


def robot_pos_at(df, t_s, robot):
    idx = (df["t_rel_s"] - t_s).abs().argmin()
    return float(df[f"x_{robot}"].iloc[idx]), float(df[f"y_{robot}"].iloc[idx])


def draw_snapshot(ax, df, t_s, active_robots, dot_ms=8, alpha=0.85, fill_alpha=0.07):
    pos = {r: robot_pos_at(df, t_s, r) for r in active_robots}
    pts = [pos[r] for r in active_robots]
    px, py = [p[0] for p in pts], [p[1] for p in pts]
    if fill_alpha > 0:
        ax.fill(px, py, color="silver", alpha=fill_alpha, zorder=7)
    xs = px + [px[0]]; ys = py + [py[0]]
    ax.plot(xs, ys, color=C_SNAP_EDGE, lw=1.4, alpha=alpha, zorder=8)
    for r in active_robots:
        ax.plot(*pos[r], "o", color=RC[r], ms=dot_ms, mew=0.7, zorder=9)



def plot_rotation_overlay(ax, df, robots, t_min=None, t_max=None, s=160, max_alpha=0.88):
    """Yellow scatter where alpha encodes yaw orientation angle deviation.

    signal = |ψ| / π  (ψ wrapped to [-π, π]).
    0 when facing the goal (+x, ψ≈0) → invisible.
    1 when facing opposite (ψ=±π) → bright.
    t_min/t_max clip to the spin window so pre-injection samples (yaw≈0,
    signal≈0) are excluded for clarity."""
    m = np.ones(len(df), dtype=bool)
    if t_min is not None:
        m &= df["t_rel_s"].values >= t_min
    if t_max is not None:
        m &= df["t_rel_s"].values <= t_max
    sub = df[m]
    data = {}
    for robot in robots:
        col = f"yaw_{robot}"
        if col not in sub.columns:
            continue
        yaw_wrap = np.angle(np.exp(1j * sub[col].values))  # [-π, π]
        signal   = np.abs(yaw_wrap) / np.pi                 # [0, 1]
        data[robot] = (sub[f"x_{robot}"].values, sub[f"y_{robot}"].values, signal)
    if not data:
        return
    rc, gc, bc = 1.0, 215 / 255, 0.0   # #FFD700
    for robot, (x, y, signal) in data.items():
        norm = np.clip(signal, 0.0, 1.0)
        rgba = np.column_stack([
            np.full(len(norm), rc),
            np.full(len(norm), gc),
            np.full(len(norm), bc),
            norm ** 0.5 * max_alpha,
        ])
        ax.scatter(x, y, s=s, c=rgba, linewidths=0, zorder=2)


def add_path_arrows(ax, df, robot, color, t_lo, t_hi, n=3, step=5, ms=9):
    mask = (df["t_rel_s"] >= t_lo) & (df["t_rel_s"] <= t_hi)
    xcol = ("centroid_x" if robot == "centroid" else f"x_{robot}")
    ycol = ("centroid_y" if robot == "centroid" else f"y_{robot}")
    x = df.loc[mask, xcol].values
    y = df.loc[mask, ycol].values
    if len(x) < step + 2:
        return
    idxs = np.linspace(int(len(x) * 0.15), int(len(x) * 0.80), n).astype(int)
    for i in idxs:
        j = min(i + step, len(x) - 1)
        dx, dy = x[j] - x[i], y[j] - y[i]
        if np.hypot(dx, dy) < 0.02:
            continue
        ax.annotate("", xy=(x[i] + dx, y[i] + dy), xytext=(x[i], y[i]),
                    arrowprops=dict(arrowstyle="-|>", color=color,
                                    lw=0.9, mutation_scale=ms,
                                    connectionstyle="arc3,rad=0"),
                    zorder=7)


# ── Load data ─────────────────────────────────────────────────────────────────
metrics  = load_metrics()
mit_rep  = representative(metrics[("mitigated",   SSP_PICK)], prefer_yaw=True)
unm_rep  = representative(metrics[("unmitigated", SSP_PICK)], prefer_yaw=True)
_base_all = metrics[("baseline", None)]
_base_yaw = [r for r in _base_all if has_yaw(r["run_tag"])]
base_rep  = min(_base_yaw or _base_all, key=lambda r: r["run_tag"])

df_mit  = load_csv(mit_rep["run_tag"])
df_unm  = load_csv(unm_rep["run_tag"])
df_base = load_csv(base_rep["run_tag"])

ROWS = [
    ("Baseline",  df_base, base_rep, "square"),
    ("Unmitigated",    df_unm,  unm_rep,  "unmitigated"),
    ("Mitigated",      df_mit,  mit_rep,  "reconfig"),
]

# ── Figure ────────────────────────────────────────────────────────────────────
# Full 0→60 m trajectory. Equal aspect dropped (68 m × 10 m ratio is impractical
# at paper width); formation outlines are slightly compressed vertically but all
# paths and endpoints are faithfully shown.
fig, axes = plt.subplots(3, 1, figsize=(16, 6.5),
                         sharex=True, sharey=True)

XLO, XHI = -3.0, 63.0
YLO, YHI = -5.1,  3.6

for row, (title, df, rep, mode) in enumerate(ROWS):
    ax = axes[row]

    t_inj  = rep.get("t_inject_s")
    t_det  = rep.get("t_detect_s")
    t_last = float(df["t_rel_s"].iloc[-1])

    # ── Rotation intensity overlay (spin window only: t_inj → t_det/t_last) ──
    if mode != "square" and t_inj is not None:
        t_clip = t_det if mode == "reconfig" else None
        plot_rotation_overlay(ax, df, ["robot3"], t_min=t_inj, t_max=t_clip)

    # ── Robot paths ───────────────────────────────────────────────────────────
    for robot in SQ_ORDER:
        ls = "--" if robot == "robot3" else "-"
        if mode == "reconfig" and robot == "robot3":
            mask = df["t_rel_s"] <= t_det
            ax.plot(df.loc[mask, f"x_{robot}"], df.loc[mask, f"y_{robot}"],
                    color=RC[robot], lw=LW_ROBOT, ls=ls, alpha=0.85, zorder=3)
        else:
            ax.plot(df[f"x_{robot}"], df[f"y_{robot}"],
                    color=RC[robot], lw=LW_ROBOT, ls=ls, alpha=0.85, zorder=3)

    # ── Centroid ──────────────────────────────────────────────────────────────
    ax.plot(df["centroid_x"], df["centroid_y"],
            color=C_CENTROID, lw=LW_CENTROID, zorder=5)

    # ── Formation snapshots ───────────────────────────────────────────────────
    draw_snapshot(ax, df, 0.0, SQ_ORDER)          # start — always square

    if mode == "square":
        draw_snapshot(ax, df, t_last, SQ_ORDER)   # goal — square formation arrived
    elif mode == "unmitigated":
        draw_snapshot(ax, df, t_last, SQ_ORDER,   # stall — faded, robots anchored
                      alpha=0.55, fill_alpha=0.10)
    elif mode == "reconfig":
        draw_snapshot(ax, df, t_last, TRI_ORDER)  # goal — triangle formation arrived
        # Detection-moment dots: all 4 robots at the isolation instant
        if t_det is not None:
            for r in SQ_ORDER:
                xi, yi = robot_pos_at(df, t_det, r)
                ax.plot(xi, yi, "o", color=RC[r], ms=8, mew=0.7, zorder=15)

    # ── Text annotations ─────────────────────────────────────────────────────────
    # "Attack injected": ha="right" puts text LEFT of xytext so the arrow exits
    #   to the right toward robot3's inject position — never through the text box.
    # "D-MUTRA …": va="top" puts text BELOW xytext so all three arrows go upward
    #   to the healthy robots, also never through the text box.
    fs_ann = plt.rcParams.get("axes.labelsize", 10) * 0.9
    ap     = dict(arrowstyle="-|>", color="black", lw=0.9, mutation_scale=8)
    bb     = dict(boxstyle="round,pad=0.3", facecolor="white",
                  edgecolor="black", linewidth=0.8, alpha=0.92)

    if t_inj is not None:
        r3xi, r3yi = robot_pos_at(df, t_inj, "robot3")
        # ha="right" → text sits LEFT of x=10; arrow exits right-center toward robot3
        ax.annotate("Attack injection",
                    xy=(r3xi, r3yi), xycoords="data",
                    xytext=(10, -4), textcoords="data",
                    va="center", ha="right", fontsize=fs_ann,
                    arrowprops=dict(**ap, connectionstyle="arc3,rad=0.2"),
                    bbox=bb, zorder=14)

    if t_det is not None and mode == "reconfig":
        healthy = [r for r in SQ_ORDER if r != "robot3"]
        hx = [robot_pos_at(df, t_det, r)[0] for r in healthy]
        hy = [robot_pos_at(df, t_det, r)[1] for r in healthy]
        # ha="left" → text sits RIGHT of x=25; arrows exit left-center to healthy robots
        # r1, r2 (y≈+2): up-left;  r4 (y≈-2): down-left
        ax.text(20, -1.5, "Attack detected; Robot 3 isolated",
                va="center", ha="left", fontsize=fs_ann,
                bbox=bb, zorder=14)
        rads = [0.15, 0.0, -0.15]   # r1 up-left, r2 up-left, r4 down-left
        for (xi, yi), rad in zip(zip(hx, hy), rads):
            ax.annotate("", xy=(xi, yi), xycoords="data",
                        xytext=(20, -1.5), textcoords="data",
                        arrowprops=dict(**ap, connectionstyle=f"arc3,rad={rad}"),
                        zorder=14)

    # ── Goal and start markers ────────────────────────────────────────────────
    ax.plot(GOAL[0], GOAL[1], "*", color="black", ms=20, mew=0.7, zorder=8)
    ax.plot(0, 0, "o", color="black", ms=7, mew=0, zorder=8, alpha=0.70)

    # ── Axes styling ──────────────────────────────────────────────────────────
    ax.set_xlim(XLO, XHI)
    ax.set_ylim(YLO, YHI)
    ax.xaxis.set_major_locator(MultipleLocator(10))
    ax.xaxis.set_minor_locator(MultipleLocator(5))
    ax.yaxis.set_major_locator(MultipleLocator(2))
    ax.grid(axis="both", which="major", linewidth=0.7, alpha=0.40, color="grey")
    ax.grid(axis="x",    which="minor", linewidth=0.3, alpha=0.20, color="grey")
    ax.set_axisbelow(True)
    ax.set_ylabel("Y (m)", labelpad=2)

    ax.text(-0.06, 0.5, title, transform=ax.transAxes,
            rotation=90, va="center", ha="center",
            fontsize=plt.rcParams.get("axes.labelsize", 10),
            fontweight="semibold", linespacing=1.4)

axes[2].set_xlabel("X (meters)")

# ── Legend ────────────────────────────────────────────────────────────────────
legend_elements = [
    Line2D([0], [0], color=C_HEALTHY,   lw=LW_ROBOT,    label="Robots 1, 2, 4"),
    Line2D([0], [0], color=C_TARGET,    lw=LW_ROBOT, ls="--", label="Robot 3"),
    Line2D([0], [0], marker="o", color=C_YAW, ms=9, alpha=0.75, lw=0,
           label="Robot 3 orientation"),
    Line2D([0], [0], color=C_CENTROID,  lw=LW_ROBOT,    label="Swarm centroid"),
    Line2D([0], [0], color=C_SNAP_EDGE, lw=1.0,         label="Formation outline"),
    Line2D([0], [0], marker="*", color="black", ms=18, lw=0, label="Goal"),
]
fig.legend(handles=legend_elements,
           loc="upper center", bbox_to_anchor=(0.5, 1.0),
           ncol=6, frameon=True, framealpha=0.9, fancybox=False,
           edgecolor="black", handlelength=1.5,
           columnspacing=0.9, borderpad=0.6, handletextpad=0.5)

fig.subplots_adjust(top=0.88, bottom=0.08, left=0.06, right=0.99,
                    hspace=0.10)
fig.savefig(OUT_PDF, bbox_inches="tight")
print(f"Saved → {OUT_PDF}")
print(f"\n── Trajectory plot values ───────────────────────────────")
print(f"  baseline    : {base_rep['run_tag']}")
print(f"  unmitigated : {unm_rep['run_tag']}")
print(f"    t_inject={unm_rep.get('t_inject_s')}s  progress={unm_rep['progress_pct']:.1f}%")
print(f"  mitigated   : {mit_rep['run_tag']}")
print(f"    t_inject={mit_rep.get('t_inject_s')}s  t_detect={mit_rep.get('t_detect_s')}s  progress={mit_rep['progress_pct']:.1f}%")
