#!/usr/bin/env python3
"""
Per-robot mission progress figure for D-MUTRA swarm experiments.

Three subplots (baseline / unmitigated / mitigated-SSP10s), each showing
the waypoints completed over time for every individual robot.
Thin lines = individual runs; bold lines = mean across runs.

Key visuals:
  • robot3 (red) freezes at 3/8 WPs immediately after the attack in both
    attack scenarios — the flat line shows the frozen controller.
  • robot4 (orange) absorbs robot3's 5 orphaned WPs in the mitigated case,
    climbing to 13 total — visibly above the 8-WP ceiling of the others.
  • robots 1 & 2 are unaffected in every scenario (serves as control).
"""

from pathlib import Path
import glob
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import seaborn as sns

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).parent
DATA_DIR   = SCRIPT_DIR.parent / "data" / "mission"
OUT_PDF    = SCRIPT_DIR / "mission_impact_per_robot.pdf"

# ── Style ─────────────────────────────────────────────────────────────────────
FONT_SCALE = 1.8
sns.set_theme(
    context="paper",
    style="ticks",
    rc={"xtick.direction": "out", "ytick.direction": "out"},
    font_scale=FONT_SCALE,
)
plt.rcParams.update({"font.family": "serif", "pdf.fonttype": 42, "ps.fonttype": 42})

# ── Colors per robot ──────────────────────────────────────────────────────────
ROBOT_COLORS = {
    "robot1": "#4B9FCC",   # palette-C blue
    "robot2": "#6a5d99",   # palette-C purple
    "robot3": "#C44E52",   # red  — compromised
    "robot4": "#E8883A",   # palette-C orange — rescuer
}
ROBOT_LABELS = {
    "robot1": "Robot 1",
    "robot2": "Robot 2",
    "robot3": "Robot 3 (compromised)",
    "robot4": "Robot 4 (rescuer)",
}
ROBOT_MARKERS = {
    "robot1": "o",   # circle
    "robot2": "s",   # square
    "robot3": "^",   # triangle up
    "robot4": "D",   # diamond
}

LW_TRACE = 0.8
LW_MAIN  = 2.0
ALPHA_TRACE = 0.18
T_MAX = 230

# ── Helpers ───────────────────────────────────────────────────────────────────
def load_csvs(pattern, min_rows=50, min_max_pct=50.0):
    files = sorted(glob.glob(str(DATA_DIR / pattern)))
    dfs = []
    for f in files:
        df = pd.read_csv(f)
        if len(df) < min_rows or df["pct"].max() < min_max_pct:
            continue
        dfs.append(df)
    return dfs


def robot_curve(dfs, robot_col, t_max=T_MAX, dt=0.5):
    """Mean + std of one robot's completed-WP count across runs."""
    tg = np.arange(0, t_max + dt, dt)
    arr = np.array([
        np.interp(tg, df["t_rel_s"].values, df[robot_col].values,
                  left=0.0, right=float(df[robot_col].values[-1]))
        for df in dfs
    ])
    return tg, arr.mean(axis=0), arr


def draw_scenario(ax, dfs, robots, title, show_xlabel):
    """Draw per-robot traces + mean + per-step markers for one scenario panel."""
    for robot in robots:
        col    = f"done_{robot}"
        color  = ROBOT_COLORS[robot]
        marker = ROBOT_MARKERS[robot]
        tg, ymean, yall = robot_curve(dfs, col)

        # Individual run traces (no markers — keep thin)
        for yi in yall:
            ax.plot(tg, yi, color=color, lw=LW_TRACE, alpha=ALPHA_TRACE)

        # Bold mean
        ax.plot(tg, ymean, color=color, lw=LW_MAIN, label=ROBOT_LABELS[robot])

        # Marker at each waypoint completion step on the mean curve
        max_wps = int(np.round(ymean.max()))
        for k in range(1, max_wps + 1):
            idx = np.argmax(ymean >= k)   # first time mean reaches WP k
            if idx > 0:
                ax.plot(tg[idx], ymean[idx], marker=marker,
                        color=color, ms=5, zorder=6, lw=0)

    ax.set_xlim(0, T_MAX)
    ax.set_ylim(-0.3, 14.5)
    ax.set_yticks([0, 4, 8, 12])
    ax.axhline(8, color="gray", lw=0.7, ls="--", alpha=0.5)   # normal ceiling
    ax.set_title(title, pad=4)
    if show_xlabel:
        ax.set_xlabel("Time (s)")
    for spine in ax.spines.values():
        spine.set_visible(True)


# ── Load ──────────────────────────────────────────────────────────────────────
base_dfs  = load_csvs("mission-baseline-r*.csv")
unmit_dfs = load_csvs("mission-unmitigated-ssp10000-r*.csv")
mit_dfs   = load_csvs("mission-mitigated-ssp10000-r*.csv")

ROBOTS = ["robot1", "robot2", "robot3", "robot4"]

# ── Figure: 3 stacked subplots ─────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(7, 9), sharex=True)

draw_scenario(axes[0], base_dfs,  ROBOTS, "Baseline (no attack)",              show_xlabel=False)
draw_scenario(axes[1], unmit_dfs, ROBOTS, "Unmitigated (attack, no recovery)", show_xlabel=False)
draw_scenario(axes[2], mit_dfs,   ROBOTS, "Mitigated (D-MUTRA, SSP 10 s)",     show_xlabel=True)

# Shared y-label
fig.text(0.02, 0.5, "Waypoints completed", va="center", rotation="vertical",
         fontsize=plt.rcParams["font.size"])

# Legend from last panel (has all 4 robots incl. labels for r3/r4)
handles, labels = axes[2].get_legend_handles_labels()
fig.legend(handles, labels,
           loc="upper center", bbox_to_anchor=(0.5, 1.01),
           ncol=2, frameon=True, framealpha=0.9,
           fancybox=False, edgecolor="black", handlelength=1.4)

fig.tight_layout(rect=[0.05, 0, 1, 0.97])
fig.savefig(OUT_PDF, bbox_inches="tight")
print(f"Saved → {OUT_PDF}")
