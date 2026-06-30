#!/usr/bin/env python3
"""
Mission-impact figure for D-MUTRA swarm experiments.

Three scenarios (baseline N=9, unmitigated N=10, mitigated N=10):
  • baseline    – no attack; 4 robots patrol to 100% (32/32 WPs, 139 s)
  • unmitigated – tamper on robot3 at WP3; mission plateaus permanently at 84% (27/32)
  • mitigated   – tamper + D-MUTRA detection + whole-queue reassignment; 100% recovered (225 s)

The mitigated SSP sweep (10 / 20 / 30 s) yields near-identical mission curves
(detection ~64 s for all, because LCM(10,20,30)=60 s aligns with the attack time).
SSP 10 s is used as the representative mitigated curve.

X-axis: 0 → 230 s (5 s past the longest mitigated run), so the unmitigated
plateau is clearly visible at the right edge without padding dead space.
"""

from pathlib import Path
import glob
import json
import statistics as s

import matplotlib
matplotlib.use("Agg")
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).parent
DATA_DIR   = SCRIPT_DIR.parent / "data" / "mission"
OUT_PDF    = SCRIPT_DIR / "mission_impact.pdf"

# ── Style — mirrors barplot_attestation_time_v5_combined.py ──────────────────
FONT_SCALE = 1.8
sns.set_theme(
    context="paper",
    style="ticks",
    rc={"xtick.direction": "out", "ytick.direction": "out"},
    font_scale=FONT_SCALE,
)
plt.rcParams.update({"font.family": "serif", "pdf.fonttype": 42, "ps.fonttype": 42})

# ── Colors ────────────────────────────────────────────────────────────────────
# Baseline and mitigated reuse two colors from the existing palette C so
# figures feel like the same paper. Unmitigated uses red (semantic: danger/attack)
# rather than orange to clearly contrast with the recovery curve.
C_BASELINE    = "#4B9FCC"   # palette-C sha256 blue  → normal operation
C_UNMITIGATED = "#C44E52"   # red                    → attack / unrecovered failure
C_MITIGATED   = "#59c396"   # palette-C verifier green → recovery / success

ALPHA_FILL = 0.12
LW_MAIN    = 2.0
LW_ANNOT   = 1.2

# ── X-axis limit ─────────────────────────────────────────────────────────────
# 5 s past the longest mitigated run; the unmitigated plateau is visible at edge.
T_MAX = 230


# ── Helpers ───────────────────────────────────────────────────────────────────
def load_csvs(pattern, min_rows=50, min_max_pct=50.0):
    files = sorted(glob.glob(str(DATA_DIR / pattern)))
    if not files:
        raise FileNotFoundError(f"No files matched: {DATA_DIR / pattern}")
    dfs = []
    for f in files:
        df = pd.read_csv(f)
        if len(df) < min_rows or df["pct"].max() < min_max_pct:
            continue
        dfs.append(df)
    return dfs


def mean_curve(dfs, t_max=T_MAX, dt=0.5, t_col="t_rel_s", y_col="pct"):
    """Interpolate each run onto a shared grid; extend last value to t_max."""
    t_grid = np.arange(0, t_max + dt, dt)
    arr = np.array([
        np.interp(t_grid, df[t_col].values, df[y_col].values,
                  left=df[y_col].values[0], right=df[y_col].values[-1])
        for df in dfs
    ])
    return t_grid, arr.mean(axis=0), arr.std(axis=0)


def median_from_summaries(pattern, key):
    files = sorted(glob.glob(str(DATA_DIR / pattern)))
    vals  = [json.load(open(f))[key] for f in files]
    return s.median(vals)


# ── Load data ─────────────────────────────────────────────────────────────────
base_dfs  = load_csvs("mission-baseline-r*.csv",             min_max_pct=50.0)
unmit_dfs = load_csvs("mission-unmitigated-ssp10000-r*.csv", min_max_pct=50.0)
mit_dfs   = load_csvs("mission-mitigated-ssp10000-r*.csv",   min_max_pct=50.0)

t_b, y_b, s_b = mean_curve(base_dfs)
t_u, y_u, s_u = mean_curve(unmit_dfs)
t_m, y_m, s_m = mean_curve(mit_dfs)


# ── Plot ──────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(8, 5))

# -- Individual run traces (thin, semi-transparent) behind the mean
ALPHA_TRACE = 0.18
LW_TRACE    = 0.9
for df in base_dfs:
    tg = np.arange(0, T_MAX + 0.5, 0.5)
    yi = np.interp(tg, df["t_rel_s"].values, df["pct"].values,
                   left=df["pct"].values[0], right=df["pct"].values[-1])
    ax.plot(tg, yi, color=C_BASELINE,    lw=LW_TRACE, alpha=ALPHA_TRACE)
for df in unmit_dfs:
    tg = np.arange(0, T_MAX + 0.5, 0.5)
    yi = np.interp(tg, df["t_rel_s"].values, df["pct"].values,
                   left=df["pct"].values[0], right=df["pct"].values[-1])
    ax.plot(tg, yi, color=C_UNMITIGATED, lw=LW_TRACE, alpha=ALPHA_TRACE)
for df in mit_dfs:
    tg = np.arange(0, T_MAX + 0.5, 0.5)
    yi = np.interp(tg, df["t_rel_s"].values, df["pct"].values,
                   left=df["pct"].values[0], right=df["pct"].values[-1])
    ax.plot(tg, yi, color=C_MITIGATED,   lw=LW_TRACE, alpha=ALPHA_TRACE)

# -- Bold mean curves + per-waypoint markers (drawn on top)
STEP = 100.0 / 32   # 3.125 % per waypoint

for y_mean, color, marker, label in [
    (y_b, C_BASELINE,    "o", "Baseline (no attack)"),
    (y_u, C_UNMITIGATED, "^", "Unmitigated (attack, no recovery)"),
    (y_m, C_MITIGATED,   "D", "Mitigated (D-MUTRA, SSP 10 s)"),
]:
    t_grid = np.arange(0, T_MAX + 0.5, 0.5)
    n_steps = int(np.round(y_mean.max() / STEP))
    step_indices = []
    for k in range(1, n_steps + 1):
        idx = int(np.argmax(y_mean >= k * STEP - 1e-6))
        if idx > 0:
            step_indices.append(idx)
    ax.plot(t_grid, y_mean, color=color, lw=LW_MAIN, label=label,
            marker=marker, markevery=step_indices, ms=5, zorder=6)

# -- Axes
ax.set_xlim(0, T_MAX)
ax.set_ylim(-3, 110)

ax.set_yticks([0, 25, 50, 75, 100])

ax.set_xlabel("Time (s)")
ax.set_ylabel("Swarm mission completion (%)")
ax.legend(loc="lower right", framealpha=0.9, frameon=True, fancybox=False, edgecolor="black", handlelength=1.4)
for spine in ax.spines.values():
    spine.set_visible(True)

fig.tight_layout()
fig.savefig(OUT_PDF, bbox_inches="tight")
print(f"Saved → {OUT_PDF}")
