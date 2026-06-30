#!/usr/bin/env python3
"""
Swarm centroid tracking plot — D-MUTRA formation experiment.

2D (x, y) path of the swarm centroid for three scenarios vs the ideal
straight-line trajectory (start → goal):
  • Baseline (no attack)      — centroid follows the ideal path closely
  • Unmitigated attack        — centroid stalls after injection, mission fails
  • Mitigated attack (SSP=10s) — centroid deviates, then resumes after exclusion

One representative run per scenario (closest to group-mean IFE).
"""

from pathlib import Path
import glob, json, re
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

SCRIPT_DIR  = Path(__file__).parent
DATA_SUBDIR = "formation"   # change to "formation-v2" for cohesion-drift experiment data
DATA_DIR    = SCRIPT_DIR.parent / "data" / DATA_SUBDIR
OUT_PDF     = SCRIPT_DIR / "formation_impact_centroid.pdf"

FONT_SCALE    = 1.8
C_BASELINE    = "#888888"
C_MITIGATED   = "#31a354"
C_UNMITIGATED = "#C44E52"
SSP_PICK      = 10
GOAL          = (30.0, 0.0)

sns.set_theme(context="paper", style="ticks",
              rc={"xtick.direction": "out", "ytick.direction": "out"},
              font_scale=FONT_SCALE)
plt.rcParams.update({"font.family": "serif", "pdf.fonttype": 42, "ps.fonttype": 42})


def scenario_key(tag):
    m = re.match(r"(baseline|unmitigated|mitigated)(?:-ssp(\d+))?-r(\d+)", tag)
    return m.group(1), (int(m.group(2)) // 1000 if m.group(2) else None)


def load_metrics():
    out = {}
    for f in glob.glob(str(DATA_DIR / "metrics-*.json")):
        d = json.load(open(f))
        out.setdefault(scenario_key(d["run_tag"]), []).append(d)
    return out


def representative(reps, key="ife_compromise"):
    vals = [r[key] for r in reps if r.get(key) is not None]
    if not vals:
        return reps[0]
    mean = np.mean(vals)
    return min((r for r in reps if r.get(key) is not None),
               key=lambda r: abs(r[key] - mean))


def load_csv(tag):
    return pd.read_csv(DATA_DIR / f"formation-{tag}.csv")


def centroid_at(df, t_s):
    idx = (df["t_rel_s"] - t_s).abs().argmin()
    return float(df["centroid_x"].iloc[idx]), float(df["centroid_y"].iloc[idx])


# ── Load representative runs ──────────────────────────────────────────────────
metrics  = load_metrics()
mit_rep  = representative(metrics[("mitigated",   SSP_PICK)])
unm_rep  = representative(metrics[("unmitigated", SSP_PICK)])
base_rep = min(metrics[("baseline", None)], key=lambda r: r["run_tag"])

df_mit  = load_csv(mit_rep["run_tag"])
df_unm  = load_csv(unm_rep["run_tag"])
df_base = load_csv(base_rep["run_tag"])

# ── Figure ────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(8.0, 5.2))

# Ideal trajectory reference line (start → goal)
ax.plot([0, GOAL[0]], [0, GOAL[1]], color="black", lw=1.0,
        ls="--", alpha=0.65, zorder=1, label="Ideal trajectory")

# Centroid paths
ax.plot(df_base["centroid_x"], df_base["centroid_y"],
        color=C_BASELINE, lw=1.8, ls="--", zorder=3,
        label="Baseline (no attack)")
ax.plot(df_unm["centroid_x"], df_unm["centroid_y"],
        color=C_UNMITIGATED, lw=2.2, zorder=4,
        label="Unmitigated attack")
ax.plot(df_mit["centroid_x"], df_mit["centroid_y"],
        color=C_MITIGATED, lw=2.2, zorder=5,
        label=f"Mitigated attack (SSP = {SSP_PICK} s)")

# Start marker
ax.plot(0, 0, "o", color="black", ms=7, zorder=8, markeredgewidth=0.5)

# Goal marker
ax.plot(GOAL[0], GOAL[1], "*", color="black", ms=11, zorder=8,
        markeredgewidth=0.5, label="Goal")

# Injection markers on unmitigated and mitigated centroid paths
for rep, df in [(unm_rep, df_unm), (mit_rep, df_mit)]:
    t_inj = rep.get("t_inject_s")
    if t_inj is not None:
        cx, cy = centroid_at(df, t_inj)
        ax.plot(cx, cy, "^", color="black", ms=8, zorder=9,
                markeredgewidth=0.5)

# Detection / exclusion marker on mitigated path (just before centroid jump)
t_det = mit_rep.get("t_detect_s")
if t_det is not None:
    cx, cy = centroid_at(df_mit, t_det)
    ax.plot(cx, cy, "D", color="black", ms=7, zorder=9,
            markeredgewidth=0.5)

# Proxy legend entries for event markers
from matplotlib.lines import Line2D
ax.add_artist(ax.legend(loc="upper left", frameon=True, framealpha=0.9,
                         fancybox=False, edgecolor="black", handlelength=1.6))

event_handles = [
    Line2D([0], [0], marker="^", color="black", ms=8, lw=0,
           markeredgewidth=0.5, label="Attack injected"),
    Line2D([0], [0], marker="D", color="black", ms=7, lw=0,
           markeredgewidth=0.5, label="Attack detected"),
    Line2D([0], [0], marker="o", color="black", ms=7, lw=0,
           markeredgewidth=0.5, label="Mission start"),
]
ax.legend(handles=event_handles, loc="center left",
          frameon=True, framealpha=0.9, fancybox=False,
          edgecolor="black", handlelength=0.8, handletextpad=0.5)

ax.set_xlim(-1, 32)
ax.set_ylim(-1.4, 1.4)
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.grid(axis="both", which="major", linestyle="-",
        linewidth=0.7, alpha=0.75)
ax.spines["top"].set_visible(True)
ax.spines["right"].set_visible(True)

fig.tight_layout()
fig.savefig(OUT_PDF, bbox_inches="tight")
print(f"Saved → {OUT_PDF}")
print(f"  baseline:    {base_rep['run_tag']}")
print(f"  unmitigated: {unm_rep['run_tag']}  final centroid x={df_unm['centroid_x'].iloc[-1]:.2f} m  ({unm_rep['progress_pct']:.1f}% of goal)")
print(f"  mitigated:   {mit_rep['run_tag']}  final centroid x={df_mit['centroid_x'].iloc[-1]:.2f} m  ({mit_rep['progress_pct']:.1f}% of goal)")
