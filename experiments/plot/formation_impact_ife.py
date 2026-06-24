#!/usr/bin/env python3
"""IFE per attack scenario — D-MUTRA formation experiment (bar chart)."""

from pathlib import Path
import glob, json, re
import matplotlib
matplotlib.use("Agg")
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
from matplotlib.transforms import blended_transform_factory
import numpy as np
import pandas as pd
import seaborn as sns


def _darken(color, factor=0.65):
    return tuple(c * factor for c in mcolors.to_rgb(color))

SCRIPT_DIR  = Path(__file__).parent
DATA_SUBDIR = "formation-v4"
DATA_DIR    = SCRIPT_DIR.parent / "data" / DATA_SUBDIR
OUT_PDF     = SCRIPT_DIR / "formation_impact_ife.pdf"

FONT_SCALE = 1.8
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


metrics = load_metrics()

# ── Baseline FE (verification only — healthy FE≈6 mm, negligible vs attack) ──
T_INJECT_REF = np.mean([r["t_inject_s"]
                         for r in metrics[("unmitigated", 10)]
                         if r.get("t_inject_s") is not None])
baseline_ifes = []
for rep in metrics.get(("baseline", None), []):
    df = pd.read_csv(DATA_DIR / f"formation-{rep['run_tag']}.csv")
    mask = df["t_rel_s"] >= T_INJECT_REF
    t  = df.loc[mask, "t_rel_s"].values
    fe = df.loc[mask, "formation_error"].values
    if len(t) > 1:
        baseline_ifes.append(np.trapz(fe, t))
baseline_mean = np.mean(baseline_ifes)
baseline_sd   = np.std(baseline_ifes)
print(f"  Baseline reference  mean={baseline_mean:.3f}  sd={baseline_sd:.3f}  n={len(baseline_ifes)}  (t≥{T_INJECT_REF:.1f} s)")

# ── Scenarios: Unmitigated first, then mitigated from largest to smallest SSP ─
SCENARIOS = [
    # (x-label,          metrics key,          bar color)
    # Color palette darkens with increasing SSP; extend with "#00441b" for 120 s
    ("Unmitigated\nattack", ("unmitigated", 10), "#C44E52"),
    # ("120 s",            ("mitigated",  120),  "#00441b"),  # ← add when data ready
    ("60 s",               ("mitigated",   60),  "#006d2c"),
    ("30 s",               ("mitigated",   30),  "#31a354"),
    ("20 s",               ("mitigated",   20),  "#74c476"),
    ("10 s",               ("mitigated",   10),  "#bae4b3"),
]

labels, means, sds, colors = [], [], [], []
for lbl, key, color in SCENARIOS:
    vals = [r["ife_compromise"] for r in metrics.get(key, [])
            if r.get("ife_compromise") is not None]
    labels.append(lbl)
    means.append(np.mean(vals))
    sds.append(np.std(vals))
    colors.append(color)
    print(f"  {lbl.replace(chr(10),' '):12s}  mean={means[-1]:.3f}  sd={sds[-1]:.3f}  n={len(vals)}")

means = np.array(means)
sds   = np.array(sds)

# ── Bar chart ─────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(7.5, 5.2))

x = np.arange(len(labels))
ax.bar(x, means, color=colors, width=0.55, zorder=3,
       edgecolor=[_darken(c) for c in colors], linewidth=0.8)

# SD error bars
ax.errorbar(x, means, yerr=sds, fmt="none",
            color="black", elinewidth=1.4, capsize=5, capthick=1.4, zorder=4)

um_mean = means[0]

# % reduction from unmitigated annotated above each mitigated bar
for i in range(1, len(means)):
    pct = (um_mean - means[i]) / um_mean * 100
    ax.text(x[i], means[i] + sds[i] + 1.5,
            f"−{pct:.0f}%", ha="center", va="bottom",
            fontsize=plt.rcParams["font.size"] * 0.8, color="black")

ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.set_ylabel("Cumulated Formation Error (m·s)")
ax.set_ylim(0, 80)
ax.set_xlim(-0.55, len(labels) - 0.45)

ax.grid(axis="y", which="major", linestyle="-", linewidth=0.7, alpha=0.75)
ax.set_axisbelow(True)

# Bracket below the x-axis grouping the five mitigated (SSP) bars.
trans = blended_transform_factory(ax.transData, ax.transAxes)
x_l, x_r = x[1] - 0.32, x[4] + 0.32
y_line, y_tick = -0.13, -0.10
ax.plot([x_l, x_r], [y_line, y_line], color="black", lw=1.0,
        clip_on=False, transform=trans)
for xp in [x_l, x_r]:
    ax.plot([xp, xp], [y_line, y_tick], color="black", lw=1.0,
            clip_on=False, transform=trans)
ax.text((x_l + x_r) / 2, y_line - 0.02,
        "Mitigated attack under different SSP",
        ha="center", va="top", fontsize=plt.rcParams["font.size"] * 0.85,
        color="black", clip_on=False, transform=trans)

fig.tight_layout()
fig.savefig(OUT_PDF, bbox_inches="tight")
print(f"Saved → {OUT_PDF}")
