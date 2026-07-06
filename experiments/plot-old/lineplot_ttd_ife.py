#!/usr/bin/env python3
"""
TTD (bars) + IFE (line) combined plot — D-MUTRA mitigated attack scenarios.

Left y-axis  (green, bars):  Time to Detection from tamper-detection CSVs.
Right y-axis (red, line):    Cumulative Formation Error from formation-v4.
X-axis: SSP in seconds, low→high (5 → 60 s).
Axis labels and tick values coloured to match their metric.
"""

import csv, glob, json, re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

import numpy as np
import seaborn as sns
import plot_style

SCRIPT_DIR    = Path(__file__).parent
TAMPER_DIR    = SCRIPT_DIR.parent / "data" / "tamper-detection" / "results" / "state_publisher"
FORMATION_DIR = SCRIPT_DIR.parent / "data" / "formation-v4"
OUT_PDF       = SCRIPT_DIR / "lineplot_ttd_ife.pdf"

FONT_SCALE = 1.8

C_TTD      = "#2E72B8"   # blue       — TTD bars border (left axis)
C_TTD_FILL = "#B9CDE5"   # light blue — TTD bar fill
C_IFE      = "#C97A2B"   # orange     — IFE line (right axis)

BAR_W = 0.5

SSP_S = [60, 30, 20, 10, 5]   # high → low

plot_style.apply(FONT_SCALE)


def _darken(color, f=0.65):
    return tuple(c * f for c in mcolors.to_rgb(color))


# ── Data loading ───────────────────────────────────────────────────────────────
def load_ttd():
    raw = {}
    for f in sorted(TAMPER_DIR.glob("SSP*ms-batch*.csv")):
        with f.open() as fh:
            for row in csv.DictReader(fh):
                if row["detected"] == "1" and row["latency_s"]:
                    s = int(row["ssp_ms"]) // 1000
                    if s in SSP_S:
                        raw.setdefault(s, []).append(float(row["latency_s"]))
    return raw


def load_ife():
    raw = {}
    for f in glob.glob(str(FORMATION_DIR / "metrics-mitigated-ssp*.json")):
        d = json.load(open(f))
        m = re.match(r"mitigated-ssp(\d+)-r\d+", d["run_tag"])
        if not m:
            continue
        s = int(m.group(1)) // 1000
        if s in SSP_S and d.get("ife_compromise") is not None:
            raw.setdefault(s, []).append(d["ife_compromise"])
    return raw


ttd_raw = load_ttd()
ife_raw = load_ife()

x      = np.arange(len(SSP_S), dtype=float)
labels = [str(s) for s in SSP_S]

ife_means = np.array([np.mean(ife_raw.get(s, [np.nan])) for s in SSP_S])
ife_sds   = np.array([np.std(ife_raw.get(s,  [np.nan])) for s in SSP_S])
ttd_means = np.array([np.mean(ttd_raw.get(s, [np.nan])) for s in SSP_S])
ttd_mins  = np.array([min(ttd_raw.get(s,     [np.nan])) for s in SSP_S])
ttd_maxs  = np.array([max(ttd_raw.get(s,     [np.nan])) for s in SSP_S])

# ── Figure ─────────────────────────────────────────────────────────────────────
fig, ax1 = plt.subplots(figsize=(7.5, 4.8))
ax2 = ax1.twinx()

# ── TTD bars — left, blue ────────────────────────────────────────────────────
ax1.bar(x, ttd_means, width=BAR_W, color=C_TTD_FILL,
        edgecolor=C_TTD, linewidth=1.0, zorder=3)
ax1.errorbar(x, ttd_means, yerr=[ttd_means - ttd_mins, ttd_maxs - ttd_means], fmt="none",
             color="black", elinewidth=1.3, capsize=4, capthick=1.3, zorder=4)

# ── IFE line — right, red ────────────────────────────────────────────────────
ax2.plot(x, ife_means, color=C_IFE, linewidth=2.0, zorder=4)
ax2.plot(x, ife_means, "o",
         color=C_IFE, markersize=8,
         markerfacecolor=C_IFE, markeredgecolor=C_IFE, 
         zorder=6)

# ── Axis colouring ─────────────────────────────────────────────────────────────

# Main axis (ax1) - Handles Left (Blue), Top (Black), Bottom (Black)
ax1.set_ylabel("Time to Detection (s)", color=C_TTD)
ax1.tick_params(axis="y", colors=C_TTD, which="both", length=6, width=1.0)

ax1.spines["left"].set_color(C_TTD)
ax1.spines["left"].set_linewidth(1.2)
ax1.spines["bottom"].set_linewidth(1.2) # Match thickness to sides
ax1.spines["top"].set_linewidth(1.2)    # Match thickness to sides
ax1.spines["right"].set_visible(False)  # Hide so ax2's red spine renders cleanly

# Twin axis (ax2) - Handles Right (Red)
ax2.set_ylabel("Cum. Formation Error (m·s)", color=C_IFE)
ax2.tick_params(axis="y", colors=C_IFE, which="both", length=6, width=1.0)

ax2.spines["right"].set_color(C_IFE)
ax2.spines["right"].set_linewidth(1.2)
ax2.spines["bottom"].set_visible(False) # Prevent double-drawing over ax1
ax2.spines["top"].set_visible(False)    # Prevent double-drawing over ax1
ax2.spines["left"].set_visible(False)   # Prevent double-drawing over ax1

ax1.set_xlabel("SSP (s)")
ax1.set_xticks(x)
ax1.set_xticklabels(labels)
ax1.set_xlim(-0.55, x[-1] + 0.55)

# ── Axis Limits ────────────────────────────────────────────────────────────────
ax1.set_ylim(0, 75)   # TTD limits (bars stretch higher to fill space)
ax2.set_ylim(0, 35)   # IFE limits (line compresses lower below bars)

ax1.grid(axis="y", **plot_style.GRID_KW)
ax1.set_axisbelow(True)

plt.subplots_adjust(left=0.11, right=0.89, top=0.97, bottom=0.13)
fig.savefig(OUT_PDF, bbox_inches="tight")
print(f"Saved → {OUT_PDF}")
print(f"\n── TTD+IFE plot values ──────────────────────────────────")
print(f"  {'SSP(s)':<8} {'TTD mean':>10} {'TTD min':>9} {'TTD max':>9} {'IFE mean':>10} {'IFE sd':>8}")
print(f"  {'-'*58}")
for s, tm, tmin, tmax, im, isd in zip(
        SSP_S, ttd_means, ttd_mins, ttd_maxs, ife_means, ife_sds):
    print(f"  {s:<8} {tm:>10.2f} {tmin:>9.2f} {tmax:>9.2f} {im:>10.3f} {isd:>8.3f}")