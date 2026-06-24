#!/usr/bin/env python3
"""
Formation Error over time — smooth gradient IFE fill (D-MUTRA formation experiment).

The shaded area under the FE curve uses a continuous green→yellow→red gradient
along the time axis, encoding IFE accumulation intuitively:
  green  = early detection → low cumulative damage
  red    = late / no detection → high cumulative damage

SSP detection times are annotated on the colorbar.
"""

from pathlib import Path
import glob, json, re
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.colors import Normalize
from matplotlib.lines import Line2D
import numpy as np
import pandas as pd
import seaborn as sns

SCRIPT_DIR  = Path(__file__).parent
DATA_SUBDIR = "formation-v4"
DATA_DIR    = SCRIPT_DIR.parent / "data" / DATA_SUBDIR
OUT_PDF     = SCRIPT_DIR / "formation_impact.pdf"

FONT_SCALE = 1.6
sns.set_theme(context="paper", style="ticks",
              rc={"xtick.direction": "out", "ytick.direction": "out"},
              font_scale=FONT_SCALE)
plt.rcParams.update({"font.family": "serif", "pdf.fonttype": 42, "ps.fonttype": 42})

T_LO, T_HI = 0.0, 80.0      # seconds relative to injection

C_BASELINE = "#888888"
CMAP       = cm.RdYlGn_r     # green (low t, fast detect) → red (high t, no detect)

SSPS = [10, 20, 30, 60]      # SSP values in seconds


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


def representative(reps, key="ife_compromise"):
    vals = [r[key] for r in reps if r.get(key) is not None]
    if not vals:
        return reps[0]
    mean = np.mean(vals)
    return min((r for r in reps if r.get(key) is not None),
               key=lambda r: abs(r[key] - mean))


def load_fe_aligned(rep, t_inject_override=None):
    df = pd.read_csv(DATA_DIR / f"formation-{rep['run_tag']}.csv")
    t_inj = t_inject_override if t_inject_override is not None else rep["t_inject_s"]
    t  = df["t_rel_s"].values - t_inj
    fe = df["formation_error"].values
    mask = (t >= T_LO) & (t <= T_HI)
    return t[mask], fe[mask]


# ── Load data ─────────────────────────────────────────────────────────────────
metrics = load_metrics()

unm_rep       = representative(metrics[("unmitigated", 10)])
t_unm, fe_unm = load_fe_aligned(unm_rep)
unm_ife       = np.mean([r["ife_compromise"] for r in metrics[("unmitigated", 10)]
                          if r.get("ife_compromise")])

base_rep         = min(metrics[("baseline", None)], key=lambda r: r["run_tag"])
_, fe_base       = load_fe_aligned(base_rep, t_inject_override=unm_rep["t_inject_s"])
baseline_level   = float(np.mean(fe_base))

mit_data = {}
for ssp in SSPS:
    reps = metrics.get(("mitigated", ssp), [])
    mit_data[ssp] = {
        "ife":   float(np.mean([r["ife_compromise"] for r in reps if r.get("ife_compromise")])),
        "t_det": float(np.mean([r["ife_window_s"]   for r in reps if r.get("ife_window_s")])),
    }

# ── Figure ────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(8.5, 5.0))

norm = Normalize(vmin=T_LO, vmax=T_HI)

# Gradient fill: one strip per sample pair — smooth colour along x
for i in range(len(t_unm) - 1):
    t_mid = (t_unm[i] + t_unm[i + 1]) / 2
    ax.fill_between(t_unm[i:i + 2], 0, fe_unm[i:i + 2],
                    color=CMAP(norm(t_mid)), linewidth=0, zorder=2)

# Thin FE curve on top for shape clarity
ax.plot(t_unm, fe_unm, color="white",   lw=2.2, zorder=3)
ax.plot(t_unm, fe_unm, color="#222222", lw=1.1, zorder=4)

# Baseline
ax.axhline(baseline_level, color=C_BASELINE, lw=1.8, ls="--", zorder=5)

# ── Colorbar with SSP detection annotations ───────────────────────────────────
sm = cm.ScalarMappable(cmap=CMAP, norm=norm)
sm.set_array([])
cbar = fig.colorbar(sm, ax=ax, pad=0.02, shrink=0.88, aspect=22)
cbar.set_label("Time since injection (s)", labelpad=8)

fs_cb = plt.rcParams["font.size"] * 0.78
for ssp in SSPS:
    t_det = mit_data[ssp]["t_det"]
    y     = norm(t_det)
    cbar.ax.axhline(y, color="white", lw=1.5, xmin=0.0, xmax=0.55)
    cbar.ax.text(0.58, y, f"SSP={ssp}s",
                 va="center", ha="left",
                 fontsize=fs_cb, color="white",
                 transform=cbar.ax.transAxes,
                 fontweight="bold")

# "Unmitigated" label near top of colorbar
cbar.ax.text(0.58, norm(T_HI) - 0.04, "Unmitigated",
             va="top", ha="left",
             fontsize=fs_cb, color="white",
             transform=cbar.ax.transAxes,
             fontweight="bold")

# ── Axes ──────────────────────────────────────────────────────────────────────
ax.set_xlim(T_LO, T_HI)
yhi = fe_unm.max() * 1.22
ax.set_ylim(0, yhi)

ax.set_xlabel("Time since attack injection (s)")
ax.set_ylabel("Formation error FE (m)")
ax.set_title("(a)  Formation error over time — shaded area = IFE", pad=6)

ax.spines["top"].set_visible(True)
ax.spines["right"].set_visible(True)
ax.grid(axis="both", which="major", linestyle="-", linewidth=0.6, alpha=0.4, zorder=0)

# ── Legend ────────────────────────────────────────────────────────────────────
legend_handles = [
    Line2D([0], [0], color=C_BASELINE, lw=1.8, ls="--", label="Baseline (no attack)"),
    Line2D([0], [0], color="#222222",  lw=1.1,           label="Formation error (compromised)"),
]
ax.legend(handles=legend_handles,
          loc="upper right", frameon=True, framealpha=0.92,
          fancybox=False, edgecolor="black",
          handlelength=1.4,
          fontsize=plt.rcParams["font.size"] * 0.78)

fig.tight_layout()
fig.savefig(OUT_PDF, bbox_inches="tight")
print(f"Saved → {OUT_PDF}")
print(f"\nIFE summary:")
for ssp in SSPS:
    d = mit_data[ssp]
    print(f"  SSP={ssp:2d}s  IFE={d['ife']:.2f} m·s  det@{d['t_det']:.1f}s")
print(f"  Unmitigated  IFE={unm_ife:.2f} m·s")
print(f"  Baseline FE level: {baseline_level*1000:.1f} mm")
