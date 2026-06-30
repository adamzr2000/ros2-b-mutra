#!/usr/bin/env python3
"""
Formation-error timeline — D-MUTRA formation experiment.

Single representative run (SSP=10 s, rep closest to group-mean IFE) showing
the full attack lifecycle, aligned at injection (t=0):

  • baseline (no attack)  : FE stays near zero throughout the mission.
  • mitigated (SSP=10 s)  : rises at injection, detected, reconfigures, recovers.
  • unmitigated           : rises at injection, never detected, never recovers.
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
DATA_SUBDIR = "formation-v4"
DATA_DIR    = SCRIPT_DIR.parent / "data" / DATA_SUBDIR
OUT_PDF     = SCRIPT_DIR / "formation_impact_fe.pdf"

FONT_SCALE = 2.0
sns.set_theme(context="paper", style="ticks",
              rc={"xtick.direction": "out", "ytick.direction": "out"},
              font_scale=FONT_SCALE)
plt.rcParams.update({"font.family": "serif", "pdf.fonttype": 42, "ps.fonttype": 42})

C_BASELINE    = "#888888"
C_UNMITIGATED = "#FF0000"
C_MITIGATED   = "#0000FF"
LW = 2.2

SSP_PICK = 10
T_LO, T_HI = 0.0, 30.0


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
    """Pick the rep whose key metric is closest to the group mean."""
    vals = [r[key] for r in reps if r.get(key) is not None]
    if not vals:
        return reps[0]
    mean = np.mean(vals)
    return min((r for r in reps if r.get(key) is not None),
               key=lambda r: abs(r[key] - mean))


def load_fe_aligned(rep, t_inject_override=None):
    """Load FE from CSV aligned so t=0 at injection, masked to [T_LO, T_HI]."""
    df = pd.read_csv(DATA_DIR / f"formation-{rep['run_tag']}.csv")
    t_inj = t_inject_override if t_inject_override is not None else rep["t_inject_s"]
    t  = df["t_rel_s"].values - t_inj
    fe = df["formation_error"].values
    mask = (t >= T_LO) & (t <= T_HI)
    return t[mask], fe[mask]


metrics   = load_metrics()
mit_rep  = representative(metrics[("mitigated", SSP_PICK)])
unm_rep  = representative(metrics[("unmitigated", 10)])
base_rep = min(metrics[("baseline", None)], key=lambda r: r["run_tag"])

t_mit,  fe_mit  = load_fe_aligned(mit_rep)
t_unm,  fe_unm  = load_fe_aligned(unm_rep)
t_base, fe_base = load_fe_aligned(base_rep, t_inject_override=mit_rep["t_inject_s"])

# Detection time relative to injection
t_det_rel = mit_rep["ife_window_s"]

# ── Figure ────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(8.2, 4.8))

ax.plot(t_base, fe_base, color=C_BASELINE,    lw=2.0, ls="--", zorder=4,
        label="Baseline")
ax.plot(t_unm,  fe_unm,  color=C_UNMITIGATED, lw=LW,  zorder=5,
        label="Unmitigated")
ax.plot(t_mit,  fe_mit,  color=C_MITIGATED,   lw=LW,  zorder=6,
        label="Mitigated")

# ── Annotation at the FE peak of the mitigated curve ─────────────────────────
peak_idx = int(np.argmax(fe_mit))
t_peak   = float(t_mit[peak_idx])
fe_peak  = float(fe_mit[peak_idx])

fs_ann = plt.rcParams.get("axes.labelsize", 10) * 0.8
ax.annotate(
    "Attack detected;\Compromised robot isolated",
    xy=(16, 0.2), xycoords="data",
    xytext=(10, 0.7), textcoords="data",
    va="bottom", ha="center", fontsize=fs_ann,
    arrowprops=dict(arrowstyle="-|>", color="black", lw=0.9,
                    mutation_scale=8, connectionstyle="arc3,rad=-0.3"),
    bbox=dict(boxstyle="round,pad=0.35", facecolor="white",
              edgecolor="black", linewidth=0.8, alpha=0.92),
    zorder=10,
)

ax.set_xlim(T_LO, T_HI)
ax.set_ylim(0, 1.40)
ax.set_xlabel("Time since attack injection (s)")
ax.set_ylabel("Swarm Formation Error (m)")
ax.legend(loc="upper right", frameon=True, framealpha=0.9, fancybox=False,
          edgecolor="black", handlelength=1.6, fontsize="small")

ax.spines["top"].set_visible(True)
ax.spines["right"].set_visible(True)

ax.grid(axis="both", which="major", linestyle="-", linewidth=0.7, alpha=0.75)

fig.tight_layout()
fig.savefig(OUT_PDF, bbox_inches="tight")
print(f"Saved → {OUT_PDF}")

checkpoints = list(range(int(T_LO), int(T_HI) + 1, 2))   # every 2 s
print(f"\n── FE values shown in plot (t={T_LO:.0f}–{T_HI:.0f}s, every 2s) ────")
print(f"  {'t(s)':<6}  {'Baseline(mm)':>14}  {'Unmitigated(m)':>16}  {'Mitigated(m)':>14}")
print(f"  {'-'*56}")
for tp in checkpoints:
    fb = float(np.interp(tp, t_base, fe_base)) * 1000
    fu = float(np.interp(tp, t_unm,  fe_unm))
    fm = float(np.interp(tp, t_mit,  fe_mit))
    det_marker = f"  ← det@{t_det_rel:.1f}s" if abs(tp - t_det_rel) < 1.5 else ""
    print(f"  {tp:<6}  {fb:>14.1f}  {fu:>16.4f}  {fm:>14.4f}{det_marker}")
print(f"\n  peak mitigated: {fe_peak:.4f} m @ t={t_peak:.1f}s")
