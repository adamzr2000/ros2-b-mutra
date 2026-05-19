#!/usr/bin/env python3
"""
Prover attestation cycle breakdown — startup and continuous modes.

Segments (bottom to top):
  SHA-256 computation : prover total_lifecycle − e2e_blockchain
  Oracle phase        : SECaaS oracle total_lifecycle  (continuous only)
  Verifier phase      : Robot verifier (continuous) / SECaaS verifier (startup)
  Blockchain confirmation : residual

Error bar = IQR (p25/p75) or ±std across independent runs (prover total_lifecycle).
"""

import sys
from pathlib import Path

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.patches import ConnectionPatch
import pandas as pd
import seaborn as sns

INPUT_STARTUP    = "../data/attestation-times/_summary/durations_per_run_startup.csv"
INPUT_CONTINUOUS = "../data/attestation-times/_summary/durations_per_run_{VARIANT}.csv"
VARIANT    = "rr"
SSP_MS     = 20000
ITERQ      = 1
CPU_LIMIT  = 0.4
N_VALUES   = [4, 8, 16, 32, 64]

METRIC     = "median"  # "median" → median + IQR (p25/p75)  |  "mean" → mean ± std

FONT_SCALE = 1.7
BAR_WIDTH  = 0.42
HEADROOM   = 1.10
EPS        = 1e-4

C_ORACLE    = "#EF6C00"
C_VERIFIER  = "#00838F"
C_SHA256    = "#993333"
C_BC        = "#336699"

LABELS = {
    "oracle":      "Oracle phase",
    "verifier":    "Verifier phase",
    "sha256":      "SHA-256 computation",
    "blockchain":  "Blockchain confirmation",
}
COLORS = {"oracle": C_ORACLE, "verifier": C_VERIFIER, "sha256": C_SHA256, "blockchain": C_BC}
ORDER  = ["sha256", "oracle", "verifier", "blockchain"]


def _agg(df, n, group, role, metric):
    """Return (central, err_lo, err_hi) according to METRIC."""
    sub = df[
        (df["n_robots"] == n) &
        (df["participant_group"] == group) &
        (df["role"] == role) &
        (df["metric"] == metric)
    ]
    if sub.empty:
        return 0.0, 0.0, 0.0
    run_vals = sub.groupby("run")["run_mean_s"].mean()
    if METRIC == "mean":
        c   = float(run_vals.mean())
        err = float(run_vals.std(ddof=1))
        return c, err, err
    else:
        c   = float(run_vals.median())
        p25 = float(run_vals.quantile(0.25))
        p75 = float(run_vals.quantile(0.75))
        return c, c - p25, p75 - c


def _central(df, n, group, role, metric):
    return _agg(df, n, group, role, metric)[0]


def build_segments(df, n, mode):
    prover_total, err_lo, err_hi = _agg(df, n, "Robot", "prover", "total_lifecycle")
    e2e    = _central(df, n, "Robot", "prover", "e2e_blockchain")
    sha256 = max(0.0, prover_total - e2e)

    if mode == "continuous":
        oracle   = _central(df, n, "SECaaS", "oracle",   "total_lifecycle")
        verifier = _central(df, n, "Robot",  "verifier", "total_lifecycle")
        blockchain = max(0.0, prover_total - oracle - verifier - sha256)
        segs = [
            ("sha256",     sha256,     C_SHA256),
            ("oracle",     oracle,     C_ORACLE),
            ("verifier",   verifier,   C_VERIFIER),
            ("blockchain", blockchain, C_BC),
        ]
    else:  # startup — SECaaS is the verifier, no oracle phase
        verifier   = _central(df, n, "SECaaS", "verifier", "total_lifecycle")
        blockchain = max(0.0, prover_total - verifier - sha256)
        segs = [
            ("sha256",     sha256,     C_SHA256),
            ("verifier",   verifier,   C_VERIFIER),
            ("blockchain", blockchain, C_BC),
        ]

    return segs, err_lo, err_hi


def draw_stack(ax, x, segments, err_lo, err_hi):
    bottom = 0.0
    for _, val, color in segments:
        if val > EPS:
            ax.bar(x, val, bottom=bottom, width=BAR_WIDTH,
                   color=color, edgecolor="black", linewidth=0.8, zorder=3)
            bottom += val
    if err_lo > 0 or err_hi > 0:
        ax.errorbar(x, bottom, yerr=[[err_lo], [err_hi]], fmt="none",
                    color="black", capsize=2.0, linewidth=0.8, zorder=5)
    return bottom + err_hi


def generate_plot(sub, mode, n_values, script_dir):
    zoom_keys = ["sha256", "oracle", "verifier"] if mode == "continuous" else ["sha256", "verifier"]

    fig, (ax, ax_z) = plt.subplots(
        1, 2, figsize=(11, 4.5),
        gridspec_kw={"width_ratios": [3, 2]},
    )
    plt.subplots_adjust(wspace=0.30, left=0.07, right=0.97, top=0.93, bottom=0.13)

    all_segs = {}
    max_top  = 0.0
    used     = set()
    for i, n in enumerate(n_values):
        segs, err_lo, err_hi = build_segments(sub, n, mode)
        all_segs[n] = (segs, err_lo, err_hi)
        top = draw_stack(ax, i, segs, err_lo, err_hi)
        max_top = max(max_top, top)
        used.update(k for k, v, _ in segs if v > EPS)

    ax.set_xticks(range(len(n_values)))
    ax.set_xticklabels([str(n) for n in n_values])
    ax.set_xlabel("Number of robots (N)")
    ax.set_ylabel("Time (s)")
    ax.set_ylim(0, max_top * HEADROOM)
    ax.set_xlim(-0.6, len(n_values) - 0.4)
    ax.tick_params(axis="both", which="major", length=6, width=1.0, direction="out")
    ax.grid(axis="y", which="major", linestyle="-", linewidth=0.7, alpha=0.75)
    ax.set_axisbelow(True)

    handles = [
        mpatches.Patch(facecolor=COLORS[k], edgecolor="black", label=LABELS[k])
        for k in ORDER if k in used
    ]
    ax.legend(handles=handles, loc="upper left", frameon=True, framealpha=0.9,
              fancybox=False, edgecolor="black", borderpad=0.4, handlelength=1.2)

    # ── right panel: zoomed (ms) ──────────────────────────────────────────────
    max_top_z = 0.0
    for i, n in enumerate(n_values):
        segs, _, _ = all_segs[n]
        bottom = 0.0
        for key, val, color in segs:
            if key not in zoom_keys:
                continue
            val_ms = val * 1000.0
            if val_ms > EPS:
                ax_z.bar(i, val_ms, bottom=bottom, width=BAR_WIDTH,
                         color=color, edgecolor="black", linewidth=0.8, zorder=3)
                bottom += val_ms
        max_top_z = max(max_top_z, bottom)

    ax_z.set_xticks(range(len(n_values)))
    ax_z.set_xticklabels([str(n) for n in n_values])
    ax_z.set_xlabel("Number of robots (N)")
    ax_z.set_ylabel("Time (ms)")
    ax_z.set_ylim(0, max_top_z * HEADROOM)
    ax_z.set_xlim(-0.6, len(n_values) - 0.4)
    ax_z.tick_params(axis="both", which="major", length=6, width=1.0, direction="out")
    ax_z.grid(axis="y", which="major", linestyle="-", linewidth=0.7, alpha=0.75)
    ax_z.set_axisbelow(True)

    # ── zoom indicator ────────────────────────────────────────────────────────
    zoom_top_s = max_top_z / 1000.0
    top_frac   = zoom_top_s / (max_top * HEADROOM)

    xlim = ax.get_xlim()
    ax.add_patch(mpatches.FancyBboxPatch(
        (xlim[0], 0), xlim[1] - xlim[0], zoom_top_s,
        boxstyle="square,pad=0", linewidth=1.2,
        edgecolor="dimgray", facecolor="gray", alpha=0.10,
        linestyle="--", zorder=2,
    ))

    for (y_frac_l, y_frac_r), shrink_b in [((top_frac, 1.0), 180), ((0.0, 0.0), 30)]:
        fig.add_artist(ConnectionPatch(
            xyA=(1, y_frac_l), coordsA="axes fraction",
            xyB=(0, y_frac_r), coordsB="axes fraction",
            axesA=ax, axesB=ax_z,
            color="dimgray", linestyle="--", linewidth=0.9, zorder=10,
            shrinkA=4, shrinkB=shrink_b,
        ))

    out_path = script_dir / f"barplot_attestation_time_v5_{mode}.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] Saved: {out_path}")

    err_label = "±std" if METRIC == "mean" else "IQR(p25/p75)"
    print(f"\n=== Prover breakdown ({mode}, metric={METRIC}) ===")
    for n in n_values:
        segs, err_lo, err_hi = build_segments(sub, n, mode)
        total = sum(v for _, v, _ in segs)
        print(f"  N={n:>3}: "
              + "  ".join(f"{k}={v:.4f}s" for k, v, _ in segs)
              + f"  total={total:.4f}s  {err_label}=(-{err_lo:.4f}/+{err_hi:.4f})")


def main():
    script_dir = Path(__file__).parent.resolve()

    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)
    plt.rcParams.update({"font.family": "serif"})

    sources = {
        "startup":    (script_dir / INPUT_STARTUP).resolve(),
        "continuous": (script_dir / INPUT_CONTINUOUS.format(VARIANT=VARIANT)).resolve(),
    }

    for mode, csv_path in sources.items():
        if not csv_path.exists():
            print(f"[WARN] CSV not found, skipping {mode}: {csv_path}")
            continue

        df = pd.read_csv(csv_path)
        df = df[df["n_robots"].isin(N_VALUES)]

        if mode == "continuous":
            sub = df[
                (df["ssp_ms"]    == SSP_MS) &
                (df["iterq"]     == ITERQ) &
                (df["cpu_limit"] == CPU_LIMIT)
            ].copy()
        else:
            sub = df.copy()

        if sub.empty:
            print(f"[WARN] No data for mode='{mode}'")
            continue

        n_values = sorted(sub["n_robots"].unique().tolist())
        generate_plot(sub, mode, n_values, script_dir)


if __name__ == "__main__":
    main()
