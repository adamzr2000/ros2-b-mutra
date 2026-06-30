#!/usr/bin/env python3
"""
Prover attestation cycle breakdown — startup (top) and continuous (bottom).
Each row has a main panel (seconds) and a zoomed panel (milliseconds).
Rows share a common x-axis (N values).
"""

from pathlib import Path
import matplotlib.colors as mcolors
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.patches import ConnectionPatch
import pandas as pd
import seaborn as sns

INPUT_STARTUP    = "../data/attestation-times/_summary/durations_per_run_startup_v1.csv"
INPUT_CONTINUOUS = "../data/attestation-times/_summary/durations_per_run_{VARIANT}.csv"
VARIANT    = "lv"
SSP_MS     = 20000
ITERQ      = 1
CPU_LIMIT  = None   # None = no cap (cpuNC)
N_VALUES   = [4, 8, 16, 32, 64]

METRIC     = "mean"  # "median" → median + IQR  |  "mean" → mean ± std

FONT_SCALE = 2.2
BAR_WIDTH  = 0.42
HEADROOM   = 1.10
EPS        = 1e-4

PALETTE = "D"

_PALETTES = {
    "A": dict(sha256="#336699", oracle="#9A7B3F", verifier="#228888", blockchain="#993333"),
    "B": dict(sha256="#4C72B0", oracle="#55A868", verifier="#C8A83E", blockchain="#C44E52"),
    "C": dict(sha256="#4B9FCC", oracle="#59c396", verifier="#f8eb4f", blockchain="#6a5d99"),
    "D": dict(sha256="#B3B3B3", oracle="#FFB3B3", verifier="#DCC7B8", blockchain="#B3B3FF"),
}

# Explicit edge colors (pgfplots-style fill+edge pairs). Palettes not listed fall back to _darken().
_PALETTE_EDGES = {
    "D": dict(sha256="#000000", oracle="#FF0000", verifier="#8B4513", blockchain="#0000FF"),
}

C_SHA256   = _PALETTES[PALETTE]["sha256"]
C_ORACLE   = _PALETTES[PALETTE]["oracle"]
C_VERIFIER = _PALETTES[PALETTE]["verifier"]
C_BC       = _PALETTES[PALETTE]["blockchain"]

LABELS = {
    "sha256":     "SHA-256 computation",
    "oracle":     "Oracle retrieval",
    "verifier":   "Verification",
    "blockchain": "Blockchain confirmation",
}
COLORS = {"sha256": C_SHA256, "oracle": C_ORACLE, "verifier": C_VERIFIER, "blockchain": C_BC}

ORDER  = ["sha256", "oracle", "verifier", "blockchain"]


def _darken(color, factor=0.65):
    return tuple(c * factor for c in mcolors.to_rgb(color))


_explicit_edges = _PALETTE_EDGES.get(PALETTE, {})
EDGE_COLORS = {k: _explicit_edges.get(k) or _darken(COLORS[k]) for k in ORDER}


def _agg(df, n, group, role, metric):
    sub = df[
        (df["n_robots"] == n) &
        (df["participant_group"] == group) &
        (df["role"] == role) &
        (df["metric"] == metric)
    ]
    if sub.empty:
        return 0.0, 0.0, 0.0
    value_col = "run_mean_s" if METRIC == "mean" else "run_median_s"
    run_vals = sub.groupby("run")[value_col].mean()
    if METRIC == "mean":
        c   = float(run_vals.mean())
        err = float(run_vals.std(ddof=1))
        return c, err, err
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
        oracle     = _central(df, n, "SECaaS", "oracle",   "total_lifecycle")
        verifier   = _central(df, n, "Robot",  "verifier", "total_lifecycle")
        blockchain = max(0.0, prover_total - oracle - verifier - sha256)
        segs = [
            ("sha256",     sha256,     C_SHA256, EDGE_COLORS["sha256"]),
            ("oracle",     oracle,     C_ORACLE, EDGE_COLORS["oracle"]),
            ("verifier",   verifier,   C_VERIFIER, EDGE_COLORS["verifier"]),
            ("blockchain", blockchain, C_BC,     EDGE_COLORS["blockchain"]),
        ]
    else:
        verifier   = _central(df, n, "SECaaS", "verifier", "total_lifecycle")
        blockchain = max(0.0, prover_total - verifier - sha256)
        segs = [
            ("sha256",     sha256,     C_SHA256, EDGE_COLORS["sha256"]),
            ("verifier",   verifier,   C_VERIFIER, EDGE_COLORS["verifier"]),
            ("blockchain", blockchain, C_BC,     EDGE_COLORS["blockchain"]),
        ]
    return segs, err_lo, err_hi


def draw_stack(ax, x, segments, err_lo, err_hi):
    bottom = 0.0
    for _, val, color, edge in segments:
        if val > EPS:
            ax.bar(x, val, bottom=bottom, width=BAR_WIDTH,
                   color=color, edgecolor=edge, linewidth=1.0, zorder=3)
            bottom += val
    if err_lo > 0 or err_hi > 0:
        ax.errorbar(x, bottom, yerr=[[err_lo], [err_hi]], fmt="none",
                    color="black", capsize=4.0, linewidth=1.4, zorder=5)
    return bottom + err_hi


def draw_row(ax, ax_z, sub, mode, n_values, fig, is_bottom, panel_label):
    zoom_keys = ["sha256", "oracle", "verifier"] if mode == "continuous" else ["sha256", "verifier"]

    # ── main panel ────────────────────────────────────────────────────────────
    all_segs = {}
    max_top  = 0.0
    used     = set()
    for i, n in enumerate(n_values):
        segs, err_lo, err_hi = build_segments(sub, n, mode)
        all_segs[n] = (segs, err_lo, err_hi)
        sec_parts = [f"{key}={val:.6f}s" for key, val, *_ in segs]
        total_s = sum(val for _, val, *_ in segs)
        sec_parts.append(f"total={total_s:.6f}s")
        sec_parts.append(f"err=-{err_lo:.6f}s/+{err_hi:.6f}s")
        print(f"[DEBUG] {panel_label} | mode={mode} | N={n} | seconds: " + ", ".join(sec_parts))
        top = draw_stack(ax, i, segs, err_lo, err_hi)
        max_top = max(max_top, top)
        used.update(k for k, v, *_ in segs if v > EPS)

    ax.set_xticks(range(len(n_values)))
    ax.set_xlim(-0.6, len(n_values) - 0.4)
    ax.set_ylim(0, max_top * HEADROOM)
    ax.set_ylabel("Time (s)")
    ax.tick_params(axis="both", which="major", length=6, width=1.0, direction="out")
    ax.grid(axis="y", which="major", linestyle="-", linewidth=0.7, alpha=0.75)
    ax.set_axisbelow(True)

    if is_bottom:
        ax.set_xticklabels([str(n) for n in n_values])
        ax.set_xlabel("Number of robots (N)")
    else:
        ax.tick_params(labelbottom=False)

    ax.text(0.02, 0.97, panel_label, transform=ax.transAxes,
            va="top", ha="left", fontsize=plt.rcParams.get("axes.titlesize", 10))

    # ── zoom panel ────────────────────────────────────────────────────────────
    max_top_z = 0.0
    for i, n in enumerate(n_values):
        segs, err_lo, err_hi = all_segs[n]
        bottom = 0.0
        ms_parts = []
        for key, val, color, edge in segs:
            if key not in zoom_keys:
                continue
            val_ms = val * 1000.0
            ms_parts.append(f"{key}={val_ms:.6f}ms")
            if val_ms > EPS:
                ax_z.bar(i, val_ms, bottom=bottom, width=BAR_WIDTH,
                         color=color, edgecolor=edge, linewidth=1.0, zorder=3)
                bottom += val_ms
        ms_parts.append(f"total={bottom:.6f}ms")
        err_lo_ms = err_lo * 1000.0
        err_hi_ms = err_hi * 1000.0
        ms_parts.append(f"err=-{err_lo_ms:.3f}ms/+{err_hi_ms:.3f}ms")
        print(f"[DEBUG] {panel_label} | mode={mode} | N={n} | milliseconds: " + ", ".join(ms_parts))
        max_top_z = max(max_top_z, bottom)

    ax_z.set_xticks(range(len(n_values)))
    ax_z.set_xlim(-0.6, len(n_values) - 0.4)
    zoom_headroom = max(max_top_z * 0.04, 1e-6)
    ax_z.set_ylim(0, max_top_z + zoom_headroom)
    ax_z.margins(y=0)
    ax_z.set_ylabel("Time (ms)")
    ax_z.tick_params(axis="both", which="major", length=6, width=1.0, direction="out")
    ax_z.grid(axis="y", which="major", linestyle="-", linewidth=0.7, alpha=0.75)
    ax_z.set_axisbelow(True)
    ax_z.ticklabel_format(axis="y", style="plain", useOffset=False)

    if is_bottom:
        ax_z.set_xticklabels([str(n) for n in n_values])
        ax_z.set_xlabel("Number of robots (N)")
    else:
        ax_z.tick_params(labelbottom=False)

    # ── zoom indicator ────────────────────────────────────────────────────────
    zoom_top_s = max_top_z / 1000.0
    top_frac   = zoom_top_s / (max_top * HEADROOM)
    xlim       = ax.get_xlim()

    ax.add_patch(mpatches.FancyBboxPatch(
        (xlim[0], 0), xlim[1] - xlim[0], zoom_top_s,
        boxstyle="square,pad=0", linewidth=1.2,
        edgecolor="dimgray", facecolor="gray", alpha=0.10,
        linestyle="--", zorder=2,
    ))

    for (y_frac_l, y_frac_r), shrink_b in [((top_frac, 1.0), 200), ((0.0, 0.0), 30)]:
        fig.add_artist(ConnectionPatch(
            xyA=(1, y_frac_l), coordsA="axes fraction",
            xyB=(0, y_frac_r), coordsB="axes fraction",
            axesA=ax, axesB=ax_z,
            color="dimgray", linestyle="--", linewidth=0.9, zorder=10,
            shrinkA=4, shrinkB=shrink_b,
        ))


def main():
    script_dir = Path(__file__).parent.resolve()

    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)
    plt.rcParams.update({"font.family": "serif", "pdf.fonttype": 42, "ps.fonttype": 42})

    startup_path    = (script_dir / INPUT_STARTUP).resolve()
    continuous_path = (script_dir / INPUT_CONTINUOUS.format(VARIANT=VARIANT)).resolve()

    for path, label in [(startup_path, "startup"), (continuous_path, "continuous")]:
        if not path.exists():
            print(f"[WARN] Missing: {path}")
            return

    df_s = pd.read_csv(startup_path)
    df_s = df_s[df_s["n_robots"].isin(N_VALUES)]

    df_c = pd.read_csv(continuous_path)
    cpu_mask = df_c["cpu_limit"].isna() if CPU_LIMIT is None else (df_c["cpu_limit"] == CPU_LIMIT)
    df_c = df_c[
        (df_c["n_robots"].isin(N_VALUES)) &
        (df_c["ssp_ms"]  == SSP_MS) &
        (df_c["iterq"]   == ITERQ) &
        cpu_mask
    ].copy()

    if df_s.empty or df_c.empty:
        print("[WARN] One or both datasets are empty after filtering.")
        return

    n_s = sorted(df_s["n_robots"].unique().tolist())
    n_c = sorted(df_c["n_robots"].unique().tolist())

    fig = plt.figure(figsize=(11, 9))
    gs  = fig.add_gridspec(
        2, 2,
        width_ratios=[3, 2],
        hspace=0.12,
        wspace=0.30,
        left=0.08, right=0.97, top=0.84, bottom=0.07,
    )

    ax_s  = fig.add_subplot(gs[0, 0])
    ax_sz = fig.add_subplot(gs[0, 1])
    ax_c  = fig.add_subplot(gs[1, 0], sharex=ax_s)
    ax_cz = fig.add_subplot(gs[1, 1], sharex=ax_sz)

    draw_row(ax_s,  ax_sz, df_s, "startup",    n_s, fig, is_bottom=False, panel_label="Initial Attestation")
    draw_row(ax_c,  ax_cz, df_c, "continuous", n_c, fig, is_bottom=True,  panel_label="Continuous Attestation")

    # Shared legend at top — continuous mode covers all segments
    legend_handles = [
        mpatches.Patch(facecolor=COLORS[k], edgecolor=EDGE_COLORS[k], linewidth=1.0, label=LABELS[k])
        for k in ORDER
    ]
    fig.legend(handles=legend_handles, loc="upper center", bbox_to_anchor=(0.5, 0.99),
               ncol=2, frameon=True, framealpha=0.9, fancybox=False,
               edgecolor="black", borderpad=0.5, handlelength=1.4,
               columnspacing=1.2)

    out_path = script_dir / f"barplot_attestation_time.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] Saved: {out_path}")


if __name__ == "__main__":
    main()
