#!/usr/bin/env python3
"""
Tamper detection — Time to Detection box + strip plot.

Single target  : one box per SSP value.
Both targets   : two boxes per SSP (state_publisher | sidecar), side by side.

Data: experiments/data/tamper-detection/results/<target>/SSP*ms-batch*.csv

Usage:
  python3 boxplot_tamper_detection.py                    # state_publisher only
  python3 boxplot_tamper_detection.py --target sidecar
  python3 boxplot_tamper_detection.py --target both      # grouped comparison
"""

import argparse
import csv
from pathlib import Path

import matplotlib.colors as mcolors
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

# ── Config ────────────────────────────────────────────────────────────────────

DATA_BASE  = Path(__file__).parent.parent / "data" / "tamper-detection" / "results"
SCRIPT_DIR = Path(__file__).parent.resolve()
FONT_SCALE = 1.7


# Palette aligned with barplot_attestation_time_v5_combined.py
_COLOR = {
    "state_publisher": "#4B9FCC",
    "sidecar":         "#59c396",
}
_LABEL = {
    "state_publisher": "Robot app",
    "sidecar":         "Robot sidecar",
}

# ── Data loading ──────────────────────────────────────────────────────────────

def load_target(target: str) -> dict[int, list[float]]:
    """Return {ssp_ms: [latency_s, ...]} for detected trials only."""
    data: dict[int, list[float]] = {}
    target_dir = DATA_BASE / target
    if not target_dir.exists():
        return data
    for f in sorted(target_dir.glob("SSP*ms-batch*.csv")):
        with f.open(encoding="utf-8") as fh:
            for row in csv.DictReader(fh):
                if row["detected"] == "1" and row["latency_s"]:
                    ssp = int(row["ssp_ms"])
                    data.setdefault(ssp, []).append(float(row["latency_s"]))
    return data


# ── Drawing helpers ───────────────────────────────────────────────────────────

def _darken(color, factor=0.65):
    return tuple(c * factor for c in mcolors.to_rgb(color))


def _draw_boxes(ax, positions, values, color, box_width, rng):
    """Draw boxplot + jitter strip for one target series."""
    dark = _darken(color)
    bp = ax.boxplot(
        values,
        positions=positions,
        widths=box_width,
        patch_artist=True,
        notch=False,
        whis=1.5,
        showfliers=False,
        medianprops=dict(color=dark, linewidth=1.0),
        whiskerprops=dict(linewidth=1.0, linestyle="-", color=dark),
        capprops=dict(linewidth=1.0, color=dark),
        boxprops=dict(linewidth=0),
    )
    for patch in bp["boxes"]:
        patch.set_facecolor(color)
        patch.set_alpha(0.75)
        patch.set_edgecolor(dark)
        patch.set_linewidth(0.8)

    jitter_half = min(0.08, box_width * 0.22)
    for pos, vals in zip(positions, values):
        arr = np.array(vals)
        q1, q3 = np.percentile(arr, 25), np.percentile(arr, 75)
        iqr = q3 - q1
        lo, hi = q1 - 1.5 * iqr, q3 + 1.5 * iqr
        outliers = arr[(arr < lo) | (arr > hi)]
        jitter = rng.uniform(-jitter_half, jitter_half, size=len(outliers))
        ax.scatter(np.full(len(outliers), pos) + jitter, outliers,
                   s=22, color=color, alpha=0.85, zorder=5, linewidths=0.6,
                   edgecolors=_darken(color))


# ── Main plot function ────────────────────────────────────────────────────────

def make_plot(data_map: dict[str, dict[int, list[float]]], out_suffix: str) -> None:
    targets  = list(data_map.keys())
    n_tgts   = len(targets)
    ssps     = sorted({s for d in data_map.values() for s in d}, reverse=True)
    n_ssps   = len(ssps)
    tick_pos = list(range(n_ssps))          # group centres: 0, 1, 2, 3
    labels   = [str(s // 1000) for s in ssps]

    # Per-target x offsets and box width
    if n_tgts == 1:
        box_width = 0.42
        offsets   = [0.0]
    else:
        box_width = 0.36
        offsets   = [-0.22, +0.22]

    all_vals = [v for d in data_map.values() for vals in d.values() for v in vals]
    y_top    = max(all_vals) * 1.06
    y_ticks  = list(range(0, int(y_top) + 2, 5))

    # ── style ─────────────────────────────────────────────────────────────────
    sns.set_theme(
        context="paper", style="ticks",
        rc={"xtick.direction": "out", "ytick.direction": "out"},
        font_scale=FONT_SCALE,
    )
    plt.rcParams.update({"font.family": "serif"})

    fig, ax = plt.subplots(figsize=(7.0, 4.2))
    plt.subplots_adjust(left=0.09, right=0.98, top=0.97, bottom=0.12)

    rng = np.random.default_rng(42)

    for target, offset in zip(targets, offsets):
        data   = data_map[target]
        pos    = [tp + offset for tp in tick_pos]
        values = [data.get(s, []) for s in ssps]
        _draw_boxes(ax, pos, values, _COLOR[target], box_width, rng)

    # ── axes ──────────────────────────────────────────────────────────────────
    ax.set_xticks(tick_pos)
    ax.set_xticklabels(labels)
    ax.set_xlim(-0.6, n_ssps - 0.4)
    ax.set_ylim(0, y_top)
    ax.set_yticks(y_ticks)
    ax.set_xlabel("SSP (s)")
    ax.set_ylabel("Time to Detection (s)")
    ax.tick_params(axis="both", which="major", length=6, width=1.0, direction="out")
    ax.grid(axis="y", which="major", linestyle="-", linewidth=0.7, alpha=0.75)
    ax.set_axisbelow(True)

    # ── legend — only when showing both targets ────────────────────────────────
    if n_tgts > 1:
        handles = [
            mpatches.Patch(facecolor=_COLOR[t], alpha=0.75, edgecolor=_darken(_COLOR[t]),
                           linewidth=0.8, label=_LABEL[t])
            for t in targets
        ]
        ax.legend(
            handles=handles,
            loc="upper right",
            frameon=True, framealpha=0.9, fancybox=False, edgecolor="black",
            borderpad=0.4, handlelength=1.2, labelspacing=0.3,
        )

    # ── console summary ───────────────────────────────────────────────────────
    print(f"\n{'target':<20} {'SSP(s)':<8} {'n':>4} {'min':>7} {'Q1':>7} "
          f"{'median':>7} {'mean':>7} {'Q3':>7} {'max':>7} {'std':>7}")
    print("-" * 78)
    for target in targets:
        data = data_map[target]
        for ssp in ssps:
            vals = data.get(ssp, [])
            if not vals:
                continue
            a = np.array(vals)
            print(f"{_LABEL[target]:<20} {ssp // 1000:<8} {len(a):>4} "
                  f"{a.min():>7.3f} {np.percentile(a,25):>7.3f} "
                  f"{np.median(a):>7.3f} {a.mean():>7.3f} "
                  f"{np.percentile(a,75):>7.3f} {a.max():>7.3f} "
                  f"{a.std():>7.3f}")
    print("-" * 78)

    # ── save ──────────────────────────────────────────────────────────────────
    out_path = SCRIPT_DIR / f"boxplot_tamper_detection{out_suffix}.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"\n[OK] Saved: {out_path.relative_to(SCRIPT_DIR.parent)}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    p = argparse.ArgumentParser(
        description="Time-to-Detection box + strip plot (tamper detection).",
    )
    p.add_argument(
        "--target",
        choices=["state_publisher", "sidecar", "both"],
        default="state_publisher",
    )
    args = p.parse_args()

    if args.target == "both":
        data_map   = {t: load_target(t) for t in ["state_publisher", "sidecar"]}
        data_map   = {t: d for t, d in data_map.items() if d}
        out_suffix = "_both"
    else:
        data_map   = {args.target: load_target(args.target)}
        out_suffix = "" if args.target == "state_publisher" else f"_{args.target}"

    missing = [t for t, d in data_map.items() if not d]
    for t in missing:
        print(f"[WARN] No data found for target={t} in {DATA_BASE / t}")

    data_map = {t: d for t, d in data_map.items() if d}
    if not data_map:
        return

    make_plot(data_map, out_suffix)


if __name__ == "__main__":
    main()
