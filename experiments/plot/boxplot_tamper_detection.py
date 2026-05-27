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
    "state_publisher": "#336699",   # C_BC  — robot binary (prover side)
    "sidecar":         "#228888",   # C_VERIFIER — sidecar self-integrity
}
_LABEL = {
    "state_publisher": "Robot binary",
    "sidecar":         "Sidecar binary",
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

def _draw_boxes(ax, positions, values, color, box_width, rng):
    """Draw boxplot + jitter strip for one target series."""
    bp = ax.boxplot(
        values,
        positions=positions,
        widths=box_width,
        patch_artist=True,
        notch=False,
        whis=(0, 100),
        showfliers=False,
        medianprops=dict(color="black", linewidth=2.0),
        whiskerprops=dict(linewidth=1.0, linestyle="-"),
        capprops=dict(linewidth=1.0),
        boxprops=dict(linewidth=0),
    )
    for patch in bp["boxes"]:
        patch.set_facecolor(color)
        patch.set_alpha(0.75)

    jitter_half = min(0.08, box_width * 0.22)
    for pos, vals in zip(positions, values):
        jitter = rng.uniform(-jitter_half, jitter_half, size=len(vals))
        ax.scatter(
            np.full(len(vals), pos) + jitter, vals,
            s=12, color=color, alpha=0.45, zorder=4, linewidths=0,
        )


# ── Main plot function ────────────────────────────────────────────────────────

def make_plot(data_map: dict[str, dict[int, list[float]]], out_suffix: str) -> None:
    targets  = list(data_map.keys())
    n_tgts   = len(targets)
    ssps     = sorted({s for d in data_map.values() for s in d})
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

    # Global floor across all targets and SSPs
    all_vals = [v for d in data_map.values() for vals in d.values() for v in vals]
    floor    = min(all_vals)
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

    # ── blockchain confirmation floor shading ──────────────────────────────────
    ax.axhspan(0, floor, color="#dddddd", alpha=0.55, zorder=1)
    ax.axhline(floor, color="#888888", linestyle="--", linewidth=1.1, zorder=2)
    ax.text(
        n_ssps - 0.45, floor * 0.45,
        "Blockchain confirmation minimum",
        ha="right", va="center",
        fontsize=plt.rcParams["axes.labelsize"] * 0.62,
        color="#555555",
    )

    # ── axes ──────────────────────────────────────────────────────────────────
    ax.set_xticks(tick_pos)
    ax.set_xticklabels(labels)
    ax.set_xlim(-0.6, n_ssps - 0.4)
    ax.set_ylim(0, y_top)
    ax.set_yticks(y_ticks)
    ax.set_xlabel("Sidecar Sleep Period (s)")
    ax.set_ylabel("Time to Detection (s)")
    ax.tick_params(axis="both", which="major", length=6, width=1.0, direction="out")
    ax.grid(axis="y", which="major", linestyle="-", linewidth=0.7, alpha=0.75)
    ax.set_axisbelow(True)

    # ── legend — only when showing both targets ────────────────────────────────
    if n_tgts > 1:
        handles = [
            mpatches.Patch(facecolor=_COLOR[t], alpha=0.75, edgecolor="none",
                           label=_LABEL[t])
            for t in targets
        ]
        ax.legend(
            handles=handles,
            loc="upper left",
            frameon=True, framealpha=0.9, fancybox=False, edgecolor="black",
            borderpad=0.4, handlelength=1.2, labelspacing=0.3,
        )

    # ── save ──────────────────────────────────────────────────────────────────
    out_path = SCRIPT_DIR / f"boxplot_tamper_detection{out_suffix}.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] Saved: {out_path.relative_to(SCRIPT_DIR.parent)}")


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
