#!/usr/bin/env python3
"""
Shared matplotlib/seaborn style for all paper plots.

Usage in each plot script:
    import plot_style
    plot_style.apply(FONT_SCALE)

    ax.grid(axis="y", **plot_style.GRID_KW)
    ax.tick_params(axis="both", **plot_style.TICK_KW)
"""

import matplotlib.pyplot as plt
import seaborn as sns

GRID_KW = dict(which="major", linestyle="-", linewidth=0.7, alpha=0.45)
TICK_KW = dict(which="major", length=6, width=1.0, direction="out")
BAR_LW  = 1.0


def apply(font_scale: float) -> None:
    """Apply shared paper style. Call once before creating any figure."""
    sns.set_theme(
        context="paper", style="ticks",
        rc={"xtick.direction": "out", "ytick.direction": "out"},
        font_scale=font_scale,
    )
    plt.rcParams.update({
        "font.family": "serif",
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    })
