#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import matplotlib.patches as mpatches
import seaborn as sns
import re
import numpy as np
from pathlib import Path
from functools import reduce

INPUT_STARTUP_TEMPLATE    = "../data/docker-stats/_summary/timeline_resource_usage_startup_run{run}.csv"
INPUT_CONTINUOUS_TEMPLATE = "../data/docker-stats/_summary/timeline_resource_usage_{variant}_run{run}.csv"
TARGET_RUN = 1
N_ROBOTS   = 4
MODES      = ["continuous"]
VARIANT    = "rr"   # ← continuous-mode variant to plot
FILTER_SSP = None   # set to e.g. 5000 to restrict to one SSP; None = all
SHOW       = "cpu"  # "cpu" | "net" | "both"
PLOT_END_S = 120

# Attestation-cycle spans (t_start_s, t_end_s) — robot sidecar rises → SECaaS drops.
# Cycle 1 fires immediately at t=0; first blockchain round-trip ~8 s.
# Subsequent cycles start every SSP=10 s after completion: 18, 28, 38, …
CYCLE_SPANS = [
    (0, 4), (8, 14), (18, 24), (28, 34), (38, 44), (48, 54),
    (58, 64), (68, 74), (78, 84), (88, 94), (98, 104), (108, 114),
]

LINE_WIDTH   = 2.6
FONT_SCALE   = 2.2
WINDOW_S     = 1      # rolling-mean smoothing window (seconds)
BAND_ALPHA   = 0.25   # opacity of ±std shaded band

# Color palette 1 (green→purple)
# COLOR_ROBOT  = "#3BA774"
# COLOR_SECAAS = "#3B3177"

# Color palette 2 (blue→red)
COLOR_ROBOT = "#0000FF"
COLOR_SECAAS   = "#FF0000"

def _clean_container_label(raw: str) -> str:
    name = str(raw).replace("-sidecar", "").strip()
    low  = name.lower()
    if low.startswith("secaas"):
        return "SECaaS"
    if low.startswith("robot"):
        return "Robot" + low.replace("robot", "").title()
    return name.title()


def _label_order(label: str):
    m = re.match(r"Robot(\d+)$", label)
    if m:
        return (0, int(m.group(1)))
    return (1, label)


def _rolling(series: pd.Series, t_rel: pd.Series) -> pd.Series:
    """Time-based rolling mean over WINDOW_S seconds."""
    df_tmp = pd.DataFrame({"val": series.values, "t": t_rel.values})
    df_tmp = df_tmp.sort_values("t").copy()
    df_tmp["t_td"] = pd.to_timedelta(df_tmp["t"], unit="s")
    df_tmp = df_tmp.set_index("t_td")
    df_tmp["sm"] = df_tmp["val"].rolling(f"{WINDOW_S}s", min_periods=1).mean()
    return df_tmp["sm"].values


def main():
    script_dir = Path(__file__).parent.resolve()

    needed = {"n_robots", "mode", "container", "t_rel_s",
              "cpu_percent", "net_tx_mb"}

    for mode in MODES:
        tmpl = INPUT_STARTUP_TEMPLATE if mode == "startup" else INPUT_CONTINUOUS_TEMPLATE
        csv_path = (script_dir / tmpl.format(run=TARGET_RUN, variant=VARIANT)).resolve()
        if not csv_path.exists():
            print(f"[WARN] CSV not found for mode={mode}: {csv_path}")
            continue

        df = pd.read_csv(csv_path)
        missing = needed - set(df.columns)
        if missing:
            print(f"[WARN] Timeline CSV missing columns: {missing}. Run summarize_docker_stats.py first.")
            continue

        subset = df[df["n_robots"] == N_ROBOTS].copy()
        if FILTER_SSP is not None and "ssp_ms" in subset.columns:
            subset = subset[subset["ssp_ms"] == FILTER_SSP]
        if subset.empty:
            print(f"[WARN] No data for N={N_ROBOTS} mode={mode}, skipping.")
            continue
        generate_plot(subset, N_ROBOTS, mode, script_dir)


def generate_plot(df, n_robots, mode, script_dir):
    df = df.copy()
    df = df[df["t_rel_s"] <= PLOT_END_S].copy()
    df["Container"] = df["container"].apply(_clean_container_label)

    # CPU % → vCPUs
    df["cpu_vcpus"] = df["cpu_percent"] / 100.0

    # Cumulative net TX MB → throughput kbit/s (per container)
    for container, idx in df.groupby("Container").groups.items():
        g  = df.loc[idx].sort_values("t_rel_s")
        dt = g["t_rel_s"].diff()
        rate = (df.loc[g.index, "net_tx_mb"].diff() / dt * 8000)
        df.loc[g.index, "net_tx_kbps"] = (
            rate.where(dt > 0, 0.0)
                .replace([float("inf"), -float("inf")], 0.0)
                .fillna(0.0).clip(lower=0.0)
        )

    containers       = sorted(df["Container"].unique(), key=_label_order)
    robot_containers = [c for c in containers if c.startswith("Robot")]
    has_secaas       = "SECaaS" in containers

    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)
    plt.rcParams.update({"font.family": "serif", "pdf.fonttype": 42, "ps.fonttype": 42})

    _all_panels = [
        ("cpu", "cpu_vcpus",   "CPU (vCPUs)"),
        ("net", "net_tx_kbps", "Net TX (kbit/s)"),
    ]
    panel_defs = [(col, ylabel) for key, col, ylabel in _all_panels
                  if SHOW in (key, "both")]
    n_panels   = len(panel_defs)

    fig_h      = 3.5 if n_panels == 1 else 5.5
    bot_margin = 0.20 if n_panels == 1 else 0.12
    top_margin = 0.80 if n_panels == 1 else 0.87
    fig = plt.figure(figsize=(10, fig_h))
    gs  = fig.add_gridspec(n_panels, 1, height_ratios=[1.0] * n_panels,
                           hspace=0.12, left=0.10, right=0.97,
                           top=top_margin, bottom=bot_margin)
    axes = [fig.add_subplot(gs[0])]
    for i in range(1, n_panels):
        axes.append(fig.add_subplot(gs[i], sharex=axes[0]))

    panels = [(axes[i], col, ylabel) for i, (col, ylabel) in enumerate(panel_defs)]

    def _robot_band(col):
        """Return (t, mean, std) arrays aggregated across robot containers."""
        smoothed = {}
        for c in robot_containers:
            sub = df[df["Container"] == c].sort_values("t_rel_s").copy()
            sm  = _rolling(sub[col], sub["t_rel_s"])
            smoothed[c] = pd.Series(sm, index=sub["t_rel_s"].values)
        band = (
            pd.DataFrame(smoothed)
              .sort_index()
              .interpolate(method="index", limit_direction="both")
        )
        t    = band.index.values
        mean = band.mean(axis=1).values
        std  = band.std(axis=1).fillna(0.0).values
        return t, mean, std

    for ax, col, ylabel in panels:
        y_max = 0.0

        # ── Robot band ────────────────────────────────────────────────────────
        if robot_containers:
            t, mean, std = _robot_band(col)
            ax.plot(t, mean, color=COLOR_ROBOT, linewidth=LINE_WIDTH, zorder=3)
            ax.fill_between(t, (mean - std).clip(0), mean + std,
                            color=COLOR_ROBOT, alpha=BAND_ALPHA, zorder=2)
            y_max = max(y_max, (mean + std).max())

        # ── SECaaS line ───────────────────────────────────────────────────────
        if has_secaas:
            sub = df[df["Container"] == "SECaaS"].sort_values("t_rel_s").copy()
            sm  = _rolling(sub[col], sub["t_rel_s"])
            ax.plot(sub["t_rel_s"].values, sm,
                    color=COLOR_SECAAS, linewidth=LINE_WIDTH, zorder=3)
            y_max = max(y_max, sm.max() if len(sm) else 0.0)

        ax.set_ylabel(ylabel)
        ax.set_axisbelow(True)
        ax.grid(axis="both", linestyle="-", linewidth=0.7, alpha=0.75)
        y_max = y_max if not pd.isna(y_max) and y_max > 0 else 1.0
        ax.set_ylim(0, y_max * 1.10)

        # ── Console print of time series (sampled) ───────────────────────────
        rows = []
        # robot band
        if robot_containers:
            t_r, mean_r, std_r = _robot_band(col)
        else:
            t_r = mean_r = std_r = np.array([])

        # secaas smoothed
        if has_secaas:
            sub = df[df["Container"] == "SECaaS"].sort_values("t_rel_s").copy()
            sm_s = _rolling(sub[col], sub["t_rel_s"]) if len(sub) else np.array([])
            t_s = sub["t_rel_s"].values
        else:
            t_s = sm_s = np.array([])

        # unified time grid
        grid = np.unique(np.concatenate([t_r, t_s])) if len(t_r) + len(t_s) > 0 else np.array([])
        if grid.size:
            step = max(1, int(len(grid) / 50))
            print(f"[TS] {ylabel} (sample every {step}):")
            for idx in range(0, len(grid), step):
                tt = grid[idx]
                r_mean = float(np.interp(tt, t_r, mean_r)) if t_r.size else float('nan')
                r_std  = float(np.interp(tt, t_r, std_r))  if t_r.size else 0.0
                s_val  = float(np.interp(tt, t_s, sm_s))   if t_s.size else float('nan')
                # format depending on column
                if col == 'cpu_vcpus':
                    print(f"  t={tt:.1f}s  robot={r_mean:.4f}±{r_std:.4f} vCPU  secaas={s_val:.4f} vCPU")
                else:
                    print(f"  t={tt:.1f}s  robot={r_mean:.1f}±{r_std:.1f} kbit/s  secaas={s_val:.1f} kbit/s")

    axes[0].set_xlim(left=0, right=PLOT_END_S)
    for ax in axes[:-1]:
        ax.tick_params(axis="x", labelbottom=False)
    axes[-1].set_xlabel("Time (s)")
    from matplotlib.ticker import MultipleLocator
    axes[-1].xaxis.set_major_locator(MultipleLocator(10))
    axes[-1].xaxis.set_minor_locator(MultipleLocator(5))

    # ── Attestation-cycle grey bands (hardcoded from printed time series) ────────
    C_ATT_BAND = "#d5d5d5"
    for t0, t1 in CYCLE_SPANS:
        for ax in axes:
            ax.axvspan(t0, t1, color=C_ATT_BAND, alpha=0.50, zorder=1, linewidth=0)

    # ── Figure legend (top right) ─────────────────────────────────────────────
    legend_handles = []
    if robot_containers:
        legend_handles += [
            mlines.Line2D([], [], color=COLOR_ROBOT, linewidth=LINE_WIDTH,
                          label="Robot sidecar"),
        ]
    if has_secaas:
        legend_handles.append(
            mlines.Line2D([], [], color=COLOR_SECAAS, linewidth=LINE_WIDTH,
                          label="SECaaS")
        )
    if CYCLE_SPANS:
        legend_handles.append(
            mpatches.Patch(facecolor=C_ATT_BAND, alpha=0.50, edgecolor="none",
                           label="Attestation cycle")
        )

    axes[0].legend(handles=legend_handles,
                   bbox_to_anchor=(0.5, 1.02), loc="lower center",
                   ncol=3, framealpha=0.9,
                   fancybox=False, edgecolor="black",
                   borderpad=0.5, handlelength=1.4, columnspacing=1.0)
    
    panel_tag = f"_{SHOW}" if SHOW != "both" else ""
    out_path = script_dir / f"timeplot_docker_stats_N{n_robots}_{mode}_run{TARGET_RUN}_{VARIANT}{panel_tag}.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved plot to: {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
