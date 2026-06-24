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
SHOW       = "cpu"  # "cpu" | "net" | "both"

LINE_WIDTH   = 2.6
FONT_SCALE   = 2.0
WINDOW_S     = 1      # rolling-mean smoothing window (seconds)
BAND_ALPHA   = 0.25   # opacity of ±std shaded band

COLOR_ROBOT  = "#3BA774"
COLOR_SECAAS = "#3B3177"

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
        if subset.empty:
            print(f"[WARN] No data for N={N_ROBOTS} mode={mode}, skipping.")
            continue
        generate_plot(subset, N_ROBOTS, mode, script_dir)


def generate_plot(df, n_robots, mode, script_dir):
    df = df.copy()
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

    axes[0].set_xlim(left=0, right=df["t_rel_s"].max())
    for ax in axes[:-1]:
        ax.tick_params(axis="x", labelbottom=False)
    axes[-1].set_xlabel("Time (s)")

    # ── Attestation-cycle grey bands on ALL peaks ─────────────────────────────
    # Detect all SECaaS CPU spike centres, then shade a ±half-width window on
    # every panel in light grey.  Uses the same smoothed series already computed
    # for plotting; falls back to a simple periodic grid if detection fails.
    C_ATT_BAND = "#bbbbbb"   # light grey fill
    att_peak_times: list[float] = []

    if has_secaas:
        sub_s   = df[df["Container"] == "SECaaS"].sort_values("t_rel_s")
        sm_s    = _rolling(sub_s["cpu_vcpus"], sub_s["t_rel_s"])
        t_s_arr = sub_s["t_rel_s"].values
        if sm_s.size:
            baseline   = np.percentile(sm_s, 25) or 1e-6
            spike_mask = (sm_s > 2.0 * baseline) & (t_s_arr > 3.0)
            try:
                from scipy.signal import find_peaks
                peak_idxs, _ = find_peaks(sm_s * spike_mask, distance=5)
            except ImportError:
                # scipy unavailable — use simple local-max detection
                masked = sm_s * spike_mask
                peak_idxs = np.where(
                    (masked[1:-1] > masked[:-2]) & (masked[1:-1] > masked[2:])
                )[0] + 1
            att_peak_times = [float(t_s_arr[i]) for i in peak_idxs]

    # fallback: if too few peaks found, guess from the SSP (if detectable)
    if len(att_peak_times) < 2 and has_secaas:
        t_max = float(df["t_rel_s"].max())
        ssp_guess = 20.0
        att_peak_times = list(np.arange(ssp_guess, t_max, ssp_guess))

    if att_peak_times:
        # Estimate spacing from detected peaks.
        if len(att_peak_times) >= 2:
            spacing = float(np.median(np.diff(att_peak_times)))
        else:
            spacing = 20.0
        # Extrapolate the first cycle backward: the initial attestation fires
        # immediately at startup but has lower magnitude (startup noise suppresses
        # the spike), so find_peaks misses it.  Prepend one period before the
        # earliest detected peak, clamped to t ≥ 0.
        first_extrap = att_peak_times[0] - spacing
        if first_extrap >= 0:
            att_peak_times = [first_extrap] + att_peak_times

        half_w = min(spacing * 0.20, 3.0)
        for t_pk in att_peak_times:
            for ax in axes:
                ax.axvspan(t_pk - half_w, t_pk + half_w,
                           color=C_ATT_BAND, alpha=0.55, zorder=1, linewidth=0)

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
    if att_peak_times:
        legend_handles.append(
            mpatches.Patch(facecolor=C_ATT_BAND, alpha=0.55, edgecolor="none",
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
