#!/usr/bin/env python3
"""
Combined CPU + RAM resource usage for the attestation sidecar in continuous mode.
Top panel: CPU (vCPUs). Bottom panel: RAM (MB). Shared x-axis (fleet size N).
Two groups per N: Robot sidecar (avg per node) and SECaaS.
"""

from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import seaborn as sns

INPUT_FILE = "../data/docker-stats/_summary/overall_resource_usage_{VARIANT}.csv"
MODE      = "continuous"
VARIANT    = "lv"   # ← continuous-mode variant to plot
SSP_MS     = 20000           # sidecar sleep period (ms)
ITERQ    = 1            # rolling-hash queue depth
CPU_LIMIT = None         # None = no cap (cpuNC)
N_VALUES  = [4, 8, 16, 32, 64]

FONT_SCALE    = 1.6
BAR_WIDTH     = 0.32
GROUP_SPACING = 1.0
HEADROOM      = 1.15

COLOR_ROBOT  = "#993333"
COLOR_SECAAS = "#336699"


def _clean_label(raw: str) -> str:
    name = raw.replace("-sidecar", "").strip().lower()
    if name.startswith("secaas"):
        return "SECaaS"
    if name.startswith("robot"):
        return "Robot"
    return raw.title()


def _aggregate(df, n_values):
    rows = []
    for n in n_values:
        sub = df[df["n_robots"] == n].copy()
        sub["group"] = sub["container"].apply(_clean_label)

        robots = sub[sub["group"] == "Robot"]
        if not robots.empty:
            rows.append({
                "n": n, "group": "Robot sidecar",
                "cpu_mean": robots["cpu_percent_mean"].mean() / 100.0,
                "cpu_std":  robots["cpu_percent_std"].mean()  / 100.0,
                "ram_mean": robots["mem_mb_mean"].mean(),
                "ram_std":  robots["mem_mb_std"].mean(),
            })

        secaas = sub[sub["group"] == "SECaaS"]
        if not secaas.empty:
            rows.append({
                "n": n, "group": "SECaaS",
                "cpu_mean": float(secaas["cpu_percent_mean"].iloc[0]) / 100.0,
                "cpu_std":  float(secaas["cpu_percent_std"].iloc[0])  / 100.0,
                "ram_mean": float(secaas["mem_mb_mean"].iloc[0]),
                "ram_std":  float(secaas["mem_mb_std"].iloc[0]),
            })
    return pd.DataFrame(rows)


def main():
    script_dir = Path(__file__).parent.resolve()
    csv_path   = (script_dir / INPUT_FILE.format(VARIANT=VARIANT)).resolve()
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    df = pd.read_csv(csv_path)
    df = df[df["n_robots"].isin(N_VALUES)]
    mode_df = df[df["mode"] == MODE].copy()
    if mode_df.empty:
        raise SystemExit(f"No data for mode={MODE} variant={VARIANT}")

    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)
    plt.rcParams.update({"font.family": "serif"})

    n_values = sorted(mode_df["n_robots"].unique())
    agg      = _aggregate(mode_df, n_values)
    group_xs = [i * GROUP_SPACING for i in range(len(n_values))]

    step    = BAR_WIDTH
    offsets = {"Robot sidecar": -step / 2, "SECaaS": step / 2}
    groups  = ["Robot sidecar", "SECaaS"]
    colors  = {"Robot sidecar": COLOR_ROBOT, "SECaaS": COLOR_SECAAS}

    fig = plt.figure(figsize=(8, 6.2))
    gs  = fig.add_gridspec(
        2, 1, hspace=0.12,
        left=0.12, right=0.97, top=0.86, bottom=0.10,
    )
    ax_cpu = fig.add_subplot(gs[0])
    ax_ram = fig.add_subplot(gs[1], sharex=ax_cpu)

    cpu_tops, ram_tops = [], []

    for i, n in enumerate(n_values):
        for grp in groups:
            row = agg[(agg["n"] == n) & (agg["group"] == grp)]
            if row.empty:
                continue
            x     = group_xs[i] + offsets[grp]
            color = colors[grp]

            cpu_mean = float(row["cpu_mean"].iloc[0])
            cpu_std  = float(row["cpu_std"].iloc[0])
            ram_mean = float(row["ram_mean"].iloc[0])
            ram_std  = float(row["ram_std"].iloc[0])

            for ax, mean, std, tops in [
                (ax_cpu, cpu_mean, cpu_std, cpu_tops),
                (ax_ram, ram_mean, ram_std, ram_tops),
            ]:
                ax.bar(x, mean, width=BAR_WIDTH,
                       color=color, edgecolor="none", zorder=3)
                if std > 0:
                    ax.errorbar(x, mean, yerr=std,
                                fmt="none", color="black",
                                capsize=3.5, linewidth=1.2, zorder=5)
                tops.append(mean + std)

            print(f"[VAL] N={n} {grp}: "
                  f"cpu={cpu_mean:.4f}±{cpu_std:.4f} vCPU  "
                  f"ram={ram_mean:.2f}±{ram_std:.2f} MB")

    # ── Top panel (CPU) ────────────────────────────────────────────────────────
    ax_cpu.set_ylabel("CPU usage (vCPUs)")
    ax_cpu.set_ylim(0, max(cpu_tops) * HEADROOM)
    ax_cpu.set_xlim(group_xs[0]  - GROUP_SPACING * 0.55,
                    group_xs[-1] + GROUP_SPACING * 0.55)
    ax_cpu.tick_params(axis="both", which="major", length=6, width=1.0, direction="out")
    ax_cpu.tick_params(labelbottom=False)
    ax_cpu.grid(axis="y", which="major", linestyle="-", linewidth=0.7, alpha=0.75)
    ax_cpu.set_axisbelow(True)

    # ── Bottom panel (RAM) ─────────────────────────────────────────────────────
    ax_ram.set_ylabel("RAM usage (MB)")
    ax_ram.set_ylim(0, max(ram_tops) * HEADROOM)
    ax_ram.set_xticks(group_xs)
    ax_ram.set_xticklabels([str(n) for n in n_values])
    ax_ram.set_xlabel("Number of robots (N)")
    ax_ram.tick_params(axis="both", which="major", length=6, width=1.0, direction="out")
    ax_ram.grid(axis="y", which="major", linestyle="-", linewidth=0.7, alpha=0.75)
    ax_ram.set_axisbelow(True)

    # ── Shared legend ──────────────────────────────────────────────────────────
    patches = [
        mpatches.Patch(facecolor=COLOR_ROBOT,  edgecolor="none", label="Robot sidecar"),
        mpatches.Patch(facecolor=COLOR_SECAAS, edgecolor="none", label="SECaaS"),
    ]
    fig.legend(handles=patches, loc="upper center", bbox_to_anchor=(0.5, 0.99),
               ncol=2, frameon=True, framealpha=0.9, fancybox=False,
               edgecolor="black", borderpad=0.5, handlelength=1.4,
               columnspacing=1.2)

    out_path = script_dir / f"barplot_docker_stats_combined_{MODE}_{VARIANT}.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"\n[OK] Saved: {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
