#!/usr/bin/env python3

from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import seaborn as sns

# ---- Config ----
INPUT_FILE = "../data/docker-stats/_summary/overall_resource_usage_per_container.csv"
MODES = ["startup", "continuous"]

FONT_SCALE    = 1.8
BAR_WIDTH     = 0.32
GROUP_SPACING = 1.0
HEADROOM      = 1.10

SHOW_VALUE_LABELS = False
SHOW_ERROR_BARS   = True



def _clean_container_label(raw: str) -> str:
    name = raw.replace("-sidecar", "").strip()
    lower = name.lower()
    if lower.startswith("secaas"):
        return "SECaaS"
    if lower.startswith("robot"):
        return "Robot"
    return name.title()


def main():
    script_dir = Path(__file__).parent.resolve()
    csv_path = (script_dir / INPUT_FILE).resolve()
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    df = pd.read_csv(csv_path)

    needed = {"n_robots", "mode", "container", "mem_mb_mean", "mem_mb_std"}
    missing = needed - set(df.columns)
    if missing:
        raise SystemExit(f"CSV is missing required columns: {missing}.")

    for mode in MODES:
        mode_df = df[df["mode"] == mode].copy()
        if mode_df.empty:
            print(f"[WARN] No data for mode={mode}, skipping.")
            continue
        generate_plot(mode_df, mode, script_dir)


def generate_plot(df, mode, script_dir):
    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)
    plt.rcParams.update({"font.family": "serif"})

    color_robot  = "#336699"   # steel blue
    color_secaas = "#993333"   # dark red

    n_values = sorted(df["n_robots"].unique())

    # Aggregate per N:
    #   Robot  = mean of per-robot mem_mb_mean (average per-node footprint)
    #   SECaaS = single container value
    agg_rows = []
    for n in n_values:
        sub = df[df["n_robots"] == n].copy()
        sub["group"] = sub["container"].apply(_clean_container_label)

        robots = sub[sub["group"] == "Robot"]
        if not robots.empty:
            agg_rows.append({
                "n": n, "group": "Robot sidecar",
                "mean_mb": robots["mem_mb_mean"].mean(),
                "std_mb":  robots["mem_mb_std"].mean(),
            })

        secaas = sub[sub["group"] == "SECaaS"]
        if not secaas.empty:
            agg_rows.append({
                "n": n, "group": "SECaaS",
                "mean_mb": float(secaas["mem_mb_mean"].iloc[0]),
                "std_mb":  float(secaas["mem_mb_std"].iloc[0]),
            })

    agg = pd.DataFrame(agg_rows)

    fig, ax = plt.subplots(figsize=(9, 4.2))

    step    = BAR_WIDTH + 0.03
    offsets = [-step / 2, step / 2]
    groups  = ["Robot sidecar", "SECaaS"]
    colors  = [color_robot, color_secaas]

    high_vals = []
    group_xs  = [i * GROUP_SPACING for i in range(len(n_values))]

    for i, n in enumerate(n_values):
        for j, (grp, color) in enumerate(zip(groups, colors)):
            row = agg[(agg["n"] == n) & (agg["group"] == grp)]
            if row.empty:
                continue
            mean = float(row["mean_mb"].iloc[0])
            std  = float(row["std_mb"].iloc[0])
            x    = group_xs[i] + offsets[j]

            ax.bar(x, mean, width=BAR_WIDTH,
                   color=color, edgecolor="black", linewidth=0.8, zorder=3)

            if SHOW_ERROR_BARS and std > 0:
                ax.errorbar(x, mean, yerr=std,
                            fmt="none", color="black",
                            capsize=3.5, linewidth=1.2, zorder=5)

            high_vals.append(mean + (std if SHOW_ERROR_BARS else 0))
            print(f"[VAL] mode={mode} N={n} group={grp} mean={mean:.2f} std={std:.2f} MB")

    # Axes
    ax.set_xticks(group_xs)
    ax.set_xticklabels([str(n) for n in n_values])
    ax.set_xlim(group_xs[0] - GROUP_SPACING * 0.55,
                group_xs[-1] + GROUP_SPACING * 0.55)
    ax.set_ylim(0, (max(high_vals) if high_vals else 1.0) * HEADROOM)
    ax.set_xlabel("Number of robots (N)")
    ax.set_ylabel("RAM usage (MB)")
    ax.tick_params(axis="both", which="major", length=6, width=1.0, direction="out")
    ax.grid(axis="y", which="major", linestyle="--", linewidth=0.7, alpha=0.75)
    ax.set_axisbelow(True)

    # Legend top-centre
    patches = [
        mpatches.Patch(facecolor=color_robot,  edgecolor="black", label="Robot sidecar (avg per node)"),
        mpatches.Patch(facecolor=color_secaas, edgecolor="black", label="SECaaS"),
    ]
    fig.legend(handles=patches, loc="upper center", bbox_to_anchor=(0.5, 1.04),
               ncol=2, frameon=True, framealpha=0.9, fancybox=True,
               fontsize=plt.rcParams.get("legend.fontsize", 9))

    # fig.suptitle(f"Attestation sidecar RAM consumption — {mode.capitalize()} mode",
    #              y=-0.04, fontsize=plt.rcParams.get("axes.titlesize", 11))

    plt.subplots_adjust(top=0.88)

    out_path = script_dir / f"barplot_docker_stats_ram_by_robot_{mode}.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved plot to: {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
