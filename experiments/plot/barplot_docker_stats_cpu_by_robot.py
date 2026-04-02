#!/usr/bin/env python3

from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import seaborn as sns

# ---- Config ----
INPUT_FILE = "../data/docker-stats/_summary/overall_resource_usage_per_container.csv"
MODES = ["startup", "continuous"]

FONT_SCALE = 1.5
SPINES_WIDTH = 1.0
FIG_SIZE = (10, 5)
BAR_WIDTH = 0.35
GROUP_SPACING = 1.2

SHOW_VALUE_LABELS = False
SHOW_ERROR_BARS = True

ERR_COLOR = "black"
palette = sns.color_palette("tab10", n_colors=10)
COLOR_ROBOT  = palette[0]
COLOR_SECAAS = palette[1]


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

    needed = {"n_robots", "mode", "container", "cpu_percent_mean", "cpu_percent_std"}
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
    n_values = sorted(df["n_robots"].unique())

    # Aggregate per N: Robot = mean across all robot containers, SECaaS = single container
    agg_rows = []
    for n in n_values:
        sub = df[df["n_robots"] == n].copy()
        sub["group"] = sub["container"].apply(_clean_container_label)

        robots = sub[sub["group"] == "Robot"]
        if not robots.empty:
            agg_rows.append({
                "n": n,
                "group": "Robot",
                "mean_vcpu": robots["cpu_percent_mean"].mean() / 100.0,
                "std_vcpu":  robots["cpu_percent_std"].mean()  / 100.0,
            })

        secaas = sub[sub["group"] == "SECaaS"]
        if not secaas.empty:
            agg_rows.append({
                "n": n,
                "group": "SECaaS",
                "mean_vcpu": float(secaas["cpu_percent_mean"].iloc[0]) / 100.0,
                "std_vcpu":  float(secaas["cpu_percent_std"].iloc[0])  / 100.0,
            })

    agg = pd.DataFrame(agg_rows)

    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "in", "ytick.direction": "in"},
                  font_scale=FONT_SCALE)

    fig, ax = plt.subplots(figsize=FIG_SIZE)

    xticks, xlabels, group_centers = [], [], []

    high_vals = []

    for i, n in enumerate(n_values):
        gc = i * GROUP_SPACING
        x_robot  = gc - BAR_WIDTH / 2 - 0.02
        x_secaas = gc + BAR_WIDTH / 2 + 0.02

        xticks.extend([x_robot, x_secaas])
        xlabels.extend(["Robot", "SECaaS"])
        group_centers.append((gc, f"N={n}"))

        for x_pos, grp, color in [(x_robot, "Robot", COLOR_ROBOT),
                                   (x_secaas, "SECaaS", COLOR_SECAAS)]:
            row = agg[(agg["n"] == n) & (agg["group"] == grp)]
            if row.empty:
                continue
            mean = float(row["mean_vcpu"].iloc[0])
            std  = float(row["std_vcpu"].iloc[0])

            ax.bar(x_pos, mean, width=BAR_WIDTH,
                   color=color, edgecolor="black", linewidth=SPINES_WIDTH, zorder=3)

            if SHOW_ERROR_BARS and std > 0:
                ax.errorbar(x_pos, mean, yerr=std,
                            fmt="none", ecolor=ERR_COLOR,
                            elinewidth=1.5, capsize=4, capthick=1.5, zorder=10)

            high_vals.append(mean + (std if SHOW_ERROR_BARS else 0))

            if SHOW_VALUE_LABELS:
                ax.text(x_pos, mean + std + 0.01, f"{mean:.2f}",
                        ha="center", va="bottom", fontsize="small")

            print(f"[VAL] mode={mode} N={n} group={grp} mean={mean:.4f} std={std:.4f} vcpu")

    # Y-limit
    y_max = max(high_vals) if high_vals else 1.0
    ax.set_ylim(0, y_max * 1.18)

    # X ticks and group labels
    ax.set_xticks(xticks)
    ax.set_xticklabels([""] * len(xticks))

    tick_fs = plt.rcParams.get("xtick.labelsize")
    for gc, lbl in group_centers:
        ax.text(gc, -0.05, lbl,
                ha="center", va="top",
                transform=ax.get_xaxis_transform(),
                fontsize=tick_fs)

    ax.set_ylabel("CPU usage (vCPUs)")
    ax.set_xlabel("")

    x_margin = BAR_WIDTH / 2 + GROUP_SPACING * 0.2
    ax.set_xlim(xticks[0] - x_margin, xticks[-1] + x_margin)

    ax.set_axisbelow(True)
    ax.grid(axis="y", linestyle="-", linewidth=1.0, alpha=0.8)
    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_color("black")
        ax.spines[side].set_linewidth(SPINES_WIDTH)

    patches = [
        mpatches.Patch(facecolor=COLOR_ROBOT,  edgecolor="black", label="Robot"),
        mpatches.Patch(facecolor=COLOR_SECAAS, edgecolor="black", label="SECaaS"),
    ]
    ax.legend(handles=patches, loc="upper right",
              frameon=True, framealpha=0.9, fancybox=True)

    plt.tight_layout()
    plt.subplots_adjust(bottom=0.12)

    out_path = script_dir / f"./barplot_docker_stats_cpu_by_robot_{mode}.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved plot to: {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
