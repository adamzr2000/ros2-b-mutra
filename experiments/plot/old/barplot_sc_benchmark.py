#!/usr/bin/env python3

from pathlib import Path
import json
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# ---- Config ----
INPUT_FILE = "../data/sc-benchmark/baseline.json"
OUTPUT_FILE = "./barplot_sc_benchmark.pdf"

FONT_SCALE = 1.5
SPINES_WIDTH = 1.0
FIG_SIZE = (9, 8)

# Toggle features
SHOW_VALUE_LABELS = True
SHOW_ERROR_BARS = True

# Styling
VALUE_LABEL_COLOR = "black"
ERR_COLOR = "black"


def main():
    # Load JSON summary
    json_path = Path(INPUT_FILE).resolve()
    if not json_path.exists():
        raise SystemExit(f"JSON not found: {json_path}")

    with open(json_path, "r") as f:
        data = json.load(f)

    if "summary" not in data or not data["summary"]:
        raise SystemExit("JSON missing 'summary' field or it's empty")

    # Extract function data and convert to seconds
    rows = []
    for item in data["summary"]:
        rows.append({
            "function": item.get("function"),
            "latency_mean_s": item.get("latency_mean_ms") / 1000.0,
            "latency_std_s": (item.get("latency_std_ms") or 0.0) / 1000.0,
            "gas_used_mean": item.get("gas_used_mean") or 0.0,
            "gas_used_std": item.get("gas_used_std") or 0.0,
        })

    plot_df = pd.DataFrame(rows)

    desired_order = [
        "RegisterAgent",
        "SendEvidence",
        "SendRefSignature",
        "CloseAttestationProcess",
    ]
    display_name_map = {
        "RegisterAgent": "registerAgent()",
        "SendEvidence": "selfAttestationRequest()",
        "SendRefSignature": "sendRefMeasurement()",
        "CloseAttestationProcess": "sendAttestationResult()",
    }
    present_desired = [fn for fn in desired_order if fn in plot_df["function"].values]
    remaining = [fn for fn in plot_df["function"].tolist() if fn not in present_desired]
    ordered_functions = present_desired + remaining
    plot_df["function"] = pd.Categorical(
        plot_df["function"],
        categories=ordered_functions,
        ordered=True,
    )
    plot_df = plot_df.sort_values("function").reset_index(drop=True)
    plot_df["function_display"] = plot_df["function"].astype(str).map(display_name_map).fillna(plot_df["function"].astype(str))

    # Theme + palette + inward ticks
    sns.set_theme(context="paper", style="ticks", rc={"xtick.direction": "out", "ytick.direction": "out"}, font_scale=FONT_SCALE)

    # Get distinct colors from tab10 for each function
    palette = sns.color_palette("tab10", n_colors=len(plot_df))

    fig, (ax_gas, ax_latency) = plt.subplots(
        2,
        1,
        figsize=FIG_SIZE,
        sharex=True,
        constrained_layout=True,
        gridspec_kw={"height_ratios": [1, 1.2], "hspace": 0.08},
    )

    bar_width = 0.7
    x_positions = list(range(len(plot_df)))

    # Top subplot: gas used mean
    for i, (idx, row) in enumerate(plot_df.iterrows()):
        ax_gas.bar(
            i,
            row["gas_used_mean"],
            color=palette[i],
            edgecolor="black",
            linewidth=SPINES_WIDTH,
            width=bar_width,
        )

        if SHOW_ERROR_BARS:
            ax_gas.errorbar(
                i,
                row["gas_used_mean"],
                yerr=row["gas_used_std"],
                fmt="none",
                ecolor=ERR_COLOR,
                elinewidth=1.5,
                capsize=5,
                capthick=1.5,
                zorder=10,
            )

    if SHOW_ERROR_BARS:
        gas_y_max = (plot_df["gas_used_mean"] + plot_df["gas_used_std"]).max()
    else:
        gas_y_max = plot_df["gas_used_mean"].max()
    gas_top_padding = gas_y_max * 0.10 if gas_y_max > 0 else 1.0
    ax_gas.set_ylim(0, gas_y_max + gas_top_padding)
    ax_gas.set_ylabel("Gas Used")

    # Bottom subplot: latency
    for i, (idx, row) in enumerate(plot_df.iterrows()):
        ax_latency.bar(
            i,
            row["latency_mean_s"],
            color=palette[i],
            edgecolor="black",
            linewidth=SPINES_WIDTH,
            width=bar_width,
        )
        
        # Add error bars
        if SHOW_ERROR_BARS:
            ax_latency.errorbar(
                i,
                row["latency_mean_s"],
                yerr=row["latency_std_s"],
                fmt="none",
                ecolor=ERR_COLOR,
                elinewidth=1.5,
                capsize=5,
                capthick=1.5,
                zorder=10,
            )

    # Set x-axis labels (shared)
    ax_latency.set_xticks(x_positions)
    ax_latency.set_xticklabels(plot_df["function_display"], rotation=15, ha="right")

    # Y-axis limit
    if SHOW_ERROR_BARS:
        y_max = (plot_df["latency_mean_s"] + plot_df["latency_std_s"]).max()
    else:
        y_max = plot_df["latency_mean_s"].max()
    top_padding = y_max * 0.10
    ax_latency.set_ylim(0, y_max + top_padding)

    # Value labels on bars
    if SHOW_VALUE_LABELS:
        for i, (idx, row) in enumerate(plot_df.iterrows()):
            y_text = row["gas_used_mean"] / 2.0
            ax_gas.text(
                i,
                y_text,
                f"{row['gas_used_mean']:.0f}",
                ha="center",
                va="center",
                color=VALUE_LABEL_COLOR,
                fontsize="small",
                zorder=20,
            )

        for i, (idx, row) in enumerate(plot_df.iterrows()):
            y_text = row["latency_mean_s"] / 2.0
            ax_latency.text(
                i,
                y_text,
                f"{row['latency_mean_s']:.2f}",
                ha="center",
                va="center",
                color=VALUE_LABEL_COLOR,
                fontsize="small",
                zorder=20,
            )

    # Formatting
    ax_latency.set_ylabel("Latency (s)")

    # Grid + spines
    for subplot_ax in (ax_gas, ax_latency):
        subplot_ax.set_axisbelow(True)
        subplot_ax.grid(axis="y", linestyle="-", linewidth=0.8, alpha=0.6)
        for side in ("top", "right", "bottom", "left"):
            subplot_ax.spines[side].set_color("black")
            subplot_ax.spines[side].set_linewidth(SPINES_WIDTH)

    # Save
    fig.savefig(OUTPUT_FILE, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved plot to: {OUTPUT_FILE}")
    plt.close(fig)


if __name__ == "__main__":
    main()
