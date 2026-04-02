#!/usr/bin/env python3
"""
v2: N-aggregated breakdown with simple 2-color stack (blockchain vs offchain).
    4 bars per fleet-size group:
      Robot Prover | Robot Verifier || SECaaS Oracle | SECaaS Verifier
    Error bars show inter-robot std on totals.
    See v4 for the fine-grained segment version of this layout.
"""

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import sys
import matplotlib.patches as mpatches

# ---- Config ----
INPUT_FILE = "../data/attestation-times/_summary/durations_summary.csv"
MODES = ["startup", "continuous"]

FONT_SCALE = 1.5
FIG_SIZE = (14, 6)
BAR_WIDTH    = 0.28
INNER_GAP    = 0.05   # gap between bars within the same sub-group (Robot / SECaaS)
INTER_GAP    = 0.14   # extra gap between Robot pair and SECaaS pair
GROUP_SPACING = 2.0   # center-to-center between N groups
TEXT_THRESHOLD_S  = 0.2
TOP_HEADROOM_FACTOR = 1.25
SHOW_STD = True
SHOW_VALUE_LABELS = False


def generate_plot(df_filtered, mode, script_dir):
    # 1. Labels
    LBL_BLOCKCHAIN = "Blockchain client (contract calls + tx submission)"
    LBL_OFFCHAIN   = "Local processing (hash computation, hash comparison, SECaaS db lookup)"

    # 2. Colors
    palette = sns.color_palette("tab10", n_colors=10)
    c_blockchain = palette[0]  # Blue
    c_offchain   = palette[1]  # Orange

    # 3. Aggregation helpers
    def get_agg(n, group, role, metric):
        """Returns (mean_across_participants, mean_of_within_participant_std).
        The std is the average of each participant's run-to-run std_s,
        representing measurement variability rather than inter-robot spread.
        """
        mask = (
            (df_filtered["n_robots"] == n) &
            (df_filtered["participant_group"] == group) &
            (df_filtered["role"] == role) &
            (df_filtered["metric"] == metric)
        )
        sub = df_filtered[mask]
        if sub.empty:
            return 0.0, 0.0
        mean_val = float(sub["mean_s"].mean())
        if "std_s" in sub.columns:
            std_val = float(sub["std_s"].dropna().mean()) if not sub["std_s"].dropna().empty else 0.0
        else:
            std_val = 0.0
        return mean_val, std_val

    def get_mean(n, group, role, metric):
        return get_agg(n, group, role, metric)[0]

    # 4. Sorted N values
    n_values = sorted(df_filtered["n_robots"].unique().tolist())

    # 5. Setup plot
    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)

    fig, ax = plt.subplots(figsize=FIG_SIZE)

    final_xticks      = []
    final_xticklabels = []
    subgroup_centers  = []  # (x_center, label) for "Robot" / "SECaaS"
    group_centers     = []  # (x_center, label) for "N=X"

    # 6. Draw function
    def draw_stack(x_pos, stack_data, total_std=0.0):
        bottom = 0.0
        shown_segments = []
        for val, color in stack_data:
            if val <= 0.001:
                continue
            ax.bar(x_pos, val, bottom=bottom, width=BAR_WIDTH,
                   color=color, edgecolor="black", linewidth=1.0, zorder=3)
            if SHOW_VALUE_LABELS and val > TEXT_THRESHOLD_S:
                shown_segments.append(f"{val:.1f}")
                ax.text(x_pos, bottom + val / 2, f"{val:.1f}",
                        ha="center", va="center", color="white",
                        fontsize="small", fontweight="normal", zorder=4)
            bottom += val

        if SHOW_STD and total_std > 0:
            ax.errorbar(x_pos, bottom, yerr=total_std,
                        fmt="none", color="black", capsize=4,
                        linewidth=1.5, zorder=5)

        shown_std = total_std if (SHOW_STD and total_std > 0) else 0.0
        if SHOW_VALUE_LABELS:
            ax.text(x_pos, bottom + shown_std + 0.05, f"{bottom:.1f}s",
                    ha="center", va="bottom", color="black", fontsize="small")

    # 7. Bar position layout
    step        = BAR_WIDTH + INNER_GAP
    half_pair   = step / 2
    pair_offset = half_pair + INTER_GAP / 2 + BAR_WIDTH / 2

    for i, n in enumerate(n_values):
        gc = i * GROUP_SPACING

        x1 = gc - pair_offset - half_pair   # Robot Prover
        x2 = gc - pair_offset + half_pair   # Robot Verifier
        x3 = gc + pair_offset - half_pair   # SECaaS Oracle
        x4 = gc + pair_offset + half_pair   # SECaaS Verifier

        final_xticks.extend([x1, x2, x3, x4])
        final_xticklabels.extend(["Prover", "Verifier", "Oracle", "Verifier"])
        subgroup_centers.append(((x1 + x2) / 2, "Robot"))
        subgroup_centers.append(((x3 + x4) / 2, "SECaaS"))
        group_centers.append((gc, f"N={n}"))

        # Stds for error bars
        _, std_rp = get_agg(n, "Robot",  "prover",   "total_lifecycle")
        _, std_rv = get_agg(n, "Robot",  "verifier", "total_lifecycle")
        _, std_so = get_agg(n, "SECaaS", "oracle",   "total_lifecycle")
        _, std_sv = get_agg(n, "SECaaS", "verifier", "total_lifecycle")

        # --- Robot Prover ---
        total_p  = get_mean(n, "Robot", "prover", "total_lifecycle")
        rpc_call = get_mean(n, "Robot", "prover", "evidence_call")
        e2e      = get_mean(n, "Robot", "prover", "e2e_blockchain")
        val_bc_p  = e2e + rpc_call
        val_off_p = max(0.0, total_p - val_bc_p)
        print(f"[VAL] mode={mode} N={n} bar=Robot-Prover   "
              f"offchain={val_off_p:.3f}s  blockchain={val_bc_p:.3f}s  "
              f"total={total_p:.3f}s  std={std_rp:.3f}s")
        draw_stack(x1, [(val_off_p, c_offchain), (val_bc_p, c_blockchain)], std_rp)

        # --- Robot Verifier ---
        total_rv  = get_mean(n, "Robot", "verifier", "total_lifecycle")
        comp_rv   = get_mean(n, "Robot", "verifier", "verify_compute")
        val_off_rv = comp_rv
        val_bc_rv  = max(0.0, total_rv - val_off_rv)
        print(f"[VAL] mode={mode} N={n} bar=Robot-Verifier "
              f"offchain={val_off_rv:.3f}s  blockchain={val_bc_rv:.3f}s  "
              f"total={total_rv:.3f}s  std={std_rv:.3f}s")
        draw_stack(x2, [(val_off_rv, c_offchain), (val_bc_rv, c_blockchain)], std_rv)

        # --- SECaaS Oracle ---
        total_o  = get_mean(n, "SECaaS", "oracle", "total_lifecycle")
        db_o     = get_mean(n, "SECaaS", "oracle", "db_lookup")
        val_off_o = db_o
        val_bc_o  = max(0.0, total_o - val_off_o)
        print(f"[VAL] mode={mode} N={n} bar=SECaaS-Oracle  "
              f"offchain={val_off_o:.3f}s  blockchain={val_bc_o:.3f}s  "
              f"total={total_o:.3f}s  std={std_so:.3f}s")
        draw_stack(x3, [(val_off_o, c_offchain), (val_bc_o, c_blockchain)], std_so)

        # --- SECaaS Verifier ---
        total_sv  = get_mean(n, "SECaaS", "verifier", "total_lifecycle")
        comp_sv   = get_mean(n, "SECaaS", "verifier", "verify_compute")
        val_off_sv = comp_sv
        val_bc_sv  = max(0.0, total_sv - val_off_sv)
        print(f"[VAL] mode={mode} N={n} bar=SECaaS-Verifier"
              f"offchain={val_off_sv:.3f}s  blockchain={val_bc_sv:.3f}s  "
              f"total={total_sv:.3f}s  std={std_sv:.3f}s")
        draw_stack(x4, [(val_off_sv, c_offchain), (val_bc_sv, c_blockchain)], std_sv)

    # 8. Styling
    ax.set_xticks(final_xticks)
    ax.set_xticklabels(final_xticklabels, rotation=0, ha="center")

    tick_fs = plt.rcParams.get("xtick.labelsize")
    for x_c, lbl in subgroup_centers:
        ax.text(x_c, -0.10, lbl,
                ha="center", va="top",
                transform=ax.get_xaxis_transform(),
                fontsize=tick_fs, fontstyle="italic")

    for x_c, lbl in group_centers:
        ax.text(x_c, -0.18, lbl,
                ha="center", va="top",
                transform=ax.get_xaxis_transform(),
                fontsize=tick_fs)

    ax.set_ylabel("Time (s)")
    ax.set_axisbelow(True)
    ax.grid(axis="y", linestyle="-", linewidth=1.0, alpha=0.8)
    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_color("black")
        ax.spines[side].set_linewidth(1.0)

    # 9. Legend
    patches = [
        mpatches.Patch(facecolor=c_blockchain, edgecolor="black", label=LBL_BLOCKCHAIN),
        mpatches.Patch(facecolor=c_offchain,   edgecolor="black", label=LBL_OFFCHAIN),
    ]
    y_lim = ax.get_ylim()[1]
    ax.set_ylim(0, y_lim * TOP_HEADROOM_FACTOR)
    ax.legend(handles=patches, loc="upper right",
              frameon=True, framealpha=0.9, fancybox=True)

    ax.set_title("Breakdown of attestation cycle time by fleet size and module")

    plt.tight_layout()
    plt.subplots_adjust(bottom=0.28)

    x_margin = BAR_WIDTH / 2 + GROUP_SPACING * 0.2
    ax.set_xlim(min(final_xticks) - x_margin, max(final_xticks) + x_margin)

    output_file = f"./barplot_attestation_cycle_time_by_robot_breakdown_v1N_{mode}.pdf"
    out_path = script_dir / output_file
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved plot to: {out_path}")
    plt.close(fig)
    return True


def main():
    script_dir = Path(__file__).parent.resolve()
    csv_path = (script_dir / INPUT_FILE).resolve()

    if not csv_path.exists():
        print(f"[ERR] CSV not found at: {csv_path}")
        sys.exit(1)

    df = pd.read_csv(csv_path)

    required_cols = {"mode", "n_robots", "participant_group", "role", "metric", "mean_s"}
    missing = required_cols - set(df.columns)
    if missing:
        print(f"[ERR] CSV missing required columns: {missing}")
        print(f"      Found columns: {df.columns.tolist()}")
        sys.exit(1)

    for mode in MODES:
        mode_df = df[df["mode"] == mode].copy()
        if mode_df.empty:
            print(f"[WARN] No data found for mode '{mode}'")
            continue
        if not generate_plot(mode_df, mode, script_dir):
            print(f"[WARN] Failed to generate plot for mode '{mode}'")


if __name__ == "__main__":
    main()
