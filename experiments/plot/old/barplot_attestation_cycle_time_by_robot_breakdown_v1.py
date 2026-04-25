#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import sys
import matplotlib.patches as mpatches

# ---- Config ----
INPUT_FILE = "../data/attestation-times/_summary/durations_summary.csv"
MODES = ["startup", "continuous"]  # Generate PDFs for both modes
N_ROBOTS = 4  # Only include robot1..robotN and secaas

FONT_SCALE = 1.5
FIG_SIZE = (10, 6)
BAR_WIDTH = 0.4
GROUP_SPACING = 1.2
TEXT_THRESHOLD_S = 0.2  # Threshold to show text inside bar
TOP_HEADROOM_FACTOR = 1.18
SHOW_STD = True
SHOW_VALUE_LABELS = False

def generate_plot(df_filtered, mode, script_dir):
    # 1. Define Labels
    LBL_BLOCKCHAIN = "Blockchain client (contract calls + tx submission)"
    LBL_OFFCHAIN   = "Local processing (hash computation, hash comparison, SECaaS db lookup)"

    # 2. Colors (Muted palette)
    palette = sns.color_palette("tab10", n_colors=10)
    c_blockchain = palette[0]  # Blue
    c_offchain   = palette[1]  # Orange

    # 3. Helper to get value safely
    def get_val(p, r, m):
        row = df_filtered[(df_filtered["participant"] == p) & (df_filtered["role"] == r) & (df_filtered["metric"] == m)]
        if row.empty: return 0.0
        return float(row["mean_s"].iloc[0])

    def get_std(p, r, m):
        row = df_filtered[(df_filtered["participant"] == p) & (df_filtered["role"] == r) & (df_filtered["metric"] == m)]
        if row.empty or "std_s" not in row.columns: return 0.0
        v = row["std_s"].iloc[0]
        return float(v) if v == v else 0.0  # guard NaN

    # 4. Sort Participants (Robots first, then SECaaS)
    participants = df_filtered["participant"].unique().tolist()
    def sort_key(s):
        s_str = str(s).lower()
        if "secaas" in s_str: return 1000
        digits = "".join(filter(str.isdigit, s_str))
        return int(digits) if digits else 999
    participants.sort(key=sort_key)

    # 5. Setup Plot
    sns.set_theme(context="paper", style="ticks", 
                  rc={"xtick.direction": "out", "ytick.direction": "out"}, 
                  font_scale=FONT_SCALE)
    
    fig, ax = plt.subplots(figsize=FIG_SIZE)

    final_xticks = []
    final_xticklabels = []
    group_centers = []
    group_labels = []

    # 6. Loop & Draw
    for i, p in enumerate(participants):
        is_secaas = (p.lower() == "secaas")
        group_center = i * GROUP_SPACING
        
        # Determine Names
        role_left_name  = "Oracle" if is_secaas else "Prover"
        role_right_name = "Verifier"
        
        # Calculate X Positions
        x_left  = group_center - (BAR_WIDTH / 2) - 0.05
        x_right = group_center + (BAR_WIDTH / 2) + 0.05
        
        # Store Ticks (role labels on bars, participant once per group)
        final_xticks.extend([x_left, x_right])
        final_xticklabels.extend([role_left_name, role_right_name])
        group_centers.append(group_center)
        group_labels.append(p)

        # --- CALCULATE LEFT BAR (Prover or Oracle) ---
        if is_secaas:
            total = get_val(p, "oracle", "total_lifecycle")
            db    = get_val(p, "oracle", "db_lookup")

            val_offchain   = db
            val_blockchain = max(0, total - val_offchain)
        else:
            # Prover Logic
            total = get_val(p, "prover", "total_lifecycle")
            rpc_call = get_val(p, "prover", "evidence_call")
            e2e   = get_val(p, "prover", "e2e_blockchain")
            
            val_blockchain = e2e + rpc_call
            val_offchain   = max(0, total - val_blockchain)

        left_stack = [(val_offchain, c_offchain), (val_blockchain, c_blockchain)]

        # --- CALCULATE RIGHT BAR (Verifier) ---
        total_v = get_val(p, "verifier", "total_lifecycle")
        comp_v  = get_val(p, "verifier", "verify_compute")

        val_offchain_v   = comp_v
        val_blockchain_v = max(0, total_v - val_offchain_v)
        
        right_stack = [(val_offchain_v, c_offchain), (val_blockchain_v, c_blockchain)]

        # --- DRAWING FUNCTION ---
        def draw_stack(x_pos, stack_data, bar_label, total_std=0.0):
            bottom = 0.0
            shown_segments = []
            for val, color in stack_data:
                if val <= 0.001: continue

                # Draw Segment
                ax.bar(x_pos, val, bottom=bottom, width=BAR_WIDTH,
                       color=color, edgecolor="black", linewidth=1.0, zorder=3)

                # Label inside bar
                if SHOW_VALUE_LABELS and val > TEXT_THRESHOLD_S:
                    shown_segments.append(f"{val:.1f}")
                    ax.text(x_pos, bottom + val/2, f"{val:.1f}",
                            ha="center", va="center", color="white",
                            fontsize="small", fontweight="normal", zorder=4)

                bottom += val

            if SHOW_STD and total_std > 0:
                ax.errorbar(x_pos, bottom, yerr=total_std,
                            fmt="none", color="black", capsize=4,
                            linewidth=1.5, zorder=5)

            shown_std = total_std if (SHOW_STD and total_std > 0) else 0.0
            print(f"[VAL] mode={mode} participant={p} bar={bar_label} segments={shown_segments} total={bottom:.1f}s")
            if SHOW_VALUE_LABELS:
                ax.text(x_pos, bottom + shown_std + 0.05, f"{bottom:.1f}s",
                        ha="center", va="bottom", color="black", fontsize="small")

        left_role  = "oracle" if is_secaas else "prover"
        std_left   = get_std(p, left_role, "total_lifecycle")
        std_right  = get_std(p, "verifier", "total_lifecycle")

        draw_stack(x_left,  left_stack,  role_left_name,  std_left)
        draw_stack(x_right, right_stack, role_right_name, std_right)

    # 7. Styling
    ax.set_xticks(final_xticks)
    ax.set_xticklabels(final_xticklabels, rotation=0, ha="center")

    participant_label_size = plt.rcParams.get("xtick.labelsize")

    # Participant labels shown once per pair of bars
    for center, label in zip(group_centers, group_labels):
        ax.text(
            center,
            -0.08,
            label,
            ha="center",
            va="top",
            transform=ax.get_xaxis_transform(),
            fontsize=participant_label_size,
        )
    ax.set_ylabel("Time (s)")
    
    ax.set_axisbelow(True)
    ax.grid(axis="y", linestyle="-", linewidth=1.0, alpha=0.8)

    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_color("black")
        ax.spines[side].set_linewidth(1.0)

    # 8. Legend
    patches = [
        mpatches.Patch(facecolor=c_blockchain, edgecolor="black", label=LBL_BLOCKCHAIN),
        mpatches.Patch(facecolor=c_offchain,   edgecolor="black", label=LBL_OFFCHAIN)
    ]
    
    # Keep only modest headroom above bars/totals
    y_lim = ax.get_ylim()[1]
    ax.set_ylim(0, y_lim * TOP_HEADROOM_FACTOR)

    # Keep legend inside axes (upper-right)
    ax.legend(handles=patches,
              loc="upper right",
              frameon=True, 
              framealpha=0.9,
              fancybox=True)

    ax.set_title("Breakdown of attestation cycle time by participant and module")

    plt.tight_layout()
    plt.subplots_adjust(bottom=0.18)
    x_margin = BAR_WIDTH / 2 + GROUP_SPACING * 0.2
    ax.set_xlim(final_xticks[0] - x_margin, final_xticks[-1] + x_margin)
    output_file = f"./barplot_attestation_cycle_time_by_robot_breakdown_v1_{mode}.pdf"
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

    required_cols = {"mode", "participant", "role", "metric", "mean_s"}
    missing = required_cols - set(df.columns)
    if missing:
        print(f"[ERR] CSV missing required columns: {missing}")
        print(f"      Found columns: {df.columns.tolist()}")
        sys.exit(1)

    # Filter to rows from the N_ROBOTS scale scenario
    df = df[df["n_robots"] == N_ROBOTS]

    # Generate plots for each mode
    for mode in MODES:
        mode_df = df[df["mode"] == mode].copy()
        if mode_df.empty:
            print(f"[WARN] No data found for mode '{mode}'")
            continue
        if not generate_plot(mode_df, mode, script_dir):
            print(f"[WARN] Failed to generate plot for mode '{mode}'")
            continue

if __name__ == "__main__":
    main()