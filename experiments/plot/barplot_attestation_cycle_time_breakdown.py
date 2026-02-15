#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import sys
import matplotlib.patches as mpatches

# ---- Config ----
INPUT_FILE = "../data/attestation-times/_summary/attestation_durations_summary.csv"
OUTPUT_FILE = "./barplot_attestation_cycle_time_breakdown.pdf"

FONT_SCALE = 1.5
FIG_SIZE = (10, 6)
BAR_WIDTH = 0.4
GROUP_SPACING = 1.2
TEXT_THRESHOLD_S = 0.2  # Threshold to show text inside bar

def main():
    script_dir = Path(__file__).parent.resolve()
    csv_path = (script_dir / INPUT_FILE).resolve()
    
    # Fallback to current dir if not found
    if not csv_path.exists():
        csv_path = Path("attestation_durations_summary.csv").resolve()

    if not csv_path.exists():
        print(f"[ERR] CSV not found at: {csv_path}")
        sys.exit(1)

    df = pd.read_csv(csv_path)

    # 1. Define Labels
    LBL_BLOCKCHAIN = "Attestation procedures using blockchain (read & write operations)"
    LBL_OFFCHAIN   = "Local compute (hashing, db query, cryptographic comparison)"

    # 2. Colors (Blue for Blockchain, Orange for Off-chain)
    palette = sns.color_palette("colorblind", n_colors=10)
    c_blockchain = palette[0]  # Blue
    c_offchain   = palette[1]  # Orange

    # 3. Helper to get value safely
    def get_val(p, r, m):
        row = df[(df["participant"] == p) & (df["role"] == r) & (df["metric"] == m)]
        if row.empty: return 0.0
        return float(row["mean_s"].iloc[0])

    # 4. Sort Participants (Robots first, then SECaaS)
    participants = df["participant"].unique().tolist()
    def sort_key(s):
        s_str = str(s).lower()
        if "secaas" in s_str: return 1000
        digits = "".join(filter(str.isdigit, s_str))
        return int(digits) if digits else 999
    participants.sort(key=sort_key)

    # 5. Setup Plot
    sns.set_theme(context="paper", style="ticks", 
                  rc={"xtick.direction": "in", "ytick.direction": "in"}, 
                  font_scale=FONT_SCALE)
    
    fig, ax = plt.subplots(figsize=FIG_SIZE)

    final_xticks = []
    final_xticklabels = []

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
        
        # Store Ticks
        final_xticks.extend([x_left, x_right])
        final_xticklabels.extend([f"{p}\n({role_left_name})", f"{p}\n({role_right_name})"])

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
        def draw_stack(x_pos, stack_data):
            bottom = 0.0
            for val, color in stack_data:
                if val <= 0.001: continue
                
                # Draw Segment
                ax.bar(x_pos, val, bottom=bottom, width=BAR_WIDTH, 
                       color=color, edgecolor="black", linewidth=1.0, zorder=3)
                
                # Label inside bar
                if val > TEXT_THRESHOLD_S:
                    ax.text(x_pos, bottom + val/2, f"{val:.1f}", 
                            ha="center", va="center", color="white", 
                            fontsize=9, fontweight="normal", zorder=4)
                
                bottom += val
            
            # Total Label on Top
            ax.text(x_pos, bottom + 0.05, f"{bottom:.1f}s", 
                    ha="center", va="bottom", color="black", fontsize=9)

        draw_stack(x_left, left_stack)
        draw_stack(x_right, right_stack)

    # 7. Styling
    ax.set_xticks(final_xticks)
    ax.set_xticklabels(final_xticklabels, rotation=45, ha="right", rotation_mode="anchor")
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
    
    # Add extra headroom for legend
    y_lim = ax.get_ylim()[1]
    ax.set_ylim(0, y_lim * 1.35)

    ax.legend(handles=patches, loc="upper right", 
              frameon=True, 
              framealpha=0.9,
              fancybox=True)

    ax.set_title("Breakdown of attestation cycle time by role and participant")

    plt.tight_layout()
    out_path = script_dir / OUTPUT_FILE
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved plot to: {out_path}")
    plt.close(fig)

if __name__ == "__main__":
    main()