#!/usr/bin/env python3

import sys
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from seaborn.utils import desaturate
import matplotlib.patches as mpatches

# ---- Config ----
INPUT_FILE = "../data/attestation-times/_summary/attestation_durations_summary.csv"
OUTPUT_FILE = "./barplot_attestation_cycle_time_breakdown_detailed.pdf"

USE_LOG_SCALE = True    # <--- NEW: Toggle this to True/False
LOG_MIN_Y = 1e-4        # <--- NEW: Log scales cannot start at absolute 0.0

FONT_SCALE = 1.5
FIG_SIZE = (11, 6)
BAR_WIDTH = 0.4
GROUP_SPACING = 1.2
TEXT_THRESHOLD_S = 0.5
EPS = 1e-3  # ignore tiny segments
COLOR_SATURATION = 0.9

# Stable segment keys in preferred legend order
SEGMENT_ORDER = [
    # Prover
    "prover_compute",
    "prover_tx",
    "prover_wait",
    # Verifier
    "verifier_read",
    "verifier_verify",
    "verifier_tx",
    "verifier_overhead",
    # Oracle
    "oracle_creds",
    "oracle_db",
    "oracle_tx",
    "oracle_overhead",
]

# One authoritative place for display labels (no duplication)
LABELS = {
    "prover_compute": "Prover: Compute Fresh Signature\n(Local Program Hashing)",
    "prover_tx":      "Prover: Send Fresh Signature\n(Blockchain Write)",
    "prover_wait":    "Prover: Wait for Result",

    "verifier_read":     "Verifier: Fetch Fresh & Ref. Signatures\n(Blockchain Read)",
    "verifier_verify":   "Verifier: Verify Signatures",
    "verifier_tx":       "Verifier: Send Result\n(Blockchain Write)",
    "verifier_overhead": "Verifier: Overhead",

    "oracle_creds":    "Oracle: Fetch Prover Address\n(Blockchain Read)",
    "oracle_db":       "Oracle: Fetch Prover Ref. Signature\n(DB)",
    "oracle_tx":       "Oracle: Send Prover Ref. Signature\n(Blockchain Write)",
    "oracle_overhead": "Oracle: Overhead",
}


def main():
    script_dir = Path(__file__).parent.resolve()
    csv_path = (script_dir / INPUT_FILE).resolve()
    if not csv_path.exists():
        csv_path = Path("attestation_durations_summary.csv").resolve()
    if not csv_path.exists():
        print(f"[ERR] CSV not found at: {csv_path}")
        sys.exit(1)

    df = pd.read_csv(csv_path)

    def get_val(p, r, m) -> float:
        row = df[(df["participant"] == p) & (df["role"] == r) & (df["metric"] == m)]
        return 0.0 if row.empty else float(row["mean_s"].iloc[0])

    # Participants sorted with SECaaS last
    participants = df["participant"].unique().tolist()

    def sort_key(s):
        s_str = str(s).lower()
        if "secaas" in s_str:
            return (1, 9999)
        digits = "".join(filter(str.isdigit, s_str))
        return (0, int(digits) if digits else 999)

    participants.sort(key=sort_key)

    # Theme
    sns.set_theme(
        context="paper",
        style="ticks",
        rc={"xtick.direction": "in", "ytick.direction": "in"},
        font_scale=FONT_SCALE,
    )

    base_palette = list(sns.color_palette("colorblind"))
    extended_palette = base_palette + ["#4d4d4d", "#cccccc"]

    COLOR = {k: extended_palette[i] for i, k in enumerate(SEGMENT_ORDER)}

    fig, ax = plt.subplots(figsize=FIG_SIZE)

    used_keys = set()
    xticks, xlabels = [], []

    def build_left_segments(p: str, is_secaas: bool):
        if is_secaas:
            total    = get_val(p, "oracle", "total_lifecycle")
            reaction = get_val(p, "oracle", "reaction_time")
            creds    = get_val(p, "oracle", "prover_addr_fetch")
            db       = get_val(p, "oracle", "db_lookup")
            tx      = get_val(p, "oracle", "ref_signature_call")

            overhead = max(0.0, total - reaction - creds - db - tx)

            return [
                ("oracle_creds", creds),
                ("oracle_db", db),
                ("oracle_tx", tx),
                ("oracle_overhead", overhead),
            ]
        else:
            total = get_val(p, "prover", "total_lifecycle")
            e2e = get_val(p, "prover", "e2e_blockchain")
            tx = get_val(p, "prover", "evidence_call")

            compute   = max(0.0, total - e2e)
            wait      = max(0.0, e2e - tx)
            
            return [
                ("prover_compute", compute),
                ("prover_tx", tx),
                ("prover_wait", wait),
            ]

    def build_verifier_segments(p: str):
        total = get_val(p, "verifier", "total_lifecycle")
        reaction = get_val(p, "verifier", "reaction_time")
        read_v = get_val(p, "verifier", "signatures_fetch")
        comp_v = get_val(p, "verifier", "verify_compute")
        write_v = get_val(p, "verifier", "result_call")
        overhead = max(0.0, total - (reaction + read_v + comp_v + write_v))
        return [
            ("verifier_read", read_v),
            ("verifier_verify", comp_v),
            ("verifier_tx", write_v),
            ("verifier_overhead", overhead),
        ]

    def draw_stack(x_pos, segments):
        # NEW: Handle absolute zero bounds for log scales
        bottom = LOG_MIN_Y if USE_LOG_SCALE else 0.0
        
        for key, val in segments:
            if val <= EPS:
                continue
            used_keys.add(key)

            ax.bar(
                x_pos,
                val,
                bottom=bottom,
                width=BAR_WIDTH,
                color=desaturate(COLOR[key], COLOR_SATURATION),
                edgecolor="black",
                linewidth=1.0,
                zorder=3,
            )

            # We lower the text threshold if log scale is enabled to see smaller labels
            threshold = TEXT_THRESHOLD_S if not USE_LOG_SCALE else 0.05
            
            if val > threshold:
                ax.text(
                    x_pos,
                    bottom + val / 2,
                    f"{val:.2f}",
                    ha="center",
                    va="center",
                    color="white" if val > 0.5 else "black",
                    fontsize=9,
                    fontweight="normal",
                    zorder=4,
                )
            bottom += val

        # NEW: Log scales need geometric multipliers, not linear additions, for offsets
        top_offset = bottom * 1.25 if USE_LOG_SCALE else bottom + 0.05
        
        ax.text(
            x_pos,
            top_offset,
            f"{bottom:.2f}s",
            ha="center",
            va="bottom",
            color="black",
            fontsize=9,
            fontweight="normal",
        )

    for i, p in enumerate(participants):
        is_secaas = (p.lower() == "secaas")
        group_center = i * GROUP_SPACING

        role_left_name = "Oracle" if is_secaas else "Prover"
        role_right_name = "Verifier"

        x_left = group_center - (BAR_WIDTH / 2) - 0.05
        x_right = group_center + (BAR_WIDTH / 2) + 0.05

        xticks.extend([x_left, x_right])
        xlabels.extend([f"{p}\n({role_left_name})", f"{p}\n({role_right_name})"])

        draw_stack(x_left, build_left_segments(p, is_secaas))
        draw_stack(x_right, build_verifier_segments(p))

    # Axes styling
    ax.set_xticks(xticks)
    ax.set_xticklabels(xlabels, rotation=45, ha="right", rotation_mode="anchor")
    
    # NEW: Apply the Log Scale rules
    if USE_LOG_SCALE:
        ax.set_yscale("log")
        ax.set_ylabel("Time (s) [Log Scale]")
        y_lim = ax.get_ylim()[1]
        ax.set_ylim(LOG_MIN_Y, y_lim * 15) # Log scales need heavily multiplied headroom for legends
    else:
        ax.set_ylabel("Time (s)")
        y_lim = ax.get_ylim()[1]
        ax.set_ylim(0, y_lim * 1.25)

    ax.set_xlabel("")
    ax.set_axisbelow(True)
    ax.grid(axis="y", linestyle="-", linewidth=1.0, alpha=0.8)
    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_color("black")
        ax.spines[side].set_linewidth(1.0)

    # Legend built from what was actually used (in preferred order)
    legend_keys = [k for k in SEGMENT_ORDER if k in used_keys]
    patches = [
        mpatches.Patch(facecolor=COLOR[k], edgecolor="black", label=LABELS[k])
        for k in legend_keys
    ]

    ax.legend(
        handles=patches,
        loc="upper right",
        fontsize=9,
        frameon=True,
        framealpha=0.9,
        fancybox=True,
        ncol=1,
    )

    ax.set_title("Breakdown of Attestation Cycle Time by Role and Participant")

    plt.tight_layout()
    
    # NEW: Save as a different file if using log scale to prevent overwriting
    final_output = str(script_dir / OUTPUT_FILE)
    if USE_LOG_SCALE:
        final_output = final_output.replace(".pdf", "_log.pdf")
        
    fig.savefig(final_output, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved plot to: {final_output}")
    plt.close(fig)

if __name__ == "__main__":
    main()