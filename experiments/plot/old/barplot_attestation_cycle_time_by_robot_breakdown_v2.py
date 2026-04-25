#!/usr/bin/env python3

import sys
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from seaborn.utils import desaturate
import matplotlib.patches as mpatches

# ---- Config ----
INPUT_FILE = "../data/attestation-times/_summary/durations_summary.csv"
MODES = ["startup", "continuous"]
N_ROBOTS = 4  # Only include rows from this scale scenario

USE_LOG_SCALE = False    # <--- NEW: Toggle this to True/False
LOG_MIN_Y = 1e-4        # <--- NEW: Log scales cannot start at absolute 0.0
SHOW_STD = True
SHOW_VALUE_LABELS = False

FONT_SCALE = 1.5
FIG_SIZE = (11, 6)
BAR_WIDTH = 0.4
GROUP_SPACING = 1.2
TEXT_THRESHOLD_S = 0.5
EPS = 1e-3  # ignore tiny segments
COLOR_SATURATION = 0.9
MS_THRESHOLD_S = 0.1
LEGEND_FONT_SCALE = 0.85
IN_BAR_FONT_SCALE = 0.82

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
    "verifier_reaction",
    "verifier_overhead",
    # Oracle
    "oracle_creds",
    "oracle_db",
    "oracle_tx",
    "oracle_reaction",
    "oracle_overhead",
]

# One authoritative place for display labels (no duplication)
LABELS = {
    "prover_compute": "Hash compute",
    "prover_tx":      "Submit fresh measurement tx",
    "prover_wait":    "Wait for result",

    "verifier_read":     "Read fresh & ref. measurements (call)",
    "verifier_verify":   "Hash comparison",
    "verifier_tx":       "Submit verification result tx",
    "verifier_reaction": "Reaction time",
    "verifier_overhead": "Other processing",

    "oracle_creds":    "Read prover address (call)",
    "oracle_db":       "Fetch ref. measurement from db",
    "oracle_tx":       "Submit ref. measurement tx",
    "oracle_reaction": "Reaction time",
    "oracle_overhead": "Other processing",
}


def main():
    script_dir = Path(__file__).parent.resolve()
    csv_path = (script_dir / INPUT_FILE).resolve()
    if not csv_path.exists():
        print(f"[ERR] CSV not found at: {csv_path}")
        sys.exit(1)

    df = pd.read_csv(csv_path)

    df = df[df["n_robots"] == N_ROBOTS]

    for mode in MODES:
        df_mode = df[df["mode"] == mode].copy()
        if df_mode.empty:
            print(f"[WARN] No data found for mode '{mode}'")
            continue
        generate_plot(df_mode, mode, script_dir)


def generate_plot(df, mode, script_dir):

    def get_val(p, r, m) -> float:
        row = df[(df["participant"] == p) & (df["role"] == r) & (df["metric"] == m)]
        return 0.0 if row.empty else float(row["mean_s"].iloc[0])

    def get_std(p, r, m) -> float:
        row = df[(df["participant"] == p) & (df["role"] == r) & (df["metric"] == m)]
        if row.empty or "std_s" not in row.columns:
            return 0.0
        v = row["std_s"].iloc[0]
        return float(v) if v == v else 0.0  # guard NaN

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
        rc={"xtick.direction": "out", "ytick.direction": "out"},
        font_scale=FONT_SCALE,
    )

    # Color semantics built from predefined Seaborn palettes.
    # We keep role families separated while avoiding hardcoded HEX values.
    prover_palette = sns.color_palette("Blues", n_colors=3)
    verifier_palette = sns.color_palette("Greens", n_colors=5)
    oracle_palette = sns.color_palette("Oranges", n_colors=5)

    COLOR = {
        # Prover family (light -> dark)
        "prover_compute": prover_palette[0],
        "prover_tx": prover_palette[1],
        "prover_wait": prover_palette[2],

        # Verifier family (light -> dark)
        "verifier_read": verifier_palette[0],
        "verifier_verify": verifier_palette[1],
        "verifier_tx": verifier_palette[2],
        "verifier_reaction": verifier_palette[3],
        "verifier_overhead": verifier_palette[4],

        # Oracle family (light -> dark)
        "oracle_creds": oracle_palette[0],
        "oracle_db": oracle_palette[1],
        "oracle_tx": oracle_palette[2],
        "oracle_reaction": oracle_palette[3],
        "oracle_overhead": oracle_palette[4],
    }

    tick_label_size = plt.rcParams.get("xtick.labelsize")
    if isinstance(tick_label_size, (int, float)):
        annotation_font_size = max(8, tick_label_size * 0.9)
        legend_font_size = max(8, tick_label_size * LEGEND_FONT_SCALE)
        in_bar_font_size = max(7, tick_label_size * IN_BAR_FONT_SCALE)
    else:
        annotation_font_size = "small"
        legend_font_size = "small"
        in_bar_font_size = "x-small"

    fig, ax = plt.subplots(figsize=FIG_SIZE)

    used_keys = set()
    xticks, xlabels = [], []
    group_centers, group_labels = [], []
    segment_rows = []
    total_rows = []

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
                ("oracle_reaction", reaction),
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
            ("verifier_reaction", reaction),
            ("verifier_overhead", overhead),
        ]

    def draw_stack(x_pos, segments, total_std=0.0):
        def format_duration(seconds: float) -> str:
            if seconds < MS_THRESHOLD_S:
                return f"{seconds * 1000:.0f} ms"
            return f"{seconds:.2f} s"

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

            threshold = TEXT_THRESHOLD_S if not USE_LOG_SCALE else 0.05

            if SHOW_VALUE_LABELS and val > threshold:
                ax.text(
                    x_pos,
                    bottom + val / 2,
                    format_duration(val),
                    ha="center",
                    va="center",
                    color="white" if val > 0.5 else "black",
                    fontsize=in_bar_font_size,
                    fontweight="normal",
                    zorder=4,
                )
            bottom += val

        if SHOW_STD and total_std > 0:
            ax.errorbar(x_pos, bottom, yerr=total_std,
                        fmt="none", color="black", capsize=4,
                        linewidth=1.5, zorder=5)

        shown_std = total_std if (SHOW_STD and total_std > 0) else 0.0
        top_offset = bottom * 1.25 if USE_LOG_SCALE else bottom + shown_std + 0.05

        if SHOW_VALUE_LABELS:
            ax.text(
                x_pos,
                top_offset,
                format_duration(bottom),
                ha="center",
                va="bottom",
                color="black",
                fontsize=annotation_font_size,
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
        xlabels.extend([role_left_name, role_right_name])
        group_centers.append(group_center)
        group_labels.append(p)

        left_segments = build_left_segments(p, is_secaas)
        right_segments = build_verifier_segments(p)

        left_role = "oracle" if is_secaas else "prover"
        draw_stack(x_left,  left_segments,  get_std(p, left_role,  "total_lifecycle"))
        draw_stack(x_right, right_segments, get_std(p, "verifier", "total_lifecycle"))

        left_total = 0.0
        for key, val in left_segments:
            val = float(val)
            if val <= EPS:
                continue
            left_total += val
            segment_rows.append({
                "participant": p,
                "role": role_left_name,
                "segment": LABELS.get(key, key),
                "value_s": val,
            })

        right_total = 0.0
        for key, val in right_segments:
            val = float(val)
            if val <= EPS:
                continue
            right_total += val
            segment_rows.append({
                "participant": p,
                "role": role_right_name,
                "segment": LABELS.get(key, key),
                "value_s": val,
            })

        total_rows.append({"participant": p, "role": role_left_name, "total_s": left_total})
        total_rows.append({"participant": p, "role": role_right_name, "total_s": right_total})

    # Console tables: all plotted values (segments + totals)
    if segment_rows:
        seg_df = pd.DataFrame(segment_rows).copy()
        seg_df["value_s"] = seg_df["value_s"].map(lambda x: round(float(x), 6))
        print("\n[INFO] Plotted segment values (seconds):")
        print(seg_df.to_string(index=False))

    if total_rows:
        tot_df = pd.DataFrame(total_rows).copy()
        tot_df["total_s"] = tot_df["total_s"].map(lambda x: round(float(x), 6))
        print("\n[INFO] Plotted bar totals (seconds):")
        print(tot_df.to_string(index=False))

    # Axes styling
    ax.set_xticks(xticks)
    ax.set_xticklabels(xlabels, rotation=0, ha="center")

    # Participant label shown once per pair of bars (avoids repeated "Robot X")
    for center, label in zip(group_centers, group_labels):
        ax.text(
            center,
            -0.08,
            label,
            ha="center",
            va="top",
            transform=ax.get_xaxis_transform(),
            fontsize=tick_label_size,
        )
    
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
        fontsize=legend_font_size,
        frameon=True,
        framealpha=0.9,
        fancybox=True,
        ncol=3,
        handlelength=1.0,
        handletextpad=0.4,
        labelspacing=0.3,
        borderpad=0.3,
    )

    # ax.set_title("Breakdown of Attestation Cycle Time by Role and Participant")

    plt.tight_layout()
    plt.subplots_adjust(bottom=0.18)

    x_margin = BAR_WIDTH / 2 + GROUP_SPACING * 0.2
    ax.set_xlim(xticks[0] - x_margin, xticks[-1] + x_margin)

    suffix = "_log" if USE_LOG_SCALE else ""
    out_path = script_dir / f"./barplot_attestation_cycle_time_by_robot_breakdown_v2_{mode}{suffix}.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved plot to: {out_path}")
    plt.close(fig)

if __name__ == "__main__":
    main()