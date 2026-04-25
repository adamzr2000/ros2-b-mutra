#!/usr/bin/env python3
"""
v4: N-aggregated breakdown — robots grouped by fleet size.
    4 bars per N group:
      Robot Prover | Robot Verifier || SECaaS Oracle | SECaaS Verifier
    Segments follow v3 style (role-family colors, fine-grained breakdown).
    Error bars on bar totals show inter-robot std for Robot bars.
"""

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

USE_LOG_SCALE = False
LOG_MIN_Y = 1e-4
SHOW_STD = True
SHOW_VALUE_LABELS = False

FONT_SCALE = 1.5
FIG_SIZE = (14, 6)
BAR_WIDTH    = 0.28
INNER_GAP    = 0.05   # between bars within same sub-group (Robot / SECaaS)
INTER_GAP    = 0.14   # extra gap between Robot pair and SECaaS pair
GROUP_SPACING = 2.0   # center-to-center between N groups

TEXT_THRESHOLD_S  = 0.5
EPS = 1e-3
COLOR_SATURATION  = 0.9
MS_THRESHOLD_S    = 0.1
LEGEND_FONT_SCALE = 0.85
IN_BAR_FONT_SCALE = 0.82

SEGMENT_ORDER = [
    "prover_compute", "prover_tx", "prover_wait",
    "verifier_read", "verifier_verify", "verifier_tx",
    "verifier_reaction", "verifier_overhead",
    "oracle_creds", "oracle_db", "oracle_tx",
    "oracle_reaction", "oracle_overhead",
]

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

    required_cols = {"mode", "n_robots", "participant_group", "role", "metric", "mean_s"}
    missing = required_cols - set(df.columns)
    if missing:
        print(f"[ERR] CSV missing required columns: {missing}")
        print(f"      Found columns: {df.columns.tolist()}")
        sys.exit(1)

    for mode in MODES:
        df_mode = df[df["mode"] == mode].copy()
        if df_mode.empty:
            print(f"[WARN] No data found for mode '{mode}'")
            continue
        generate_plot(df_mode, mode, script_dir)


def generate_plot(df, mode, script_dir):

    # --- Aggregation helpers ---
    def get_agg(n, group, role, metric):
        """Returns (mean_across_participants, mean_of_within_participant_std).
        The std is the average of each participant's run-to-run std_s,
        representing measurement variability rather than inter-robot spread.
        """
        mask = (
            (df["n_robots"] == n) &
            (df["participant_group"] == group) &
            (df["role"] == role) &
            (df["metric"] == metric)
        )
        sub = df[mask]
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

    n_values = sorted(df["n_robots"].unique().tolist())

    # --- Colors (same as v3) ---
    prover_palette   = sns.color_palette("Blues",   n_colors=3)
    verifier_palette = sns.color_palette("Greens",  n_colors=5)
    oracle_palette   = sns.color_palette("Oranges", n_colors=5)

    COLOR = {
        "prover_compute": prover_palette[0],
        "prover_tx":      prover_palette[1],
        "prover_wait":    prover_palette[2],
        "verifier_read":     verifier_palette[0],
        "verifier_verify":   verifier_palette[1],
        "verifier_tx":       verifier_palette[2],
        "verifier_reaction": verifier_palette[3],
        "verifier_overhead": verifier_palette[4],
        "oracle_creds":    oracle_palette[0],
        "oracle_db":       oracle_palette[1],
        "oracle_tx":       oracle_palette[2],
        "oracle_reaction": oracle_palette[3],
        "oracle_overhead": oracle_palette[4],
    }

    # --- Segment builders ---
    def build_prover_segments(n):
        total = get_mean(n, "Robot", "prover", "total_lifecycle")
        e2e   = get_mean(n, "Robot", "prover", "e2e_blockchain")
        tx    = get_mean(n, "Robot", "prover", "evidence_call")
        compute = max(0.0, total - e2e)
        wait    = max(0.0, e2e - tx)
        return [
            ("prover_compute", compute),
            ("prover_tx",      tx),
            ("prover_wait",    wait),
        ]

    def build_robot_verifier_segments(n):
        total    = get_mean(n, "Robot", "verifier", "total_lifecycle")
        reaction = get_mean(n, "Robot", "verifier", "reaction_time")
        read_v   = get_mean(n, "Robot", "verifier", "signatures_fetch")
        comp_v   = get_mean(n, "Robot", "verifier", "verify_compute")
        write_v  = get_mean(n, "Robot", "verifier", "result_call")
        overhead = max(0.0, total - (reaction + read_v + comp_v + write_v))
        return [
            ("verifier_read",     read_v),
            ("verifier_verify",   comp_v),
            ("verifier_tx",       write_v),
            ("verifier_reaction", reaction),
            ("verifier_overhead", overhead),
        ]

    def build_oracle_segments(n):
        total    = get_mean(n, "SECaaS", "oracle", "total_lifecycle")
        reaction = get_mean(n, "SECaaS", "oracle", "reaction_time")
        creds    = get_mean(n, "SECaaS", "oracle", "prover_addr_fetch")
        db       = get_mean(n, "SECaaS", "oracle", "db_lookup")
        tx       = get_mean(n, "SECaaS", "oracle", "ref_signature_call")
        overhead = max(0.0, total - reaction - creds - db - tx)
        return [
            ("oracle_creds",    creds),
            ("oracle_db",       db),
            ("oracle_tx",       tx),
            ("oracle_reaction", reaction),
            ("oracle_overhead", overhead),
        ]

    def build_secaas_verifier_segments(n):
        total    = get_mean(n, "SECaaS", "verifier", "total_lifecycle")
        reaction = get_mean(n, "SECaaS", "verifier", "reaction_time")
        read_v   = get_mean(n, "SECaaS", "verifier", "signatures_fetch")
        comp_v   = get_mean(n, "SECaaS", "verifier", "verify_compute")
        write_v  = get_mean(n, "SECaaS", "verifier", "result_call")
        overhead = max(0.0, total - (reaction + read_v + comp_v + write_v))
        return [
            ("verifier_read",     read_v),
            ("verifier_verify",   comp_v),
            ("verifier_tx",       write_v),
            ("verifier_reaction", reaction),
            ("verifier_overhead", overhead),
        ]

    # --- Theme ---
    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)

    fig, ax = plt.subplots(figsize=FIG_SIZE)

    tick_label_size = plt.rcParams.get("xtick.labelsize")
    if isinstance(tick_label_size, (int, float)):
        annotation_font_size = max(8, tick_label_size * 0.9)
        legend_font_size     = max(8, tick_label_size * LEGEND_FONT_SCALE)
        in_bar_font_size     = max(7, tick_label_size * IN_BAR_FONT_SCALE)
    else:
        annotation_font_size = "small"
        legend_font_size     = "small"
        in_bar_font_size     = "x-small"

    used_keys       = set()
    xticks          = []
    xlabels         = []
    subgroup_centers = []  # (x_center, label) → "Robot" / "SECaaS"
    group_centers   = []   # (x_center, label) → "N=X"

    # --- Draw function ---
    def draw_stack(x_pos, segments, total_std=0.0):
        def fmt(s):
            return f"{s * 1000:.0f} ms" if s < MS_THRESHOLD_S else f"{s:.2f} s"

        bottom = LOG_MIN_Y if USE_LOG_SCALE else 0.0
        for key, val in segments:
            if val <= EPS:
                continue
            used_keys.add(key)
            ax.bar(x_pos, val, bottom=bottom, width=BAR_WIDTH,
                   color=desaturate(COLOR[key], COLOR_SATURATION),
                   edgecolor="black", linewidth=1.0, zorder=3)
            threshold = TEXT_THRESHOLD_S if not USE_LOG_SCALE else 0.05
            if SHOW_VALUE_LABELS and val > threshold:
                ax.text(x_pos, bottom + val / 2, fmt(val),
                        ha="center", va="center",
                        color="white" if val > 0.5 else "black",
                        fontsize=in_bar_font_size, fontweight="normal", zorder=4)
            bottom += val

        # Error bar on total
        if SHOW_STD and total_std > 0:
            ax.errorbar(x_pos, bottom, yerr=total_std,
                        fmt="none", color="black", capsize=4,
                        linewidth=1.5, zorder=5)

        shown_std = total_std if (SHOW_STD and total_std > 0) else 0.0
        top_offset = bottom * 1.25 if USE_LOG_SCALE else bottom + shown_std + 0.05
        if SHOW_VALUE_LABELS:
            ax.text(x_pos, top_offset, fmt(bottom),
                    ha="center", va="bottom", color="black",
                    fontsize=annotation_font_size, fontweight="normal")

    # --- Bar position layout ---
    step        = BAR_WIDTH + INNER_GAP
    half_pair   = step / 2
    pair_offset = half_pair + INTER_GAP / 2 + BAR_WIDTH / 2

    segment_rows = []
    total_rows   = []

    for i, n in enumerate(n_values):
        gc = i * GROUP_SPACING

        x1 = gc - pair_offset - half_pair   # Robot Prover
        x2 = gc - pair_offset + half_pair   # Robot Verifier
        x3 = gc + pair_offset - half_pair   # SECaaS Oracle
        x4 = gc + pair_offset + half_pair   # SECaaS Verifier

        xticks.extend([x1, x2, x3, x4])
        xlabels.extend(["Prover", "Verifier", "Oracle", "Verifier"])

        subgroup_centers.append(((x1 + x2) / 2, "Robot"))
        subgroup_centers.append(((x3 + x4) / 2, "SECaaS"))
        group_centers.append((gc, f"N={n}"))

        _, std_rp = get_agg(n, "Robot",  "prover",   "total_lifecycle")
        _, std_rv = get_agg(n, "Robot",  "verifier", "total_lifecycle")
        _, std_so = get_agg(n, "SECaaS", "oracle",   "total_lifecycle")
        _, std_sv = get_agg(n, "SECaaS", "verifier", "total_lifecycle")

        segs_rp = build_prover_segments(n)
        segs_rv = build_robot_verifier_segments(n)
        segs_so = build_oracle_segments(n)
        segs_sv = build_secaas_verifier_segments(n)

        draw_stack(x1, segs_rp, std_rp)
        draw_stack(x2, segs_rv, std_rv)
        draw_stack(x3, segs_so, std_so)
        draw_stack(x4, segs_sv, std_sv)

        for bar_label, segs in [("Robot-Prover", segs_rp), ("Robot-Verifier", segs_rv),
                                  ("SECaaS-Oracle", segs_so), ("SECaaS-Verifier", segs_sv)]:
            total = 0.0
            for key, val in segs:
                if val <= EPS:
                    continue
                total += val
                segment_rows.append({"n_robots": n, "bar": bar_label,
                                     "segment": LABELS.get(key, key), "value_s": val})
            total_rows.append({"n_robots": n, "bar": bar_label, "total_s": total})

    # Console output
    if segment_rows:
        seg_df = pd.DataFrame(segment_rows)
        seg_df["value_s"] = seg_df["value_s"].map(lambda x: round(float(x), 6))
        print(f"\n[INFO] mode={mode} — segment values (seconds):")
        print(seg_df.to_string(index=False))
    if total_rows:
        tot_df = pd.DataFrame(total_rows)
        tot_df["total_s"] = tot_df["total_s"].map(lambda x: round(float(x), 6))
        print(f"\n[INFO] mode={mode} — bar totals (seconds):")
        print(tot_df.to_string(index=False))

    # --- Axes styling ---
    ax.set_xticks(xticks)
    ax.set_xticklabels(xlabels, rotation=0, ha="center")

    for x_c, lbl in subgroup_centers:
        ax.text(x_c, -0.10, lbl,
                ha="center", va="top",
                transform=ax.get_xaxis_transform(),
                fontsize=tick_label_size, fontstyle="italic")

    for x_c, lbl in group_centers:
        ax.text(x_c, -0.18, lbl,
                ha="center", va="top",
                transform=ax.get_xaxis_transform(),
                fontsize=tick_label_size)

    if USE_LOG_SCALE:
        ax.set_yscale("log")
        ax.set_ylabel("Time (s) [Log Scale]")
        ax.set_ylim(LOG_MIN_Y, ax.get_ylim()[1] * 15)
    else:
        ax.set_ylabel("Time (s)")
        ax.set_ylim(0, ax.get_ylim()[1] * 1.25)

    ax.set_axisbelow(True)
    ax.grid(axis="y", linestyle="-", linewidth=1.0, alpha=0.8)
    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_color("black")
        ax.spines[side].set_linewidth(1.0)

    # --- Legend ---
    legend_keys = [k for k in SEGMENT_ORDER if k in used_keys]
    patches = [
        mpatches.Patch(facecolor=COLOR[k], edgecolor="black", label=LABELS[k])
        for k in legend_keys
    ]
    ax.legend(handles=patches, loc="upper right",
              fontsize=legend_font_size,
              frameon=True, framealpha=0.9, fancybox=True,
              ncol=3, handlelength=1.0, handletextpad=0.4,
              labelspacing=0.3, borderpad=0.3)

    plt.tight_layout()
    plt.subplots_adjust(bottom=0.28)

    x_margin = BAR_WIDTH / 2 + GROUP_SPACING * 0.2
    ax.set_xlim(xticks[0] - x_margin, xticks[-1] + x_margin)

    suffix = "_log" if USE_LOG_SCALE else ""
    out_path = script_dir / f"./barplot_attestation_cycle_time_by_robot_breakdown_v2N_{mode}{suffix}.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved plot to: {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
