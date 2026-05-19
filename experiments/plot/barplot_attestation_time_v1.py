#!/usr/bin/env python3
"""
Dual-panel breakdown of attestation cycle time by fleet size.

Left panel  (full scale): Robot Prover — the dominant role.
Right panel (zoomed):     Non-zero secondary roles for each mode:
  Startup    : SECaaS Verifier | SECaaS Oracle
  Continuous : Robot Verifier  | SECaaS Oracle

X-axis: N values (4, 10, 15, 20, 25). Bars stacked:
  bottom = Off-chain (orange) | top = Blockchain (blue).
Error bars = std across 5 independent run-level means.
One PDF per mode.
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import seaborn as sns
from pathlib import Path
import sys

INPUT_STARTUP    = "../data/attestation-times/_summary/durations_per_run_startup.csv"
INPUT_CONTINUOUS = "../data/attestation-times/_summary/durations_per_run_{VARIANT}.csv"
MODES      = ["startup", "continuous"]
VARIANT    = "rr"   # ← continuous-mode variant to plot
SSP_MS      = 20000           # sidecar sleep period (ms)
ITERQ     = 1            # rolling-hash queue depth
CPU_LIMIT  = 0.4          # sidecar CPU limit
N_VALUES   = [4, 8, 16, 32, 64]

FONT_SCALE    = 1.6
BAR_WIDTH     = 0.32
BAR_WIDTH_L   = 0.52   # wider bar for left panel (1 bar per group)
BAR_GAP       = 0.12   # gap between the 2 bars in the right panel
BAR_GAP_CONT  = 0.05   # tighter gap for the 2 continuous-mode bars
GROUP_SPACING = 1.0    # centre-to-centre between N-groups (both panels)
HEADROOM      = 1.10   # top y-limit = max_bar_top * HEADROOM
SEC_TO_MS     = 1000.0

# Secondary roles per mode: (participant_group, role_key, label_line1, label_line2)
# Startup: only SECaaS Verifier shown; oracle db_lookup is absorbed into its off-chain
# component since both roles are co-located and the oracle→blockchain round-trip
# is an internal implementation detail, not a separate architectural role.
SECONDARY = {
    "startup": [
        ("SECaaS", "verifier", "SECaaS", "Verifier"),
    ],
    "continuous": [
        ("Robot",  "verifier", "Robot",  "Verifier"),
        ("SECaaS", "oracle",   "SECaaS", "Oracle"),
    ],
}


# ── helpers ────────────────────────────────────────────────────────────────────

def _get_agg(df, n, group, role, metric):
    mask = (
        (df["n_robots"] == n) &
        (df["participant_group"] == group) &
        (df["role"] == role) &
        (df["metric"] == metric)
    )
    sub = df[mask]
    if sub.empty:
        return 0.0, 0.0
    run_means = sub.groupby("run")["run_mean_s"].mean()
    return float(run_means.mean()), float(run_means.std(ddof=1))


def _bc_off(df, n, group, role):
    """Return (bc_s, off_s, total_s, std_s) with blockchain/off-chain split."""
    total, std = _get_agg(df, n, group, role, "total_lifecycle")
    gm = lambda m: _get_agg(df, n, group, role, m)[0]

    if role == "prover":
        bc  = gm("e2e_blockchain")
        off = max(0.0, total - bc)
    elif role == "verifier":
        off = gm("verify_compute")
        bc  = max(0.0, total - off)
    elif role == "oracle":
        off = gm("db_lookup")
        bc  = max(0.0, total - off)
    else:
        bc, off = total, 0.0

    return bc, off, total, std


def _draw_bar(ax, x, bc, off, std, bw, c_bc, c_off, hatch=""):
    """Draw a stacked bar; return top of bar."""
    bottom = 0.0
    for val, col in [(off, c_off), (bc, c_bc)]:
        if val > 1e-4:
            ax.bar(x, val, bottom=bottom, width=bw,
                   color=col, edgecolor="black", linewidth=0.8,
                   hatch=hatch, zorder=3)
            bottom += val
    if std > 0:
        ax.errorbar(x, bottom, yerr=std,
                    fmt="none", color="black", capsize=3.5,
                    linewidth=1.2, zorder=5)
    return bottom + std


# ── main plot function ─────────────────────────────────────────────────────────

def generate_plot(df, mode, script_dir):
    c_bc  = "#336699"   # steel blue — blockchain
    c_off = "#993333"   # dark red   — local computation

    LBL_BC  = "Blockchain operations (tx, state queries)"
    LBL_OFF = "Local computation (SHA-256, DB lookup, measurement comparison)"

    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)
    plt.rcParams.update({"font.family": "serif"})

    n_values  = sorted(df["n_robots"].unique().tolist())
    secondary = SECONDARY[mode]
    n_sec     = len(secondary)

    left_rows = []
    right_rows = []

    # bar offsets within a right-panel group (centred at 0)
    gap     = BAR_GAP_CONT if mode == "continuous" else BAR_GAP
    step    = BAR_WIDTH + gap
    offsets = [(j - (n_sec - 1) / 2.0) * step for j in range(n_sec)]

    # ── figure & axes ──────────────────────────────────────────────────────────
    fig, (ax_l, ax_r) = plt.subplots(
        1, 2, figsize=(11, 4.6),
        gridspec_kw={"width_ratios": [1, 1], "wspace": 0.22},
    )

    # ── LEFT PANEL: Robot Prover ───────────────────────────────────────────────
    max_top_l = 0.0
    for i, n in enumerate(n_values):
        bc, off, _, std = _bc_off(df, n, "Robot", "prover")
        top = _draw_bar(ax_l, i * GROUP_SPACING, bc, off, std, BAR_WIDTH_L, c_bc, c_off)
        max_top_l = max(max_top_l, top)
        left_rows.append({
            "n": n,
            "off_s": off,
            "bc_s": bc,
            "total_s": off + bc,
            "std_s": std,
        })

    ax_l.set_xticks([i * GROUP_SPACING for i in range(len(n_values))])
    ax_l.set_xticklabels([str(n) for n in n_values])
    ax_l.set_xlabel("Number of robots (N)")
    ax_l.set_ylabel("Time (s)")
    ax_l.set_title("Robot (Prover)", pad=7)
    ax_l.set_ylim(0, max_top_l * HEADROOM)
    ax_l.set_xlim(-0.55 * GROUP_SPACING, (len(n_values) - 0.45) * GROUP_SPACING)
    ax_l.tick_params(axis="both", which="major", length=6, width=1.0, direction="out")
    ax_l.grid(axis="y", which="major", linestyle="-", linewidth=0.7, alpha=0.75)
    ax_l.set_axisbelow(True)

    # ── RIGHT PANEL: Secondary roles ───────────────────────────────────────────
    # Hatch encodes participant: Robot = solid, SECaaS = ///
    PARTICIPANT_HATCH = {"Robot": "", "SECaaS": "///"}

    max_top_r = 0.0
    for i, n in enumerate(n_values):
        for j, (group, role, *_) in enumerate(secondary):
            x   = i * GROUP_SPACING + offsets[j]
            bc, off, _, std = _bc_off(df, n, group, role)
            # Startup: absorb oracle DB lookup into SECaaS Verifier off-chain
            if mode == "startup" and group == "SECaaS" and role == "verifier":
                off += _get_agg(df, n, "SECaaS", "oracle", "db_lookup")[0]
            # Use wider bar when only one secondary role (mirrors left panel)
            bw = BAR_WIDTH_L if n_sec == 1 else BAR_WIDTH
            top = _draw_bar(ax_r, x,
                            bc * SEC_TO_MS,
                            off * SEC_TO_MS,
                            std * SEC_TO_MS,
                            bw, c_bc, c_off,
                            hatch=PARTICIPANT_HATCH.get(group, ""))
            max_top_r = max(max_top_r, top)
            right_rows.append({
                "n": n,
                "group": group,
                "role": role,
                "off_ms": off * SEC_TO_MS,
                "bc_ms": bc * SEC_TO_MS,
                "total_ms": (off + bc) * SEC_TO_MS,
                "std_ms": std * SEC_TO_MS,
            })

    # N= group labels on the x-axis tick positions
    group_xs = [i * GROUP_SPACING for i in range(len(n_values))]
    ax_r.set_xticks(group_xs)
    ax_r.set_xticklabels([str(n) for n in n_values])

    ax_r.set_xlabel("Number of robots (N)")
    ax_r.set_ylabel("Time (ms)")
    ax_r.set_ylim(0, max(max_top_r * HEADROOM, 40.0))
    ax_r.set_xlim(
        group_xs[0]  - GROUP_SPACING * 0.55,
        group_xs[-1] + GROUP_SPACING * 0.55,
    )
    ax_r.tick_params(axis="both", which="major", length=6, width=1.0, direction="out")
    ax_r.grid(axis="y", which="major", linestyle="-", linewidth=0.7, alpha=0.75)
    ax_r.set_axisbelow(True)

    # Right panel title: explicit role names, hatch key in parentheses
    r_title = "  |  ".join(
        f"{l1} ({l2})"
        for _, _, l1, l2 in secondary
    )
    ax_r.set_title(r_title, pad=7)

    # ── Shared legend (top centre, above subplots) ─────────────────────────────
    legend_handles = [
        mpatches.Patch(facecolor=c_bc,  edgecolor="black", label=LBL_BC),
        mpatches.Patch(facecolor=c_off, edgecolor="black", label=LBL_OFF),
    ]
    fig.legend(
        handles=legend_handles,
        loc="upper center", bbox_to_anchor=(0.5, -0.01),
        ncol=1, frameon=True, framealpha=0.9, fancybox=True,
        fontsize=plt.rcParams.get("legend.fontsize", 9),
    )

    # fig.suptitle(
    #     f"Attestation cycle time — {mode.capitalize()} mode",
    #     y=-0.04, fontsize=plt.rcParams.get("axes.titlesize", 11),
    # )

    plt.subplots_adjust(top=0.92, bottom=0.18)

    out_path = script_dir / f"barplot_attestation_time_v1_{mode}.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved: {out_path}")

    print(f"\n=== Plot values ({mode}) ===")
    print("Left panel: Robot Prover (seconds)")
    for row in left_rows:
        print(
            f"  N={row['n']:>3} | off={row['off_s']:.6f}s | "
            f"bc={row['bc_s']:.6f}s | total={row['total_s']:.6f}s | "
            f"std={row['std_s']:.6f}s"
        )

    print("Right panel: Secondary roles (milliseconds)")
    for row in right_rows:
        print(
            f"  N={row['n']:>3} | {row['group']} {row['role']} | "
            f"off={row['off_ms']:.3f}ms | bc={row['bc_ms']:.3f}ms | "
            f"total={row['total_ms']:.3f}ms | std={row['std_ms']:.3f}ms"
        )

    plt.close(fig)


# ── entry point ────────────────────────────────────────────────────────────────

def main():
    script_dir = Path(__file__).parent.resolve()

    files = {
        "startup":    INPUT_STARTUP,
        "continuous": INPUT_CONTINUOUS.format(VARIANT=VARIANT),
    }

    for mode in MODES:
        csv_path = (script_dir / files[mode]).resolve()
        if not csv_path.exists():
            print(f"[WARN] CSV not found, skipping {mode}: {csv_path}")
            continue
        df = pd.read_csv(csv_path)
        df = df[df["n_robots"].isin(N_VALUES)]
        if mode == "continuous":
            sub = df[
                (df["ssp_ms"]     == SSP_MS) &
                (df["iterq"]      == ITERQ) &
                (df["cpu_limit"]  == CPU_LIMIT)
            ].copy()
        else:
            sub = df.copy()
        if sub.empty:
            print(f"[WARN] No data for mode='{mode}' contract='{VARIANT}' SSP={SSP_MS}ms ITERQ={ITERQ} cpu={CPU_LIMIT}")
            continue
        generate_plot(sub, mode, script_dir)


if __name__ == "__main__":
    main()
