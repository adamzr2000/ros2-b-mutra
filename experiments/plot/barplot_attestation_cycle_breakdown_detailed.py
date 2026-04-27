#!/usr/bin/env python3
"""
Detailed breakdown of attestation cycle time — fine-grained segment view.

Same dual-panel layout as barplot_attestation_cycle_breakdown_claude.py:
  Left panel  (seconds):      Robot Prover — 3 segments
  Right panel (milliseconds): Secondary roles — per-operation segments

  Continuous: Robot Verifier | SECaaS Oracle
  Startup:    SECaaS Verifier (oracle DB lookup shown as distinct segment on top)

Hatch pattern encodes participant: Robot = solid, SECaaS = ///.
One PDF per mode.
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import seaborn as sns
from pathlib import Path
import sys

INPUT_FILE    = "../data/attestation-times/_summary/durations_per_run.csv"
MODES         = ["startup", "continuous"]
N_VALUES      = [4, 8, 16, 32, 64]

FONT_SCALE    = 1.6
BAR_WIDTH     = 0.32
BAR_WIDTH_L   = 0.52
BAR_GAP       = 0.12
BAR_GAP_CONT  = 0.05
GROUP_SPACING = 1.0
HEADROOM      = 1.10
SEC_TO_MS     = 1000.0
EPS           = 1e-4

SECONDARY = {
    "startup": [
        ("SECaaS", "verifier", "SECaaS", "Verifier"),
    ],
    "continuous": [
        ("Robot",  "verifier", "Robot",  "Verifier"),
        ("SECaaS", "oracle",   "SECaaS", "Oracle"),
    ],
}

# ── Segment colors ─────────────────────────────────────────────────────────────
# Prover (left panel) — consistent with claude version's 2-color palette
C_COMPUTE = "#993333"   # dark red   — local computation (SHA-256, measurement)
C_TX_SUB  = "#CC7722"   # amber      — transaction submission
C_BC_WAIT = "#336699"   # steel blue — blockchain wait

# Verifier (right panel) — teal/cyan palette
C_VER_READ     = "#4DD0E1"
C_VER_VERIFY   = "#00838F"
C_VER_TX       = "#006064"
C_VER_OVERHEAD = "#B0BEC5"

# Oracle (right panel) — orange palette
C_ORA_CREDS    = "#FFB74D"
C_ORA_DB       = "#EF6C00"
C_ORA_TX       = "#BF360C"
C_ORA_OVERHEAD = "#B0BEC5"

SEGMENT_LABELS = {
    "compute":      "Local computation (SHA-256, measurement)",
    "tx_sub":       "Submit evidence (tx)",
    "bc_wait":      "Blockchain wait",
    "ver_read":     "Fetch signatures (blockchain call)",
    "ver_verify":   "Hash comparison",
    "ver_tx":       "Submit verification result (tx)",
    "ver_overhead": "Verifier overhead",
    "ora_creds":    "Fetch prover address (blockchain call)",
    "ora_db":       "DB lookup (reference measurement)",
    "ora_tx":       "Submit reference measurement (tx)",
    "ora_overhead": "Oracle overhead",
}

SEGMENT_COLORS = {
    "compute":      C_COMPUTE,
    "tx_sub":       C_TX_SUB,
    "bc_wait":      C_BC_WAIT,
    "ver_read":     C_VER_READ,
    "ver_verify":   C_VER_VERIFY,
    "ver_tx":       C_VER_TX,
    "ver_overhead": C_VER_OVERHEAD,
    "ora_creds":    C_ORA_CREDS,
    "ora_db":       C_ORA_DB,
    "ora_tx":       C_ORA_TX,
    "ora_overhead": C_ORA_OVERHEAD,
}

# Display order in legend
SEGMENT_ORDER = [
    "compute", "tx_sub", "bc_wait",
    "ver_read", "ver_verify", "ver_tx", "ver_overhead",
    "ora_creds", "ora_db", "ora_tx", "ora_overhead",
]


# ── data helpers ───────────────────────────────────────────────────────────────

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
    # One system-level mean per run, then mean ± std across the 5 independent runs.
    run_means = sub.groupby("run")["run_mean_s"].mean()
    return float(run_means.mean()), float(run_means.std(ddof=1))


def _mean(df, n, group, role, metric):
    return _get_agg(df, n, group, role, metric)[0]


# ── segment builders ───────────────────────────────────────────────────────────

def _prover_segs(df, n):
    """Robot Prover: 3 non-overlapping segments.

    dur_prover_e2e starts at p_send_evidence_START (same as evidence_call),
    so evidence_call ⊂ e2e. Correct partition:
      compute  = total - e2e          (local overhead: SHA-256 + scheduling)
      tx_sub   = evidence_call        (TX submission call)
      bc_wait  = e2e - evidence_call  (pure blockchain polling after TX sent)
    """
    total, std = _get_agg(df, n, "Robot", "prover", "total_lifecycle")
    e2e = _mean(df, n, "Robot", "prover", "e2e_blockchain")
    tx  = _mean(df, n, "Robot", "prover", "evidence_call")
    compute = max(0.0, total - e2e)
    bc_wait = max(0.0, e2e - tx)
    return [
        ("compute", compute, C_COMPUTE),
        ("tx_sub",  tx,      C_TX_SUB),
        ("bc_wait", bc_wait, C_BC_WAIT),
    ], std


def _verifier_segs(df, n, group):
    """Robot or SECaaS Verifier: 4 segments (sub-components of total_lifecycle only)."""
    total, std = _get_agg(df, n, group, "verifier", "total_lifecycle")
    read_v  = _mean(df, n, group, "verifier", "signatures_fetch")
    comp_v  = _mean(df, n, group, "verifier", "verify_compute")
    write_v = _mean(df, n, group, "verifier", "result_call")
    overhead = max(0.0, total - (read_v + comp_v + write_v))
    return [
        ("ver_read",     read_v,   C_VER_READ),
        ("ver_verify",   comp_v,   C_VER_VERIFY),
        ("ver_tx",       write_v,  C_VER_TX),
        ("ver_overhead", overhead, C_VER_OVERHEAD),
    ], std


def _oracle_segs(df, n):
    """SECaaS Oracle: 4 segments (sub-components of total_lifecycle only)."""
    total, std = _get_agg(df, n, "SECaaS", "oracle", "total_lifecycle")
    creds    = _mean(df, n, "SECaaS", "oracle", "prover_addr_fetch")
    db       = _mean(df, n, "SECaaS", "oracle", "db_lookup")
    tx       = _mean(df, n, "SECaaS", "oracle", "ref_signature_call")
    overhead = max(0.0, total - (creds + db + tx))
    return [
        ("ora_creds",    creds,    C_ORA_CREDS),
        ("ora_db",       db,       C_ORA_DB),
        ("ora_tx",       tx,       C_ORA_TX),
        ("ora_overhead", overhead, C_ORA_OVERHEAD),
    ], std


# ── drawing ────────────────────────────────────────────────────────────────────

def _draw_stack(ax, x, segments, std, bw, scale=1.0, hatch=""):
    """Draw a multi-segment stacked bar. Returns top of bar (including std)."""
    bottom = 0.0
    for _, val, color in segments:
        v = val * scale
        if v > EPS:
            ax.bar(x, v, bottom=bottom, width=bw,
                   color=color, edgecolor="black", linewidth=0.8,
                   hatch=hatch, zorder=3)
            bottom += v
    if std * scale > 0:
        ax.errorbar(x, bottom, yerr=std * scale,
                    fmt="none", color="black", capsize=3.5,
                    linewidth=1.2, zorder=5)
    return bottom + std * scale


# ── main plot ──────────────────────────────────────────────────────────────────

def generate_plot(df, mode, script_dir):
    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)
    plt.rcParams.update({"font.family": "serif"})

    n_values  = sorted(df["n_robots"].unique().tolist())
    secondary = SECONDARY[mode]
    n_sec     = len(secondary)

    gap     = BAR_GAP_CONT if mode == "continuous" else BAR_GAP
    step    = BAR_WIDTH + gap
    offsets = [(j - (n_sec - 1) / 2.0) * step for j in range(n_sec)]

    PARTICIPANT_HATCH = {"Robot": "", "SECaaS": "///"}
    used_keys = set()

    fig, (ax_l, ax_r) = plt.subplots(
        1, 2, figsize=(11, 4.6),
        gridspec_kw={"width_ratios": [1, 1], "wspace": 0.22},
    )

    # ── LEFT PANEL: Robot Prover ───────────────────────────────────────────────
    max_top_l = 0.0
    for i, n in enumerate(n_values):
        segs, std = _prover_segs(df, n)
        top = _draw_stack(ax_l, i * GROUP_SPACING, segs, std, BAR_WIDTH_L)
        max_top_l = max(max_top_l, top)
        for key, val, _ in segs:
            if val > EPS:
                used_keys.add(key)

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
    max_top_r = 0.0
    for i, n in enumerate(n_values):
        for j, (group, role, *_) in enumerate(secondary):
            x     = i * GROUP_SPACING + offsets[j]
            bw    = BAR_WIDTH_L if n_sec == 1 else BAR_WIDTH
            hatch = PARTICIPANT_HATCH.get(group, "")

            if role == "verifier":
                segs, std = _verifier_segs(df, n, group)
                # Startup: SECaaS also performs oracle DB lookup — absorb into overhead
                if mode == "startup" and group == "SECaaS":
                    db_lkp = _mean(df, n, "SECaaS", "oracle", "db_lookup")
                    # Replace overhead entry with overhead + db_lookup
                    segs = segs[:-1] + [
                        ("ora_db",       db_lkp,                    C_ORA_DB),
                        ("ver_overhead", segs[-1][1],                C_VER_OVERHEAD),
                    ]
            elif role == "oracle":
                segs, std = _oracle_segs(df, n)
            else:
                continue

            top = _draw_stack(ax_r, x, segs, std, bw, scale=SEC_TO_MS, hatch=hatch)
            max_top_r = max(max_top_r, top)
            for key, val, _ in segs:
                if val > EPS:
                    used_keys.add(key)

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

    r_title = "  |  ".join(f"{l1} ({l2})" for _, _, l1, l2 in secondary)
    ax_r.set_title(r_title, pad=7)

    # ── Shared legend (bottom centre) ─────────────────────────────────────────
    legend_handles = [
        mpatches.Patch(facecolor=SEGMENT_COLORS[k], edgecolor="black",
                       label=SEGMENT_LABELS[k])
        for k in SEGMENT_ORDER if k in used_keys
    ]
    n_cols = 2 if len(legend_handles) <= 6 else 3
    fig.legend(
        handles=legend_handles,
        loc="upper center", bbox_to_anchor=(0.5, -0.01),
        ncol=n_cols, frameon=True, framealpha=0.9, fancybox=True,
        fontsize=plt.rcParams.get("legend.fontsize", 9),
    )

    plt.subplots_adjust(top=0.92, bottom=0.18)

    out_path = script_dir / f"barplot_attestation_cycle_breakdown_detailed_{mode}.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved: {out_path}")

    # Print values for reference
    print(f"\n=== Detailed values ({mode}) ===")
    for n in n_values:
        segs, std = _prover_segs(df, n)
        total = sum(v for _, v, _ in segs)
        print(f"  Prover N={n:>3}: "
              + "  ".join(f"{k}={v:.4f}s" for k, v, _ in segs)
              + f"  total={total:.4f}s  std={std:.4f}s")

    plt.close(fig)


# ── entry point ────────────────────────────────────────────────────────────────

def main():
    script_dir = Path(__file__).parent.resolve()
    csv_path   = (script_dir / INPUT_FILE).resolve()

    if not csv_path.exists():
        print(f"[ERR] CSV not found: {csv_path}")
        sys.exit(1)

    df = pd.read_csv(csv_path)
    df = df[df["n_robots"].isin(N_VALUES)]

    required = {"mode", "n_robots", "participant_group", "role", "metric", "run_mean_s"}
    missing  = required - set(df.columns)
    if missing:
        print(f"[ERR] Missing columns: {missing}")
        sys.exit(1)

    for mode in MODES:
        sub = df[df["mode"] == mode].copy()
        if sub.empty:
            print(f"[WARN] No data for mode '{mode}'")
            continue
        generate_plot(sub, mode, script_dir)


if __name__ == "__main__":
    main()
