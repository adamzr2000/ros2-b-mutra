#!/usr/bin/env python3
"""
Final attestation cycle figure + CSV export.

Figure (barplot_attestation_cycle_breakdown_final.pdf):
  Left panel  — Robot Prover, startup mode
  Right panel — Robot Prover, continuous mode
  Shared y-axis. 3 segments: local computation / submit evidence (tx) / blockchain wait.

CSV (attestation_cycle_breakdown_final.csv):
  Flat long-format table — all roles, all N, both modes, per-operation mean ± std (ms).
  std_ms is provided only for the "total" operation row of each role; NaN otherwise.
  Designed for direct import by LaTeX pgfplotstable.
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import seaborn as sns
from pathlib import Path
import sys
import math

INPUT_FILE = "../data/attestation-times/_summary/durations_summary.csv"
N_VALUES   = [4, 8, 16, 32, 64]

FONT_SCALE    = 1.8
BAR_WIDTH     = 0.52
GROUP_SPACING = 1.0
HEADROOM      = 1.10
SEC_TO_MS     = 1000.0
EPS           = 1e-4

C_COMPUTE = "#993333"   # dark red   — local computation
C_BC_OPS  = "#336699"   # steel blue — blockchain operations

SEG_ORDER  = ["local_computation", "blockchain_operations"]
SEG_LABELS = {
    "local_computation":   "Local computation (SHA-256)",
    "blockchain_operations": "Blockchain operations (tx, state queries)",
}
SEG_COLORS = {
    "local_computation":   C_COMPUTE,
    "blockchain_operations": C_BC_OPS,
}


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
    mean_v = float(sub["mean_s"].mean())
    std_v  = (float(sub["std_s"].dropna().mean())
              if "std_s" in sub.columns and not sub["std_s"].dropna().empty
              else 0.0)
    return mean_v, std_v


def _mean(df, n, group, role, metric):
    return _get_agg(df, n, group, role, metric)[0]


def _prover_segs(df, n):
    """Return (segments, total_s, std_s) for Robot Prover."""
    total, std = _get_agg(df, n, "Robot", "prover", "total_lifecycle")
    e2e     = _mean(df, n, "Robot", "prover", "e2e_blockchain")
    tx      = _mean(df, n, "Robot", "prover", "evidence_call")
    compute = max(0.0, total - e2e - tx)
    segs = [
        ("local_computation",    compute,    C_COMPUTE),
        ("blockchain_operations", e2e + tx,  C_BC_OPS),
    ]
    return segs, total, std


# ── drawing ────────────────────────────────────────────────────────────────────

def _draw_stack(ax, x, segments, std, bw):
    bottom = 0.0
    for _, val, color in segments:
        if val > EPS:
            ax.bar(x, val, bottom=bottom, width=bw,
                   color=color, edgecolor="black", linewidth=0.8, zorder=3)
            bottom += val
    if std > 0:
        ax.errorbar(x, bottom, yerr=std,
                    fmt="none", color="black", capsize=3.5,
                    linewidth=1.2, zorder=5)
    return bottom + std


# ── figure ─────────────────────────────────────────────────────────────────────

def generate_figure(df, n_values, script_dir):
    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)
    plt.rcParams.update({"font.family": "serif"})

    fig, (ax_l, ax_r) = plt.subplots(
        1, 2, figsize=(11, 4.6),
        gridspec_kw={"width_ratios": [1, 1], "wspace": 0.08},
        sharey=True,
    )

    # Shared y-limit across both modes
    max_top = 0.0
    for mode in ["startup", "continuous"]:
        sub = df[df["mode"] == mode]
        for n in n_values:
            _, total, std = _prover_segs(sub, n)
            max_top = max(max_top, total + std)
    ylim = max_top * HEADROOM

    for ax, mode, title in [
        (ax_l, "startup",    "Startup Attestation"),
        (ax_r, "continuous", "Continuous Attestation"),
    ]:
        sub = df[df["mode"] == mode]
        print(f"\n=== Robot Prover — {mode} ===")
        print(f"  {'N':>4}  {'local_comp (s)':>16}  {'bc_ops (s)':>12}  {'total (s)':>10}  {'std (s)':>9}")
        for i, n in enumerate(n_values):
            segs, total, std = _prover_segs(sub, n)
            seg_map = {k: v for k, v, _ in segs}
            lc = seg_map.get("local_computation", 0.0)
            bc = seg_map.get("blockchain_operations", 0.0)
            print(f"  N={n:>3}  {lc:>16.4f}  {bc:>12.4f}  {total:>10.4f}  {std:>9.4f}")
            _draw_stack(ax, i * GROUP_SPACING, segs, std, BAR_WIDTH)

        ax.set_xticks([i * GROUP_SPACING for i in range(len(n_values))])
        ax.set_xticklabels([str(n) for n in n_values])
        ax.set_xlabel("Number of robots (N)")
        ax.set_ylim(0, ylim)
        ax.set_xlim(-0.55 * GROUP_SPACING, (len(n_values) - 0.45) * GROUP_SPACING)
        ax.tick_params(axis="both", which="major", length=6, width=1.0, direction="out")
        ax.grid(axis="y", which="major", linestyle="-", linewidth=0.7, alpha=0.75)
        ax.set_axisbelow(True)
        ax.set_title(title, pad=7)

    ax_l.set_ylabel("Time (s)")

    legend_handles = [
        mpatches.Patch(facecolor=SEG_COLORS[k], edgecolor="black", label=SEG_LABELS[k])
        for k in SEG_ORDER
    ]
    fig.legend(
        handles=legend_handles,
        loc="upper center", bbox_to_anchor=(0.5, -0.01),
        ncol=3, frameon=True, framealpha=0.9, fancybox=True,
        fontsize=plt.rcParams.get("legend.fontsize", 9),
    )
    plt.subplots_adjust(top=0.92, bottom=0.18)

    out_path = script_dir / "barplot_attestation_cycle_breakdown_final.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved figure: {out_path}")
    plt.close(fig)


# ── CSV export ─────────────────────────────────────────────────────────────────

def generate_csv(df, n_values, script_dir):
    """
    Flat long-format CSV with columns:
      attestation, n_robots, participant, role, operation, mean_ms, std_ms

    One row per (mode, n_robots, participant, role, operation).
    std_ms is populated only for operation="total"; NaN for all sub-operations.
    """
    NaN = float("nan")
    rows = []

    def add(mode, n, participant, role, operation, mean_s, std_s=NaN):
        rows.append({
            "attestation": mode,
            "n_robots":    n,
            "participant": participant,
            "role":        role,
            "operation":   operation,
            "mean_ms":     round(mean_s * SEC_TO_MS, 3),
            "std_ms":      round(std_s  * SEC_TO_MS, 3) if not math.isnan(std_s) else NaN,
        })

    for mode in ["startup", "continuous"]:
        sub = df[df["mode"] == mode]

        for n in n_values:

            # ── Robot Prover (both modes) ──────────────────────────────────
            total, std = _get_agg(sub, n, "Robot", "prover", "total_lifecycle")
            e2e     = _mean(sub, n, "Robot", "prover", "e2e_blockchain")
            tx      = _mean(sub, n, "Robot", "prover", "evidence_call")
            compute = max(0.0, total - e2e - tx)
            add(mode, n, "Robot", "prover", "local_computation",  compute)
            add(mode, n, "Robot", "prover", "submit_evidence_tx", tx)
            add(mode, n, "Robot", "prover", "blockchain_wait",    e2e)
            add(mode, n, "Robot", "prover", "total",              total,   std)

            if mode == "startup":

                # ── SECaaS Verifier — startup (includes oracle DB lookup) ──
                vtotal, vstd = _get_agg(sub, n, "SECaaS", "verifier", "total_lifecycle")
                ver_read = _mean(sub, n, "SECaaS", "verifier", "signatures_fetch")
                ver_comp = _mean(sub, n, "SECaaS", "verifier", "verify_compute")
                ver_tx   = _mean(sub, n, "SECaaS", "verifier", "result_call")
                ver_ovhd = max(0.0, vtotal - (ver_read + ver_comp + ver_tx))
                db_lkp   = _mean(sub, n, "SECaaS", "oracle",   "db_lookup")
                combined = vtotal + db_lkp
                add(mode, n, "SECaaS", "verifier", "fetch_signatures",  ver_read)
                add(mode, n, "SECaaS", "verifier", "hash_comparison",   ver_comp)
                add(mode, n, "SECaaS", "verifier", "submit_result_tx",  ver_tx)
                add(mode, n, "SECaaS", "verifier", "verifier_overhead", ver_ovhd)
                add(mode, n, "SECaaS", "verifier", "db_lookup",         db_lkp)
                add(mode, n, "SECaaS", "verifier", "total",             combined, vstd)

            else:  # continuous

                # ── Robot Verifier — continuous ────────────────────────────
                vtotal, vstd = _get_agg(sub, n, "Robot", "verifier", "total_lifecycle")
                ver_read = _mean(sub, n, "Robot", "verifier", "signatures_fetch")
                ver_comp = _mean(sub, n, "Robot", "verifier", "verify_compute")
                ver_tx   = _mean(sub, n, "Robot", "verifier", "result_call")
                ver_ovhd = max(0.0, vtotal - (ver_read + ver_comp + ver_tx))
                add(mode, n, "Robot",  "verifier", "fetch_signatures",  ver_read)
                add(mode, n, "Robot",  "verifier", "hash_comparison",   ver_comp)
                add(mode, n, "Robot",  "verifier", "submit_result_tx",  ver_tx)
                add(mode, n, "Robot",  "verifier", "verifier_overhead", ver_ovhd)
                add(mode, n, "Robot",  "verifier", "total",             vtotal,  vstd)

                # ── SECaaS Oracle — continuous ─────────────────────────────
                ototal, ostd = _get_agg(sub, n, "SECaaS", "oracle", "total_lifecycle")
                ora_creds = _mean(sub, n, "SECaaS", "oracle", "prover_addr_fetch")
                ora_db    = _mean(sub, n, "SECaaS", "oracle", "db_lookup")
                ora_tx    = _mean(sub, n, "SECaaS", "oracle", "ref_signature_call")
                ora_ovhd  = max(0.0, ototal - (ora_creds + ora_db + ora_tx))
                add(mode, n, "SECaaS", "oracle", "fetch_prover_addr",         ora_creds)
                add(mode, n, "SECaaS", "oracle", "db_lookup",                 ora_db)
                add(mode, n, "SECaaS", "oracle", "submit_ref_measurement_tx", ora_tx)
                add(mode, n, "SECaaS", "oracle", "oracle_overhead",           ora_ovhd)
                add(mode, n, "SECaaS", "oracle", "total",                     ototal,  ostd)

    out_df   = pd.DataFrame(rows)
    out_path = script_dir / "attestation_cycle_breakdown_final.csv"
    out_df.to_csv(out_path, index=False, float_format="%.3f")
    print(f"[OK] Saved CSV:    {out_path}")
    print(f"     {len(rows)} rows  |  columns: {list(out_df.columns)}")
    return out_df


# ── entry point ────────────────────────────────────────────────────────────────

def main():
    script_dir = Path(__file__).parent.resolve()
    csv_path   = (script_dir / INPUT_FILE).resolve()

    if not csv_path.exists():
        print(f"[ERR] CSV not found: {csv_path}")
        sys.exit(1)

    df = pd.read_csv(csv_path)
    df = df[df["n_robots"].isin(N_VALUES)]

    required = {"mode", "n_robots", "participant_group", "role", "metric", "mean_s"}
    missing  = required - set(df.columns)
    if missing:
        print(f"[ERR] Missing columns: {missing}")
        sys.exit(1)

    n_values = sorted(df["n_robots"].unique().tolist())

    generate_figure(df, n_values, script_dir)
    generate_csv(df, n_values, script_dir)


if __name__ == "__main__":
    main()
