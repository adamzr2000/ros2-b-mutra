#!/usr/bin/env python3
"""
Attestation cycle Gantt chart — single representative cycle.

One row per participant (Robot1–4, SECaaS).  For each Robot (Prover):
  Dark-red bar : local computation   (t_prover_start  → t_evidence_sent)
  Blue bar     : blockchain wait      (t_evidence_sent → t_prover_finished)
For SECaaS (Oracle):
  Green bar    : oracle processing    (t_oracle_start  → t_oracle_finished)
                 drawn once per robot request — reveals sequential dispatch.

Colors match the attestation-cycle breakdown bar chart for cross-figure consistency.
Data: N=4, continuous mode, run 1, first attestation cycle.
"""

from pathlib import Path
import json
import re
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import seaborn as sns

DATA_FILE = (
    Path(__file__).parent.parent
    / "data/attestation-times/_summary/attestation_events_by_run.json"
)

N_KEY     = "N4_continuous"
RUN       = "run1"
N_CYCLES  = 1       # attestation cycles per robot to display

FONT_SCALE = 1.4
BAR_H      = 0.52   # row bar height (fraction of row spacing)

C_LOCAL  = "#993333"  # dark red   — local computation
C_CHAIN  = "#336699"  # steel blue — blockchain wait
C_ORACLE = "#4A7C59"  # muted green — SECaaS oracle


# ── helpers ────────────────────────────────────────────────────────────────────

def _att_order(label: str) -> int:
    m = re.match(r"att(\d+)", label)
    return int(m.group(1)) if m else 9999


def _hbar(ax, y: float, t_start: float, t_end: float, color: str, h: float = BAR_H):
    if t_end <= t_start:
        return
    ax.broken_barh(
        [(t_start, t_end - t_start)],
        (y - h / 2, h),
        facecolors=color, edgecolors="none", zorder=3,
    )


# ── main ───────────────────────────────────────────────────────────────────────

def main():
    data = json.loads(DATA_FILE.read_text())["data"][N_KEY][RUN]

    robot_re = re.compile(r"^Robot(\d+)$")
    robots   = sorted(
        [k for k in data if robot_re.match(k)],
        key=lambda x: int(robot_re.match(x).group(1)),
    )

    # ── Robot prover phases ────────────────────────────────────────────────────
    robot_cycles = {}
    x_max = 0.0
    for robot in robots:
        prover = data[robot].get("prover", {})
        labels = sorted(prover, key=_att_order)[:N_CYCLES]
        cycles = []
        for lbl in labels:
            e = prover[lbl]
            cycles.append({
                "local_start": e["t_prover_start"],
                "local_end":   e["t_evidence_sent"],
                "chain_start": e["t_evidence_sent"],
                # absorb negligible result-processing (~5 ms) into blockchain bar
                "chain_end":   e["t_prover_finished"],
            })
            x_max = max(x_max, e["t_prover_finished"])
        robot_cycles[robot] = cycles

    # ── SECaaS oracle calls ────────────────────────────────────────────────────
    oracle_bars = []
    if "SECaaS" in data:
        for lbl, e in data["SECaaS"].get("oracle", {}).items():
            m = re.match(r"att(\d+)-Robot\d+$", lbl)
            if m and int(m.group(1)) <= N_CYCLES:
                t0 = e.get("t_oracle_start")
                t1 = e.get("t_oracle_finished")
                if t0 is not None and t1 is not None:
                    oracle_bars.append((t0, t1))
    oracle_bars.sort()

    # ── y-axis positions: Robot1 at bottom, SECaaS at top ─────────────────────
    y_robot  = {r: i for i, r in enumerate(robots)}
    y_secaas = len(robots)
    ylabels  = robots + ["SECaaS"]

    # ── figure ─────────────────────────────────────────────────────────────────
    sns.set_theme(
        context="paper", style="ticks",
        rc={"xtick.direction": "out", "ytick.direction": "out"},
        font_scale=FONT_SCALE,
    )
    plt.rcParams.update({"font.family": "serif"})

    fig, ax = plt.subplots(figsize=(8.0, 3.2))

    # Robot rows
    for robot, cycles in robot_cycles.items():
        y = y_robot[robot]
        for c in cycles:
            _hbar(ax, y, c["local_start"], c["local_end"],  C_LOCAL)
            _hbar(ax, y, c["chain_start"], c["chain_end"],  C_CHAIN)

    # SECaaS row — one green bar per oracle call (sequential dispatch visible)
    for t0, t1 in oracle_bars:
        _hbar(ax, y_secaas, t0, t1, C_ORACLE)

    # ── axes ───────────────────────────────────────────────────────────────────
    ax.set_xlim(0, x_max + 0.25)
    ax.set_ylim(-0.75, y_secaas + 0.75)

    ax.set_yticks(range(len(ylabels)))
    ax.set_yticklabels(ylabels)
    ax.set_xlabel("Elapsed time (s)")

    ax.tick_params(axis="both", which="major", length=5, width=1.0, direction="out")
    ax.tick_params(axis="y", which="major", length=0)   # no tick marks on y
    ax.grid(axis="x", linestyle="-", linewidth=0.6, alpha=0.65, zorder=1)
    ax.set_axisbelow(True)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    # ── legend ─────────────────────────────────────────────────────────────────
    handles = [
        mpatches.Patch(facecolor=C_LOCAL,  edgecolor="black", linewidth=0.6,
                       label="Local computation (evidence preparation)"),
        mpatches.Patch(facecolor=C_CHAIN,  edgecolor="black", linewidth=0.6,
                       label="Blockchain wait (transaction confirmation)"),
        mpatches.Patch(facecolor=C_ORACLE, edgecolor="black", linewidth=0.6,
                       label="SECaaS oracle processing"),
    ]
    fig.legend(
        handles=handles,
        loc="lower center", bbox_to_anchor=(0.5, 0.0),
        ncol=3, frameon=True, framealpha=0.90, fancybox=True,
        fontsize=plt.rcParams.get("legend.fontsize", 9),
    )
    plt.subplots_adjust(bottom=0.30)

    out_path = Path(__file__).parent / "gantt_attestation_cycle.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved: {out_path}")

    # Print values useful for the figure caption
    print("\n=== Cycle timing (run1, att1) ===")
    for robot, cycles in robot_cycles.items():
        c = cycles[0]
        local = c["local_end"]  - c["local_start"]
        bc    = c["chain_end"]  - c["chain_start"]
        print(f"  {robot}: local={local:.3f} s  blockchain={bc:.3f} s  total={local+bc:.3f} s")
    print("  SECaaS oracle calls:")
    for i, (t0, t1) in enumerate(oracle_bars, 1):
        print(f"    call {i}: {t0:.3f}–{t1:.3f} s  (dur={t1-t0:.3f} s)")

    plt.close(fig)


if __name__ == "__main__":
    main()
