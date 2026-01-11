#!/usr/bin/env python3
import json
import re
from pathlib import Path
from typing import Dict, List, Tuple, Optional

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
import seaborn as sns
from matplotlib.colors import to_rgb

# ---- IO ----
INPUT_FILE = "../data/attestation-times/_summary/attestation_events_summary_per_attestation.json"
OUTPUT_FILE = "./attestation_event_variance.pdf"

# ---- Style ----
FONT_SCALE = 1.5
LINE_WIDTH = 1.5
SPINES_WIDTH = 1.5
FIG_SIZE = (11, 6.5)
RECT_ALPHA = 0.35
VERT_LINE_LEN = 0.6        # vertical line height in y-units
RECT_HEIGHT = 0.45         # rectangle height in y-units
NUM_OFFSET = 0.15          # vertical distance of event number under the bar

EDGE_COLORS = [
    "#8B4513",  # brown
    "#0000FF",  # blue
    "#FF0000",  # red
    "#008000",  # green
    "#FF7F00",  # orange
    "#00A6A6",  # teal
]

EVENT_LABELS: Dict[int, str] = {
    1: "Evidence sent",
    2: "Attestation started",
    3: "Ref. signature sent",
    4: "Evaluation ready received",
    5: "Result sent",
    6: "Result received",
}

def _lighten(hex_color: str, factor: float = 0.65) -> str:
    """Lighten hex color towards white by 'factor' (0..1)."""
    r, g, b = to_rgb(hex_color)
    r = r + (1 - r) * factor
    g = g + (1 - g) * factor
    b = b + (1 - b) * factor
    return "#{:02X}{:02X}{:02X}".format(int(r * 255), int(g * 255), int(b * 255))

def _order_participants(labels: List[str]) -> List[str]:
    """SECaaS first, then Robot1..N, then alpha."""
    def key(lbl: str) -> Tuple[int, int, str]:
        if lbl == "SECaaS":
            return (0, 0, "")
        m = re.match(r"Robot(\d+)$", lbl)
        if m:
            return (1, int(m.group(1)), "")
        return (2, 0, lbl.lower())
    return sorted(labels, key=key)

def _map_to_event_code(is_robot: bool, is_secaas: bool, role: str, t_key: str) -> Optional[int]:
    """
    Mapping:
      1. evidence sent (RobotX/prover): t_evidence_sent
      2. attestation started (SECaaS/oracle, Robots/verifier): t_attestation_started_received
      3. ref signature sent (SECaaS/oracle): t_ref_signatures_sent
      4. evaluation ready received (Robots/verifier): t_evaluation_ready_received
      5. result sent (SECaaS/verifier, Robots/verifier): t_result_sent
      6. result received (RobotX/prover): t_result_received
    """
    t = t_key.strip().lower()
    r = role.strip().lower()

    if is_robot:
        if r == "prover":
            if t == "t_evidence_sent":      return 1
            if t == "t_result_received":    return 6
        elif r == "verifier":
            if t == "t_attestation_started_received":  return 2
            if t == "t_evaluation_ready_received":     return 4
            if t == "t_result_sent":                   return 5
        return None

    if is_secaas:
        if r == "oracle":
            if t == "t_attestation_started_received":  return 2
            if t == "t_ref_signatures_sent":           return 3
        elif r == "verifier":
            if t == "t_result_sent":                   return 5
        return None

    # Fallback (shouldn't be needed)
    if t == "t_attestation_started_received": return 2
    if t == "t_result_sent":                  return 5
    return None

def _collect_rows(summary: Dict) -> pd.DataFrame:
    """
    Build rows without pooling:
      participant, att (att1-RobotX...), event_code, mean_s, std_s
    """
    rows: List[Dict] = []
    for participant, roles in summary.items():
        is_robot  = participant.lower().startswith("robot")
        is_secaas = (participant == "SECaaS")
        for role_name, per_att in roles.items():              # per_att: dict of attX-RobotY -> t_key -> stats
            for att_label, t_map in per_att.items():
                for t_key, stats in t_map.items():
                    mean_s = stats.get("mean_s", None)
                    std_s  = stats.get("std_s", None)
                    if mean_s is None:
                        continue
                    code = _map_to_event_code(is_robot, is_secaas, role_name, t_key)
                    if code is None:
                        continue
                    rows.append({
                        "participant": participant,
                        "role": role_name,
                        "att": att_label,
                        "event_code": int(code),
                        "mean_s": float(mean_s),
                        "std_s":  float(std_s) if std_s is not None else 0.0,
                    })
    return pd.DataFrame(rows)

def main():
    sns.set_theme(context="paper", style="ticks", font_scale=FONT_SCALE)

    src = Path(INPUT_FILE).resolve()
    if not src.exists():
        raise SystemExit(f"Input JSON not found: {src}")
    with open(src, "r", encoding="utf-8") as f:
        data = json.load(f)

    summary = data.get("summary", {})
    if not summary:
        raise SystemExit("No 'summary' object found in JSON.")

    df = _collect_rows(summary)
    if df.empty:
        raise SystemExit("No event rows collected.")

    # Participant order + y positions
    participants = _order_participants(sorted(df["participant"].unique().tolist()))
    y_map = {p: i for i, p in enumerate(participants)}
    df["y"] = df["participant"].map(y_map)

    # Unique color per participant (no repeats), fill is lightened edge color
    if len(participants) > len(EDGE_COLORS):
        extra = len(participants) - len(EDGE_COLORS)
        hs = np.linspace(0, 1, extra + 1, endpoint=False)[1:]
        extras = []
        for h in hs:
            c = plt.cm.hsv(h)
            extras.append("#{:02X}{:02X}{:02X}".format(int(c[0]*255), int(c[1]*255), int(c[2]*255)))
        palette = EDGE_COLORS + extras
    else:
        palette = EDGE_COLORS[:]
    edge_map = {p: palette[i] for i, p in enumerate(participants)}
    fill_map = {p: _lighten(edge_map[p], factor=0.6) for p in participants}

    fig, ax = plt.subplots(figsize=FIG_SIZE)

    # --- Draw rectangles + vertical bars first (no numbers yet) ---
    for _, row in df.iterrows():
        part = row["participant"]
        y    = float(row["y"])
        x    = float(row["mean_s"])
        std  = float(row["std_s"]) if not np.isnan(row["std_s"]) else 0.0
        half = std if std > 0 else 0.0

        # Variance rectangle (mean ± std)
        if half > 0:
            ax.add_patch(Rectangle(
                (x - half, y - RECT_HEIGHT/2.0),
                2*half, RECT_HEIGHT,
                facecolor=fill_map[part],
                edgecolor='none',
                alpha=RECT_ALPHA,
                zorder=2,
            ))

        # Vertical mean line
        ax.plot(
            [x, x],
            [y - VERT_LINE_LEN/2.0, y + VERT_LINE_LEN/2.0],
            color=edge_map[part],
            linewidth=LINE_WIDTH + 0.4,
            zorder=3,
        )

    # ------------- Pixel-aware de-overlap for numbers (per participant row) -------------
    fig.canvas.draw()  # ensure we have renderer and up-to-date extents
    x0, x1 = ax.get_xlim()
    ax_w_px = ax.get_window_extent().width
    data_per_px = (x1 - x0) / ax_w_px if ax_w_px > 0 else 0.0
    MIN_SEP_PX = 18  # tweak to taste
    delta_x_data = MIN_SEP_PX * data_per_px

    jitter_x: Dict[int, float] = {}
    for part, g in df.groupby("participant"):
        g = g.sort_values("mean_s")
        xs = g["mean_s"].astype(float).tolist()
        xs_j = xs[:]
        for i in range(1, len(xs_j)):
            if xs_j[i] - xs_j[i-1] < delta_x_data:
                xs_j[i] = xs_j[i-1] + delta_x_data
        # clamp to current x-limits
        xs_j = [min(max(x, x0), x1) for x in xs_j]
        jitter_x.update({idx: xj for idx, xj in zip(g.index, xs_j)})

    # --- Draw the event NUMBERS below each bar using jittered x for readability ---
    for idx, row in df.iterrows():
        y     = float(row["y"])
        x_num = jitter_x.get(idx, float(row["mean_s"]))
        code  = int(row["event_code"])
        ax.text(
            x_num, y - (RECT_HEIGHT/2.0) - NUM_OFFSET,
            f"{code}",
            va="top", ha="center",
            fontsize=11, color="black", fontweight="normal",
            zorder=5, clip_on=False,
        )

    # Axes & grids
    ax.set_yticks([y_map[p] for p in participants])
    ax.set_yticklabels(participants)
    ax.set_xlabel("Time (s)")
    # ax.set_ylabel("Participants")
    ax.set_title("Attestation event variance")

    ax.grid(axis="x", linestyle="--", linewidth=1.0, alpha=0.8)
    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_color("black")
        ax.spines[side].set_linewidth(SPINES_WIDTH)

    # Events legend (numbers → names), framed, text-only
    labels = [f"{k}. {v}" for k, v in sorted(EVENT_LABELS.items())]
    handles = [Line2D([], [], linestyle="none", marker=None, color="none") for _ in labels]
    leg = ax.legend(
        handles, labels,
        frameon=True,
        loc="upper left",
        bbox_to_anchor=(1.02, 1.0),
        handlelength=0,
        handletextpad=0.4,
        borderaxespad=0.0,
        labelspacing=0.4,
        columnspacing=0.0,
    )
    leg.get_frame().set_edgecolor("black")
    leg.get_frame().set_linewidth(LINE_WIDTH)

    ax.margins(y=0.18)
    plt.tight_layout()
    plt.show()

    fig.savefig(OUTPUT_FILE, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved plot to: {OUTPUT_FILE}")
    plt.close(fig)

if __name__ == "__main__":
    main()
