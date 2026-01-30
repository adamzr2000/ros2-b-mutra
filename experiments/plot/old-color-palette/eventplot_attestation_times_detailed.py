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
FIG_SIZE = (12.8, 6.5)   # a bit wider to comfortably host legend column
RECT_ALPHA = 0.35
VERT_LINE_LEN = 0.6
RECT_HEIGHT = 0.45
NUM_OFFSET = 0.2
MIN_SEP_PX = 12
GROUP_GAP = 0.4

# ---- Role colors ----
ROLE_EDGE = {
    "prover":   "#0000FF",
    "verifier": "#FF0000",
    "oracle":   "#008000",
}

EVENT_LABELS: Dict[int, str] = {
    1: "Evidence sent",
    2: "Attestation started",
    3: "Ref. signature sent",
    4: "Evaluation ready received",
    5: "Result sent",
    6: "Result received",
}

def _lighten(hex_color: str, factor: float = 0.65) -> str:
    r, g, b = to_rgb(hex_color)
    r = r + (1 - r) * factor
    g = g + (1 - g) * factor
    b = b + (1 - b) * factor
    return "#{:02X}{:02X}{:02X}".format(int(r * 255), int(g * 255), int(b * 255))

def _order_role_labels(labels: List[str]) -> List[str]:
    def key(lbl: str) -> Tuple[int, int, int, str]:
        if lbl == "SECaaS-oracle": return (0, 0, 0, "")
        if lbl == "SECaaS-verifier": return (0, 0, 1, "")
        m = re.match(r"Robot(\d+)-(prover|verifier)$", lbl)
        if m:
            robot_n = int(m.group(1)); role = m.group(2)
            role_rank = 0 if role == "prover" else 1
            return (1, robot_n, role_rank, "")
        return (2, 0, 0, lbl.lower())
    return sorted(labels, key=key)

def _map_to_event_code(is_robot: bool, is_secaas: bool, role: str, t_key: str) -> Optional[int]:
    t = t_key.strip().lower()
    r = role.strip().lower()
    if is_robot:
        if r == "prover":
            if t == "t_evidence_sent": return 1
            if t == "t_result_received": return 6
        elif r == "verifier":
            if t == "t_attestation_started_received": return 2
            if t == "t_evaluation_ready_received": return 4
            if t == "t_result_sent": return 5
        return None
    if is_secaas:
        if r == "oracle":
            if t == "t_attestation_started_received": return 2
            if t == "t_ref_signatures_sent": return 3
        elif r == "verifier":
            if t == "t_result_sent": return 5
        return None
    return None

def _collect_rows(summary: Dict) -> pd.DataFrame:
    rows: List[Dict] = []
    for participant, roles in summary.items():
        is_robot  = participant.lower().startswith("robot")
        is_secaas = (participant == "SECaaS")
        for role_name, per_att in roles.items():
            role_name_l = role_name.strip().lower()
            if is_secaas and role_name_l not in ("oracle", "verifier"): continue
            if is_robot and role_name_l not in ("prover", "verifier"): continue
            role_label = f"{participant}-{role_name_l}"
            for att_label, t_map in per_att.items():
                for t_key, stats in t_map.items():
                    mean_s = stats.get("mean_s", None)
                    std_s  = stats.get("std_s", None)
                    if mean_s is None: continue
                    code = _map_to_event_code(is_robot, is_secaas, role_name, t_key)
                    if code is None: continue
                    rows.append({
                        "participant": participant,
                        "role": role_name_l,
                        "role_label": role_label,
                        "att": att_label,
                        "event_code": int(code),
                        "mean_s": float(mean_s),
                        "std_s": float(std_s) if std_s is not None else 0.0,
                    })
    return pd.DataFrame(rows)

def main():
    sns.set_theme(context="paper", style="ticks", rc={"xtick.direction": "in", "ytick.direction": "in"}, font_scale=FONT_SCALE)


    src = Path(INPUT_FILE).resolve()
    if not src.exists():
        raise SystemExit(f"Input JSON not found: {src}")
    with open(src, "r", encoding="utf-8") as f:
        data = json.load(f)
    summary = data.get("summary", {})
    if not summary: raise SystemExit("No 'summary' object found in JSON.")

    df = _collect_rows(summary)
    if df.empty: raise SystemExit("No event rows collected.")

    # --- Order + Y positions with group gaps ---
    role_labels = _order_role_labels(sorted(df["role_label"].unique().tolist()))
    y_map = {}; y_pos = 0.0; prev_part = None
    for lbl in role_labels:
        part = lbl.split("-")[0]
        if prev_part is not None and part != prev_part:
            y_pos += GROUP_GAP
        y_map[lbl] = y_pos
        y_pos += 1.0
        prev_part = part
    df["y"] = df["role_label"].map(y_map)

    # --- Colors ---
    edge_map = {lbl: ROLE_EDGE[df.loc[df["role_label"] == lbl, "role"].iloc[0]] for lbl in role_labels}
    fill_map = {lbl: _lighten(edge_map[lbl], factor=0.65) for lbl in role_labels}

    # ---- Figure with a dedicated legend column (no cropping ever) ----
    fig = plt.figure(figsize=FIG_SIZE, constrained_layout=False)
    gs = fig.add_gridspec(nrows=1, ncols=2, width_ratios=[1.0, 0.4], wspace=0.05)
    ax = fig.add_subplot(gs[0, 0])
    legax = fig.add_subplot(gs[0, 1])
    legax.set_axis_off()

    # --- Bars + rectangles ---
    for _, row in df.iterrows():
        role_lbl = row["role_label"]; y = float(row["y"])
        x = float(row["mean_s"])
        std = float(row["std_s"]) if not np.isnan(row["std_s"]) else 0.0
        half = std if std > 0 else 0.0
        if half > 0:
            ax.add_patch(Rectangle(
                (x - half, y - RECT_HEIGHT / 2.0),
                2 * half, RECT_HEIGHT,
                facecolor=fill_map[role_lbl], edgecolor='none', alpha=RECT_ALPHA, zorder=2,
            ))
        ax.plot([x, x], [y - VERT_LINE_LEN / 2.0, y + VERT_LINE_LEN / 2.0],
                color=edge_map[role_lbl], linewidth=LINE_WIDTH + 0.4, zorder=3)

    # --- X limits from data + padding ---
    x_left  = float((df["mean_s"] - df["std_s"]).min())
    x_right = float((df["mean_s"] + df["std_s"]).max())
    span = max(x_right - x_left, 1.0)
    pad = 0.08 * span
    ax.set_xlim(x_left - pad, x_right + pad)

    # --- Pixel-aware de-overlap for numbers, clamped inside padded limits ---
    fig.canvas.draw()
    x0, x1 = ax.get_xlim()
    ax_w_px = ax.get_window_extent().width
    data_per_px = (x1 - x0) / ax_w_px if ax_w_px > 0 else 0.0
    delta_x_data = MIN_SEP_PX * data_per_px

    jitter_x: Dict[int, float] = {}
    for _, g in df.groupby("y"):
        g = g.sort_values("mean_s")
        xs = g["mean_s"].astype(float).tolist()
        xs_j = xs[:]
        for i in range(1, len(xs_j)):
            if xs_j[i] - xs_j[i - 1] < delta_x_data:
                xs_j[i] = xs_j[i - 1] + delta_x_data
        inner = max(0.4 * pad, 0.5 * delta_x_data)  # stronger inner margin to avoid right spine collisions
        xs_j = [min(max(x, x0 + inner), x1 - inner) for x in xs_j]
        jitter_x.update({idx: xj for idx, xj in zip(g.index, xs_j)})

    # --- Numbers below bars ---
    for idx, row in df.iterrows():
        y = float(row["y"])
        x_num = jitter_x.get(idx, float(row["mean_s"]))
        code = int(row["event_code"])
        ax.text(
            x_num, y - (RECT_HEIGHT / 2.0) - NUM_OFFSET,
            f"{code}", va="top", ha="center",
            fontsize=9, color="black", fontweight="normal",
            zorder=5, clip_on=False,
        )

    # --- Axes styling ---
    ax.set_yticks([y_map[lbl] for lbl in role_labels])
    ax.set_yticklabels(role_labels)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Participant-role")
    ax.set_title("Attestation event variance")
    ax.grid(axis="x", linestyle="--", linewidth=1.0, alpha=0.8)
    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_color("black")
        ax.spines[side].set_linewidth(SPINES_WIDTH)
    # ax.margins(x=0.02, y=0.1)

    # ---- Build legends inside the dedicated legend axis ----
    # Events (text-only)
    ev_labels = [f"{k}. {v}" for k, v in sorted(EVENT_LABELS.items())]
    ev_handles = [Line2D([], [], linestyle="none", color="none") for _ in ev_labels]
    leg_events = legax.legend(
        ev_handles, ev_labels, frameon=True, loc="upper left",
        bbox_to_anchor=(0.0, 1.0), handlelength=0, handletextpad=0.2,
        borderaxespad=0.0, labelspacing=0.2, columnspacing=0.0, title="Events",
    )
    leg_events.get_frame().set_edgecolor("black")
    leg_events.get_frame().set_linewidth(LINE_WIDTH)

    # Roles (with colored lines)
    role_handles = [
        Line2D([0], [0], color=ROLE_EDGE["prover"],   linewidth=LINE_WIDTH + 0.4, label="prover"),
        Line2D([0], [0], color=ROLE_EDGE["verifier"], linewidth=LINE_WIDTH + 0.4, label="verifier"),
        Line2D([0], [0], color=ROLE_EDGE["oracle"],   linewidth=LINE_WIDTH + 0.4, label="oracle"),
    ]
    leg_roles = legax.legend(
        handles=role_handles, title="Attestation role", frameon=True,
        loc="upper left", bbox_to_anchor=(0.0, 0.40),
        borderaxespad=0.0, handlelength=1.8,
    )
    leg_roles.get_frame().set_edgecolor("black")
    leg_roles.get_frame().set_linewidth(LINE_WIDTH)
    legax.add_artist(leg_events)  # ensure events legend stays on top

    # plt.show()
    # Save (no bbox_extra_artists needed; legends are inside figure area)
    fig.savefig(OUTPUT_FILE, dpi=300, bbox_inches="tight", pad_inches=0.15)
    print(f"[OK] Saved plot to: {OUTPUT_FILE}")
    plt.close(fig)

if __name__ == "__main__":
    main()
