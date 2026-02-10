#!/usr/bin/env python3
import json
import re
from pathlib import Path
from typing import Dict, List, Tuple, Optional

import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
import seaborn as sns
from matplotlib.colors import to_rgb

# ---- Config ----
# Set to integer (e.g., 1) for a specific run, or None for Average
TARGET_RUN = 1 
ZOOM_X_LIMIT = 65  # Set to None to auto-scale

# ---- IO ----
SUMMARY_DIR = "../data/attestation-times/_summary"
FILE_BY_RUN = "attestation_events_by_run.json"
FILE_SUMMARY = "attestation_events_summary_per_attestation.json"
OUTPUT_FILE = f"./attestation_event_flow_run{TARGET_RUN}.pdf" if TARGET_RUN else "./attestation_event_variance_avg.pdf"

# ---- Style ----
FONT_SCALE = 1.5
LINE_WIDTH = 1.2
SPINES_WIDTH = 1.0
FIG_SIZE = (12, 6)
RECT_ALPHA = 0.35
VERT_LINE_LEN = 0.6
RECT_HEIGHT = 0.45
NUM_OFFSET = 0.25
MIN_SEP_PX = 12
GROUP_GAP = 0.4
EVENT_LABELS_FONT_SIZE = 8

# UPDATED: 7 Events as requested
EVENT_LABELS: Dict[int, str] = {
    1: "Start local hashing",
    2: "Fresh signatures sent",
    3: "Attestation started",
    4: "Ref. signatures sent",
    5: "Evaluation ready",
    6: "Result sent",
    7: "Result received",
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

def _format_label(lbl: str) -> str:
    """Make 'robot1-prover' look like 'Robot1 (Prover)'"""
    parts = lbl.split("-")
    name = parts[0]
    role = parts[1].title() if len(parts) > 1 else ""
    
    if name == "SECaaS":
        return f"SECaaS ({role})"
    
    return f"{name} ({role})"

def _map_to_event_code(is_robot: bool, is_secaas: bool, role: str, t_key: str) -> Optional[int]:
    """
    Maps raw timestamp keys to Event IDs (1-7).
    """
    t = t_key.strip() # keep case sensitivity if needed, or .lower() 
    # normalize for comparison
    t_lower = t.lower()
    r = role.strip().lower()

    if is_robot:
        if r == "prover":
            # 1. Start local hashing
            if t_lower == "t_prover_start": return 1
            # 2. Fresh signatures sent
            if t_lower == "t_evidence_sent": return 2
            # 7. Result received
            if t_lower == "t_result_received": return 7
        
        elif r == "verifier":
            # 5. Evaluation ready (Verifier start)
            if t_lower == "t_verifier_start": return 5
            # 6. Result sent
            if t_lower == "t_result_sent": return 6

    if is_secaas:
        if r == "oracle":
            # 3. Attestation started
            if t_lower == "t_oracle_start": return 3
            # 4. Ref. signatures sent
            # Check both requested key and standard key just in case
            if t_lower == "t_prover_ref_signatures_sent": return 4
            if t_lower == "t_ref_signatures_sent": return 4

        elif r == "verifier":
            # 5. Evaluation ready (Verifier start)
            if t_lower == "t_verifier_start": return 5
            # 6. Result sent
            if t_lower == "t_result_sent": return 6

    return None

def _collect_rows(data_root: Dict, mode_single_run: bool) -> pd.DataFrame:
    rows: List[Dict] = []
    for participant, roles in data_root.items():
        is_robot  = participant.lower().startswith("robot")
        is_secaas = (participant == "SECaaS")
        
        for role_name, per_att in roles.items():
            role_name_l = role_name.strip().lower()
            if is_secaas and role_name_l not in ("oracle", "verifier"): continue
            if is_robot and role_name_l not in ("prover", "verifier"): continue
            
            role_label = f"{participant}-{role_name_l}"
            
            for att_label, t_map in per_att.items():
                for t_key, val_obj in t_map.items():
                    code = _map_to_event_code(is_robot, is_secaas, role_name, t_key)
                    if code is None: continue

                    if mode_single_run:
                        mean_s = float(val_obj) if val_obj is not None else None
                        std_s = 0.0
                    else:
                        mean_s = val_obj.get("mean_s")
                        std_s = val_obj.get("std_s", 0.0)

                    if mean_s is None: continue

                    rows.append({
                        "participant": participant,
                        "role_label": role_label,
                        "event_code": int(code),
                        "mean_s": float(mean_s),
                        "std_s": float(std_s) if std_s is not None else 0.0,
                    })
    return pd.DataFrame(rows)

def main():
    sns.set_theme(context="paper", style="ticks", rc={"xtick.direction": "in", "ytick.direction": "in"}, font_scale=FONT_SCALE)

    # ---- LOAD DATA ----
    dir_path = Path(SUMMARY_DIR).resolve()
    if TARGET_RUN is not None:
        src = dir_path / FILE_BY_RUN
        print(f"[INFO] Mode: Single Run (Run {TARGET_RUN})")
        # Ensure file exists
        if not src.exists():
            print(f"[ERR] File not found: {src}")
            return
        with open(src, "r") as f:
            full_json = json.load(f)
        data_root = full_json["by_run"].get(f"run{TARGET_RUN}", {})
        mode_single = True
    else:
        src = dir_path / FILE_SUMMARY
        print(f"[INFO] Mode: Average Summary")
        if not src.exists():
            print(f"[ERR] File not found: {src}")
            return
        with open(src, "r") as f:
            full_json = json.load(f)
        data_root = full_json.get("summary", {})
        mode_single = False

    df = _collect_rows(data_root, mode_single)
    if df.empty: 
        print("No event rows collected. Check if JSON keys match _map_to_event_code.")
        return

    # --- Order + Y positions ---
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
    participants = sorted(df["participant"].unique(), key=lambda x: (0 if "secaas" in x.lower() else 1, x))
    palette = sns.color_palette("colorblind", n_colors=len(participants))
    part_color_map = dict(zip(participants, palette))

    edge_map = {}
    fill_map = {}
    for lbl in role_labels:
        part_name = lbl.split("-")[0]
        base_color = part_color_map.get(part_name, "#333333")
        edge_map[lbl] = base_color
        fill_map[lbl] = _lighten(base_color, factor=0.65)

    # ---- Figure ----
    # Adjusted width ratios
    fig = plt.figure(figsize=FIG_SIZE, constrained_layout=False)
    gs = fig.add_gridspec(nrows=1, ncols=2, width_ratios=[1.0, 0.25], wspace=0.02)
    ax = fig.add_subplot(gs[0, 0])
    legax = fig.add_subplot(gs[0, 1])
    legax.set_axis_off()

    # --- Plotting ---
    for _, row in df.iterrows():
        if ZOOM_X_LIMIT is not None and float(row["mean_s"]) > ZOOM_X_LIMIT: continue

        role_lbl = row["role_label"]; y = float(row["y"])
        x = float(row["mean_s"]); std = float(row["std_s"])
        
        # Draw Box (std dev)
        if std > 0:
            ax.add_patch(Rectangle(
                (x - std, y - RECT_HEIGHT / 2.0),
                2 * std, RECT_HEIGHT,
                facecolor=fill_map[role_lbl], edgecolor='none', alpha=RECT_ALPHA, zorder=2,
            ))
        # Draw Line (mean)
        ax.plot([x, x], [y - VERT_LINE_LEN / 2.0, y + VERT_LINE_LEN / 2.0],
                color=edge_map[role_lbl], linewidth=LINE_WIDTH + 0.4, zorder=3)

    # --- X Limits ---
    if ZOOM_X_LIMIT is not None:
        ax.set_xlim(-3.0, ZOOM_X_LIMIT)
    else:
        # Auto-scale with padding
        x_min = (df["mean_s"] - df["std_s"]).min()
        x_max = (df["mean_s"] + df["std_s"]).max()
        pad = (x_max - x_min) * 0.05
        ax.set_xlim(x_min - pad, x_max + pad)

    # --- Jitter Logic for Numbers ---
    fig.canvas.draw()
    x0, x1 = ax.get_xlim()
    bbox = ax.get_window_extent()
    ax_w_px = bbox.width if bbox else 1
    data_per_px = (x1 - x0) / ax_w_px if ax_w_px > 0 else 0.001
    delta_x_data = MIN_SEP_PX * data_per_px

    jitter_x = {}
    for _, g in df.groupby("y"):
        if ZOOM_X_LIMIT: g = g[g["mean_s"] <= ZOOM_X_LIMIT]
        g = g.sort_values("mean_s")
        xs = g["mean_s"].tolist()
        xs_j = xs[:]
        
        # Simple collision avoidance
        for i in range(1, len(xs_j)):
            if xs_j[i] - xs_j[i - 1] < delta_x_data:
                xs_j[i] = xs_j[i - 1] + delta_x_data
        
        # Keep inside bounds
        xs_j = [min(x, x1 - delta_x_data) for x in xs_j] 
        jitter_x.update({idx: xj for idx, xj in zip(g.index, xs_j)})

    # Draw Numbers
    for idx, row in df.iterrows():
        if ZOOM_X_LIMIT is not None and float(row["mean_s"]) > ZOOM_X_LIMIT: continue
        
        x_num = jitter_x.get(idx, float(row["mean_s"]))
        ax.text(
            x_num, float(row["y"]) - (RECT_HEIGHT / 2.0) - NUM_OFFSET,
            f"{int(row['event_code'])}",
            va="top", ha="center", fontsize=EVENT_LABELS_FONT_SIZE, color="black", zorder=5
        )

    # --- Axes styling ---
    # Use formatted labels
    formatted_labels = [_format_label(lbl) for lbl in role_labels]
    ax.set_yticks([y_map[lbl] for lbl in role_labels])
    ax.set_yticklabels(formatted_labels)

    y_bottom, y_top = ax.get_ylim()
    ax.set_ylim(y_bottom - 0.2, y_top + 0.2)
    
    ax.set_xlabel("Time (s)")
    ax.grid(axis="x", linestyle="-", linewidth=1.0, alpha=0.8)
    
    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_color("black")
        ax.spines[side].set_linewidth(SPINES_WIDTH)

    # ---- Legend (Events Only) ----
    ev_labels = [f"{k}: {v}" for k, v in sorted(EVENT_LABELS.items())]
    # Invisible handles just to list text
    ev_handles = [Line2D([], [], linestyle="none", color="none") for _ in ev_labels]
    
    leg = legax.legend(
        ev_handles, ev_labels, 
        loc="upper left", bbox_to_anchor=(0.0, 1.0),
        title="Event types", frameon=True, handlelength=0, handletextpad=0
    )
    # leg.get_title().set_fontweight("bold")
    leg.get_frame().set_edgecolor("black")
    leg.get_frame().set_linewidth(SPINES_WIDTH)

    fig.savefig(OUTPUT_FILE, dpi=300, bbox_inches="tight", pad_inches=0.1)
    print(f"[OK] Saved plot to: {OUTPUT_FILE}")
    plt.close(fig)

if __name__ == "__main__":
    main()