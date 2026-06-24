#!/usr/bin/env python3
"""
Offline metric extraction for the D-MUTRA FORMATION experiment.

Reads a ros2 bag recorded by the bag-recorder service (see
docker-compose.formation.yml) and recomputes the swarm-level damage metrics from
ground-truth odometry — fully decoupled from the live orchestrator, so a metric
formula change never requires re-running an experiment.

Metrics (all from recorded /robotN/odom; the compromised robot is never trusted
for anything beyond its own ground-truth pose):
  * Formation Error (FE)   : mean over ACTIVE robots of ||pose_r - (centroid+slot_r)||
  * Cumulative Damage      : trapezoidal ∫ FE dt over the run window
  * Trajectory Tracking    : cross-track distance of the active-group centroid from
    the ideal straight line (start centroid -> goal); RMSE reported

Event timing comes from the recorded latched markers:
  * /formation/goal     -> t0 (start of the window)
  * /formation/excluded -> t_detect (mitigation fired; that robot leaves the
                           active set and the rest reconfigure to the triangle)
The mitigation flag / compromised robot / scale are read from the orchestrator's
events-<tag>.json when available (falls back to CLI args).

Run inside the ROS image (has rosbag2_py + message types), e.g.:
  docker run --rm -v "$PWD:/w" -w /w turtlebot3-gazebo \\
    bash -lc 'source /opt/ros/humble/setup.bash &&
      python3 tools/formation_metrics.py \\
        --bag experiments/data/formation/bags/<tag> \\
        --events experiments/data/formation/events-<tag>.json \\
        --out-dir experiments/data/formation'

Outputs (under --out-dir):
  formation-<tag>.csv      per-step time series (t_rel, FE, cum_damage, centroid, ...)
  metrics-<tag>.json       headline summary (cumulative_damage, tte_rmse, max_fe, ...)
"""

import argparse
import bisect
import csv
from datetime import datetime, timezone
import json
import math
import os
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


# ── Formation slot tables (mirror formation_agent.cpp / orchestrator) ─────────
def square_slots(d):
    return {"robot1": (d, d), "robot2": (-d, d), "robot3": (-d, -d), "robot4": (d, -d)}


def triangle_slots(d, exclude):
    # Robot3-excluded triangle, matching formation_agent.cpp build_slot_tables().
    # Slots must sum to (0,0) for APF equilibrium to exist:
    #   (d) + (-d) + (0) = 0  ✓ (x)
    #   (d) + (d)  + (-2d) = 0  ✓ (y)
    base = {"robot1": (d, d), "robot2": (-d, d), "robot4": (0.0, -2.0 * d)}
    if exclude == "robot3":
        return base
    # Generic fallback: drop the excluded robot's square slot, keep the rest.
    return {r: s for r, s in square_slots(d).items() if r != exclude}


# ── Bag reading ───────────────────────────────────────────────────────────────
def read_bag(bag_path):
    """Return (gt, odom, goal, t0_ns, detect, clock_pairs) from the bag.

    gt[robot]    = sorted (t_ns, x, y, yaw) from /robotN/ground_truth (P3D truth)
    odom[robot]  = sorted (t_ns, x, y, yaw) from /robotN/odom (encoder belief)
    goal         = (x, y) or None
    t0_ns        = recv time of /formation/goal (None if absent)
    detect       = (t_ns, robot) of /formation/excluded (None if absent)
    clock_pairs  = sorted [(wall_ns, sim_ns)] from /clock — used to convert a
                   host wall-clock inject timestamp into sim time offline.
    """
    storage_id = "sqlite3"
    meta = os.path.join(bag_path, "metadata.yaml")
    if os.path.exists(meta):
        with open(meta) as f:
            if "mcap" in f.read():
                storage_id = "mcap"

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""))
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_cache = {}

    gt = {}
    odom = {}
    goal = None
    t0_ns = None
    detect = None
    clock_pairs = []   # (wall_ns, sim_ns)

    while reader.has_next():
        topic, data, t = reader.read_next()
        typ = type_map.get(topic)
        if typ is None:
            continue
        cls = msg_cache.get(typ) or msg_cache.setdefault(typ, get_message(typ))
        msg = deserialize_message(data, cls)

        if topic.endswith("/ground_truth") or topic.endswith("/odom"):
            robot = topic.strip("/").split("/")[0]
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            sink = gt if topic.endswith("/ground_truth") else odom
            sink.setdefault(robot, []).append((t, p.x, p.y, yaw))
        elif topic == "/formation/goal":
            goal = (msg.x, msg.y)
            if t0_ns is None:
                t0_ns = t
        elif topic == "/formation/excluded":
            if msg.data and detect is None:
                detect = (t, msg.data)
        elif topic == "/clock":
            sim_ns = msg.clock.sec * 10**9 + msg.clock.nanosec
            clock_pairs.append((t, sim_ns))

    for d in (gt, odom):
        for r in d:
            d[r].sort(key=lambda e: e[0])
    clock_pairs.sort()
    return gt, odom, goal, t0_ns, detect, clock_pairs


def interp_xy(series, t_ns):
    """Linear-interpolate (x, y, yaw) at t_ns from a sorted (t,x,y,yaw) series."""
    if not series:
        return None
    ts = [e[0] for e in series]
    if t_ns <= ts[0]:
        return series[0][1], series[0][2], series[0][3]
    if t_ns >= ts[-1]:
        return series[-1][1], series[-1][2], series[-1][3]
    i = bisect.bisect_left(ts, t_ns)
    a, b = series[i - 1], series[i]
    span = b[0] - a[0]
    f = (t_ns - a[0]) / span if span else 0.0
    return (a[1] + f * (b[1] - a[1]),
            a[2] + f * (b[2] - a[2]),
            a[3] + f * (b[3] - a[3]))


# ── Metric computation ────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="path to the ros2 bag directory")
    ap.add_argument("--events", default="", help="orchestrator events-<tag>.json")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--tag", default="", help="override run tag (else from bag dir name)")
    ap.add_argument("--scale", type=float, default=2.0, help="formation half-diagonal d (m)")
    ap.add_argument("--compromised", default="robot3")
    ap.add_argument("--mitigated", choices=["auto", "true", "false"], default="auto")
    ap.add_argument("--window", type=float, default=0.0,
                    help="analysis window (s); 0 = until last pose sample")
    ap.add_argument("--dt", type=float, default=0.1, help="resample step (s)")
    args = ap.parse_args()

    tag = args.tag or os.path.basename(os.path.normpath(args.bag))

    # Merge orchestrator events metadata when present.
    d = args.scale
    compromised = args.compromised
    mitigation = None
    if args.events and os.path.exists(args.events):
        with open(args.events) as f:
            ev = json.load(f)
        mitigation = ev.get("mitigation")
        comp = ev.get("compromised") or []
        if comp:
            compromised = comp[0]

    # Load inject record if present (written by campaign driver after tamper.sh).
    inject_path = os.path.join(args.out_dir, f"inject-{tag}.json")
    inject_data = {}
    if os.path.exists(inject_path):
        try:
            with open(inject_path) as f:
                inject_data = json.load(f)
        except (OSError, ValueError):
            pass

    gt, odom, goal, t0_ns, detect, clock_pairs = read_bag(args.bag)
    # Evaluate on TRUE pose (P3D ground_truth); fall back to encoder odom only if
    # ground truth was not recorded, with a clear warning (encoder odom drifts
    # while the compromised robot spins, so it under/over-states the real damage).
    if gt:
        poses = gt
        pose_source = "ground_truth"
    elif odom:
        poses = odom
        pose_source = "odom"
        print("⚠️ no /ground_truth in bag — falling back to encoder /odom "
              "(metrics may be biased under the spinning attack)", file=sys.stderr)
    else:
        print(f"❌ no pose topics in bag {args.bag}", file=sys.stderr)
        sys.exit(1)
    robots = sorted(poses.keys())

    if t0_ns is None:
        # No goal marker — fall back to the earliest odom sample as t0.
        t0_ns = min(s[0][0] for s in poses.values() if s)
        print("⚠️ no /formation/goal in bag; using first odom sample as t0", file=sys.stderr)

    # Mitigated iff a /formation/excluded marker was recorded (unless overridden).
    if args.mitigated == "true":
        is_mit = True
    elif args.mitigated == "false":
        is_mit = False
    elif mitigation is not None:
        is_mit = bool(mitigation)
    else:
        is_mit = detect is not None
    t_detect_rel = (detect[0] - t0_ns) / 1e9 if detect else None
    # For unmitigated runs the orchestrator never publishes /formation/excluded
    # (mitigation=off), so detect is None from the bag. Fall back to the
    # blockchain FAILURE time from the events JSON so t_detect_s is still
    # populated (useful for cross-referencing with the micro-benchmark).
    if t_detect_rel is None and ev.get("failure_events"):
        t_detect_rel = ev["failure_events"][0][0]

    # Run-relative time of injection. The bag stamps every message with its host
    # wall-clock (UTC epoch ns) RECEIVE time; t0_ns (the /formation/goal recv time)
    # is on that same clock, and so are the run timeline t_rel_s used below and
    # t_detect_rel above. The inject record's t_inject_wall is host UTC too (written
    # by `date -u` right after tamper.sh). So the injection's position on the run
    # timeline is simply the wall-clock delta from goal publication — no /clock sim
    # correlation is needed. (Mixing in the sim clock here was the old bug: it
    # subtracted a ~1.78e18 ns wall t0 from a ~1e10 ns sim value.)
    t_inject_wall = inject_data.get("t_inject_wall")  # e.g. "2026-06-20T14:55:39.123Z"
    t_inject_s = None  # run-relative seconds (same timeline as t_rel_s / t_detect_s)
    if t_inject_wall and t0_ns is not None:
        try:
            dt = datetime.fromisoformat(t_inject_wall.replace("Z", "+00:00"))
            t_inj_wall_ns = int(dt.timestamp() * 1e9)
            t_inject_s = round((t_inj_wall_ns - t0_ns) / 1e9, 3)
        except ValueError:
            pass

    last_rel = max((s[-1][0] for s in poses.values() if s), default=t0_ns)
    last_rel = (last_rel - t0_ns) / 1e9
    window = args.window if args.window > 0 else last_rel
    sq = square_slots(d)
    tri = triangle_slots(d, compromised)

    # Reference line: centroid of all robots at t0.
    c0 = _centroid({r: interp_xy(poses[r], t0_ns) for r in robots}, robots)
    gx, gy = goal if goal else (c0[0] + 1.0, c0[1])
    lx, ly = gx - c0[0], gy - c0[1]
    L = math.hypot(lx, ly) or 1.0
    ux, uy = lx / L, ly / L

    rows = []
    cum = 0.0
    tte_sq = 0.0
    tte_n = 0
    max_fe = 0.0
    prev_t = 0.0
    n = int(window / args.dt) + 1
    for k in range(n):
        t_rel = k * args.dt
        t_ns = t0_ns + int(t_rel * 1e9)
        # Active set + slot table: the compromised robot leaves the formation only
        # in the mitigated case, at t_detect.
        if is_mit and t_detect_rel is not None and t_rel >= t_detect_rel:
            active = [r for r in robots if r != compromised]
            slots = tri
        else:
            active = list(robots)
            slots = sq
        pos = {r: interp_xy(poses[r], t_ns) for r in robots}
        c = _centroid(pos, active)
        # Formation error over active robots.
        errs = []
        for r in active:
            s = slots.get(r)
            p = pos.get(r)
            if s is None or p is None:
                continue
            errs.append(math.hypot(p[0] - (c[0] + s[0]), p[1] - (c[1] + s[1])))
        fe = sum(errs) / len(errs) if errs else 0.0
        max_fe = max(max_fe, fe)
        # Along-track progress + cross-track of the active centroid relative to the
        # ideal straight line start_centroid -> goal. Along-track is the headline
        # damage signal: a frozen/spinning compromised robot anchors the cohesive
        # swarm, so the centroid stops advancing toward the goal.
        rx, ry = c[0] - c0[0], c[1] - c0[1]
        along = rx * ux + ry * uy
        cross = abs(rx * (-uy) + ry * ux)
        tte_sq += cross * cross
        tte_n += 1
        # Trapezoidal integral of FE.
        if k > 0:
            cum += 0.5 * (fe + rows[-1][1]) * (t_rel - prev_t)
        prev_t = t_rel
        rows.append((t_rel, fe, cum, c[0], c[1], cross, along,
                     [pos[r] for r in robots]))

    os.makedirs(args.out_dir, exist_ok=True)
    csv_path = os.path.join(args.out_dir, f"formation-{tag}.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_rel_s", "formation_error", "cum_damage",
                    "centroid_x", "centroid_y", "cross_track", "along_track"]
                   + [f"x_{r}" for r in robots] + [f"y_{r}" for r in robots]
                   + [f"yaw_{r}" for r in robots])
        for t_rel, fe, c_, cx, cy, cross, along, pp in rows:
            xs   = [f"{p[0]:.4f}" if p else "" for p in pp]
            ys   = [f"{p[1]:.4f}" if p else "" for p in pp]
            yaws = [f"{p[2]:.4f}" if p else "" for p in pp]
            w.writerow([f"{t_rel:.2f}", f"{fe:.4f}", f"{c_:.4f}",
                        f"{cx:.4f}", f"{cy:.4f}", f"{cross:.4f}", f"{along:.4f}"]
                       + xs + ys + yaws)

    tte_rmse = math.sqrt(tte_sq / tte_n) if tte_n else 0.0
    final_along = rows[-1][6] if rows else 0.0
    goal_dist = math.hypot(gx - c0[0], gy - c0[1])
    progress_pct = 100.0 * final_along / goal_dist if goal_dist else 0.0
    t_attack_to_detect = (
        round(t_detect_rel - t_inject_s, 3)
        if (t_detect_rel is not None and t_inject_s is not None)
        else None
    )

    # ── ife_compromise: ∫FE over the COMPROMISE WINDOW (injection → detection) ────
    # Isolates the geometric distortion that accrues while the attack is active and
    # uncorrected (injection → detection/exclusion for mitigated; injection → mission
    # end for unmitigated). ife_window_s is the integration width (≈ SSP+B for
    # mitigated). NOTE: windows differ between mitigated and unmitigated, making
    # direct comparison harder to justify in the paper. Use ife_remaining instead
    # when the cohesion-drift attack is used (mission always completes → same window).
    ife_compromise = None
    ife_window_s = None
    if t_inject_s is not None:
        if is_mit and t_detect_rel is not None:
            ife_end = t_detect_rel
        elif not is_mit:
            ife_end = window  # never mitigated: distortion persists to mission end
        else:
            ife_end = None
        if ife_end is not None and ife_end > t_inject_s:
            ife_compromise = round(_integ_fe(rows, t_inject_s, ife_end), 4)
            ife_window_s = round(ife_end - t_inject_s, 2)

    # ── ife_remaining: ∫FE over the REMAINING MISSION (injection → mission_end) ──
    # SAME integration window for every attacked scenario: from t_inject to the
    # fixed mission-end (window). Requires the cohesion-drift attack because the
    # mission must complete (otherwise unmitigated has no "mission end" to integrate
    # to). With equal windows: lower ife_remaining ↔ faster D-MUTRA detection and
    # shorter recovery → stronger, paper-defensible comparison across SSP values.
    # baseline has no injection → ife_remaining = None.
    ife_remaining = None
    if t_inject_s is not None and t_inject_s < window:
        ife_remaining = round(_integ_fe(rows, t_inject_s, window), 4)

    summary = {
        "run_tag": tag,
        "mode": "formation",
        "pose_source": pose_source,
        "mitigation": is_mit,
        "compromised": compromised,
        "formation_scale": d,
        "goal": [gx, gy],
        "window_s": round(window, 2),
        # Injection timing — t_inject_wall is ms-precision host UTC (from inject JSON);
        # t_inject_s is derived by correlating that wall time with the /clock stream
        # in the bag. t_attack_to_detect_s should ≈ SSP + B (cross-check against
        # the micro-benchmark in run_experiments_and_collect_tamper_detection.py).
        "t_inject_wall": t_inject_wall,
        "t_inject_s": t_inject_s,
        "t_detect_s": round(t_detect_rel, 2) if t_detect_rel is not None else None,
        "t_attack_to_detect_s": t_attack_to_detect,
        # Headline: along-track mission progress (the compromised robot anchors the
        # cohesive swarm, so the centroid stops advancing toward the goal).
        "final_progress_m": round(final_along, 4),
        "goal_distance_m": round(goal_dist, 4),
        "progress_pct": round(progress_pct, 2),
        # ife_compromise: ∫FE from injection to detection (mitigated) or mission
        # end (unmitigated). Windows differ between scenarios — harder to justify.
        "ife_compromise": ife_compromise,
        "ife_window_s": ife_window_s,
        # ife_remaining: ∫FE from injection to mission_end — SAME window for all
        # attacked scenarios (requires cohesion-drift attack, mission always ends).
        "ife_remaining": ife_remaining,
        # Reference only: full-run ∫FE. Does NOT scale with SSP (window-dominated).
        "cumulative_damage": round(cum, 4),
        "tte_rmse_m": round(tte_rmse, 4),
        "max_formation_error_m": round(max_fe, 4),
        "robots": robots,
        "csv": csv_path,
    }
    out_json = os.path.join(args.out_dir, f"metrics-{tag}.json")
    with open(out_json, "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))
    print(f"Saved → {csv_path}")
    print(f"Saved → {out_json}")


def _centroid(pos, robots):
    pts = [pos[r] for r in robots if pos.get(r) is not None]
    if not pts:
        return (0.0, 0.0)
    return (sum(p[0] for p in pts) / len(pts),
            sum(p[1] for p in pts) / len(pts))


def _integ_fe(rows, t0, t1):
    """Trapezoidal ∫FE dt over [t0, t1] from per-step rows (t_rel, fe, ...)."""
    s = 0.0
    prev = None
    for r in rows:
        t, fe = r[0], r[1]
        if t0 <= t <= t1:
            if prev is not None:
                s += 0.5 * (fe + prev[1]) * (t - prev[0])
            prev = (t, fe)
        else:
            prev = None
    return s


if __name__ == "__main__":
    main()
