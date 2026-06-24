# D-MUTRA Formation Tampering Experiments

This document describes the methodology behind the data collected for the formation
experiment plots in the D-MUTRA (Decentralized Multi-robot Trust and Runtime
Attestation) evaluation.  All metrics are computed *offline* from ROS 2 bags so
that formula changes never require re-running an experiment.

---

## 1  Formation Agent

Each robot runs an instance of `formation_agent` — a fully decentralized ROS 2 node
(`dockerfiles/turtlebot3/src/formation_agent/src/formation_agent.cpp`).  There is
no central planner: every robot independently computes its own velocity command at
10 Hz from an Artificial Potential Field (APF) using *neighbor odometry*.

### Subscriptions (per agent)

| Topic | Type | Purpose |
|-------|------|---------|
| `/robotN/odom` (all N) | `nav_msgs/Odometry` | Ground-truth poses of own robot and all neighbors |
| `/formation/goal` | `geometry_msgs/Point` | Shared global goal, latched; published once by orchestrator |
| `/formation/excluded` | `std_msgs/String` | Mitigation signal: id of the robot to drop, latched |

### Publications

| Topic | Type | Purpose |
|-------|------|---------|
| `cmd_vel` (relative) | `geometry_msgs/Twist` | Differential-drive velocity command |
| `formation_status` | `std_msgs/String` | JSON heartbeat with pose + instantaneous slot error |

### Control law (tick, 10 Hz)

The resultant force is the sum of a goal-attraction term and a formation-cohesion
term over all active neighbors:

```
F_total = F_goal + F_formation

F_goal      = k_att · (goal − own_pos) / ‖goal − own_pos‖     [k_att = 0.8]

F_formation = Σ_j  k_form · ((pos_j + slot_i − slot_j) − own_pos)
                                                               [k_form = 3.0]
```

The agent then uses a rotate-to-heading + drive-straight P-controller:

1. Compute heading error: `yaw_err = atan2(F.y, F.x) − own_yaw`
2. Angular command: `ang = kp_ang · yaw_err`  (kp_ang = 2.5 rad/s per rad)
3. Linear command: if `|yaw_err| ≤ 0.20 rad`, `lin = kp_lin · ‖F‖`  (kp_lin = 0.5)
   otherwise `lin = 0` (rotate in place first)

### Formation slot tables (d = 2.0 m)

**Square (4 robots, normal operation):**

| Robot | x-offset | y-offset |
|-------|----------|----------|
| robot1 | +2.0 | +2.0 |
| robot2 | −2.0 | +2.0 |
| robot3 | −2.0 | −2.0 |
| robot4 | +2.0 | −2.0 |

**Triangle (robot3 excluded, post-mitigation):**

| Robot | x-offset | y-offset |
|-------|----------|----------|
| robot1 | +2.0 | +2.0 |
| robot2 | −2.0 | +2.0 |
| robot4 |  0.0 | −4.0 |

Slots sum to (0, 0) in both tables, which guarantees that an APF equilibrium exists
(the formation does not drift under the cohesion term alone).

### Mitigation reconfiguration

When the orchestrator publishes `/formation/excluded` naming robot3:

- **robot3 itself** enters `State::EXCLUDED` and halts (`cmd_vel = 0`).
- **Every healthy robot** drops robot3 from its neighbor set and switches its active
  slot table from square to triangle, re-converging the remaining group immediately.

---

## 2  Scenarios

Three scenario types are evaluated, differing only in whether an attack is injected
and whether the D-MUTRA mitigation is armed.

### 2.1  Baseline (no attack)

All 4 robots start in the square formation.  No tampering occurs.  The swarm travels
toward the shared goal in normal operation for the full fixed window (240 s).  This
scenario provides the reference Formation Error level (~6 mm) and the ideal
trajectory for TTE measurement.

### 2.2  Unmitigated attack

The attack is injected at time `t_inject` (described in §3).  The D-MUTRA
mitigation flag is **OFF**: the orchestrator monitors the blockchain and records the
attestation FAILURE event, but it deliberately does **not** publish
`/formation/excluded`.  Robot3 continues to spin; the formation remains deformed
for the rest of the 240 s run window.  This scenario shows the worst-case damage
when attestation succeeds at detecting the compromise but the operator chooses not to
act (or the isolation action is absent).

### 2.3  Mitigated attack (multiple SSP values)

Same injection as the unmitigated case, but mitigation is **ON**.  The attestation
sidecar re-hashes robot3's `.text` segment every SSP seconds and submits the result
to the blockchain.  Once the on-chain `AttestationCompleted` event reports a
FAILURE, the orchestrator publishes `/formation/excluded = "robot3"`.  The total
detection latency is approximately SSP + blockchain-commit time.  Experiments are
repeated for SSP ∈ {5, 10, 20, 30, 60} s.

---

## 3  Tampering

### Mechanism

The attack is injected with:

```bash
./tools/tamper.sh robot3 --target formation_agent
```

The script runs inside robot3's *sidecar container*, which shares the PID namespace
with the robot container and holds `CAP_SYS_PTRACE`.  It:

1. Locates the `formation_agent` process in the shared PID namespace via `pgrep`.
2. Reads `/proc/<pid>/maps` to find the base address of the executable's `r-xp`
   (read-execute) memory segment.
3. Writes **5 bytes of `0x90` (x86 NOP)** at offset `0xbff1` from that base,
   overwriting the `mulsd -0x20(%rbp),%xmm0` instruction inside `steer_cmd()`.
4. The patch is **in-memory only**; restarting the container restores the binary.

The target instruction is the `mulsd` (multiply scalar double) that computes
`ang = kp_ang_ × yaw_err` inside the `steer_cmd()` hot path.

### Effect on robot3

After the 5-byte NOP sled, `ang` in `steer_cmd()` is never multiplied by
`yaw_err` — it remains equal to `kp_ang_ = 2.5 rad/s` at all times.  The
consequence is:

- Robot3 rotates at a constant ω ≈ 2.5 rad/s (measured: ~1.77 rad/s after
  saturation and the control loop dynamics).
- `|yaw_err|` never falls below the alignment threshold (0.20 rad), so `lin = 0`
  permanently.  Robot3 **spins in place** without translating.
- Via the APF coupling, the healthy robots are attracted toward the position robot3
  implies for them, anchoring the formation centroid.  The centroid stalls and
  Formation Error grows monotonically.

### Detection by D-MUTRA

The attestation sidecar measures the SHA-256 hash of robot3's `.text` segment
(the first `r-xp` mapping of `formation_agent`) at every SSP boundary and calls the
blockchain smart contract.  Because the NOP patch changes 5 bytes of the measured
`.text`, the hash no longer matches the registered reference → the contract emits
`AttestationCompleted` with `result = FAILURE`.  The orchestrator receives this
event, logs it, and (in the mitigated scenario) isolates robot3.

---

## 4  Data Collection and Metrics

### 4.1  Recording

A `bag-recorder` service (defined in `docker-compose.formation.yml`) records a ROS 2
bag for every run.  Recorded topics include:

- `/robotN/ground_truth` — P3D plugin ground-truth pose (preferred source)
- `/robotN/odom` — encoder-based odometry (fallback if ground truth absent)
- `/formation/goal` — latched goal message; its receive timestamp is `t0`
- `/formation/excluded` — latched mitigation message; its receive timestamp is
  the authoritative `t_detect`
- `/clock` — Gazebo simulation clock (used for wall↔sim time conversion)

After each run the campaign driver also writes `inject-<tag>.json` with the UTC
wall-clock timestamp of the tamper injection (`t_inject_wall`), recorded
immediately after `tamper.sh` exits.

### 4.2  Offline metric extraction

`tools/formation_metrics.py` reads the bag with `rosbag2_py.SequentialReader`,
resamples all pose signals to a uniform 0.1 s grid, and computes every metric
*deterministically* from the recorded data.  It outputs:

- `formation-<tag>.csv` — per-step time series
- `metrics-<tag>.json` — headline summary

**Pose source priority:** ground-truth from `/robotN/ground_truth` (P3D plugin) is
used when available; encoder odometry (`/robotN/odom`) is used as a fallback.
Under the spin attack the encoder under-reports angular velocity, so ground truth
is essential for unbiased metrics.

**Active set and slot table** switch at `t_detect` in mitigated runs: before
detection all 4 robots participate with square slots; after detection the set drops
to {robot1, robot2, robot4} with triangle slots.

### 4.3  Metric definitions

#### Formation Error (FE)

Mean displacement of each active robot from its target slot:

```
FE(t) = (1 / |A(t)|) · Σ_{r ∈ A(t)}  ‖p_r(t) − (c_A(t) + slot_r)‖

where  A(t)    = active robot set at time t (size 4 before t_detect, 3 after)
       c_A(t)  = centroid of A(t)
       slot_r  = formation slot for robot r under the active table
       p_r(t)  = ground-truth (x, y) position of robot r
```

Unit: metres.  FE = 0 when the formation is geometrically perfect.

#### Cumulative Damage

Trapezoidal integral of FE over the entire run window [0, T]:

```
CumDamage = ∫₀ᵀ FE(t) dt   [trapezoidal rule at 0.1 s steps]
```

Unit: m·s.  Captures both the magnitude and duration of formation distortion.

#### Integrated Formation Error — compromise window (IFE_compromise)

```
IFE_compromise = ∫_{t_inject}^{t_end}  FE(t) dt

where  t_end = t_detect       (mitigated runs)
       t_end = mission end T  (unmitigated runs)
```

Unit: m·s.  Measures geometric damage accrued *while the attack is active and
uncorrected*.  The integration window differs between mitigated and unmitigated
scenarios, which makes direct cross-scenario comparison less straightforward (use
`IFE_remaining` when equal windows are required).

#### Integrated Formation Error — remaining mission (IFE_remaining)

```
IFE_remaining = ∫_{t_inject}^{T}  FE(t) dt
```

Same fixed window (injection → mission end) for every attacked scenario.  A lower
value indicates faster detection plus faster formation recovery, enabling a
paper-defensible comparison across SSP values.

#### Trajectory Tracking Error (TTE) RMSE

Cross-track distance of the active-group centroid from the ideal straight line
(initial centroid → goal):

```
cross_track(t) = |( c_A(t) − c_0 ) × û|   (scalar, metres)

TTE_RMSE = √( (1/N) · Σ_k  cross_track(t_k)² )

where  û = (goal − c_0) / ‖goal − c_0‖   (unit vector along ideal path)
       c_0 = active centroid at t = 0
```

Unit: metres.  Quantifies lateral deviation of the swarm from its intended straight-
line trajectory.

#### Along-track progress

```
along(t) = ( c_A(t) − c_0 ) · û   (metres along ideal path)
```

Reports how far the formation centroid has advanced toward the goal.  Under the spin
attack, along-track stalls (robot3 anchors the centroid) until mitigation fires.

---

## 5  Plot Descriptions

### `experiments/plot/formation_impact_trajectory.py`

A three-row spatial trajectory plot showing the full 2-D paths of all robots and
the formation centroid:

- **Row 1 — Baseline:** normal 4-robot square formation, no attack.  Goal marker at
  the right edge; start dots at the left.
- **Row 2 — Unmitigated:** same injection, mitigation OFF.  A gold scatter overlay
  encodes robot3's yaw orientation (`|ψ|/π`, where ψ ∈ [−π, π]) from injection
  onward; brighter gold indicates robot3 is facing away from the goal.  The centroid
  stalls because robot3 spins in place.
- **Row 3 — Mitigated (SSP = 10 s):** yaw overlay clipped at `t_detect`.  Coloured
  dots at all four robots' positions mark the detection moment.  After detection the
  healthy robots reform the triangle and the centroid resumes its path.

All rows share the same x/y axes (global frame, metres).

### `experiments/plot/formation_impact_fe.py`

Formation Error timeline for a single representative mitigated run (SSP = 10 s,
the rep whose `IFE_compromise` is closest to the group mean).  Time is aligned so
that `t = 0` corresponds to the injection moment.  Three curves:

- **Grey dashed — Baseline:** FE stays near ~6 mm throughout (negligible on the
  0–1.4 m axis, genuine residual from imperfect numerical convergence).
- **Red — Unmitigated:** FE rises from injection and never recovers.
- **Green — Mitigated (SSP = 10 s):** FE rises then drops sharply when the
  attestation FAILURE triggers isolation and the triangle reform.  The post-
  mitigation residual (~63 mm) is a genuine triangle formation error: robot4's
  triangle slot is (0, −4 m), farther from the centroid than its square slot, so
  the reformed triangle has a larger inherent geometric spread.

An annotation with arrow marks the detection/isolation instant on the mitigated
curve.

### `experiments/plot/formation_impact_ife.py`

Bar chart comparing `IFE_compromise` across all scenario variants:

- **Bar 1 (red) — Unmitigated attack:** `IFE_compromise` integrated from injection
  to mission end (worst case, no detection action taken).
- **Bars 2–5 (green gradient) — Mitigated, SSP = 60 / 30 / 20 / 10 s:** darker
  green for longer SSP (slower detection), lighter for shorter SSP (faster
  detection).  Error bars show one standard deviation across repetitions.
- **% reduction annotations** above each mitigated bar show the gain relative to
  the unmitigated baseline.
- A bracket below the x-axis groups the mitigated bars under the label "Mitigated
  attack under different SSP".

This plot is the primary quantitative argument that shorter SSP directly and
substantially reduces cumulative formation damage.

### `experiments/plot/lineplot_ttd_ife.py`

Combined bar + line plot for the mitigated scenarios at SSP ∈ {60, 30, 20, 10, 5} s:

- **Green bars (left axis) — IFE (m·s):** mean `IFE_compromise` per SSP with ±1 SD
  error bars.  Left axis label and tick values are coloured dark green.
- **Blue line + circles (right axis) — TTD (s):** mean Time-to-Detection per SSP,
  with min/max whiskers.  Right axis label and tick values are coloured blue.

The dual-axis design shows the trade-off on a single figure: lower SSP reduces the
integration window (IFE drops) at the cost of more frequent blockchain attestation
cycles.
