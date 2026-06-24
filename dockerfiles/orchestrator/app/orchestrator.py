#!/usr/bin/env python3
"""
Swarm mission orchestrator for the D-MUTRA runtime-tampering experiments.

It assigns a waypoint mission to a swarm of mission_agent nodes, tracks
mission progress over time, and — independently of SECaaS — watches the
blockchain for attestation FAILUREs. On a failure it identifies the
compromised robot and (if mitigation is enabled) reassigns that robot's
remaining waypoints to the healthy members so the mission can still complete.

This produces the paper's three curves (run the same mission per config):
  * baseline           : no attack            -> 100%
  * unmitigated attack : MITIGATION=false     -> progress plateaus < 100%
  * mitigated (D-MUTRA): MITIGATION=true      -> stalls, then recovers to 100%
The recovery delay in the mitigated curve is the on-chain detection latency.

How it learns of a failure (decoupled, read-only, no SECaaS/sidecar change):
  watch AttestationCompleted(id) -> GetAttestationInfo(id) -> result == Failure
  -> prover address -> robotN (mapped from config/robotN.json eth_address).

ROS interfaces (per robot, namespaced):
  pub  /robotN/assigned_waypoints  nav_msgs/Path            (re-published until acked)
  sub  /robotN/mission_status      std_msgs/String (JSON)   progress + heartbeat

Configuration (env vars):
  N_ROBOTS              number of robots                       (default 4)
  ROBOT_PREFIX          namespace prefix                       (default robot)
  CONFIG_DIR            dir with robotN.json agent configs      (default /config)
  ETH_NODE_URL          override RPC url from config            (default: from config)
  MISSION_FILE          JSON {robotN:[[x,y,yaw],...]}           (default: auto-generate)
  WAYPOINTS_PER_ROBOT   used only when auto-generating          (default 4)
  MITIGATION            true|false  reassign on failure         (default true)
  READY_TIMEOUT_S       wait for all agents before assigning    (default 120)
  MISSION_TIMEOUT_S     give up (used for the unmitigated case) (default 600)
  POLL_INTERVAL_S       blockchain log poll period              (default 1.0)
  LOG_DIR               output dir for the progress CSV         (default /logs)
  RUN_TAG               filename tag (e.g. ssp20000-mitigated)   (default run)
"""

import csv
import json
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock

from web3 import Web3
try:  # POA middleware moved across web3 versions; tolerate both.
    from web3.middleware import geth_poa_middleware as _POA
except Exception:  # pragma: no cover
    _POA = None

# AttestationResult enum (contract): None=0, Failure=1, Success=2
RESULT_FAILURE = 1


def _env(name, default):
    return os.environ.get(name, default)


def _env_bool(name, default):
    return _env(name, str(default)).strip().lower() in ("1", "true", "yes", "on")


class Orchestrator(Node):
    def __init__(self):
        super().__init__("orchestrator")

        self.n_robots = int(_env("N_ROBOTS", "4"))
        self.prefix = _env("ROBOT_PREFIX", "robot")
        self.config_dir = _env("CONFIG_DIR", "/config")
        self.mitigation = _env_bool("MITIGATION", True)
        self.ready_timeout = float(_env("READY_TIMEOUT_S", "120"))
        self.mission_timeout = float(_env("MISSION_TIMEOUT_S", "600"))
        self.poll_interval = float(_env("POLL_INTERVAL_S", "1.0"))
        self.log_dir = _env("LOG_DIR", "/logs")
        self.run_tag = _env("RUN_TAG", "run")
        # Minimum effective real-time factor for a run to count as valid. Below
        # this the simulator could not keep up with the wall clock the blockchain
        # detection runs on, so the run is not physically comparable (see below).
        self.rtf_min = float(_env("RTF_MIN", "0.9"))

        self.robots = [f"{self.prefix}{i}" for i in range(1, self.n_robots + 1)]

        # ── Chain wiring from the agent configs ──────────────────────────────
        rpc_url, contract_addr, abi, self.addr_to_robot = self._load_chain_config()
        self.w3 = Web3(Web3.HTTPProvider(rpc_url))
        if _POA is not None:
            try:
                self.w3.middleware_onion.inject(_POA, layer=0)
            except Exception:
                pass
        self.contract = self.w3.eth.contract(
            address=Web3.to_checksum_address(contract_addr), abi=abi)
        self.get_logger().info(
            f"Chain: rpc={rpc_url} contract={contract_addr} "
            f"connected={self.w3.is_connected()}")

        # ── Mission state ────────────────────────────────────────────────────
        # desired[robot] = list of PoseStamped (the robot's full current queue).
        self.desired = self._load_mission()
        # mission_total is fixed at the INITIAL assignment; reassignment moves
        # waypoints between robots but never changes the global denominator.
        self.mission_total = sum(len(v) for v in self.desired.values())
        # status[robot] = parsed last mission_status (completed, total_assigned, state)
        self.status = {r: None for r in self.robots}
        self.compromised = set()
        # frozen_completed[robot] = the verified progress at the instant its
        # attestation FAILED. After that point the robot is untrusted: we credit
        # only this frozen value, never its later self-reports. Keeps the global
        # count deterministic and immune to a compromised robot inflating it.
        self.frozen_completed = {}
        # distance[robot] = cumulative path length driven, integrated from odom.
        # This is the RTF-independent mission-impact metric: it measures the
        # PHYSICAL travel cost of mitigation (the extra distance healthy robots
        # drive to cover a compromised robot's orphaned waypoints), unaffected by
        # simulator real-time factor, timer jitter, or host load — unlike duration.
        self.distance = {r: 0.0 for r in self.robots}
        self.last_xy = {r: None for r in self.robots}
        self.failure_events = []           # (t_rel, robot)
        self.started = False
        self.finished = False
        self.start_time = None
        self.boot_time = time.monotonic()
        # Sim-time clock (from Gazebo /clock), used purely to compute the
        # effective real-time factor of the run: RTF = sim_elapsed / wall_elapsed.
        # A run with RTF well below 1 means the simulator could not keep up with
        # the wall clock that the blockchain detection pipeline runs on, so the
        # mission-impact figures are not physically comparable — flagged invalid
        # downstream. Computed authoritatively here (gzserver does not log RTF).
        self.sim_time_now = None
        self.sim_time_start = None

        # ── ROS interfaces ───────────────────────────────────────────────────
        assign_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        # Odom is published best-effort by the Gazebo diff-drive plugin; a
        # best-effort subscriber is compatible with both reliable and
        # best-effort publishers, so this avoids any QoS-mismatch dropout.
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        # /clock is global (not namespaced). Best-effort subscriber is compatible
        # with Gazebo's publisher regardless of its reliability setting.
        self.create_subscription(Clock, "/clock", self._on_clock, odom_qos)
        self.assign_pub = {}
        for r in self.robots:
            self.assign_pub[r] = self.create_publisher(
                Path, f"/{r}/assigned_waypoints", assign_qos)
            self.create_subscription(
                String, f"/{r}/mission_status",
                lambda msg, rb=r: self._on_status(rb, msg), 10)
            self.create_subscription(
                Odometry, f"/{r}/odom",
                lambda msg, rb=r: self._on_odom(rb, msg), odom_qos)

        # ── Logging ──────────────────────────────────────────────────────────
        os.makedirs(self.log_dir, exist_ok=True)
        self.csv_path = os.path.join(self.log_dir, f"mission-{self.run_tag}.csv")
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(
            ["t_rel_s", "wall_iso", "pct", "completed", "total"]
            + [f"done_{r}" for r in self.robots]
            + ["healthy", "event"])
        self.get_logger().info(
            f"Mission: {self.mission_total} waypoints across {self.n_robots} robots; "
            f"mitigation={'ON' if self.mitigation else 'OFF'}; log={self.csv_path}")

        # ── Threads / timers ─────────────────────────────────────────────────
        self._chain_stop = threading.Event()
        self._chain_thread = threading.Thread(target=self._watch_chain, daemon=True)
        self._chain_thread.start()
        self.create_timer(0.5, self._control_loop)

    # ── Config / mission loading ─────────────────────────────────────────────
    def _load_chain_config(self):
        ref = os.path.join(self.config_dir, f"{self.prefix}1.json")
        with open(ref) as f:
            c = json.load(f)
        rpc_url = _env("ETH_NODE_URL", "") or c["eth_node_url"]
        contract_addr = c["contract_address"]
        abi = c["contract_abi"]
        addr_to_robot = {}
        for r in self.robots:
            p = os.path.join(self.config_dir, f"{r}.json")
            try:
                with open(p) as f:
                    addr_to_robot[json.load(f)["eth_address"].lower()] = r
            except FileNotFoundError:
                self.get_logger().warn(f"config missing for {r}: {p}")
        return rpc_url, contract_addr, abi, addr_to_robot

    def _load_mission(self):
        mf = _env("MISSION_FILE", "")
        if mf and os.path.exists(mf):
            with open(mf) as f:
                raw = json.load(f)
            self.get_logger().info(f"Mission loaded from {mf}")
            return {r: [self._pose(*wp) for wp in raw.get(r, [])] for r in self.robots}
        # Auto-generate a placeholder mission. PROVIDE A MISSION_FILE for real
        # runs — these coordinates are world-agnostic and only meant for smoke
        # tests on the empty world.
        k = int(_env("WAYPOINTS_PER_ROBOT", "4"))
        self.get_logger().warn(
            "No MISSION_FILE — auto-generating placeholder waypoints "
            "(supply MISSION_FILE for real experiments).")
        mission = {}
        for i, r in enumerate(self.robots):
            lane = (-1.0) ** i * (1.0 + 0.5 * (i // 2))
            mission[r] = [self._pose(lane, -1.0 + j * (2.0 / max(1, k - 1)), 0.0)
                          for j in range(k)]
        return mission

    def _pose(self, x, y, yaw=0.0):
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.z = math.sin(float(yaw) / 2.0)
        ps.pose.orientation.w = math.cos(float(yaw) / 2.0)
        return ps

    # ── ROS callbacks ────────────────────────────────────────────────────────
    def _on_status(self, robot, msg):
        try:
            self.status[robot] = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_clock(self, msg):
        self.sim_time_now = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_odom(self, robot, msg):
        # Integrate cumulative driven distance from successive odom samples.
        # Odom is continuous (no teleports), but cap each delta defensively to
        # reject any single glitch sample; at ~0.26 m/s a real step is tiny.
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        prev = self.last_xy[robot]
        if prev is not None:
            d = math.hypot(x - prev[0], y - prev[1])
            if d < 0.5:
                self.distance[robot] += d
        self.last_xy[robot] = (x, y)

    def _publish_assignment(self, robot):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = self.desired[robot]
        self.assign_pub[robot].publish(path)

    def _acked(self, robot):
        st = self.status[robot]
        return st is not None and int(st.get("total_assigned", -1)) == len(self.desired[robot])

    def _ready(self, robot):
        # A robot is ready to receive its mission only once its agent reports it
        # can actually navigate (AMCL localized + Nav2 action server active).
        # Heartbeat presence alone is NOT enough — the agent heartbeats from its
        # first tick, ~20s before Nav2 is usable, which would start the clock too
        # early and make goals abort instantly.
        st = self.status[robot]
        return st is not None and st.get("ready") is True

    # ── Main control loop (0.5 Hz progress sampling) ─────────────────────────
    def _control_loop(self):
        if self.finished:
            return
        now = time.monotonic()

        if not self.started:
            all_ready = all(self._ready(r) for r in self.robots)
            if all_ready or (now - self.boot_time) > self.ready_timeout:
                missing = [r for r in self.robots if not self._ready(r)]
                if missing:
                    self.get_logger().warn(
                        f"Ready timeout — proceeding without (not navigation-ready): {missing}")
                for r in self.robots:
                    self._publish_assignment(r)
                self.started = True
                self.start_time = now
                self.sim_time_start = self.sim_time_now
                self._log_row(event="mission_start")
                self.get_logger().info("Mission assigned; clock started.")
            return

        # Re-publish any assignment the agent hasn't acked yet (covers late
        # joins, dropped messages, and freshly reassigned queues).
        for r in self.robots:
            if r not in self.compromised and not self._acked(r):
                self._publish_assignment(r)

        self._log_row()

        completed = self._global_completed()
        if completed >= self.mission_total:
            self._log_row(event="mission_complete")
            self.get_logger().info(
                f"Mission complete: {completed}/{self.mission_total} "
                f"in {now - self.start_time:.1f}s")
            self._finish()
        elif (now - self.start_time) > self.mission_timeout:
            self._log_row(event="mission_timeout")
            self.get_logger().warn(
                f"Mission timeout: {completed}/{self.mission_total} "
                f"({100.0 * completed / self.mission_total:.1f}%)")
            self._finish()

    def _credited(self, robot):
        # Credit a compromised robot only up to its frozen (verified) progress;
        # otherwise use its live self-report.
        if robot in self.frozen_completed:
            return self.frozen_completed[robot]
        st = self.status.get(robot)
        return int(st.get("completed", 0)) if st else 0

    def _global_completed(self):
        return sum(self._credited(r) for r in self.robots)

    def _log_row(self, event=""):
        completed = self._global_completed()
        pct = 100.0 * completed / self.mission_total if self.mission_total else 0.0
        t_rel = (time.monotonic() - self.start_time) if self.start_time else 0.0
        per = [self._credited(r) for r in self.robots]
        healthy = self.n_robots - len(self.compromised)
        self.csv.writerow(
            [f"{t_rel:.2f}", time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
             f"{pct:.2f}", completed, self.mission_total] + per + [healthy, event])
        self.csv_file.flush()

    def _finish(self):
        self.finished = True
        self._chain_stop.set()
        wall_dur = (time.monotonic() - self.start_time) if self.start_time else 0.0
        # Effective real-time factor over the mission window. If /clock was never
        # received (no sim), leave it null and the run is treated as ungated.
        sim_dur = None
        eff_rtf = None
        if self.sim_time_start is not None and self.sim_time_now is not None:
            sim_dur = self.sim_time_now - self.sim_time_start
            if wall_dur > 0:
                eff_rtf = round(sim_dur / wall_dur, 3)
        summary = {
            "run_tag": self.run_tag,
            "mitigation": self.mitigation,
            "mission_total": self.mission_total,
            "completed": self._global_completed(),
            "duration_s": wall_dur,
            "sim_duration_s": round(sim_dur, 3) if sim_dur is not None else None,
            "effective_rtf": eff_rtf,
            # Validity gate: a run is valid only if the simulator sustained
            # ~real-time. null when /clock was never seen (gate not applicable).
            "valid_run": (eff_rtf >= self.rtf_min) if eff_rtf is not None else None,
            "compromised": sorted(self.compromised),
            "failure_events": self.failure_events,
            # RTF-independent mission-impact metric (see __init__): per-robot and
            # total path length driven. The mitigation's travel overhead is
            # distance_total_m(mitigated) - distance_total_m(baseline).
            "distance_m": {r: round(self.distance[r], 3) for r in self.robots},
            "distance_total_m": round(sum(self.distance.values()), 3),
            "csv": self.csv_path,
        }
        with open(os.path.join(self.log_dir, f"summary-{self.run_tag}.json"), "w") as f:
            json.dump(summary, f, indent=2)
        self.csv_file.close()
        self.get_logger().info(f"Summary written. {json.dumps(summary)}")

    # ── Blockchain watcher (independent, read-only) ──────────────────────────
    def _watch_chain(self):
        try:
            from_block = self.w3.eth.block_number
        except Exception as e:
            self.get_logger().error(f"Chain unreachable, watcher disabled: {e}")
            return
        event = self.contract.events.AttestationCompleted
        while not self._chain_stop.is_set():
            try:
                latest = self.w3.eth.block_number
                if latest >= from_block:
                    for log in event.get_logs(fromBlock=from_block, toBlock=latest):
                        self._handle_completed(log["args"]["id"])
                    from_block = latest + 1
            except Exception as e:
                self.get_logger().warn(f"chain poll error: {e}")
            self._chain_stop.wait(self.poll_interval)

    def _handle_completed(self, att_id):
        try:
            prover, _verifier, result, _ts = self.contract.functions.\
                GetAttestationInfo(att_id).call()
        except Exception:
            return  # not yet readable / reverted; ignore
        if int(result) != RESULT_FAILURE:
            return
        robot = self.addr_to_robot.get(str(prover).lower())
        if robot is None or robot in self.frozen_completed:
            return  # unknown prover, or this robot's failure already handled
        # Freeze the robot's credited progress at this instant — from here on it
        # is untrusted (applies to both the mitigated and unmitigated cases, so
        # the plateau / orphaned set is deterministic).
        done_now = int((self.status.get(robot) or {}).get("completed", 0))
        self.frozen_completed[robot] = done_now
        t_rel = (time.monotonic() - self.start_time) if self.start_time else 0.0
        self.failure_events.append((round(t_rel, 2), robot))
        self.get_logger().warn(
            f"⚠️ Attestation FAILURE for {robot} (prover={prover}) at t={t_rel:.1f}s; "
            f"freezing credited progress at {done_now}")
        if self.mitigation:
            self.compromised.add(robot)
            self._reassign(robot)
            self._log_row(event=f"reassign:{robot}")
        else:
            self._log_row(event=f"failure_detected:{robot}")

    @staticmethod
    def _xy(ps):
        return (ps.pose.position.x, ps.pose.position.y)

    def _region_centroid(self, robot):
        # Mean (x, y) of a robot's currently-queued waypoints — a stable proxy
        # for its patrol region, independent of where it happens to be at any
        # instant. Used to pick the rescuer deterministically.
        q = self.desired.get(robot) or []
        if not q:
            return (0.0, 0.0)
        xs = [self._xy(p) for p in q]
        return (sum(x for x, _ in xs) / len(xs), sum(y for _, y in xs) / len(xs))

    def _reassign(self, victim):
        # Move ALL of the victim's not-yet-completed waypoints to a SINGLE
        # rescuer, appended in order, so recovery is one deterministic, measurable
        # traversal (not a per-waypoint split). Append-only keeps each agent's
        # completed prefix valid; the global denominator is unchanged.
        #
        # The rescuer is chosen PHASE-INDEPENDENTLY: the healthy robot whose
        # patrol-region centroid is nearest the victim's last VERIFIED waypoint.
        # Both references are fixed every run, so the rescuer's identity is fully
        # deterministic (unlike using live odom, whose mission-phase drift could
        # flip the choice between two equidistant neighbours run-to-run).
        done = self.frozen_completed.get(
            victim, int((self.status.get(victim) or {}).get("completed", 0)))
        remaining = self.desired[victim][done:]
        healthy = [r for r in self.robots if r not in self.compromised]
        if not healthy or not remaining:
            self.get_logger().warn(f"Nothing to reassign for {victim}.")
            return
        # Victim's last verified position = the last waypoint it actually
        # completed (deterministic); fall back to its frozen odom, then origin.
        if done > 0:
            vref = self._xy(self.desired[victim][done - 1])
        else:
            vref = self.last_xy.get(victim) or (0.0, 0.0)
        rescuer = min(
            healthy,
            key=lambda r: math.hypot(self._region_centroid(r)[0] - vref[0],
                                     self._region_centroid(r)[1] - vref[1]))
        self.desired[rescuer].extend(remaining)
        self.desired[victim] = self.desired[victim][:done]
        self._publish_assignment(rescuer)
        rc = self._region_centroid(rescuer)
        self.get_logger().info(
            f"Reassigned all {len(remaining)} waypoint(s) from {victim} "
            f"(last verified at {vref[0]:.2f},{vref[1]:.2f}) to nearest healthy "
            f"robot {rescuer} (region centroid {rc[0]:.2f},{rc[1]:.2f}).")


def main():
    rclpy.init()
    node = Orchestrator()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
