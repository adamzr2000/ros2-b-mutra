#!/usr/bin/env python3
"""
Swarm FORMATION orchestrator (CONTROL-ONLY) for the D-MUTRA experiments.

Companion to orchestrator.py (the patrol experiment). This node owns ONLY the
experiment-critical, must-be-live decisions; all metric computation is decoupled
and done offline from a ros2 bag (see bag-recorder in docker-compose.formation.yml
and tools/formation_metrics.py). Its jobs:

  1. wait for the formation_agent swarm to be ready, then publish the shared
     straight-line goal once (/formation/goal, latched) — this also marks t0 in
     the recorded bag;
  2. watch the blockchain for an attestation FAILURE (identical machinery to
     orchestrator.py), and on detection isolate the compromised robot by
     publishing its id on /formation/excluded (latched) — this is the mitigation
     event, and its bag timestamp is the authoritative detect time offline;
  3. run for a FIXED window (RUN_DURATION_S) so every scenario/SSP integrates the
     damage metrics over the SAME time base (a fixed window, not goal-reach,
     keeps baseline / unmitigated / mitigated directly comparable), then exit.

It deliberately does NOT subscribe to odom or compute Formation Error / TTE /
cumulative damage — those are recomputed offline from the bag, so a metric-formula
change never requires re-running an experiment.

Configuration (env vars):
  N_ROBOTS / ROBOT_PREFIX / CONFIG_DIR / ETH_NODE_URL / MITIGATION /
  READY_TIMEOUT_S / POLL_INTERVAL_S / LOG_DIR / RUN_TAG       -- as orchestrator.py
  RUN_DURATION_S  fixed run window after the goal is published   (default 240)
  GOAL_X, GOAL_Y  shared straight-line goal for the formation     (default 8, 0)
  EXCLUDE_ROBOT   which robot is isolated on FAILURE (bookkeeping) (default robot3)
"""

import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from geometry_msgs.msg import Point

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


class FormationOrchestrator(Node):
    def __init__(self):
        super().__init__("formation_orchestrator")

        self.n_robots = int(_env("N_ROBOTS", "4"))
        self.prefix = _env("ROBOT_PREFIX", "robot")
        self.config_dir = _env("CONFIG_DIR", "/config")
        self.mitigation = _env_bool("MITIGATION", True)
        self.ready_timeout = float(_env("READY_TIMEOUT_S", "120"))
        self.run_duration = float(_env("RUN_DURATION_S", "240"))
        self.poll_interval = float(_env("POLL_INTERVAL_S", "1.0"))
        self.log_dir = _env("LOG_DIR", "/logs")
        self.run_tag = _env("RUN_TAG", "run")

        self.goal = (float(_env("GOAL_X", "8.0")), float(_env("GOAL_Y", "0.0")))
        self.exclude_robot = _env("EXCLUDE_ROBOT", "robot3")

        self.robots = [f"{self.prefix}{i}" for i in range(1, self.n_robots + 1)]

        # ── Chain wiring from the agent configs (identical to orchestrator.py) ──
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

        # ── State (control-only) ───────────────────────────────────────────────
        self.ready = {r: False for r in self.robots}
        self.compromised = set()
        self.excluded = set()
        self.failure_events = []           # (t_rel, robot)
        self.started = False
        self.finished = False
        self.start_time = None
        self.boot_time = time.monotonic()

        # ── ROS interfaces ───────────────────────────────────────────────────
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.goal_pub = self.create_publisher(Point, "/formation/goal", latched)
        self.excluded_pub = self.create_publisher(String, "/formation/excluded", latched)
        # Readiness only — a lightweight String heartbeat, NOT used for metrics.
        for r in self.robots:
            self.create_subscription(
                String, f"/{r}/formation_status",
                lambda msg, rb=r: self._on_status(rb, msg), 10)

        os.makedirs(self.log_dir, exist_ok=True)
        self.get_logger().info(
            f"Formation (control-only): {self.n_robots} robots, goal={self.goal}, "
            f"window={self.run_duration}s; mitigation={'ON' if self.mitigation else 'OFF'}")

        # ── Threads / timers ─────────────────────────────────────────────────
        self._chain_stop = threading.Event()
        self._chain_thread = threading.Thread(target=self._watch_chain, daemon=True)
        self._chain_thread.start()
        self.create_timer(0.5, self._control_loop)

    # ── Config loading (identical to orchestrator.py) ────────────────────────
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

    # ── ROS callbacks ────────────────────────────────────────────────────────
    def _on_status(self, robot, msg):
        try:
            st = json.loads(msg.data)
            self.ready[robot] = bool(st.get("ready"))
        except json.JSONDecodeError:
            pass

    # ── Control loop (0.5 Hz) ────────────────────────────────────────────────
    def _control_loop(self):
        if self.finished:
            return
        now = time.monotonic()

        if not self.started:
            all_ready = all(self.ready[r] for r in self.robots)
            if all_ready or (now - self.boot_time) > self.ready_timeout:
                missing = [r for r in self.robots if not self.ready[r]]
                if missing:
                    self.get_logger().warn(f"Ready timeout — proceeding without: {missing}")
                g = Point(); g.x, g.y = self.goal[0], self.goal[1]
                self.goal_pub.publish(g)
                self.started = True
                self.start_time = now
                self.get_logger().info(
                    f"Goal published {self.goal} (t0); running fixed window {self.run_duration}s.")
            return

        if (now - self.start_time) >= self.run_duration:
            self.get_logger().info(f"Run window elapsed ({self.run_duration}s).")
            self._finish()

    def _finish(self):
        self.finished = True
        self._chain_stop.set()
        wall_dur = (time.monotonic() - self.start_time) if self.start_time else 0.0
        # Lightweight run metadata only — the headline metrics (Formation Error,
        # cumulative damage, TTE) are computed offline from the bag.
        events = {
            "run_tag": self.run_tag,
            "mode": "formation",
            "mitigation": self.mitigation,
            "n_robots": self.n_robots,
            "goal": list(self.goal),
            "duration_s": round(wall_dur, 2),
            "compromised": sorted(self.compromised),
            "failure_events": self.failure_events,
        }
        with open(os.path.join(self.log_dir, f"events-{self.run_tag}.json"), "w") as f:
            json.dump(events, f, indent=2)
        self.get_logger().info(f"Events written. {json.dumps(events)}")

    # ── Blockchain watcher (identical to orchestrator.py) ────────────────────
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
            return
        if int(result) != RESULT_FAILURE:
            return
        robot = self.addr_to_robot.get(str(prover).lower())
        if robot is None or robot in self.compromised:
            return
        self.compromised.add(robot)
        t_rel = (time.monotonic() - self.start_time) if self.start_time else 0.0
        self.failure_events.append((round(t_rel, 2), robot))
        self.get_logger().warn(
            f"⚠️ Attestation FAILURE for {robot} (prover={prover}) at t={t_rel:.1f}s")
        if self.mitigation:
            self._isolate(robot)
        # In the unmitigated case we record the detection but do NOT isolate —
        # the robot keeps veering and the formation stays deformed.

    def _isolate(self, robot):
        # Announce the isolation (latched) so every healthy agent reconfigures to
        # the reduced formation. The bag timestamp of this message is the
        # authoritative mitigation time used offline. Pre-defined topology.
        self.excluded.add(robot)
        msg = String(); msg.data = robot
        self.excluded_pub.publish(msg)
        self.get_logger().info(f"Isolated {robot}; published /formation/excluded.")


def main():
    rclpy.init()
    node = FormationOrchestrator()
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
