#!/usr/bin/env python3
"""
ROS2 /scan topic arrival timestamp collector.

HTTP API (port 7000):
  POST /start  {"output_file": "/path/run1.csv", "duration_s": 120}
  POST /stop
  POST /reset
  GET  /status  → {"status": "idle|running|finished", "messages_received": N}

Records wall-clock arrival time of every /robot{i}/scan message while running.
Auto-stops after duration_s and writes CSV. Can also be stopped early via /stop.

Output CSV columns: robot, ros_stamp_ns, wall_stamp_ns
"""

import csv
import os
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from flask import Flask, request, jsonify

N_ROBOTS = int(os.environ.get("N_ROBOTS", "4"))
PORT     = int(os.environ.get("COLLECTOR_PORT", "7000"))

app = Flask(__name__)

# ── Shared state ───────────────────────────────────────────────────────────────
_lock        = threading.Lock()
_status      = "idle"       # idle | running | finished
_recording   = False
_records     = []
_output_file = None
_timer       = None
_node        = None         # set after rclpy init


# ── ROS2 node ──────────────────────────────────────────────────────────────────

class ScanCollector(Node):
    def __init__(self):
        super().__init__("scan_collector")
        self._total_msgs = 0
        for i in range(1, N_ROBOTS + 1):
            self.create_subscription(
                LaserScan,
                f"/robot{i}/scan",
                lambda msg, r=f"robot{i}": self._cb(r, msg),
                10,
            )
        self.get_logger().info(f"Subscribed to {N_ROBOTS} /robotX/scan topics")

    def _cb(self, robot, msg):
        self._total_msgs += 1
        if not _recording:
            return
        wall_ns = time.time_ns()
        ros_ns  = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        with _lock:
            _records.append({
                "robot":        robot,
                "ros_stamp_ns": ros_ns,
                "wall_stamp_ns": wall_ns,
            })

    @property
    def total_msgs(self):
        return self._total_msgs


def _rclpy_spin():
    global _node
    rclpy.init()
    _node = ScanCollector()
    rclpy.spin(_node)
    _node.destroy_node()
    rclpy.shutdown()


# ── CSV writer ─────────────────────────────────────────────────────────────────

def _save(records, output_file):
    if not output_file:
        return 0
    Path(output_file).parent.mkdir(parents=True, exist_ok=True)
    with open(output_file, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["robot", "ros_stamp_ns", "wall_stamp_ns"])
        writer.writeheader()
        writer.writerows(records)
    print(f"[OK] {len(records)} records → {output_file}")
    return len(records)


# ── Internal stop ──────────────────────────────────────────────────────────────

def _do_stop():
    global _status, _recording, _records, _output_file, _timer
    with _lock:
        if not _recording:
            return 0
        _recording = False
        records = list(_records)
        out     = _output_file

    n = _save(records, out)
    with _lock:
        _status = "finished"
    return n


# ── Flask endpoints ────────────────────────────────────────────────────────────

@app.route("/start", methods=["POST"])
def start():
    global _status, _recording, _records, _output_file, _timer

    data = request.get_json() or {}
    with _lock:
        if _status == "running":
            return jsonify({"error": "already running"}), 400
        _output_file = data.get("output_file")
        duration_s   = float(data.get("duration_s", 120))
        _records     = []
        _recording   = True
        _status      = "running"

    _timer = threading.Timer(duration_s, _do_stop)
    _timer.start()
    print(f"[START] → {_output_file}  duration={duration_s}s")
    return jsonify({"status": "started"})


@app.route("/stop", methods=["POST"])
def stop():
    global _timer
    if _timer:
        _timer.cancel()
    n = _do_stop()
    return jsonify({"status": "stopped", "records": n})


@app.route("/reset", methods=["POST"])
def reset():
    global _status, _recording, _records, _output_file
    with _lock:
        if _status == "running":
            return jsonify({"error": "cannot reset while running"}), 400
        _status      = "idle"
        _records     = []
        _output_file = None
    return jsonify({"status": "idle"})


@app.route("/status", methods=["GET"])
def status():
    with _lock:
        s = _status
    msgs = _node.total_msgs if _node else 0
    return jsonify({"status": s, "messages_received": msgs})


# ── Entry point ────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    t = threading.Thread(target=_rclpy_spin, daemon=True)
    t.start()
    time.sleep(2)   # allow rclpy to initialise and subscriptions to register
    print(f"[Collector] Listening on 0.0.0.0:{PORT}")
    app.run(host="0.0.0.0", port=PORT)
