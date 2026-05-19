#!/usr/bin/env python3
"""
ROS2 topic arrival timestamp collector.

HTTP API (port 7000):
  POST /start  {"output_file": "/path/run1.csv", "duration_s": 120}
  POST /stop
  POST /reset
  GET  /status  → {"status": "idle|running|finished", "messages_received": N}

Subscribes to /robot{i}<TOPIC_NAME> for i in 1..N_ROBOTS.
  TOPIC_NAME  e.g. /scan  →  /robot1/scan, /robot2/scan, …   (default: /scan)
  MSG_TYPE    e.g. sensor_msgs.msg/LaserScan                  (default: sensor_msgs.msg/LaserScan)

ros_stamp_ns is extracted from msg.header.stamp when available; 0 otherwise.
Auto-stops after duration_s and writes CSV. Can also be stopped early via /stop.

Output CSV columns: robot, topic, ros_stamp_ns, wall_stamp_ns
"""

import csv
import importlib
import os
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from flask import Flask, request, jsonify

ROBOT_ID   = int(os.environ.get("ROBOT_ID", "1"))
PORT       = int(os.environ.get("COLLECTOR_PORT", "7000"))
TOPIC_NAME = os.environ.get("TOPIC_NAME", "/scan")          # suffix, e.g. /scan or /odom
_MSG_TYPE  = os.environ.get("MSG_TYPE", "sensor_msgs.msg/LaserScan")

# Dynamic message-type import: "pkg.subpkg/ClassName"
_pkg, _cls = _MSG_TYPE.rsplit("/", 1)
MsgType = getattr(importlib.import_module(_pkg), _cls)

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

class TopicCollector(Node):
    def __init__(self):
        super().__init__("topic_collector")
        self._total_msgs = 0
        full_topic = f"/robot{ROBOT_ID}{TOPIC_NAME}"
        self.create_subscription(
            MsgType,
            full_topic,
            lambda msg, r=f"robot{ROBOT_ID}", t=full_topic: self._cb(r, t, msg),
            10,
        )
        self.get_logger().info(
            f"Subscribed to {full_topic}  [{_MSG_TYPE}]"
        )

    def _cb(self, robot, topic, msg):
        self._total_msgs += 1
        if not _recording:
            return
        wall_ns = time.time_ns()

        # For TFMessage: only record messages from robot_state_publisher.
        # robot_state_publisher bundles all moving joint transforms in one message
        # (child_frame_id = wheel links, etc.).  The Gazebo diff-drive plugin
        # publishes a separate single-transform message with child_frame_id =
        # "base_footprint" (odom → base_footprint) — skip those.
        try:
            transforms = msg.transforms   # tf2_msgs/TFMessage
            if transforms and all(t.child_frame_id == "base_footprint"
                                  for t in transforms):
                return                    # diff-drive odometry transform — ignore
            stamp = transforms[0].header.stamp if transforms else None
        except AttributeError:
            try:
                stamp = msg.header.stamp  # standard stamped message
            except AttributeError:
                stamp = None

        ros_ns = (stamp.sec * 1_000_000_000 + stamp.nanosec) if stamp is not None else 0
        with _lock:
            _records.append({
                "robot":         robot,
                "topic":         topic,
                "ros_stamp_ns":  ros_ns,
                "wall_stamp_ns": wall_ns,
            })

    @property
    def total_msgs(self):
        return self._total_msgs


def _rclpy_spin():
    global _node
    rclpy.init()
    _node = TopicCollector()
    rclpy.spin(_node)
    _node.destroy_node()
    rclpy.shutdown()


# ── CSV writer ─────────────────────────────────────────────────────────────────

def _save(records, output_file):
    if not output_file:
        return 0
    Path(output_file).parent.mkdir(parents=True, exist_ok=True)
    with open(output_file, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["robot", "topic", "ros_stamp_ns", "wall_stamp_ns"])
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
