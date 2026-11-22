#!/usr/bin/env python3
"""Records when the messages of one ROS 2 topic arrive, for
experiment_robot_impact.py. Rows stay in memory; the experiment pulls them.

HTTP (COLLECTOR_PORT, default 7001):
  POST /start    clears the rows and starts recording
  POST /stop     stops recording
  GET  /results  {"arrivals": [{"t_recv_ns": ..., "ros_stamp_ns": ...}, ...]}
  GET  /status   {"recording": bool, "messages": received since start-up}

Topic: /robot{ROBOT_ID}{TOPIC_NAME}, of type MSG_TYPE (pkg.module/Class).
t_recv_ns: monotonic clock of this process when the message is handed to the
callback; the intervals between arrivals come from it. ros_stamp_ns: the
message stamp (the Gazebo simulation clock for /tf, of fixed steps).

For /tf, only robot_state_publisher's transforms are kept (the attested
process): the diff-drive plugin of gzserver publishes odom -> base_footprint
on the same topic.
"""

import importlib
import os
import threading
import time

import rclpy
from flask import Flask, jsonify
from rclpy.node import Node

ROBOT_ID = int(os.environ.get("ROBOT_ID", "1"))
PORT = int(os.environ.get("COLLECTOR_PORT", "7001"))
TOPIC = f"/robot{ROBOT_ID}{os.environ.get('TOPIC_NAME', '/tf')}"
_pkg, _cls = os.environ.get("MSG_TYPE", "tf2_msgs.msg/TFMessage").rsplit("/", 1)
MsgType = getattr(importlib.import_module(_pkg), _cls)

_lock = threading.Lock()
_recording = False
_rows = []
_messages = 0


def _stamp(msg):
    """The message stamp, or None for a diff-drive transform."""
    transforms = getattr(msg, "transforms", None)
    if transforms is not None:
        if transforms and all(t.child_frame_id == "base_footprint" for t in transforms):
            return None
        return transforms[0].header.stamp if transforms else 0
    header = getattr(msg, "header", None)
    return header.stamp if header is not None else 0


def _on_message(msg):
    global _messages
    t_recv_ns = time.monotonic_ns()
    stamp = _stamp(msg)
    if stamp is None:
        return
    ros_ns = stamp.sec * 1_000_000_000 + stamp.nanosec if stamp else 0
    with _lock:
        _messages += 1
        if _recording:
            _rows.append({"t_recv_ns": t_recv_ns, "ros_stamp_ns": ros_ns})


def _spin():
    rclpy.init()
    node = Node("topic_collector")
    node.create_subscription(MsgType, TOPIC, _on_message, 10)
    node.get_logger().info(f"Subscribed to {TOPIC}")
    rclpy.spin(node)


app = Flask(__name__)


@app.post("/start")
def start():
    global _recording, _rows
    with _lock:
        _rows, _recording = [], True
    return jsonify({"recording": True})


@app.post("/stop")
def stop():
    global _recording
    with _lock:
        _recording = False
        n = len(_rows)
    return jsonify({"recording": False, "arrivals": n})


@app.get("/results")
def results():
    with _lock:
        return jsonify({"arrivals": list(_rows)})


@app.get("/status")
def status():
    with _lock:
        return jsonify({"recording": _recording, "messages": _messages})


if __name__ == "__main__":
    threading.Thread(target=_spin, daemon=True).start()
    app.run(host="0.0.0.0", port=PORT, threaded=True)
