"""Resource collector for the experiment runner. Samples CPU, memory and
network of the given containers from the Docker stats stream (about one
sample per second each) and keeps the samples in memory.

  POST /start {"containers": [...]}  starts sampling, clearing earlier samples
  POST /stop                         stops sampling
  GET  /results                      {"samples": [...]}

A sample: t_ms (Unix ms), container, cpu_percent (100 = one core), mem_mb
(usage minus inactive page cache, as docker stats shows it), net_rx_kb and
net_tx_kb (received and sent since sampling started).
"""

import logging
import threading
import time
from typing import List

import docker
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

logging.basicConfig(format="%(asctime)s - %(levelname)s - %(message)s", level=logging.INFO)
log = logging.getLogger(__name__)

app = FastAPI(title="Resource collector")
client = docker.from_env()

lock = threading.Lock()
samples: List[dict] = []
stop = threading.Event()
threads: List[threading.Thread] = []


def cpu_percent(s):
    cpu = s["cpu_stats"]["cpu_usage"]["total_usage"] - s["precpu_stats"]["cpu_usage"]["total_usage"]
    system = s["cpu_stats"].get("system_cpu_usage", 0) - s["precpu_stats"].get("system_cpu_usage", 0)
    cpus = s["cpu_stats"].get("online_cpus") or 1
    return cpu / system * cpus * 100 if system > 0 and cpu >= 0 else 0.0


def mem_mb(s):
    m = s.get("memory_stats", {})
    stats = m.get("stats", {})
    inactive = stats.get("inactive_file", stats.get("total_inactive_file", 0))
    return max(0, m.get("usage", 0) - inactive) / 2**20


def net_bytes(s):
    nets = (s.get("networks") or {}).values()
    return sum(n["rx_bytes"] for n in nets), sum(n["tx_bytes"] for n in nets)


def sample(name):
    try:
        stream = client.containers.get(name).stats(stream=True, decode=True)
        next(stream)  # the first one has no previous CPU reading
        rx0 = tx0 = None
        for s in stream:
            if stop.is_set():
                break
            rx, tx = net_bytes(s)
            if rx0 is None:
                rx0, tx0 = rx, tx
            row = {"t_ms": int(time.time() * 1000), "container": name,
                   "cpu_percent": round(cpu_percent(s), 2), "mem_mb": round(mem_mb(s), 2),
                   "net_rx_kb": round((rx - rx0) / 1024, 1), "net_tx_kb": round((tx - tx0) / 1024, 1)}
            with lock:
                samples.append(row)
    except Exception as e:
        log.error("sampling %s stopped: %s", name, e)


class StartRequest(BaseModel):
    containers: List[str]


@app.post("/start")
def start(req: StartRequest):
    if any(t.is_alive() for t in threads):
        raise HTTPException(409, "already sampling: POST /stop first")
    for name in req.containers:
        try:
            client.containers.get(name)
        except docker.errors.NotFound:
            raise HTTPException(404, f"no container {name}")
    stop.clear()
    threads.clear()
    with lock:
        samples.clear()
    for name in req.containers:
        t = threading.Thread(target=sample, args=(name,), daemon=True)
        t.start()
        threads.append(t)
    return {"sampling": req.containers}


@app.post("/stop")
def stop_sampling():
    stop.set()
    for t in threads:
        t.join(timeout=3)
    return {"samples": len(samples)}


@app.get("/results")
def results():
    with lock:
        return {"samples": list(samples)}
