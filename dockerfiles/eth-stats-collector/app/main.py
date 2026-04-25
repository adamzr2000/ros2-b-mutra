# app/main.py
import glob
import logging
import os
import re
from contextlib import asynccontextmanager
from typing import Optional

from fastapi import FastAPI, HTTPException, Query
from pydantic import BaseModel, Field

from app.monitoring import BlockchainMonitor

logging.basicConfig(format="%(asctime)s - %(levelname)s - %(message)s", level=logging.INFO)
logger = logging.getLogger(__name__)


# ---------- Models ----------

class StartRequest(BaseModel):
    rpc_url: str = Field(..., description="Besu RPC endpoint, e.g. http://host.docker.internal:21001")
    poll_interval: float = Field(1.0, description="Seconds between block polls")
    csv_dir: Optional[str] = Field(None, description="Directory to write the CSV file into")
    csv_name: Optional[str] = Field(None, description="CSV filename (without .csv). Auto-named runN if omitted.")

class MessageResponse(BaseModel):
    success: bool = True
    message: str

class StatusResponse(BaseModel):
    running: bool
    blocks_collected: int
    rpc_url: Optional[str] = None
    csv_path: Optional[str] = None


# ---------- Helpers ----------

def _next_run_csv(csv_dir: str) -> str:
    os.makedirs(csv_dir, exist_ok=True)
    pattern = os.path.join(csv_dir, "blockchain-run*.csv")
    existing = glob.glob(pattern)
    max_idx = 0
    rx = re.compile(r"blockchain-run(\d+)\.csv$")
    for path in existing:
        m = rx.search(os.path.basename(path))
        if m:
            max_idx = max(max_idx, int(m.group(1)))
    return os.path.join(csv_dir, f"blockchain-run{max_idx + 1}.csv")

def _resolve_csv_path(csv_dir: Optional[str], csv_name: Optional[str]) -> Optional[str]:
    if not csv_dir:
        return None
    if csv_name:
        name = csv_name if csv_name.endswith(".csv") else csv_name + ".csv"
        return os.path.join(csv_dir, name)
    return _next_run_csv(csv_dir)


# ---------- Lifespan ----------

@asynccontextmanager
async def lifespan(app: FastAPI):
    app.state.monitor: Optional[BlockchainMonitor] = None
    app.state.csv_path: Optional[str] = None
    yield
    mon: Optional[BlockchainMonitor] = app.state.monitor
    if mon and mon.is_running():
        mon.stop()

app = FastAPI(title="Ethereum block stats collector", version="1.0.0", lifespan=lifespan)


# ---------- Endpoints ----------

@app.get("/", tags=["Health"])
def root():
    return {"message": "eth-stats-collector running"}

@app.post("/monitor/start", response_model=MessageResponse, tags=["Monitoring"])
def monitor_start(req: StartRequest):
    mon: Optional[BlockchainMonitor] = app.state.monitor
    if mon and mon.is_running():
        raise HTTPException(status_code=409, detail="Monitor already running. Stop it first.")

    csv_path = _resolve_csv_path(req.csv_dir, req.csv_name)
    monitor = BlockchainMonitor(
        rpc_url=req.rpc_url,
        poll_interval=req.poll_interval,
        csv_path=csv_path,
    )
    monitor.start()
    app.state.monitor = monitor
    app.state.csv_path = csv_path

    msg = f"Monitor started (rpc={req.rpc_url})"
    if csv_path:
        msg += f", csv={csv_path}"
    return MessageResponse(message=msg)

@app.post("/monitor/stop", response_model=MessageResponse, tags=["Monitoring"])
def monitor_stop():
    mon: Optional[BlockchainMonitor] = app.state.monitor
    if not mon or not mon.is_running():
        raise HTTPException(status_code=400, detail="No active monitor to stop.")
    stats = mon.get_stats()
    mon.stop()
    app.state.monitor = None
    return MessageResponse(message=f"Monitor stopped. Blocks collected: {stats['blocks_collected']}.")

@app.get("/monitor/status", response_model=StatusResponse, tags=["Monitoring"])
def monitor_status():
    mon: Optional[BlockchainMonitor] = app.state.monitor
    if not mon:
        return StatusResponse(running=False, blocks_collected=0)
    stats = mon.get_stats()
    return StatusResponse(
        running=stats["running"],
        blocks_collected=stats["blocks_collected"],
        rpc_url=mon.rpc_url,
        csv_path=app.state.csv_path,
    )

@app.get("/monitor/last", tags=["Monitoring"])
def monitor_last():
    mon: Optional[BlockchainMonitor] = app.state.monitor
    if not mon:
        raise HTTPException(status_code=404, detail="No monitor active.")
    return {"sample": mon.get_last_sample()}
