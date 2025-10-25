# utils.py
import os
import json
import time
import threading
import tempfile
import random
from typing import Optional, Dict, Any
import re

import logging

logger = logging.getLogger(__name__)

# One lock per agent file (process-local)
_JSON_FILE_LOCKS = {}

def _get_json_lock(path):
    lock = _JSON_FILE_LOCKS.get(path)
    if lock is None:
        lock = threading.Lock()
        _JSON_FILE_LOCKS[path] = lock
    return lock

def _ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)

def _atomic_write_json(path: str, data: dict):
    dirpath = os.path.dirname(path) or "."
    # ensure dir is accessible
    os.makedirs(dirpath, exist_ok=True)
    # optional: make sure dir perms are OK for reading
    try:
        os.chmod(dirpath, 0o755)
    except PermissionError:
        pass

    with tempfile.NamedTemporaryFile("w", dir=dirpath, delete=False) as tmp:
        json.dump(data, tmp, separators=(",", ":"), ensure_ascii=False)
        tmp.flush()
        os.fsync(tmp.fileno())
        # ensure final file is 0644 regardless of umask
        try:
            os.fchmod(tmp.fileno(), 0o644)
        except AttributeError:
            import os as _os
            _os.chmod(tmp.name, 0o644)
        tmp_path = tmp.name
    os.replace(tmp_path, path)

def _load_or_init_results(json_path: str, participant: str, now_ms: int):
    if os.path.isfile(json_path):
        try:
            with open(json_path, "r") as f:
                return json.load(f)
        except Exception:
            # Corrupted or empty file -> reinit (conservative)
            pass
    return {
        "participant": participant,
        "clock": "unix_epoch",
        "time_unit": "ms",
        "t_start": now_ms,
        "prover": [],
        "verifier": [],
        "oracle": [], 
    }

def ensure_results_initialized(json_dir: str, participant: str):
    _ensure_dir(json_dir)
    json_path = os.path.join(json_dir, f"{participant}.json")
    lock = _get_json_lock(json_path)
    now_ms = int(time.time() * 1000)
    with lock:
        if os.path.isfile(json_path):
            try:
                data = json.load(open(json_path, "r"))
                # If already initialized with t_start, do nothing
                if "t_start" in data:
                    return
            except Exception:
                pass
        # Fresh init
        data = {
            "participant": participant,
            "clock": "unix_epoch",
            "time_unit": "ms",
            "t_start": now_ms,
            "prover": [],
            "verifier": [],
            "oracle": [],
        }
        _atomic_write_json(json_path, data)

def export_attestation_times_json(participant_name: str,
                                  attestation_id: str,
                                  role: str,           # "prover" | "verifier" | "oracle"
                                  timestamps: dict,            # ms dict
                                  json_dir: str):

    _ensure_dir(json_dir)
    json_path = os.path.join(json_dir, f"{participant_name}.json")
    lock = _get_json_lock(json_path)

    # Build role-specific record
    if role == "prover":
        att = {"attestation_id": attestation_id}
        if "evidence_sent" in timestamps:
            att["t_evidence_sent"] = timestamps["evidence_sent"]
        if "result_received" in timestamps:
            att["t_result_received"] = timestamps["result_received"]

    elif role == "verifier":
        att = {"attestation_id": attestation_id}
        if "attestation_started_received" in timestamps:
            att["t_attestation_started_received"] = timestamps["attestation_started_received"]
        if "evaluation_ready_received" in timestamps:
            att["t_evaluation_ready_received"] = timestamps["evaluation_ready_received"]
        if "result_sent" in timestamps:
            att["t_result_sent"] = timestamps["result_sent"]
    
    elif role == "oracle":
        att = {"attestation_id": attestation_id}
        if "attestation_started_received" in timestamps:
            att["t_attestation_started_received"] = timestamps["attestation_started_received"]
        if "get_prover_addr_start" in timestamps:
            att["t_get_prover_addr_start"] = timestamps["get_prover_addr_start"]
        if "get_prover_addr_finished" in timestamps:
            att["t_get_prover_addr_finished"] = timestamps["get_prover_addr_finished"]
        if "get_prover_ref_signatures_db_start" in timestamps:
            att["t_ref_signatures_db_start"] = timestamps["get_prover_ref_signatures_db_start"]
        if "get_prover_ref_signatures_db_finished" in timestamps:
            att["t_ref_signatures_db_finished"] = timestamps["get_prover_ref_signatures_db_finished"]
        if "prover_ref_signatures_sent" in timestamps:
            att["t_ref_signatures_sent"] = timestamps["prover_ref_signatures_sent"]    
    
    else:
        raise ValueError(f"Unknown role: {role}")


    now_ms = int(time.time() * 1000)

    with lock:
        data = _load_or_init_results(json_path, participant_name, now_ms)
        # optional hard stop gate:
        if data.get("stopped"):
            return
        # ensure list exists in case old files don’t have it yet
        if role not in data:
            data[role] = []
        data[role].append(att)
        data["last_write_ms"] = now_ms
        _atomic_write_json(json_path, data)

    try:
        logger.info(f"💾 Results saved to '{json_path}'")
    except Exception:
        pass

def mark_experiment_stop(json_dir: str, participant: str):
    json_path = os.path.join(json_dir, f"{participant}.json")
    lock = _get_json_lock(json_path)
    now_ms = int(time.time() * 1000)
    with lock:
        data = _load_or_init_results(json_path, participant, now_ms)
        if "t_end" not in data:
            data["t_end"] = now_ms
            data["stopped"] = True
            _atomic_write_json(json_path, data)
        # else: already stopped → no-op (idempotent)

def get_env_int(*names, default: int = 0):
    for n in names:
        v = os.getenv(n)
        if v is not None:
            return int(v)
    return int(default)

def get_env_float(*names, default: float = 0.0):
    for n in names:
        v = os.getenv(n)
        if v is not None:
            return float(v)
    return float(default)

def get_env_str(*names, default: str = ""):
    for n in names:
        v = os.getenv(n)
        if v is not None:
            return v
    return default

def generate_attestation_id():
    timestamp = int(time.time())                     # 10 digits
    rand_suffix = f"{random.randint(1000, 9999)}"    # 4 digits
    return f"attestation{timestamp % 1000000000}{rand_suffix}"

def _normalize_eth_address(addr: Optional[str]) -> str:
    """Lowercase, trim, and remove a leading '0x' if present."""
    if not addr:
        return ""
    a = addr.strip().lower()
    return a[2:] if a.startswith("0x") else a

def get_agent_by_blockchain_address(
    directory: str,
    target_address: str,
) -> Optional[Dict[str, Any]]:
    """Return the first JSON object in `directory` whose 'eth_address' matches `target_address`."""
    target_norm = _normalize_eth_address(target_address)
    if not target_norm or not os.path.isdir(directory):
        return None

    try:
        with os.scandir(directory) as it:
            for entry in it:
                if not (entry.is_file() and entry.name.endswith(".json")):
                    continue
                try:
                    with open(entry.path, "r", encoding="utf-8") as f:
                        data = json.load(f)
                except (OSError, json.JSONDecodeError):
                    continue

                json_address_norm = _normalize_eth_address(data.get("eth_address"))
                if json_address_norm and json_address_norm == target_norm:
                    return data
    except OSError:
        return None

    return None

def short_att_id(attestation_id: Optional[str], n: int = 4) -> str:
    """
    Return 'att...<last n digits>' from an attestation_id.
    - If there are trailing digits, use those.
    - Else, use the last n characters of the whole string.
    - Handles None/empty safely.
    """
    if not attestation_id:
        return f"att...{'?'*n}"

    # Grab trailing digits (if any)
    m = re.search(r'(\d+)$', attestation_id)
    if m:
        tail = m.group(1)[-n:]
    else:
        tail = attestation_id[-n:]

    return f"att...{tail}"