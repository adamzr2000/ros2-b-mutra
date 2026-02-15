# app/internal/utils/helpers.py

import os
import json
import time
import threading
import tempfile
import random
from typing import Optional, Dict, Any
import re
from app.internal.logger import info, warn, error, debug, green_text

# One lock per agent file (process-local)
_JSON_FILE_LOCKS = {}

# -------------------------------------------------------------------------
# High-precision timing helpers
# -------------------------------------------------------------------------
def now_ms() -> int:
    """Unix epoch time in milliseconds (wall clock)."""
    return int(time.time() * 1000)

def perf_ns() -> int:
    """High-resolution monotonic clock (nanoseconds)."""
    return time.perf_counter_ns()

def _upsert_by_attestation_id(role_list: list, att: dict):
    """
    Insert or update an attestation record in role_list based on attestation_id.
    - If an entry with same attestation_id exists, merge fields (new overwrites old).
    - Else append.
    """
    att_id = att.get("attestation_id")
    if not att_id:
        role_list.append(att)
        return

    for i, existing in enumerate(role_list):
        if isinstance(existing, dict) and existing.get("attestation_id") == att_id:
            merged = dict(existing)
            merged.update(att)
            role_list[i] = merged
            return

    role_list.append(att)

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

def ensure_results_initialized(json_dir: str, participant: str, json_path: str = None):
    _ensure_dir(json_dir)
    json_path = json_path or os.path.join(json_dir, f"{participant}.json")
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
                                  role: str,
                                  timestamps: dict,
                                  json_dir: str,
                                  json_path: str = None):

    _ensure_dir(json_dir)
    json_path = json_path or os.path.join(json_dir, f"{participant_name}.json")
    lock = _get_json_lock(json_path)

    att = {"attestation_id": attestation_id}

    # For compatibility: accept multiple possible source keys for the same output field
    ROLE_FIELDS = {
        "prover": {
            "t_prover_start": ["prover_start"],
            "t_evidence_sent": ["evidence_sent"],
            "t_evidence_sent_tx_confirmed": ["evidence_sent_tx_confirmed"],
            "t_result_received": ["result_received"],
            "t_prover_finished": ["prover_finished"],
        },
        "verifier": {
            "meta_verification_result": ["verification_result"],
            "meta_is_secaas_verifier": ["is_secaas_verifier"],
            "t_attestation_started_received": ["attestation_started_received"],
            "t_ready_for_evaluation_received": ["ready_for_evaluation_received"],
            "t_verifier_start": ["verifier_start"],
            "t_get_signatures_start": ["get_signatures_start"],
            "t_get_signatures_finished": ["get_signatures_finished"],
            "t_verify_compute_start": ["verify_compute_start"],
            "t_verify_compute_finished": ["verify_compute_finished"],
            "t_result_sent": ["result_sent"],
            "t_result_sent_tx_confirmed": ["result_sent_tx_confirmed"],
            "t_verifier_finished": ["verifier_finished"],
        },
        "oracle": {
            "meta_is_secaas_verifier": ["is_secaas_verifier"],
            "t_attestation_started_received": ["attestation_started_received"],
            "t_oracle_start": ["oracle_start"],
            "t_get_prover_addr_start": ["get_prover_addr_start"],
            "t_get_prover_addr_finished": ["get_prover_addr_finished"],
            "t_get_prover_ref_signature_db_start": ["get_prover_ref_signatures_db_start"],
            "t_get_prover_ref_signature_db_finished": ["get_prover_ref_signatures_db_finished"],
            "t_prover_ref_signature_sent": ["prover_ref_signatures_sent"],
            "t_prover_ref_signature_sent_tx_confirmed": ["prover_ref_signatures_sent_tx_confirmed"],
            "t_oracle_finished": ["oracle_finished"],
        },
    }

    ROLE_PRECISION_MAP = {
        "prover": [
            ("dur_send_evidence_call", "p_send_evidence_start", "p_send_evidence_finished"),
            ("dur_send_evidence_tx_confirm", "p_send_evidence_start", "p_send_evidence_finished_tx_confirmed"),
            ("dur_prover_e2e", "p_send_evidence_start", "p_result_received"),
            ("dur_prover_total", "p_prover_start", "p_prover_finished"),
        ],
        "verifier": [
            # --- SHARED FROM ORACLE (if SECaaS is Verifier) ---
            ("dur_prover_addr_fetch", "p_get_prover_addr_start", "p_get_prover_addr_finished"),
            ("dur_db_fetch", "p_get_prover_ref_signatures_db_start", "p_get_prover_ref_signatures_db_finished"),

            ("dur_signatures_fetch", "p_get_signatures_start", "p_get_signatures_finished"),
            ("dur_verify_compute", "p_verify_compute_start", "p_verify_compute_finished"),
            ("dur_send_result_call", "p_send_result_start", "p_send_result_finished"),
            ("dur_send_result_tx_confirm", "p_send_result_start", "p_send_result_finished_tx_confirmed"),
            ("dur_verifier_reaction", "p_attestation_started_received", "p_verifier_start"),
            ("dur_verifier_total", "p_verifier_start", "p_verifier_finished"),

        ],
        "oracle": [
            ("dur_prover_addr_fetch", "p_get_prover_addr_start", "p_get_prover_addr_finished"),
            ("dur_db_fetch", "p_get_prover_ref_signatures_db_start", "p_get_prover_ref_signatures_db_finished"),
            ("dur_send_prover_ref_signature_call", "p_send_prover_ref_signature_start", "p_send_prover_ref_signature_finished"),
            ("dur_send_prover_ref_signatures_tx_confirm", "p_send_prover_ref_signature_start", "p_send_prover_ref_signature_finished_tx_confirmed"),
            ("dur_oracle_reaction", "p_attestation_started_received", "p_oracle_start"),
            ("dur_oracle_total", "p_oracle_start", "p_oracle_finished"),
        ],
    }

    # Step 1: Copy Wall-clock timestamps
    fields = ROLE_FIELDS.get(role, {})
    for out_key, candidates in fields.items():
        for src_key in candidates:
            if src_key in timestamps:
                att[out_key] = timestamps[src_key]
                break

    # Step 2: Compute High-Precision Durations (p_ fields)
    precision_durs = ROLE_PRECISION_MAP.get(role, [])
    for prefix, p_start, p_end in precision_durs:
        if p_start in timestamps and p_end in timestamps:
            delta_ns = timestamps[p_end] - timestamps[p_start]
            # Export both ns and high-precision ms
            att[f"{prefix}_ns"] = delta_ns
            att[f"{prefix}_ms"] = round(delta_ns / 1_000_000.0, 4)

    # 3) Atomic write
    now_ts = now_ms()
    with lock:
        data = _load_or_init_results(json_path, participant_name, now_ts)
        if data.get("stopped"):
            return
        if role not in data:
            data[role] = []
        _upsert_by_attestation_id(data[role], att)
        data["last_write_ms"] = now_ts
        _atomic_write_json(json_path, data)

    try:
        info(f"💾 Results saved to '{json_path}'")
    except Exception:
        pass

def mark_experiment_stop(json_dir: str, participant: str, json_path: str = None):
    json_path = json_path or os.path.join(json_dir, f"{participant}.json")
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