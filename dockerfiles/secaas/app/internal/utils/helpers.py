# app/internal/utils/helpers.py

import hashlib
import os
import json
import time
import random
from typing import Optional, Dict, Any
import re
from app.internal.logger import info, warn, error, debug, green_text


def rolling_hash(m_hex: str, k: int) -> str:
    """Mirror of compute.RollingHash in Go — must stay bit-for-bit identical.

    Returns the K-iterated chained SHA-256:
        H_1 = m
        H_i = SHA256( H_{i-1} || m )    for i = 2..K

    Concatenation is byte-level (32 + 32 = 64 bytes per step). Accepts the
    seed with or without "0x" prefix; returns bare hex (no prefix).
    K must be >= 1; K=1 returns m unchanged.
    """
    s = m_hex.lower()
    if s.startswith("0x"):
        s = s[2:]
    raw = bytes.fromhex(s)
    if len(raw) != 32:
        raise ValueError(f"rolling hash seed must be 32 bytes, got {len(raw)}")
    if k <= 1:
        return raw.hex()
    h = raw
    for _ in range(2, k + 1):
        h = hashlib.sha256(h + raw).digest()
    return h.hex()


def bind_to_block(m_hex: str, block_hash_hex: str) -> str:
    """Mirror of compute.BindToBlock in Go: SHA256(m || block_hash), bare hex.
    Binds the reference to the hash of the block the evidence is bound to."""
    m = bytes.fromhex(m_hex.lower().removeprefix("0x"))
    h = bytes.fromhex(block_hash_hex.lower().removeprefix("0x"))
    if len(m) != 32 or len(h) != 32:
        raise ValueError("bind_to_block needs two 32-byte values")
    return hashlib.sha256(m + h).hexdigest()


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