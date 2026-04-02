#!/usr/bin/env python3
import json
import glob
import os
import re
import sys

GENESIS_PATH = "besu/QBFTgenesis.json"
AGENT_GLOB = "nodes/agent*/address"
BALANCE_FOR_AGENTS = "1000000000000000000000000000"  # 1e24 (like your base genesis)

# Addresses that must NEVER be removed from alloc.
# This includes: validators, members, rpcnode, pre-funded test accounts,
# and the permissioning smart contracts (0x...8888, 0x...9999).
# We identify them by checking if they do NOT belong to an agent directory.
# As a safety net, we also whitelist the well-known genesis addresses here.
PROTECTED_PREFIXES = {
    "0xfe3b557e8fb62b89f4916b721be55ceb828dbd73",  # pre-funded test account
    "0x627306090abab3a6e1400e9345bc60c78a8bef57",  # pre-funded test account
    "0xf17f52151ebef6c7334fad080c5704d77216b732",  # pre-funded test account
    "0x0000000000000000000000000000000000008888",  # Account Ingress contract
    "0x0000000000000000000000000000000000009999",  # Node Ingress contract
}

# ---------- Minimal RLP encoding ----------
def int_to_min_bytes(x: int) -> bytes:
    if x == 0:
        return b""
    s = []
    while x:
        s.append(x & 0xff)
        x >>= 8
    return bytes(reversed(s))

def rlp_encode(item):
    if isinstance(item, list):
        payload = b"".join(rlp_encode(x) for x in item)
        if len(payload) <= 55:
            return bytes([0xc0 + len(payload)]) + payload
        else:
            l = int_to_min_bytes(len(payload))
            return bytes([0xf7 + len(l)]) + l + payload
    elif isinstance(item, int):
        if item == 0:
            return b"\x80"  # RLP integer zero = empty string
        bts = int_to_min_bytes(item)
        return rlp_encode(bts)
    elif isinstance(item, (bytes, bytearray)):
        bts = bytes(item)
        if len(bts) == 1 and bts[0] < 0x80:
            return bts
        if len(bts) <= 55:
            return bytes([0x80 + len(bts)]) + bts
        l = int_to_min_bytes(len(bts))
        return bytes([0xb7 + len(l)]) + l + bts
    elif isinstance(item, str):
        hs = item.lower()
        if hs.startswith("0x"):
            hs = hs[2:]
        if re.fullmatch(r"[0-9a-f]*", hs) and len(hs) % 2 == 0:
            return rlp_encode(bytes.fromhex(hs))
        return rlp_encode(item.encode("utf-8"))
    else:
        raise TypeError(f"Unsupported RLP type: {type(item)}")

# ---------- Helpers ----------
def read_file(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.read()

def normalize_hex_addr(s: str) -> str:
    s = s.strip()
    if s.startswith("0x") or s.startswith("0X"):
        s = s[2:]
    s = s.lower()
    if len(s) != 40 or not re.fullmatch(r"[0-9a-f]{40}", s):
        raise ValueError(f"Invalid hex address (need 40 hex chars): {s}")
    return s

def find_agent_dirs_sorted():
    files = glob.glob(AGENT_GLOB)
    pairs = []
    for f in files:
        m = re.search(r"agent(\d+)/address$", f.replace("\\", "/"))
        if not m:
            continue
        idx = int(m.group(1))
        pairs.append((idx, f))
    pairs.sort(key=lambda x: x[0])
    return [f for _, f in pairs]

def load_agent_node_addresses(n):
    addrs = []
    paths = find_agent_dirs_sorted()
    if n > len(paths):
        raise RuntimeError(f"Requested {n} agents but found only {len(paths)}.")
    for p in paths[:n]:
        raw = read_file(p).replace("\n", "").replace("\r", "")
        addrs.append(normalize_hex_addr(raw))
    return addrs

def load_agent_account_address(directory):
    keystore_path = os.path.join(directory, "accountKeystore")
    with open(keystore_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    acc = data.get("address")
    if not acc:
        raise RuntimeError(f"No 'address' in {keystore_path}")
    return "0x" + normalize_hex_addr(acc)

def collect_non_agent_addresses():
    """
    Collect account addresses from all NON-agent node directories
    (validators, members, rpcnode) so we know what to protect.
    """
    protected = set(PROTECTED_PREFIXES)
    node_base = "nodes"
    if not os.path.isdir(node_base):
        return protected
    for entry in os.listdir(node_base):
        # Skip agent directories
        if re.match(r"^agent\d+$", entry):
            continue
        ks_path = os.path.join(node_base, entry, "accountKeystore")
        if os.path.isfile(ks_path):
            try:
                with open(ks_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                acc = data.get("address", "")
                if acc:
                    protected.add("0x" + normalize_hex_addr(acc))
            except Exception:
                pass
        # Also protect the node's own address file if present
        addr_path = os.path.join(node_base, entry, "address")
        if os.path.isfile(addr_path):
            try:
                raw = read_file(addr_path).strip()
                protected.add("0x" + normalize_hex_addr(raw))
            except Exception:
                pass
    return protected

def update_alloc_for_agents(genesis_obj, count):
    alloc = genesis_obj.setdefault("alloc", {})

    # 1) Build the set of addresses we must NOT remove
    protected = collect_non_agent_addresses()

    # 2) Build the new agent address set
    new_agent_addrs = set()
    paths = find_agent_dirs_sorted()[:count]
    for addr_path in paths:
        vdir = os.path.dirname(addr_path)
        acc_addr = load_agent_account_address(vdir)
        new_agent_addrs.add(acc_addr)

    # 3) Remove old entries that are NOT protected and NOT in the new agent set.
    #    These are the orphaned old-agent allocations.
    to_remove = []
    for addr in alloc:
        addr_lower = addr.lower()
        if addr_lower in protected:
            continue
        if addr_lower in new_agent_addrs:
            continue
        # Check if the entry has code/storage (smart contract) — keep those
        entry = alloc[addr]
        if entry.get("code") or entry.get("storage"):
            continue
        # Check if it has a privateKey comment (original test accounts) — keep
        if entry.get("privateKey") or entry.get("comment"):
            continue
        # This is a balance-only entry not belonging to any current agent → stale
        to_remove.append(addr)

    for addr in to_remove:
        del alloc[addr]

    if to_remove:
        print(f"🧹 Removed {len(to_remove)} stale agent alloc entries")

    # 4) Add new agent entries
    added = 0
    for acc_addr in new_agent_addrs:
        if acc_addr not in alloc:
            alloc[acc_addr] = {"balance": BALANCE_FOR_AGENTS}
            added += 1

    print(f"➕ Added {added} new agent alloc entries (total agents: {count})")
    return alloc

def main():
    if len(sys.argv) != 2 or not sys.argv[1].isdigit():
        print("Usage: python3 update_qbft_genesis_agent_credentials.py <NUM_AGENTS>")
        sys.exit(1)
    n = int(sys.argv[1])
    if n <= 0:
        print("NUM_AGENTS must be > 0")
        sys.exit(1)

    # Load existing genesis (we keep everything else unchanged)
    with open(GENESIS_PATH, "r", encoding="utf-8") as f:
        genesis = json.load(f)

    # Collect list from nodes/agentX/address
    agents = load_agent_node_addresses(n)

    # Clean old + add new agent addresses to alloc (funding)
    update_alloc_for_agents(genesis, n)

    # Write back
    with open(GENESIS_PATH, "w", encoding="utf-8") as f:
        json.dump(genesis, f, indent=2)
        f.write("\n")

    print(f"✅ Updated QBFT genesis at {GENESIS_PATH}")
    print(f"   {n} agent accounts funded")

if __name__ == "__main__":
    main()
