"""Reads the attestation chain for the experiment runner. Runs in the secaas
container, which has web3, the contract ABI and the node URL:

  docker exec -i secaas python3 - extract FROM TO < tools/chain.py
      JSON on stdout: the blocks FROM..TO, the transactions sent to the
      contract in them (gas, calldata, signed and log bytes), and the rounds
      whose evidence (Prove) is among them.
  docker exec -i secaas python3 - eligible < tools/chain.py
      the number of agents eligible as jurors.

A round is closed by its jury (closed_by=jury) or by the SECaaS: secaas when
no jury could be drawn (each robot's first round after a reset),
secaas_dissent when the jurors disagreed, secaas_expiry when the jury timed out.
"""

import json
import sys

import rlp
from web3 import Web3

try:
    from web3.middleware import ExtraDataToPOAMiddleware as poa
except ImportError:
    from web3.middleware import geth_poa_middleware as poa

cfg = json.load(open("/config/secaas.json"))
w3 = Web3(Web3.HTTPProvider(cfg["eth_node_url"]))
w3.middleware_onion.inject(poa, layer=0)
contract = w3.eth.contract(address=Web3.to_checksum_address(cfg["contract_address"]), abi=cfg["contract_abi"])

EVENTS = {
    bytes(Web3.keccak(text=f"{e['name']}({','.join(i['type'] for i in e['inputs'])})")): e["name"]
    for e in cfg["contract_abi"] if e["type"] == "event"
}
HANDOFF = {"Prove": "secaas", "Vote": "secaas_dissent", "Expire": "secaas_expiry"}


def tx_bytes(tx):
    """Size of the signed transaction as stored in its block. The agents send
    legacy transactions: RLP-encoded here, no call to the node."""
    if tx.type != 0:
        return len(w3.eth.get_raw_transaction(tx.hash))
    return len(rlp.encode([tx.nonce, tx.gasPrice, tx.gas, bytes.fromhex(tx.to[2:]), tx.value, bytes(tx.input),
                           tx.v, int.from_bytes(tx.r, "big"), int.from_bytes(tx.s, "big")]))


def round_id(args):
    rid = args.get("id")
    return rid.rstrip(b"\0").decode(errors="replace") if isinstance(rid, bytes) else ""


def extract(first, last):
    blocks, txs, rounds = [], [], {}
    for n in range(first, last + 1):
        blk = w3.eth.get_block(n, full_transactions=True)
        blocks.append({"block": n, "timestamp": blk.timestamp, "tx_count": len(blk.transactions),
                       "gas_used": blk.gasUsed, "gas_limit": blk.gasLimit, "size_bytes": blk.size})
        # One call for the receipts of the block, not one per transaction.
        receipts = {r.transactionHash: r for r in w3.eth.get_block_receipts(n)} if blk.transactions else {}
        for tx in blk.transactions:
            if tx.to != contract.address:
                continue
            receipt = receipts[tx.hash]
            fn, args = contract.decode_function_input(tx.input)
            fn, rid = fn.fn_name, round_id(args)
            events = [EVENTS.get(bytes(log.topics[0]), "") for log in receipt.logs
                      if log.address == contract.address and log.topics]
            log_bytes = sum(32 * len(log.topics) + len(log.data) for log in receipt.logs)
            txs.append({"block": n, "sender": tx["from"], "function": fn, "round_id": rid,
                        "gas_used": receipt.gasUsed, "status": receipt.status, "events": "|".join(events),
                        "calldata_bytes": len(tx.input), "tx_bytes": tx_bytes(tx), "log_bytes": log_bytes})

            r = rounds.get(rid)
            if fn == "Prove" and receipt.status == 1:
                r = rounds[rid] = {"round_id": rid, "prover": tx["from"], "prove_block": n, "close_block": "",
                                   "blocks_to_close": "", "closed_by": "", "verdict": "", "votes": 0,
                                   "gas_prove": receipt.gasUsed, "gas_votes": 0, "gas_secaas": 0,
                                   "gas_expire": 0, "gas_reverted": 0, "handoff": ""}
            elif r is None:
                continue
            elif receipt.status != 1:
                r["gas_reverted"] += receipt.gasUsed
                continue
            elif fn == "Vote":
                r["votes"] += 1
                r["gas_votes"] += receipt.gasUsed
            elif fn == "Expire":
                r["gas_expire"] += receipt.gasUsed
            elif fn == "VerifyBySECaaS":
                r["gas_secaas"] += receipt.gasUsed

            if "AttestationStarted" in events:
                r["handoff"] = HANDOFF.get(fn, fn)
            if "AttestationCompleted" in events:
                done = next(contract.events.AttestationCompleted().process_log(log) for log in receipt.logs
                            if log.address == contract.address and EVENTS.get(bytes(log.topics[0])) == "AttestationCompleted")
                r["close_block"] = n
                r["blocks_to_close"] = n - r["prove_block"]
                r["verdict"] = "success" if done.args.success else "failure"
                r["closed_by"] = "jury" if fn == "Vote" else r["handoff"]

    for r in rounds.values():
        del r["handoff"]
    return {"blocks": blocks, "transactions": txs, "rounds": list(rounds.values())}


if __name__ == "__main__":
    if sys.argv[1:2] == ["extract"]:
        json.dump(extract(int(sys.argv[2]), int(sys.argv[3])), sys.stdout)
    elif sys.argv[1:2] == ["eligible"]:
        print(len(contract.functions.GetEligibleAgents().call()))
    else:
        sys.exit(__doc__)
