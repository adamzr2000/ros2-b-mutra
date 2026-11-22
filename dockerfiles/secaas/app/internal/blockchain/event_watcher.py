# app/internal/blockchain/event_watcher.py

import json
import time
import os
from typing import Iterable, Optional, Dict, Any
from web3 import Web3
from app.internal.logger import info, warn, error, debug

class EventWatcher:
    def __init__(
        self,
        web3: Web3,
        contract,
        event_abi: dict,
        address: str,
        topics: Optional[list] = None,
        checkpoint_path: str = "event_checkpoint.json",
        confirmations: int = 2,
        batch_size: int = 1000,
        poll_interval: float = 1.0,
        heads=None,
    ):
        self.w3 = web3
        self.address = Web3.to_checksum_address(address)
        self.contract = contract
        self.event_abi = event_abi
        self.topics = topics
        self.checkpoint_path = checkpoint_path
        self.confirmations = confirmations
        self.batch_size = batch_size
        self.poll_interval = poll_interval
        self.heads = heads  # Heads: read at each new block; None: every poll_interval

        cp = self._load_checkpoint()
        saved_block = cp.get("from_block")
        chain_tip   = self.w3.eth.block_number
        if saved_block is not None and saved_block > chain_tip + 1:
            # from_block is the next block to read: tip+1 once the watcher read
            # up to the tip. Further ahead, the checkpoint is from a previous
            # chain instance (e.g. Quorum restarted): scanning from it would
            # cause the watcher to wait forever.
            warn(
                f"Checkpoint from_block={saved_block} is ahead of chain tip={chain_tip}"
                " — stale chain detected, resetting to 0"
            )
            self.from_block = 0
            self.seen_keys = set()
        else:
            self.from_block = saved_block if saved_block is not None else chain_tip
            self.seen_keys = set(cp.get("seen_keys", []))

    def _load_checkpoint(self) -> Dict[str, Any]:
        if os.path.exists(self.checkpoint_path):
            try:
                with open(self.checkpoint_path, "r") as f:
                    data = json.load(f)
                    if isinstance(data, dict):
                        return data
            except (json.JSONDecodeError, OSError):
                pass
        return {}

    def _save_checkpoint(self):
        directory = os.path.dirname(self.checkpoint_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
            
        with open(self.checkpoint_path, "w") as f:
            json.dump(
                {"from_block": self.from_block, "seen_keys": list(self.seen_keys)},
                f,
                indent=2,
            )

    def _latest_safe_block(self) -> int:
        # The block just notified, else asked to the node.
        current = self.heads.num if self.heads and self.heads.live else self.w3.eth.block_number
        return max(0, current - self.confirmations)

    def _unique_key(self, log) -> str:
        return f"{log['blockHash'].hex()}-{log['logIndex']}"

    def _decode_log(self, raw_log):
        event_name = self.event_abi['name']
        event_cls = getattr(self.contract.events, event_name)
        return event_cls.process_log(raw_log)

    def _fetch_logs_range(self, start_block: int, end_block: int) -> Iterable[dict]:
        # FIX: Use 'fromBlock' and 'toBlock' (CamelCase) for raw RPC compatibility
        filter_params = {
            "fromBlock": start_block,
            "toBlock": end_block,
            "address": self.address,
        }
        if self.topics:
            filter_params["topics"] = self.topics
            
        return self.w3.eth.get_logs(filter_params)

    def _wait(self, seq):
        if self.heads:
            self.heads.wait(seq)
        else:
            time.sleep(self.poll_interval)

    def run(self, handle_event, stop_event=None):
        event_name = self.event_abi.get("name", "UnknownEvent")
        topics_str = str(self.topics) if self.topics else "None"

        info(
            f"EventWatcher start: event={event_name} address={self.address} "
            f"from_block={self.from_block} confirmations={self.confirmations} "
            f"batch_size={self.batch_size} poll_interval={self.poll_interval:.3f}s topics={topics_str}"
        )

        while not (stop_event and stop_event.is_set()):
            try:
                seq = self.heads.seq if self.heads else None
                safe_to = self._latest_safe_block()
                
                if self.from_block > safe_to:
                    self._wait(seq)
                    continue

                while self.from_block <= safe_to:
                    to_block = min(self.from_block + self.batch_size - 1, safe_to)
                    
                    raw_logs = self._fetch_logs_range(self.from_block, to_block)
                    logs_list = list(raw_logs)
                    logs_list.sort(key=lambda l: (l['blockNumber'], l['logIndex']))

                    for raw in logs_list:
                        key = self._unique_key(raw)
                        if key in self.seen_keys:
                            continue
                        
                        try:
                            evt = self._decode_log(raw)
                            handle_event(evt)
                            self.seen_keys.add(key)
                        except Exception as e:
                            error(f"Error decoding/handling log {key}: {e}")

                    # Advance from_block and persist checkpoint before clearing seen_keys
                    # so a crash between here and the clear still yields a valid,
                    # deduplicated restart point (new from_block + keys for replay dedup).
                    self.from_block = to_block + 1
                    self._save_checkpoint()
                    self.seen_keys.clear()

                self._wait(seq)

            except Exception as e:
                error(f"Watcher error, backing off: {e}")
                time.sleep(3)