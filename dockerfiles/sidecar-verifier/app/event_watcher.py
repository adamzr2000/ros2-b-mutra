# event_watcher.py
import json, time, logging, os
from web3 import Web3
from typing import Iterable, Optional, Tuple, Dict, Any

logger = logging.getLogger(__name__)

class EventWatcher:
    def __init__(
        self,
        web3: Web3,
        contract,
        event_abi,                   # contract.events.EventName._get_event_abi()
        address: str,
        topics: Optional[list] = None,
        checkpoint_path: str = "event_checkpoint.json",
        confirmations: int = 2,
        batch_size: int = 1000,
        poll_interval: float = 1.0,
    ):
        self.w3 = web3
        self.address = Web3.toChecksumAddress(address)
        self.contract = contract
        self.event_abi = event_abi
        self.topics = topics
        self.checkpoint_path = checkpoint_path
        self.confirmations = confirmations
        self.batch_size = batch_size
        self.poll_interval = poll_interval

        cp = self._load_checkpoint()
        self.from_block = cp.get("from_block", self.w3.eth.block_number)
        self.seen_keys = set(cp.get("seen_keys", []))  # list of "blockHash-logIndex"

    def _load_checkpoint(self) -> Dict[str, Any]:
        if os.path.exists(self.checkpoint_path):
            with open(self.checkpoint_path, "r") as f:
                return json.load(f)
        return {}

    def _save_checkpoint(self):
        # ensure parent dir exists
        os.makedirs(os.path.dirname(self.checkpoint_path) or ".", exist_ok=True)
        with open(self.checkpoint_path, "w") as f:
            json.dump(
                {"from_block": self.from_block, "seen_keys": list(self.seen_keys)},
                f,
                indent=2,
            )

    def _latest_safe_block(self) -> int:
        return max(0, self.w3.eth.block_number - self.confirmations)

    def _unique_key(self, log) -> str:
        return f"{log['blockHash'].hex()}-{log['logIndex']}"

    def _decode_log(self, raw_log):
        return self.contract.events[self.event_abi['name']]().processLog(raw_log)

    def _fetch_logs_range(self, start_block: int, end_block: int) -> Iterable[dict]:
        # raw logs via eth_getLogs (address+topics filter)
        return self.w3.eth.getLogs({
            "fromBlock": start_block,
            "toBlock": end_block,
            "address": self.address,
            "topics": self.topics,
        })

    def run(self, handle_event):
        """
        handle_event(decoded_event) -> None
        """
        logger.info(f"Event watcher starting from block {self.from_block}")
        while True:
            try:
                safe_to = self._latest_safe_block()
                if self.from_block > safe_to:
                    time.sleep(self.poll_interval)
                    continue

                while self.from_block <= safe_to:
                    to_block = min(self.from_block + self.batch_size - 1, safe_to)
                    raw_logs = self._fetch_logs_range(self.from_block, to_block)

                    # deterministic ordering
                    raw_logs.sort(key=lambda l: (l['blockNumber'], l['logIndex']))

                    for raw in raw_logs:
                        key = self._unique_key(raw)
                        if key in self.seen_keys:
                            continue
                        evt = self._decode_log(raw)
                        handle_event(evt)  # your business logic
                        self.seen_keys.add(key)

                    self.from_block = to_block + 1
                    self._save_checkpoint()

                time.sleep(self.poll_interval)

            except Exception as e:
                logger.exception(f"Watcher error, backing off: {e}")
                time.sleep(3)
