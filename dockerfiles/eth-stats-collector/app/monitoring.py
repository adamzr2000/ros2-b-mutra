#!/usr/bin/env python3
"""
Blockchain block monitor.

Polls for new blocks and records per-block metrics:
  block_number, block_timestamp, block_time_s,
  tx_count, gas_used, gas_limit, gas_used_pct,
  size_bytes, extra_data_bytes, avg_gas_per_tx
"""
from __future__ import annotations

import csv
import logging
import threading
import time
import os
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Dict, Optional

from web3 import Web3
from web3.middleware import ExtraDataToPOAMiddleware

logger = logging.getLogger(__name__)

CSV_FIELDS = [
    "collected_at_iso",
    "block_number",
    "block_timestamp",
    "block_time_s",
    "tx_count",
    "gas_used",
    "gas_limit",
    "gas_used_pct",
    "size_bytes",
    "extra_data_bytes",
    "avg_gas_per_tx",
]


def _block_to_sample(block, prev_timestamp: Optional[int], collected_at: datetime) -> dict:
    tx_count = len(block.transactions)
    gas_used = block.gasUsed
    gas_limit = block.gasLimit
    gas_used_pct = round(gas_used / gas_limit * 100.0, 3) if gas_limit > 0 else 0.0
    block_time = (block.timestamp - prev_timestamp) if prev_timestamp is not None else None
    raw_extra = block.get("extraData", None)
    extra_data_bytes = len(raw_extra) if raw_extra else 0
    size_bytes = block.get("size", 0) or 0
    avg_gas_per_tx = round(gas_used / tx_count, 2) if tx_count > 0 else 0.0

    return {
        "collected_at_iso": collected_at.isoformat(),
        "block_number":     block.number,
        "block_timestamp":  block.timestamp,
        "block_time_s":     block_time,
        "tx_count":         tx_count,
        "gas_used":         gas_used,
        "gas_limit":        gas_limit,
        "gas_used_pct":     gas_used_pct,
        "size_bytes":       size_bytes,
        "extra_data_bytes": extra_data_bytes,
        "avg_gas_per_tx":   avg_gas_per_tx,
    }


@dataclass
class BlockchainMonitor:
    rpc_url: str
    poll_interval: float = 1.0
    csv_path: Optional[str] = None

    _thread: Optional[threading.Thread] = field(default=None, init=False, repr=False)
    _stop_event: threading.Event = field(default_factory=threading.Event, init=False, repr=False)
    _csv_file: Optional[Any] = field(default=None, init=False, repr=False)
    _csv_writer: Optional[csv.DictWriter] = field(default=None, init=False, repr=False)
    _last_sample: Optional[Dict[str, Any]] = field(default=None, init=False, repr=False)
    _blocks_collected: int = field(default=0, init=False, repr=False)

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            logger.warning("BlockchainMonitor is already running.")
            return

        self._stop_event.clear()
        self._last_sample = None
        self._blocks_collected = 0

        if self.csv_path:
            is_new = not os.path.exists(self.csv_path) or os.path.getsize(self.csv_path) == 0
            os.makedirs(os.path.dirname(self.csv_path) or ".", exist_ok=True)
            self._csv_file = open(self.csv_path, "a", newline="")
            self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=CSV_FIELDS)
            if is_new:
                self._csv_writer.writeheader()
                self._csv_file.flush()

        self._thread = threading.Thread(target=self._run, name="BlockchainMonitor", daemon=True)
        self._thread.start()
        logger.info("BlockchainMonitor started (rpc=%s, interval=%.1fs)", self.rpc_url, self.poll_interval)

    def stop(self, timeout: float = 5.0) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=timeout)
        if self._csv_file:
            try:
                self._csv_file.close()
            except Exception:
                pass
            finally:
                self._csv_file = None
                self._csv_writer = None
        logger.info("BlockchainMonitor stopped. Blocks collected: %d", self._blocks_collected)

    def is_running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    def get_last_sample(self) -> Optional[Dict[str, Any]]:
        return self._last_sample

    def get_stats(self) -> Dict[str, Any]:
        return {"blocks_collected": self._blocks_collected, "running": self.is_running()}

    def _run(self) -> None:
        w3 = Web3(Web3.HTTPProvider(self.rpc_url))
        w3.middleware_onion.inject(ExtraDataToPOAMiddleware, layer=0)
        if not w3.is_connected():
            logger.error("Cannot connect to Ethereum node at %s", self.rpc_url)
            return

        logger.info("Connected to node %s (chain_id=%s)", self.rpc_url, w3.eth.chain_id)

        last_block_number: Optional[int] = None
        prev_timestamp: Optional[int] = None

        while not self._stop_event.is_set():
            try:
                current = w3.eth.block_number
                if last_block_number is None:
                    last_block_number = current - 1

                for num in range(last_block_number + 1, current + 1):
                    if self._stop_event.is_set():
                        break
                    try:
                        block = w3.eth.get_block(num, full_transactions=False)
                    except Exception as e:
                        logger.warning("Failed to fetch block %d: %s", num, e)
                        continue

                    collected_at = datetime.now(timezone.utc)
                    sample = _block_to_sample(block, prev_timestamp, collected_at)
                    self._last_sample = sample
                    self._blocks_collected += 1
                    prev_timestamp = block.timestamp
                    last_block_number = num

                    if self._csv_writer:
                        try:
                            self._csv_writer.writerow(sample)
                            self._csv_file.flush()
                        except Exception as e:
                            logger.warning("CSV write error: %s", e)

                    logger.debug(
                        "Block #%d | txs=%d | gas=%d/%d (%.1f%%) | size=%d B | dt=%ss",
                        sample["block_number"], sample["tx_count"],
                        sample["gas_used"], sample["gas_limit"], sample["gas_used_pct"],
                        sample["size_bytes"],
                        f"{sample['block_time_s']:.1f}" if sample["block_time_s"] is not None else "?",
                    )

            except Exception as e:
                logger.warning("Poll error: %s", e)

            self._stop_event.wait(timeout=max(0.1, self.poll_interval))
