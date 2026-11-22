# app/internal/blockchain/heads.py
"""Tells the waiters when the node imports a block. Follows the node's
newHeads subscription over WebSocket; without a WebSocket URL, or while the
subscription is down, ticks every fallback interval instead."""

import json
import logging
import threading
import time

from websockets.sync.client import connect

from app.internal.logger import info, warn, error

WAIT_MAX = 5.0  # a wait for the next block ends after this many seconds anyway

# The library logs every frame at DEBUG: one line per block.
logging.getLogger("websockets").setLevel(logging.WARNING)


class Heads:
    def __init__(self):
        self._cond = threading.Condition()
        self.seq = 0       # blocks notified so far
        self.num = 0       # latest block notified, 0 when polling
        self.at = 0.0      # time.perf_counter() when it arrived
        self.live = False  # following the subscription

    def start(self, ws_url, fallback):
        threading.Thread(target=self._run, args=(ws_url, fallback), name="heads", daemon=True).start()

    def wait(self, seq, timeout=WAIT_MAX):
        """Waits for a block after seq: read seq before checking the chain,
        so that a block imported in between is not missed."""
        with self._cond:
            self._cond.wait_for(lambda: self.seq != seq, timeout=timeout)

    def mode(self):
        return "websocket" if self.live else "polling"

    def _fire(self, num, live):
        with self._cond:
            self.seq += 1
            self.num, self.at, self.live = num, time.perf_counter(), live
            self._cond.notify_all()

    def _run(self, ws_url, fallback):
        while True:
            if ws_url:
                self._follow(ws_url)
                warn(f"Block notifications lost: polling every {fallback}s, reconnecting in 5s")
            retry = time.monotonic() + 5
            while not ws_url or time.monotonic() < retry:
                time.sleep(fallback)
                self._fire(0, False)

    def _follow(self, ws_url):
        """Returns when the subscription cannot start or ends."""
        try:
            with connect(ws_url, open_timeout=10) as ws:
                ws.send(json.dumps({"jsonrpc": "2.0", "id": 1, "method": "eth_subscribe", "params": ["newHeads"]}))
                reply = json.loads(ws.recv(timeout=10))
                if "result" not in reply:
                    error(f"newHeads subscription on {ws_url} refused: {reply}")
                    return
                info(f"Following new blocks on {ws_url}")
                for message in ws:
                    head = (json.loads(message).get("params") or {}).get("result") or {}
                    if "number" in head:
                        self._fire(int(head["number"], 16), True)
        except Exception as e:
            error(f"newHeads subscription on {ws_url} ended: {e}")
