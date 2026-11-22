"""Timing rows of the SECaaS verdicts, served by GET /results and cleared by
DELETE /results. Same columns as the verifier rows of the sidecars."""

import threading
import time

_lock = threading.Lock()
_rows = []


def add(row: dict):
    with _lock:
        _rows.append(row)


def snapshot() -> list:
    with _lock:
        return list(_rows)


def clear():
    with _lock:
        _rows.clear()


def ms_since(t0: float) -> float:
    """Milliseconds since t0, a time.perf_counter() value, to the microsecond."""
    return round((time.perf_counter() - t0) * 1000, 3)
