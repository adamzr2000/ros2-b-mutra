# app/internal/logger/logger.py

import logging
import os
import sys
from enum import IntEnum
from datetime import datetime

# ANSI color codes
GREEN   = "\033[92m"
YELLOW  = "\033[93m"
BLUE    = "\033[94m"
MAGENTA = "\033[95m"
CYAN    = "\033[96m"
RED     = "\033[91m"
RESET   = "\033[0m"


class LogLevel(IntEnum):
    DEBUG = 10
    INFO  = 20
    WARN  = 30
    ERROR = 40
    NONE  = 100


def _parse_level(env_value: str | None) -> LogLevel:
    v = (env_value or "INFO").strip().upper()
    if v == "DEBUG":
        return LogLevel.DEBUG
    if v == "INFO":
        return LogLevel.INFO
    if v in ("WARN", "WARNING"):
        return LogLevel.WARN
    if v == "ERROR":
        return LogLevel.ERROR
    if v == "NONE":
        return LogLevel.NONE
    return LogLevel.INFO


_CURRENT_LEVEL: LogLevel = _parse_level(os.getenv("LOG_LEVEL"))


def _format(level: str, color: str, msg: str) -> str:
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    return f"{timestamp} - {color}{level}{RESET} - {msg}"


def _print(line: str) -> None:
    # go-style: write to stdout, flush immediately
    sys.stdout.write(line + "\n")
    sys.stdout.flush()


def init_logging() -> None:
    """
    Optional init hook.
    - Reads LOG_LEVEL
    - Disables stdlib logging formatting (we format ourselves)
    """
    global _CURRENT_LEVEL
    _CURRENT_LEVEL = _parse_level(os.getenv("LOG_LEVEL"))

    # Keep python logging available, but don't let it add its own timestamp/level formatting.
    logging.basicConfig(level=logging.DEBUG, format="%(message)s")

    if _CURRENT_LEVEL == LogLevel.NONE:
        _print("[logging] LOG_LEVEL=NONE → all logs disabled")
    else:
        _print(f"[logging] LOG_LEVEL={os.getenv('LOG_LEVEL', 'INFO')}")

    # Optional: quiet noisy libs
    logging.getLogger("web3").setLevel(logging.WARNING)
    logging.getLogger("urllib3").setLevel(logging.WARNING)


def debug(fmt: str, *args) -> None:
    if _CURRENT_LEVEL <= LogLevel.DEBUG:
        _print(_format("DEBUG", CYAN, fmt % args if args else fmt))


def info(fmt: str, *args) -> None:
    if _CURRENT_LEVEL <= LogLevel.INFO:
        _print(_format("INFO", GREEN, fmt % args if args else fmt))


def warn(fmt: str, *args) -> None:
    if _CURRENT_LEVEL <= LogLevel.WARN:
        _print(_format("WARN", YELLOW, fmt % args if args else fmt))


def error(fmt: str, *args) -> None:
    if _CURRENT_LEVEL <= LogLevel.ERROR:
        _print(_format("ERROR", RED, fmt % args if args else fmt))


# Colored output helpers
def green_text(s: str) -> str:
    return f"{GREEN}{s}{RESET}"


def yellow_text(s: str) -> str:
    return f"{YELLOW}{s}{RESET}"
